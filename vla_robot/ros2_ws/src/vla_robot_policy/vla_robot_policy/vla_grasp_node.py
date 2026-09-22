"""vla_grasp_node — VLA 정책 루프로 파지 한 번을 수행하는 액션 서버.

    action vla/run_grasp (RunVlaGrasp)

    프레임(gripper_cam 토픽) + 관절(arm/get_state.policy_state)
      -> 정책(local PolicyRunner | remote RemotePolicyClient) -> 청크
      -> arm/execute_chunk 로 재생 -> 반복

- 카메라와 시리얼을 직접 열지 않는다(각 소유 노드의 토픽/서비스만 쓴다).
- 완료 판정: shoulder_lift 가 한 번 뻗었다가(> extended_lift_deg) 다시 접히면(< returned_lift_deg)
  끝. 학습 118회차 중 117회차에서 맞았다(2026-09-04). **"물체를 집었다"가 아니다** —
  그 판정은 pi_mission_node 가 그리퍼 개구율/부하로 한다.
- 멈춘 카메라 판정: 청크 사이 그림 변화가 거의 0 이면 중단한다. USB 가 끊겨 같은 그림이
  새 타임스탬프로 계속 오면, 정책이 허공을 짚는다(2026-09-05).
"""
from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from vla_common.arm_units import SHOULDER_LIFT_INDEX
from vla_common.config import load_robot_config
from vla_robot_interfaces.action import ExecuteJointChunk, RunVlaGrasp
from vla_robot_interfaces.srv import GetArmState


def wait_future(future, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while not future.done():
        if time.monotonic() > deadline:
            return False
        time.sleep(0.005)
    return True


class GraspAborted(Exception):
    pass


class VlaGraspNode(Node):
    def __init__(self) -> None:
        super().__init__("vla_grasp_node")
        self.declare_parameter("config_file", "")
        robot = load_robot_config(str(self.get_parameter("config_file").value))
        self.pcfg = robot.policy
        self.gcfg = robot.gripper_cam
        self.max_temp_c = robot.arm.max_servo_temp_c
        self.runner = self._make_runner()

        cb = ReentrantCallbackGroup()
        self._frame: Optional[tuple[float, np.ndarray]] = None
        self._frame_lock = threading.Lock()
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.gcfg.topic, self._on_frame, qos, callback_group=cb)
        self._state_client = self.create_client(GetArmState, "arm/get_state", callback_group=cb)
        self._chunk_client = ActionClient(self, ExecuteJointChunk, "arm/execute_chunk", callback_group=cb)
        self._busy = threading.Lock()
        self._server = ActionServer(
            self, RunVlaGrasp, "vla/run_grasp", execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.REJECT if self._busy.locked() else GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT, callback_group=cb)
        self.get_logger().info(f"vla_grasp_node 준비 — {self.runner.describe()}")

    # -- 정책 ---------------------------------------------------------------
    def _make_runner(self):
        p = self.pcfg
        t0 = time.monotonic()
        if p.source == "remote":
            from vla_common.policy_client import RemotePolicyClient
            runner = RemotePolicyClient(p.url, timeout_s=p.remote_timeout_s,
                                        n_action_steps=p.n_action_steps or None)
            info = runner.health()   # 일찍 실패한다
            self.get_logger().info(f"원격 정책 서버 연결 {p.url}: {info}")
        else:
            from vla_common.policy_runner import PolicyRunner
            runner = PolicyRunner(
                p.checkpoint, device=p.device, image_color=p.image_color,
                image_size=p.image_size,
                wrist_roll_value=p.wrist_roll_value if p.freeze_wrist_roll else None,
                n_action_steps=p.n_action_steps or None)
            h, w = runner.policy_hw
            runner.predict_chunk(np.zeros((h, w, 3), np.uint8), [0.0] * 6, "warmup")
        self.get_logger().info(f"정책 준비 {time.monotonic() - t0:.1f}s")
        return runner

    # -- 입력 ---------------------------------------------------------------
    def _on_frame(self, msg: Image) -> None:
        if msg.encoding != "bgr8":
            self.get_logger().warn(f"bgr8 이 아닌 프레임을 버린다: {msg.encoding}", throttle_duration_sec=5.0)
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._frame_lock:
            self._frame = (stamp, frame)

    def _fresh_frame(self) -> np.ndarray:
        with self._frame_lock:
            item = self._frame
        if item is None:
            raise GraspAborted("그리퍼캠 프레임이 없다 — gripper_cam_node 가 떠 있는지 확인")
        age = self.get_clock().now().nanoseconds * 1e-9 - item[0]
        if age > self.pcfg.max_frame_age_s:
            raise GraspAborted(f"그리퍼캠 프레임이 {age:.1f}s 낡았다")
        return item[1]

    def _arm_state(self):
        if not self._state_client.wait_for_service(timeout_sec=2.0):
            raise GraspAborted("arm/get_state 서비스가 없다")
        future = self._state_client.call_async(GetArmState.Request())
        if not wait_future(future, 3.0) or future.result() is None:
            raise GraspAborted("arm/get_state 응답이 없다")
        res = future.result()
        if not res.ok:
            raise GraspAborted(f"관절 상태 읽기 실패: {res.message}")
        return res

    def _policy_state(self) -> list[float]:
        return list(self._arm_state().policy_state)

    def _check_temperature(self, state) -> None:
        """청크 사이마다 본다. 파지는 청크를 여러 번 도는 긴 동작이라, 시작할 때만 보면
        도중에 달궈지는 것을 놓친다. 서보 자체 보호(70°C 근처)가 걸리면 토크가 끊겨
        팔이 떨어지므로 그 앞에서 멈춘다."""
        hot = [f"servo {i + 1} {t}°C" for i, t in enumerate(state.temperature_c)
               if t and t >= self.max_temp_c]
        if hot:
            raise GraspAborted(f"서보 온도 상한({self.max_temp_c:.0f}°C) 초과: {', '.join(hot)}")

    def _play(self, chunk: np.ndarray, goal_handle) -> None:
        if not self._chunk_client.wait_for_server(timeout_sec=2.0):
            raise GraspAborted("arm/execute_chunk 서버가 없다")
        goal = ExecuteJointChunk.Goal(
            positions=[float(v) for v in np.asarray(chunk, dtype=np.float32).ravel()],
            fps=float(self.pcfg.fps), max_step_deg=0.0)
        send = self._chunk_client.send_goal_async(goal)
        if not wait_future(send, 5.0) or not send.result().accepted:
            raise GraspAborted("청크 goal 이 거부됐다(다른 팔 동작 중?)")
        handle = send.result()
        result_future = handle.get_result_async()
        deadline = time.monotonic() + len(chunk) / max(self.pcfg.fps, 1.0) + 10.0
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                cancel = handle.cancel_goal_async()
                wait_future(cancel, 2.0)
                wait_future(result_future, 3.0)
                raise GraspAborted("취소됨")
            if time.monotonic() > deadline:
                handle.cancel_goal_async()
                raise GraspAborted("청크 재생 결과를 못 받았다")
            time.sleep(0.01)
        result = result_future.result().result
        if not result.ok:
            raise GraspAborted(f"청크 재생 실패: {result.message}")

    # -- 루프 ---------------------------------------------------------------
    def _execute(self, goal_handle):
        result = RunVlaGrasp.Result()
        with self._busy:
            req = goal_handle.request
            label = req.label or "object"
            task = f"pick up the {label}"
            timeout_s = req.timeout_s if req.timeout_s > 0 else self.pcfg.timeout_s
            started = time.monotonic()
            chunks, extended, prev = 0, False, None
            feedback = RunVlaGrasp.Feedback()
            try:
                while True:
                    if goal_handle.is_cancel_requested:
                        raise GraspAborted("취소됨")
                    if chunks >= self.pcfg.max_chunks:
                        raise GraspAborted(f"{chunks}청크를 다 썼는데 복귀를 못 봤다")
                    if time.monotonic() - started > timeout_s:
                        raise GraspAborted(f"시간 초과 {timeout_s:.0f}s")

                    frame = self._fresh_frame()
                    if prev is not None:
                        change = float(np.mean(np.abs(frame.astype(np.int16) - prev.astype(np.int16))))
                        if change < self.pcfg.min_frame_change:
                            raise GraspAborted(f"그리퍼캠이 멈췄다(청크 사이 변화 {change:.3f}) — USB 확인")
                    prev = frame.copy()

                    arm = self._arm_state()
                    self._check_temperature(arm)
                    state = list(arm.policy_state)
                    t_inf = time.monotonic()
                    chunk = self.runner.predict_chunk(frame, state, task)
                    inference_ms = (time.monotonic() - t_inf) * 1000.0
                    if chunk.ndim != 2 or chunk.shape[1] != 6 or not np.isfinite(chunk).all():
                        raise GraspAborted(f"정책 출력이 올바르지 않다: {chunk.shape}")
                    self._play(chunk, goal_handle)
                    chunks += 1

                    feedback.chunk = chunks
                    feedback.elapsed_s = float(time.monotonic() - started)
                    feedback.last_inference_ms = float(inference_ms)
                    goal_handle.publish_feedback(feedback)

                    # 명령이 아니라 **실측 자세**로 판정한다. 못 읽으면 판정을 건너뛴다
                    # (기본값 0 을 쓰면 "뻗었다"로 잘못 걸려 거짓 성공이 난다).
                    try:
                        lift = self._policy_state()[SHOULDER_LIFT_INDEX]
                    except GraspAborted as exc:
                        self.get_logger().warn(f"완료 판정용 읽기 실패, 다음 청크에서 다시: {exc}")
                        continue
                    extended = extended or lift > self.pcfg.extended_lift_deg
                    if extended and chunks >= self.pcfg.min_chunks and lift < self.pcfg.returned_lift_deg:
                        result.ok = True
                        result.message = f"{chunks}청크 — 뻗었다가 복귀 (lift {lift:.1f})"
                        break
            except GraspAborted as exc:
                result.ok, result.message = False, str(exc)
            except Exception as exc:  # noqa: BLE001 — 실기 루프는 죽지 않는다
                result.ok, result.message = False, f"예외: {exc}"
            result.chunks = chunks
            result.elapsed_s = float(time.monotonic() - started)
            (self.get_logger().info if result.ok else self.get_logger().warn)(
                f"VLA 파지 {'완료' if result.ok else '실패'}: {result.message}")
            if result.ok:
                goal_handle.succeed()
            elif goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VlaGraspNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
