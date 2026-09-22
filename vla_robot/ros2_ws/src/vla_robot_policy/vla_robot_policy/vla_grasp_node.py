"""vla_grasp_node — VLA 정책 루프로 파지 한 번을 수행하는 액션 서버.

    action vla/run_grasp (RunVlaGrasp)

    프레임(gripper_cam 토픽) + 관절(arm/get_state.policy_state)
      -> 정책(local PolicyRunner | remote RemotePolicyClient) -> 청크
      -> arm/execute_chunk 로 재생 -> 반복

- 카메라와 시리얼을 직접 열지 않는다(각 소유 노드의 토픽/서비스만 쓴다).
- 재시도 감지: 어중간하게 내려왔다가(< retry_dip_deg) **다시 뻗으면** 두 번째 시도로 보고 즉시 끝낸다.
  같은 자리에서 반복해 봐야 관측이 같아 결과도 같다 — 재시도는 Host 몫이다. max_chunks 는 최후 안전장치.
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

from vla_common.arm_units import GRIPPER_INDEX, SHOULDER_LIFT_INDEX
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


#: 기준 프레임과 비교하려면 팔이 그때와 **같은 자세**여야 한다(관절 최대 오차, 도).
#: 2026-09-22 실기: 물체가 없어 정책이 복귀를 못 하고 뻗은 자세로 끝났는데, 그 프레임을
#: 기준(시작 자세)과 비교했더니 빈손인데도 50.8% 가 나왔다 — 물체가 아니라 **자세 차이**를
#: 재고 있었다. 자세가 다르면 비교 자체가 성립하지 않으므로 -1(모름)로 돌려준다.
SAME_POSE_TOL_DEG = 10.0


#: 골짜기 바닥에서 이만큼(도) 다시 오르면 "재시도를 시작했다"고 본다. 측정 잡음은 넘고
#: 사람 눈에 보이는 움직임(수 도)보다는 작은 값이다.
RETRY_RISE_DEG = 3.0


def _scan_attempts(lift_cmd, above: bool, attempts: int, extended_deg: float, dip_deg: float,
                   returned_deg: float):
    """명령 궤적을 훑어 "몇 번째 시도인가"를 센다.

    한 번의 시도는 `extended_deg` 위로 올라갔다가 `dip_deg` 아래로 내려오는 것이다. 이력을
    둔 이유는 문턱 근처의 잔떨림을 시도로 세지 않기 위해서다(2026-09-22 실측: 시도 사이
    저점 -66, 성공 복귀 -103).

    ⚠️ 재시도 지점은 "extended 를 다시 넘는 순간"이 아니라 **골짜기 바닥**이다.
    넘는 순간까지 재생하면 팔이 이미 다시 뻗기 시작한 뒤라 눈에 보인다(2026-09-22 확인).
    바닥에서 끊으면 올라오는 동작 자체가 나오지 않는다.

    반환: (above, attempts, retry_at) — retry_at 은 멈출 스텝 번호. 없으면 None.
    """
    dip_min = None          # 골짜기에 들어간 뒤의 최저값
    dip_idx = 0
    for i, value in enumerate(lift_cmd):
        if above:
            if value < dip_deg:
                above, dip_min, dip_idx = False, value, i
            continue
        if dip_min is None or value < dip_min:
            dip_min, dip_idx = value, i
        if attempts >= 1 and dip_min < returned_deg:
            # 한 번 뻗었다가 완전히 복귀했다 = 회차가 끝나는 중이다(시작 자세 -103 에서
            # 출발하는 첫 청크와 구분하려고 attempts 를 함께 본다). 여기서 다시 오르는 것을 재시도로
            # 세면 **성공하는 청크를 중단**시킨다(바닥에서 명령이 몇 도만 흔들려도 걸린다).
            # 성공 판정은 청크를 다 재생한 뒤 실측 자세로 한다 — 그쪽에 맡긴다.
            return above, attempts, None
        if value > extended_deg or (dip_min is not None and value > dip_min + RETRY_RISE_DEG):
            above = True
            attempts += 1
            if attempts >= 2:
                return above, attempts, dip_idx
            dip_min = None
    return above, attempts, None


def roi_changed_percent(before: np.ndarray, after: np.ndarray,
                        roi_fractions, pixel_threshold: float) -> float:
    """두 프레임의 근접 ROI 에서 달라진 픽셀 비율(%).

    물체를 쥐면 턱 바로 앞이 물체로 덮이므로 이 값이 크게 뛴다.
    2026-09-22 실측: 같은 자세의 빈손끼리 0.0% · 룩을 쥔 상태 30~33%.
    개구율·부하가 TPU 때문에 빈손과 겹치는 것과 달리 여기서는 10배 갈린다.

    ⚠️ **같은 자세끼리만 의미가 있다.** 호출부가 자세를 확인하고 부른다.
    """
    if before is None or after is None or before.shape != after.shape:
        return -1.0
    h, w = before.shape[:2]
    y0, y1, x0, x1 = roi_fractions
    a = before[int(h * y0):int(h * y1), int(w * x0):int(w * x1)].astype(np.int16)
    b = after[int(h * y0):int(h * y1), int(w * x0):int(w * x1)].astype(np.int16)
    if a.size == 0:
        return -1.0
    return float(np.mean(np.max(np.abs(b - a), axis=2) > pixel_threshold) * 100.0)


class VlaGraspNode(Node):
    def __init__(self) -> None:
        super().__init__("vla_grasp_node")
        self.declare_parameter("config_file", "")
        robot = load_robot_config(str(self.get_parameter("config_file").value))
        self.pcfg = robot.policy
        self.gcfg = robot.gripper_cam
        self.max_temp_c = robot.arm.max_servo_temp_c
        self.check = robot.grasp_check
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
            # 재시도 감지용. above = 지금 뻗어 있는가, attempts = 뻗기 시작한 횟수.
            above, attempts = False, 0
            feedback = RunVlaGrasp.Feedback()
            # 파지 직전 기준 프레임. 끝난 뒤 같은 자세에서 다시 찍어 비교한다
            # (학습 회차는 물체를 문 채 시작 자세로 돌아와 끝난다).
            reference, reference_state = None, None
            try:
                reference = self._fresh_frame().copy()
                reference_state = self._policy_state()
            except GraspAborted:
                self.get_logger().warn("기준 프레임을 못 잡았다 — 파지 확인을 못 한다")
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

                    # 재시도는 **명령 궤적**에서 찾는다. 측정값은 청크 경계(3.33초)에서만 보므로
                    # 그 사이에 일어난 시도를 통째로 놓친다 — 2026-09-22 실기에서 실제로는
                    # 여러 번 시도했는데 로그에는 한 번으로 보였다. 청크 안에는 30Hz 100스텝이
                    # 들어 있어 100배 촘촘하고, 무엇보다 **정책의 의도**가 그대로 담겨 있다.
                    lift_cmd = np.asarray(chunk)[:, SHOULDER_LIFT_INDEX]
                    above, attempts, retry_at = _scan_attempts(
                        lift_cmd, above, attempts, self.pcfg.extended_lift_deg,
                        self.pcfg.retry_dip_deg, self.pcfg.returned_lift_deg)
                    self.get_logger().info(
                        f"청크 {chunks + 1} 명령 lift 처음 {lift_cmd[0]:.0f} 최소 {lift_cmd.min():.0f} "
                        f"최대 {lift_cmd.max():.0f} 끝 {lift_cmd[-1]:.0f} · 시도 {attempts}"
                        + (f" · {retry_at} 스텝에서 재시도 시작" if retry_at is not None else ""))
                    if retry_at is not None:
                        # 재시도가 시작되는 지점 **앞까지만** 재생하고 멈춘다. 그래야 팔이
                        # 다시 뻗지 않고, 다음 접근을 Host 가 새 관측으로 시작할 수 있다.
                        if retry_at > 0:
                            self._play(chunk[:retry_at], goal_handle)
                        raise GraspAborted(
                            f"두 번째 시도를 시작했다 — {chunks}청크 + {retry_at}스텝에서 중단. "
                            "재시도는 Host 가 다시 세운 뒤에 한다")
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
                    # 청크마다 lift 를 남긴다 — 재시도 문턱(retry_dip_deg)을 실측으로 정하려면
                    # 실패 회차에서 이 값이 어디까지 내려갔다 올라오는지를 봐야 한다.
                    self.get_logger().info(f"청크 {chunks} 실측 lift {lift:.1f}")
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
            result.held_change_percent = -1.0
            result.gripper_percent = -1.0
            result.gripper_load = -1.0
            try:
                after = self._fresh_frame()
                state = self._arm_state()
                result.gripper_percent = float(state.policy_state[GRIPPER_INDEX])
                result.gripper_load = float(state.load_ratio[GRIPPER_INDEX])
                # 자세가 기준과 같을 때만 비교한다(위 SAME_POSE_TOL_DEG 주석).
                if reference_state is None:
                    pose_gap = None
                else:
                    now = list(state.policy_state)
                    pose_gap = max(abs(a - b) for a, b in
                                   zip(now[:GRIPPER_INDEX], reference_state[:GRIPPER_INDEX]))
                if pose_gap is not None and pose_gap <= SAME_POSE_TOL_DEG:
                    result.held_change_percent = roi_changed_percent(
                        reference, after, self.check.image_roi, self.check.image_pixel_threshold)
                else:
                    gap = "기준 자세 없음" if pose_gap is None else f"관절 최대 {pose_gap:.0f}도 차이"
                    self.get_logger().warn(
                        f"파지 확인 불가 — 시작 자세로 안 돌아왔다({gap}). 영상 비교를 건너뛴다")
            except GraspAborted as exc:
                self.get_logger().warn(f"파지 확인용 관측 실패: {exc}")
            self.get_logger().info(
                f"파지 관측: 근접 변화 {result.held_change_percent:.1f}% · "
                f"그리퍼 {result.gripper_percent:.1f}% · 부하 {result.gripper_load:.2f}")
            # ⚠️ 한 줄에서 심각도를 바꾸면 rclpy 가 죽는다 —
            # "Logger severity cannot be changed between calls" (호출 위치별로 캐시한다).
            # 2026-09-22: 같은 프로세스에서 성공(info) 다음 실패(warn)가 나오자 예외가 터져
            # 액션 결과가 통째로 비었다(chunks 0, "Goal state not set, assuming aborted").
            if result.ok:
                self.get_logger().info(f"VLA 파지 완료: {result.message}")
            else:
                self.get_logger().warn(f"VLA 파지 실패: {result.message}")
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
