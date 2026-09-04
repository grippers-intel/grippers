"""vla_inference_node — VLA 정책으로 파지 한 번을 수행한다.

미션 FSM 의 GRASP 를 대신하는 노드다. 정책을 들고 있는 곳은 여기 하나이고,
미션 노드는 액션으로 부르기만 한다 — classic 백엔드만 쓸 때 토치 무게를
하나도 지지 않게 하려는 것이다.

## 한 바퀴

    그리퍼캠 프레임(토픽)  +  관절값(GetArmState.policy_state)
      -> PolicyRunner.predict_chunk  ->  100스텝 청크
      -> arm_driver/execute_joint_chunk 로 재생
      -> 반복

## ⚠️ 카메라를 직접 열지 않는다

`/dev/gripper_cam` 은 perception_node 가 붙들고 있다. 여기서 또 열면
`Device or resource busy` 다. perception_node 의 `gripper_cam_publish_hz` 를
0 보다 크게 켜서 토픽으로 받는다. 프레임은 이미 **180도 회전이 적용된**
상태로 오므로(gripper_cam_geometry.orient) 여기서 다시 돌리면 안 된다 —
학습 때와 같은 방향이어야 한다.

## ⚠️ 관절값을 직접 읽지 않는다

시리얼 포트는 arm_driver 가 독점한다. 그리고 정책 좌표계 변환을 아는 곳도
arm_driver 하나여야 한다 — 두 좌표계의 Homing_Offset 이 달라 wrist_roll 에서
85.8도까지 어긋나고, 그 차이는 서보 EEPROM 에 있지 git 에 있지 않다.
그래서 `GetArmState.policy_state` 를 그대로 쓴다.

## 완료 판정

`shoulder_lift` 가 한 번 펴졌다가(> EXTENDED_DEG) 다시 접히면(< RETURNED_DEG)
한 바퀴가 끝난 것으로 본다. 학습 118회차 중 **117회차**에서 이 규칙이 맞았고,
평균 506프레임(16.9초), 최대 712프레임에서 걸렸다(2026-09-04 측정).

⚠️ 이 판정은 "동작이 끝났다"이지 **"물체를 집었다"가 아니다.** 진짜 판정은
미션 FSM 이 classic 과 똑같은 두 신호(서보 부하 + 뎁스 카메라)로 한다.
여기서 성공을 판정하면 정책이 자기 실패를 스스로 판정하는 꼴이 된다.
"""

import threading
import time

import numpy as np
import rclpy
from grippers_interfaces.action import ExecuteJointChunk, RunVlaGrasp
from grippers_interfaces.srv import GetArmState
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from grippers_vla.policy_runner import PolicyRunner

#: 완료 판정 임계값(도, LeRobot 단위의 shoulder_lift).
#: 학습 데이터에서 lift 는 -104 에서 시작해 뻗을 때 +99 까지 간다.
EXTENDED_DEG = -50.0
RETURNED_DEG = -95.0
#: 완료 판정 전에 최소 이만큼은 돈다. 시작 자세가 이미 "접힘"이라
#: 첫 청크에서 곧바로 끝난 것으로 읽히는 것을 막는다.
MIN_CHUNKS = 2
#: 안전 상한. 실측 최대가 7.1청크였다.
MAX_CHUNKS = 10
#: 프레임이 이보다 오래되면 안 쓴다. 청크가 3.3초이므로 1초면 충분히 신선하다.
MAX_FRAME_AGE_S = 1.0


def _bgr_from_image_msg(msg):
    """Image(bgr8) -> numpy BGR. cv_bridge 를 쓰지 않는다.

    이 환경의 cv_bridge 는 numpy 2.x 에서 세그폴트를 낸다(perception_node 의
    같은 주석 참고). 인코딩이 bgr8 이 아니면 None — 조용히 색을 바꿔 넘기면
    정책이 학습과 다른 색을 본다."""
    if msg.encoding != "bgr8":
        return None
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    return buf.reshape(msg.height, msg.width, 3)


class VlaInferenceNode(Node):
    def __init__(self):
        super().__init__("vla_inference_node")
        cb_group = ReentrantCallbackGroup()

        self.declare_parameter("checkpoint", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("gripper_cam_topic", "gripper_cam/image_raw")
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("max_step_deg", 5.0)
        self.declare_parameter("timeout_s", 45.0)
        # ⚠️ 줄이지 말 것. ACT 는 시간을 안 봐서 "언제 펴는가"가 청크 안에
        # 들어 있다. 2026-09-02 실측: 30 으로 줄였더니 그리퍼가 열리기 직전에
        # 잘려 같은 청크를 무한 반복했고 팔이 25초 동안 안 움직였다.
        # 0 이면 체크포인트 값(100)을 그대로 쓴다.
        self.declare_parameter("n_action_steps", 0)

        ckpt = str(self.get_parameter("checkpoint").value or "").strip()
        if not ckpt:
            raise RuntimeError("checkpoint 파라미터가 필요합니다")
        n_steps = int(self.get_parameter("n_action_steps").value or 0)

        self.get_logger().info(f"정책 적재 중: {ckpt}")
        t0 = time.monotonic()
        self._runner = PolicyRunner(
            ckpt, device=str(self.get_parameter("device").value),
            n_action_steps=n_steps if n_steps > 0 else None,
        )
        self.get_logger().info(
            f"정책 준비 {time.monotonic() - t0:.1f}s — "
            f"입력 {self._runner.policy_hw}, chunk {self._runner.chunk_size}, "
            f"n_action_steps {self._runner.n_action_steps}"
        )

        self._frame = None          # (stamp_sec, bgr)
        self._frame_lock = threading.Lock()
        self.create_subscription(
            Image, str(self.get_parameter("gripper_cam_topic").value),
            self._on_frame, 1, callback_group=cb_group)

        self._state_client = self.create_client(
            GetArmState, "arm_driver/get_arm_state", callback_group=cb_group)
        self._chunk_client = ActionClient(
            self, ExecuteJointChunk, "arm_driver/execute_joint_chunk",
            callback_group=cb_group)

        self._server = ActionServer(
            self, RunVlaGrasp, "vla_inference/run_grasp",
            execute_callback=self._execute,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb_group,
        )
        self.get_logger().info("vla_inference_node ready")

    def _on_frame(self, msg):
        frame = _bgr_from_image_msg(msg)
        if frame is None:
            self.get_logger().warn(
                f"그리퍼캠 인코딩이 bgr8 이 아닙니다: {msg.encoding}",
                throttle_duration_sec=5.0)
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._frame_lock:
            self._frame = (stamp, frame)

    def _latest_frame(self):
        """충분히 신선한 프레임. 없으면 None."""
        with self._frame_lock:
            item = self._frame
        if item is None:
            return None
        stamp, frame = item
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - stamp > MAX_FRAME_AGE_S:
            return None
        return frame

    def _read_policy_state(self):
        """정책 좌표계의 관절값 6개. 못 읽으면 None."""
        if not self._state_client.wait_for_service(timeout_sec=2.0):
            return None
        future = self._state_client.call_async(GetArmState.Request())
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return None
        response = future.result()
        if response is None or not response.ok or not response.policy_state_valid:
            return None
        return list(response.policy_state)

    def _send_chunk(self, chunk) -> bool:
        """청크를 arm_driver 에 넘기고 재생이 끝날 때까지 기다린다."""
        if not self._chunk_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("execute_joint_chunk 서버가 없습니다")
            return False
        goal = ExecuteJointChunk.Goal(
            positions=[float(v) for v in np.asarray(chunk, dtype=np.float32).ravel()],
            fps=float(self.get_parameter("fps").value),
            max_step_deg=float(self.get_parameter("max_step_deg").value),
        )
        send = self._chunk_client.send_goal_async(goal)
        deadline = time.monotonic() + 5.0
        while not send.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        handle = send.result() if send.done() else None
        if handle is None or not handle.accepted:
            self.get_logger().error("청크 goal 이 거부됐습니다")
            return False
        # 재생 시간 + 여유. 100스텝 30fps = 3.33초.
        play_s = len(chunk) / max(float(self.get_parameter("fps").value), 1.0)
        result_future = handle.get_result_async()
        deadline = time.monotonic() + play_s + 10.0
        while not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not result_future.done():
            self.get_logger().error("청크 재생 결과를 못 받았습니다")
            return False
        result = result_future.result().result
        if not result.ok:
            self.get_logger().error(f"청크 재생 실패: {result.message}")
        return bool(result.ok)

    def _execute(self, goal_handle):
        request = goal_handle.request
        result = RunVlaGrasp.Result()
        feedback = RunVlaGrasp.Feedback()
        label = request.label or "queen"
        # ⚠️ ACT 는 이 문자열을 입력으로 받지 않는다. 후일 언어 정책과 기록용이다.
        task = f"pick up the {label}"
        timeout_s = (float(request.timeout_s) if request.timeout_s > 0
                     else float(self.get_parameter("timeout_s").value))

        started = time.monotonic()
        chunks = 0
        extended = False
        try:
            while chunks < MAX_CHUNKS:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.ok, result.chunks = False, chunks
                    result.message = f"취소됨 — {chunks}청크"
                    return result
                if time.monotonic() - started > timeout_s:
                    result.ok, result.chunks = False, chunks
                    result.message = f"시간 초과 {timeout_s:.0f}s — {chunks}청크"
                    self.get_logger().warn(result.message)
                    goal_handle.abort()
                    return result

                frame = self._latest_frame()
                if frame is None:
                    result.ok, result.chunks = False, chunks
                    result.message = ("그리퍼캠 프레임이 없습니다 — perception_node 의 "
                                      "gripper_cam_publish_hz 가 0 이 아닌지 보십시오")
                    self.get_logger().error(result.message)
                    goal_handle.abort()
                    return result
                state = self._read_policy_state()
                if state is None:
                    result.ok, result.chunks = False, chunks
                    result.message = ("관절값을 못 읽었습니다 — arm_driver 의 "
                                      "policy_calibration_file 이 설정돼 있는지 보십시오")
                    self.get_logger().error(result.message)
                    goal_handle.abort()
                    return result

                chunk = self._runner.predict_chunk(frame, state, task)
                if not np.isfinite(chunk).all():
                    result.ok, result.chunks = False, chunks
                    result.message = "정책이 NaN/Inf 를 냈습니다"
                    self.get_logger().error(result.message)
                    goal_handle.abort()
                    return result
                if not self._send_chunk(chunk):
                    result.ok, result.chunks = False, chunks
                    result.message = "청크 재생 실패"
                    goal_handle.abort()
                    return result

                chunks += 1
                feedback.chunk = chunks
                feedback.elapsed_s = time.monotonic() - started
                goal_handle.publish_feedback(feedback)

                # 완료 판정은 **명령이 아니라 실측 자세**로 한다. 정책이 접으라고
                # 했어도 팔이 거기 못 갔을 수 있다.
                #
                # ⚠️ 못 읽었으면 판정을 **건너뛴다.** 0.0 같은 기본값을 쓰면
                # 그 값이 EXTENDED_DEG(-50)보다 커서 "뻗었다"로 잘못 걸리고,
                # 다음 청크에서 곧바로 "복귀"로 읽혀 파지가 안 끝났는데 성공을
                # 돌려준다. 한 번 못 읽는 것은 흔한 일이라(시리얼 패킷 유실)
                # 다음 청크에서 다시 보면 된다.
                measured = self._read_policy_state()
                if measured is None:
                    self.get_logger().warn("완료 판정용 관절값 읽기 실패 — 다음 청크에서 다시 봅니다")
                    continue
                lift = measured[1]
                if lift > EXTENDED_DEG:
                    extended = True
                if extended and chunks >= MIN_CHUNKS and lift < RETURNED_DEG:
                    result.ok, result.chunks = True, chunks
                    result.message = (f"{chunks}청크 {time.monotonic() - started:.1f}s "
                                      f"— 뻗었다가 복귀 (lift {lift:.1f})")
                    self.get_logger().info(result.message)
                    goal_handle.succeed()
                    return result

            result.ok, result.chunks = False, chunks
            result.message = f"{MAX_CHUNKS}청크를 다 썼는데 복귀를 못 봤습니다"
            self.get_logger().warn(result.message)
            goal_handle.abort()
            return result
        except Exception as e:  # noqa: BLE001 — 실기 루프
            self.get_logger().error(f"VLA 파지 예외: {e}")
            result.ok, result.chunks = False, chunks
            result.message = str(e)
            goal_handle.abort()
            return result


def main(args=None):
    rclpy.init(args=args)
    node = VlaInferenceNode()
    executor = MultiThreadedExecutor()
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
