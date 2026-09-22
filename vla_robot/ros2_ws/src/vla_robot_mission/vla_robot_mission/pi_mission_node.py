"""pi_mission_node — Host UDP 명령을 받아 차체를 움직이고 팔 작업을 돌린다.

    UDP  <- HostCommand (5005)       -> PiStatus (5006)
    pub  base.cmd_vel_topic (Twist)   매 사이클, 정지도 계속 낸다
    sub  base.feedback_topic          도착 시각만 본다(구동계 생존)
    pub  /vla_robot/status (RobotStatus)
    sub  /vla_robot/estop (Empty)     래치
    sub  /vla_robot/estop_reset (Empty)

    action client  vla/run_grasp, arm/move_to_pose
    srv client     arm/get_state, arm/set_gripper, arm/hold

판단 로직은 pi_mission_core.PiMissionCore 에 있고, 이 파일은 ROS 배선과 팔 작업 순서만 갖는다.

## 팔 작업 순서

GRASP:  (알려진 자세인지 확인) -> start_pose 로 이동 -> VLA 파지 -> 파지 확인(개구율/부하)
        -> carry 포즈(그리퍼 유지)
PLACE:  (알려진 자세인지 확인) -> carry(그리퍼 유지) -> drop(그리퍼 유지) -> 그리퍼 열기
        -> settle -> return_pose
"""
from __future__ import annotations

import threading
import time
import uuid
from dataclasses import replace
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from vla_common.arm_units import GRIPPER_INDEX
from vla_common.config import RobotConfig, load_poses, load_robot_config
from vla_common.grasp_check import roi_changed_percent
from vla_common.motion_limits import MotionLimits
from vla_common.protocol import State
from vla_robot_interfaces.action import MoveToPose, RunVlaGrasp
from vla_robot_interfaces.msg import RobotStatus
from vla_robot_interfaces.srv import GetArmState, SetGripper
from vla_robot_mission.pi_mission_core import PiMissionCore
from vla_robot_mission.udp_link import UdpLink

try:
    from nav_msgs.msg import Odometry
except ImportError:  # pragma: no cover
    Odometry = None


def wait_future(future, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while not future.done():
        if time.monotonic() > deadline:
            return False
        time.sleep(0.005)
    return True


class JobFailed(Exception):
    pass


class RosJobRunner:
    """팔 작업을 백그라운드 스레드에서 순서대로 실행한다."""

    def __init__(self, node: Node, cfg: RobotConfig) -> None:
        self._node = node
        self._cfg = cfg
        self._poses = load_poses(cfg.arm.poses_file)
        cb = ReentrantCallbackGroup()
        self._grasp = ActionClient(node, RunVlaGrasp, "vla/run_grasp", callback_group=cb)
        self._pose = ActionClient(node, MoveToPose, "arm/move_to_pose", callback_group=cb)
        self._state = node.create_client(GetArmState, "arm/get_state", callback_group=cb)
        self._gripper = node.create_client(SetGripper, "arm/set_gripper", callback_group=cb)
        self._hold = node.create_client(Trigger, "arm/hold", callback_group=cb)

        # 파지 확인용 프레임. 판정은 **팔이 시작 자세로 돌아온 뒤** 같은 자세끼리 비교한다.
        self._frame = None
        self._frame_lock = threading.Lock()
        node.create_subscription(
            Image, cfg.gripper_cam.topic, self._on_frame,
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=cb)

        self._lock = threading.Lock()
        self._busy = False
        self._finished: Optional[tuple[int, bool, str]] = None
        self._cancel = threading.Event()
        self._active_goal = None

    def _on_frame(self, msg: Image) -> None:
        if msg.encoding != "bgr8":
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        with self._frame_lock:
            self._frame = frame.copy()

    def _grab_frame(self, timeout_s: float = 2.0):
        """최신 프레임 한 장. 못 받으면 None — 모르는 것을 성공으로 읽지 않는다."""
        with self._frame_lock:
            self._frame = None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._frame_lock:
                if self._frame is not None:
                    return self._frame
            time.sleep(0.05)
        return None

    # -- JobRunner 프로토콜 ----------------------------------------------------
    @property
    def busy(self) -> bool:
        with self._lock:
            return self._busy

    def start(self, job_id: int, action: str, label: str) -> None:
        with self._lock:
            if self._busy:
                raise RuntimeError("이미 작업 중")
            self._busy = True
        self._cancel.clear()
        threading.Thread(target=self._run, args=(job_id, action, label),
                         name=f"job{job_id}", daemon=True).start()

    def poll(self) -> Optional[tuple[int, bool, str]]:
        with self._lock:
            done, self._finished = self._finished, None
            return done

    def cancel(self) -> None:
        if self._cancel.is_set():
            return
        self._cancel.set()
        goal = self._active_goal
        if goal is not None:
            goal.cancel_goal_async()
        if self._hold.service_is_ready():
            self._hold.call_async(Trigger.Request())

    # -- 실행 -----------------------------------------------------------------
    def _run(self, job_id: int, action: str, label: str) -> None:
        log = self._node.get_logger()
        started = time.monotonic()
        try:
            log.info(f"작업 {job_id} 시작: {action} label={label or '-'}")
            if action == State.GRASP:
                detail = self._do_grasp(label)
            elif action == State.PLACE:
                detail = self._do_place()
            else:
                raise JobFailed(f"모르는 작업: {action}")
            ok = True
        except JobFailed as exc:
            ok, detail = False, str(exc)
        except Exception as exc:  # noqa: BLE001 — 작업 스레드는 결과를 반드시 남긴다
            ok, detail = False, f"예외: {exc}"
        detail = f"{detail} ({time.monotonic() - started:.1f}s)"
        # ⚠️ 한 줄에서 심각도를 바꾸지 말 것 — rclpy 가 호출 위치별로 캐시해서 예외를 던진다
        # (2026-09-22 실기에서 vla_grasp_node 가 같은 이유로 액션 결과를 잃었다).
        if ok:
            log.info(f"작업 {job_id} 성공: {detail}")
        else:
            log.warn(f"작업 {job_id} 실패: {detail}")
        with self._lock:
            self._finished = (job_id, ok, detail)
            self._busy = False
            self._active_goal = None

    def _check_cancel(self) -> None:
        if self._cancel.is_set():
            raise JobFailed("E-STOP 으로 취소됨")

    def _arm_state(self) -> GetArmState.Response:
        if not self._state.wait_for_service(timeout_sec=2.0):
            raise JobFailed("arm/get_state 서비스가 없다 — arm_driver_node 확인")
        future = self._state.call_async(GetArmState.Request())
        if not wait_future(future, 3.0) or future.result() is None:
            raise JobFailed("arm/get_state 응답 없음")
        res = future.result()
        if not res.ok:
            raise JobFailed(f"팔 상태 읽기 실패: {res.message}")
        return res

    def _require_known_pose(self) -> None:
        now = list(self._arm_state().policy_state)
        tol = self._cfg.mission.known_pose_tolerance_deg
        names = {self._cfg.mission.start_pose, self._cfg.place.carry_pose, self._cfg.place.return_pose}
        best_name, best_err = None, float("inf")
        for name in names:
            pose = self._poses.get(name)
            if pose is None or not pose.measured:
                continue
            err = max(abs(a - b) for a, b in zip(now[:GRIPPER_INDEX], pose.values[:GRIPPER_INDEX]))
            if err < best_err:
                best_name, best_err = name, err
        if best_err > tol:
            raise JobFailed(
                f"팔이 알려진 자세가 아니다(가장 가까운 {best_name} 에서 {best_err:.0f}도) — "
                "바닥을 쓸 수 있어 자동 이동하지 않는다. 사람이 팔을 idle 근처로 옮길 것")

    def _run_action(self, client: ActionClient, goal, timeout_s: float, name: str):
        self._check_cancel()
        if not client.wait_for_server(timeout_sec=3.0):
            raise JobFailed(f"{name} 액션 서버가 없다")
        send = client.send_goal_async(goal)
        if not wait_future(send, 5.0) or send.result() is None or not send.result().accepted:
            raise JobFailed(f"{name} goal 거부")
        handle = send.result()
        self._active_goal = handle
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout_s
        while not result_future.done():
            if self._cancel.is_set():
                handle.cancel_goal_async()
                wait_future(result_future, 3.0)
                raise JobFailed("E-STOP 으로 취소됨")
            if time.monotonic() > deadline:
                handle.cancel_goal_async()
                raise JobFailed(f"{name} 시간 초과 {timeout_s:.0f}s")
            time.sleep(0.02)
        self._active_goal = None
        result = result_future.result().result
        if not result.ok:
            raise JobFailed(f"{name} 실패: {result.message}")
        return result

    def _move(self, pose_name: str, keep_gripper: bool, timeout_s: float = 20.0):
        goal = MoveToPose.Goal(pose_name=pose_name, gripper_mode=1 if keep_gripper else 0, duration_s=0.0)
        return self._run_action(self._pose, goal, timeout_s, f"move_to_pose({pose_name})")

    def _set_gripper(self, percent: float, what: str) -> float:
        self._check_cancel()
        if not self._gripper.wait_for_service(timeout_sec=2.0):
            raise JobFailed("arm/set_gripper 서비스가 없다")
        future = self._gripper.call_async(SetGripper.Request(percent=float(percent)))
        if not wait_future(future, 10.0) or future.result() is None or not future.result().ok:
            msg = future.result().message if future.done() and future.result() else "응답 없음"
            raise JobFailed(f"{what} 실패: {msg}")
        return float(future.result().final_percent)

    def _do_grasp(self, label: str) -> str:
        """파지 한 번. 판정은 **복귀한 뒤** 그리퍼캠으로 한다.

        순서가 곧 판정의 전제다 — 기준 프레임과 비교 프레임이 **같은 자세**여야 한다.
        2026-09-22 실기: 자세가 다른 채로 비교했더니 빈손인데 50.8% 가 나왔다.
        """
        mcfg, check = self._cfg.mission, self._cfg.grasp_check
        self._require_known_pose()
        self._move(mcfg.start_pose, keep_gripper=False)
        reference = self._grab_frame()
        if reference is None and check.enabled and check.method == "image":
            raise JobFailed("그리퍼캠 프레임이 없다 — 파지를 확인할 수 없어 시작하지 않는다")

        failure = None
        try:
            grasp = self._run_action(
                self._grasp, RunVlaGrasp.Goal(label=label, timeout_s=0.0),
                mcfg.grasp_timeout_s, "vla/run_grasp")
        except JobFailed as exc:
            grasp, failure = None, exc

        # 성공·실패와 무관하게 **먼저 집으로 돌린다.** 정책은 사이클 끝(복귀 문턱이나 재시도
        # 골짜기)에서 멈추므로 팔이 공중에 남고, 그대로 두면 다음 작업이
        # `_require_known_pose` 에서 거부되어 재시도 자체가 막힌다.
        # 그리퍼는 유지한다 — 물체를 쥐고 있을 수 있다.
        try:
            self._move(mcfg.start_pose, keep_gripper=True)
        except JobFailed as recover:
            raise JobFailed(f"{failure or '파지 후'} / 복귀 실패: {recover}") from None
        if failure is not None:
            raise failure

        # ⚠️ 비교 전에 **그리퍼 개도도 기준과 맞춘다.** 턱이 ROI 아래쪽을 크게 차지해서,
        # 물체가 없어도 턱이 벌어진 것만으로 30% 가 바뀐다(2026-09-22 거짓 성공).
        # 물체를 쥐고 있으면 그 폭에서 멎고, 빈손이면 기준과 같은 자리까지 닫힌다.
        start_pose = self._poses.get(mcfg.start_pose)
        if start_pose is not None:
            self._set_gripper(start_pose.values[GRIPPER_INDEX], "비교 전 그리퍼 정렬")

        state = self._arm_state()
        opening = float(state.policy_state[GRIPPER_INDEX])
        load = float(state.load_ratio[GRIPPER_INDEX])
        changed = -1.0
        if reference is not None:
            after = self._grab_frame()
            if after is not None:
                changed = roi_changed_percent(reference, after, check.image_roi,
                                              check.image_pixel_threshold)
        observed = f"근접 변화 {changed:.1f}% · 그리퍼 {opening:.1f}% · 부하 {load:.2f}"

        if check.enabled:
            if check.method == "image":
                if changed < 0:
                    raise JobFailed(f"파지를 확인할 수 없다(프레임 없음) — {observed}")
                if changed < check.image_changed_percent:
                    raise JobFailed(f"빈손으로 보인다: {observed} "
                                    f"(기준 {check.image_changed_percent:.0f}%)")
            elif check.method == "opening":
                # ⚠️ TPU 턱에서는 빈손과 겹친다. 근거는 GraspCheckConfig 주석.
                if opening < check.min_gripper_percent:
                    raise JobFailed(f"빈손으로 보인다: {observed} (기준 {check.min_gripper_percent:.1f}%)")
                if check.min_load_ratio > 0 and load < check.min_load_ratio:
                    raise JobFailed(f"그리퍼 부하가 낮다: {observed}")
        return f"파지 {grasp.chunks}청크 — {observed}"

    def _do_place(self) -> str:
        pcfg = self._cfg.place
        drop = self._poses.get(pcfg.drop_pose)
        if drop is None or not drop.measured:
            raise JobFailed(f"'{pcfg.drop_pose}' 포즈가 실측되지 않았다 — tools/teach_pose.py --name {pcfg.drop_pose}")
        self._require_known_pose()
        self._move(pcfg.carry_pose, keep_gripper=True)
        self._move(pcfg.drop_pose, keep_gripper=True)
        self._set_gripper(pcfg.release_percent, "그리퍼 열기")
        time.sleep(pcfg.settle_s)
        self._move(pcfg.return_pose, keep_gripper=False)
        return "투하 완료"


class PiMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("pi_mission_node")
        self.declare_parameter("config_file", "")
        self.cfg = load_robot_config(str(self.get_parameter("config_file").value))
        bcfg, lcfg = self.cfg.base, self.cfg.link

        self.boot_id = uuid.uuid4().hex[:8]
        self.jobs = RosJobRunner(self, self.cfg)
        self.core = PiMissionCore(
            MotionLimits(bcfg.max_linear_mps, bcfg.max_angular_rad_s, bcfg.reject_mixed_rotation),
            bcfg.watchdog_s, self.boot_id, self.jobs)
        self.link = UdpLink(lcfg.bind_ip, lcfg.command_port, lcfg.status_port, lcfg.fixed_host_ip,
                            log=lambda m: self.get_logger().info(m))

        cb = ReentrantCallbackGroup()
        self._twist_pub = self.create_publisher(Twist, bcfg.cmd_vel_topic, 10)
        self._status_pub = self.create_publisher(RobotStatus, "/vla_robot/status", 10)
        self.create_subscription(Empty, "/vla_robot/estop", self._on_estop, 10, callback_group=cb)
        self.create_subscription(Empty, "/vla_robot/estop_reset", self._on_estop_reset, 10, callback_group=cb)
        self._feedback_at: Optional[float] = None
        self._born = time.monotonic()
        if Odometry is not None:
            self.create_subscription(Odometry, bcfg.feedback_topic, self._on_feedback, 10, callback_group=cb)

        self._last_status_sent = 0.0
        self._last_state_logged = None
        self.create_timer(1.0 / max(self.cfg.mission.cycle_hz, 1.0), self._cycle, callback_group=cb)
        self.get_logger().info(
            f"pi_mission_node 준비 boot_id={self.boot_id} UDP {lcfg.command_port}/{lcfg.status_port} "
            f"cmd_vel={bcfg.cmd_vel_topic}")

    def _on_estop(self, _msg) -> None:
        self.get_logger().warn("E-STOP 래치")
        self.core.latch_estop()

    def _on_estop_reset(self, _msg) -> None:
        self.get_logger().info("E-STOP 해제")
        self.core.reset_estop()

    def _on_feedback(self, _msg) -> None:
        self._feedback_at = time.monotonic()

    def _base_alive(self, now: float) -> bool:
        if self._twist_pub.get_subscription_count() == 0:
            return False
        if Odometry is None:
            return True
        if self._feedback_at is None:
            # 기동 직후에는 피드백이 아직 없을 수 있다
            return now - self._born < self.cfg.base.feedback_timeout_s * 3
        return now - self._feedback_at <= self.cfg.base.feedback_timeout_s

    def _cycle(self) -> None:
        now = time.monotonic()
        incoming = self.link.take_new()
        if incoming is not None:
            cmd, received_at = incoming
            self.core.on_command(cmd, received_at)
        out = self.core.step(now, self._base_alive(now))

        twist = Twist()
        twist.linear.x = out.motion.linear_x
        twist.linear.y = out.motion.linear_y
        twist.angular.z = out.motion.angular_z
        self._twist_pub.publish(twist)

        st = out.status
        if st.state != self._last_state_logged:
            self.get_logger().info(f"상태 {self._last_state_logged} -> {st.state} {st.detail}")
            self._last_state_logged = st.state
        if now - self._last_status_sent >= 1.0 / max(self.cfg.link.status_hz, 1.0):
            self._last_status_sent = now
            if self.link.bad_packets:
                note = f"잘못된 패킷 {self.link.bad_packets}개: {self.link.last_bad_reason}"
                st = replace(st, detail=f"{st.detail} / {note}" if st.detail else note)
            self.link.send_status(st)
            msg = RobotStatus()
            msg.stamp = self.get_clock().now().to_msg()
            msg.state, msg.busy, msg.job_id = st.state, st.busy, st.job_id
            if st.result is not None:
                msg.last_action, msg.last_ok, msg.last_detail = st.result.action, st.result.ok, st.result.detail
            msg.base_ok, msg.watchdog = st.base_ok, st.watchdog
            msg.estop_latched = self.core.estop_latched
            msg.host_address = self.link.host_address
            self._status_pub.publish(msg)

    def shutdown(self) -> None:
        self.jobs.cancel()
        for _ in range(3):
            self._twist_pub.publish(Twist())
        self.link.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PiMissionNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
