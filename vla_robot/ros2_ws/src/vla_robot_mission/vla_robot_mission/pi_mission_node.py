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
PLACE:  (알려진 자세인지 확인) -> carry(그리퍼 유지)
        -> drop + base 회전(place.base_yaw_deg + HostCommand.arm_yaw_deg, 그리퍼 유지)
        -> 그리퍼 열기 -> settle -> return_pose
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
from std_msgs.msg import Empty, UInt16
from std_srvs.srv import Trigger

from vla_common.arm_units import GRIPPER_INDEX, with_base_yaw
from vla_common.config import RobotConfig, load_poses, load_robot_config
from vla_common.grasp_check import roi_changed_percent
from vla_common.motion_limits import MotionLimits
from vla_common.protocol import State
from vla_robot_interfaces.action import MoveToPose, RunVlaGrasp
from vla_robot_interfaces.msg import RobotStatus
from vla_robot_interfaces.srv import GetArmState, SetGripper
from vla_robot_mission.base_recovery import BaseRecovery
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


#: 파지 뒤 그리퍼가 idle 개도보다 이만큼(%p) 넘게 열려 있을 때만 "복귀 전 닫기"를 한다.
#: 오늘 쥔 기물은 9.9~26.8%, 빈손으로 닫힌 턱은 ~10% — 정책이 턱을 연 채 끝난 경우만 잡는다.
GRIPPER_LEFT_OPEN_PCT = 20.0


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
        # 작업 안의 단계 — PiStatus.job_stage 로 Host 화면에 간다("" = 작업 없음)
        self.stage = ""

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

    def start(self, job_id: int, action: str, label: str, arm_yaw_deg: float = 0.0,
              place_pose: str = "") -> None:
        with self._lock:
            if self._busy:
                raise RuntimeError("이미 작업 중")
            self._busy = True
        self._cancel.clear()
        threading.Thread(target=self._run, args=(job_id, action, label, float(arm_yaw_deg), place_pose),
                         name=f"job{job_id}", daemon=True).start()

    def poll(self) -> Optional[tuple[int, bool, str]]:
        with self._lock:
            done, self._finished = self._finished, None
            return done

    def cancel(self) -> None:
        """진행 중인 작업을 접는다. **종료 중에도 불린다.**

        2026-09-22 실기: Ctrl-C 로 스택을 내릴 때 여기서 RCLError("node's context is
        invalid")가 나 pi_mission_node 가 exit 1 로 죽었다. 이 시점에는 팔 드라이버가
        이미 자기 shutdown 으로 자세를 잡은 뒤라 실제 피해는 없지만, 종료 경로가
        예외로 끝나면 다음 사람이 "종료가 실패했다"고 읽는다. 여기서는 삼킨다 —
        취소는 최선 노력이고, 안전은 arm_driver 쪽 hold 가 책임진다.
        """
        if self._cancel.is_set():
            return
        self._cancel.set()
        try:
            goal = self._active_goal
            if goal is not None:
                goal.cancel_goal_async()
            if self._hold.service_is_ready():
                self._hold.call_async(Trigger.Request())
        except Exception as exc:  # noqa: BLE001 — 종료 중 컨텍스트가 닫혔을 수 있다
            self._node.get_logger().warn(f"작업 취소 중 무시된 오류: {exc}")

    # -- 실행 -----------------------------------------------------------------
    def _run(self, job_id: int, action: str, label: str, arm_yaw_deg: float = 0.0,
             place_pose: str = "") -> None:
        log = self._node.get_logger()
        started = time.monotonic()
        try:
            log.info(f"작업 {job_id} 시작: {action} label={label or '-'} "
                     f"arm_yaw={arm_yaw_deg:+.1f}도{' · ' + place_pose if place_pose else ''}")
            if action == State.GRASP:
                detail = self._do_grasp(label)
            elif action == State.PLACE:
                detail = self._do_place(arm_yaw_deg, place_pose)
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
            self.stage = ""

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

    def _move_values(self, values, keep_gripper: bool, what: str, timeout_s: float = 20.0):
        """이름 없는 목표로 이동한다(pose_name 이 비면 arm_driver 가 target 을 쓴다)."""
        goal = MoveToPose.Goal(pose_name="", target=[float(v) for v in values],
                               gripper_mode=1 if keep_gripper else 0, duration_s=0.0)
        return self._run_action(self._pose, goal, timeout_s, f"move_to_pose({what})")

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

        # ⚠️ 복귀보다 **그리퍼를 먼저 닫는다.** 정책은 뻗으면서 턱을 열고 내려가서 닫는데,
        # 실패하면 그 사이클이 중간에 끝나 **턱이 열린 채로 남는다.** 그대로 복귀하면 팔이
        # 열린 그리퍼로 움직인다(2026-09-22 관찰). 물체를 쥐고 있으면 그 폭에서 멎으므로
        # 놓치지 않고, 오히려 운반 중에 더 단단히 문다.
        #
        # 이 정렬은 판정의 전제이기도 하다 — 기준 프레임과 **같은 개도**여야 비교가 성립한다.
        #
        # 단, **이미 닫혀 있으면 건드리지 않는다.** 정책이 끝까지 닫아 물체를 조이고 있는데 idle 의
        # 개도(6.9%)로 다시 맞추면 목표가 올라가 조이는 힘이 줄어든다(2026-09-30 soccer 낙하).
        start_pose = self._poses.get(mcfg.start_pose)
        if start_pose is not None:
            idle_open = start_pose.values[GRIPPER_INDEX]
            try:
                now_open = float(self._arm_state().policy_state[GRIPPER_INDEX])
            except JobFailed:
                now_open = None
            if now_open is None or now_open > idle_open + GRIPPER_LEFT_OPEN_PCT:
                try:
                    self._set_gripper(idle_open, "복귀 전 그리퍼 닫기")
                except JobFailed as exc:
                    self._node.get_logger().warn(f"그리퍼 정렬 실패, 그대로 복귀한다: {exc}")

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

    def _do_place(self, arm_yaw_deg: float = 0.0, place_pose: str = "") -> str:
        """차는 상자 정면에 서 있다. **좌우 정렬은 차체가 아니라 팔의 base 가 한다.**

        트는 각도는 두 몫의 합이다.
          place.base_yaw_deg  설정값. 팔의 고정 장착 오프셋처럼 매번 같은 몫
          arm_yaw_deg         Host 가 이번 정차 자리에서 잰 잔차(HostCommand)
        합이 place.max_base_yaw_deg 를 넘으면 거부한다 — 그 각도는 차를 다시 세워야 한다.
        복귀 포즈는 틀지 않는다 — idle 은 정책 시작 자세라 항상 제자리여야 한다.
        """
        pcfg = self._cfg.place
        # place_pose == "handover": 사람 손에 건네기 — 같은 흐름에서 놓는 자세만 바꾸고, 열기 전에 잠깐 기다린다.
        handover = place_pose == "handover"
        pose_name = pcfg.handover_pose if handover else pcfg.drop_pose
        drop = self._poses.get(pose_name)
        if drop is None or not drop.measured:
            raise JobFailed(f"'{pose_name}' 포즈가 실측되지 않았다 — tools/teach_pose.py --name {pose_name}")
        host_yaw = pcfg.host_yaw_sign * float(arm_yaw_deg)
        yaw = pcfg.base_yaw_deg + host_yaw
        try:
            target = with_base_yaw(drop.values, yaw, pcfg.max_base_yaw_deg)
        except ValueError as exc:
            raise JobFailed(
                f"base 회전을 쓸 수 없다: {exc} "
                f"(설정 {pcfg.base_yaw_deg:+.1f} + Host {host_yaw:+.1f})") from None
        note = f" · base {yaw:+.1f}도(설정 {pcfg.base_yaw_deg:+.1f} + Host {host_yaw:+.1f})" if yaw else ""
        self._require_known_pose()
        self.stage = "move"
        self._move(pcfg.carry_pose, keep_gripper=True)
        if yaw:
            self._move_values(target, keep_gripper=True, what=f"{pose_name}{note}")
        else:
            self._move(pose_name, keep_gripper=True)
        if handover:
            time.sleep(pcfg.handover_wait_s)
            self._check_cancel()
        self._set_gripper(pcfg.release_percent, "그리퍼 열기")
        self.stage = "release"                  # 여기서부터 화면의 "쥔 기물"을 지운다
        time.sleep(pcfg.settle_s)
        self.stage = "return"
        self._move(pcfg.return_pose, keep_gripper=False)
        return f"{'건네기' if handover else '투하'} 완료{note}"


ARM_VOLT_POLL_S = 10.0       # 팔 서보 전압 읽는 주기(작업 중에는 건너뛴다)
VOLT_STALE_S = 30.0          # 이보다 오래된 전압은 모름(0)으로 보낸다


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
        self.recovery = BaseRecovery(bcfg.recover_script, bcfg.recover_timeout_s, bcfg.recover_cooldown_s,
                                     log=lambda m: self.get_logger().warn(m))

        cb = ReentrantCallbackGroup()
        self._twist_pub = self.create_publisher(Twist, bcfg.cmd_vel_topic, 10)
        self._status_pub = self.create_publisher(RobotStatus, "/vla_robot/status", 10)
        self.create_subscription(Empty, "/vla_robot/estop", self._on_estop, 10, callback_group=cb)
        self.create_subscription(Empty, "/vla_robot/estop_reset", self._on_estop_reset, 10, callback_group=cb)
        self._feedback_at: Optional[float] = None
        self._born = time.monotonic()
        if Odometry is not None:
            self.create_subscription(Odometry, bcfg.feedback_topic, self._on_feedback, 10, callback_group=cb)
        # 전압 — Host 화면 VEH / ARM 칸. 오래된 값(VOLT_STALE_S)은 0(모름)으로 보낸다.
        self._battery_v, self._battery_at = 0.0, None
        self._arm_v, self._arm_at = 0.0, None
        self.create_subscription(UInt16, bcfg.battery_topic, self._on_battery, 1, callback_group=cb)
        self._arm_state_cli = self.create_client(GetArmState, "arm/get_state", callback_group=cb)
        self.create_timer(ARM_VOLT_POLL_S, self._poll_arm_voltage, callback_group=cb)

        self._last_status_sent = 0.0
        self._last_state_logged = None
        self.create_timer(1.0 / max(self.cfg.mission.cycle_hz, 1.0), self._cycle, callback_group=cb)
        if bcfg.recover_on_start:
            # 한 번만 — 다른 노드가 다 뜬 뒤. 도는 동안 Host 에는 base_recovering 으로 보이고 바퀴는 정지.
            self._start_reset_timer = self.create_timer(
                max(bcfg.recover_on_start_delay_s, 0.1), self._recover_on_start, callback_group=cb)
        self.get_logger().info(
            f"pi_mission_node 준비 boot_id={self.boot_id} UDP {lcfg.command_port}/{lcfg.status_port} "
            f"cmd_vel={bcfg.cmd_vel_topic}")

    def _recover_on_start(self) -> None:
        self._start_reset_timer.cancel()
        self.recovery.request(startup=True)

    def _on_estop(self, _msg) -> None:
        self.get_logger().warn("E-STOP 래치")
        self.core.latch_estop()

    def _on_estop_reset(self, _msg) -> None:
        self.get_logger().info("E-STOP 해제")
        self.core.reset_estop()

    def _on_feedback(self, _msg) -> None:
        self._feedback_at = time.monotonic()

    def _on_battery(self, msg) -> None:
        self._battery_v, self._battery_at = msg.data / 1000.0, time.monotonic()

    def _poll_arm_voltage(self) -> None:
        """팔 작업 중에는 묻지 않는다 — 서보 버스를 정책 스트리밍과 나눠 쓰지 않게. 비동기라 기다리지 않는다."""
        if self.jobs.busy or not self._arm_state_cli.service_is_ready():
            return

        def done(fut) -> None:
            try:
                res = fut.result()
            except Exception:  # noqa: BLE001 — 화면 표시용, 놓쳐도 된다
                return
            if res is not None and res.ok and res.voltage_v > 0:
                self._arm_v, self._arm_at = float(res.voltage_v), time.monotonic()

        self._arm_state_cli.call_async(GetArmState.Request()).add_done_callback(done)

    def _volts(self, now: float) -> tuple[float, float]:
        fresh = lambda at: at is not None and now - at <= VOLT_STALE_S  # noqa: E731
        return (self._battery_v if fresh(self._battery_at) else 0.0,
                self._arm_v if fresh(self._arm_at) else 0.0)

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
            if getattr(cmd, "recover_base", False):
                self.recovery.request()           # 이미 도는 중·쿨다운이면 알아서 무시한다
        out = self.core.step(now, self._base_alive(now))

        twist = Twist()
        if not self.recovery.recovering:          # 컨트롤러를 다시 띄우는 동안은 정지만 낸다
            twist.linear.x = out.motion.linear_x
            twist.linear.y = out.motion.linear_y
            twist.angular.z = out.motion.angular_z
        self._twist_pub.publish(twist)

        battery_v, arm_v = self._volts(now)
        st = replace(out.status, base_recovering=self.recovery.recovering,
                     base_recoveries=self.recovery.count, battery_v=battery_v, arm_v=arm_v,
                     job_stage=self.jobs.stage if self.jobs.busy else "")
        if st.base_recovering:
            st = replace(st, detail="차체 컨트롤러 복구 중")
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
            try:
                self._twist_pub.publish(Twist())
            except Exception as exc:  # noqa: BLE001 — 종료 경로
                print(f"[pi_mission_node] 종료 정지 명령 실패: {exc}", flush=True)
                break
        self.link.close()


def main(args=None) -> None:
    # rclpy 의 SIGINT 처리기는 컨텍스트부터 닫는다 — 그러면 아래 finally 에서 보내는 **마지막
    # 정지 명령**이 "publisher's context is invalid" 로 실패한다(2026-09-30 로그). 주행 중에
    # 스택을 내리면 바퀴가 돌던 속도가 남는다. 그래서 SIGINT 는 파이썬이 KeyboardInterrupt 로
    # 받게 두고, 정지를 먼저 보낸 뒤 우리가 컨텍스트를 닫는다.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except ImportError:  # pragma: no cover — 옛 rclpy
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
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
