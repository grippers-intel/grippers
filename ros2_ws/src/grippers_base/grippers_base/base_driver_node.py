"""base_driver_node — MentorPi mecanum 베이스 제어 노드.
controller/odom_publisher_node가 이미 만들어둔 /cmd_vel(안전 클램프) →
/odom(cmd_vel 적분 dead reckoning)을 그대로 재사용. 새 모터 제어는 안 함,
목표 좌표까지의 proportional 제어 루프 + DriveTo 액션 서버만 얹는다."""

import json
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from grippers_interfaces.action import ApproachObject, DriveTo
from grippers_interfaces.srv import AlignToBox, ObserveTarget
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

from .drive_control import compute_drive_command
from .visual_approach_control import (
    choose_dodge_side,
    compute_approach_command,
    compute_approach_error,
    compute_dodge_command,
    min_range_in_arc,
    obstacle_ahead,
)

ARRIVE_YAW_TOL = 0.05  # rad
DRIVE_TIMEOUT_SEC = 55.0  # client의 60초 결과 timeout 전에 반드시 정지·응답

# ── approach_object (2026-08-23 신설, 실기 미검증) ──────────────────────────
# tools/perception/approach.py를 액션 서버로 이식한다 — domain/ports/
# base_driver.py의 `approach()` docstring, HANDOFF.md "왜 제어 루프인가"
# 참고. 원본과 다르게 튜닝하지 않는다: 여기서 나쁜 결과가 나오면 먼저
# tools/perception/approach.py로 실기 확인하고 이 상수를 맞출 것.
#
# ⚠️ 코드는 구조적으로 완결돼 있지만(perception/observe_target 서비스 +
# 이 액션 서버 + visual_approach_control.py 순수 제어 수학), 카메라·실기
# 없이는 끝까지 검증할 수 없다 — 최초 실기 테스트에서 가장 먼저 의심할
# 지점이다.
APPROACH_TARGET_DIR = "/grippers/config"  # tools/perception/approach.py TARGET_DIR과 동일
APPROACH_MAX_ITER = 40  # tools/perception/approach.py --max-iter 기본값과 동일
APPROACH_TIMEOUT_SEC = 45.0  # client의 60초 결과 timeout보다 여유 있게 짧다
APPROACH_OBSERVE_TIMEOUT_SEC = 1.0

# ── 전방 장애물 회피 (2026-08-23 신설, 실기 미검증) ──────────────────────────
# 사용자 지적: approach가 회전 없이 순수 이동만 쓰다 보니 불필요하게 지그재그로
# 움직였다 — 회전+전진으로 바꾸면서, 전진 경로에 실제 장애물이 있을 때의
# 대비책도 같이 넣는다(회전만으로는 전방 충돌을 못 막는다).
#
# ⚠️ 실기 확인(2026-08-23): LD19 라이다는 `/scan_raw`에 발행한다(launch
# 인자로 `scan` 요청해도 하위 launch가 이렇게 remap함 — peripherals/launch/
# lidar.launch.py 참고). 장착 높이(base_link 기준 9.25cm)에서는 체스말·
# 축구공 같은 파지 대상이 전방 스캔에 안 잡힌다(실측: 최근접 반환이 배경
# 벽까지 뚫려 0.97m) — 즉 이 게이트가 지금 접근 중인 파지 대상 자체를
# 장애물로 오인할 위험은 낮다. 회전+전진 조합과 마찬가지로 이 값들도
# 실기 미검증 자리 표시자다.
SCAN_TOPIC = "/scan_raw"
OBSTACLE_FRONT_HALF_WIDTH_DEG = 20.0
OBSTACLE_SIDE_CENTER_DEG = 60.0
OBSTACLE_SIDE_HALF_WIDTH_DEG = 25.0

# TODO(#148): 아래 세 값은 M3 시연장에서 실측 후 확정한다.
# PHASE 1 -> 2 진입 허용 yaw 오차. 현재 값은 자리 표시자(약 5.7도).
YAW_ALIGN_TOL_RAD = 0.1
# PHASE 2 -> 1 복귀 임계값. 진입값보다 크게 두어 채터링을 막는다.
YAW_REALIGN_TRIG_RAD = 0.3
# 도착 근처 atan2 요동 구간에서는 PHASE 1로 되돌아가지 않는다.
REALIGN_MIN_DIST_M = 0.10

# 클라이언트 ACTION_RESULT_TIMEOUT_SEC(60s)보다 먼저 서버가 정지/abort한다.
DRIVE_TO_TIMEOUT_SEC = 55.0


def _yaw_from_quat(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class BaseDriverNode(Node):
    def __init__(self):
        super().__init__("base_driver_node")
        cb_group = ReentrantCallbackGroup()

        self._cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._pose = None  # (x, y, yaw)
        self._latest_scan = None  # LaserScan — approach_object 장애물 회피용

        # LD19는 신뢰성 있는 배달을 보장 안 하는 BEST_EFFORT로 발행한다
        # (실기 확인) — RELIABLE로 구독하면 QoS 불일치로 아예 안 들어온다.
        scan_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, scan_qos)

        # ⚠️ 2026-08-23: "odom"이 아니라 "odom_raw"를 구독한다 — HANDOFF.md
        # 실기 확인: imu_calib 부재로 EKF를 못 띄워서(grippers_bringup의
        # bringup.launch.py 참고) /odom이 계속 비어 있다. 그대로 두면
        # self._pose가 영원히 None이라 drive_to()가 DRIVE_TIMEOUT_SEC(55초)
        # 마다 항상 실패한다. /odom_raw(바퀴 오도메트리 원본, EKF 미적용)는
        # 실기에서 발행되는 게 확인됐다 — tools/perception/approach.py도
        # 같은 이유로 이 토픽을 쓴다.
        self.create_subscription(Odometry, "odom_raw", self._on_odom, 10)

        self._drive_action_server = ActionServer(
            self,
            DriveTo,
            "base_driver/drive_to",
            execute_callback=self._execute_drive_to,
            callback_group=cb_group,
        )
        self._approach_action_server = ActionServer(
            self,
            ApproachObject,
            "base_driver/approach_object",
            execute_callback=self._execute_approach_object,
            callback_group=cb_group,
        )
        self._observe_client = self.create_client(
            ObserveTarget, "perception/observe_target", callback_group=cb_group
        )
        self.create_service(
            AlignToBox,
            "base_driver/align_to_box",
            self._on_align,
            callback_group=cb_group,
        )
        self.create_service(
            Trigger,
            "base_driver/stop",
            self._on_stop,
            callback_group=cb_group,
        )
        self.get_logger().info("base_driver_node ready")

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._pose = (p.x, p.y, yaw)

    def _on_scan(self, msg: LaserScan):
        self._latest_scan = msg

    def _read_obstacle_sectors(self):
        """(전방, 좌측, 우측) 최소 거리(m)를 돌려준다. 라이다 데이터가 아직
        없으면 셋 다 None — obstacle_ahead(None)이 "모르면 막지 않는다"로
        처리한다(visual_approach_control.py obstacle_ahead 참고)."""
        scan = self._latest_scan
        if scan is None:
            return None, None, None
        front = min_range_in_arc(
            scan.angle_min, scan.angle_increment, scan.ranges,
            center_deg=0.0, half_width_deg=OBSTACLE_FRONT_HALF_WIDTH_DEG,
        )
        left = min_range_in_arc(
            scan.angle_min, scan.angle_increment, scan.ranges,
            center_deg=OBSTACLE_SIDE_CENTER_DEG, half_width_deg=OBSTACLE_SIDE_HALF_WIDTH_DEG,
        )
        right = min_range_in_arc(
            scan.angle_min, scan.angle_increment, scan.ranges,
            center_deg=-OBSTACLE_SIDE_CENTER_DEG, half_width_deg=OBSTACLE_SIDE_HALF_WIDTH_DEG,
        )
        return front, left, right

    def _execute_drive_to(self, goal_handle):
        target = goal_handle.request.target  # Pose2D(x, y, theta)
        rate = self.create_rate(20)
        result = DriveTo.Result()
        started_at = time.monotonic()

        while rclpy.ok():
            if time.monotonic() - started_at >= DRIVE_TIMEOUT_SEC:
                self._cmd_vel_pub.publish(Twist())
                goal_handle.abort()
                result.arrived = False
                return result

            if self._pose is None:
                rate.sleep()
                continue
            x, y, yaw = self._pose
            dx, dy = target.x - x, target.y - y
            command = compute_drive_command(dx, dy, yaw)

            if goal_handle.is_cancel_requested:
                self._cmd_vel_pub.publish(Twist())
                goal_handle.canceled()
                result.arrived = False
                return result

            if command.arrived:
                self._cmd_vel_pub.publish(Twist())
                goal_handle.succeed()
                result.arrived = True
                return result

            twist = Twist()
            twist.linear.x = command.linear_x
            twist.angular.z = command.angular_z
            self._cmd_vel_pub.publish(twist)

            fb = DriveTo.Feedback()
            fb.distance_remaining = command.distance
            goal_handle.publish_feedback(fb)
            rate.sleep()

        self._cmd_vel_pub.publish(Twist())
        result.arrived = False
        return result

    def _execute_approach_object(self, goal_handle):
        """물체 앞 파지 위치로 시각 서보 폐루프 접근한다 —
        원래 tools/perception/approach.py 이식(좌우-이동 방식)이었으나
        2026-08-23 회전+전진으로 재설계했다(visual_approach_control.py
        모듈 상단 경고 참고, 이 조합은 실기 미검증).

        정지→관측(perception/observe_target)→소이동을 반복한다. 매 반복
        관측이 없거나(물체를 순간적으로 놓침) 오차가 커도 다음 반복이
        다시 잡으므로, 한 번 실패했다고 바로 포기하지 않는다 — 최대
        APPROACH_MAX_ITER회, 또는 APPROACH_TIMEOUT_SEC를 넘기면 실패로
        끊는다.

        매 반복 전방 라이다도 함께 확인한다 — 전방 안전거리 안에 뭔가
        있으면(파지 대상 자체는 안 걸린다, obstacle_ahead() 참고) 시각
        서보 지령 대신 옆으로 비키는 지령을 낸다."""
        raw_cls = goal_handle.request.raw_cls
        result = ApproachObject.Result()

        target = self._load_approach_target(raw_cls)
        if target is None:
            self.get_logger().warn(
                f"approach_object: '{raw_cls}' 교시값 없음 — "
                "tools/perception/approach.py --teach로 먼저 만들 것"
            )
            goal_handle.abort()
            result.arrived = False
            return result

        target_x, target_h = target["x"], target["h"]
        started_at = time.monotonic()

        for _ in range(APPROACH_MAX_ITER):
            if time.monotonic() - started_at >= APPROACH_TIMEOUT_SEC:
                break
            if goal_handle.is_cancel_requested:
                self._cmd_vel_pub.publish(Twist())
                goal_handle.canceled()
                result.arrived = False
                return result

            observed = self._observe_target_once(raw_cls)
            if observed is None:
                continue  # 물체를 순간적으로 놓침 — 다음 반복에서 다시 찾는다
            obs_x, obs_h = observed

            err_x, err_h = compute_approach_error(obs_x, obs_h, target_x, target_h)

            front, left, right = self._read_obstacle_sectors()
            if obstacle_ahead(front):
                side = choose_dodge_side(left, right)
                self.get_logger().warn(
                    f"approach_object: 전방 장애물 감지(front={front:.2f}m) — "
                    f"{'왼쪽' if side > 0 else '오른쪽'}으로 회피"
                )
                command = compute_dodge_command(side)
            else:
                command = compute_approach_command(err_x, err_h)

            fb = ApproachObject.Feedback()
            fb.err_x_px = err_x
            fb.err_h_px = err_h
            goal_handle.publish_feedback(fb)

            if command.arrived:
                self._cmd_vel_pub.publish(Twist())
                goal_handle.succeed()
                result.arrived = True
                return result

            twist = Twist()
            twist.linear.x = command.linear_x
            twist.linear.y = command.linear_y
            twist.angular.z = command.angular_z
            self._cmd_vel_pub.publish(twist)
            time.sleep(command.burst_s)
            self._cmd_vel_pub.publish(Twist())

        self._cmd_vel_pub.publish(Twist())
        goal_handle.abort()
        result.arrived = False
        return result

    def _observe_target_once(self, raw_cls):
        """perception/observe_target을 한 번 호출한다. 서비스가 없거나
        응답이 없거나 물체를 못 찾으면 `None`.

        ⚠️ 2026-08-23 실기 확인(첫 전체 FSM 실기 테스트): 여기서
        `rclpy.spin_until_future_complete(self, ...)`를 쓰면 안 된다 — 이
        메서드는 `approach_object` 액션 서버의 execute_callback 안에서
        반복 호출된다. 그 콜백은 이미 이 노드의 MultiThreadedExecutor
        워커 스레드 하나를 점유 중인데, spin_until_future_complete()는
        **같은 노드를 또 스핀하는 임시 executor**를 그 안에서 새로 만든다.
        Pi는 코어 4개(=기본 워커 스레드 4개)뿐이라 반복 중첩되면 스레드가
        고갈된다 — 실기에서 첫 approach_object goal은 끝까지 돌았지만
        두 번째 goal부터 "goal 수락 응답 없음"으로 실패했고, 결국
        base_driver_node가 `base_driver/stop` 서비스 요청에도 응답하지
        못하는 상태(스레드 전부 중첩 대기에 묶임)에 빠졌다.

        대신 future 완료를 콜백 스레드 자체에서 threading.Event로 기다린다
        — 완료 자체(perception_node가 응답을 보내는 것)는 이미 돌고 있는
        바깥 executor의 다른 워커 스레드가 처리하므로 추가 executor가
        필요 없다. domain/adapters/real/_ros_call.py도 같은 이름의 함수를
        쓰지만 거기는 mission_orchestrator의 전용 FSM 스레드(ROS 콜백이
        아님)에서 불려 이 문제가 없다 — 콜백 안에서 부르는 이 자리만
        다르게 짜야 한다."""
        if not self._observe_client.wait_for_service(timeout_sec=APPROACH_OBSERVE_TIMEOUT_SEC):
            return None
        future = self._observe_client.call_async(ObserveTarget.Request(raw_cls=raw_cls))
        done_event = threading.Event()
        future.add_done_callback(lambda _f: done_event.set())
        if not done_event.wait(timeout=APPROACH_OBSERVE_TIMEOUT_SEC):
            future.cancel()
            return None
        res = future.result()
        if res is None or not res.found:
            return None
        return res.x, res.h

    @staticmethod
    def _load_approach_target(raw_cls):
        """tools/perception/approach.py --teach가 저장한 교시값을 읽는다 —
        같은 경로(APPROACH_TARGET_DIR)를 그대로 쓴다. 이 노드가 교시를
        대신하지 않는다 — 교시는 실기에서 사람이 손가락 사이에 물체를
        놓고 하는 작업이다(approach.py --teach 참고)."""
        path = os.path.join(APPROACH_TARGET_DIR, f"approach_target_{raw_cls}.json")
        legacy_path = os.path.join(APPROACH_TARGET_DIR, "approach_target.json")
        try:
            with open(path, encoding="utf-8") as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            pass
        try:
            with open(legacy_path, encoding="utf-8") as f:
                legacy = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return None
        # 이름 없이 저장해 둔 예전 교시본(approach.py load_target()과 동일 규칙).
        return legacy if legacy.get("class") == raw_cls else None

    def _on_align(self, request, response):
        # TODO: request.box(BoxObservation: color/pose/opening_mm/long_axis_rad)를
        # 기준으로 마커·박스 검출 기반 정렬 로직을 붙인다 (지금은 자리만 잡아둠).
        # perception이 실제로 box pose를 재관측해 넘겨주기 전까지는 여기서
        # 할 수 있는 게 없어 항상 성공으로 스텁 응답한다.
        # request.box.color는 와이어 필드명이 아직 레거시라 그렇다 — 2026-08-23
        # 확정 미션 명세서로 domain.values.BoxColor가 Destination(LEFT/RIGHT)
        # 으로 바뀌었고 지금 이 필드엔 그 이름이 들어온다(domain/adapters/
        # real/_ros_convert.py 상단 경고 참고).
        self.get_logger().warn(
            f"align_to_box(dest={request.box.color}): 마커/박스 정렬 미구현 — "
            "aligned=True로 스텁 응답"
        )
        response.aligned = True
        response.yaw_error = 0.0
        return response

    def _on_stop(self, request, response):
        self._cmd_vel_pub.publish(Twist())
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BaseDriverNode()
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
