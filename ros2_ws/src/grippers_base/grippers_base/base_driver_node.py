"""base_driver_node — MentorPi mecanum 베이스 제어 노드.

controller/odom_publisher_node가 이미 만들어둔 /cmd_vel(안전 클램프)과
/odom을 재사용한다. 현재 /odom은 휠 엔코더/EKF가 아니라 cmd_vel 명령값을
적분한 오픈루프 dead reckoning이다. 새 모터 제어는 추가하지 않고,
목표 좌표까지의 DriveTo 액션 서버를 얹는다.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from grippers_interfaces.action import DriveTo
from grippers_interfaces.srv import AlignToBox
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger

from .drive_control import forward_speed

ARRIVE_XY_TOL = 0.03  # m
KP_LINEAR = 0.6
KP_ANGULAR = 1.2
MAX_LINEAR = 0.2  # app_cmd_vel_callback 클램프와 동일
MAX_ANGULAR = 0.5

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

        self.create_subscription(Odometry, "odom", self._on_odom, 10)

        self._drive_action_server = ActionServer(
            self,
            DriveTo,
            "base_driver/drive_to",
            execute_callback=self._execute_drive_to,
            callback_group=cb_group,
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

    def _execute_drive_to(self, goal_handle):
        target = goal_handle.request.target  # Pose2D(x, y, theta)
        rate = self.create_rate(20)
        result = DriveTo.Result()
        started_at = time.monotonic()

        # #148: 회전과 직진을 동시에 명령하면 오픈루프 odom에서 목표 주변을
        # 도는 발산이 생긴다. 먼저 제자리 정렬하고, 그 뒤에는 직진만 한다.
        phase = "ALIGN"

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._cmd_vel_pub.publish(Twist())
                goal_handle.canceled()
                result.arrived = False
                return result

            elapsed = time.monotonic() - started_at
            if elapsed >= DRIVE_TO_TIMEOUT_SEC:
                self._cmd_vel_pub.publish(Twist())
                self.get_logger().error(
                    f"drive_to timeout after {elapsed:.1f}s "
                    f"(phase={phase}, target=({target.x:.3f}, {target.y:.3f}))"
                )
                goal_handle.abort()
                result.arrived = False
                return result

            if self._pose is None:
                rate.sleep()
                continue

            x, y, yaw = self._pose
            dx, dy = target.x - x, target.y - y
            dist = math.hypot(dx, dy)

            if dist <= ARRIVE_XY_TOL:
                self._cmd_vel_pub.publish(Twist())
                goal_handle.succeed()
                result.arrived = True
                return result

            target_yaw = math.atan2(dy, dx)
            yaw_err = math.atan2(
                math.sin(target_yaw - yaw),
                math.cos(target_yaw - yaw),
            )

            twist = Twist()

            if phase == "ALIGN":
                # PHASE 1: 제자리 회전만 한다.
                twist.linear.x = 0.0
                twist.angular.z = max(
                    -MAX_ANGULAR,
                    min(MAX_ANGULAR, KP_ANGULAR * yaw_err),
                )

                if abs(yaw_err) <= YAW_ALIGN_TOL_RAD:
                    phase = "DRIVE"
                    twist.angular.z = 0.0
                    self.get_logger().info(
                        "drive_to phase ALIGN -> DRIVE "
                        f"(dist={dist:.3f}m, yaw_err={yaw_err:.3f}rad)"
                    )

            else:
                # PHASE 2: 직진만 한다. 회전 명령을 섞지 않아 #148의
                # 목표 주변 회전 발산을 구조적으로 막는다.
                # #148 잔여: dist 는 부호가 없어 목표가 등 뒤에 있어도 전진했다.
                # 전진축 투영을 쓰면 뒤에 있을 때 음수가 되어 후진으로 거리를 줄인다.
                twist.linear.x = forward_speed(dist, yaw_err)
                twist.angular.z = 0.0

                # 슬립 등으로 방위가 크게 틀어졌다면 다시 제자리 정렬한다.
                # 단, 도착 근처에서는 atan2가 불안정하므로 재정렬하지 않는다.
                if dist > REALIGN_MIN_DIST_M and abs(yaw_err) >= YAW_REALIGN_TRIG_RAD:
                    phase = "ALIGN"
                    twist.linear.x = 0.0
                    self.get_logger().warn(
                        "drive_to phase DRIVE -> ALIGN "
                        f"(dist={dist:.3f}m, yaw_err={yaw_err:.3f}rad)"
                    )

            self._cmd_vel_pub.publish(twist)

            fb = DriveTo.Feedback()
            fb.distance_remaining = dist
            goal_handle.publish_feedback(fb)
            rate.sleep()

        self._cmd_vel_pub.publish(Twist())
        result.arrived = False
        return result

    def _on_align(self, request, response):
        # TODO: request.box(BoxObservation: color/pose/opening_mm/long_axis_rad)를
        # 기준으로 마커·박스 검출 기반 정렬 로직을 붙인다 (지금은 자리만 잡아둠).
        # perception이 실제로 box pose를 재관측해 넘겨주기 전까지는 여기서
        # 할 수 있는 게 없어 항상 성공으로 스텁 응답한다.
        self.get_logger().warn(
            f"align_to_box(color={request.box.color}): 마커/박스 정렬 미구현 — "
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
