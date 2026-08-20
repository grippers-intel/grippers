"""base_driver_node — MentorPi mecanum 베이스 제어 노드.
controller/odom_publisher_node가 이미 만들어둔 /cmd_vel(안전 클램프) →
/odom(cmd_vel 적분 dead reckoning)을 그대로 재사용. 새 모터 제어는 안 함,
목표 좌표까지의 proportional 제어 루프 + DriveTo 액션 서버만 얹는다."""

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

from .drive_control import compute_drive_command

ARRIVE_YAW_TOL = 0.05  # rad
DRIVE_TIMEOUT_SEC = 55.0  # client의 60초 결과 timeout 전에 반드시 정지·응답


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
