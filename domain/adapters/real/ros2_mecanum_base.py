# -*- coding: utf-8 -*-
"""Ros2MecanumBase — mission_orchestrator가 쓰는 BaseDriver 포트 구현.
base_driver_node에 액션/서비스로 말을 건다. domain.values.Pose2D <->
geometry_msgs/Pose2D 변환은 여기서만 한다 (domain은 ROS2 타입을 모름)."""
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose2D as RosPose2D
from grippers_interfaces.action import DriveTo
from grippers_interfaces.srv import AlignToCenterline
from std_srvs.srv import Trigger
from domain.ports.base_driver import BaseDriver


class Ros2MecanumBase(BaseDriver):
    def __init__(self, node):
        self._node = node
        self._drive_client = ActionClient(node, DriveTo, 'base_driver/drive_to')
        self._align_client = node.create_client(AlignToCenterline, 'base_driver/align')
        self._stop_client = node.create_client(Trigger, 'base_driver/stop')

    def drive_to(self, target) -> bool:
        self._drive_client.wait_for_server()
        ros_target = RosPose2D(x=target.x, y=target.y, theta=target.theta)
        goal = DriveTo.Goal(target=ros_target)
        future = self._drive_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future)
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        return result_future.result().result.arrived

    def align_to_centerline(self) -> float:
        self._align_client.wait_for_service()
        future = self._align_client.call_async(AlignToCenterline.Request())
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().yaw_error

    def stop(self) -> None:
        self._stop_client.wait_for_service()
        self._stop_client.call_async(Trigger.Request())
