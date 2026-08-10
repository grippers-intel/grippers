"""Ros2ArmDriver — mission_orchestrator가 쓰는 ArmDriver 포트 구현.
arm_driver_node에 액션/서비스로 말을 건다."""

import rclpy
from geometry_msgs.msg import Point
from grippers_interfaces.action import MoveToCartesian
from grippers_interfaces.srv import GetLoad, SetGripper
from rclpy.action import ActionClient

from domain.ports.arm_driver import ArmDriver


class Ros2ArmDriver(ArmDriver):
    def __init__(self, node):
        self._node = node
        self._move_client = ActionClient(node, MoveToCartesian, "arm_driver/move_to_cartesian")
        self._gripper_client = node.create_client(SetGripper, "arm_driver/set_gripper")
        self._load_client = node.create_client(GetLoad, "arm_driver/get_load")

    def move_to_cartesian(self, xyz, grip=None, down=False) -> bool:
        self._move_client.wait_for_server()
        goal = MoveToCartesian.Goal(
            target=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
            grip=float(grip) if grip is not None else 0.0,
            down=down,
        )
        future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        return result_future.result().result.reached

    def set_gripper(self, deg: float) -> None:
        self._gripper_client.wait_for_service()
        req = SetGripper.Request(closed=(deg < 50))
        future = self._gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)

    def get_load(self) -> float:
        self._load_client.wait_for_service()
        future = self._load_client.call_async(GetLoad.Request())
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().load_ratio
