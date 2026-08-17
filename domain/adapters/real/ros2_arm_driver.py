"""Ros2ArmDriver — mission_orchestrator가 쓰는 ArmDriver 포트 구현.
arm_driver_node에 액션/서비스로 말을 건다. domain.values.Point3 인스턴스를
geometry_msgs/Point 생성자 자리에 그대로 넘기면 rclpy가 필드 타입을
assert로 검사해서 런타임 AssertionError가 난다 — 여기서 필드별로 옮긴다."""

import rclpy
from geometry_msgs.msg import Point
from grippers_interfaces.action import MoveToCartesian, ReorientArm
from grippers_interfaces.srv import GetLoad, SetGripper
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

from domain.ports.arm_driver import ArmDriver
from domain.values import Point3


class Ros2ArmDriver(ArmDriver):
    def __init__(self, node):
        self._node = node
        self._move_client = ActionClient(node, MoveToCartesian, "arm_driver/move_to_cartesian")
        self._reorient_client = ActionClient(node, ReorientArm, "arm_driver/reorient")
        self._gripper_client = node.create_client(SetGripper, "arm_driver/set_gripper")
        self._load_client = node.create_client(GetLoad, "arm_driver/get_load")
        self._fold_client = node.create_client(Trigger, "arm_driver/fold_to_cradle")
        self._hold_client = node.create_client(Trigger, "arm_driver/hold_position")

    def move_to_cartesian(self, xyz_m: Point3, down: bool = False) -> bool:
        self._move_client.wait_for_server()
        goal = MoveToCartesian.Goal(
            target=Point(x=xyz_m.x, y=xyz_m.y, z=xyz_m.z),
            down=down,
        )
        future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        return result_future.result().result.reached

    def set_gripper(self, width_mm: float) -> None:
        # ⚠️ 단위 변경: 예전엔 deg(각도)를 받아 SetGripper.srv의 bool closed로
        # 이진화했다. 이제 width_mm(mm)을 그대로 실어 보낸다 — 각도 변환은
        # arm_driver_node의 캘리브레이션 테이블 몫이지 여기서 하지 않는다.
        self._gripper_client.wait_for_service()
        req = SetGripper.Request(width_mm=width_mm)
        future = self._gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)

    def get_load(self) -> float:
        self._load_client.wait_for_service()
        future = self._load_client.call_async(GetLoad.Request())
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().load_ratio

    def reorient(self, phi_rad: float) -> bool:
        self._reorient_client.wait_for_server()
        goal = ReorientArm.Goal(phi=phi_rad)
        future = self._reorient_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        return result_future.result().result.settled

    def fold_to_cradle(self) -> bool:
        self._fold_client.wait_for_service()
        future = self._fold_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().success

    def hold_position(self) -> None:
        # stop()과 같은 이유로 응답을 기다리지 않는다 — E-STOP 경로에서 호출되므로
        # (states.py EstopState) 늦어지면 안 된다.
        self._hold_client.wait_for_service()
        self._hold_client.call_async(Trigger.Request())
