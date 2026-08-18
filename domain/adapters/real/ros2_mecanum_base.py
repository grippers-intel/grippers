"""Ros2MecanumBase — mission_orchestrator가 쓰는 BaseDriver 포트 구현.
base_driver_node에 액션/서비스로 말을 건다. domain.values ↔ geometry_msgs
변환은 여기서만 한다 (domain은 ROS2 타입을 모름) — Pose2D 인스턴스를
geometry_msgs/Pose2D 생성자 자리에 그대로 넘기면 rclpy가 필드 타입을
assert로 검사해서 런타임 AssertionError가 난다."""

import math

from geometry_msgs.msg import Pose2D as RosPose2D
from grippers_interfaces.action import DriveTo
from grippers_interfaces.srv import AlignToBox
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

from domain.adapters.real._ros_call import ESTOP_TIMEOUT_SEC, call_action, call_service
from domain.adapters.real._ros_convert import box_observation_to_msg
from domain.ports.base_driver import BaseDriver
from domain.values import BoxObservation, Pose2D

# 정렬에 실패했을 때 돌려줄 yaw 오차. 0.0(= 완벽 정렬)을 쓰면 실패가 최선의
# 성공으로 읽힌다 — 임계값과 비교하는 어떤 판정도 통과해 버린다. 무한대는
# 어떤 허용 오차와 비교해도 실패로 남는다.
ALIGN_FAILED_YAW_ERROR_RAD = math.inf


class Ros2MecanumBase(BaseDriver):
    def __init__(self, node):
        self._node = node
        self._drive_client = ActionClient(node, DriveTo, "base_driver/drive_to")
        self._align_client = node.create_client(AlignToBox, "base_driver/align_to_box")
        self._stop_client = node.create_client(Trigger, "base_driver/stop")

    def drive_to(self, target: Pose2D) -> bool:
        """도착하면 True. 액션 서버가 없거나 결과가 오지 않으면 **False** —
        `APPROACH`/`TRANSPORT` 가 대상을 보류 등록하고 `SCAN` 으로 복귀한다."""
        ros_target = RosPose2D(x=target.x, y=target.y, theta=target.theta)
        goal = DriveTo.Goal(target=ros_target)
        result = call_action(self._node, self._drive_client, goal, label="drive_to")
        if result is None:
            return False
        return result.arrived

    def align_to_box(self, box: BoxObservation) -> float:
        """정렬 후 yaw 오차(rad). 서비스가 없거나 응답이 없으면
        **`ALIGN_FAILED_YAW_ERROR_RAD`(무한대)** — 실패를 정렬 성공으로 읽지 않는다."""
        req = AlignToBox.Request(box=box_observation_to_msg(box))
        res = call_service(self._node, self._align_client, req, label="align_to_box")
        if res is None:
            return ALIGN_FAILED_YAW_ERROR_RAD
        return res.yaw_error

    def stop(self) -> None:
        # 응답을 기다리지 않는다 — E-STOP 경로에서 호출되므로 (states.py EstopState)
        # 늦어지면 안 된다. 같은 이유로 wait_for_service()도 인자 없이 부르면 안 된다 —
        # 서비스가 안 떠 있을 때 무기한 블록돼 의도가 정반대로 뒤집힌다.
        if not self._stop_client.wait_for_service(timeout_sec=ESTOP_TIMEOUT_SEC):
            self._node.get_logger().error("stop: 서비스 없음 — 정지 실패")
            return
        self._stop_client.call_async(Trigger.Request())
