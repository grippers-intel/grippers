"""Ros2Perception — mission_orchestrator가 쓰는 Perception 포트 구현.
perception_node에 서비스로 말을 건다.

⚠️ domain.values ↔ ROS2 메시지 변환은 반드시 여기(그리고 이 파일의 형제
어댑터들)에서만 한다. domain.values 인스턴스를 ROS2 메시지 생성자 자리에
그대로 넘기면 안 된다 — geometry_msgs/Pose2D 같은 rclpy 메시지는 자기
필드 타입을 assert로 검사하므로, domain.values.Pose2D를 그 자리에 그대로
넘기면 런타임에 AssertionError가 난다. 필드 하나하나를 명시적으로
꺼내 옮긴다."""

import rclpy
from grippers_interfaces.srv import FindBox, MeasureOpening, MonitorClearance, ScanFloor

from domain.adapters.real._ros_convert import box_observation_from_msg, box_observation_to_msg
from domain.ports.perception import Perception
from domain.values import BoxColor, BoxObservation, Clearance, Detection, ObjectClass, Point3


def _detection_from_msg(msg) -> Detection:
    return Detection(
        track_id=msg.track_id,
        cls=ObjectClass[msg.cls],
        pose_m=Point3(x=msg.pose.x, y=msg.pose.y, z=msg.pose.z),
        dims_m=Point3(x=msg.dims.x, y=msg.dims.y, z=msg.dims.z),
        yaw_rad=msg.yaw_rad,
        confidence=msg.confidence,
    )


class Ros2Perception(Perception):
    def __init__(self, node):
        self._node = node
        self._scan_client = node.create_client(ScanFloor, "perception/scan_floor")
        self._find_box_client = node.create_client(FindBox, "perception/find_box")
        self._measure_opening_client = node.create_client(
            MeasureOpening, "perception/measure_opening"
        )
        self._clearance_client = node.create_client(
            MonitorClearance, "perception/monitor_clearance"
        )

    def scan_floor(self) -> list[Detection]:
        self._scan_client.wait_for_service()
        future = self._scan_client.call_async(ScanFloor.Request())
        rclpy.spin_until_future_complete(self._node, future)
        res = future.result()
        return [_detection_from_msg(d) for d in res.detections.detections]

    def find_box(self, color: BoxColor) -> BoxObservation | None:
        self._find_box_client.wait_for_service()
        req = FindBox.Request(color=color.name)
        future = self._find_box_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        res = future.result()
        if not res.found:
            return None
        return box_observation_from_msg(res.box)

    def measure_opening(self, box: BoxObservation) -> float:
        self._measure_opening_client.wait_for_service()
        req = MeasureOpening.Request(box=box_observation_to_msg(box))
        future = self._measure_opening_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().opening_mm

    def monitor_clearance(self) -> Clearance:
        self._clearance_client.wait_for_service()
        future = self._clearance_client.call_async(MonitorClearance.Request())
        rclpy.spin_until_future_complete(self._node, future)
        res = future.result()
        # MonitorClearance.srv에는 top 필드도 있지만 domain.values.Clearance에는
        # 없다 — 서버 쪽(perception_node, #9 범위 밖) 인터페이스는 그대로 두고
        # 여기서만 조용히 버린다.
        return Clearance(
            front_m=res.front,
            left_m=res.left,
            right_m=res.right,
            contact_risk=res.contact_risk,
        )
