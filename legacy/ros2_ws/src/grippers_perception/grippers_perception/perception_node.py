"""perception_node — 카메라 기반 인식. 지금은 실제 비전 파이프라인(YOLO/마커) 미구현.

⚠️ 안전 원칙 (domain/ports/perception.py의 Perception ABC 계약, 실측 전까지 절대
어기면 안 됨):
- monitor_clearance: 모르면 항상 contact_risk=True(정지)로 응답한다. False로 두면
  실제 장애물을 못 보고 밀고 지나가는 사고로 직결된다.
- scan_floor: 모르면 빈 목록으로 응답한다 — SCAN이 이걸 '대상 없음'으로 해석해
  DONE으로 유도한다.
- find_box: 모르면 found=False로 응답한다 — TRANSPORT가 이걸 받으면 대상을
  보류 등록하고 SCAN으로 복귀한다.
"""

import rclpy
from grippers_interfaces.msg import BoxObservation, DetectionArray
from grippers_interfaces.srv import FindBox, MeasureOpening, MonitorClearance, ScanFloor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

try:
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    _CV_AVAILABLE = True
except ImportError:
    _CV_AVAILABLE = False


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        cb_group = ReentrantCallbackGroup()

        self._latest_frame = None
        self._bridge = CvBridge() if _CV_AVAILABLE else None
        if _CV_AVAILABLE:
            self.create_subscription(
                Image,
                "camera/color/image_raw",
                self._on_image,
                10,
                callback_group=cb_group,
            )
        else:
            self.get_logger().warn("cv_bridge 미설치 — 카메라 구독 비활성화")

        self.create_service(
            ScanFloor,
            "perception/scan_floor",
            self._on_scan_floor,
            callback_group=cb_group,
        )
        self.create_service(
            FindBox,
            "perception/find_box",
            self._on_find_box,
            callback_group=cb_group,
        )
        self.create_service(
            MeasureOpening,
            "perception/measure_opening",
            self._on_measure_opening,
            callback_group=cb_group,
        )
        self.create_service(
            MonitorClearance,
            "perception/monitor_clearance",
            self._on_monitor_clearance,
            callback_group=cb_group,
        )
        self.get_logger().info("perception_node ready (vision pipeline: NOT IMPLEMENTED)")

    def _on_image(self, msg):
        self._latest_frame = msg  # TODO: cv_bridge.imgmsg_to_cv2 후 YOLO/마커 파이프라인 연결

    # ---- 서비스 콜백 (전부 TODO — 지금은 정직하게 미구현 응답) ----
    def _on_scan_floor(self, request, response):
        self.get_logger().warn("scan_floor: 비전 파이프라인 미구현 — 빈 목록 반환")
        # TODO: 상자 영역 마스킹 (state_machine.md §4 재진입 방지 방어선) — 실제
        # 검출 파이프라인이 붙으면, 여기서 상자 ROI와 겹치는 detection을 걸러내야
        # 한다. 필터링을 빼먹으면 이미 처리된 상자 내부 물체를 계속 재검출해
        # 무한 루프 방지의 첫 번째 방어선(done_ids/held_ids 필터링)이 무력화된다.
        response.detections = DetectionArray(detections=[])
        return response

    def _on_find_box(self, request, response):
        self.get_logger().warn(
            f"find_box(color={request.color}): 비전 파이프라인 미구현 — found=False 반환"
        )
        response.found = False
        response.box = BoxObservation()
        return response

    def _on_measure_opening(self, request, response):
        self.get_logger().warn("measure_opening: 비전 파이프라인 미구현 — 0.0 반환")
        response.opening_mm = 0.0
        return response

    def _on_monitor_clearance(self, request, response):
        # 안전 원칙: 실제 측정 전까지는 항상 정지 신호. 절대 False로 바꾸지 말 것.
        self.get_logger().warn(
            "monitor_clearance: 비전 파이프라인 미구현 — contact_risk=True(정지) 반환"
        )
        response.front = 0.0
        response.left = 0.0
        response.right = 0.0
        response.top = 0.0
        response.contact_risk = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
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
