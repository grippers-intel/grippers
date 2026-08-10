# -*- coding: utf-8 -*-
"""perception_node — 카메라 기반 인식. 지금은 실제 비전 파이프라인(YOLO/마커) 미구현.
안전 원칙: monitor_clearance는 모르면 contact_risk=True(정지)로 응답한다.
detect_target/measure_gap은 모르면 found=False/기본값으로 응답해 재시도를 유도한다."""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, Vector3, Pose2D
from grippers_interfaces.srv import (
    DetectTarget, MeasureGap, SetLightProfile, MonitorClearance,
)

try:
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    _CV_AVAILABLE = True
except ImportError:
    _CV_AVAILABLE = False


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        cb_group = ReentrantCallbackGroup()

        self._latest_frame = None
        self._bridge = CvBridge() if _CV_AVAILABLE else None
        if _CV_AVAILABLE:
            self.create_subscription(
                Image, 'camera/color/image_raw', self._on_image, 10,
                callback_group=cb_group,
            )
        else:
            self.get_logger().warn('cv_bridge 미설치 — 카메라 구독 비활성화')

        self.create_service(
            DetectTarget, 'perception/detect_target',
            self._on_detect_target, callback_group=cb_group,
        )
        self.create_service(
            MeasureGap, 'perception/measure_gap',
            self._on_measure_gap, callback_group=cb_group,
        )
        self.create_service(
            SetLightProfile, 'perception/set_light_profile',
            self._on_set_light_profile, callback_group=cb_group,
        )
        self.create_service(
            MonitorClearance, 'perception/monitor_clearance',
            self._on_monitor_clearance, callback_group=cb_group,
        )
        self.get_logger().info('perception_node ready (vision pipeline: NOT IMPLEMENTED)')

    def _on_image(self, msg):
        self._latest_frame = msg  # TODO: cv_bridge.imgmsg_to_cv2 후 YOLO/마커 파이프라인 연결

    # ---- 서비스 콜백 (전부 TODO — 지금은 정직하게 미구현 응답) ----
    def _on_detect_target(self, request, response):
        self.get_logger().warn('detect_target: 비전 파이프라인 미구현 — found=False 반환')
        response.found = False
        response.pose = Pose()
        response.dims = Vector3()
        return response

    def _on_measure_gap(self, request, response):
        self.get_logger().warn('measure_gap: 비전 파이프라인 미구현 — 기본값 반환')
        response.h_gap = 0.0
        response.centerline = Pose2D()
        return response

    def _on_set_light_profile(self, request, response):
        self.get_logger().info(f'set_light_profile({request.profile}): 카메라 노출/AWB 제어 TODO')
        response.ready = True  # 프로파일 전환 자체는 실패로 볼 이유가 없어 True
        return response

    def _on_monitor_clearance(self, request, response):
        # 안전 원칙: 실제 측정 전까지는 항상 정지 신호
        self.get_logger().warn('monitor_clearance: 비전 파이프라인 미구현 — contact_risk=True(정지) 반환')
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


if __name__ == '__main__':
    main()
