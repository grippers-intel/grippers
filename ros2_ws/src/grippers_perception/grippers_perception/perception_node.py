"""perception_node — 카메라 기반 인식. 지금은 실제 비전 파이프라인(YOLO/마커) 미구현.

⚠️ 안전 원칙 (domain/ports/perception.py의 Perception ABC 계약, 실측 전까지 절대
어기면 안 됨):
- monitor_clearance: 모르면 항상 contact_risk=True(정지)로 응답한다. False로 두면
  실제 장애물을 못 보고 밀고 지나가는 사고로 직결된다.
- scan_floor: 모르면 빈 목록으로 응답한다 — SCAN이 이걸 '대상 없음'으로 해석해
  DONE으로 유도한다.
- find_box: 모르면 found=False로 응답한다 — TRANSPORT가 이걸 받으면 대상을
  보류 등록하고 SCAN으로 복귀한다.
- confirm_grasp: 카메라를 못 열거나 기준 프레임이 없으면 confirmed=False다 —
  다른 관측 포트와 같은 "모르면 실패" 관례(domain/ports/perception.py 참고).
"""

import rclpy
from grippers_interfaces.msg import BoxObservation, DetectionArray
from grippers_interfaces.srv import (
    ConfirmGrasp,
    FindBox,
    MeasureOpening,
    MonitorClearance,
    ScanFloor,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

try:
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    _CV_AVAILABLE = True
except ImportError:
    _CV_AVAILABLE = False

try:
    import cv2
    import numpy as np

    _GRASP_CAM_CV_AVAILABLE = True
except ImportError:
    _GRASP_CAM_CV_AVAILABLE = False


# ── confirm_grasp (1단계, classical CV 임시 구현) ───────────────────────────
# YOLO가 아직 안 붙어서(2026-08-21 기준) 실기 로그 수집을 시작하려고 정교한
# 검출 없이 "기준(빈 그리퍼) 프레임과 지금 프레임이 얼마나 다른가"만 본다.
# GRASP는 이 결과를 아직 판정에 안 쓴다(domain/task/states.py GraspState 참고) —
# 로그만 쌓아서 나중에 임계값을 실측으로 잡는다.
GRIPPER_CAM_DEVICE_DEFAULT = "/dev/gripper_cam"
GRIPPER_CAM_WIDTH = 640
GRIPPER_CAM_HEIGHT = 480
GRIPPER_CAM_WARMUP_FRAMES = 5  # 노출 자동조정 전 프레임은 검게 나온다 (실기 확인됨)
# 손가락은 프레임 하단 중앙 일부에만 작게 잡힌다(2026-08-21 실기 스냅샷 확인) —
# 전체 프레임으로 diff를 내면 배경(의자·책상) 변화에 신호가 희석된다. 정확한
# 비율은 카메라 장착이 바뀌면 같이 바뀌니 재장착 후 스냅샷으로 재확인할 것.
GRIPPER_CAM_ROI = (0.30, 0.55, 0.70, 1.00)  # (x0, y0, x1, y1), 프레임 폭/높이 비율
# TODO: 실측 — 지금은 근거 없는 자리 표시자다. confirm_grasp 로그(diff_score)를
# 실제 파지 성공/실패 케이스별로 모은 뒤 재보정한다. ros2 param set으로
# 재배포 없이 튜닝할 수 있게 파라미터로도 노출한다.
CONFIRM_GRASP_DIFF_THRESHOLD_DEFAULT = 15.0


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
        self.create_service(
            ConfirmGrasp,
            "perception/confirm_grasp",
            self._on_confirm_grasp,
            callback_group=cb_group,
        )

        self.declare_parameter("gripper_cam_device", GRIPPER_CAM_DEVICE_DEFAULT)
        self.declare_parameter("confirm_grasp_diff_threshold", CONFIRM_GRASP_DIFF_THRESHOLD_DEFAULT)
        self._grasp_cam = None
        self._grasp_cam_reference = None  # 기준(빈 그리퍼) 프레임 — 그레이스케일
        if _GRASP_CAM_CV_AVAILABLE:
            self._grasp_cam_reference = self._capture_grasp_frame()
            if self._grasp_cam_reference is None:
                self.get_logger().warn(
                    "confirm_grasp: 기준 프레임 캡처 실패 — 그리퍼캠 연결/조명 확인 필요"
                )
        else:
            self.get_logger().warn("opencv 미설치 — confirm_grasp 항상 confirmed=False 반환")

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

    # ---- confirm_grasp (1단계, classical CV 임시 구현) ----
    def _open_grasp_cam(self):
        device = self.get_parameter("gripper_cam_device").value
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, GRIPPER_CAM_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, GRIPPER_CAM_HEIGHT)
        return cap

    def _capture_grasp_frame(self):
        """그리퍼캠에서 그레이스케일 프레임 한 장을 잡는다. 카메라가 없거나
        읽기에 실패하면 **None** — 호출자가 confirmed=False로 접는다.

        열려 있던 캡처가 죽어 있으면(핫플러그 재연결 등) 한 번 다시 연다.
        노출 자동조정이 끝나기 전 프레임은 실기에서 검게 나오는 게 확인됐으므로
        (2026-08-21) 앞쪽 몇 프레임은 버린다."""
        if not _GRASP_CAM_CV_AVAILABLE:
            return None
        if self._grasp_cam is None or not self._grasp_cam.isOpened():
            self._grasp_cam = self._open_grasp_cam()
        if self._grasp_cam is None:
            return None
        for _ in range(GRIPPER_CAM_WARMUP_FRAMES):
            self._grasp_cam.grab()
        ok, frame = self._grasp_cam.read()
        if not ok or frame is None:
            self._grasp_cam.release()
            self._grasp_cam = None
            return None
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def _grasp_roi(self, gray_frame):
        """GRIPPER_CAM_ROI(비율)를 실제 픽셀 슬라이스로 잘라낸다."""
        h, w = gray_frame.shape
        x0, y0, x1, y1 = GRIPPER_CAM_ROI
        return gray_frame[int(y0 * h) : int(y1 * h), int(x0 * w) : int(x1 * w)]

    def _on_confirm_grasp(self, request, response):
        frame = self._capture_grasp_frame()
        if frame is None or self._grasp_cam_reference is None:
            self.get_logger().warn("confirm_grasp: 프레임/기준 없음 — confirmed=False 반환")
            response.confirmed = False
            response.confidence = 0.0
            return response

        # 기준(빈 그리퍼) 프레임과의 평균 절대 밝기 차이 — 정교한 검출이 아니라
        # "뭔가 달라졌다"만 보는 1단계 임시 신호다. GRIPPER_CAM_ROI로 손가락
        # 부근만 잘라서 비교한다 — 전체 프레임으로 하면 배경 변화에 묻힌다
        # (2026-08-21 실기 확인: 전체 프레임 diff는 물체 유무와 무관하게 ~1로 고정).
        # threshold는 미실측 자리 표시자이니 로그(diff_score)를 실제 파지
        # 성공/실패와 대조해 재보정한다.
        diff_score = float(
            np.mean(cv2.absdiff(self._grasp_roi(frame), self._grasp_roi(self._grasp_cam_reference)))
        )
        threshold = self.get_parameter("confirm_grasp_diff_threshold").value
        confirmed = diff_score > threshold
        confidence = max(0.0, min(1.0, diff_score / (2.0 * threshold)))

        self.get_logger().info(
            f"confirm_grasp: diff_score={diff_score:.2f} threshold={threshold:.2f} "
            f"confirmed={confirmed} confidence={confidence:.3f}"
        )
        response.confirmed = confirmed
        response.confidence = confidence
        return response

    def destroy_node(self):
        if self._grasp_cam is not None:
            self._grasp_cam.release()
        super().destroy_node()


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
