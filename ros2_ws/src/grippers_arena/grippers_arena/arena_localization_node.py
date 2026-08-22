"""arena_localization_node — 외부(아레나) 카메라로 ArUco를 읽어 좌표계를
세우고 로봇 자세를 퍼블리시한다 (미션 명세서 2026-08-23 파이프라인 02번).

⚠️ 아직 실기로 검증 안 됨 — 이번 세션 내내 Pi/외부 카메라 둘 다 연결 안 된
상태라 이 노드를 실제로 띄워 본 적이 없다. 순수 수학(aruco_localization.py)
만 pytest로 검증했다. 카메라가 연결되면 반드시 확인할 것:
- 아래 이미지 토픽 이름(`arena_cam_a/image_raw` 등)이 실제 카메라 드라이버가
  퍼블리시하는 이름과 맞는지 (`ros2 topic list`로 확인 — perception_node.py의
  RGB_CAMERA_INFO_TOPIC_DEFAULT와 같은 처지의 추측값이다)
- `MARKER_LAYOUT_MM`을 실측(줄자)으로 채웠는지 — 지금은 전부 `None`
  placeholder라 호모그래피가 절대 안 잡힌다(의도적 — 지어낸 좌표로 "일단
  도는 것처럼" 만들지 않는다)
- `ROBOT_MARKER_ID_DEFAULT`가 실제 로봇에 붙인 마커 ID와 맞는지
- `aruco_localization.robot_pose_from_marker_corners`의 `front_edge` 기본값이
  실제 마커 부착 방향과 맞는지(그 함수 docstring의 확인 절차 참고)

카메라 두 대(A·B, `tools/a2/a2_homography.py`와 같은 명명 규약)를 각각
독립적으로 처리한다 — 호모그래피도 카메라별로 따로 구한다. 두 카메라가
`tools/a2/README_A2.md` §1이 정한 같은 원점·축을 공유하는 세계 좌표계를
쓰는 한, 로봇이 어느 카메라에 잡히든 같은 (x, y)가 나와야 한다."""

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node

try:
    import cv2
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    _CV_AVAILABLE = True
except ImportError:
    _CV_AVAILABLE = False

from grippers_arena.aruco_localization import (
    axis_aligned_marker_world_corners,
    build_correspondences,
    fit_homography_dlt,
    robot_pose_from_marker_corners,
)

# ⚠️ 전부 미실측 placeholder — 아레나에 마커를 실제로 붙이고 줄자로 잰 뒤
# 채울 것. tools/a2/README_A2.md §1의 공유 원점·축 규약을 그대로 따른다.
# 값을 채우기 전엔 build_correspondences()가 항상 빈 결과를 내서
# fit_homography_dlt()가 점 부족으로 None을 반환한다 — "모르면 실패"로
# 안전하게 막힌다(지어낸 좌표로 도는 것처럼 보이게 하지 않는다).
MARKER_LAYOUT_MM = {
    # marker_id: (center_x_mm, center_y_mm, size_mm) — TODO 실측
}
ROBOT_MARKER_ID_DEFAULT = -1  # TODO: 실제 로봇에 붙인 마커 ID로 교체
ARUCO_DICT_DEFAULT = "DICT_4X4_50"  # TODO: 실제로 인쇄한 마커 사전과 맞출 것


def _marker_layout_corners():
    layout = {}
    for marker_id, (cx, cy, size) in MARKER_LAYOUT_MM.items():
        if cx is None or cy is None or size is None:
            continue
        layout[marker_id] = axis_aligned_marker_world_corners((cx, cy), size)
    return layout


class ArenaLocalizationNode(Node):
    def __init__(self):
        super().__init__("arena_localization_node")

        self.declare_parameter("robot_marker_id", ROBOT_MARKER_ID_DEFAULT)
        self.declare_parameter("aruco_dict", ARUCO_DICT_DEFAULT)

        self._bridge = CvBridge() if _CV_AVAILABLE else None
        self._marker_layout = _marker_layout_corners()
        self._homography_by_cam = {}  # cam_id -> 호모그래피(구해지면 캐시)
        self._detector = self._build_detector() if _CV_AVAILABLE else None

        if not self._marker_layout:
            self.get_logger().warn(
                "MARKER_LAYOUT_MM이 비어 있음(미실측) — 좌표계를 절대 못 세운다. "
                "아레나 마커 실측 후 grippers_arena/arena_localization_node.py를 채울 것."
            )

        self._pose_pubs = {}
        for cam_id, topic in (("A", "arena_cam_a/image_raw"), ("B", "arena_cam_b/image_raw")):
            if _CV_AVAILABLE:
                self.create_subscription(
                    Image, topic, lambda msg, cid=cam_id: self._on_frame(cid, msg), 10
                )
            self._pose_pubs[cam_id] = self.create_publisher(Pose2D, f"arena/robot_pose_{cam_id}", 10)

        if not _CV_AVAILABLE:
            self.get_logger().warn("cv2/cv_bridge 미설치 — arena_localization_node 비활성")

    def _build_detector(self):
        dict_name = self.get_parameter("aruco_dict").value
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        params = cv2.aruco.DetectorParameters()
        return cv2.aruco.ArucoDetector(aruco_dict, params)

    def _on_frame(self, cam_id, msg):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        corners, ids, _rejected = self._detector.detectMarkers(frame)
        if ids is None:
            return

        detections = {
            int(marker_id[0]): tuple((float(u), float(v)) for u, v in corner_set[0])
            for marker_id, corner_set in zip(ids, corners, strict=True)
        }

        h = self._homography_for(cam_id, detections)
        if h is None:
            return

        robot_marker_id = self.get_parameter("robot_marker_id").value
        robot_corners = detections.get(robot_marker_id)
        if robot_corners is None:
            return

        pose = robot_pose_from_marker_corners(h, robot_corners)
        if pose is None:
            return
        x_mm, y_mm, theta_rad = pose
        self._pose_pubs[cam_id].publish(Pose2D(x=x_mm, y=y_mm, theta=theta_rad))

    def _homography_for(self, cam_id, detections):
        """카메라별 호모그래피를 한 번 구해 캐시한다. 카메라는 고정 삼각대에
        올려두는 게 전제(tools/a2/README_A2.md의 A1-c 파라미터 고정과 같은
        전제)라 매 프레임 다시 풀 필요가 없다 — 다시 풀면 그 프레임의 검출
        노이즈가 그대로 좌표계 자체를 흔든다."""
        if cam_id in self._homography_by_cam:
            return self._homography_by_cam[cam_id]

        image_points, world_points = build_correspondences(detections, self._marker_layout)
        h = fit_homography_dlt(image_points, world_points)
        if h is not None:
            self._homography_by_cam[cam_id] = h
            self.get_logger().info(f"arena_localization: 카메라 {cam_id} 호모그래피 확정")
        return h


def main(args=None):
    rclpy.init(args=args)
    node = ArenaLocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
