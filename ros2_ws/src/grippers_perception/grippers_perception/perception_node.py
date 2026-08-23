"""perception_node — 카메라 기반 인식.

scan_floor는 Hailo-10H YOLO로 실제 검출을 반환할 수 있지만 `scan_floor_enabled`
파라미터(기본값 False)로 잠겨 있다(2026-08-21, 구조 검증용 — 모듈 하단 HAILO_*
상수 블록의 경고 참고: pose_m은 자리표시자고 클래스 매핑도 불완전하다). 게이트를
켜지 않으면 지금까지처럼 빈 목록을 반환한다. find_box/measure_opening은 아직
정직한 미구현 스텁.

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

import math
import time

import rclpy
from geometry_msgs.msg import Point, Vector3
from grippers_interfaces.msg import BoxObservation, Detection, DetectionArray
from grippers_interfaces.srv import (
    ConfirmGrasp,
    FindBox,
    MeasureOpening,
    MonitorClearance,
    ObserveTarget,
    ScanFloor,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

# 클래스 이름·매핑은 각 백엔드별 매핑 모듈로 뽑았다 — rclpy 없이 순수 pytest로
# 테스트하려면(2026-08-22, PR #185 리뷰 후속) 이 파일 밖에 둬야 한다.
# perception_node.py는 rclpy를 무조건 import해서 ROS2 없이는 아예 못 불러온다.
from grippers_perception.cpu_yolo_scan_mapping import (
    object_class_for_cpu_yolo_class_name,
)
from grippers_perception.floor_consensus import CONF_THRESHOLD, confirmed_tracks, track_bbox_xyxy
from grippers_perception.hailo_scan_mapping import HAILO_CLASS_NAMES, object_class_for_hailo_id

try:
    from sensor_msgs.msg import CameraInfo, Image

    _CV_AVAILABLE = True
except ImportError:
    _CV_AVAILABLE = False

try:
    import cv2
    import numpy as np

    _GRASP_CAM_CV_AVAILABLE = True
except ImportError:
    _GRASP_CAM_CV_AVAILABLE = False

try:
    from hailo_platform import FormatType, HailoSchedulingAlgorithm, VDevice

    _HAILO_AVAILABLE = True
except ImportError:
    _HAILO_AVAILABLE = False

# 🔴 임시 비활성화 (2026-08-22, 배포 전용 — 커밋 안 함): AI HAT+2가 부팅마다
# "Timeout waiting for firmware file / Failed writing SOC firmware on stage 2"로
# 죽는다 (재부팅 2회 + 완전 파워사이클 1회로도 재현, hailortcli scan에 장치 자체가
# 안 잡힘). Hailo 커뮤니티의 동일 증상 스레드가 온보드 DDR 손상으로 결론났다.
# 새 보드가 들어오기 전까지 여기서 강제로 꺼서 scan_floor가 Hailo 경로를 안 탄다 —
# 아래 CPU YOLO 경로가 대신한다. 하드웨어 복구되면 이 한 줄만 지우면 된다.
_HAILO_AVAILABLE = False

try:
    from ultralytics import YOLO

    _CPU_YOLO_AVAILABLE = True
except ImportError:
    _CPU_YOLO_AVAILABLE = False


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
# 실측 4건(2026-08-21, n=1 각각) 기준 임시치 — "실측 확정"은 아니지만 최소한
# 관측값 안쪽에 두는 게 관측값 밖(15.0)보다 낫다:
#   빈 그리퍼 4.65 · 축구공(위치 이탈) 1.88 · 별 7.46 · 큐브 10.97
# 15.0은 네 값 전부보다 커서 confirmed가 상시 False였다(PR #185 리뷰 지적).
# TODO: 실측 — 케이스를 더 모아 재보정한다. confirm_grasp 로그(diff_score)를
# 실제 파지 성공/실패 케이스별로 모은다. ros2 param set으로 재배포 없이
# 튜닝할 수 있게 파라미터로도 노출한다.
CONFIRM_GRASP_DIFF_THRESHOLD_DEFAULT = 6.0

# ── scan_floor (2026-08-21 신설, 2026-08-23 갱신) ────────────────────────────
# ⚠️ 클래스별 거리 보정값(CLASS_DISTANCE_CALIBRATION_SQRT_PX_M, 아래 "RGB
# bbox 면적 기반 거리 추정" 참고)이 전부 미실측(None)인 동안은 모든 CPU YOLO
# 검출이 _approach_pose_m()에서 걸러져 scan_floor가 항상 빈 목록을 반환한다.
# Hailo 경로는 하드웨어 고장(#189)으로 애초에 안 쓴다.
#
# 클래스 매핑도 불완전하다. domain.values.ObjectClass는 GABE/CHESS_PIECE
# 둘뿐인데 Hailo 모델은 7종(container/knight/queen/rook/box/soccer/star)이다.
# knight/queen/rook은 CHESS_PIECE로, box/soccer/star는 GABE로 매핑했지만
# "cube"는 애초에 학습 클래스에 없고, container(Hailo 전용)는 목적지 상자로
# 보여서 **바닥 물체 후보에서 제외**했다 — 확실하지 않은 매핑을 코드에 박지
# 않는다(cpu_yolo_scan_mapping.py/hailo_scan_mapping.py 참고).
#
# 🔴 안전 게이트 (PR #185 리뷰 지적, 2026-08-21): 위 두 가지(거리 보정값
# 미실측, 클래스 매핑) 때문에 지금 이 게이트가 없어도 사실상 안전하지만,
# "우연히 안전한" 상태에 기대지 않는다 — 별도 파라미터로 기본값을 꺼둔다.
# 구조 검증(SCAN→SELECT→APPROACH 실기 경로 확인)이 필요할 때만 명시적으로
# `-p scan_floor_enabled:=true`로 켤 것.
SCAN_FLOOR_ENABLED_DEFAULT = False
HAILO_HEF_PATH_DEFAULT = "/tmp/best_640.hef"
# 2026-08-22: 0.35였다가 0.8로 올림 — 실기 검증 중 물체를 놓기 전인데도
# CPU YOLO가 0.35~0.48대 confidence로 오검출을 냈다(사용자 실측 지적).
# 오검출이 SELECT/APPROACH로 흘러 들어가는 게 더 위험하므로, 놓친 검출(재스캔하면
# 그만) 쪽보다 오검출(엉뚱한 좌표로 주행) 쪽을 훨씬 더 강하게 억제한다.
HAILO_SCORE_THRESHOLD = 0.8
# HAILO_CLASS_NAMES · 클래스 매핑은 hailo_scan_mapping.py 참고 (모듈 상단 import).

# CPU YOLO(ultralytics) 폴백 — 2026-08-22, Hailo-10H 하드웨어 고장(위 _HAILO_AVAILABLE
# 강제 비활성화 참고)으로 임시 대체. 클래스 구성이 Hailo 모델과 다르다(6종,
# "container" 없음) — cpu_yolo_scan_mapping.py 참고.
CPU_YOLO_MODEL_PATH_DEFAULT = "/tmp/best_cpu.pt"
# ⚠️ 2026-08-23: 단일 프레임 신뢰도 임계값을 0.8까지 올려 오검출을 억누르던
# 방식(2026-08-22 시도)을 폐기했다 — 대신 HANDOFF.md가 실기로 검증한 2단계
# 게이트를 쓴다: 프레임당 admission은 conf 0.45(floor_consensus.CONF_
# THRESHOLD)로 넉넉하게 열어두고, 여러 프레임에 걸친 다중 프레임 합의
# (k-of-n·순도·산포·거리 게이트, floor_consensus.py)로 오검출을 거른다.
# 물체 없는 장면의 오탐 4개(벽 밑동·바구니 주변)는 배경이 정지해 있어
# 합의 필터로 원리적으로 못 거르므로, 거리 게이트(y≥290)가 대신 막는다
# (HANDOFF.md "인식 > 한계" 참고).
CONSENSUS_N_FRAMES = 10  # tools/perception/approach.py --frames 기본값과 동일
CONSENSUS_COLLECT_TIMEOUT_SEC = 2.5  # SERVICE_TIMEOUT_SEC(3.0s, _ros_call.py)보다 짧게

# 진짜 3D 위치가 아니다 — 위 경고 참고. base_link 앞 임의 고정점.
# Hailo 경로는 아직 이 자리표시자를 쓴다 — 하드웨어가 고장(#189)이라 bbox 좌표계를
# 실기로 검증할 방법이 없다. 복구되면 아래 CPU YOLO와 같은 방식으로 바꿀 것.
FAKE_POSE_M = (0.3, 0.0, 0.0)
FAKE_DIMS_M = (0.05, 0.05, 0.05)

# ── 접근 자세(standoff/theta) 계산 (2026-08-22) ─────────────────────────────
# 사용자 지시: pose_m을 고정값 대신 실측 거리로 계산해서, 베이스가 도착했을
# 때 물체가 차체 앞 APPROACH_STANDOFF_M 지점에 오도록 전진 거리를 역산한다.
# 이어서 사용자가 "물체가 정면이 아니라 좌우로 벗어나 있으면?"이라고 물어서
# 좌우(y) 오프셋도 카메라 fx·cx로 같이 계산하게 넓혔다 — 픽셀 오프셋과 거리로
# 카메라 광학축 기준 좌우 각도를 구하는 표준 핀홀 역투영이라 메카넘(홀로노믹)
# 베이스가 곧장 옆으로 스트레이프해 정렬할 수 있다.
#
# ⚠️ 범위: 처음엔 "도착 위치(x, y)"만 풀고 방위각(theta)은 이슈 #171 팀 결정
#   전이라 0으로 미뤄뒀다. 그런데 사용자가 "파지를 위해 물체와 일직선상으로
#   마주보게 자세를 잡을 것"이라고 명시적으로 지시해서(2026-08-22), 도메인
#   코드 오너 본인의 지시로 이 자리에서 theta까지 함께 푼다 — #171을 팀
#   대신 여기서 결정하는 게 아니라, "파지하려면 물체를 정면으로 마주봐야
#   한다"는 이번 사용자 지시를 그대로 구현하는 것이다. 계산: 물체 원시 위치
#   (x_obj, y_obj)에서 베어링각 phi=atan2(y_obj, x_obj)만큼 회전해 마주보고,
#   그 방향으로 APPROACH_STANDOFF_M만큼 물러난 지점에 도착한다.
#     x_final = x_obj - STANDOFF*cos(phi), y_final = y_obj - STANDOFF*sin(phi),
#     theta_final = phi
#   이렇게 하면 도착 지점에서 물체까지 거리가 정확히 STANDOFF이고, 로봇이
#   phi만큼 돌아 있어 물체가 정면(차체 중심선상)에 온다.
#   - 카메라 장착 위치(차체 기준 오프셋)를 실측한 상수가 없어 차체 기준점과
#     같다고 근사한다. 카메라 광학축이 차체 정면 중심선과 나란하다고도
#     가정한다(둘 다 실측 전 근사 — 오차 요인).
#   - pose_m은 (state_machine.md의 "base_link 로부터 최단 거리" 관례와 일치하게)
#     **스캔 시점 base_link 기준 상대 좌표**다.
#   ⚠️ 2026-08-23: ApproachState는 더 이상 이 pose_m을 base.drive_to()에
#     넘기지 않는다(domain/task/states.py, domain/ports/base_driver.py의
#     `approach` 참고 — 실기 검증된 시각 서보 폐루프로 교체됐다). 이제 이
#     pose_m의 유일한 소비자는 SelectState의 최단 거리 정렬뿐이라, 위 odom
#     원점 정합 문제는 더 이상 치명적이지 않다 — 후보 우선순위가 조금
#     틀려도 시각 서보가 알아서 수렴한다.
APPROACH_STANDOFF_M = 0.18

# ── RGB bbox 면적 기반 거리 추정 (2026-08-23, depth 폐기 후 대체) ───────────
# depth 카메라(구조광 IR)는 체스말·축구공처럼 작고 광택 있는 물체를 거리와
# 무관하게 거의 못 본다(flying-pixel — 프레임 전체를 훑어도 물체의 진짜
# 거리값이 어디에도 없었다, 41cm/75cm 양쪽 실측 확인). 반대로 벽처럼 크고
# 평평한 면은 1% 이내로 맞았다 — 센서 자체는 정상이고 "작고 광택 있는 바닥
# 소품"이라는 이번 데모의 실제 대상에만 근본적으로 안 맞는다. baseline 역산·
# 패럴랙스 보정으로도 없는 값을 만들어낼 수는 없으므로 depth를 포기하고
# RGB만으로 거리를 추정한다.
#
# 원리: 같은 물체는 카메라에 가까울수록 bbox가 커진다 — 핀홀 모델에서
# 물체의 화면상 선형 치수는 거리에 반비례하므로 면적(=선형치수²)은 거리제곱에
# 반비례한다. 즉 distance_m = K_class / sqrt(bbox_area_px). K_class는 클래스별
# 실제 크기에 좌우되는 상수라 **클래스마다 따로 실측**해야 한다 — 이게 depth
# 방식과의 핵심 차이다: depth의 결손은 센서 한계라 고칠 방법이 없지만, 이
# 상수는 물체 하나 놓고 거리 한 번 재면 바로 채워진다.
CLASS_DISTANCE_CALIBRATION_SQRT_PX_M = {
    # 2026-08-23 실측(핫스팟 연결 실기, observe_target 서비스로 단일 프레임
    # h×w 직접 측정 — scan_floor의 consensus 게이트(MIN_BOTTOM_Y_PX=290)는
    # 이 거리대(0.66~1.13m)의 물체를 전부 걸러내 우회했다. 줄자 실측:
    # 축구공 0.66m, 나이트 0.84m, 룩 1.04m, 퀸 1.13m(전방 거리, base_link
    # 기준 — 좌우 오프셋은 z_m 보정과 무관해 무시). K = distance_m *
    # sqrt(bbox_area_px), bbox_area_px = h*w(observe_target 응답).
    "knight": 38.0307,
    "queen": 31.1632,
    "rook": 37.3992,
    "box": None,  # 미실측 — 60프레임 중 0회 검출(floor_consensus.py 경고 참고), 물체 자체를 아직 못 잡음
    "soccer": 20.6092,
    "star": None,  # 미실측 — RELIABLE_CLASSES에서도 제외된 상태(floor_consensus.py)
}
# 이보다 작은 bbox는 너무 멀거나 오검출일 가능성이 높아 거리 추정을 시도하지
# 않는다 — "모르면 제외"(hailo_scan_mapping.py와 같은 관례).
MIN_BBOX_AREA_PX = 25.0
# 2026-08-23 실기 확인(ros2 topic list) — 실제 네임스페이스는 "ascamera_hp60c"가
# 아니라 "ascamera"다(depth_cam_rotate_node.py의 같은 날짜 경고 참고).
RGB_CAMERA_INFO_TOPIC_DEFAULT = "/ascamera/camera_publisher/rgb0/camera_info"


def _standoff_arrival_pose(x_obj, y_obj):
    """물체 원시 위치(x_obj=전방 m, y_obj=좌측 m, base_link 기준)에서, 물체를
    정면으로 마주보고 APPROACH_STANDOFF_M만큼 물러난 최종 도착 자세
    (x, y, theta_rad)를 계산한다.

    사용자 지시(2026-08-22) — "파지를 위해 물체와 일직선상으로 마주보게
    자세를 잡을 것": 베어링각 phi=atan2(y_obj, x_obj)만큼 회전해 물체를
    정면에 두고, 그 방향으로 STANDOFF만큼 물러난 지점에 도착한다. 도착
    지점에서 물체까지 거리는 정확히 STANDOFF이고, theta=phi라 도착 시
    로봇 정면(차체 중심선)이 물체를 향한다.

    rclpy 없이 순수 계산이라 실기 스크립트(자로 잰 값을 직접 넣는 수동
    테스트 등)에서도 그대로 재사용한다."""
    phi = math.atan2(y_obj, x_obj)
    x_final = x_obj - APPROACH_STANDOFF_M * math.cos(phi)
    y_final = y_obj - APPROACH_STANDOFF_M * math.sin(phi)
    return x_final, y_final, phi


def _bgr_from_image_msg(msg):
    """Image 메시지를 BGR cv2 배열로 바꾼다 — cv_bridge를 쓰지 않는다.

    ⚠️ 2026-08-23 실기 확인: cv_bridge의 컴파일된 확장(cv_bridge_boost.so)이
    numpy 1.x ABI로 빌드돼 있어 이 환경의 numpy 2.x와 안 맞는다
    (`AttributeError: _ARRAY_API not found` → 세그폴트, ultralytics/torch가
    numpy 2.x를 끌어온 뒤 발생). tools/perception/floor_observer.py의
    to_bgr()과 같은 방식(원시 바이트 버퍼를 numpy로 직접 해석)으로 우회
    한다 — 같은 환경에서 이미 동작이 증명된 경로다."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("bgr8", "rgb8"):
        img = buf.reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1] if enc == "rgb8" else img
    if enc == "mono8":
        return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    raise ValueError(f"지원하지 않는 인코딩: {msg.encoding}")


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        cb_group = ReentrantCallbackGroup()

        self._latest_frame = None
        self._rgb_fx = self._rgb_cx = None
        if _CV_AVAILABLE:
            # depth_cam_rotate_node가 내보내는 회전 보정된 컬러 스트림.
            # (예전엔 "camera/color/image_raw"를 구독했는데, 실제로 이 이름으로
            # 퍼블리시하는 노드가 없어 _on_image가 한 번도 안 불렸다 — 2026-08-21 확인)
            self.create_subscription(
                Image,
                "depth_cam/rgb/image_rotated",
                self._on_image,
                10,
                callback_group=cb_group,
            )
            # ⚠️ 토픽명은 2026-08-23 실기로 확인됨(ros2 topic list). 다만 이
            # 토픽은 회전 보정 전(depth_cam_rotate_node 이전) intrinsics다 —
            # 180도 회전이면 cx/cy가 뒤집혀야 하는데(cx'=width-cx 등) 그
            # 보정은 아직 안 했다. 좌우(y) 계산 정확도에 영향을 준다. RGB
            # bbox 면적 자체(z_m 계산)는 이 값과 무관해 문제없다.
            self.create_subscription(
                CameraInfo,
                RGB_CAMERA_INFO_TOPIC_DEFAULT,
                self._on_rgb_camera_info,
                10,
                callback_group=cb_group,
            )
        else:
            self.get_logger().warn("sensor_msgs 미설치 — 카메라 구독 비활성화")

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
        self.create_service(
            ObserveTarget,
            "perception/observe_target",
            self._on_observe_target,
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

        self.declare_parameter("scan_floor_enabled", SCAN_FLOOR_ENABLED_DEFAULT)
        self.declare_parameter("hailo_hef_path", HAILO_HEF_PATH_DEFAULT)
        self.declare_parameter("cpu_yolo_model_path", CPU_YOLO_MODEL_PATH_DEFAULT)
        self._hailo_model = None
        self._hailo_output_shape = None
        self._hailo_input_size = None
        self._cpu_yolo_model = None
        if _HAILO_AVAILABLE:
            self._load_hailo_model()
        elif _CPU_YOLO_AVAILABLE:
            self._load_cpu_yolo_model()
        else:
            self.get_logger().warn(
                "hailo_platform·ultralytics 둘 다 미설치 — scan_floor 항상 빈 목록 반환"
            )

        if self._hailo_model is not None:
            backend_state = "Hailo"
        elif self._cpu_yolo_model is not None:
            backend_state = "CPU YOLO(ultralytics, Hailo 하드웨어 고장 임시 대체 — 이슈 #189)"
        else:
            backend_state = "백엔드 없음 → 항상 빈 목록"
        scan_floor_state = (
            f"{backend_state} (게이트 켜짐)"
            if self.get_parameter("scan_floor_enabled").value
            else f"{backend_state} · 게이트 꺼짐 → 빈 목록 반환"
        )
        self.get_logger().info(
            "perception_node ready "
            f"(scan_floor: {scan_floor_state}, "
            "find_box/measure_opening/monitor_clearance: NOT IMPLEMENTED)"
        )

    def _load_hailo_model(self):
        """VDevice/ConfiguredInferModel을 한 번만 만든다. 물리 Hailo-10H가
        1개뿐이라, tools/hailo/live_yolo_demo.py 같은 다른 프로세스가 이미
        VDevice를 쥐고 있으면 HAILO_OUT_OF_PHYSICAL_DEVICES로 실패한다 —
        둘 중 하나만 켜둘 것.

        ⚠️ vdevice/infer_model을 self에 안 붙이고 지역 변수로만 두면 이 함수가
        끝나는 순간 가비지 컬렉션되고, 나중에 self._hailo_model.run()을 부를 때
        "Lost communication with the server. This may happen if VDevice is
        released while the CIM is in use."로 죽는다 (2026-08-21 실기 확인 —
        MultiThreadedExecutor 탓으로 오진했다가, 단일 스레드로 바꿔도 똑같이
        죽는 걸 보고서야 진짜 원인을 찾았다). configure()가 반환하는
        ConfiguredInferModel이 부모를 안 붙잡아 주므로 노드 수명 내내
        직접 붙잡아야 한다."""
        hef_path = self.get_parameter("hailo_hef_path").value
        try:
            params = VDevice.create_params()
            params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
            self._hailo_vdevice = VDevice(params)
            self._hailo_infer_model = self._hailo_vdevice.create_infer_model(hef_path)
            self._hailo_infer_model.input().set_format_type(FormatType.UINT8)
            self._hailo_model = self._hailo_infer_model.configure()
            self._hailo_output_shape = self._hailo_infer_model.output().shape
            self._hailo_input_size = self._hailo_infer_model.input().shape[0]
            self.get_logger().info(
                f"scan_floor: Hailo-10H 모델 로드됨 {hef_path} (입력={self._hailo_input_size})"
            )
        except Exception as exc:  # noqa: BLE001 -- 장치 경합 등 다양한 원인을 전부 접는다
            self.get_logger().warn(f"scan_floor: Hailo 모델 로드 실패, 빈 목록으로 접음 ({exc})")
            self._hailo_model = None

    def _load_cpu_yolo_model(self):
        """ultralytics YOLO를 CPU로 로드한다. Hailo-10H 하드웨어 고장(이슈 #189)
        임시 대체 — 모듈 상단 CPU_YOLO_* 상수 경고 참고. 클래스 구성이 Hailo
        모델과 다르다(cpu_yolo_scan_mapping.py 참고)."""
        model_path = self.get_parameter("cpu_yolo_model_path").value
        try:
            self._cpu_yolo_model = YOLO(model_path)
            self.get_logger().info(f"scan_floor: CPU YOLO 모델 로드됨 {model_path}")
        except Exception as exc:  # noqa: BLE001 -- 파일 없음 등 다양한 원인을 전부 접는다
            self.get_logger().warn(f"scan_floor: CPU YOLO 모델 로드 실패, 빈 목록으로 접음 ({exc})")
            self._cpu_yolo_model = None

    def _on_image(self, msg):
        self._latest_frame = msg

    def _on_rgb_camera_info(self, msg):
        self._rgb_fx = msg.k[0]
        # 2026-08-23: 이 camera_info는 회전 보정 전(depth_cam_rotate_node
        # 이전) 원본 스트림의 것인데, bbox_xyxy는 180도 회전된 프레임
        # (depth_cam/rgb/image_rotated) 기준이다. 180도 회전에서 cx는
        # width - cx로 뒤집힌다(fx는 회전과 무관해 그대로 둔다). 실기
        # 확인된 해상도 640×480(ascamera_node 로그, hp60c 기본값) 기준.
        self._rgb_cx = msg.width - msg.k[2]

    def _approach_pose_m(self, class_name, bbox_xyxy):
        """검출 bbox(회전 보정된 RGB 프레임 기준 픽셀)로 최종 도착 자세
        (x, y, theta)를 구한다 — 단위 m/rad, 스캔 시점 base_link 기준.

        모듈 상단 "RGB bbox 면적 기반 거리 추정" 경고 참고 — depth 카메라는
        이번 데모 소품(작고 광택 있는 바닥 물체)을 근본적으로 못 봐서 폐기
        했다. 대신 bbox_area_px = 폭×높이에서 distance_m = K_class /
        sqrt(bbox_area_px)로 거리(z_m)를 구하고, 원래 RGB bbox 중심의 픽셀
        오프셋을 핀홀 역투영해 좌우(y_obj)를 구한다. 그 다음 물체 원시 위치
        (x_obj=z_m, y_obj)에서 베어링각 phi=atan2(y_obj, x_obj)만큼 회전해
        물체를 정면으로 마주보고, 그 방향으로 APPROACH_STANDOFF_M만큼 물러난
        지점을 반환한다(사용자 지시 — "파지를 위해 물체와 일직선상으로
        마주보게").

        다음 중 하나라도 있으면 **`None`** — 호출자가 이 검출을 후보에서
        제외해야 한다는 신호다:
        - bbox_area_px < MIN_BBOX_AREA_PX(너무 멀거나 오검출 가능성)
        - class_name의 K_class가 아직 실측 안 됨(CLASS_DISTANCE_CALIBRATION_
          SQRT_PX_M이 None) — 이 경우 bbox_area_px를 경고 로그에 남겨
          실측 자료로 쓸 수 있게 한다
        - RGB camera_info를 아직 못 받음(self._rgb_fx가 None)"""
        x1, y1, x2, y2 = bbox_xyxy
        bbox_area_px = (x2 - x1) * (y2 - y1)
        if bbox_area_px < MIN_BBOX_AREA_PX:
            return None

        k_class = CLASS_DISTANCE_CALIBRATION_SQRT_PX_M.get(class_name)
        if k_class is None:
            self.get_logger().warn(
                f"scan_floor: {class_name} 거리 보정값 미실측 — 후보에서 제외 "
                f"(실측용: bbox_area_px={bbox_area_px:.1f})"
            )
            return None

        if self._rgb_fx is None:
            return None

        z_m = k_class / math.sqrt(bbox_area_px)
        u = (x1 + x2) / 2.0
        y_obj = -(u - self._rgb_cx) * z_m / self._rgb_fx
        return _standoff_arrival_pose(z_m, y_obj)

    # ---- 서비스 콜백 ----
    def _on_scan_floor(self, request, response):
        # TODO: 상자 영역 마스킹 (state_machine.md §4 재진입 방지 방어선) — 실제
        # 위치 추정이 붙으면, 여기서 상자 ROI와 겹치는 detection을 걸러내야 한다.
        # 필터링을 빼먹으면 이미 처리된 상자 내부 물체를 계속 재검출해 무한 루프
        # 방지의 첫 번째 방어선(done_ids/held_ids 필터링)이 무력화된다.
        if not self.get_parameter("scan_floor_enabled").value:
            # 안전 게이트 — 모듈 상단 SCAN_FLOOR_ENABLED_DEFAULT 경고 참고.
            # pose_m이 자리표시자인 채로 SELECT/APPROACH가 실제 베이스를
            # 움직이는 걸 막는 기본값이다. 구조 검증 때만 명시적으로 켤 것.
            response.detections = DetectionArray(detections=[])
            return response

        if self._latest_frame is None:
            self.get_logger().warn("scan_floor: 프레임 없음 — 빈 목록 반환")
            response.detections = DetectionArray(detections=[])
            return response

        if self._hailo_model is not None:
            frame = _bgr_from_image_msg(self._latest_frame)
            detections = self._scan_floor_detections_hailo(frame)
        elif self._cpu_yolo_model is not None:
            # 단일 프레임이 아니라 여러 프레임을 새로 모은다 — 다중 프레임
            # 합의 필터(아래 _scan_floor_detections_cpu_yolo 참고)의 입력이
            # 필요해서, 여기서 미리 떠 둔 self._latest_frame 한 장으로는
            # 부족하다.
            detections = self._scan_floor_detections_cpu_yolo()
        else:
            self.get_logger().warn("scan_floor: 백엔드 미로드 — 빈 목록 반환")
            detections = []

        response.detections = DetectionArray(detections=detections)
        return response

    def _scan_floor_detections_hailo(self, frame):
        canvas = self._letterbox(frame, self._hailo_input_size)

        bindings = self._hailo_model.create_bindings()
        bindings.input().set_buffer(np.ascontiguousarray(canvas))
        bindings.output().set_buffer(np.empty(self._hailo_output_shape, dtype=np.float32))
        self._hailo_model.run([bindings], timeout=1000)
        detections_by_class = bindings.output().get_buffer()

        detections = []
        track_id = 0
        for class_id, dets in enumerate(detections_by_class):
            object_class = object_class_for_hailo_id(class_id)
            if object_class is None:
                continue  # 매핑 미확정 클래스(container/box 등) — 바닥 후보에서 제외
            class_name = HAILO_CLASS_NAMES[class_id]
            for det in dets:
                score = float(det[4])
                if score < HAILO_SCORE_THRESHOLD:
                    continue
                track_id += 1
                detections.append(self._make_detection(track_id, object_class, score))
                self.get_logger().info(
                    f"scan_floor(Hailo): {class_name}->{object_class} score={score:.2f} "
                    f"track_id={track_id}"
                )
        return detections

    def _yolo_raw_frame_detections(self, frame):
        """프레임 한 장의 ultralytics 추론 결과를 합의 필터 입력 형식
        `[(raw_cls, conf, bbox_xyxy), ...]`으로 바꾼다. domain ObjectClass
        매핑이나 거리 계산은 하지 않는다 — 합의로 확정된 뒤에만 한다
        (미확정 검출에 계산을 낭비하지 않는다, floor_consensus.py 참고).

        HANDOFF.md 동작점의 conf 0.45(CONF_THRESHOLD)로 admission을 넉넉하게
        열어둔다 — 오검출 억제는 여기가 아니라 다중 프레임 합의가 한다."""
        results = self._cpu_yolo_model.predict(frame, verbose=False)[0]
        raw_detections = []
        for box in results.boxes:
            score = float(box.conf[0])
            if score < CONF_THRESHOLD:
                continue
            class_name = results.names[int(box.cls[0])]
            bbox_xyxy = tuple(float(v) for v in box.xyxy[0])
            raw_detections.append((class_name, score, bbox_xyxy))
        return raw_detections

    def _collect_cpu_yolo_frames(self, n_frames, timeout_sec):
        """정지 상태를 전제로 서로 다른 n_frames개 프레임에 대해 YOLO 추론을
        돌려 raw 검출 리스트를 모은다(다중 프레임 합의 필터의 입력 형식,
        floor_consensus.confirmed_tracks 참고).

        카메라 콜백(_on_image)은 MultiThreadedExecutor의 다른 스레드에서
        계속 돈다 — 여기서 rclpy.spin_once()를 부르면 이 실행기 설정에서는
        불필요한 중첩 스핀이라 안 쓴다. 대신 짧게 자면서 self._latest_frame
        이 새 메시지 객체로 바뀌었는지 identity로 확인한다(같은 프레임을
        두 번 세지 않기 위함)."""
        frames = []
        last_msg = None
        deadline = time.monotonic() + timeout_sec
        while len(frames) < n_frames and time.monotonic() < deadline:
            current = self._latest_frame
            if current is None or current is last_msg:
                time.sleep(0.01)
                continue
            last_msg = current
            frame = _bgr_from_image_msg(current)
            frames.append(self._yolo_raw_frame_detections(frame))
        return frames

    def _scan_floor_detections_cpu_yolo(self):
        """ultralytics CPU 추론 + 다중 프레임 합의 필터(HANDOFF.md 2026-08-23
        검증: 산포 0.2~1.1px, 순도 1.00). 단일 프레임 검출을 그대로 쓰지
        않는다 — 정지 상태에서 CONSENSUS_N_FRAMES장을 새로 모아
        floor_consensus.confirmed_tracks()로 확정된 물체만 후보로 낸다."""
        frames = self._collect_cpu_yolo_frames(CONSENSUS_N_FRAMES, CONSENSUS_COLLECT_TIMEOUT_SEC)
        if len(frames) < CONSENSUS_N_FRAMES:
            self.get_logger().warn(
                f"scan_floor(CPU YOLO): {CONSENSUS_COLLECT_TIMEOUT_SEC}s 안에 "
                f"{len(frames)}/{CONSENSUS_N_FRAMES}프레임만 모임 — 있는 만큼으로 합의 시도"
            )
        if not frames:
            return []

        tracks = confirmed_tracks(frames, CONSENSUS_N_FRAMES)

        detections = []
        track_id = 0
        for t in tracks:
            object_class = object_class_for_cpu_yolo_class_name(t.label)
            if object_class is None:
                continue  # 매핑 미확정 클래스(box 등) — 바닥 후보에서 제외

            bbox_xyxy = track_bbox_xyxy(t)
            approach_pose = self._approach_pose_m(t.label, bbox_xyxy)
            if approach_pose is None:
                # 상세 사유(bbox 너무 작음/보정값 미실측/camera_info 없음)는
                # _approach_pose_m 내부에서 필요한 경우에만 따로 경고한다.
                continue
            x_final, y_final, theta_final = approach_pose

            track_id += 1
            score = max(t.confs)
            detections.append(
                self._make_detection(
                    track_id, object_class, score, pose_m=(x_final, y_final, 0.0), yaw_rad=theta_final
                )
            )
            self.get_logger().info(
                f"scan_floor(CPU YOLO, 합의 {len(t.frames)}/{CONSENSUS_N_FRAMES}프레임): "
                f"{t.label}->{object_class} score={score:.2f} purity={t.purity:.2f} "
                f"spread={t.spread:.1f}px x={x_final:.3f} y={y_final:.3f} "
                f"theta={theta_final:.3f} track_id={track_id}"
            )
        return detections

    @staticmethod
    def _make_detection(track_id, object_class, score, pose_m=None, yaw_rad=0.0):
        """Detection 메시지를 만든다.

        `pose_m`을 주면 그걸 쓴다(CPU YOLO — RGB bbox 면적 기반 접근 자세,
        모듈 상단 "RGB bbox 면적 기반 거리 추정" 경고 참고). 안 주면 자리표시자 FAKE_POSE_M을
        쓴다(Hailo — 하드웨어 고장(#189)으로 bbox 좌표계를 실기로 검증할 방법이
        없어 아직 자리표시자에 머문다). `yaw_rad`도 같은 이유로 CPU YOLO는
        _standoff_arrival_pose()가 계산한 값을, Hailo는 기본값 0.0을 쓴다."""
        x, y, z = pose_m if pose_m is not None else FAKE_POSE_M
        return Detection(
            track_id=track_id,
            cls=object_class,
            pose=Point(x=x, y=y, z=z),
            dims=Vector3(x=FAKE_DIMS_M[0], y=FAKE_DIMS_M[1], z=FAKE_DIMS_M[2]),
            yaw_rad=yaw_rad,
            confidence=score,
        )

    def _on_observe_target(self, request, response):
        """base_driver_node의 approach_object 액션(시각 서보 루프)이 매 반복마다
        부르는 저지연 관측. scan_floor(다중 프레임 합의)와는 목적이 다르다 —
        이건 SELECT가 이미 고른 특정 raw 클래스 하나를 반복 재관측하며
        수렴시키는 제어 루프 입력이라, 매 반복의 지연이 곧 루프 주기다.
        tools/perception/approach.py도 이동마다 다시 관측해 오차를 스스로
        지우는 폐루프라 여기서도 최신 프레임 1장이면 충분하다 — 노이즈는
        다음 반복이 알아서 고친다.

        CPU YOLO 백엔드 전용(Hailo는 하드웨어 고장 #189로 다루지 않는다).
        모델 미로드·프레임 없음·해당 클래스 미검출은 전부 found=False —
        "모르면 실패" 관례. 여러 후보가 있으면 가장 큰(=가까운) 것을 고른다
        (tools/perception/approach.py의 pick()과 동일)."""
        response.found = False
        response.x = 0.0
        response.h = 0.0
        response.w = 0.0
        if self._cpu_yolo_model is None or self._latest_frame is None:
            return response

        frame = _bgr_from_image_msg(self._latest_frame)
        raw_detections = self._yolo_raw_frame_detections(frame)
        candidates = [d for d in raw_detections if d[0] == request.raw_cls]
        if not candidates:
            return response

        _, _, bbox = max(candidates, key=lambda d: d[2][3] - d[2][1])  # 가장 큰 높이
        x1, y1, x2, y2 = bbox
        response.found = True
        response.x = (x1 + x2) / 2.0
        response.h = y2 - y1
        response.w = x2 - x1
        return response

    def _on_find_box(self, request, response):
        # request.color는 와이어 필드명이 아직 레거시라 그렇다 — 2026-08-23
        # 확정 미션 명세서로 domain.values.BoxColor가 Destination(LEFT/RIGHT)
        # 으로 바뀌었고 지금 이 필드엔 그 이름이 들어온다(domain/adapters/
        # real/_ros_convert.py 상단 경고 참고).
        self.get_logger().warn(
            f"find_box(dest={request.color}): 비전 파이프라인 미구현 — found=False 반환"
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

    @staticmethod
    def _letterbox(frame, size):
        """정사각형 size x size로 비율 유지 레터박스한다 (tools/hailo/
        live_yolo_demo.py의 letterbox()와 동일 로직)."""
        h, w = frame.shape[:2]
        scale = min(size / h, size / w)
        resized = cv2.resize(frame, (round(w * scale), round(h * scale)))
        canvas = np.zeros((size, size, 3), dtype=np.uint8)
        y0 = (size - resized.shape[0]) // 2
        x0 = (size - resized.shape[1]) // 2
        canvas[y0 : y0 + resized.shape[0], x0 : x0 + resized.shape[1]] = resized
        return canvas

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
    # 2026-08-21 실기 디버깅 메모: scan_floor의 Hailo 추론이 "Lost communication
    # with the server..."로 죽어서 한때 이 MultiThreadedExecutor가 원인인 줄
    # 알았다(서비스 콜백이 __init__과 다른 워커 스레드에서 돈다고 추정) —
    # 단일 스레드(rclpy.spin)로 바꿔도 똑같이 죽는 걸 보고 오진이었다는 걸
    # 확인했다. 진짜 원인은 _load_hailo_model()이 vdevice/infer_model을
    # self에 안 붙이고 지역 변수로 둬서 가비지 컬렉션된 것이었다 (그 함수의
    # docstring 참고). 그래서 원래대로 되돌린다 — monitor_clearance 같은
    # 안전 판정이 다른 서비스 처리에 밀리지 않게 유지한다.
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
