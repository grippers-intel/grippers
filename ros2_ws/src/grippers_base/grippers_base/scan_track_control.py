"""SCAN 대상 물체로 제자리회전+직진 추적, 원위치 복귀, 시각 기반 장애물
회피 — 순수 제어 수학. rclpy 없이 pytest로 검증한다(visual_approach_control.py와
같은 이유 — 카메라 구독·cmd_vel 발행 배선은 여기 없다).

2026-08-23 재설계(visual_approach_control.py)는 회전+전진을 **결합**해서
냈다(예: linear_x=0.05, angular_z=0.1 동시에). 그 버전 실기에서 로봇이
좌측으로 90도 돌아 목표를 이탈했고, 이후 제자리 회전만 단독으로도 실기
테스트했으나(0.3~0.6 rad/s) 모터가 소리만 내고 전혀 안 돌았다.

2026-08-24: 코드 조사(원인 규명) 결과, 어제 실패는 하드웨어 한계가 아니라
**세 가지가 겹친 커맨드 방식 문제**였다:

1. **토픽이 틀렸다.** `odom_publisher_node.py`는 두 토픽을 구독한다 —
   `controller/cmd_vel`(원본, 제한 없음)과 `cmd_vel`(안전 클램프:
   `angular.z`를 **±0.5 rad/s로 강제로 자른다**). 어제 이 프로젝트의
   모든 도구(`base_driver_node.py` 포함)는 클램프가 걸리는 `cmd_vel`에
   발행했다 — 벤더 자체 텔레옵/조이스틱/lidar 회피 코드는 전부
   **`controller/cmd_vel`**에 발행한다(조이스틱 max_angular=3.0,
   lidar_controller 1.2 등). **제자리 회전은 반드시 `controller/cmd_vel`에
   발행해야 클램프 없이 나간다.**
2. **속도가 정지마찰 문턱보다 낮았다(고 추정했으나 아니었다).** 당시엔
   직진 데드밴드(0.05 m/s)를 회전팔 길이(0.1407m)로 환산한 **0.355 rad/s**를
   문턱으로 보고, 어제 쓴 0.3·0.6이 그 위아래에 걸쳐 있어 실패했다고
   해석했다.

   ✅ 2026-08-24 실측으로 이 해석은 **틀렸다**: 1.2부터 0.3까지 일곱 개
   값을 제자리 회전시켜 **전부 돌았다**(tools/inplace_rotation_test.py,
   `controller/cmd_vel`, 2초 버스트, 사람 판정). 문턱은 0.3보다 낮다.
   따라서 어제의 실패 원인은 속도가 아니라 1번(토픽)이거나 발행 방식
   자체였을 가능성이 높다 — 이 도구는 버스트 동안 20Hz로 계속 발행한다.
3. **`/odom_raw`는 회전 여부를 검증 못 한다.** `cal_odom_fun`이 적분하는
   값은 엔코더가 아니라 **명령으로 받은 linear_x/angular_z 그 자체**다
   — 바퀴가 완전히 멈춰 있어도 완벽한 회전을 했다고 보고한다. 즉
   `compute_return_vector`가 쓰는 (x,y,yaw)는 **그 전까지의 모든 이동
   명령이 실제로 바퀴를 움직였을 때만** 유효하다 — 데드밴드 아래로
   내려가는 명령을 하나라도 섞으면 원위치 복귀 벡터 전체가 틀어진다.

이 파일은 이 세 가지를 전제로 다시 짰다: **정렬(제자리 회전만)과
전진(직진만)을 완전히 분리된 단계로** 낸다(동시에 내지 않는다) —
`compute_align_command`가 `linear_x=0`을 강제하고 `compute_drive_command`가
`angular_z=0`을 강제하는 이유가 이것이다. `DEFAULT_ALIGN_TURN_RAD_S`는
실측 후 사용자가 정한 정렬용 값(0.35 rad/s)이다 — 그래도
**호출부(실행 노드)는 반드시 `controller/cmd_vel`에 발행해야 한다**,
`cmd_vel`에 발행하면 이 값도 결국 클램프 후보에 걸린다(0.5는 안 잘리지만
그 이상 값을 나중에 올리려면 반드시 필요).
"""
from __future__ import annotations

import math
from typing import NamedTuple

# perception_node.py CLASS_DISTANCE_CALIBRATION_SQRT_PX_M과 반드시 값이
# 같아야 한다 — 여기 따로 둔 이유는 visual_approach_control.py와 같다
# (카메라 구독 없이 순수 함수로 pytest에서 검증하기 위해서다).
K_CLASS = {
    "knight": 35.9307,
    "queen": 28.3382,
    "rook": 34.8340,
    "box": None,
    "soccer": 18.9592,
    "star": None,
}
# 2026-08-24: 거리 모델이 z = K/sqrt(hw)에서 z = K/(sqrt(hw) - BBOX_PADDING_PX)로
# 바뀌었다(근거는 perception_node.py의 같은 날짜 주석). K_CLASS도 그에 맞춰 함께
# 갱신했으므로 둘은 반드시 같이 움직여야 한다 — 한쪽만 베끼면 조용히 어긋난다.
BBOX_PADDING_PX = 2.5

# ⚠️ 2026-08-24 수정: 여기 있던 FX_PX=602.7175 / CX_PX=351.3056은
# yolov5_ros2/cv_tool.py에서 가져온 값인데, 그건 **이 로봇에 달린 카메라가
# 아니다**. 실제 /ascamera/camera_publisher/rgb0/camera_info는 fx=588.9755,
# cx=325.3051이다. 잘못된 값으로 40cm 실측 시 좌우 오차가 5.56cm였고, 실제
# 내참수로 바꾸자 3.07cm로 줄었다.
#
# depth_cam_rotate_node가 영상을 180° 회전시키므로 주점의 가로 좌표도 뒤집힌다:
# cx' = width - cx. 회전된 영상 위의 좌표(obs_x)를 쓰는 이 파일은 반드시
# 회전된 주점을 써야 한다.
FX_PX = 588.9754638671875
CX_PX = 640 - 325.3050842285156  # = 314.6949 (180° 회전 반영)

# 내참수를 고친 뒤에도 남은 계통 편향 — 카메라가 차체 중심선에서 옆으로
# 어긋나게 **장착**된 몫이다. 2026-08-24에 룩을 40cm·70cm 정중앙에 놓고
# 두 모델을 비교해 확정했다: 거리와 무관한 상수 미터 편향(장착 위치 오프셋)
# 쪽이 잔차 0.80cm로, 거리에 비례하는 픽셀 편향(광축 yaw 틀어짐) 쪽의
# 1.47cm보다 잘 맞았다. 그래서 픽셀이 아니라 미터로 더한다.
LATERAL_BIAS_M = 0.0291


class ScanTrackCommand(NamedTuple):
    linear_x: float
    linear_y: float
    angular_z: float
    arrived: bool


# --- 거리 신호: 기본(h) + 신뢰도 저하 시 폴백(화면 면적) ------------------


def establish_target_h(obs_h: float, obs_w: float, k_class: float | None, target_distance_m: float) -> float | None:
    """SCAN 직후 첫 유효 관측 1회로 이번 실행의 target_h(px)를 역산한다.

    h(박스 세로 픽셀)는 선형 치수라 거리에 반비례한다 — 즉 h*z_m은 물체
    고유의 상수다. 이 상수를 클래스마다 따로 실측해두지 않아도(K_CLASS는
    면적 기반이라 다른 상수다), **지금 이 순간의 관측 h·w로 z_m을 구하고,
    그 비례관계를 그대로 목표 거리로 스케일**하면 된다:

        target_h = obs_h * (현재_z_m / target_distance_m)

    k_class가 None(box·star, 아직 거리 보정 미실측)이면 z_m 자체를 못
    구하므로 None을 돌려준다 — 호출자는 이 경우 h 기반 추적을 포기하고
    화면 면적만으로 대략적인 접근 여부를 판단해야 한다(둘 다 결국 같은
    K_CLASS가 필요하므로, box·star는 근본적으로 아직 정밀 추적 대상이
    아니다 — 이 함수의 반환값으로 그 사실이 드러난다)."""
    if k_class is None or obs_h <= 0 or obs_w <= 0 or target_distance_m <= 0:
        return None
    current_z_m = bbox_area_distance_m(obs_h, obs_w, k_class)
    if current_z_m is None:
        return None
    return obs_h * (current_z_m / target_distance_m)


def z_from_established_h(obs_h: float, target_h: float, target_distance_m: float) -> float | None:
    """establish_target_h()의 역함수 — 이번 프레임의 h 하나만으로 현재
    거리(m)를 구한다(기본/우선 방식). h*z=상수(=target_h*target_distance_m,
    establish_target_h가 SCAN 시점에 고정한 값)라는 물리적 관계를 그대로
    쓴다 — w는 필요 없다."""
    if obs_h <= 0 or target_h <= 0:
        return None
    return target_h * target_distance_m / obs_h


def bbox_area_distance_m(obs_h: float, obs_w: float, k_class: float | None) -> float | None:
    """화면 면적 기반 거리(m) — h 신호가 못 미더울 때의 폴백."""
    if k_class is None or obs_h <= 0 or obs_w <= 0:
        return None
    # z = K / (sqrt(h*w) - BBOX_PADDING_PX) — 검출 bbox가 실루엣보다 항상
    # 일정 픽셀 크게 잡히는 몫을 빼준다(perception_node.py의 같은 날짜 주석).
    effective_px = math.sqrt(obs_h * obs_w) - BBOX_PADDING_PX
    if effective_px <= 0.0:
        return None
    return k_class / effective_px


def h_signal_reliable(obs_h: float, obs_w: float, ref_aspect_ratio: float, max_relative_deviation: float = 0.4) -> bool:
    """h 기반 거리 오차를 믿어도 되는지 판단한다.

    2026-08-23 세션(`rook_grasp_session_analysis.md`)에서 오검출·병합된
    박스가 비정상 종횡비(예: 세워진 룩인데 w>h)로 나타난 걸 실측으로
    확인했다 — 종횡비가 SCAN 시점 기준값에서 너무 벗어나면 그 프레임의
    h는 못 믿는다는 신호로 쓴다. ⚠️ 40% 임계값은 실기 미검증 자리표시자."""
    if obs_h <= 0 or ref_aspect_ratio <= 0:
        return False
    aspect = obs_w / obs_h
    deviation = abs(aspect - ref_aspect_ratio) / ref_aspect_ratio
    return deviation <= max_relative_deviation


# --- 정렬(회전만) / 전진(직진만) — 완전히 분리된 단계 -----------------------

DEFAULT_TOL_X_PX = 15.0
DEFAULT_TOL_DIST_M = 0.03

# ✅ 2026-08-24 실기 실측(tools/inplace_rotation_test.py, controller/cmd_vel에
# 2초 버스트, 사람이 눈으로 y/n 판정). 1.2 / 0.8 / 0.6 / 0.5 / 0.4 / 0.355 /
# 0.3을 시험해 **전부 돌았다** — 즉 정지마찰 문턱은 0.3보다도 낮다.
#
# 이로써 그 전까지의 추정 두 가지가 다 틀렸음이 확인됐다:
#   - "문턱 ≈ 0.355 rad/s"는 직진 데드밴드(0.05m/s)를 회전팔 0.1407m로
#     환산한 계산값이었는데, 실측은 그 아래에서도 돈다.
#   - 그래서 "안정적으로 돌리려면 1.0~1.2가 필요하다"던 기존 값도 근거를
#     잃었다. 벤더가 라이다 회피에 1.2를 쓰는 건 **빨리 피하려는** 것이지
#     그만큼 필요해서가 아니다.
#
# 사용자 판단(2026-08-24, 실기를 보면서): 정렬용으로는 0.3~0.4가 적당하다.
# 정렬은 목표를 화면 중앙에 맞추는 미세 동작이라 빠를수록 오버슈트가 커진다
# — 돌 수 있는 가장 느린 쪽을 쓰는 게 맞다.
#
# 부수 효과 하나: 이 값들은 전부 0.5 미만이라 `cmd_vel`의 ±0.5 클램프에
# 걸리지 않는다. 즉 정렬 단계에 한해서는 어느 토픽에 쏘든 결과가 같다.
# 그래도 `controller/cmd_vel` 권장은 유지한다 — 나중에 이 값을 올릴 때
# 조용히 잘리는 걸 막기 위해서다(모듈 docstring 참고).
DEFAULT_ALIGN_TURN_RAD_S = 0.35
# 이 아래 값은 정지마찰을 못 이긴다고 보고 아예 안 낸다(0으로 반올림하지
# 않고 이 값까지 끌어올린다) — apply_align_turn_floor 참고.
# 0.3은 실측으로 도는 것을 확인한 값이다(위 참고).
MIN_ALIGN_TURN_RAD_S = 0.3

DEFAULT_DRIVE_SPEED_MPS = 0.06


def apply_align_turn_floor(wz: float, min_turn: float = MIN_ALIGN_TURN_RAD_S, max_turn: float = DEFAULT_ALIGN_TURN_RAD_S) -> float:
    """0이 아닌 회전 지령이 정지마찰 문턱보다 작으면 문턱까지 끌어올리고,
    상한을 넘으면 자른다 — visual_approach_control.apply_axis_floor와 같은
    패턴. 이 파일의 정렬 단계는 항상 상수 속도(DEFAULT_ALIGN_TURN_RAD_S)로
    회전하므로 지금은 자동으로 이 범위 안에 있지만, 게인 기반 비례 회전으로
    바꿀 경우를 대비해 별도 함수로 남겨둔다."""
    if wz == 0.0:
        return 0.0
    magnitude = min(max_turn, max(min_turn, abs(wz)))
    return magnitude if wz > 0 else -magnitude


def compute_align_command(
    err_x: float,
    *,
    tol_x: float = DEFAULT_TOL_X_PX,
    turn_speed: float = DEFAULT_ALIGN_TURN_RAD_S,
) -> ScanTrackCommand:
    """제자리 회전 전용(linear_x=linear_y=0 고정) — 전진과 절대 안 섞는다.

    err_x 부호 규약은 visual_approach_control.compute_approach_command와
    동일: +면 물체가 화면 오른쪽 → 오른쪽으로 돌아야 함 → REP103 관례상
    angular.z<0.

    ⚠️ 이 명령은 반드시 `controller/cmd_vel`(클램프 없음)에 발행할 것 —
    `cmd_vel`은 angular.z를 ±0.5 rad/s로 자른다(모듈 docstring 참고)."""
    if abs(err_x) <= tol_x:
        return ScanTrackCommand(0.0, 0.0, 0.0, True)
    wz = -turn_speed if err_x > 0 else turn_speed
    return ScanTrackCommand(0.0, 0.0, wz, False)


def compute_drive_command(
    err_dist_m: float,
    *,
    tol_dist_m: float = DEFAULT_TOL_DIST_M,
    speed: float = DEFAULT_DRIVE_SPEED_MPS,
) -> ScanTrackCommand:
    """직진 전용(angular_z=linear_y=0 고정) — 정렬과 절대 안 섞는다.
    err_dist_m: 현재거리−목표거리. +면 아직 멀다(전진해야 함)."""
    if abs(err_dist_m) <= tol_dist_m:
        return ScanTrackCommand(0.0, 0.0, 0.0, True)
    vx = speed if err_dist_m > 0 else -speed
    return ScanTrackCommand(vx, 0.0, 0.0, False)


# --- 원위치 복귀 — 경로를 그대로 재생하지 않고 직접 벡터로 되돌아간다 ------


def normalize_angle_rad(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """odom_raw의 orientation 쿼터니언에서 yaw(rad)만 뽑는다(2D 평면
    로봇이라 roll·pitch는 0으로 본다)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def compute_return_vector(
    start_x_m: float,
    start_y_m: float,
    current_x_m: float,
    current_y_m: float,
    current_yaw_rad: float,
) -> tuple[float, float]:
    """원위치로 돌아가려면 (지금 heading 기준 얼마나 돌아야 하는지 rad,
    그 다음 직진해야 할 거리 m).

    ⚠️ 설계 결정 — 지나온 경로를 그대로 재생(각 회전·직진 구간을 순서대로
    역재생)하지 않고, odom_raw가 주는 절대 (x,y)로 **시작점까지 직선
    벡터 하나**를 계산해 "제자리 회전 1회 + 직진 1회"로 돌아간다(사용자가
    제시한 두 안 중 후자 — "제자리에서 360도 회전하여 경로를 계산해
    이동"). 경로를 그대로 되짚으면 구간 수만큼 오차가 누적되는데, 직선
    벡터 방식은 재생 단계가 단 두 번이라 누적 오차가 훨씬 적다. 장애물
    회피(find_path_obstacle/compute_dodge_command)는 복귀 중에도 그대로
    켜 두면 이 직선 경로 위의 장애물에도 대응한다.

    ⚠️ 2026-08-24 코드 조사로 확인: `/odom_raw`(`odom_publisher_node.py`
    `cal_odom_fun`)는 엔코더가 아니라 **명령으로 보낸 linear_x/angular_z
    자체를 적분**한다 — 바퀴가 실제로 안 움직여도(정지마찰로 멈춰
    있어도) 명령대로 이동했다고 보고한다. 즉 이 함수가 받는 (current_x,
    current_y, current_yaw)는 **그 전까지의 모든 회전·전진 명령이 전부
    실제로 바퀴를 움직였을 때만** 유효한 값이다. 정지마찰 문턱
    (MIN_ALIGN_TURN_RAD_S) 아래로 내려가는 명령을 하나라도 실행부가
    허용하면, 그 프레임의 "이동"은 실제로는 없었는데 odom엔 반영돼
    복귀 벡터 전체가 틀어진다."""
    dx = start_x_m - current_x_m
    dy = start_y_m - current_y_m
    distance_m = math.hypot(dx, dy)
    target_bearing_rad = math.atan2(dy, dx)
    heading_error_rad = normalize_angle_rad(target_bearing_rad - current_yaw_rad)
    return heading_error_rad, distance_m


# --- 시각 기반 장애물 회피 — LiDAR 대신 YOLO로 "다른 물체"를 본다 ----------
#
# 2026-08-23 실기 확인: LD19 라이다는 장착 높이(9.25cm)에서 체스말 크기
# 물체를 아예 못 본다 — 그래서 LiDAR 기반 회피(visual_approach_control.
# obstacle_ahead 등)는 이 프로젝트의 바닥 물체 회피에는 쓸모가 없다. 대신
# 이미 있는 YOLO 검출(observe_target을 목표 클래스가 아닌 나머지 클래스에
# 대해서도 호출)로 "장애물"을 정의한다.


class ObstacleObservation(NamedTuple):
    cls: str
    forward_m: float
    lateral_m: float  # +면 우측(REP103 y축과는 부호가 반대이니 주의)


def lateral_offset_m(
    obs_x: float,
    z_m: float,
    fx_px: float = FX_PX,
    cx_px: float = CX_PX,
    bias_m: float = LATERAL_BIAS_M,
) -> float:
    """핀홀 근사(왜곡 무시) — grasp_test_console.py estimate_position()과 동일.

    bias_m은 카메라 장착 위치 오프셋 보정이다(LATERAL_BIAS_M 주석 참고)."""
    return (obs_x - cx_px) * z_m / fx_px + bias_m


def find_path_obstacle(
    observations: list[ObstacleObservation],
    *,
    path_half_width_m: float = 0.15,
    max_range_m: float | None = None,
) -> ObstacleObservation | None:
    """목표 클래스가 아닌 관측 목록에서 "경로 위" 장애물을 찾는다.
    조건: 좌우로 path_half_width_m 안, 전방으로 max_range_m(목표까지 남은
    거리)보다 가깝다. 여러 개면 가장 가까운 것 하나만 돌려준다 — 한 번에
    한쪽으로만 피하는 게 목적이라(사용자 지시: "좌/우 둘 중 하나만
    선택") 여러 장애물을 동시에 고려할 필요가 없다."""
    candidates = [
        o
        for o in observations
        if o.forward_m > 0.0
        and abs(o.lateral_m) <= path_half_width_m
        and (max_range_m is None or o.forward_m < max_range_m)
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda o: o.forward_m)


def choose_dodge_side(obstacle_lateral_m: float) -> float:
    """장애물의 반대쪽으로 피한다. +1.0=왼쪽(양의 linear.y), -1.0=오른쪽.
    obstacle_lateral_m은 +면 장애물이 우측(ObstacleObservation 규약) —
    우측 장애물은 좌측(+1.0)으로, 좌측 장애물은 우측(-1.0)으로 피한다.
    정확히 중앙(0)이면 오른쪽을 기본값으로 쓴다(visual_approach_control.
    choose_dodge_side와 같은 관례 — 방향을 안 정하면 회피가 안 된다)."""
    if obstacle_lateral_m > 0.0:
        return 1.0
    if obstacle_lateral_m < 0.0:
        return -1.0
    return -1.0


DEFAULT_DODGE_SPEED_MPS = 0.05


def compute_dodge_command(dodge_side: float, dodge_speed: float = DEFAULT_DODGE_SPEED_MPS) -> ScanTrackCommand:
    """linear_y만 낸다 — 전진·회전은 멈추고 옆으로만 비킨다. 회피 중
    동시에 전진/회전하면 장애물에 더 가까워질 수 있어서다(visual_approach_
    control.compute_dodge_command와 같은 이유)."""
    return ScanTrackCommand(0.0, dodge_speed * dodge_side, 0.0, False)
