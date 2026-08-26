"""라이다로 바구니 정면을 잡아 INSERT 전환 최종 판단을 낸다.

Host PC가 바구니 앞까지 경로를 주지만, 오버헤드 관측과 주행 오차가 남는다.
INSERT는 팔을 전개해 물체를 떨어뜨리는 동작이라 "바구니 정면에 똑바로,
알맞은 거리에 서 있는가"가 맞아야 한다. 그 최종 확인을 차량 자기 라이다로
한다(사용자 지시, 2026-08-25).

## "입구 검출"이 아니다

2D 라이다는 수평 한 평면만 본다. 바구니 입구는 위를 향한 구멍이라 그 평면에
잡히지 않는다 — 평면이 바구니를 지나면 **옆벽 표면**만 돌아온다.

그런데 INSERT 전환에 실제로 필요한 것은 입구의 형상이 아니라 정면 정렬과
거리다. 옆벽을 선분으로 피팅하면 그 둘이 동시에 나온다 — `align_to_box()`가
돌려주기로 한 yaw 오차가 바로 이 값이다.

## 높이·기울기 전제 (이 모듈이 성립하는 조건)

2026-08-26 실측으로 두 값이 바뀌었다. 라이다는 바닥 위 **약 140mm**에
있고(그전에 적어 둔 91mm는 틀렸다), 수평이 아니라 정면 아래로 **약 11.3도**
기울어져 있다.

기울기가 생기는 결과는 두 가지다:

1. **거리가 길게 읽힌다.** 수직 벽면까지의 수평거리 d를 기울어진 빔은
   `d / cos(11.3도)` = d의 1.020배로 읽는다. 그래서 읽은 값에 `cos(기울기)`를
   곱해야 수평거리가 된다 — `scan_to_front_points()`가 이 보정을 한다.
   피팅 자체는 보정 전에도 성립한다(수직 평면은 기울어진 평면으로 잘라도
   여전히 직선이다). 보정은 거리 스케일만 바로잡는다.
2. **빔이 앞으로 갈수록 낮아진다.** 라이다 기준 x mm 앞에서 빔 높이는
   `z(x) = 140 - 0.1998 * x` (mm)다. 바구니 테두리(실측 115mm)를 넘어가려면
   빔이 그보다 낮아야 하므로 x >= 125mm가 필요하다. 반대로 바닥에 닿는
   지점은 x = 701mm라, 그보다 먼 정면 반사는 바닥이다.

   그래서 **너무 가까이 서면 빔이 테두리 위를 스쳐 지나가 바구니를 놓친다.**
   2026-08-26 실기에서 라이다 판독 약 139mm 자리가 여유 2.7mm로 아슬아슬하게
   성립했다. 5cm 대까지 붙이는 것은 성립하지 않는다.

체스말 같은 바닥 물체는 여전히 이 평면에 안 잡힌다 — 라이다를 바닥 장애물
회피에 못 쓴다는 기존 제약은 그대로다. 라이다가 볼 수 있는 것은 벽과
바구니, 그리고 70cm 너머의 바닥뿐이고, 이 모듈은 그중 바구니만 본다.

## 방위각 원점

라이다의 0도는 차량 정면이 **아니다**. 2026-08-26에 차체 정면 30cm에 판을
대고 확인한 결과 **정면 = 라이다 +90도**다. 원시 스캔 각도를 그대로 쓰면
90도 엉뚱한 곳을 본다 — `scan_to_front_points()`를 거쳐야 한다.

## 벽과의 분리

바구니는 뒷벽에 붙어 있지만 작업 영역 쪽으로 `BOX_L/2` = 175mm 튀어나와
있다. 스캔에서 벽과 17.5cm 계단으로 갈리므로, 기대 방위각 창 안에서 가장
가까운 덩어리만 취하면 벽이 섞이지 않는다.

rclpy·센서 없이 순수 계산만 담아 pytest로 검증한다
(grippers_arena/aruco_localization.py와 같은 이유). numpy도 쓰지 않는다."""

import math
from typing import NamedTuple

# 사용자 실측, 2026-08-26. URDF(mecanum.xacro)의 base_footprint->base_link
# 0.07 + base_link->lidar_frame 0.0925 = 0.1625와 다르다 — URDF는 벤더 제공
# 값이라 실물과 어긋난다. 실측을 따른다.
#
# 2026-08-25에 적어 둔 0.091은 틀린 값이었다(같은 날 다시 재서 0.14로 정정).
LIDAR_HEIGHT_M = 0.140

# 라이다가 수평이 아니라 정면 아래로 기울어진 각도(사용자 실측, 2026-08-26).
# 위 docstring "높이·기울기 전제" 참고.
LIDAR_TILT_DEG = 11.3

# 차량 정면에 해당하는 라이다 방위각. 2026-08-26 판 실험으로 확정했다.
FRONT_OFFSET_DEG = 90.0

# 바구니 테두리 높이(2026-08-20 실측, floor_grasp_profiles.py와 같은 값).
BASKET_RIM_HEIGHT_M = 0.115

# 바구니 정면의 실제 폭. 2026-08-26 접근 실기에서 라이다로 직접 쟀다 —
# 방위각 창이 자르기 전 구간(0.18~0.53m)의 겉보기 폭이 233~251mm로
# 일관됐다. Host aruco/config.py의 BOX_W(210mm)와 32mm 다르다.
BASKET_FACE_WIDTH_M = 0.242

# 벽과 바구니를 가르는 깊이. 바구니가 벽에서 175mm 나와 있으므로 그보다
# 넉넉히 작게 잡아야 벽이 같은 덩어리로 묶이지 않는다.
DEFAULT_CLUSTER_DEPTH_M = 0.08

# 기대 방위각 기준 좌우로 이만큼만 본다. 두 바구니가 90cm 떨어져 있어
# 이 창이면 옆 바구니가 안 들어온다.
DEFAULT_BEARING_WINDOW_RAD = math.radians(35.0)

DEFAULT_MIN_POINTS = 5
# 평면이 평평한 판을 때리면 잔차는 라이다 거리 잡음 수준이어야 한다.
#
# 값을 고른 근거: 다리 100mm짜리 직각 모서리가 잔차 14.9mm를 낸다(모서리를
# 비스듬히 보거나 벽과 바구니가 한 덩어리로 묶인 경우가 이 모양이다). 그걸
# 걸러내려면 15mm로는 모자라 10mm로 조인다. 실장된 LDRobot LD19의 거리
# 정확도가 0.3m에서 수 mm 수준이므로 정상적인 평면은 여유 있게 통과한다
# (2026-08-26 실기: 12cm 바구니 정면 잔차 2.8mm).
#
# 조이는 쪽이 안전한 이유: 여기서 실패하면 INSERT로 안 넘어가고 다시 볼
# 뿐이라 비용이 재시도 한 번이다. 반대로 느슨해서 모서리를 평면으로 받으면
# 엉뚱한 자세에서 팔을 전개한다.
#
# ⚠️ 실기 조정 대상 — 실제 라이다 잡음이 10mm RMS를 넘으면 정상 평면도
# 계속 실패한다. 그때는 이 값을 올리되, 위 모서리 수치(14.9mm)보다는
# 아래에 두어야 검사가 의미를 유지한다.
DEFAULT_MAX_RESIDUAL_M = 0.010
# 겉보기 폭의 허용 범위는 **거리에 따라 달라진다** — 고정 상수로 두면
# 안 된다(2026-08-26 접근 실기에서 드러났다). 자세한 근거는
# face_width_bounds() 참고. 아래 두 값은 그 함수가 쓰는 비율이다.
#
# 하한을 절반으로 두는 이유: 정면을 비스듬히 보면 겉보기 폭이 cos만큼
# 줄어들어, 45도까지는 받아들이겠다는 뜻이다.
FACE_WIDTH_MIN_RATIO = 0.50
# 상한: 실측 겉보기 폭이 예측의 0.96~1.02배였다. 잡음과 가장자리 점을
# 감안해 1.15배까지 받는다.
FACE_WIDTH_MAX_RATIO = 1.15


class FaceFit(NamedTuple):
    ok: bool
    distance_m: float
    yaw_error_rad: float
    face_width_m: float
    residual_m: float
    point_count: int
    reason: str
    # 차량 중심선에서 바구니 정면 중심까지의 좌우 거리(+ 왼쪽).
    # `lateral_known`이 False면 **모르는 것**이지 0이 아니다 — 바구니가
    # 방위각 창을 양쪽 다 채워 가장자리가 안 보이는 경우다.
    lateral_offset_m: float = 0.0
    lateral_known: bool = False


def scan_to_points(ranges, angle_min: float, angle_increment: float,
                   range_min: float = 0.05, range_max: float = 3.0) -> list:
    """LaserScan의 ranges를 base_link 기준 (x, y) 점들로 바꾼다.

    inf·nan·범위 밖 값은 버린다 — 라이다는 반사가 없으면 inf를 낸다."""
    points = []
    for i, r in enumerate(ranges):
        if r is None:
            continue
        if math.isnan(r) or math.isinf(r):
            continue
        if r < range_min or r > range_max:
            continue
        angle = angle_min + i * angle_increment
        points.append((r * math.cos(angle), r * math.sin(angle)))
    return points


def scan_to_front_points(ranges, angle_min: float, angle_increment: float,
                         range_min: float = 0.02, range_max: float = 3.0,
                         front_offset_deg: float = FRONT_OFFSET_DEG,
                         tilt_deg: float = LIDAR_TILT_DEG) -> list:
    """`scan_to_points`에 **차량 정면 기준 회전과 기울기 보정**을 얹는다.

    실기에서 라이다 스캔을 쓸 때는 항상 이쪽을 쓴다. 원시 각도를 그대로
    쓰면 정면이 90도 어긋나고(모듈 docstring "방위각 원점"), 거리는 2.0%
    길게 나온다(같은 docstring "높이·기울기 전제").

    돌려주는 점들은 차량 정면이 +x, 왼쪽이 +y인 평면 좌표다 — 그래서
    `expected_bearing_rad=0.0`으로 `fit_basket_face()`에 그대로 넣으면
    정면의 바구니를 본다."""
    cos_tilt = math.cos(math.radians(tilt_deg))
    offset = math.radians(front_offset_deg)
    points = []
    for i, r in enumerate(ranges):
        if r is None:
            continue
        if math.isnan(r) or math.isinf(r):
            continue
        if r < range_min or r > range_max:
            continue
        angle = angle_min + i * angle_increment - offset
        horizontal = r * cos_tilt
        points.append((horizontal * math.cos(angle), horizontal * math.sin(angle)))
    return points


def beam_height_m(forward_m: float,
                  lidar_height_m: float = LIDAR_HEIGHT_M,
                  tilt_deg: float = LIDAR_TILT_DEG) -> float:
    """라이다에서 정면으로 `forward_m` 떨어진 곳에서의 빔 높이(m).

    바구니 테두리를 넘길 수 있는 거리인지 판단할 때 쓴다 — 값이
    `BASKET_RIM_HEIGHT_M`보다 커지면 빔이 테두리 위를 스쳐 바구니를
    놓친다."""
    return lidar_height_m - forward_m * math.tan(math.radians(tilt_deg))


def select_face_points(points: list, expected_bearing_rad: float,
                       window_rad: float = DEFAULT_BEARING_WINDOW_RAD,
                       cluster_depth_m: float = DEFAULT_CLUSTER_DEPTH_M) -> list:
    """기대 방위각 창 안에서 **가장 가까운 덩어리**만 남긴다.

    Host가 어느 바구니로 가는지 알려주므로 기대 방위각은 항상 있다. 그
    창 안에서 최근접 거리부터 `cluster_depth_m` 안에 드는 점만 취하면
    뒤쪽 벽이 떨어져 나간다."""
    in_window = []
    for x, y in points:
        bearing = math.atan2(y, x)
        delta = (bearing - expected_bearing_rad + math.pi) % (2.0 * math.pi) - math.pi
        if abs(delta) <= window_rad:
            in_window.append((x, y))
    if not in_window:
        return []
    nearest = min(math.hypot(x, y) for x, y in in_window)
    return [
        (x, y) for x, y in in_window
        if math.hypot(x, y) <= nearest + cluster_depth_m
    ]


def face_width_bounds(distance_m: float,
                      window_rad: float = DEFAULT_BEARING_WINDOW_RAD,
                      face_width_m: float = BASKET_FACE_WIDTH_M):
    """그 거리에서 **볼 수 있는** 겉보기 폭의 하한·상한을 낸다.

    2026-08-26 접근 실기에서 폭 검사가 조용히 무의미해지는 것을 발견해
    넣었다. 가까워지면 겉보기 폭이 바구니가 아니라 **방위각 창**으로
    결정된다 — 창이 잘라 낸 폭은 `2 * 거리 * tan(창)`이고, 실측이 이
    예측과 0.96~1.02배로 맞았다:

        거리 0.130m -> 창 182mm, 실측 182mm
        거리 0.145m -> 창 203mm, 실측 196mm
        거리 0.177m -> 창 248mm(> 바구니 242mm) -> 실측 242mm
        거리 0.532m -> 바구니 242mm            -> 실측 239mm

    그래서 기대 폭은 `min(바구니 폭, 창이 자른 폭)`이다. 고정 상수
    (예전의 100~320mm)로 두면 근거리에서 상한이 너무 헐거워 아무것도
    못 거른다.

    ⚠️ **한계**: 창이 바구니보다 좁아지는 거리
    (`WIDTH_TEST_USEFUL_ABOVE_M`) 아래로는 벽이 섞여도 폭이 똑같이 창
    폭으로 나오므로, 이 검사는 판별력을 잃는다. 그 구간에서는 잔차와
    점 개수가 유일한 방어선이다."""
    geometric_limit = 2.0 * distance_m * math.tan(window_rad)
    visible = min(face_width_m, geometric_limit)
    return FACE_WIDTH_MIN_RATIO * visible, FACE_WIDTH_MAX_RATIO * visible


# 이 거리 아래로는 방위각 창이 바구니보다 좁아져 폭 검사가 판별력을 잃는다.
WIDTH_TEST_USEFUL_ABOVE_M = BASKET_FACE_WIDTH_M / (2.0 * math.tan(DEFAULT_BEARING_WINDOW_RAD))


def fit_line(points: list):
    """점들에 직선을 총최소제곱으로 맞춘다.

    `(nx, ny, cx, cy, residual_rms, width)`를 돌려준다. `(nx, ny)`는 단위
    법선이고 무게중심에서 원점 반대쪽을 향하도록 부호를 맞춘다. `width`는
    직선 방향으로의 점 분포 폭이다. 점이 2개 미만이면 None."""
    n = len(points)
    if n < 2:
        return None
    cx = sum(p[0] for p in points) / n
    cy = sum(p[1] for p in points) / n
    sxx = sum((p[0] - cx) ** 2 for p in points)
    syy = sum((p[1] - cy) ** 2 for p in points)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in points)
    # 공분산행렬의 주축(큰 고윳값) 방향 = 직선 방향.
    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    dx, dy = math.cos(theta), math.sin(theta)
    nx, ny = -dy, dx
    # 법선이 로봇에서 바구니를 향하게 한다.
    if nx * cx + ny * cy < 0.0:
        nx, ny = -nx, -ny
    residual_sq = sum((nx * (p[0] - cx) + ny * (p[1] - cy)) ** 2 for p in points)
    residual_rms = math.sqrt(residual_sq / n)
    projections = [dx * (p[0] - cx) + dy * (p[1] - cy) for p in points]
    width = max(projections) - min(projections)
    return nx, ny, cx, cy, residual_rms, width


def face_lateral_offset_m(points: list, distance_m: float,
                          window_rad: float = DEFAULT_BEARING_WINDOW_RAD,
                          face_width_m: float = BASKET_FACE_WIDTH_M,
                          edge_margin_m: float = 0.010):
    """차량이 바구니 정면에 대해 좌우로 얼마나 밀려 있는지 낸다.

    `(오프셋 m, 알아냈는가)`. 못 알아내면 `(0.0, False)` — **0은 "가운데"가
    아니라 "모른다"다.**

    ## 왜 이게 따로 필요한가

    선분 피팅이 주는 거리와 yaw로는 이 오차가 **안 보인다**. 차량이 바구니와
    나란한 채로 옆으로 8cm 밀려 있으면 수직 거리도 yaw도 정상으로 나오는데
    물체는 바구니 밖에 떨어진다. 2026-08-26까지의 판정에 있던 구멍이다.

    ## 어떻게 보이는가

    방위각 창이 자르는 폭은 `2 * 거리 * tan(창)`이다. 0.140m에서 196mm로
    바구니 폭 242mm보다 좁으므로, 차량이 가운데 있으면 창 양쪽 끝까지
    바구니가 꽉 찬다. 옆으로 밀리면 **한쪽에서 바구니 가장자리가 창 안으로
    들어온다** — 그 가장자리가 보이는 순간 중심 위치를 역산할 수 있다.

    0.140m에서 오프셋 23mm부터 가장자리가 보이기 시작한다. 투하가 실패하는
    오프셋(바구니 반폭 121mm에서 물체와 여유를 뺀 약 78mm)보다 한참 먼저라
    위험 구간을 덮는다.

    ## 근사

    yaw가 작다는 전제로 점의 y좌표를 그대로 좌우 위치로 쓴다. INSERT 허용
    yaw가 5도이고 그때 오차가 거리의 0.4% 수준이라 무시할 수 있다."""
    if not points or distance_m <= 0.0:
        return 0.0, False
    ys = [y for _x, y in points]
    y_min, y_max = min(ys), max(ys)
    window_half = distance_m * math.tan(window_rad)
    half_face = face_width_m / 2.0

    left_edge_visible = y_max < window_half - edge_margin_m
    right_edge_visible = y_min > -window_half + edge_margin_m

    if left_edge_visible and right_edge_visible:
        # 양쪽 가장자리가 다 보인다 — 중심은 그 한가운데다. 가장 확실한 경우.
        return (y_min + y_max) / 2.0, True
    if left_edge_visible:
        return y_max - half_face, True
    if right_edge_visible:
        return y_min + half_face, True
    # 양쪽 다 창에 막혔다 — 바구니가 창보다 넓다는 것만 알 뿐 중심은 모른다.
    return 0.0, False


def fit_basket_face(
    points: list,
    expected_bearing_rad: float,
    window_rad: float = DEFAULT_BEARING_WINDOW_RAD,
    cluster_depth_m: float = DEFAULT_CLUSTER_DEPTH_M,
    min_points: int = DEFAULT_MIN_POINTS,
    max_residual_m: float = DEFAULT_MAX_RESIDUAL_M,
    min_face_width_m: float = None,
    max_face_width_m: float = None,
) -> FaceFit:
    """바구니 정면까지의 거리와 정렬 오차를 낸다.

    **모르면 실패**(`ok=False`)다 — 점이 모자라거나, 잔차가 크거나(평면이
    아니다), 겉보기 폭이 바구니로 볼 수 없으면 판정하지 않는다. INSERT
    전환을 막는 쪽이 안전하다."""
    face = select_face_points(points, expected_bearing_rad, window_rad, cluster_depth_m)
    if len(face) < min_points:
        return FaceFit(False, math.inf, math.inf, 0.0, math.inf, len(face),
                       f"점 부족 — {len(face)}개(최소 {min_points})")
    fit = fit_line(face)
    if fit is None:
        return FaceFit(False, math.inf, math.inf, 0.0, math.inf, len(face), "직선 피팅 실패")
    nx, ny, cx, cy, residual, width = fit
    distance = nx * cx + ny * cy
    yaw_error = math.atan2(ny, nx)
    if residual > max_residual_m:
        return FaceFit(False, distance, yaw_error, width, residual, len(face),
                       f"잔차 {residual * 1000:.1f}mm — 평면이 아니다"
                       f"(상한 {max_residual_m * 1000:.0f}mm)")
    # 폭 한계는 거리에 따라 달라진다 — face_width_bounds() 참고.
    lower, upper = face_width_bounds(distance, window_rad)
    if min_face_width_m is not None:
        lower = min_face_width_m
    if max_face_width_m is not None:
        upper = max_face_width_m
    if not (lower <= width <= upper):
        return FaceFit(False, distance, yaw_error, width, residual, len(face),
                       f"겉보기 폭 {width * 1000:.0f}mm — 바구니로 볼 수 없다"
                       f"({lower * 1000:.0f}~{upper * 1000:.0f}mm)")
    offset, offset_known = face_lateral_offset_m(face, distance, window_rad)
    return FaceFit(True, distance, yaw_error, width, residual, len(face), "정면 확보",
                   offset, offset_known)


# 잔차 검사가 잡는 것과 못 잡는 것
#
# 잡는다: 모서리(두 면이 만나는 곳), 벽과 바구니가 섞인 덩어리 — 둘 다
#         단일 직선에서 크게 벗어난다.
# 못 잡는다: 완만한 곡면. 반지름 0.30m 호를 200mm 구간에서 보면 잔차가
#         5.9mm에 불과해 정상 평면과 구분되지 않는다. 이 아레나에 그런
#         장애물이 없다는 전제에 기대는 부분이다(tests/test_basket_lidar_align.py
#         의 test_완만한_곡면은_구분하지_못한다 참고).
