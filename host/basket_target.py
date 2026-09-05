"""바구니 입구 목표 영역 기반 INSERT 게이트 (사용자 지시, 2026-09-04).

## 배경

기존 NUDGE_BOX/PLACE 폐루프는 Pi 라이다 판독(`_plan_basket_fix`)에 기대
"정면으로, 정해진 거리까지" 반복 보정한다 — 이 세션 초반에 이걸 걷어내고
Host의 ArUco 좌표만으로 판단하게 바꿨다가(commit 597738b) 검증이 덜 된
채였다는 게 드러나 되돌렸다(그리퍼-lidar-may-be-removed-again 메모리
참고). 대신 이번엔 사용자가 직접 기하 조건을 정했다:

    바구니 입구 중심 기준 로컬 좌표로 가로(±3cm) x 안쪽(0~3cm)인 작은
    직사각형 "목표 영역"에서 차량이 15cm 이내에 있고, 그 영역을
    바라보고 있기만 하면(정면일 필요 없음 — 사선 진입 허용) INSERT로
    넘어가도 된다. (가로 폭은 처음엔 ±5cm였다가 같은 날 저녁 ±3cm로
    좁혔다 — TARGET_HALF_WIDTH_M 주석 참고.)

이 세션에 실측한 라이다 데이터(정면·45도·30도 모두 벽/모서리를 잔차
1.5~7mm 수준으로 분간)가 "사선에서도 바구니 존재 자체는 믿을 만하게
확인된다"는 근거가 됐다 — 그래서 Host가 정면을 강제하지 않아도 되는
쪽으로 조건을 완화할 수 있다고 판단했다.

## 이 모듈이 하는 일과 안 하는 일

여기는 **1차 관문**이다 — "Host가 아는 좌표로 볼 때 시도해 볼 만한
위치인가"만 순수 계산으로 답한다(ROS/하드웨어 의존 없음, aruco/config.py
`box_pose()`의 (x, y, yaw) 튜플만 있으면 된다 — grippers_arena/
aruco_localization.py, domain/task/basket_lidar_align.py와 같은 이유로
rclpy 없이 pytest로 검증한다).

**최종 확인은 여전히 Pi가 한다** — `check_insert`(domain/task/
preconditions.py)의 라이다 조건이 이 게이트를 통과한 뒤에도 그대로
남아 있고, `baseline_constants.LIDAR_INSERT_CHECK_ENABLED`로 언제든
끌 수 있게 해 뒀다(사용자 지시: "LiDAR가 필요 없다 싶으면 뺄 수
있도록"). 이 모듈은 그 스위치와 무관하게 항상 켜져 있는 Host 쪽
1차 판단일 뿐이다.

## 좌표 규약

`_box_front_xy()`(mission.py)와 같은 단순화를 그대로 따른다 — 이
프로젝트의 두 상자(toy, chess)는 전부 뒤쪽 벽에 붙어 같은 방향(작업
영역 쪽, 세계좌표 -y)으로 열려 있다는 게 실측으로 고정돼 있어서,
`box_pose()`가 돌려주는 yaw는 이 계산에 안 쓴다(그 함수도 그렇다) —
나중에 상자를 임의 방향으로 놓게 되면 이 단순화부터 같이 깨질 것이므로
그때 이 모듈도 손볼 것.
"""

from __future__ import annotations

import math
from typing import NamedTuple

import config as cfg
import mission_config as mcfg
from localizer import box_pose

# 사용자 실측 지시값(2026-09-04, 같은 날 저녁 재조정 — 처음엔 좌우 5cm였다가
# "아까 준 좌표에서 양옆으로 3cm, 바구니 안쪽 3cm로 조정해"로 좁혔다).
TARGET_HALF_WIDTH_M = 0.03      # 입구 중심 기준 좌우(가로) 절반폭
TARGET_INSET_DEPTH_M = 0.03    # 입구 중심에서 바구니 안쪽으로 깊이

# 2026-09-05 실기(manual_insert_probe.py)로 처음 이 값을 실제 자세에서
# 확인해보니 0.15에서 NUDGE_LINE이 걸려 사용자가 보기에 바구니에 "너무
# 가깝다"고 판단 — 10cm 더 여유를 두라는 지시로 0.25로 늘렸다. 아래
# SOUTH_APPROACH_SECTOR_RADIUS_M도 이 값을 그대로 참조하므로 같이 늘어난다
# (basket_target.py 뒷부분 주석 — 두 반경은 항상 같아야 한다는 설계 그대로
# 유지). 목표 영역 자체(TARGET_HALF_WIDTH_M/TARGET_INSET_DEPTH_M)는 안 건드
# 렸다 — "언제 멈출까"만 바뀌었지 "어디가 목표인가"는 그대로다.
MAX_APPROACH_DIST_M = 0.25     # 목표 영역에서 이 거리 안이면 "가깝다"

# "바라보고 있다"의 허용 오차. 이번 세션 라이다 실측으로 정면·45도·30도
# 사선 전부에서 벽/모서리 분간이 됐다(잔차 1.5~7mm) — 그러니 45도까지는
# 받아들여야 이 기능을 도입한 취지(사선 허용)가 산다. 그보다 넉넉히 잡되
# 90도(완전히 옆을 봄)까지는 안 열어 준다.
# ⚠️ 실기 조정 대상 — 아직 이 각도로 실제 INSERT를 검증하지 않았다.
MAX_FACING_ERROR_DEG = 50.0


class InsertGateResult(NamedTuple):
    ok: bool
    distance_m: float
    facing_error_deg: float
    target_xy: tuple[float, float]
    reason: str


def _nearest_point_in_rect(rx: float, ry: float,
                            x_lo: float, x_hi: float,
                            y_lo: float, y_hi: float) -> tuple[float, float]:
    return (min(max(rx, x_lo), x_hi), min(max(ry, y_lo), y_hi))


def target_rect(
    box_name: str,
    half_width_m: float = TARGET_HALF_WIDTH_M,
    inset_depth_m: float = TARGET_INSET_DEPTH_M,
) -> tuple[float, float, float, float]:
    """`box_name` 바구니의 입구 목표 영역을 `(x_lo, x_hi, y_lo, y_hi)`로
    낸다. `check_basket_insert_gate()`와 LiveMap 표시(2026-09-04, 사용자
    지시)가 이 한 곳을 같이 쓴다 — 계산이 두 곳에서 갈라지면 화면에
    보이는 표시와 실제 판정이 다른 곳을 가리키게 된다."""
    bx, by, _byaw = box_pose(box_name)
    if box_name == "toy":
        # mission.py의 _box_front_xy와 같은 보정 — 2026-09-03 실기로
        # 밝혀진 toy 바구니 목적지 좌측 편향을 여기서도 그대로 반영한다.
        bx -= mcfg.TOY_DEST_X_SHIFT_LEFT_M
    edge_y = by - cfg.BOX_L / 2.0   # 상자 중심 -> 입구(작업영역 쪽 앞면)
    if box_name == "chess":
        # 2026-09-04 실기 보정 — mission_config.CHESS_APPROACH_EXTRA_DEPTH_M
        # 주석 참고. config.BOXES 실측값은 그대로 두고 목표영역만 그만큼
        # 안쪽으로(로봇에서 더 먼 쪽으로) 옮긴다.
        edge_y += mcfg.CHESS_APPROACH_EXTRA_DEPTH_M
    return bx - half_width_m, bx + half_width_m, edge_y, edge_y + inset_depth_m


def target_center(box_name: str) -> tuple[float, float]:
    """목표 영역의 중심점 — LiveMap에 X로 찍을 때 쓴다."""
    x_lo, x_hi, y_lo, y_hi = target_rect(box_name)
    return (x_lo + x_hi) / 2.0, (y_lo + y_hi) / 2.0


def check_basket_insert_gate(
    robot_xy: tuple[float, float],
    robot_yaw_deg: float,
    box_name: str,
    half_width_m: float = TARGET_HALF_WIDTH_M,
    inset_depth_m: float = TARGET_INSET_DEPTH_M,
    max_distance_m: float = MAX_APPROACH_DIST_M,
    facing_tolerance_deg: float = MAX_FACING_ERROR_DEG,
) -> InsertGateResult:
    """차량이 `box_name` 바구니의 입구 목표 영역 근처에서 그쪽을 보고
    있는지 판정한다. INSERT를 시도해 볼 만한지에 대한 Host 쪽 1차
    관문이다 — 최종 확인은 Pi의 `check_insert`가 한다(모듈 docstring
    참고)."""
    x_lo, x_hi, y_lo, y_hi = target_rect(box_name, half_width_m, inset_depth_m)

    rx, ry = robot_xy
    tx, ty = _nearest_point_in_rect(rx, ry, x_lo, x_hi, y_lo, y_hi)
    dx, dy = tx - rx, ty - ry
    distance = math.hypot(dx, dy)

    if distance < 1e-6:
        # 로봇이 목표 영역 안에 있다(사실상 안 생기는 경우지만, atan2(0,0)
        # 로 방위각이 정의되지 않는 것을 막는다) — 지향 오차는 0으로 본다.
        facing_error = 0.0
    else:
        bearing_to_target_deg = math.degrees(math.atan2(dy, dx))
        facing_error = (bearing_to_target_deg - robot_yaw_deg + 180.0) % 360.0 - 180.0

    reasons = []
    if distance > max_distance_m:
        reasons.append(f"목표 영역까지 {distance * 1000:.0f}mm > "
                        f"{max_distance_m * 1000:.0f}mm")
    if abs(facing_error) > facing_tolerance_deg:
        reasons.append(f"목표 영역을 안 보고 있다 ({facing_error:+.1f}deg > "
                        f"±{facing_tolerance_deg:.0f}deg)")

    ok = not reasons
    reason = "목표 영역 도달 + 지향 확인" if ok else "; ".join(reasons)
    return InsertGateResult(ok, distance, facing_error, (tx, ty), reason)


# ---------------------------------------------------------------------------
# 접근 부채꼴 게이트 (사용자 지시, 2026-09-05)
#
# 위 check_basket_insert_gate()는 "지금 로봇이 그쪽을 보고 있는가"(지향)까지
# 같이 본다 — NUDGE_BOX 안에서 이미 정렬된 뒤에 "여전히 유효한가"를 되묻는
# 용도라 그게 맞다. 여기 새로 두는 건 그보다 앞 단계, CARRY_TO_DEST가
# "이제 멈추고 방향을 잡아도 되는가"를 정하는 게이트라 로봇이 지금 어디를
# 보고 있는지는 상관없다 — 위치만 본다.
#
# 목표영역 "중심"(target_center)을 기준으로 반경 원을 120도씩 3등분해서,
# 접근 방향(남쪽 = 작업구역 쪽 = -y = 6시) 부채꼴 안에 로봇이 있으면
# "예비 INSERT 후보"로 본다. 이 부채꼴의 바깥 호(원주의 1/3)가 그 판정
# 경계선이다 — 호 위 어디로 들어오든 유효하고, 그 순간 그 자리에서
# 목표중심을 향한 방위각(align_yaw_deg)으로 제자리 정렬하면 된다.
# 정중앙(정남쪽)으로 들어오면 이 각도가 정확히 기존
# mission_config.BOX_FACE_YAW_DEG(90도)와 같아진다 — 그 값이 이 일반식의
# 특수 케이스였다는 뜻이다.
#
# ⚠️ 반경 = MAX_APPROACH_DIST_M(0.15), check_basket_insert_gate()와 같은 값.
#
# 2026-09-05 낮에는 이 값을 0.30으로 따로 더 크게 뒀었다 — 로봇의 ArUco
# 기준점과 실제 라이다/그리퍼가 물리적으로 앞으로 튀어나온 만큼(tests/
# conftest.py의 LIDAR_AHEAD_M, 2026-08-28 실측에서 역산한 약 0.099m, 이
# 저장소 실제 코드엔 이 오프셋을 아는 상수가 없다) 감안 못 한 채 0.15를
# 그대로 쓰면, 부채꼴 진입 판정 순간 Pi 라이다가 이미 바구니 앞면을 지나쳐
# 있다는 걸(시뮬레이션 회귀: -0.03~-0.05m) 발견했기 때문이다.
#
# 그런데 그 값을 정하려면 실측 못한 그 오프셋을 계속 추정해야 했다 —
# 사용자 지시(2026-09-05 저녁): "라이다 뺀 상황으로 전제하고 다시 수정해".
# 그리퍼-prefer-no-lidar-for-insert 메모리와 같은 방향이다 — INSERT 최종
# 확인을 Pi 라이다 실측(그리고 그 실측이 어디서 나는지도 모르는 물리
# 오프셋)에 기대지 않고, Host가 아는 ArUco 기하만으로 완결되게 설계한다.
# 그러면 "라이다가 앞면을 지나쳤는가"라는 질문 자체가 무의미해진다 — 이
# 게이트가 보는 것도, NUDGE_BOX의 무라이다 경로(LIDAR_INSERT_CHECK_ENABLED
# =False일 때 check_basket_insert_gate로 전진량을 스스로 계산하는 쪽, 아래
# mission.py NUDGE_BOX 주석 참고)가 완주를 확인하는 것도 전부 같은
# check_basket_insert_gate()의 목표영역-거리·지향 판정이라, 이 부채꼴이
# 그 판정 반경(0.15)보다 넓을 이유가 없다 — 오히려 같은 반경을 써야 부채꼴
# 진입 = "그 뒤 무라이다 창이 곧바로 열린다"가 보장된다. 물리적 안전은
# 이 반경과 무관하게 hard_stop(상자 중심 기준 순수 ArUco 반경, 아래
# NUDGE_BOX 참고)이 항상 별도로 지킨다.
SOUTH_APPROACH_SECTOR_RADIUS_M = MAX_APPROACH_DIST_M
SOUTH_APPROACH_SECTOR_DEG = 120.0   # 3등분 중 접근(남쪽) 부채꼴의 폭
_SOUTH_BEARING_DEG = -90.0          # 목표중심 기준 "정남쪽"(atan2 규약, -y)


class ApproachSectorResult(NamedTuple):
    ok: bool
    distance_m: float
    bearing_from_center_deg: float   # 목표중심 -> 로봇 방향(atan2 규약)
    align_yaw_deg: float             # 그 자리에서 목표중심을 향할 정렬 방위각
    center_xy: tuple[float, float]
    reason: str


def _normalize_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def check_approach_sector(
    robot_xy: tuple[float, float],
    box_name: str,
    radius_m: float = SOUTH_APPROACH_SECTOR_RADIUS_M,
    sector_deg: float = SOUTH_APPROACH_SECTOR_DEG,
) -> ApproachSectorResult:
    """`box_name` 목표영역 중심을 기준으로, 로봇이 접근(남쪽) 부채꼴의
    반경 `radius_m` 안에 있는지를 위치만으로 판정한다. ok면
    `align_yaw_deg`가 그 자리에서 목표중심을 향하는 제자리 정렬 목표각이다
    — CARRY_TO_DEST가 이 결과로 FACE_BOX 진입 여부와 정렬각을 함께 정한다."""
    cx, cy = target_center(box_name)
    rx, ry = robot_xy
    dx, dy = rx - cx, ry - cy
    distance = math.hypot(dx, dy)
    if distance < 1e-6:
        # 로봇이 목표중심과 정확히 겹친다(사실상 안 생기는 경우) — atan2(0,0)
        # 미정의를 피하려고 정남쪽을 가정한다(check_basket_insert_gate의
        # distance<1e-6 처리와 같은 이유).
        bearing = _SOUTH_BEARING_DEG
    else:
        bearing = math.degrees(math.atan2(dy, dx))
    align_yaw = _normalize_deg(bearing + 180.0)

    half = sector_deg / 2.0
    rel = _normalize_deg(bearing - _SOUTH_BEARING_DEG)
    in_sector = abs(rel) <= half
    ok = distance <= radius_m and in_sector

    if ok:
        reason = "접근 부채꼴 진입 — 예비 INSERT 후보"
    elif distance > radius_m:
        reason = f"목표중심까지 {distance * 1000:.0f}mm > {radius_m * 1000:.0f}mm"
    else:
        reason = f"접근 부채꼴 밖 (중심 기준 {bearing:+.1f}deg, 남쪽 ±{half:.0f}deg만 유효)"

    return ApproachSectorResult(ok, distance, bearing, align_yaw, (cx, cy), reason)
