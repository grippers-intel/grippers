"""물체 접근 시각 서보 루프의 순수 제어 수학.

⚠️ 2026-08-23 재설계: 원래 tools/perception/approach.py를 그대로 이식해
좌우(linear_y) 옆걸음 + 전진(linear_x)만으로 화면 중앙에 맞추는 방식이었다.
사용자 지적(실기 관찰) — 회전 없이 순수 이동만 쓰다 보니, SCAN이 골라준
물체의 초기 방위각(차체 정면축과 물체 사이 각도)이 큰 경우(실측 로그: 9~24도)
그 오차를 전부 좌우 이동으로 상쇄해야 해서 접근 경로가 필요 이상으로
지그재그였다. 이번 개정은 **회전(제자리 정렬) + 전진**으로 바꾼다 —
linear_y는 더 이상 화면 중앙 정렬에 안 쓰고, 전방 장애물 회피(옆으로 잠깐
비키기) 전용으로 돌린다.

⚠️ 이 파일의 회전+전진 조합은 아직 실기 미검증이다(기존 좌우-이동 버전은
HANDOFF.md가 검증했지만, 이 회전 버전은 처음이다) — GAIN_TURN·MAX_TURN·
MIN_TURN 세 값은 실측 전 자리 표시자다. 실기에서 가장 먼저 확인할 것.

**두 개의 독립된 신호를 쓴다** (기존과 동일):
  거리 ← 검출 박스의 높이(px). 가까워질수록 커지고 좌우 이동과 무관하다.
  방위 ← 박스 아래변 중점의 x(px) — 이제 좌우 이동이 아니라 제자리 회전으로 없앤다.

카메라 구독·cmd_vel 발행·ROS2 액션 서버 배선은 이 파일에 없다 — 오차 →
속도 지령 변환만 담아 rclpy 없이 pytest로 검증한다(drive_control.py와
같은 이유). 라이다 섹터 추출(min_range_in_arc)도 각도·거리 리스트만
받는 순수 함수라 여기 둔다 — LaserScan 메시지 자체는 base_driver_node.py가
풀어서 넘긴다.
"""
from __future__ import annotations

import math
from typing import NamedTuple

# tools/perception/approach.py의 argparse 기본값과 동일 — 원본 좌우-이동
# 버전에서 실기 검증된 값이라 거리(h) 쪽은 그대로 가져온다.
DEFAULT_TOL_X_PX = 8.0
DEFAULT_TOL_H_PX = 6.0
DEFAULT_GAIN_H = 0.0016
DEFAULT_MAX_SPEED = 0.08
DEFAULT_MIN_SPEED = 0.05
DEFAULT_BURST_S = 0.35
DEFAULT_MIN_BURST_S = 0.15
DEFAULT_ALIGN_FIRST = 2.0

# ⚠️ 실기 미검증 자리 표시자 — 회전 버전 최초 실기 테스트에서 가장 먼저
# 확인·조정할 값들. 원본 gain_x(0.0009 m/s/px, 옆걸음용)를 그대로 회전에
# 옮길 수 없어(단위가 다르다 — m/s가 아니라 rad/s) 보수적으로 낮게 잡았다.
DEFAULT_GAIN_TURN = 0.003  # rad/s per px
DEFAULT_MAX_TURN = 0.5  # rad/s (~29도/s)
DEFAULT_MIN_TURN = 0.15  # rad/s — 이 아래로는 제자리 회전이 정지마찰을 못 이길 것으로 추정(미실측)

# ⚠️ 실기 미검증 — 장애물 회피(옆으로 비키기) 상수.
DEFAULT_OBSTACLE_SAFETY_M = 0.35
DEFAULT_DODGE_SPEED = 0.06
DEFAULT_DODGE_BURST_S = 0.4


class ApproachCommand(NamedTuple):
    linear_x: float
    linear_y: float
    angular_z: float
    burst_s: float
    arrived: bool


def compute_approach_error(obs_x: float, obs_h: float, target_x: float, target_h: float):
    """관측(obs_x=박스 아래변 중점 x, obs_h=박스 높이)과 교시 목표의 오차.

    err_x: +면 물체가 목표보다 오른쪽에 보인다(→ 제자리에서 오른쪽으로 돌아야 한다).
    err_h: +면 아직 멀다(박스가 작다) — h는 거리에 반비례하지 않고 비례하므로
    부호가 직관과 반대다(목표 h − 관측 h)."""
    return obs_x - target_x, target_h - obs_h


def apply_axis_floor(v: float, min_v: float, max_v: float) -> float:
    """0이 아닌 지령이 데드밴드(min_v)보다 작으면 min_v까지 끌어올리고, 상한
    (max_v)을 넘으면 자른다. v가 정확히 0이면 그대로 0(정지 유지).

    선속도·각속도는 물리 단위가 달라 하나의 배율로 묶어 같이 스케일할 근거가
    없다(이전 버전 apply_floor는 vx·vy가 둘 다 선속도라 묶었다) — 축마다
    독립적으로 처리한다."""
    if v == 0.0:
        return 0.0
    magnitude = min(max_v, max(min_v, abs(v)))
    return magnitude if v > 0 else -magnitude


def compute_approach_command(
    err_x: float,
    err_h: float,
    *,
    tol_x: float = DEFAULT_TOL_X_PX,
    tol_h: float = DEFAULT_TOL_H_PX,
    gain_turn: float = DEFAULT_GAIN_TURN,
    gain_h: float = DEFAULT_GAIN_H,
    max_speed: float = DEFAULT_MAX_SPEED,
    min_speed: float = DEFAULT_MIN_SPEED,
    max_turn: float = DEFAULT_MAX_TURN,
    min_turn: float = DEFAULT_MIN_TURN,
    burst: float = DEFAULT_BURST_S,
    align_first: float = DEFAULT_ALIGN_FIRST,
    invert_turn: bool = False,
) -> ApproachCommand:
    """오차 한 쌍을 다음 한 번의 `nudge(vx, wz, secs)` 지령으로 바꾼다.

    수렴(양쪽 오차가 허용치 안)이면 `arrived=True`와 함께 속도 0을 돌려준다 —
    호출자는 이때 정지 명령을 내고 루프를 끝낸다. `linear_y`는 항상 0 —
    화면 중앙 정렬은 이제 회전(angular_z)만 담당한다(장애물 회피 시에만
    별도로 `compute_dodge_command`가 linear_y를 쓴다)."""
    done_x, done_h = abs(err_x) <= tol_x, abs(err_h) <= tol_h
    if done_x and done_h:
        return ApproachCommand(0.0, 0.0, 0.0, 0.0, True)

    sign = -1.0 if invert_turn else 1.0
    # err_x>0(물체가 오른쪽) → 오른쪽으로 돌아야 한다 → REP103 관례상 angular.z<0.
    wz = 0.0 if done_x else max(-max_turn, min(max_turn, -err_x * gain_turn * sign))
    vx = 0.0 if done_h else max(-max_speed, min(max_speed, err_h * gain_h))

    # 좌우가 크게 어긋난 채로 전진하면 물체를 지나쳐버린다(실측 실패 사례 —
    # HANDOFF.md "수동으로 파지 위치 맞추기 — 실패"). 회전으로 바꾼 뒤에도
    # 같은 이유로 방위 정렬을 먼저 시킨다 — 전진을 늦춰 회전이 따라잡게 한다.
    if align_first > 0 and abs(err_x) > align_first * tol_x:
        vx *= 0.25

    vx = apply_axis_floor(vx, min_speed, max_speed)
    wz = apply_axis_floor(wz, min_turn, max_turn)
    return ApproachCommand(vx, 0.0, wz, burst, False)


def obstacle_ahead(min_front_range_m: float | None, safety_distance_m: float = DEFAULT_OBSTACLE_SAFETY_M) -> bool:
    """전방 라이다 최소 거리가 안전거리 안이면 True.

    ⚠️ 실기 확인(2026-08-23, LD19 라이다 장착 상태): 라이다 장착 높이(base_link
    기준 9.25cm)에서는 체스말·축구공 같은 파지 대상이 전방 스캔에 아예 안
    잡힌다(전방 반환이 뒤쪽 배경 벽까지 뚫려 보임, 최근접 반환이 0.97m —
    실제로는 그보다 훨씬 가까운 물체가 눈앞에 있었는데도). 즉 이 게이트가
    지금 접근 중인 파지 대상 자체를 장애물로 오인할 걱정은 없다 — 다만
    라이다가 못 보는 낮은 물체(문턱 등)에는 여전히 무력하다는 뜻이기도 하다.

    `min_front_range_m=None`(라이다 미기동·구독 전·전방 반환 없음)이면
    **False**를 돌려준다 — "모르면 막는다"가 아니라 "모르면 이 기능이 없던
    것처럼 행동한다"를 택했다. 반대로 하면(모르면 장애물 있음으로 간주) 라이다가
    하나라도 안 떠 있을 때 접근이 매번 옆으로 비키기만 반복하는 새 고장
    모드가 생긴다 — 이 게이트는 있으면 좋은 안전장치지, 없으면 접근 자체가
    안 되는 필수 전제가 아니다."""
    if min_front_range_m is None:
        return False
    return min_front_range_m < safety_distance_m


def choose_dodge_side(left_min_m: float | None, right_min_m: float | None) -> float:
    """장애물을 피해 어느 쪽으로 비킬지 정한다. +1.0=왼쪽(양의 y), -1.0=오른쪽.
    더 여유 있는(거리가 더 먼) 쪽으로 피한다. 둘 다 모르면(라이다 없음)
    임의로 오른쪽을 기본값으로 정한다 — 방향을 안 정하면 회피 자체가 안 된다."""
    if left_min_m is None and right_min_m is None:
        return -1.0
    if left_min_m is None:
        return -1.0
    if right_min_m is None:
        return 1.0
    return 1.0 if left_min_m >= right_min_m else -1.0


def compute_dodge_command(
    dodge_side: float,
    *,
    dodge_speed: float = DEFAULT_DODGE_SPEED,
    dodge_burst: float = DEFAULT_DODGE_BURST_S,
) -> ApproachCommand:
    """장애물 회피 중 낼 한 번의 지령. 전진·회전은 멈추고 옆으로만 비킨다 —
    회피 중에 동시에 돌거나 전진하면 장애물에 더 가까워질 수 있어서다."""
    return ApproachCommand(0.0, dodge_speed * dodge_side, 0.0, dodge_burst, False)


def _normalize_deg(angle_rad: float) -> float:
    deg = math.degrees(angle_rad)
    return ((deg + 180.0) % 360.0) - 180.0


def min_range_in_arc(
    angle_min: float,
    angle_increment: float,
    ranges,
    center_deg: float,
    half_width_deg: float,
) -> float | None:
    """LaserScan의 `ranges`에서 `center_deg` ± `half_width_deg` 안의 유효한
    (finite, >0) 최소 거리를 돌려준다. 해당 구간에 유효한 반환이 하나도
    없으면 `None`(라이다 미기동/그 방향 반환 없음 — obstacle_ahead()가
    "모르면 이 기능 없던 것처럼" 처리)."""
    best = None
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0:
            continue
        deg = _normalize_deg(angle_min + i * angle_increment)
        diff = abs(((deg - center_deg + 180.0) % 360.0) - 180.0)
        if diff <= half_width_deg and (best is None or r < best):
            best = r
    return best
