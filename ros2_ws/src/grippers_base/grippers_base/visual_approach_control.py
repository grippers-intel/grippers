"""물체 접근 시각 서보 루프의 순수 제어 수학 (tools/perception/approach.py 이식).

⚠️ 2026-08-23: HANDOFF.md가 실기로 검증한 알고리즘을 그대로 옮긴 것이다 —
재유도하지 않는다. 원본과 다르게 튜닝하고 싶으면 먼저 tools/perception/
approach.py에서 실기로 확인하고 이 상수를 맞춰야 한다.

**두 개의 독립된 신호를 쓴다** (원본 모듈 docstring과 동일):
  거리 ← 검출 박스의 높이(px). 가까워질수록 커지고 좌우 이동과 무관하다.
  좌우 ← 박스 아래변 중점의 x(px).

카메라 구독·cmd_vel 발행·ROS2 액션 서버 배선은 이 파일에 없다 — 오차 →
속도 지령 변환만 담아 rclpy 없이 pytest로 검증한다(drive_control.py와
같은 이유). 실제 배선은 base_driver_node.py의 `approach_object` 액션
참고 — 그쪽은 카메라 관측이 있어야 확인되므로 아직 실기 미검증이다.
"""
from __future__ import annotations

from typing import NamedTuple

# tools/perception/approach.py의 argparse 기본값과 동일 — cycle.sh가 검증한
# 조합(offset-h=15, push=50)은 이 기본값 위에 오프셋만 얹은 것이다.
DEFAULT_TOL_X_PX = 8.0
DEFAULT_TOL_H_PX = 6.0
DEFAULT_GAIN_X = 0.0009
DEFAULT_GAIN_H = 0.0016
DEFAULT_MAX_SPEED = 0.08
DEFAULT_MIN_SPEED = 0.05
DEFAULT_BURST_S = 0.35
DEFAULT_MIN_BURST_S = 0.15
DEFAULT_ALIGN_FIRST = 2.0


class ApproachCommand(NamedTuple):
    linear_x: float
    linear_y: float
    burst_s: float
    arrived: bool


def compute_approach_error(obs_x: float, obs_h: float, target_x: float, target_h: float):
    """관측(obs_x=박스 아래변 중점 x, obs_h=박스 높이)과 교시 목표의 오차.

    err_x: +면 물체가 목표보다 오른쪽에 보인다.
    err_h: +면 아직 멀다(박스가 작다) — h는 거리에 반비례하지 않고 비례하므로
    부호가 직관과 반대다(목표 h − 관측 h)."""
    return obs_x - target_x, target_h - obs_h


def apply_floor(vx, vy, min_speed, max_speed, burst, min_burst):
    """데드밴드 아래 지령은 바퀴를 돌리지 못한다(실측: 0.017 m/s 에서 정지).

    속도 벡터를 통째로 키워 가장 작은 성분이 min_speed 에 닿게 하고, 같은 배율로
    시간을 줄인다. 방향과 이동 거리는 그대로 두고 '실제로 움직이는' 속도만 만든다."""
    mags = [abs(v) for v in (vx, vy) if v != 0.0]
    if not mags:
        return 0.0, 0.0, burst
    k = min(min_speed / min(mags), max_speed / max(mags))
    if k > 1.0:
        vx, vy, burst = vx * k, vy * k, max(min_burst, burst / k)
    # 상한에 걸려 아직도 데드밴드 아래인 성분은 버린다 — 모터만 울릴 뿐이다.
    if 0.0 < abs(vx) < min_speed:
        vx = 0.0
    if 0.0 < abs(vy) < min_speed:
        vy = 0.0
    return vx, vy, burst


def compute_approach_command(
    err_x: float,
    err_h: float,
    *,
    tol_x: float = DEFAULT_TOL_X_PX,
    tol_h: float = DEFAULT_TOL_H_PX,
    gain_x: float = DEFAULT_GAIN_X,
    gain_h: float = DEFAULT_GAIN_H,
    max_speed: float = DEFAULT_MAX_SPEED,
    min_speed: float = DEFAULT_MIN_SPEED,
    burst: float = DEFAULT_BURST_S,
    min_burst: float = DEFAULT_MIN_BURST_S,
    align_first: float = DEFAULT_ALIGN_FIRST,
    invert_y: bool = False,
) -> ApproachCommand:
    """오차 한 쌍을 다음 한 번의 `nudge(vx, vy, secs)` 지령으로 바꾼다.

    수렴(양쪽 오차가 허용치 안)이면 `arrived=True`와 함께 속도 0을 돌려준다 —
    호출자는 이때 정지 명령을 내고 루프를 끝낸다."""
    done_x, done_h = abs(err_x) <= tol_x, abs(err_h) <= tol_h
    if done_x and done_h:
        return ApproachCommand(0.0, 0.0, 0.0, True)

    sign_y = -1.0 if invert_y else 1.0
    vx = 0.0 if done_h else max(-max_speed, min(max_speed, err_h * gain_h))
    vy = 0.0 if done_x else max(-max_speed, min(max_speed, -err_x * gain_x * sign_y))

    # 좌우가 크게 어긋난 채로 전진하면 물체를 지나쳐버린다(실측 실패 사례 —
    # HANDOFF.md "수동으로 파지 위치 맞추기 — 실패"). 전진을 늦춰 횡보정이
    # 따라잡게 한다.
    if align_first > 0 and abs(err_x) > align_first * tol_x:
        vx *= 0.25

    vx, vy, secs = apply_floor(vx, vy, min_speed, max_speed, burst, min_burst)
    return ApproachCommand(vx, vy, secs, False)
