"""바구니 입구 목표 영역 게이트 (사용자 지시, 2026-09-04).

말씀하신 조건을 그대로 수치로 확인한다: 목표 영역(가로 ±3cm, 안쪽 0~3cm —
처음엔 ±5cm였다가 같은 날 저녁 ±3cm로 좁혔다)에서 15cm 이내 + 그 영역을
바라보고 있으면 ok. 정면 접근뿐 아니라 45도·30도 사선(이번 세션에 라이다로
실측 확인한 각도들)도 통과해야 한다."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

import basket_target as bt         # noqa: E402
import config as cfg               # noqa: E402
import mission_config as mcfg      # noqa: E402
from localizer import box_pose     # noqa: E402


def _edge_center(box_name: str) -> tuple[float, float]:
    bx, by, _ = box_pose(box_name)
    if box_name == "toy":
        bx -= mcfg.TOY_DEST_X_SHIFT_LEFT_M
    edge_y = by - cfg.BOX_L / 2.0
    if box_name == "chess":
        edge_y += mcfg.CHESS_APPROACH_EXTRA_DEPTH_M
    return bx, edge_y


def test_정면_목표영역_안쪽에서_바라보면_통과():
    ex, ey = _edge_center("chess")
    # 목표 영역 바로 앞, 12cm 떨어져 정면(world 90도)으로 바라본다.
    robot_xy = (ex, ey - 0.12)
    result = bt.check_basket_insert_gate(robot_xy, robot_yaw_deg=90.0, box_name="chess")
    assert result.ok, result.reason
    assert result.distance_m == pytest.approx(0.12, abs=1e-3)


def test_15cm_넘게_멀면_실패():
    ex, ey = _edge_center("chess")
    robot_xy = (ex, ey - 0.30)   # 목표 영역 앞쪽으로 30cm
    result = bt.check_basket_insert_gate(robot_xy, robot_yaw_deg=90.0, box_name="chess")
    assert not result.ok
    assert "mm >" in result.reason


def test_45도_사선에서도_바라보면_통과():
    """이번 세션 라이다 실측(45도 사선에서도 모서리를 잔차 1.5~1.7mm로
    분간)이 근거 — Host 게이트도 이 각도를 받아 줘야 사선 진입 허용이라는
    취지가 산다."""
    ex, ey = _edge_center("chess")
    dist = 0.12
    robot_xy = (ex - dist * math.sin(math.radians(45.0)),
                ey - dist * math.cos(math.radians(45.0)))
    dx, dy = ex - robot_xy[0], ey - robot_xy[1]
    yaw_deg = math.degrees(math.atan2(dy, dx))   # 목표 중심을 정확히 바라본다
    result = bt.check_basket_insert_gate(robot_xy, robot_yaw_deg=yaw_deg, box_name="chess")
    assert result.ok, result.reason


def test_30도_사선도_통과():
    ex, ey = _edge_center("chess")
    dist = 0.12
    robot_xy = (ex - dist * math.sin(math.radians(30.0)),
                ey - dist * math.cos(math.radians(30.0)))
    dx, dy = ex - robot_xy[0], ey - robot_xy[1]
    yaw_deg = math.degrees(math.atan2(dy, dx))
    result = bt.check_basket_insert_gate(robot_xy, robot_yaw_deg=yaw_deg, box_name="chess")
    assert result.ok, result.reason


def test_가까워도_다른_방향을_보면_실패():
    ex, ey = _edge_center("chess")
    robot_xy = (ex, ey - 0.10)
    # 목표 쪽(90도)이 아니라 완전히 반대(-90도, 뒤돌아섬)를 본다.
    result = bt.check_basket_insert_gate(robot_xy, robot_yaw_deg=-90.0, box_name="chess")
    assert not result.ok
    assert "안 보고" in result.reason


def test_toy_바구니_x_시프트는_2026_09_05_사용자_지시로_0이다():
    """2026-09-03 실기로 검증됐던 좌측 7cm 보정을 2026-09-05 사용자 지시로
    되돌렸다(mission_config.TOY_DEST_X_SHIFT_LEFT_M 주석 참고) — toy도
    이제 chess와 같은 방식(시프트 없음)으로 목표영역을 잡아야 한다."""
    assert mcfg.TOY_DEST_X_SHIFT_LEFT_M == 0.0

    bx, by, _ = box_pose("toy")
    edge_y = by - cfg.BOX_L / 2.0
    robot_xy = (bx, edge_y - 0.10)
    result = bt.check_basket_insert_gate(robot_xy, robot_yaw_deg=90.0, box_name="toy")
    assert result.ok, result.reason
    # 시프트가 없으니 상자 "실측 중심" x 그대로가 목표영역 중심이어야 한다.
    x_lo, x_hi, _y_lo, _y_hi = bt.target_rect("toy")
    assert (x_lo + x_hi) / 2.0 == pytest.approx(bx)


def test_목표영역_안에_있으면_지향_상관없이_통과():
    ex, ey = _edge_center("chess")
    robot_xy = (ex, ey + 0.01)   # 안쪽 1cm 지점, 영역 안
    result = bt.check_basket_insert_gate(robot_xy, robot_yaw_deg=123.0, box_name="chess")
    assert result.distance_m < 1e-6
    assert result.ok


def test_목표영역_폭은_물리_바구니_폭_안에서만_넓힌다():
    """2026-09-05 밤 사용자 지시 — "servo1을 안돌려도 되도록 바구니 내
    투하영역을 키우자". 목표영역이 넓을수록 PLACE가 매 사이클 재는
    facing_error_deg(=safe_300 servo1 보정각)가 작아진다 — 그런데 물리
    바구니 폭(config.BOX_W)을 넘기면 벽에 닿는다. 벽까지 여유가 남는지
    수치로 고정해 둔다(이력 전체가 실기 사고로 오르내린 값이라 회귀
    방지용)."""
    assert bt.TARGET_HALF_WIDTH_M == pytest.approx(0.065)
    assert bt.TARGET_INSET_DEPTH_M == pytest.approx(0.06)
    margin_each_side = cfg.BOX_W / 2.0 - bt.TARGET_HALF_WIDTH_M
    assert margin_each_side > 0.03, (
        "목표영역이 물리 바구니 폭에 비해 너무 넓다 — 벽에 닿을 여유가 3cm도 안 된다")


def test_무회전_영역은_넓은_목표영역보다_확실히_좁다():
    """2026-09-05 밤 사용자 지시 — "servo1이 안 돌아도 되는 영역을 따로
    만들자". 이 좁은 영역이 위 넓은 목표영역(TARGET_HALF_WIDTH_M/
    TARGET_INSET_DEPTH_M)보다 넓어지면, "이미 괜찮으면 안 돌린다"는
    취지가 무너지고 사실상 항상 안 돌리는 쪽으로 퇴화한다."""
    assert bt.NO_ROTATION_HALF_WIDTH_M == pytest.approx(0.03)
    assert bt.NO_ROTATION_INSET_DEPTH_M == pytest.approx(0.03)
    assert bt.NO_ROTATION_HALF_WIDTH_M < bt.TARGET_HALF_WIDTH_M
    assert bt.NO_ROTATION_INSET_DEPTH_M < bt.TARGET_INSET_DEPTH_M


def test_무회전_영역을_이미_보고_있으면_지향_오차가_있어도_통과한다():
    """정면(90도)에서 8도 벗어난 자세 — 넓은 목표영역 기준 지향오차는
    0이 아니지만(그래서 이전엔 servo1을 조금이라도 돌렸다), 무회전
    영역(NO_ROTATION_FACING_TOLERANCE_DEG=8도) 안이라 그대로 둬도
    된다고 판정해야 한다."""
    ex, ey = _edge_center("chess")
    robot_xy = (ex, ey - 0.15)
    result = bt.check_no_rotation_zone(robot_xy, robot_yaw_deg=82.0, box_name="chess")
    assert result.ok, result.reason


def test_무회전_영역_밖이면_그대로_통과하지_않는다():
    """정면에서 30도나 벗어나면 무회전 영역으로도 통과시키면 안 된다 —
    그 정도 오차는 여전히 servo1 보정이 필요하다."""
    ex, ey = _edge_center("chess")
    robot_xy = (ex, ey - 0.15)
    result = bt.check_no_rotation_zone(robot_xy, robot_yaw_deg=60.0, box_name="chess")
    assert not result.ok
