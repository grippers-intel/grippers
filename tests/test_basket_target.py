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


def test_toy_바구니는_x_시프트가_반영된_목표영역을_쓴다():
    bx, by, _ = box_pose("toy")
    shifted_x = bx - mcfg.TOY_DEST_X_SHIFT_LEFT_M
    edge_y = by - cfg.BOX_L / 2.0
    robot_xy = (shifted_x, edge_y - 0.10)
    result = bt.check_basket_insert_gate(robot_xy, robot_yaw_deg=90.0, box_name="toy")
    assert result.ok, result.reason
    # 시프트가 실제로 반영됐다는 것을 대조로 확인 — 원래(시프트 안 된)
    # 중심 기준으로는 이 x가 목표 영역 절반폭(5cm) 밖으로 밀려난다.
    assert abs(shifted_x - bx) > bt.TARGET_HALF_WIDTH_M


def test_목표영역_안에_있으면_지향_상관없이_통과():
    ex, ey = _edge_center("chess")
    robot_xy = (ex, ey + 0.01)   # 안쪽 1cm 지점, 영역 안
    result = bt.check_basket_insert_gate(robot_xy, robot_yaw_deg=123.0, box_name="chess")
    assert result.distance_m < 1e-6
    assert result.ok
