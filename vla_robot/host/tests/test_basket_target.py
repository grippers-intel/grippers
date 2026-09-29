"""바구니 투입 목표 기하 — 배치도 REV.2(2026-09-29, 상자 1개)의 값과 맞춘다."""
import math

import pytest

from mission.basket_target import basket_target, facing_error_deg


@pytest.fixture
def basket(cfg):
    m = cfg.mission
    bx, by, _yaw = cfg.arena.boxes["basket"]
    return basket_target("basket", (bx, by), cfg.arena.box_size,
                         m.insert_half_width_m, m.insert_inset_depth_m)


def test_target_matches_the_drawing(basket):
    # 배치도: 상자 입구 y 1.480, 투입 목표 x[0.960, 1.020] y[1.480, 1.510]
    x0, x1, y0, y1 = basket.rect
    assert (x0, x1) == pytest.approx((0.960, 1.020))
    assert (y0, y1) == pytest.approx((1.480, 1.510))
    assert basket.center == pytest.approx((0.990, 1.495))


def test_heading_is_90_from_straight_in_front(basket):
    """상자 정면에서는 목표 중심 방위각이 90° 다 — 팔이 틀 각도가 0 이다."""
    cx, cy = basket.center
    assert basket.heading_deg((cx, cy - 0.3)) == pytest.approx(90.0)
    assert facing_error_deg(basket, (cx, cy - 0.3), 90.0) == pytest.approx(0.0)


def test_residual_angle_is_what_the_arm_must_turn(basket):
    """정차점이 옆으로 밀리면 그만큼 팔이 틀어야 한다. 0.30 m 앞에서 0.05 m 옆 = 9.5°."""
    cx, cy = basket.center
    # 목표의 **왼쪽**에 서 있으면 목표는 내 오른쪽 — 시계방향(-)으로 틀어야 한다
    left_of_target = facing_error_deg(basket, (cx - 0.05, cy - 0.30), 90.0)
    assert left_of_target == pytest.approx(-9.46, abs=0.05)
    assert abs(left_of_target) < 15.0                     # 팔 한계 안
    # 오른쪽에 서면 부호가 뒤집힌다
    right_of_target = facing_error_deg(basket, (cx + 0.05, cy - 0.30), 90.0)
    assert right_of_target == pytest.approx(+9.46, abs=0.05)


def test_residual_beyond_the_arm_limit(basket):
    """0.20 m 옆으로 서면 34° — 팔로 못 메운다. 이때만 차체를 돌린다."""
    cx, cy = basket.center
    residual = facing_error_deg(basket, (cx + 0.20, cy - 0.30), 90.0)
    assert residual == pytest.approx(33.7, abs=0.2)
    assert abs(residual) > 15.0


def test_arc_points_still_match_the_layout_table(cfg):
    """도면의 "호의 주요 점" 표 — 호 판정은 걷어냈지만 목표 중심은 그대로여야 한다."""
    m = cfg.mission
    for box, want_center, south in (("basket", (0.990, 1.495), (0.990, 1.345)),):
        bx, by, _yaw = cfg.arena.boxes[box]
        t = basket_target(box, (bx, by), cfg.arena.box_size,
                          m.insert_half_width_m, m.insert_inset_depth_m)
        assert t.center == pytest.approx(want_center)
        assert (t.center[0], t.center[1] - 0.15) == pytest.approx(south)
