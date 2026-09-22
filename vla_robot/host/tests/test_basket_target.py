"""바구니 투입 목표 기하 — 도면(docs/workshop_floorplan.html, 2026-09-05)의 값과 맞춘다."""
import math

import pytest

from mission.basket_target import (approach_ready, basket_target, crossed_arc,
                                   in_approach_sector)

R = 0.15          # mission.max_approach_dist_m
SECTOR = 120.0    # mission.approach_sector_deg


@pytest.fixture
def chess(cfg):
    m = cfg.mission
    bx, by, _yaw = cfg.arena.boxes["chess"]
    return basket_target("chess", (bx, by), cfg.arena.box_size,
                         m.insert_half_width_m, m.insert_inset_depth_m)


def test_target_matches_the_drawing(chess):
    # 도면: INSERT x[1.320, 1.380] y[1.450, 1.480]
    x0, x1, y0, y1 = chess.rect
    assert (x0, x1) == pytest.approx((1.320, 1.380))
    assert (y0, y1) == pytest.approx((1.450, 1.480))
    assert chess.center == pytest.approx((1.350, 1.465))


def test_heading_is_30_90_150_on_the_arc(chess):
    """호 위 진입 지점마다 정렬각이 다르다 — 좌 30° · 중앙 90° · 우 150°."""
    cx, cy = chess.center
    for entry_deg, want in ((-150.0, 30.0), (-90.0, 90.0), (-30.0, 150.0)):
        rad = math.radians(entry_deg)
        p = (cx + R * math.cos(rad), cy + R * math.sin(rad))
        assert chess.heading_deg(p) == pytest.approx(want, abs=1e-6)
        assert in_approach_sector(chess, p, SECTOR)


def test_sector_rejects_side_and_back(chess):
    cx, cy = chess.center
    assert not in_approach_sector(chess, (cx + R, cy), SECTOR)        # 정측면(0°)
    assert not in_approach_sector(chess, (cx, cy + R), SECTOR)        # 상자 뒤(90°)


def test_arc_and_approval_radius(chess):
    cx, cy = chess.center
    on_arc = (cx, cy - R)
    assert crossed_arc(chess, on_arc, R, SECTOR)
    assert not crossed_arc(chess, (cx, cy - R - 0.01), R, SECTOR)
    # dest_xy(1.350, 1.300)는 호 밖 1.5 cm — 사각형 기준 승인 반경에는 걸친다
    dest = (1.350, 1.300)
    assert chess.distance(dest) == pytest.approx(0.165)
    assert not crossed_arc(chess, dest, R, SECTOR)
    assert approach_ready(chess, dest, R, SECTOR)


def test_side_entry_needs_a_body_turn_not_just_the_arm(chess):
    """호 왼쪽 끝 진입은 90°에서 60° 벗어난다 — 팔의 ±15°로는 못 메운다."""
    cx, cy = chess.center
    rad = math.radians(-150.0)
    p = (cx + R * math.cos(rad), cy + R * math.sin(rad))
    assert abs(chess.heading_deg(p) - 90.0) == pytest.approx(60.0)
