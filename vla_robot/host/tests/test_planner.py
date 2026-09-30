import math

import pytest

from planning.planner import (DriveMode, DriveSequencer, GridPathPlanner, ObstacleHold, point_in_rect,
                              segment_circle_clearance, segment_hits_rect)


def _planner(cfg):
    return GridPathPlanner(cfg.planner, cfg.arena, arrive_tol=0.35)


def test_straight_path_without_obstacles(cfg):
    p = _planner(cfg)
    sub, corner, blocked = p.update((0.9, 0.45), (0.9, 1.2), [])
    assert blocked is None and corner is None
    assert len(p.last_path) == 2
    assert abs(sub[0] - 0.9) < 0.03 and sub[1] > 0.8


def test_detours_around_obstacle_with_clearance(cfg):
    p = _planner(cfg)
    obstacle = (0.9, 0.8)
    _sub, _corner, blocked = p.update((0.9, 0.45), (0.9, 1.2), [obstacle])
    assert blocked == "piece"
    path = p.last_path
    assert len(path) > 2
    for a, b in zip(path, path[1:]):
        assert segment_circle_clearance(a, b, obstacle)[0] >= p.safe - 1e-6


def test_reports_blocked_when_wall_of_obstacles(cfg):
    p = _planner(cfg)
    wall = [(x / 10.0, 0.75) for x in range(0, 19)]
    sub, _corner, blocked = p.update((0.9, 0.40), (0.9, 1.25), wall)
    assert blocked == "blocked"


def test_never_parks_on_goal_cell_outside_trigger(cfg):
    # 시뮬 재현: 로봇이 도착 칸 중심 근처(목표에서 0.362 m)에 서서 STOP 만 영원히 나갔다.
    p = _planner(cfg)
    robot, target = (0.745, 1.09), (0.45, 1.30)
    sub, _corner, blocked = p.update(robot, target, [(0.55, 0.95)])
    assert blocked != "blocked"
    assert math.hypot(sub[0] - robot[0], sub[1] - robot[1]) > cfg.planner.axis_leg_tolerance_m


def test_sequencer_rotates_then_drives(cfg):
    s = DriveSequencer(cfg.planner)
    assert s.update((0.9, 0.5), 0.0, (0.9, 1.2)).mode == DriveMode.ROTATE      # 90° 틀어짐
    s.reset()
    assert s.update((0.9, 0.5), 90.0, (0.9, 1.2)).mode == DriveMode.FORWARD


def test_sequencer_hysteresis(cfg):
    s = DriveSequencer(cfg.planner)
    assert s.update((0.9, 0.5), 90.0, (0.9, 1.2)).mode == DriveMode.FORWARD
    # 8° 틀어짐: 나올 때 문턱(5°)은 넘지만 들어갈 때 문턱(12°) 안 — 계속 직진
    assert s.update((0.9, 0.5), 82.0, (0.9, 1.2)).mode == DriveMode.FORWARD
    # 15°: 이번 사이클은 FORWARD 를 내고 다음은 STOP, 그다음 ROTATE
    assert s.update((0.9, 0.5), 75.0, (0.9, 1.2)).mode == DriveMode.FORWARD
    assert s.update((0.9, 0.5), 75.0, (0.9, 1.2)).mode == DriveMode.STOP
    assert s.update((0.9, 0.5), 75.0, (0.9, 1.2)).mode == DriveMode.ROTATE


def test_sequencer_stops_when_subgoal_is_robot(cfg):
    # 막혀서 부분목표가 로봇 자신이면 전진하면 안 된다(이전 구현의 버그).
    s = DriveSequencer(cfg.planner)
    assert s.update((0.9, 0.5), 90.0, (0.9, 0.5)).mode == DriveMode.STOP


def test_obstacle_hold_keeps_flickering_obstacle(cfg):
    h = ObstacleHold(3, 0.06)
    assert h.update([(1.0, 1.0)]) == [(1.0, 1.0)]
    assert h.update([]) == [(1.0, 1.0)]
    assert h.update([]) == [(1.0, 1.0)]
    assert h.update([]) == []
    assert math.isclose(h.update([(1.02, 1.0)])[0][0], 1.02)


# ---------------------------------------------------------------------------
# 차체 모양으로 피하기 — 직진은 반폭, 꺾는 점은 회전 반지름
# ---------------------------------------------------------------------------
def test_clearances_come_from_the_body_size(cfg):
    p = _planner(cfg)
    c = cfg.planner
    pad = c.piece_obstacle_radius_m + c.obstacle_margin_m
    assert p.safe == pytest.approx(c.robot_width_m / 2 + pad)
    assert p.turn_safe == pytest.approx(math.hypot(c.robot_width_m / 2, c.robot_length_m / 2) + pad)
    assert p.turn_safe > p.safe


def test_drives_straight_between_two_pieces_the_body_fits_through(cfg):
    """2026-09-30 실기: 한 원(0.25)으로 보면 막히던 틈을, 직진이면 차체가 지나간다."""
    p = _planner(cfg)
    gap = 2 * p.safe + 0.10                 # 차체 옆면이 양쪽 기물에서 5 cm 씩 남는 간격
    left, right = (0.9 - gap / 2, 0.85), (0.9 + gap / 2, 0.85)
    _sub, corner, blocked = p.update((0.9, 0.45), (0.9, 1.25), [left, right])
    assert blocked is None and corner is None
    assert len(p.last_path) == 2            # 꺾지 않고 곧장
    assert gap < 2 * p.turn_safe            # 회전 여유로 막았다면 못 지나갔을 틈이다


def test_corners_keep_the_turning_clearance(cfg):
    """돌아가야 하는 배치에서 꺾이는 점은 모든 기물에서 회전 여유 밖, 직선 구간은 직진 여유 밖."""
    p = _planner(cfg)
    obstacles = [(0.95, 0.80), (1.20, 0.95), (0.70, 1.05), (1.45, 0.70)]
    robot, target = (0.60, 0.40), (1.40, 1.25)
    _sub, _corner, blocked = p.update(robot, target, obstacles)
    assert blocked != "blocked"
    path = p.last_path
    assert len(path) > 2, "이 배치는 돌아가야 한다"
    for v in path[1:]:
        for o in obstacles:
            assert math.dist(v, o) >= p.turn_safe - 1e-6
    for a, b in zip(path, path[1:]):
        for o in obstacles:
            assert segment_circle_clearance(a, b, o)[0] >= p.safe - 1e-6


# ---------------------------------------------------------------------------
# 상자 금지 구역 — 주행 구역을 장판 전체로 넓히면서 생겼다(배치도 REV.2)
# ---------------------------------------------------------------------------
def _keepout(p):
    assert len(p.keepouts) == 1
    return p.keepouts[0]


def test_keepout_matches_the_stop_point(cfg):
    """금지 구역의 앞쪽 경계가 곧 정차점이다 — 그래야 정차점에 설 수 있다."""
    p = _planner(cfg)
    x0, x1, y0, y1 = _keepout(p)
    bx, by, _yaw = cfg.arena.boxes["basket"]
    assert y0 == pytest.approx(by - cfg.arena.box_size[1] / 2.0 - cfg.mission.box_approach_margin_m)
    assert (x0, x1) == pytest.approx((bx - 0.105 - 0.20, bx + 0.105 + 0.20))
    assert not point_in_rect((bx, y0), (x0, x1, y0, y1))


def test_path_goes_around_the_box(cfg):
    """상자 좌우를 잇는 직선은 상자를 지난다 — 계획기는 상자 앞으로 돌아가야 한다."""
    p = _planner(cfg)
    rect = _keepout(p)
    _sub, _corner, blocked = p.update((0.45, 1.50), (1.60, 1.50), [])
    assert blocked != "blocked"
    path = p.last_path
    assert len(path) > 2
    for a, b in zip(path, path[1:]):
        assert not segment_hits_rect(a, b, rect)


def test_stop_point_is_reachable_from_the_front(cfg):
    p = _planner(cfg)
    rect = _keepout(p)
    _sub, _corner, blocked = p.update((0.990, 0.60), (0.990, rect[2]), [])
    assert blocked is None
    for a, b in zip(p.last_path, p.last_path[1:]):
        assert not segment_hits_rect(a, b, rect)


def test_robot_that_stopped_too_close_can_still_leave(cfg):
    """허용치만큼 더 붙어 서면 금지 구역 안이다 — 막으면 빠져나오지도 못한다."""
    p = _planner(cfg)
    x0, x1, y0, y1 = _keepout(p)
    robot = (0.990, y0 + 0.02)
    assert point_in_rect(robot, (x0, x1, y0, y1))
    _sub, _corner, blocked = p.update(robot, (0.60, 0.80), [])
    assert blocked != "blocked"
