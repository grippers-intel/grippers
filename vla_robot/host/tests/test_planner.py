import math

from planning.planner import DriveMode, DriveSequencer, GridPathPlanner, ObstacleHold, segment_circle_clearance


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
