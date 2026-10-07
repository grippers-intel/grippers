import math
"""바구니 정차 구역(2026-10-07): y 는 정차점 그대로, x 는 가운데 ±8 cm 안에서 기물에 안 닿는 가장 가운데 자리."""
from localization.pose import Pose
from mission.host_fsm import HostState, MissionFSM, Order
from vla_common.protocol import PiStatus, State


def P(x, y, yaw=90.0):
    return Pose(x, y, yaw, ok=True, n_cams=2, fresh=True)


def S():
    return PiStatus(boot_id="B", state=State.IDLE, busy=False, job_id=0, result=None,
                    base_ok=True, watchdog=False)


def _cfg():
    from host_config import load_host_config
    return load_host_config(None)


def _carrying(extra):
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("queen",)))
    fsm.step(P(0.99, 0.50), {"queen": [(0.99, 0.80)]}, S(), 0.0)
    centre = fsm.dest_xy
    fsm._enter(HostState.CARRY_TO_DEST)
    pmap = {"queen": [(0.99, 0.82)], **extra}                                  # 쥔 퀸(그리퍼 안)
    fsm.step(P(0.99, 0.56), pmap, S(), 0.1)
    return fsm, centre


def test_centre_is_used_when_clear():
    fsm, centre = _carrying({})
    assert fsm.dest_xy == centre and fsm._aim_shift == 0.0
    assert not any("stop zone" in e for e in fsm.events)


def test_steps_aside_from_the_knight_by_the_basket():
    """10-07 실기: 나이트가 정차점 오른쪽 위 17 cm — 바퀴와 1 cm. 왼쪽으로 비켜 선다."""
    fsm, centre = _carrying({"knight": [(1.103, 1.395)]})
    assert fsm.state == HostState.CARRY_TO_DEST
    dx = fsm.dest_xy[0] - centre[0]
    assert fsm.dest_xy[1] == centre[1] and -0.08 - 1e-9 <= dx < 0               # y 그대로, 왼쪽으로
    assert abs(fsm._aim_shift - max(-0.06, dx)) < 1e-9                        # 겨누는 점도 같이(최대 6 cm)
    assert abs((fsm._basket().aim[0] - fsm._basket("basket").aim[0]) - fsm._aim_shift) < 1e-9
    assert any("stop zone" in e for e in fsm.events)


def test_whole_zone_blocked_halts_after_retrying_and_names_the_piece():
    cx, cy = 0.99, 1.26                                                         # 바구니 정차점(가운데)
    extra = {"rook": [(cx - 0.06, cy + 0.12)], "knight": [(cx + 0.06, cy + 0.12)]}
    fsm, _ = _carrying(extra)
    assert fsm.state == HostState.CARRY_TO_DEST                                 # 바로 멈추지 않는다
    fsm._stall.update = lambda *a, **k: False                                   # 시험 로봇은 안 움직인다
    pmap = {"queen": [(0.99, 0.82)], **extra}
    t = 0.2
    while fsm.state == HostState.CARRY_TO_DEST and t < 6.0:
        fsm.step(P(0.99, 0.56), pmap, S(), t)
        t += 0.1
    assert fsm.state == HostState.HALTED and t > 3.0                            # 3 s 다시 본 뒤
    assert "정차 구역" in fsm.halt_reason and "치워" in fsm.halt_reason and "(" in fsm.halt_reason


def test_a_brief_blocked_map_right_after_the_grasp_does_not_halt():
    """10-07: 잡은 순간 지도로 막혀 HALTED — 다음 사이클 지도로는 −4 cm 에 섰다."""
    cx, cy = 0.99, 1.26
    fsm, _ = _carrying({"rook": [(cx - 0.06, cy + 0.12)], "knight": [(cx + 0.06, cy + 0.12)]})
    assert fsm.state == HostState.CARRY_TO_DEST
    fsm.step(P(0.99, 0.56), {"queen": [(0.99, 0.82)], "knight": [(1.106, 1.307)]}, S(), 0.3)   # 룩 유령 사라짐
    assert fsm.state == HostState.CARRY_TO_DEST and fsm.dest_xy[0] < cx


def test_assumes_the_robot_stands_tilted_and_off_by_a_little():
    """10-07 실기: 0.95(−4 cm)를 골랐는데 0.964 · 79° 로 서서 나이트와 1.2 cm. 자리 검사는 ±15° · 좌우 ±2 cm 까지 본다."""
    fsm, centre = _carrying({"knight": [(1.106, 1.307)]})
    assert fsm.state == HostState.CARRY_TO_DEST
    dx = fsm.dest_xy[0] - centre[0]
    assert dx <= -0.06 + 1e-9                                                  # −6 cm 이상 비켜 선다


def test_from_the_side_at_stop_height_stands_on_the_robots_side():
    """정차점 높이 근처(옆)에서 잡았으면 구역에서 로봇 쪽 자리에 선다 — 옆으로 가는 거리·첫 회전을 줄인다(10-07)."""
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("queen",)))
    fsm.step(P(0.60, 1.00), {"queen": [(0.60, 1.20)]}, S(), 0.0)
    centre = fsm.dest_xy
    fsm._enter(HostState.CARRY_TO_DEST)
    fsm.step(P(0.70, 1.22, yaw=150.0), {"queen": [(0.48, 1.30)]}, S(), 0.1)   # 정차점 높이, 왼쪽
    assert fsm.dest_xy[1] == centre[1] and fsm.dest_xy[0] < centre[0] - 0.09   # 왼쪽 끝(−10 cm)


def _leaving_fsm(pose, target=(1.20, 0.90)):
    """바구니에 넣고 다음 기물(target)로 떠나는 순간."""
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("queen",)))
    fsm.step(pose, {"queen": [target], "knight": [(1.106, 1.307)]}, S(), 0.0)
    return fsm


def test_leaving_the_basket_never_drives_forward_into_it():
    """10-07 6기물: 상자를 넣고 공으로 떠나며 앞으로 빠져나가기가 바구니 쪽으로 10.7 cm — 앞면이 바구니를 넘었다."""
    pose = P(0.927, 1.262, yaw=98.5)
    fsm = _leaving_fsm(pose, target=(0.53, 1.15))
    for k in range(5):
        cmd = fsm.step(pose, {"queen": [(0.53, 1.15)], "knight": [(1.106, 1.307)]}, S(), 0.1 * (k + 1))
        assert not fsm.last_cmd_text.startswith("exit forward"), fsm.last_cmd_text
        assert cmd.linear_x == 0.0


def test_forward_exit_stops_short_of_the_basket_front():
    fsm = MissionFSM(_cfg())
    fwd = (0.0, 1.0)
    assert fsm._front_hits_basket((0.99, 1.36), fwd)          # 앞면 1.485 > 바구니 1.48 − 2 cm
    assert not fsm._front_hits_basket((0.99, 1.30), fwd)      # 앞면 1.425


def test_stop_zone_prefers_a_spot_where_it_can_turn_away():
    """나이트 옆에서 넣고 떠날 때 그 자리에서 돌 수 있는 자리(모서리 반경 밖)를 먼저 고른다."""
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("queen",)))
    fsm.step(P(0.60, 1.00), {"queen": [(0.60, 1.20)], "knight": [(1.106, 1.307)]}, S(), 0.0)
    fsm._enter(HostState.CARRY_TO_DEST)
    fsm.step(P(0.90, 0.80, yaw=90.0), {"queen": [(0.90, 1.06)], "knight": [(1.106, 1.307)]}, S(), 0.1)
    c = fsm.cfg.planner
    free_r = math.hypot(c.robot_length_m / 2, c.robot_width_m / 2) + c.piece_obstacle_radius_m \
        + fsm.cfg.mission.basket_stop_clear_m
    assert math.dist(fsm.dest_xy, (1.106, 1.307)) >= free_r


def test_near_waypoint_does_not_drive_straight_in_the_wrong_direction():
    """10-07 별: 경로 다음 점이 4 cm 라 방향을 안 재고 보던 쪽(바구니 반대)으로 5 cm 직진했다."""
    fsm = MissionFSM(_cfg())
    fsm._now = 1.0
    pose = P(0.635, 1.062, yaw=-143.0)
    cmd = fsm._drive_to(pose, (0.93, 0.96), [(1.106, 1.307)])
    assert cmd.linear_x == 0.0                                # 직진 아님(서거나 돈다)


def test_pre_stop_is_never_next_to_a_piece():
    """10-07: 구역 −10 cm 아래 30 cm 점이 퀸에서 5 cm — 계획기가 그 점으로 가며 퀸을 7.7 cm 밀었다."""
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("rook",)))
    pm = {"rook": [(1.20, 0.86)], "queen": [(0.90, 0.91)], "knight": [(1.106, 1.307)]}
    fsm.step(P(1.19, 0.58, yaw=90.0), pm, S(), 0.0)
    fsm._enter(HostState.CARRY_TO_DEST)
    pm["rook"] = [(1.19, 0.86)]
    fsm.step(P(1.19, 0.58, yaw=131.0), pm, S(), 0.1)
    goal = fsm.nav_goal
    assert goal is not None
    assert math.dist(goal, (0.90, 0.91)) >= fsm._planner.safe
