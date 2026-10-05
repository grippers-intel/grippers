"""제자리 회전 전 주변 확인(2026-10-05): 회전 반경 안의 기물에서 앞뒤로 물러난 뒤 돈다."""
from localization.pose import Pose
from mission.host_fsm import HostState, MissionFSM, Order
from vla_common.protocol import PiStatus, State


def P(x, y, yaw=90.0):
    return Pose(x, y, yaw, ok=True, n_cams=2, fresh=True)


def S():
    return PiStatus(boot_id="B", state=State.IDLE, busy=False, job_id=0, result=None,
                    base_ok=True, watchdog=False)


def _first_moves(fsm, pose, pmap, n=5):
    cmds = []
    for k in range(n):
        cmds.append(fsm.step(pose, pmap, S(), 0.1 * k))
    return cmds


def test_backs_off_from_a_ball_in_front_before_turning_away():
    """공을 놓친 뒤처럼 19 cm 앞에 공이 있고 왼쪽 룩으로 가야 한다 — 그 자리에서 돌지 않고 먼저 후진."""
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("rook",)))
    pmap = {"soccer": [(0.9, 0.81)], "rook": [(0.4, 0.62)]}
    cmds = _first_moves(fsm, P(0.9, 0.62), pmap)
    assert fsm.target_label == "rook"
    moving = [c for c in cmds if not c.stop]
    assert moving and moving[0].linear_x < 0 and moving[0].angular_z == 0
    assert any("turn guard" in e and "후진" in e for e in fsm.events)


def test_turns_normally_when_nothing_is_within_the_sweep():
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("rook",)))
    pmap = {"soccer": [(0.9, 1.0)], "rook": [(0.4, 0.62)]}       # 공이 38 cm 앞 — 회전 반경 밖
    cmds = _first_moves(fsm, P(0.9, 0.62), pmap)
    moving = [c for c in cmds if not c.stop]
    assert moving and moving[0].angular_z != 0
    assert not any("turn guard" in e for e in fsm.events)


def test_no_room_to_back_off_turns_anyway_and_says_so():
    """앞(공)·뒤(나이트)가 모두 막혔으면 예전처럼 돈다 — 멈춰 서지 않는다."""
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("rook",)))
    pmap = {"soccer": [(0.9, 0.81)], "knight": [(0.9, 0.45)], "rook": [(0.4, 0.62)]}
    cmds = _first_moves(fsm, P(0.9, 0.62), pmap)
    moving = [c for c in cmds if not c.stop]
    assert moving and moving[0].angular_z != 0 and moving[0].linear_x == 0
    assert any("물러날 자리가 없어" in e for e in fsm.events)


def _cfg():
    from host_config import load_host_config
    return load_host_config(None)


def test_a_small_turn_next_to_a_piece_does_not_back_off():
    """10-05 "후진이 너무 많다": 5° 맞추기는 옆 기물을 쓸지 않는다 — 한 바퀴 원으로 보지 않는다."""
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("queen",)))
    # 퀸은 정면에서 10° 오른쪽 0.28 m, 룩은 왼쪽 옆 18 cm(한 바퀴 원 안, 10° 회전 영역 밖)
    import math
    q = (0.9 + 0.28 * math.sin(math.radians(10)), 0.62 + 0.28 * math.cos(math.radians(10)))
    pmap = {"queen": [q], "rook": [(0.72, 0.62)]}
    cmds = _first_moves(fsm, P(0.9, 0.62), pmap, n=3)
    moving = [c for c in cmds if not c.stop]
    assert moving and moving[0].linear_x == 0 and moving[0].angular_z < 0
    assert not any("turn guard" in e for e in fsm.events)


def test_straight_basket_approach_waits_for_a_clear_line():
    """10-05: 정차점 0.35 m 안에서 곧장 가는 단계로 넘겼다가 직선 옆 나이트를 쳤다 — 직선이 비어야 넘긴다."""
    import math
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("queen",)))
    fsm.step(P(1.0, 0.5), {"queen": [(1.0, 0.9)]}, S(), 0.0)  # 목표를 고르면 바구니 정차점이 정해진다
    dest = fsm.dest_xy
    fsm.state = HostState.CARRY_TO_DEST
    start = (dest[0], dest[1] - 0.30)                         # 정차점 30 cm 앞, 정차점을 본다
    knight = (dest[0] + 0.06, dest[1] - 0.15)                 # 직선 바로 옆 6 cm
    held = (start[0], start[1] + 0.26)                        # 쥔 퀸(그리퍼 안)
    fsm.step(P(*start), {"queen": [held], "knight": [knight]}, S(), 0.1)
    assert fsm.state == HostState.CARRY_TO_DEST               # 곧장 가지 않는다
    fsm.step(P(*start), {"queen": [held], "knight": [(dest[0] + 0.40, dest[1] - 0.15)]}, S(), 0.2)
    assert fsm.state in (HostState.NUDGE_BOX, HostState.PLACE)  # 비었으면 넘긴다
    assert math.dist(start, dest) <= fsm.cfg.mission.place_trigger_dist_m
