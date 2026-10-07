"""바구니(손) 앞 곧장 가는 구간은 직선이 비었을 때만 (2026-10-05).

상자 앞 맞추기는 계획기 없이 정차점까지 곧장 간다. 0.35 m 에서 넘겼다가 직선 옆 나이트를 쳤다.
(그때 넣었던 "돌기 전 후진으로 물러나기"는 10-07 사용자 결정으로 뺐다 — 직선으로 한 번에 가고
도착해서 제자리 회전으로만 맞춘다.)
"""
from localization.pose import Pose
from mission.host_fsm import HostState, MissionFSM, Order
from vla_common.protocol import PiStatus, State


def P(x, y, yaw=90.0):
    return Pose(x, y, yaw, ok=True, n_cams=2, fresh=True)


def S():
    return PiStatus(boot_id="B", state=State.IDLE, busy=False, job_id=0, result=None,
                    base_ok=True, watchdog=False)


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


def _cfg():
    from host_config import load_host_config
    return load_host_config(None)


def _carry_to_basket(knight):
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("queen",)))
    fsm.step(P(1.0, 0.5), {"queen": [(1.0, 0.9)]}, S(), 0.0)
    fsm.state = HostState.CARRY_TO_DEST
    return fsm, fsm.dest_xy


def test_at_the_stop_point_hands_over_even_if_a_piece_is_close():
    """10-07 실기: 정차점 2 cm 안에 서 있는데 정차점이 나이트에서 13 cm 라 "직선 막힘"으로 영원히 서 있었다."""
    fsm, dest = _carry_to_basket(None)
    knight = (dest[0] + 0.06, dest[1] + 0.12)
    at = (dest[0] - 0.016, dest[1] - 0.011)
    held = (at[0], at[1] + 0.26)
    fsm.step(P(*at), {"queen": [held], "knight": [knight]}, S(), 0.1)
    assert fsm.state != HostState.CARRY_TO_DEST


def test_blocked_line_near_the_stop_halts_and_names_the_piece():
    fsm, dest = _carry_to_basket(None)
    start = (dest[0], dest[1] - 0.30)
    held = (start[0], start[1] + 0.26)
    knight = (dest[0] + 0.06, dest[1] - 0.15)                 # 직선 바로 옆
    fsm._stall.update = lambda *a, **k: False                 # 시험 로봇은 안 움직인다 — 무응답 감지는 뺀다
    t = 0.1
    while fsm.state == HostState.CARRY_TO_DEST and t < 10.0:
        fsm.step(P(*start), {"queen": [held], "knight": [knight]}, S(), t)
        t += 0.1
    assert fsm.state == HostState.HALTED
    assert "knight" in fsm.halt_reason and "치워" in fsm.halt_reason
