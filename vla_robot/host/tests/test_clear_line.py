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
