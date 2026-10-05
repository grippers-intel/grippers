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
    pmap = {"soccer": [(0.9, 0.81)], "knight": [(0.9, 0.43)], "rook": [(0.4, 0.62)]}
    cmds = _first_moves(fsm, P(0.9, 0.62), pmap)
    moving = [c for c in cmds if not c.stop]
    assert moving and moving[0].angular_z != 0 and moving[0].linear_x == 0
    assert any("물러날 자리가 없어" in e for e in fsm.events)


def _cfg():
    from host_config import load_host_config
    return load_host_config(None)
