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


def test_whole_zone_blocked_halts_and_names_the_piece():
    cx, cy = 0.99, 1.26                                                         # 바구니 정차점(가운데)
    fsm, _ = _carrying({"rook": [(cx - 0.06, cy + 0.12)], "knight": [(cx + 0.06, cy + 0.12)]})
    assert fsm.state == HostState.HALTED
    assert "정차 구역" in fsm.halt_reason and "치워" in fsm.halt_reason
