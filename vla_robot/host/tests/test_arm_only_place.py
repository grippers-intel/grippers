"""바구니 정차점에서 몸을 돌리면 옆 기물에 닿을 때(2026-10-07 나이트): 팔로만 넣거나, 너무 모자라면 다시 접근."""
import math

from localization.pose import Pose
from mission.basket_target import facing_error_deg
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


def _at_stop(residual_deg, knight_body=(-0.12, -0.15)):
    """정차점에 서서 바구니까지 residual_deg 남은 상태. 나이트는 차체 기준 (앞+, 왼+) 위치."""
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("soccer",)))
    fsm.step(P(0.97, 0.40), {"soccer": [(0.97, 1.0)]}, S(), 0.0)           # 바구니 정차점 정해짐
    sx, sy = fsm.dest_xy
    target = fsm._basket()
    base = facing_error_deg(target, (sx, sy), 0.0)                          # yaw 0 일 때 남는 각
    yaw = base - residual_deg                                               # 이 방향이면 residual_deg 가 남는다
    th = math.radians(yaw)
    lx, ly = knight_body
    knight = (sx + lx * math.cos(th) - ly * math.sin(th), sy + lx * math.sin(th) + ly * math.cos(th))
    fsm._enter(HostState.NUDGE_BOX)
    held = (sx + 0.26 * math.cos(th), sy + 0.26 * math.sin(th))             # 그리퍼 안의 공
    pmap = {"soccer": [held], "knight": [knight]}
    return fsm, P(sx, sy, yaw), pmap


def test_places_with_the_arm_only_when_turning_would_hit_a_piece():
    fsm, pose, pmap = _at_stop(19.7)
    cmds = [fsm.step(pose, pmap, S(), 1.0 + 0.1 * k) for k in range(4)]
    assert any("팔로만 넣음" in e for e in fsm.events), list(fsm.events)
    assert all(c.angular_z == 0 for c in cmds)                              # 몸은 돌리지 않는다
    assert fsm.state == HostState.PLACE
    assert abs(abs(fsm.place_arm_yaw_deg) - fsm.cfg.mission.max_arm_yaw_deg) < 1e-6


def test_turns_the_body_as_before_when_nothing_is_beside():
    fsm, pose, pmap = _at_stop(19.7, knight_body=(-0.60, -0.60))           # 멀리
    cmds = [fsm.step(pose, pmap, S(), 1.0 + 0.1 * k) for k in range(3)]
    assert any(c.angular_z != 0 for c in cmds)
    assert not any("팔로만" in e for e in fsm.events)


def test_too_much_short_goes_back_to_approach():
    fsm, pose, pmap = _at_stop(30.0)                                        # 팔로 15° 모자란다(상한 10°)
    fsm.step(pose, pmap, S(), 1.0)
    assert fsm.state == HostState.CARRY_TO_DEST
    assert any("팔로는" in e and "모자란다" in e for e in fsm.events)
