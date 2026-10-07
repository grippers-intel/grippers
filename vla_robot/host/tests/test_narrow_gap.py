"""좁은 틈(2026-10-07): 틈 밖에서 방향을 정확히 맞추고 들어가, 틈 안에서는 돌지 않는다(후진 없음)."""
from localization.pose import Pose
from mission.host_fsm import MissionFSM, Order
from planning.planner import DriveMode, DriveSequencer
from vla_common.protocol import PiStatus, State


def P(x, y, yaw=90.0):
    return Pose(x, y, yaw, ok=True, n_cams=2, fresh=True)


def S():
    return PiStatus(boot_id="B", state=State.IDLE, busy=False, job_id=0, result=None,
                    base_ok=True, watchdog=False)


def _cfg():
    from host_config import load_host_config
    return load_host_config(None)


# 룩·상자 사이 34 cm 틈(직진 28 cm 이상 · 제자리 회전 40 cm 미만), 그 너머에 퀸
GAP = {"rook": [(0.78, 0.75)], "box": [(1.12, 0.75)]}


def test_sequencer_tighter_tolerance_and_entry_threshold():
    cfg = _cfg()
    d = DriveSequencer(cfg.planner)
    assert d.update((0, 0), 4.0, (1, 0)).mode == DriveMode.FORWARD          # 기본 허용치 5°
    d.reset()
    assert d.update((0, 0), 4.0, (1, 0), enter_deg=5.0, tol_deg=2.5).mode == DriveMode.ROTATE


def test_aligns_precisely_before_entering_a_narrow_gap():
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("queen",)))
    pmap = dict(GAP, queen=[(0.95, 1.30)])
    cmd = fsm.step(P(0.95, 0.40, yaw=94.0), pmap, S(), 0.0)                 # 4° 틀어짐, 틈 밖
    assert cmd.angular_z != 0 and cmd.linear_x == 0                          # 들어가기 전에 맞춘다
    assert any("narrow gap ahead" in e for e in fsm.events)
    # 틈이 없으면 4° 는 그대로 간다
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("queen",)))
    cmd = fsm.step(P(0.95, 0.40, yaw=94.0), {"queen": [(0.95, 1.30)]}, S(), 0.0)
    assert cmd.linear_x > 0 and cmd.angular_z == 0


def test_keeps_going_straight_inside_the_gap():
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("queen",)))
    pmap = dict(GAP, queen=[(0.95, 1.25)])                         # 바구니 금지 구역 밖
    fsm.step(P(0.95, 0.75, yaw=90.0), pmap, S(), 0.0)                       # 틈 한가운데, 바로 봄
    cmd = fsm.step(P(0.95, 0.76, yaw=105.0), pmap, S(), 0.1)                # 15° 틀어졌다(기본 문턱 12°)
    assert cmd.linear_x > 0 and cmd.angular_z == 0                           # 틈 안에서는 돌지 않는다


def test_does_not_keep_realigning_far_from_the_gap():
    """10-07 실기: 틈으로 가는 구간 전체에 5° 재정렬을 걸어 "조금 가고 yaw 보정"을 되풀이했다.
    입구에서 먼 곳에서는 직진 중 8° 틀어져도 그대로 간다(어차피 돌 때만 2.5° 로 맞춘다)."""
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("queen",)))
    pmap = dict(GAP, queen=[(0.95, 1.25)])
    fsm.step(P(0.95, 0.30, yaw=90.0), pmap, S(), 0.0)                       # 틈에서 45 cm 앞, 바로 봄
    cmd = fsm.step(P(0.95, 0.31, yaw=98.0), pmap, S(), 0.1)                 # 8° 틀어짐
    assert cmd.linear_x > 0 and cmd.angular_z == 0
    # 입구 바로 앞이면 들어가기 전에 다시 맞춘다
    cmds = [fsm.step(P(0.95, 0.52, yaw=98.0), pmap, S(), 0.2 + 0.1 * k) for k in range(3)]
    assert any(c.angular_z != 0 for c in cmds)                               # 한 박자 서고 돈다
