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


def _carry_from_inside_gap(extra=None):
    """10-07 실기 재현: 틈 바로 너머 공을 틈 한가운데서 잡았다. 쥔 공은 그리퍼 안에서 보인다."""
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("soccer",)))
    fsm.step(P(0.97, 0.40), {"soccer": [(0.97, 1.11)], **GAP}, S(), 0.0)    # 목표·정차점 정해짐
    from mission.host_fsm import HostState
    fsm.state = HostState.CARRY_TO_DEST
    fsm.dest_xy = (1.55, 1.20)                                              # 오른쪽으로 크게 돌아야 한다
    pmap = {"soccer": [(0.97, 1.10)], "box": [(1.13, 0.78)], "rook": [(0.78, 0.79)], **(extra or {})}
    return fsm, pmap


def test_leaves_the_gap_forward_before_turning_after_a_grasp():
    fsm, pmap = _carry_from_inside_gap()
    cmds = [fsm.step(P(0.97, 0.82), pmap, S(), 0.1 + 0.1 * k) for k in range(4)]
    moving = [c for c in cmds if not c.stop]
    assert moving and moving[0].linear_x > 0 and moving[0].angular_z == 0    # 먼저 앞으로
    assert any("turn exit" in e for e in fsm.events)
    # 틈을 벗어나면(옆 기물이 회전 영역 밖) 돈다
    later = [fsm.step(P(0.97, 1.05), pmap, S(), 1.0 + 0.1 * k) for k in range(4)]
    assert any(c.angular_z != 0 for c in later)


def test_turns_in_place_when_the_way_ahead_is_blocked_too():
    fsm, pmap = _carry_from_inside_gap(extra={"knight": [(0.97, 1.02)]})  # 바로 앞(공 너머)에 나이트
    cmds = [fsm.step(P(0.97, 0.82), pmap, S(), 0.1 + 0.1 * k) for k in range(4)]
    assert not any("turn exit" in e for e in fsm.events)
    assert all(c.linear_x <= 0 for c in cmds)                                # 앞으로 밀고 나가지 않는다


def test_passing_beside_one_piece_is_not_a_gap():
    """10-07 실기: 상자 바깥으로 15 cm 붙어 돌아가는 우회를 틈으로 봐서 5° 재정렬로 좌우로 왔다갔다 했다.
    한쪽에만 기물이 있으면 틈이 아니다 — 평소 문턱(12°/25°)으로 간다."""
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("queen",)))
    pmap = {"box": [(1.143, 0.897)], "queen": [(1.30, 1.25)]}
    fsm.step(P(1.30, 0.55, yaw=90.0), pmap, S(), 0.0)                       # 상자 오른쪽 15.7 cm 를 지나는 직선
    cmd = fsm.step(P(1.30, 0.62, yaw=98.0), pmap, S(), 0.1)                 # 상자 35 cm 안, 8° 틀어짐
    assert not any("narrow gap" in e for e in fsm.events)
    assert cmd.linear_x > 0 and cmd.angular_z == 0


def test_detour_into_the_grasp_zone_does_not_turn_into_the_piece_beside():
    """10-07 실기 재현: 룩을 끼고 돌아 공 0.30 m 안에 옆구리로 들어와(127°), 공 쪽(38°)으로 90° 돌다 룩을 밀었다.
    그 회전이 룩을 쓸면 아직 잡지 않고, 앞으로 빠져나간 뒤 돈다."""
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("soccer",)))
    pmap = {"soccer": [(0.93, 1.115)], "rook": [(0.825, 0.889)], "box": [(1.143, 0.897)]}
    fsm.step(P(0.99, 0.35), pmap, S(), 0.0)                                 # 목표 고르기
    cmds = [fsm.step(P(0.694, 0.929, yaw=127.0), pmap, S(), 0.1 + 0.1 * k) for k in range(4)]
    assert any("grasp zone" in e for e in fsm.events)
    moving = [c for c in cmds if not c.stop]
    assert moving and moving[0].angular_z == 0 and moving[0].linear_x > 0   # 그 자리에서 돌지 않고 앞으로
