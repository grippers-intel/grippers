import pytest

from mission.host_fsm import HostState, MissionFSM
from sim.sim_world import SimWorld


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def test_sim_moves_two_pieces_into_boxes(cfg):
    clock = FakeClock()
    world = SimWorld(cfg, clock=clock, seed=1)
    fsm = MissionFSM(cfg)
    dt = 1.0 / cfg.mission.cycle_hz
    delivered_at = None
    for cycle in range(6000):
        clock.t += dt
        world.update()
        cmd = fsm.step(world.pose(), world.piece_map(), world.link.latest_status(), clock.t)
        world.link.send(cmd)
        if len(world.pieces_in_box("chess")) + len(world.pieces_in_box("toy")) == 2:
            delivered_at = cycle
            break
    assert delivered_at is not None, f"미완료: state={fsm.state.name} events={list(fsm.events)}"
    assert world.pieces_in_box("chess") == ["queen"]
    assert world.pieces_in_box("toy") == ["star"]
    assert not fsm.skipped
    # 몇 사이클 더 돌려도 새 대상을 고르지 않고 대기한다
    for _ in range(20):
        clock.t += dt
        world.update()
        world.link.send(fsm.step(world.pose(), world.piece_map(), world.link.latest_status(), clock.t))
    assert fsm.state == HostState.SEARCH_TARGET and fsm.target_label is None


def _run_mission(cfg, world, cycles=6000):
    """기물을 전부 옮길 때까지 돌린다. 옮긴 개수를 돌려준다."""
    fsm = MissionFSM(cfg)
    clock = world.clock
    dt = 1.0 / cfg.mission.cycle_hz
    for _ in range(cycles):
        clock.t += dt
        world.update()
        world.link.send(fsm.step(world.pose(), world.piece_map(),
                                 world.link.latest_status(), clock.t))
        if len(world.pieces_in_box("chess")) + len(world.pieces_in_box("toy")) == 2:
            break
    return fsm


def test_place_lands_inside_the_basket(cfg):
    """팔이 겨눈 자리에 떨어지는지 — 시뮬 Pi 가 arm_yaw_deg 를 실제로 적용한다."""
    world = SimWorld(cfg, clock=FakeClock(), seed=3)
    _run_mission(cfg, world)
    assert world.last_drop is not None, "투하가 한 번도 없었다"
    assert world.last_arm_yaw_deg is not None
    # 판정 목표 중심에서 이 정도면 상자 안이다(상자 깊이 0.35 · 폭 0.21)
    assert world.last_drop_offset_m < 0.10
    assert len(world.pieces_in_box("chess")) + len(world.pieces_in_box("toy")) == 2


def test_ignoring_the_arm_yaw_misses(cfg):
    """각도를 무시하는 옛 Pi 라면 정차 오차가 그대로 투하 오차가 된다.

    이 테스트가 통과한다는 것은 `HostCommand.arm_yaw_deg` 가 장식이 아니라는 뜻이다.
    """
    honoring = SimWorld(cfg, clock=FakeClock(), seed=3)
    _run_mission(cfg, honoring)
    ignoring = SimWorld(cfg, clock=FakeClock(), seed=3, honor_arm_yaw=False)
    _run_mission(cfg, ignoring)
    assert honoring.last_drop_offset_m < ignoring.last_drop_offset_m


def test_arm_reach_clears_the_rim_but_not_the_box(cfg):
    """팔 길이가 기하와 맞물리는지 — 양쪽 끝을 본다.

    짧으면 기물이 상자 앞에 떨어지고, 너무 길면 상자 너머로 넘어간다. 지금 값(0.32 m)은
    테두리를 0.17 m 넘고 상자 깊이 0.35 m 안이라 가운데에 떨어진다.
    """
    m = cfg.mission
    fsm = MissionFSM(cfg)
    for name, (_bx, by, _yaw) in cfg.arena.boxes.items():
        rim_gap = (by - cfg.arena.box_size[1] / 2.0) - fsm._box_front_xy(name)[1]
        assert m.arm_reach_m >= rim_gap + m.place_arrive_tol_m, "가장 덜 붙어 서면 테두리를 못 넘는다"
        depth = m.arm_reach_m - rim_gap + m.place_min_gap_m     # 가장 붙어 섰을 때 투하 깊이
        assert depth <= cfg.arena.box_size[1], "상자 너머로 넘어간다"


def test_a_shorter_arm_still_lands_inside(cfg):
    """실측(0.32 m)보다 짧은 팔(0.25 m)로도 기물이 테두리 안에 떨어져야 한다.

    정차가 덜 붙는 쪽으로 place_arrive_tol_m(0.06) 까지 벌어질 수 있으므로, 도달거리에
    그만큼 여유가 있는지 보는 것이다.
    """
    world = SimWorld(cfg, clock=FakeClock(), seed=3, place_reach_m=0.25)
    _run_mission(cfg, world)
    assert len(world.pieces_in_box("chess")) + len(world.pieces_in_box("toy")) == 2
    edge = cfg.arena.boxes["chess"][1] - cfg.arena.box_size[1] / 2.0
    assert world.last_drop[1] >= edge, "기물이 상자 테두리 앞에 떨어졌다"


def test_warns_at_startup_when_the_arm_is_too_short(cfg):
    """팔이 테두리를 못 넘는 설정이면 **기동할 때** 알린다.

    주행으로 풀 수 있는 문제가 아니다 — 차는 dest_xy 보다 더 붙을 수 없다(차체가 상자에
    닿는다). 그래서 사람이 상자를 옮기거나 drop 자세를 다시 교시해야 한다.
    """
    from dataclasses import replace
    short = replace(cfg, mission=replace(cfg.mission, arm_reach_m=0.10))
    fsm = MissionFSM(short)
    assert any("상자 앞에 떨어진다" in e for e in fsm.events), list(fsm.events)
    # 지금 설정(실측 0.17~0.20 의 중앙값)에서는 경고가 없어야 한다
    assert not any("상자 앞에 떨어진다" in e for e in MissionFSM(cfg).events)


def test_a_too_short_arm_does_not_deliver(cfg):
    """경고를 무시하고 돌리면 기물이 상자 앞에 떨어지고, 재시도 끝에 멈춘다."""
    from dataclasses import replace
    short = replace(cfg, mission=replace(cfg.mission, arm_reach_m=0.10))
    world = SimWorld(short, clock=FakeClock(), seed=3, place_reach_m=0.10)
    fsm = _run_mission(short, world)
    assert len(world.pieces_in_box("chess")) + len(world.pieces_in_box("toy")) == 0
    assert fsm.state == HostState.HALTED
