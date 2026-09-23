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


def test_arm_reach_matches_the_geometry(cfg):
    """정차점에서 조준점까지가 곧 팔이 뻗어야 하는 거리다. 설정이 기하와 맞는지 본다."""
    fsm = MissionFSM(cfg)
    for name in cfg.arena.boxes:
        need = fsm._basket(name).distance(fsm._box_front_xy(name))
        assert need == pytest.approx(cfg.mission.arm_reach_m, abs=cfg.mission.place_arrive_tol_m)
