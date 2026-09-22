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
