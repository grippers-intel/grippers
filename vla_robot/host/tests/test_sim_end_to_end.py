import pytest

from mission.host_fsm import HostState, MissionFSM
from sim.sim_world import SimWorld


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def test_sim_moves_two_pieces_into_the_basket(cfg):
    """상자가 하나라(배치도 REV.2) 두 기물 모두 basket 으로 간다."""
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
        if len(world.pieces_in_box("basket")) == 2:
            delivered_at = cycle
            break
    assert delivered_at is not None, f"미완료: state={fsm.state.name} events={list(fsm.events)}"
    assert sorted(world.pieces_in_box("basket")) == ["queen", "star"]
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
        if len(world.pieces_in_box("basket")) == 2:
            break
    return fsm


def test_place_lands_inside_the_basket(cfg):
    """팔이 겨눈 자리에 떨어지는지 — 시뮬 Pi 가 arm_yaw_deg 를 실제로 적용한다."""
    world = SimWorld(cfg, clock=FakeClock(), seed=3)
    _run_mission(cfg, world)
    assert world.last_drop is not None, "투하가 한 번도 없었다"
    assert world.last_arm_yaw_deg is not None
    # 지켜야 할 것은 "상자 **안**에 떨어지는가" 다. 팔이 0.32 m 라 기물은 테두리 근처가 아니라
    # 상자 가운데쯤 떨어지도록 설계돼 있다 — 테두리 쪽 판정 중심과의 거리는 기준이 아니다.
    dx, dy = world.last_drop
    bw, bl, _bh = cfg.arena.box_size
    inside = any(abs(dx - bx) <= bw / 2.0 and by - bl / 2.0 <= dy <= by + bl / 2.0
                 for bx, by, _yaw in cfg.arena.boxes.values())
    assert inside, f"상자 밖에 떨어졌다: {world.last_drop}"
    assert len(world.pieces_in_box("basket")) == 2


def test_ignoring_the_arm_yaw_misses(cfg):
    """각도를 무시하는 옛 Pi 라면 정차 오차가 그대로 투하 오차가 된다.

    이 테스트가 통과한다는 것은 `HostCommand.arm_yaw_deg` 가 장식이 아니라는 뜻이다.
    겨누는 선으로 붙기(10-08)는 차체가 겨누는 점을 보고 서서 팔 각이 거의 0 이라 끄고, 정차 구역(가장 가까운 자리,
    10-08)은 겨누는 점을 일부러 옆으로 옮기므로 가운데 한 점으로 둔다 — 여기서는 팔 각이 쓰이는지만 본다.
    """
    from dataclasses import replace as _replace
    cfg = _replace(cfg, mission=_replace(cfg.mission, place_here_max_m=0.0, basket_stop_zone_half_m=0.0))
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
    """실측(0.32 m)보다 4 cm 짧은 팔(0.28 m)로도 기물이 테두리 안에 떨어져야 한다.

    정차가 덜 붙는 쪽으로 place_arrive_tol_m 까지 벌어지고 팔 base 를 place_turn_to_deg 까지
    틀 수 있으므로, 도달거리에 그만큼 여유가 있는지 보는 것이다. 정차점을 상자에서 0.22 m 로
    물린 뒤(2026-09-30 저녁) 필요한 도달거리는 (0.22 + 0.04) / cos 12° = 0.266 m 다.
    """
    world = SimWorld(cfg, clock=FakeClock(), seed=3, place_reach_m=0.28)
    _run_mission(cfg, world)
    assert len(world.pieces_in_box("basket")) == 2
    edge = cfg.arena.boxes["basket"][1] - cfg.arena.box_size[1] / 2.0
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
    assert len(world.pieces_in_box("basket")) == 0
    assert fsm.state == HostState.HALTED


# ---------------------------------------------------------------------------
# 차체 무응답 자동 복구 (2026-09-30 실기: 보드가 쓰기를 조용히 무시했다)
# ---------------------------------------------------------------------------
def test_base_that_stops_responding_is_recovered_and_the_mission_finishes(cfg):
    """운반 도중 바퀴가 명령을 무시하기 시작한다 — Host 가 알아채 복구를 요청하고 이어간다."""
    world = SimWorld(cfg, clock=FakeClock(), seed=3)
    fsm = MissionFSM(cfg)
    clock = world.clock
    failed = False
    for _ in range(8000):
        clock.t += 0.1
        world.update()
        world.link.send(fsm.step(world.pose(), world.piece_map(), world.link.latest_status(), clock.t))
        if not failed and fsm.state == HostState.CARRY_TO_DEST:
            world.fail_base()
            failed = True
        if len(world.pieces_in_box("basket")) == 2:
            break
    assert failed
    assert world.base_recoveries == 1, "복구를 한 번 요청해야 한다"
    assert sorted(world.pieces_in_box("basket")) == ["queen", "star"]
    assert any("복구 완료" in e for e in fsm.events) or world.base_recoveries == 1


def test_runaway_base_is_reset_and_the_mission_finishes(cfg):
    """2026-09-30 실기: 상자 앞에서 회전만 보냈는데 굳은 직진 속도로 1 m 를 달려 장판 밖으로 나갔다.
    Host 가 "직진을 안 시켰는데 달린다"를 알아채 보드 리셋을 요청하고, 멀리 가기 전에 선다."""
    import math
    world = SimWorld(cfg, clock=FakeClock(), seed=3)
    fsm = MissionFSM(cfg)
    clock = world.clock
    froze_at, stopped_at = None, None
    seen = set()                               # fsm.events 는 최근 것만 남는다 — 그때그때 모은다
    for _ in range(8000):
        clock.t += 0.1
        world.update()
        cmd = fsm.step(world.pose(), world.piece_map(), world.link.latest_status(), clock.t)
        world.link.send(cmd)
        seen.update(list(fsm.events)[-3:])
        if froze_at is None and fsm.state in (HostState.CARRY_TO_DEST, HostState.NUDGE_BOX) and cmd.linear_x > 0 \
                and world._vel[0] > 0:
            world.fail_runaway()
            froze_at = (world.x, world.y)
        if froze_at is not None and stopped_at is None and not world.runaway:
            stopped_at = (world.x, world.y)
        if len(world.pieces_in_box("basket")) == 2:
            break
    assert froze_at is not None and stopped_at is not None, list(fsm.events)
    assert any("폭주" in e for e in seen), sorted(seen)
    assert world.base_recoveries == 1
    # 굳은 뒤 멈출 때까지: 부분목표에 닿을 때까지(최대 ~0.3 m) + 폭주 판단(grace 1 s + 창 0.5 s)
    assert math.dist(froze_at, stopped_at) < 0.6
    assert sorted(world.pieces_in_box("basket")) == ["queen", "star"]


def test_spin_runaway_is_reset_and_the_mission_finishes(cfg):
    """2026-10-05 실기: 다음 기물로 돌던 중 반시계 ~8°/s 로 굳어 yaw-/yaw+ · ESTOP 을 모두 무시했다.
    Host 가 "반대로(또는 안) 돌라고 했는데 돈다"를 알아채 보드 리셋을 요청하고, 미션을 끝낸다."""
    import math
    world = SimWorld(cfg, clock=FakeClock(), seed=3)
    fsm = MissionFSM(cfg)
    clock = world.clock
    froze_at, stopped_at = None, None
    seen = set()                               # fsm.events 는 최근 것만 남는다 — 그때그때 모은다
    for _ in range(8000):
        clock.t += 0.1
        world.update()
        cmd = fsm.step(world.pose(), world.piece_map(), world.link.latest_status(), clock.t)
        world.link.send(cmd)
        seen.update(list(fsm.events)[-3:])
        if froze_at is None and fsm.state == HostState.APPROACH_PIECE and abs(cmd.angular_z) > 0                 and abs(world._vel[2]) > 0:
            world.fail_spin(math.copysign(math.radians(8.0), cmd.angular_z))
            froze_at = clock.t
        if froze_at is not None and stopped_at is None and not world.runaway:
            stopped_at = clock.t
        if len(world.pieces_in_box("basket")) == 2:
            break
    assert froze_at is not None and stopped_at is not None, list(fsm.events)
    assert any("회전 폭주" in e for e in seen), sorted(seen)
    assert world.base_recoveries == 1
    # 굳은 뒤 리셋까지 돈 양(8°/s x 시간): 목표 방향을 지나 반대 명령이 나올 때까지 + 판단(grace 1 s + 창 1 s)
    assert 8.0 * (stopped_at - froze_at) < 120
    assert sorted(world.pieces_in_box("basket")) == ["queen", "star"]


def test_quiet_reset_during_arm_jobs_is_waited_out(cfg):
    """10-05: Pi 가 팔 작업 시작마다 컨트롤러를 다시 띄운다(소음 정리). 투하보다 길게 걸려도 Host 는
    그동안 바퀴 명령을 내지 않고 기다려야 한다 — 무응답으로 오판해 복구를 요청하면 안 된다."""
    world = SimWorld(cfg, clock=FakeClock(), seed=1)
    world.quiet_reset_s = 4.0                       # 시뮬 투하(2 s)보다 길다
    fsm = MissionFSM(cfg)
    clock = world.clock
    seen, waited = set(), 0
    for _ in range(8000):
        clock.t += 0.1
        world.update()
        cmd = fsm.step(world.pose(), world.piece_map(), world.link.latest_status(), clock.t)
        world.link.send(cmd)
        seen.update(list(fsm.events)[-3:])
        if fsm.last_cmd_text == "base reset (wait)":
            waited += 1
            assert cmd.stop and not cmd.recover_base
        if len(world.pieces_in_box("basket")) == 2:
            break
    assert sorted(world.pieces_in_box("basket")) == ["queen", "star"]
    assert waited > 0, "투하 뒤 재기동이 끝날 때까지 기다린 적이 있어야 한다"
    assert not any("무응답" in e or "폭주" in e for e in seen), sorted(seen)
    # 파지 2 + 투하 2 번 재기동(마지막 투하의 것은 루프가 끝날 때 아직 도는 중일 수 있다). Host 요청 복구는 없다.
    assert world.base_recoveries in (3, 4)


def test_runaway_is_not_confused_with_the_arm_or_coasting(cfg):
    """팔이 펴지며 마커가 움직이는 것(GRASP/PLACE)이나 멈춘 뒤 관성으로 몇 cm 더 가는 것은 폭주가 아니다."""
    world = SimWorld(cfg, clock=FakeClock(), seed=1)
    fsm = _run_mission(cfg, world)
    assert len(world.pieces_in_box("basket")) == 2
    assert world.base_recoveries == 0
    assert not any("폭주" in e for e in fsm.events)


def test_base_that_never_recovers_halts_with_a_reason(cfg):
    """복구해도 계속 안 움직이면(전원·배선) 무한히 반복하지 않고 멈춰서 사람을 부른다."""
    world = SimWorld(cfg, clock=FakeClock(), seed=3)
    world.recover_s = 1.0
    fsm = MissionFSM(cfg)
    clock = world.clock
    for _ in range(3000):
        clock.t += 0.1
        world.base_dead = True               # 복구가 끝나도 다시 죽는다
        world.update()
        world.link.send(fsm.step(world.pose(), world.piece_map(), world.link.latest_status(), clock.t))
        if fsm.state == HostState.HALTED:
            break
    assert fsm.state == HostState.HALTED
    assert "차체가 명령을 따르지 않는다" in fsm.halt_reason
    assert world.base_recoveries == cfg.mission.base_recover_max
