import math

import pytest

from localization.pose import Pose
from mission.host_fsm import HostState, MissionFSM
from vla_common.protocol import JobResult, PiStatus, State


def P(x, y, yaw=90.0):
    return Pose(x, y, yaw, ok=True, n_cams=2, fresh=True)


def S(job_id=0, result=None, busy=False, boot="B"):
    return PiStatus(boot_id=boot, state=State.IDLE, busy=busy, job_id=job_id, result=result,
                    base_ok=True, watchdog=False)


PM = {"queen": [(0.9, 0.9)], "star": [(1.5, 1.2)]}


def _into_grasp(fsm, status):
    cmd = fsm.step(P(0.9, 0.62), PM, status, 0.0)     # 0.28 m < 트리거 0.30
    assert fsm.state == HostState.GRASP
    assert cmd.state == State.GRASP and cmd.stop and cmd.label == "queen"
    return cmd


def test_search_picks_nearest_and_drives(cfg):
    fsm = MissionFSM(cfg)
    cmd = fsm.step(P(0.9, 0.45), PM, S(), 0.0)
    assert fsm.state == HostState.APPROACH_PIECE
    assert fsm.target_label == "queen" and fsm.dest_box == "basket"
    assert cmd.state == State.APPROACH and not cmd.stop and cmd.linear_x > 0


def test_grasp_ok_goes_to_carry(cfg):
    fsm = MissionFSM(cfg)
    before = S(job_id=4, result=JobResult(4, State.GRASP, True))
    _into_grasp(fsm, before)
    # 옛 결과가 반복돼도 완료가 아니다
    assert fsm.step(P(0.9, 0.62), PM, before, 0.1).state == State.GRASP
    assert fsm.step(P(0.9, 0.62), PM, S(job_id=5, busy=True, result=before.result), 0.2).state == State.GRASP
    cmd = fsm.step(P(0.9, 0.62), PM, S(job_id=5, result=JobResult(5, State.GRASP, True)), 0.3)
    assert fsm.state == HostState.CARRY_TO_DEST
    assert cmd.state == State.CARRY


def _no_retry(cfg):
    from dataclasses import replace
    return replace(cfg, mission=replace(cfg.mission, grasp_retry_max=0))


def test_grasp_failure_reapproaches_before_giving_up(cfg):
    """실패하면 바로 다음 기물로 가지 않는다 — 위치를 다시 보고 정면을 맞춘 뒤 한 번 더."""
    assert cfg.mission.grasp_retry_max == 1
    fsm = MissionFSM(cfg)
    _into_grasp(fsm, S())
    cmd = fsm.step(P(0.9, 0.62), PM, S(job_id=1, result=JobResult(1, State.GRASP, False, "empty")), 0.5)
    assert fsm.state == HostState.APPROACH_PIECE and fsm.target_label == "queen"
    assert fsm.grasp_tries == 1 and not fsm.skipped
    assert cmd.stop
    # 기물이 건드려져 4 cm 밀렸다 — 탑뷰로 새 위치를 읽고 그쪽을 향해 돈다
    moved = {"queen": [(0.95, 0.9)], "star": PM["star"]}
    cmd = fsm.step(P(0.9, 0.62), moved, S(job_id=1), 0.6)
    assert fsm.target_xy == (0.95, 0.9)
    assert fsm.state == HostState.APPROACH_PIECE and cmd.angular_z < 0     # 오른쪽(시계)으로
    # 정면을 맞추면 다시 잡는다
    fsm.step(P(0.9, 0.62, yaw=80.6), moved, S(job_id=1), 0.7)
    assert fsm.state == HostState.GRASP
    # 두 번째도 실패하면 그때 보류하고 star 로 간다
    fsm.step(P(0.9, 0.62, yaw=80.6), moved, S(job_id=2, result=JobResult(2, State.GRASP, False)), 0.8)
    assert fsm.state == HostState.SEARCH_TARGET and len(fsm.skipped) == 1
    fsm.step(P(0.9, 0.62), moved, S(job_id=2), 0.9)
    assert fsm.target_label == "star"


def test_grasp_failure_skips_target_without_retry(cfg):
    fsm = MissionFSM(_no_retry(cfg))
    _into_grasp(fsm, S())
    cmd = fsm.step(P(0.9, 0.62), PM, S(job_id=1, result=JobResult(1, State.GRASP, False, "empty")), 0.5)
    assert fsm.state == HostState.SEARCH_TARGET
    assert cmd.state == State.IDLE and cmd.stop
    assert len(fsm.skipped) == 1
    fsm.step(P(0.9, 0.62), PM, S(job_id=1, result=JobResult(1, State.GRASP, False)), 0.6)
    assert fsm.target_label == "star"


def test_does_not_grasp_until_it_faces_the_piece(cfg):
    """2026-09-30 실기: 거리만 보고 18° · 30° 어긋난 채 잡기 시작해 둘 다 실패했다."""
    fsm = MissionFSM(cfg)
    cmd = fsm.step(P(0.9, 0.62, yaw=60.0), PM, S(), 0.0)     # 0.3 m 안이지만 30° 어긋남
    assert fsm.state == HostState.APPROACH_PIECE
    assert cmd.angular_z > 0 and cmd.linear_x == 0            # 반시계로 돌아 기물을 본다
    assert fsm.grasp_face_err_deg == pytest.approx(30.0)
    fsm.step(P(0.9, 0.62, yaw=86.0), PM, S(), 0.1)             # 4° — 허용치 안
    assert fsm.state == HostState.GRASP


def test_grasp_zone_has_hysteresis(cfg):
    """돌면서 마커가 몇 cm 흔들려도 파지 구역을 들락날락하지 않는다."""
    fsm = MissionFSM(cfg)
    fsm.step(P(0.9, 0.62, yaw=40.0), PM, S(), 0.0)             # 구역 안(0.28 m), 돌기 시작
    cmd = fsm.step(P(0.9, 0.55, yaw=50.0), PM, S(), 0.1)      # 0.35 m — 트리거 밖, 히스테리시스 안
    assert cmd.linear_x == 0 and cmd.angular_z > 0           # 전진하지 않고 계속 돈다


# ---------------------------------------------------------------------------
# 파지 거리는 범위로 — 2026-09-30: 0.27~0.31 6/6 성공, 0.33 이상·0.25 는 모두 빈손
# ---------------------------------------------------------------------------
def test_creeps_forward_into_the_grasp_range_then_settles(cfg):
    m = cfg.mission
    fsm = MissionFSM(cfg)
    far = 0.9 - m.grasp_trigger_dist_m + 0.005                  # 트리거 바로 안, 범위(0.31) 밖
    fsm.step(P(0.9, far), PM, S(), 0.0)
    cmd = fsm.step(P(0.9, far), PM, S(), 0.1)
    assert fsm.state == HostState.APPROACH_PIECE
    assert cmd.linear_x > 0 and cmd.angular_z == 0 and not cmd.stop     # 정면이니 곧장 앞으로
    cmd = fsm.step(P(0.9, 0.61), PM, S(), 0.2)                  # 0.29 m — 가운데 근처, 멈춘다
    assert cmd.stop and fsm.state == HostState.APPROACH_PIECE
    fsm.step(P(0.9, 0.61), PM, S(), 0.2 + m.grasp_settle_s / 2)  # 서서 기다린다
    assert fsm.state == HostState.APPROACH_PIECE
    fsm.step(P(0.9, 0.61), PM, S(), 0.3 + m.grasp_settle_s)      # 다시 재니 범위 안 -> 파지
    assert fsm.state == HostState.GRASP


def test_backs_off_when_too_close_to_grasp(cfg):
    m = cfg.mission
    fsm = MissionFSM(cfg)
    fsm.step(P(0.9, 0.68), PM, S(), 0.0)                        # 0.22 m — 너무 가깝다
    cmd = fsm.step(P(0.9, 0.68), PM, S(), 0.1)
    assert fsm.state == HostState.APPROACH_PIECE
    assert cmd.linear_x < 0 and cmd.angular_z == 0
    cmd = fsm.step(P(0.9, 0.64), PM, S(), 0.3)                  # 0.26 m — 미리 멈춘다(지연만큼)
    assert cmd.stop
    fsm.step(P(0.9, 0.62), PM, S(), 0.4 + m.grasp_settle_s)      # 0.28 m 에 섰다 -> 파지
    assert fsm.state == HostState.GRASP


def test_a_few_mm_outside_the_range_still_moves_and_never_loops(cfg):
    """시뮬 재현: 0.314 m 에서 "미리 멈추기"가 곧바로 참이라 한 번도 안 움직이고 700 s 동안
    멈춤·대기만 되풀이했다. 시작하면 한 사이클은 움직이고, CREEP_TRIES 번 뒤에는 그 자리에서 잡는다."""
    fsm = MissionFSM(cfg)
    y = 0.9 - (cfg.mission.grasp_dist_max_m + 0.004)
    moved, t = 0, 0.0
    while fsm.state != HostState.GRASP and t < 20.0:
        cmd = fsm.step(P(0.9, y), PM, S(), t)                    # 바퀴가 안 먹는 경우까지 가정
        moved += cmd.linear_x > 0
        t += 0.1
    assert moved >= 1
    assert fsm.state == HostState.GRASP
    assert t < 10.0
    assert any("grasp here" in e for e in fsm.events)


def test_star_uses_its_own_farther_grasp_range(cfg):
    """10-01 실기: star 는 0.27 m 에서 두 번 다 그리퍼가 기물을 지나쳤다 — star 만 범위를 멀리 둔다."""
    lo, hi = cfg.mission.grasp_dist_by_label["star"]
    assert lo > cfg.mission.grasp_dist_min_m
    star = {"star": [(0.9, 0.9)]}
    fsm = MissionFSM(cfg)
    fsm.step(P(0.9, 0.63), star, S(), 0.0)                      # 0.27 m — 기본 범위 안, star 에는 가깝다
    cmd = fsm.step(P(0.9, 0.63), star, S(), 0.1)
    assert fsm.target_label == "star" and fsm.state == HostState.APPROACH_PIECE
    assert cmd.linear_x < 0                                     # 뒤로 물러나 star 범위로
    # 같은 거리의 queen 은 그대로 잡는다
    fsm = MissionFSM(cfg)
    queen = {"queen": [(0.9, 0.9)]}
    fsm.step(P(0.9, 0.63), queen, S(), 0.0)
    fsm.step(P(0.9, 0.63), queen, S(), 0.1)
    assert fsm.state == HostState.GRASP


def test_star_inside_its_own_range_is_grasped_without_adjusting(cfg):
    """star 범위 가운데에 섰으면 앞뒤로 움직이지 않고 바로 잡는다(기본 범위 밖이어도)."""
    lo, hi = cfg.mission.grasp_dist_by_label["star"]
    star = {"star": [(0.9, 0.9)]}
    fsm = MissionFSM(cfg)
    y = 0.9 - (lo + hi) / 2
    fsm.step(P(0.9, y), star, S(), 0.0)
    fsm.step(P(0.9, y), star, S(), 0.1)
    assert fsm.state == HostState.GRASP


def test_faces_the_piece_before_adjusting_the_distance(cfg):
    """정면부터 — 비스듬히 앞뒤로 가면 거리가 아니라 옆으로 움직인다."""
    fsm = MissionFSM(cfg)
    fsm.step(P(0.9, 0.68, yaw=60.0), PM, S(), 0.0)              # 0.22 m, 30° 어긋남
    cmd = fsm.step(P(0.9, 0.68, yaw=60.0), PM, S(), 0.1)
    assert cmd.angular_z > 0 and cmd.linear_x == 0


def test_rotation_slows_near_the_target_heading(cfg):
    fsm = MissionFSM(cfg)
    d = cfg.drive
    big = fsm._rotate(90.0).angular_z
    small = fsm._rotate(20.0).angular_z
    tiny = fsm._rotate(-1.0).angular_z
    assert big == pytest.approx(d.rotation_rad_s)
    assert d.rotation_min_rad_s < small < big
    assert tiny == pytest.approx(-d.rotation_min_rad_s)


def test_skip_expires(cfg):
    cfg = _no_retry(cfg)
    fsm = MissionFSM(cfg)
    _into_grasp(fsm, S())
    fsm.step(P(0.9, 0.62), {"queen": PM["queen"]}, S(job_id=1, result=JobResult(1, State.GRASP, False)), 1.0)
    fsm.step(P(0.9, 0.62), {"queen": PM["queen"]}, None, 2.0)
    assert fsm.state == HostState.SEARCH_TARGET and fsm.target_label is None
    fsm.step(P(0.9, 0.62), {"queen": PM["queen"]}, None, 1.0 + cfg.mission.skip_expiry_s + 1)
    assert fsm.target_label == "queen"


def test_grasp_timeout_skips(cfg):
    fsm = MissionFSM(cfg)
    _into_grasp(fsm, None)
    fsm.step(P(0.9, 0.62), PM, None, cfg.mission.grasp_timeout_s + 1)
    assert fsm.state == HostState.SEARCH_TARGET and len(fsm.skipped) == 1


def test_pose_lost_during_grasp_still_sends_grasp(cfg):
    fsm = MissionFSM(cfg)
    _into_grasp(fsm, S())
    cmd = fsm.step(Pose(), PM, S(job_id=1, busy=True), 0.2)
    assert cmd.state == State.GRASP and cmd.stop


def test_pose_lost_while_driving_sends_stop_with_state(cfg):
    fsm = MissionFSM(cfg)
    fsm.step(P(0.9, 0.45), PM, S(), 0.0)
    cmd = fsm.step(Pose(), PM, S(), 0.1)
    assert fsm.state == HostState.APPROACH_PIECE
    assert cmd.state == State.APPROACH and cmd.stop


def _setup_place(fsm):
    fsm.target_label, fsm.target_xy = "queen", (0.9, 0.9)
    fsm.dest_box = "basket"
    fsm.dest_xy = fsm._box_front_xy("basket")
    fsm._enter(HostState.PLACE)


def _place_attempt_from_face(fsm, prev_status, t):
    """상자 정면까지 직진해서 붙고, 남는 각도는 팔이 맡는다(차체 정렬 없음)."""
    assert fsm.state == HostState.NUDGE_BOX
    cmd = fsm.step(P(0.990, 1.160), {}, prev_status, t)         # 아직 멀다 -> 전진
    assert cmd.state == State.APPROACH_BOX and cmd.linear_x > 0
    # dest_xy(0.990, 1.260) 에서 place_arrive_tol_m 안 = 정차 완료
    cmd = fsm.step(P(0.990, 1.250), {}, prev_status, t + 0.2)
    assert fsm.state == HostState.PLACE and cmd.state == State.PLACE


def test_place_failures_retry_then_halt(cfg):
    assert cfg.mission.place_retry_max == 2
    fsm = MissionFSM(cfg)
    _setup_place(fsm)
    assert fsm.step(P(0.990, 1.0), {}, S(), 0.0).state == State.PLACE
    s1 = S(job_id=1, result=JobResult(1, State.PLACE, False, "drop"))
    fsm.step(P(0.990, 1.0), {}, s1, 1.0)
    assert fsm.state == HostState.NUDGE_BOX and fsm.place_tries == 1

    _place_attempt_from_face(fsm, s1, 2.0)
    s2 = S(job_id=2, result=JobResult(2, State.PLACE, False, "drop"))
    fsm.step(P(0.990, 1.0), {}, s2, 3.0)
    assert fsm.state == HostState.NUDGE_BOX and fsm.place_tries == 2

    _place_attempt_from_face(fsm, s2, 4.0)
    cmd = fsm.step(P(0.990, 1.0), {}, S(job_id=3, result=JobResult(3, State.PLACE, False)), 5.0)
    assert fsm.state == HostState.HALTED and fsm.halt_reason
    assert cmd.state == State.IDLE and cmd.stop
    # 사람이 prev 를 누르면 상자 앞 정렬부터 다시
    fsm.request_back()
    fsm.step(P(0.990, 1.0, 0.0), {}, None, 6.0)
    assert fsm.state == HostState.NUDGE_BOX


def test_place_ok_returns_to_search(cfg):
    fsm = MissionFSM(cfg)
    _setup_place(fsm)
    fsm.step(P(0.990, 1.0), {}, S(), 0.0)
    fsm.step(P(0.990, 1.0), {}, S(job_id=1, result=JobResult(1, State.PLACE, True)), 1.0)
    assert fsm.state == HostState.SEARCH_TARGET and fsm.target_label is None


def test_estop_latches_until_reset(cfg):
    fsm = MissionFSM(cfg)
    fsm.step(P(0.9, 0.45), PM, S(), 0.0)
    fsm.request_estop()
    for t in (0.1, 0.2):
        cmd = fsm.step(P(0.9, 0.45), PM, S(), t)
        assert cmd.state == State.ESTOP and cmd.stop
    fsm.reset()
    assert fsm.step(P(0.9, 0.45), PM, S(), 0.3).state != State.ESTOP


def test_manual_mode_waits_for_next(cfg):
    fsm = MissionFSM(cfg, manual_mode=True)
    cmd = fsm.step(P(0.9, 0.45), PM, S(), 0.0)
    assert fsm.state == HostState.SEARCH_TARGET and fsm.ready_to_advance and cmd.stop
    fsm.request_advance()
    fsm.step(P(0.9, 0.45), PM, S(), 0.1)
    assert fsm.state == HostState.APPROACH_PIECE


def test_place_carries_the_residual_angle_for_the_arm(cfg):
    """차체를 정렬하지 않는다 — 남는 각도를 재서 arm_yaw_deg 로 보낸다."""
    fsm = MissionFSM(cfg)
    _setup_place(fsm)
    fsm._enter(HostState.NUDGE_BOX)
    # dest_xy 에서 1.5 cm 왼쪽으로 치우쳐 서고 정북을 본다(허용 거리 오차 안)
    cmd = fsm.step(P(0.975, 1.260, yaw=90.0), {}, S(), 0.0)
    assert fsm.state == HostState.PLACE
    assert cmd.state == State.PLACE and cmd.stop
    assert cmd.angular_z == 0                                   # 제자리 회전 없음
    # 목표가 내 오른쪽에 있으니 팔은 시계방향(-)으로 튼다. 한계 15도 안이다
    assert -15.0 < cmd.arm_yaw_deg < 0.0
    # 명령이 반복돼도 같은 각도가 계속 실린다
    assert fsm.step(P(0.975, 1.260, yaw=90.0), {}, S(), 0.1).arm_yaw_deg == cmd.arm_yaw_deg


def test_stop_point_is_far_enough_to_turn_in_place(cfg):
    """2026-09-30 저녁: 정차점을 상자에서 0.22 m 로 물렸다. 허용치만큼 더 붙어 서도
    제자리 회전(차체 반대각)이 상자에 닿지 않아야 한다 — 그래서 상자 앞 후진이 필요 없다."""
    c, m = cfg.planner, cfg.mission
    half_diag = math.hypot(c.robot_width_m / 2, c.robot_length_m / 2)
    assert m.box_approach_margin_m - m.place_min_gap_m >= half_diag
    # 덜 붙어 서도 팔이 테두리를 넘는다
    assert m.arm_reach_m > m.box_approach_margin_m + m.place_arrive_tol_m


def test_turns_the_body_at_the_stop_point_when_the_arm_cannot_cover(cfg):
    """팔이 못 메우는 각도면 정차점에서 그대로 돈다 — 물러나지 않는다."""
    fsm = MissionFSM(cfg)
    _setup_place(fsm)
    fsm._enter(HostState.NUDGE_BOX)
    dx, dy = fsm.dest_xy
    cmd = fsm.step(P(dx, dy, yaw=40.0), {}, S(), 0.0)             # 정차는 했지만 50도 틀어짐
    assert fsm.state == HostState.NUDGE_BOX
    assert cmd.angular_z > 0 and cmd.linear_x == 0 and cmd.linear_y == 0
    cmd = fsm.step(P(dx, dy, yaw=76.0), {}, S(), 0.1)             # 14도 — 팔 한계 안이지만
    assert cmd.angular_z > 0 and fsm.state == HostState.NUDGE_BOX  # 돌기 시작했으면 12도 안까지
    fsm.step(P(dx, dy, yaw=80.0), {}, S(), 0.2)                   # 10도 — 정면까진 안 맞춘다
    assert fsm.state == HostState.PLACE
    # 같은 사유는 한 번만 기록한다(예전엔 매 사이클 찍혀 수백 줄이 쌓였다)
    assert sum("arm cannot cover" in e for e in fsm.events) == 1


def test_carry_goes_straight_to_the_stop_point(cfg):
    """집은 뒤 가운데 진입점을 거치지 않고 정차점으로 곧장 간다(2026-09-30 저녁 요청)."""
    fsm = MissionFSM(cfg)
    _setup_place(fsm)
    fsm._enter(HostState.CARRY_TO_DEST)
    fsm.step(P(0.60, 1.10, yaw=0.0), {}, S(), 0.0)
    assert fsm.nav_goal == pytest.approx(fsm.dest_xy)
    # 트리거 거리 안이면 바로 상자 앞 단계로
    dx, dy = fsm.dest_xy
    fsm.step(P(dx + 0.20, dy - 0.10, yaw=150.0), {}, S(), 0.1)
    assert fsm.state == HostState.NUDGE_BOX


def test_nudge_gives_up_when_it_never_reaches_the_front(cfg):
    """계속 밀지 않는다 — 상자를 치기 전에 다시 접근한다."""
    fsm = MissionFSM(cfg)
    _setup_place(fsm)
    fsm._enter(HostState.NUDGE_BOX)
    fsm.step(P(1.00, 1.00), {}, S(), 0.0)                       # 출발점 기록 + 전진
    assert fsm.state == HostState.NUDGE_BOX
    start = math.dist((1.00, 1.00), fsm.dest_xy)                 # 올라갈 거리만큼은 허용한다
    moved = max(cfg.mission.nudge_max_m, start + 0.15) + 0.01
    fsm.step(P(1.00, 1.00 - moved), {}, S(), 0.1)               # 멀어진 채 한계까지
    assert fsm.state == HostState.CARRY_TO_DEST and fsm.place_tries == 1


def _world_velocity(cmd, yaw_deg):
    th = math.radians(yaw_deg)
    return (cmd.linear_x * math.cos(th) - cmd.linear_y * math.sin(th),
            cmd.linear_x * math.sin(th) + cmd.linear_y * math.cos(th))


def test_too_close_backs_away_sideways_too(cfg):
    fsm = MissionFSM(cfg)
    _setup_place(fsm)
    fsm._enter(HostState.NUDGE_BOX)
    dx, dy = fsm.dest_xy
    cmd = fsm.step(P(dx, dy + cfg.mission.place_min_gap_m + 0.02, yaw=100.0), {}, S(), 0.0)  # 지나쳤다
    assert fsm.last_cmd_text.endswith("back off") and cmd.angular_z == 0
    wx, wy = _world_velocity(cmd, 100.0)
    assert wy < 0 and abs(wx) < 1e-9


def test_nudge_times_out_back_to_carry(cfg):
    fsm = MissionFSM(cfg)
    _setup_place(fsm)
    fsm._enter(HostState.CARRY_TO_DEST)
    fsm.step(P(0.40, 0.60, yaw=0.0), {}, S(), 0.0)
    fsm._enter(HostState.NUDGE_BOX)
    dx, dy = fsm.dest_xy
    t = 0.1
    k = 0
    while t < cfg.mission.nudge_timeout_s - 0.5:                     # 상자 앞에서 왔다갔다 맴돈다
        k += 1
        fsm.step(P(dx, dy - 0.10 - 0.03 * (k % 2), yaw=160.0), {}, S(), t)
        assert fsm.state == HostState.NUDGE_BOX
        t += 0.5
    fsm.step(P(dx, dy - 0.10, yaw=160.0), {}, S(), cfg.mission.nudge_timeout_s + 0.2)
    assert fsm.state == HostState.CARRY_TO_DEST and fsm.place_tries == 1


def test_carry_still_avoids_other_pieces_next_to_the_robot(cfg):
    """쥔 기물(라벨)만 뺀다. 2026-09-30: 30 cm 안의 box 까지 빠져 box 를 밀고 지나갔다."""
    fsm = MissionFSM(cfg)
    _setup_place(fsm)                      # queen 을 쥐고 basket 으로
    fsm._enter(HostState.CARRY_TO_DEST)
    robot = (1.00, 0.70)
    near = {"queen": [(1.03, 0.72)],       # 쥐고 있는 것 — 빠져야 한다
            "box": [(1.00, 0.92)]}         # 22 cm 앞의 다른 기물 — 남아야 한다
    fsm.step(P(*robot), near, S(), 0.0)
    path = fsm.nav_path
    assert path is not None
    from planning.planner import segment_circle_clearance
    for a, b in zip(path, path[1:]):
        assert segment_circle_clearance(a, b, (1.00, 0.92))[0] >= fsm._planner.safe - 1e-6


def test_after_placing_turns_toward_a_nearby_piece_in_place(cfg):
    """놓은 자리(정차점)에서 곧장 다음 기물 쪽으로 돈다 — 물러나지 않는다."""
    fsm = MissionFSM(cfg)
    pm = {"knight": [(1.16, 1.00)]}
    fsm.step(P(0.975, 1.260, yaw=103.0), pm, S(), 0.0)
    cmd = fsm.step(P(0.975, 1.260, yaw=103.0), pm, S(), 0.1)
    assert fsm.state == HostState.APPROACH_PIECE
    assert cmd.angular_z < 0 and cmd.linear_x == 0 and cmd.linear_y == 0


def test_keeps_driving_when_the_way_ahead_is_clear(cfg):
    """2026-09-30 저녁: 주변이 비었는데도 12° 틀어질 때마다 멈춰 돌아 직진·회전을 되풀이했다.
    앞길이 비어 있으면 yaw_enter_clear_deg 까지는 계속 직진한다."""
    fsm = MissionFSM(cfg)
    pm = {"queen": [(0.90, 1.20)]}
    fsm.step(P(0.90, 0.40, yaw=90.0), pm, S(), 0.0)
    cmd = fsm.step(P(0.90, 0.40, yaw=90.0), pm, S(), 0.1)
    assert fsm.state == HostState.APPROACH_PIECE and cmd.linear_x > 0
    off = (cfg.planner.yaw_enter_deg + cfg.planner.yaw_enter_clear_deg) / 2     # 12 과 25 사이
    for t in (0.2, 0.3, 0.4):
        cmd = fsm.step(P(0.90, 0.45, yaw=90.0 - off), pm, S(), t)
        assert cmd.linear_x > 0 and cmd.angular_z == 0 and not cmd.stop


def test_still_turns_early_when_a_piece_is_on_the_way(cfg):
    """앞길에 기물이 걸리면 지금처럼 12° 에서 멈춰 돈다."""
    fsm = MissionFSM(cfg)
    pm = {"queen": [(0.90, 1.20)], "star": [(0.72, 0.80)]}
    fsm.step(P(0.90, 0.40, yaw=90.0), pm, S(), 0.0)
    fsm.step(P(0.90, 0.40, yaw=90.0), pm, S(), 0.1)
    off = (cfg.planner.yaw_enter_deg + cfg.planner.yaw_enter_clear_deg) / 2     # 왼쪽(star 쪽)으로 틀어짐
    cmd = fsm.step(P(0.90, 0.45, yaw=90.0 + off), pm, S(), 0.2)
    cmd = fsm.step(P(0.90, 0.45, yaw=90.0 + off), pm, S(), 0.3)
    assert cmd.angular_z != 0 or cmd.stop
