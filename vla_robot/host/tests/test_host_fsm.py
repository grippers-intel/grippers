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
    cmd = fsm.step(P(0.9, 0.6), PM, status, 0.0)     # 0.3 m < 트리거 0.35
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
    assert fsm.step(P(0.9, 0.6), PM, before, 0.1).state == State.GRASP
    assert fsm.step(P(0.9, 0.6), PM, S(job_id=5, busy=True, result=before.result), 0.2).state == State.GRASP
    cmd = fsm.step(P(0.9, 0.6), PM, S(job_id=5, result=JobResult(5, State.GRASP, True)), 0.3)
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
    cmd = fsm.step(P(0.9, 0.6), PM, S(job_id=1, result=JobResult(1, State.GRASP, False, "empty")), 0.5)
    assert fsm.state == HostState.APPROACH_PIECE and fsm.target_label == "queen"
    assert fsm.grasp_tries == 1 and not fsm.skipped
    assert cmd.stop
    # 기물이 건드려져 4 cm 밀렸다 — 탑뷰로 새 위치를 읽고 그쪽을 향해 돈다
    moved = {"queen": [(0.95, 0.9)], "star": PM["star"]}
    cmd = fsm.step(P(0.9, 0.6), moved, S(job_id=1), 0.6)
    assert fsm.target_xy == (0.95, 0.9)
    assert fsm.state == HostState.APPROACH_PIECE and cmd.angular_z < 0     # 오른쪽(시계)으로
    # 정면을 맞추면 다시 잡는다
    fsm.step(P(0.9, 0.6, yaw=80.6), moved, S(job_id=1), 0.7)
    assert fsm.state == HostState.GRASP
    # 두 번째도 실패하면 그때 보류하고 star 로 간다
    fsm.step(P(0.9, 0.6, yaw=80.6), moved, S(job_id=2, result=JobResult(2, State.GRASP, False)), 0.8)
    assert fsm.state == HostState.SEARCH_TARGET and len(fsm.skipped) == 1
    fsm.step(P(0.9, 0.6), moved, S(job_id=2), 0.9)
    assert fsm.target_label == "star"


def test_grasp_failure_skips_target_without_retry(cfg):
    fsm = MissionFSM(_no_retry(cfg))
    _into_grasp(fsm, S())
    cmd = fsm.step(P(0.9, 0.6), PM, S(job_id=1, result=JobResult(1, State.GRASP, False, "empty")), 0.5)
    assert fsm.state == HostState.SEARCH_TARGET
    assert cmd.state == State.IDLE and cmd.stop
    assert len(fsm.skipped) == 1
    fsm.step(P(0.9, 0.6), PM, S(job_id=1, result=JobResult(1, State.GRASP, False)), 0.6)
    assert fsm.target_label == "star"


def test_does_not_grasp_until_it_faces_the_piece(cfg):
    """2026-09-30 실기: 거리만 보고 18° · 30° 어긋난 채 잡기 시작해 둘 다 실패했다."""
    fsm = MissionFSM(cfg)
    cmd = fsm.step(P(0.9, 0.6, yaw=60.0), PM, S(), 0.0)     # 0.3 m 안이지만 30° 어긋남
    assert fsm.state == HostState.APPROACH_PIECE
    assert cmd.angular_z > 0 and cmd.linear_x == 0            # 반시계로 돌아 기물을 본다
    assert fsm.grasp_face_err_deg == pytest.approx(30.0)
    fsm.step(P(0.9, 0.6, yaw=86.0), PM, S(), 0.1)             # 4° — 허용치 안
    assert fsm.state == HostState.GRASP


def test_grasp_zone_has_hysteresis(cfg):
    """돌면서 마커가 몇 cm 흔들려도 파지 구역을 들락날락하지 않는다."""
    fsm = MissionFSM(cfg)
    fsm.step(P(0.9, 0.6, yaw=40.0), PM, S(), 0.0)             # 구역 안(0.30 m), 돌기 시작
    cmd = fsm.step(P(0.9, 0.53, yaw=50.0), PM, S(), 0.1)      # 0.37 m — 트리거 밖, 히스테리시스 안
    assert cmd.linear_x == 0 and cmd.angular_z > 0           # 전진하지 않고 계속 돈다


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
    fsm.step(P(0.9, 0.6), {"queen": PM["queen"]}, S(job_id=1, result=JobResult(1, State.GRASP, False)), 1.0)
    fsm.step(P(0.9, 0.6), {"queen": PM["queen"]}, None, 2.0)
    assert fsm.state == HostState.SEARCH_TARGET and fsm.target_label is None
    fsm.step(P(0.9, 0.6), {"queen": PM["queen"]}, None, 1.0 + cfg.mission.skip_expiry_s + 1)
    assert fsm.target_label == "queen"


def test_grasp_timeout_skips(cfg):
    fsm = MissionFSM(cfg)
    _into_grasp(fsm, None)
    fsm.step(P(0.9, 0.6), PM, None, cfg.mission.grasp_timeout_s + 1)
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
    cmd = fsm.step(P(0.990, 1.230), {}, prev_status, t)         # 아직 멀다 -> 전진
    assert cmd.state == State.APPROACH_BOX and cmd.linear_x > 0
    # dest_xy(0.990, 1.330) 에서 place_arrive_tol_m 안 = 정차 완료
    cmd = fsm.step(P(0.990, 1.320), {}, prev_status, t + 0.2)
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
    cmd = fsm.step(P(0.975, 1.330, yaw=90.0), {}, S(), 0.0)
    assert fsm.state == HostState.PLACE
    assert cmd.state == State.PLACE and cmd.stop
    assert cmd.angular_z == 0                                   # 제자리 회전 없음
    # 목표가 내 오른쪽에 있으니 팔은 시계방향(-)으로 튼다. 한계 15도 안이다
    assert -15.0 < cmd.arm_yaw_deg < 0.0
    # 명령이 반복돼도 같은 각도가 계속 실린다
    assert fsm.step(P(0.975, 1.330, yaw=90.0), {}, S(), 0.1).arm_yaw_deg == cmd.arm_yaw_deg


def test_never_turns_the_body_right_next_to_the_box(cfg):
    """팔이 못 메우는 각도면 차체를 돌리되, 정차점(상자 2 cm 앞)에서는 돌지 않고 먼저 물러난다.
    2026-09-30 rook: 정차점에서 돌다가 모서리로 상자를 쳤고 뒤 카메라까지 밀렸다."""
    fsm = MissionFSM(cfg)
    _setup_place(fsm)
    fsm._enter(HostState.NUDGE_BOX)
    cmd = fsm.step(P(0.990, 1.330, yaw=40.0), {}, S(), 0.0)       # 정차는 했지만 50도 틀어짐
    assert fsm.state == HostState.NUDGE_BOX
    assert cmd.linear_x < 0 and cmd.angular_z == 0                # 돌지 않고 뒤로
    # 상자 입구에서 box_turn_clear_m 밖으로 나오면 그때 돈다
    front_y = cfg.arena.boxes["basket"][1] - cfg.arena.box_size[1] / 2
    y = front_y - cfg.mission.box_turn_clear_m - 0.005
    cmd = fsm.step(P(0.990, y, yaw=40.0), {}, S(), 0.1)
    assert cmd.angular_z > 0 and cmd.linear_x == 0
    # 같은 사유는 한 번만 기록한다(예전엔 매 사이클 찍혀 수백 줄이 쌓였다)
    assert sum("arm cannot cover" in e for e in fsm.events) == 1


def test_carry_goes_to_the_lead_in_point_first(cfg):
    """운반은 정차점 바로 앞 진입점까지 — 거기서 상자를 향해 똑바로 올라간다."""
    fsm = MissionFSM(cfg)
    _setup_place(fsm)
    fsm._enter(HostState.CARRY_TO_DEST)
    dx, dy = fsm.dest_xy
    lead = (dx, dy - cfg.mission.box_lead_in_m)
    fsm.step(P(0.40, 0.60, yaw=0.0), {}, S(), 0.0)
    assert fsm.nav_goal == pytest.approx(lead)
    # 정차점 트리거(0.35) 안이어도 진입점에 닿기 전에는 넘어가지 않는다
    fsm.step(P(dx + 0.20, dy - 0.10, yaw=90.0), {}, S(), 0.1)
    assert fsm.state == HostState.CARRY_TO_DEST
    fsm.step(P(lead[0] + 0.02, lead[1], yaw=150.0), {}, S(), 0.2)
    assert fsm.state == HostState.NUDGE_BOX
    # 진입점에서는 상자에서 충분히 멀어 제자리에서 돈다(물러나지 않는다)
    cmd = fsm.step(P(lead[0] + 0.02, lead[1], yaw=150.0), {}, S(), 0.3)
    assert cmd.angular_z < 0 and cmd.linear_x == 0


def test_carry_falls_back_to_the_stop_point_when_the_lead_in_is_blocked(cfg):
    fsm = MissionFSM(cfg)
    _setup_place(fsm)
    fsm._enter(HostState.CARRY_TO_DEST)
    dx, dy = fsm.dest_xy
    piece_on_lead_in = {"star": [(dx + 0.05, dy - cfg.mission.box_lead_in_m)]}
    fsm.step(P(0.40, 0.60, yaw=0.0), piece_on_lead_in, S(), 0.0)
    assert fsm.nav_goal == pytest.approx((dx, dy))


def test_nudge_gives_up_when_it_never_reaches_the_front(cfg):
    """계속 밀지 않는다 — 상자를 치기 전에 다시 접근한다."""
    fsm = MissionFSM(cfg)
    _setup_place(fsm)
    fsm._enter(HostState.NUDGE_BOX)
    fsm.step(P(1.00, 1.00), {}, S(), 0.0)                       # 출발점 기록 + 전진
    assert fsm.state == HostState.NUDGE_BOX
    moved = cfg.mission.nudge_max_m + 0.01
    fsm.step(P(1.00, 1.00 - moved), {}, S(), 0.1)               # 멀어진 채 한계까지
    assert fsm.state == HostState.CARRY_TO_DEST and fsm.place_tries == 1
