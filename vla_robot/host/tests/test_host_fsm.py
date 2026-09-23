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
    assert fsm.target_label == "queen" and fsm.dest_box == "chess"
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


def test_grasp_failure_skips_target(cfg):
    fsm = MissionFSM(cfg)
    _into_grasp(fsm, S())
    cmd = fsm.step(P(0.9, 0.6), PM, S(job_id=1, result=JobResult(1, State.GRASP, False, "empty")), 0.5)
    assert fsm.state == HostState.SEARCH_TARGET
    assert cmd.state == State.IDLE and cmd.stop
    assert len(fsm.skipped) == 1
    # 보류된 queen 대신 star 를 고른다
    fsm.step(P(0.9, 0.6), PM, S(job_id=1, result=JobResult(1, State.GRASP, False)), 0.6)
    assert fsm.target_label == "star"


def test_skip_expires(cfg):
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
    fsm.dest_box = "chess"
    fsm.dest_xy = fsm._box_front_xy("chess")
    fsm._enter(HostState.PLACE)


def _place_attempt_from_face(fsm, prev_status, t):
    """상자 정면까지 직진해서 붙고, 남는 각도는 팔이 맡는다(차체 정렬 없음)."""
    assert fsm.state == HostState.NUDGE_BOX
    cmd = fsm.step(P(1.35, 1.20), {}, prev_status, t)           # 아직 멀다 -> 전진
    assert cmd.state == State.APPROACH_BOX and cmd.linear_x > 0
    # dest_xy(1.350, 1.300) 에서 place_arrive_tol_m 안 = 정차 완료
    cmd = fsm.step(P(1.35, 1.29), {}, prev_status, t + 0.2)
    assert fsm.state == HostState.PLACE and cmd.state == State.PLACE


def test_place_failures_retry_then_halt(cfg):
    assert cfg.mission.place_retry_max == 2
    fsm = MissionFSM(cfg)
    _setup_place(fsm)
    assert fsm.step(P(1.35, 1.0), {}, S(), 0.0).state == State.PLACE
    s1 = S(job_id=1, result=JobResult(1, State.PLACE, False, "drop"))
    fsm.step(P(1.35, 1.0), {}, s1, 1.0)
    assert fsm.state == HostState.NUDGE_BOX and fsm.place_tries == 1

    _place_attempt_from_face(fsm, s1, 2.0)
    s2 = S(job_id=2, result=JobResult(2, State.PLACE, False, "drop"))
    fsm.step(P(1.35, 1.0), {}, s2, 3.0)
    assert fsm.state == HostState.NUDGE_BOX and fsm.place_tries == 2

    _place_attempt_from_face(fsm, s2, 4.0)
    cmd = fsm.step(P(1.35, 1.0), {}, S(job_id=3, result=JobResult(3, State.PLACE, False)), 5.0)
    assert fsm.state == HostState.HALTED and fsm.halt_reason
    assert cmd.state == State.IDLE and cmd.stop
    # 사람이 prev 를 누르면 상자 앞 정렬부터 다시
    fsm.request_back()
    fsm.step(P(1.35, 1.0, 0.0), {}, None, 6.0)
    assert fsm.state == HostState.NUDGE_BOX


def test_place_ok_returns_to_search(cfg):
    fsm = MissionFSM(cfg)
    _setup_place(fsm)
    fsm.step(P(1.35, 1.0), {}, S(), 0.0)
    fsm.step(P(1.35, 1.0), {}, S(job_id=1, result=JobResult(1, State.PLACE, True)), 1.0)
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
    # dest_xy 에서 2 cm 왼쪽으로 치우쳐 서고 정북을 본다(허용 거리 오차 안)
    cmd = fsm.step(P(1.33, 1.30, yaw=90.0), {}, S(), 0.0)
    assert fsm.state == HostState.PLACE
    assert cmd.state == State.PLACE and cmd.stop
    assert cmd.angular_z == 0                                   # 제자리 회전 없음
    # 목표가 내 오른쪽에 있으니 팔은 시계방향(-)으로 튼다. 한계 15도 안이다
    assert -15.0 < cmd.arm_yaw_deg < 0.0
    # 명령이 반복돼도 같은 각도가 계속 실린다
    assert fsm.step(P(1.33, 1.30, yaw=90.0), {}, S(), 0.1).arm_yaw_deg == cmd.arm_yaw_deg


def test_body_turns_only_when_the_arm_cannot_cover(cfg):
    """팔 한계(±15도)를 넘는 각도에서만 차체를 돌린다."""
    fsm = MissionFSM(cfg)
    _setup_place(fsm)
    fsm._enter(HostState.NUDGE_BOX)
    cmd = fsm.step(P(1.35, 1.30, yaw=40.0), {}, S(), 0.0)       # 정차는 했지만 50도 틀어짐
    assert fsm.state == HostState.NUDGE_BOX and cmd.angular_z != 0


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
