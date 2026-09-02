"""Host 주도 Pi 미션 FSM의 계약을 고정한다 (팀 확정, 2026-08-26).

여기서 지키려는 성질은 하나로 요약된다 — **Pi는 명령을 실행하고 보고할 뿐,
스스로 정하지 않는다.** 상태 전이는 Host가 보낸 state가 만들고, 주행은
Host가 보낸 속도가 만든다. 예외는 GRASP/INSERT를 실행한 뒤 그 결과로
넘어가는 두 자리뿐이다.
"""

import threading

import pytest

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.fake_host_link import FakeHostLink, FakeLidar
from domain.adapters.fake.scripted_perception import ScriptedPerception
from domain.ports.baseline_ports import BasketFace, HostCommand, MissionState, Report
from domain.task import baseline_constants as bc
from domain.task.baseline_mission import (
    BaselineApproachState,
    BaselineCarryState,
    BaselineGraspState,
    BaselineIdleState,
    BaselineInsertState,
    BaselinePorts,
    LinkWatchdog,
    plan_for_label,
)
from domain.task.floor_grasp_policy import GRIPPER_GRASP_MIN_MM
from domain.task.motion import AGREED_LINEAR_MPS, AGREED_ROTATION_RAD_S
from domain.values import TargetObservation


EMPTY_LOAD = 0.03      # FakeArm.LOAD_EMPTY — 빈 그리퍼 실측 분포 안의 값
HOLDING_LOAD = 0.14    # FakeArm.LOAD_HOLDING

# 정렬 판정이 통과하려면 턱 선을 알아야 한다 — 실측 전이라 테스트에서 주입한다.
JAW_LINE_M = 0.36
SERVO1_REACH_MM = 240.0


@pytest.fixture(autouse=True)
def _measured_geometry(monkeypatch):
    """턱 선과 팔 길이를 실측값 자리에 넣어 준다.

    실기에서 이 둘이 None인 동안에는 정렬 판정이 전부 Host로 넘어간다 —
    그 동작도 아래에서 따로 검증한다."""
    monkeypatch.setattr(bc, "JAW_LINE_DEPTH_FORWARD_M",
                        {**bc.JAW_LINE_DEPTH_FORWARD_M, "queen": JAW_LINE_M})
    # 좌우 영점은 클래스별이다. 테스트 물체는 영점 0으로 둬서 읽은 값이
    # 그대로 중심선 기준 오차가 되게 한다.
    monkeypatch.setattr(bc, "DEPTH_LATERAL_TO_JAW_CENTER_M",
                        {**bc.DEPTH_LATERAL_TO_JAW_CENTER_M,
                         "queen": 0.0, "바나나": 0.0})
    monkeypatch.setattr(bc, "SERVO1_AXIS_TO_JAW_MM", SERVO1_REACH_MM)


def _centered(label="queen"):
    """턱 쓸기 구간 한가운데, 좌우 정렬된 관측."""
    return ScriptedPerception(script=[TargetObservation(label, JAW_LINE_M + 0.02, 0.0, True)])


def _ports(host=None, base=None, arm=None, perception=None, lidar=None, estop=None):
    return BaselinePorts(
        base=base or FakeBase(),
        arm=arm or FakeArm(load_ratio=EMPTY_LOAD),
        perception=perception or ScriptedPerception(),
        host=host or FakeHostLink(),
        lidar=lidar or FakeLidar(),
        estop=estop or threading.Event(),
        watchdog=LinkWatchdog(),
    )


def _good_face(distance_m=None):
    """2026-08-26 검증 지점 수준의 정상 관측."""
    return BasketFace(True, distance_m or bc.BASKET_STOP_LIDAR_M, 0.01,
                      "정면 확보", point_count=97,
                      lateral_offset_m=0.0, lateral_known=False)


def _carry_with_previous(label="queen", face=None, load=HOLDING_LOAD):
    """직전 사이클 표본을 이미 들고 있는 CARRY 상태.

    안정성 검사가 표본 비교라, 한 사이클만 돌리는 테스트는 이걸 써야
    "직전 판독이 없다"에 걸리지 않는다."""
    return BaselineCarryState(label, MissionState.CARRY,
                              (face or _good_face(), load))


# ── 명령 실행 ──────────────────────────────────────────────────────────────


def test_APPROACH는_Host가_준_속도를_그대로_낸다():
    base = FakeBase()
    host = FakeHostLink([HostCommand(MissionState.APPROACH, linear_x=0.1)])
    ports = _ports(host=host, base=base)

    BaselineApproachState().execute(ports)

    assert base.last_velocity == (0.1, 0.0, 0.0)


def test_합의보다_빠른_속도는_잘라서_낸다():
    """Host 버그나 패킷 손상이 그대로 바퀴로 가면 안 된다."""
    base = FakeBase()
    host = FakeHostLink([HostCommand(MissionState.APPROACH, linear_x=1.0, linear_y=-2.0)])
    ports = _ports(host=host, base=base)

    BaselineApproachState().execute(ports)

    assert base.last_velocity == (AGREED_LINEAR_MPS, -AGREED_LINEAR_MPS, 0.0)


def test_제자리회전은_합의_속도로_잘린다():
    base = FakeBase()
    host = FakeHostLink([HostCommand(MissionState.APPROACH, angular_z=9.0)])
    ports = _ports(host=host, base=base)

    BaselineApproachState().execute(ports)

    assert base.last_velocity == (0.0, 0.0, AGREED_ROTATION_RAD_S)


def test_제자리정지는_다른_필드를_무시한다():
    base = FakeBase()
    host = FakeHostLink([HostCommand(MissionState.APPROACH, linear_x=0.1, stop=True)])
    ports = _ports(host=host, base=base)

    BaselineApproachState().execute(ports)

    assert base.velocity_calls == []
    assert base.stop_calls >= 1


def test_회전과_병진이_섞인_명령은_거부하고_되돌려준다():
    """추측해서 하나를 고르면 Host는 자기가 뭘 잘못 보냈는지 영영 모른다."""
    base = FakeBase()
    host = FakeHostLink([HostCommand(MissionState.APPROACH, linear_x=0.1, angular_z=0.25)])
    ports = _ports(host=host, base=base)

    BaselineApproachState().execute(ports)

    assert base.velocity_calls == []
    assert base.stop_calls >= 1
    assert Report.REJECTED in host.reported_kinds


# ── 임무 1번: 상태 보고 ────────────────────────────────────────────────────


def test_매_사이클_현재_state를_보고한다():
    host = FakeHostLink([HostCommand(MissionState.APPROACH)])
    ports = _ports(host=host)

    BaselineApproachState().execute(ports)

    assert (Report.STATE, MissionState.APPROACH, "", None) in host.reports


def test_Host가_APPROACH_BOX를_부르면_그_이름으로_보고한다():
    host = FakeHostLink([HostCommand(MissionState.APPROACH_BOX)])
    ports = _ports(host=host)

    nxt = BaselineCarryState("queen").execute(ports)

    assert MissionState.APPROACH_BOX in host.reported_states
    assert nxt.reported_as == MissionState.APPROACH_BOX


def test_CARRY_중에도_IDLE_명령을_받으면_IdleState로_돌아간다():
    """10:06 실기 재현 — 회귀 방지.

    BaselineIdleState/BaselineApproachState는 둘 다 IDLE 명령을 받으면
    IdleState로 돌아가는데, CarryState만 그 분기가 없어서 `return self`로
    빠져 제자리에 갇혔다. run_mission.py는 미션을 중간에 멈출 때(사용자가
    Enter/q로 끌 때) DONE이 아니라 "stop"+SEARCH_TARGET(-> 전선에서는
    IDLE)을 보낸다 — 그 순간 Pi가 CARRY나 APPROACH_BOX(바구니 접근)
    어딘가에 있었으면 그 자리에 영영 갇히고, 다음 미션이 새로 APPROACH/
    GRASP를 보내도 Pi는 여전히 CarryState라 못 알아듣는다(GRASP가 영원히
    대기하는 락업)."""
    host = FakeHostLink([HostCommand(MissionState.IDLE, stop=True)])
    ports = _ports(host=host)

    nxt = BaselineCarryState("queen", MissionState.APPROACH_BOX).execute(ports)

    assert isinstance(nxt, BaselineIdleState)


# ── APPROACH_BOX 접근 중 실시간 라이다 점검 (2026-09-02) ───────────────────
#
# NUDGE_BOX가 Host 계획 거리(want_m)를 다 밀 때까지 라이다를 안 보고 있다가
# PLACE에서야 확인해서 늦었던 사고(09-02 실기 2건)의 재발 방지. 접근 중에도
# 매 사이클 확인해서 너무 가까우면 더 밀지 않고, 이미 알맞으면 그만 민다.


def test_접근_중_라이다가_하한보다_가까우면_더_밀지_않는다():
    """Host가 계속 go를 보내도, 이미 너무 가까우면 Pi가 그 명령을 무시하고
    정지한다 — 바퀴를 실제로 돌리는 쪽이 최종 안전판이다."""
    too_close_face = _good_face(bc.BASKET_MIN_LIDAR_M - 0.01)
    base = FakeBase()
    host = FakeHostLink([HostCommand(MissionState.APPROACH_BOX, linear_x=0.1)])
    ports = _ports(host=host, base=base, lidar=FakeLidar([too_close_face]))

    BaselineCarryState("queen", MissionState.APPROACH_BOX).execute(ports)

    assert base.velocity_calls == [], "이미 너무 가까운데 계속 밀었다"
    assert base.stop_calls >= 1
    assert Report.INSERT_BLOCKED in host.reported_kinds


def test_접근_중_너무_가까우면_물러나는_보정을_같이_보낸다():
    too_close_face = _good_face(bc.BASKET_MIN_LIDAR_M - 0.01)
    host = FakeHostLink([HostCommand(MissionState.APPROACH_BOX, linear_x=0.1)])
    ports = _ports(host=host, lidar=FakeLidar([too_close_face]))

    BaselineCarryState("queen", MissionState.APPROACH_BOX).execute(ports)

    fixes = host.reported_fixes
    assert fixes, "보정 없이 막기만 하면 Host가 고칠 방법을 모른다"
    report, fix = fixes[0]
    assert report == Report.INSERT_BLOCKED
    assert fix.action == "retreat"


def test_접근_중_이미_목표창_안이면_그만_밀고_알린다():
    good_face = _good_face(bc.BASKET_STOP_LIDAR_M)
    base = FakeBase()
    host = FakeHostLink([HostCommand(MissionState.APPROACH_BOX, linear_x=0.1)])
    ports = _ports(host=host, base=base, lidar=FakeLidar([good_face]))

    BaselineCarryState("queen", MissionState.APPROACH_BOX).execute(ports)

    assert base.velocity_calls == [], "이미 목표창 안인데 계획한 거리를 마저 밀었다"
    assert base.stop_calls >= 1
    assert Report.APPROACH_BOX_READY in host.reported_kinds


def test_접근_중_창_밖이면_평소대로_계속_민다():
    far_face = _good_face(bc.BASKET_STOP_LIDAR_M + bc.BASKET_STOP_TOLERANCE_M + 0.1)
    base = FakeBase()
    host = FakeHostLink([HostCommand(MissionState.APPROACH_BOX, linear_x=0.1)])
    ports = _ports(host=host, base=base, lidar=FakeLidar([far_face]))

    BaselineCarryState("queen", MissionState.APPROACH_BOX).execute(ports)

    assert base.velocity_calls, "창 밖인데 안 밀었다"
    assert Report.APPROACH_BOX_READY not in host.reported_kinds
    assert Report.INSERT_BLOCKED not in host.reported_kinds


def test_접근_중_라이다_관측_실패면_평소대로_계속_민다():
    """모르면 실패 원칙 — 못 잰다고 멈추면 오히려 INSERT까지 영영 못 간다.
    관측 실패는 PLACE의 check_insert가 REACQUIRE로 다루는 것과 같은 이유로
    여기서는 그냥 넘어가고 평소 주행을 유지한다."""
    base = FakeBase()
    host = FakeHostLink([HostCommand(MissionState.APPROACH_BOX, linear_x=0.1)])
    ports = _ports(host=host, base=base)  # 기본 FakeLidar = 관측 실패

    BaselineCarryState("queen", MissionState.APPROACH_BOX).execute(ports)

    assert base.velocity_calls, "관측 실패인데 안 밀었다"


# ── 링크 워치독 ────────────────────────────────────────────────────────────


def test_Host_명령이_계속_없으면_멈춘다():
    """None은 '정지'가 아니라 '모른다'다 — 마지막 명령대로 계속 굴러가면 안 된다."""
    base = FakeBase()
    host = FakeHostLink([None])
    ports = _ports(host=host, base=base)

    state = BaselineApproachState()
    for _ in range(bc.HOST_COMMAND_TIMEOUT_CYCLES):
        state = state.execute(ports)

    assert base.velocity_calls == []
    assert Report.REJECTED in host.reported_kinds


# ── 임무 2번: GRASP 조건 판정 ──────────────────────────────────────────────


def test_조건이_충족되면_GRASP_READY를_보고하고_넘어간다():
    host = FakeHostLink([HostCommand(MissionState.GRASP, stop=True)])
    ports = _ports(host=host, perception=_centered())

    nxt = BaselineApproachState().execute(ports)

    assert Report.GRASP_READY in host.reported_kinds
    assert isinstance(nxt, BaselineGraspState)
    assert nxt.label == "queen"
    # 관측 전방거리 - 턱 선 + GRASP_CREEP_EXTRA_MM(2026-09-02 사용자 지시)
    assert nxt.creep_m == pytest.approx(0.02 + bc.GRASP_CREEP_EXTRA_MM / 1000.0)


def test_그리퍼가_비어있지_않으면_GRASP를_막고_제자리에_머문다():
    """물고 있는 것을 떨어뜨리는 사고를 막는다."""
    host = FakeHostLink([HostCommand(MissionState.GRASP, stop=True)])
    ports = _ports(host=host, arm=FakeArm(load_ratio=HOLDING_LOAD))

    nxt = BaselineApproachState().execute(ports)

    assert Report.GRASP_BLOCKED in host.reported_kinds
    assert isinstance(nxt, BaselineApproachState)


def test_자기_카메라가_목표를_못_보면_내려가지_않는다():
    host = FakeHostLink([HostCommand(MissionState.GRASP, stop=True)])
    ports = _ports(host=host, perception=ScriptedPerception(label=None))

    nxt = BaselineApproachState().execute(ports)

    assert Report.GRASP_BLOCKED in host.reported_kinds
    assert isinstance(nxt, BaselineApproachState)


def test_교시_자세가_없는_라벨이면_내려가지_않는다():
    host = FakeHostLink([HostCommand(MissionState.GRASP, stop=True)])
    ports = _ports(host=host, perception=ScriptedPerception(script=[TargetObservation("바나나", JAW_LINE_M + 0.02, 0.0, True)]))

    nxt = BaselineApproachState().execute(ports)

    assert Report.GRASP_BLOCKED in host.reported_kinds
    assert isinstance(nxt, BaselineApproachState)


def test_차체가_움직이는_중이면_GRASP를_막는다():
    host = FakeHostLink([HostCommand(MissionState.GRASP, linear_x=0.1)])
    ports = _ports(host=host, perception=_centered())

    nxt = BaselineApproachState().execute(ports)

    assert Report.GRASP_BLOCKED in host.reported_kinds
    assert isinstance(nxt, BaselineApproachState)


# ── 임무 3번: GRASP 수행 ───────────────────────────────────────────────────


def test_파지에_성공하면_CARRY로_가고_완료를_보고한다():
    host = FakeHostLink()
    arm = FakeArm(load_ratio=HOLDING_LOAD)
    ports = _ports(host=host, arm=arm)

    nxt = BaselineGraspState("queen", 0.02).execute(ports)

    assert Report.GRASP_DONE in host.reported_kinds
    assert isinstance(nxt, BaselineCarryState)


def test_파지_후_IDLE이_아니라_CARRY로_접는다():
    """물체를 문 채 IDLE로 접으면 그리퍼가 라이다 정면을 가린다(2026-08-26 실측)."""
    arm = FakeArm(load_ratio=HOLDING_LOAD)
    ports = _ports(arm=arm)

    BaselineGraspState("queen", 0.02).execute(ports)

    stages = [stage for _profile, stage in arm.floor_pose_calls]
    assert "carry" in stages
    assert "idle" not in stages


def test_파지에_실패하면_APPROACH로_돌아가고_스스로_재시도하지_않는다():
    host = FakeHostLink()
    ports = _ports(host=host, arm=FakeArm(load_ratio=0.0))

    nxt = BaselineGraspState("queen", 0.02).execute(ports)

    assert Report.GRASP_FAILED in host.reported_kinds
    assert isinstance(nxt, BaselineApproachState)


def test_들어올린_직후_부하가_한_계단_떨어져도_재확인에서_넘기면_성공한다():
    """09-02 10:41 실기 재현 — 회귀 방지.

    LOAD_THRESHOLD 자체가 "빈손 11/256"과 "파지 13/256"의 중점이라 두
    계단 사이 틈이 코드 값 하나(약 0.0039)뿐이다. 실기에서 그리퍼를
    막 닫고 잰 값(0.0508, 13/256 대역)은 threshold를 넘겼는데, 곧바로
    이어진 midpoint 이동 직후 다시 잰 값(0.0430, 11/256 대역)만 그 틈
    하나만큼 밑돌아 "들어 올리지 못함"으로 실패 처리됐다 — 사용자가
    직접 옆에서 파지 성공을 확인한 자리였다."""
    host = FakeHostLink()
    arm = FakeArm(load_ratio=[0.0508, 0.0430, 0.0508, 0.0508])
    ports = _ports(host=host, arm=arm)

    nxt = BaselineGraspState("queen", 0.02).execute(ports)

    assert Report.GRASP_DONE in host.reported_kinds
    assert isinstance(nxt, BaselineCarryState)


def test_재확인에서도_부하가_낮으면_그때는_진짜_실패다():
    """재시도 한 번으로 진짜 실패까지 가려 버리면 안 된다 — 재확인마저
    낮으면 여전히 실패로 본다."""
    host = FakeHostLink()
    arm = FakeArm(load_ratio=[0.0508, 0.0430, 0.0430])
    ports = _ports(host=host, arm=arm)

    nxt = BaselineGraspState("queen", 0.02).execute(ports)

    assert Report.GRASP_FAILED in host.reported_kinds
    assert isinstance(nxt, BaselineApproachState)


def test_파지_명령_폭은_ros2_프로파일_공식에서_나온다():
    """도메인과 ros2 프로파일이 갈라져 파지가 헐거워진 2026-08-26 사고 방지."""
    assert plan_for_label("queen").close_width_mm == 7.0
    # rook은 공식값(9.5)이 아니라 2026-09-02 지시로 GRIPPER_GRASP_MIN_MM까지
    # 덮어쓴 값이다 — 09-02 실기에서 이 폭(9.5)이 knight 실측 스윕의
    # "9.0mm 명령 -> 부하 0.0235"와 같은 헐거운 대역이었다
    # (baseline_mission._CLOSE_WIDTH_OVERRIDE_MM 참고).
    assert plan_for_label("rook").close_width_mm == GRIPPER_GRASP_MIN_MM
    assert plan_for_label("soccer").close_width_mm == 31.0


# ── 임무 4번: INSERT 조건 판정과 수행 ──────────────────────────────────────


def test_라이다가_정면을_잡고_거리가_맞으면_INSERT로_간다():
    host = FakeHostLink([HostCommand(MissionState.INSERT, stop=True)])
    ports = _ports(host=host, arm=FakeArm(load_ratio=HOLDING_LOAD),
                   lidar=FakeLidar([_good_face()]))

    nxt = _carry_with_previous().execute(ports)

    assert Report.INSERT_READY in host.reported_kinds
    assert isinstance(nxt, BaselineInsertState)


def test_라이다가_정면을_못_잡으면_INSERT를_막는다():
    """모르면 실패 — 팔을 크게 전개하는 동작이라 막는 쪽이 싸다."""
    host = FakeHostLink([HostCommand(MissionState.INSERT, stop=True)])
    ports = _ports(host=host, arm=FakeArm(load_ratio=HOLDING_LOAD), lidar=FakeLidar())

    nxt = _carry_with_previous().execute(ports)

    assert Report.INSERT_BLOCKED in host.reported_kinds
    assert isinstance(nxt, BaselineCarryState)


def test_바구니가_절벽보다_가까우면_INSERT를_막는다():
    """판독이 하한 아래면 테두리를 넘겨보고 있을 수 있다."""
    host = FakeHostLink([HostCommand(MissionState.INSERT, stop=True)])
    close = _good_face(bc.BASKET_MIN_LIDAR_M - 0.005)
    ports = _ports(host=host, arm=FakeArm(load_ratio=HOLDING_LOAD), lidar=FakeLidar([close]))

    nxt = _carry_with_previous(face=close).execute(ports)

    assert Report.INSERT_BLOCKED in host.reported_kinds
    assert isinstance(nxt, BaselineCarryState)


def test_빈손이면_INSERT를_막는다():
    host = FakeHostLink([HostCommand(MissionState.INSERT, stop=True)])
    ports = _ports(host=host, arm=FakeArm(load_ratio=0.0), lidar=FakeLidar([_good_face()]))

    nxt = _carry_with_previous(load=0.0).execute(ports)

    assert Report.INSERT_BLOCKED in host.reported_kinds


def test_투하_후_부하가_줄면_성공으로_보고하고_IDLE로_돌아간다():
    host = FakeHostLink()
    arm = FakeArm(load_ratio=[0.0626, 0.0313])
    ports = _ports(host=host, arm=arm)

    nxt = BaselineInsertState("queen").execute(ports)

    assert Report.INSERT_DONE in host.reported_kinds
    assert Report.IDLE_DONE in host.reported_kinds
    assert isinstance(nxt, BaselineIdleState)


def test_부하가_안_줄면_실패로_보고한다():
    host = FakeHostLink()
    arm = FakeArm(load_ratio=0.0626)
    ports = _ports(host=host, arm=arm)

    BaselineInsertState("queen").execute(ports)

    assert Report.INSERT_FAILED in host.reported_kinds


def test_접기_전에_그리퍼를_닫는다():
    """벌린 채로 접으면 손가락이 차체에 걸린다(2026-08-25 사용자 지시)."""
    arm = FakeArm(load_ratio=[0.0626, 0.0313])
    ports = _ports(arm=arm)

    BaselineInsertState("queen").execute(ports)

    widths = arm.gripper_widths
    stages = [stage for _profile, stage in arm.floor_pose_calls]
    assert widths[-1] == 9.0
    assert stages[-1] == "idle"


# ── E-STOP ────────────────────────────────────────────────────────────────
#
# ⚠️ 2026-09-01 사용자 지시로 check_grasp()가 더 이상 estop_set을 보지
# 않는다(preconditions.check_grasp 문서 참고) — E-STOP은 이제 오롯이
# `BaselineMission.run()`의 최상위 검사(사이클마다 상태 실행 전에 먼저
# 보고 ESTOP이면 BaselineEstopState로 갈아친다) 하나로만 막는다. 여기
# 있던 테스트는 `BaselineApproachState().execute()`를 직접 불러 그
# run() 개입을 건너뛰므로, 이 레이어에서 검증할 계약이 더는 없다 —
# 지웠다.


# ── 정렬 판정 (사용자 지시 2026-08-26) ────────────────────────────────────


def test_영역_안에서_치우쳐도_그대로_내려간다():
    """2026-09-01 사용자 지시로 PI_CENTER(servo 1 미세 보정)를 없앴다 —
    턱 폭 안이면 가운데가 아니어도 servo 1을 건드리지 않고 곧장 파지로
    간다(grasp_alignment.judge()/baseline_mission._judge_alignment 주석
    참고)."""
    host = FakeHostLink([HostCommand(MissionState.GRASP, stop=True)])
    arm = FakeArm(load_ratio=EMPTY_LOAD)
    perception = ScriptedPerception(
        script=[TargetObservation("queen", JAW_LINE_M + 0.02, 0.040, True)])
    ports = _ports(host=host, arm=arm, perception=perception)

    nxt = BaselineApproachState().execute(ports)

    assert Report.GRASP_READY in host.reported_kinds
    assert arm.yaw_offsets == []               # servo 1 을 아예 건드리지 않는다
    assert isinstance(nxt, BaselineGraspState)


def test_보정_후_곧장_내려가지_않는다():
    """관측 -> 소이동 -> 재관측 폐루프. 열린 루프는 오차가 쌓인다."""
    host = FakeHostLink([HostCommand(MissionState.GRASP, stop=True)])
    arm = FakeArm(load_ratio=EMPTY_LOAD)
    perception = ScriptedPerception(
        script=[TargetObservation("queen", JAW_LINE_M + 0.02, 0.040, True)])
    ports = _ports(host=host, arm=arm, perception=perception)

    BaselineApproachState().execute(ports)

    assert arm.floor_pose_calls == []


def test_턱_폭_밖이면_Host에_재회전을_요구한다():
    host = FakeHostLink([HostCommand(MissionState.GRASP, stop=True)])
    perception = ScriptedPerception(
        script=[TargetObservation("queen", JAW_LINE_M + 0.02, 0.090, True)])
    ports = _ports(host=host, arm=FakeArm(load_ratio=EMPTY_LOAD), perception=perception)

    nxt = BaselineApproachState().execute(ports)

    assert Report.GRASP_BLOCKED in host.reported_kinds
    assert "재회전" in host.reports[-1][2]
    assert isinstance(nxt, BaselineApproachState)


def test_전진_거리_밖이면_Host에_재직진을_요구한다():
    host = FakeHostLink([HostCommand(MissionState.GRASP, stop=True)])
    far = JAW_LINE_M + bc.GRASP_CREEP_FORWARD_MM / 1000.0 + 0.05
    perception = ScriptedPerception(script=[TargetObservation("queen", far, 0.0, True)])
    ports = _ports(host=host, arm=FakeArm(load_ratio=EMPTY_LOAD), perception=perception)

    BaselineApproachState().execute(ports)

    assert Report.GRASP_BLOCKED in host.reported_kinds
    assert "재직진" in host.reports[-1][2]


def test_거리_환산에_실패하면_내려가지_않는다():
    """metric_ok=False의 0.0을 그대로 쓰면 '바로 앞 정중앙'으로 읽힌다."""
    host = FakeHostLink([HostCommand(MissionState.GRASP, stop=True)])
    perception = ScriptedPerception(
        script=[TargetObservation("queen", 0.0, 0.0, False)])
    ports = _ports(host=host, arm=FakeArm(load_ratio=EMPTY_LOAD), perception=perception)

    nxt = BaselineApproachState().execute(ports)

    assert Report.GRASP_BLOCKED in host.reported_kinds
    assert isinstance(nxt, BaselineApproachState)


def test_턱_선_미실측이면_정렬_판정을_포기하고_Host에_넘긴다(monkeypatch):
    """실측 전 오늘의 동작 — 지어낸 프레임 변환으로 팔을 내리지 않는다."""
    monkeypatch.setattr(bc, "JAW_LINE_DEPTH_FORWARD_M", {})
    host = FakeHostLink([HostCommand(MissionState.GRASP, stop=True)])
    ports = _ports(host=host, arm=FakeArm(load_ratio=EMPTY_LOAD), perception=_centered())

    nxt = BaselineApproachState().execute(ports)

    assert Report.GRASP_BLOCKED in host.reported_kinds
    assert isinstance(nxt, BaselineApproachState)


# ── 강제 파지 (Host 지시, 2026-08-31) ───────────────────────────────────────
#
# Host 가 재정렬을 GRASP_ALIGN_MAX_TRIES 훨씬 넘게 반복해도 계속 영역
# 밖이면, mission_config.GRASP_FORCE_AFTER_TRIES 문턱에서 MissionState.
# GRASP_FORCE 로 한 번 강제 진행한다. 여기서 지키려는 성질: **정렬 창
# (HOST_CORRECTION)만 건너뛴다, 기본 전제와 UNKNOWN(위치를 아예 모름)은
# 절대 안 건너뛴다.**


def test_GRASP_FORCE는_턱_폭_밖이어도_내려간다():
    """정상 GRASP 라면 재회전을 요구했을 관측인데, FORCE 는 그대로 진행한다."""
    host = FakeHostLink([HostCommand(MissionState.GRASP_FORCE, stop=True)])
    perception = ScriptedPerception(
        script=[TargetObservation("queen", JAW_LINE_M + 0.02, 0.090, True)])
    ports = _ports(host=host, arm=FakeArm(load_ratio=EMPTY_LOAD), perception=perception)

    nxt = BaselineApproachState().execute(ports)

    assert Report.GRASP_READY in host.reported_kinds
    assert "강제" in host.reports[-1][2]
    assert isinstance(nxt, BaselineGraspState)


def test_GRASP_FORCE도_위치를_아예_모르면_안_내려간다():
    """UNKNOWN(거리 환산 실패)은 force 로도 못 건너뛴다 — 어디 있는지조차 모른다."""
    host = FakeHostLink([HostCommand(MissionState.GRASP_FORCE, stop=True)])
    perception = ScriptedPerception(script=[TargetObservation("queen", 0.0, 0.0, False)])
    ports = _ports(host=host, arm=FakeArm(load_ratio=EMPTY_LOAD), perception=perception)

    nxt = BaselineApproachState().execute(ports)

    assert Report.GRASP_BLOCKED in host.reported_kinds
    assert isinstance(nxt, BaselineApproachState)


def test_GRASP_FORCE도_기본_전제는_안_건너뛴다():
    """force는 `_judge_alignment`(2단계, 정렬 창)만 건너뛴다 — `check_grasp`
    (1단계)은 force와 무관하게 항상 본다. 애초에 목표를 못 본 것까지
    강제로 내려가게 하지는 않는다."""
    host = FakeHostLink([HostCommand(MissionState.GRASP_FORCE, stop=True)])
    ports = _ports(host=host, arm=FakeArm(load_ratio=EMPTY_LOAD),
                   perception=ScriptedPerception())  # 아무것도 안 보임

    nxt = BaselineApproachState().execute(ports)

    assert Report.GRASP_BLOCKED in host.reported_kinds
    assert isinstance(nxt, BaselineApproachState)


def test_GRASP_FORCE도_영역_안이면_그냥_평소대로_내려간다():
    """이미 READY 인 경우엔 force 문구가 안 붙는다 — 진짜로 강제한 경우만 표시."""
    host = FakeHostLink([HostCommand(MissionState.GRASP_FORCE, stop=True)])
    ports = _ports(host=host, arm=FakeArm(load_ratio=EMPTY_LOAD), perception=_centered())

    nxt = BaselineApproachState().execute(ports)

    assert Report.GRASP_READY in host.reported_kinds
    assert "강제" not in host.reports[-1][2]
    assert isinstance(nxt, BaselineGraspState)


# ── 두 신호 파지 판정 (사용자 지시 2026-08-26, 2026-09-01 AND -> OR) ────────
#
# 2026-08-26 ~ 2026-09-01: 부하와 뎁스(confirm_grasp) 둘 다 있어야 성공
# (AND)이었다. 2026-09-01 실기에서 CARRY 자세는 카메라 프레임 밖이 맞는데도
# confirm_grasp() 가 "그대로 있다"를 반환했다(뎁스 오탐) — 정상적으로 문턱을
# 넘은 부하(0.0547)가 그 오탐 하나 때문에 막혔다. 사용자 지시로 **둘 중
# 하나만 있어도 성공**으로 바꿨다 — 둘 다 실패를 가리킬 때만 진짜 실패다.


def test_부하만_높아도_뎁스_오탐과_무관하게_성공이다():
    """뎁스가 "그대로 있다"고 해도 부하가 충분하면 성공이다(OR)."""
    host = FakeHostLink()
    perception = ScriptedPerception(grasp_confirmed=False)
    ports = _ports(host=host, arm=FakeArm(load_ratio=HOLDING_LOAD), perception=perception)

    nxt = BaselineGraspState("queen", 0.02).execute(ports)

    assert Report.GRASP_DONE in host.reported_kinds
    assert "뎁스 오탐 무시" in host.reports[-1][2]
    assert isinstance(nxt, BaselineCarryState)


def test_부하가_거의_없으면_뎁스가_뭐라든_실패다():
    """들어올리기 자체를 못 하면(부하 0.0) OR 판정까지 갈 필요도 없이
    실패다 — 뎁스만으로 성공을 만들어내지 않는다는 하한선."""
    host = FakeHostLink()
    ports = _ports(host=host, arm=FakeArm(load_ratio=0.0),
                   perception=ScriptedPerception(grasp_confirmed=True))

    nxt = BaselineGraspState("queen", 0.02).execute(ports)

    assert Report.GRASP_FAILED in host.reported_kinds
    assert isinstance(nxt, BaselineApproachState)


def test_두_신호가_모두_있으면_당연히_성공이다():
    host = FakeHostLink()
    ports = _ports(host=host, arm=FakeArm(load_ratio=HOLDING_LOAD),
                   perception=ScriptedPerception(grasp_confirmed=True))

    nxt = BaselineGraspState("queen", 0.02).execute(ports)

    assert Report.GRASP_DONE in host.reported_kinds
    assert "목표 사라짐 확인" in host.reports[-1][2]
    assert isinstance(nxt, BaselineCarryState)


# ── 미세 전진 시점 (2026-08-29) ────────────────────────────────────────────
#
# 이 전진의 목적은 "물체 가까이 가는 것"이 아니라 **물체를 벌어진 턱 사이로
# 밀어 넣는 것**이다(사용자 설명 2026-08-26). 그래야 평행 턱의 넓은 목이
# 좌우 자기정렬 효과를 낸다.
#
# 2026-08-29까지 코드는 전진을 **먼저** 하고 팔을 나중에 내렸다 — 밀어 넣는
# 것이 아니라 물체 위로 내려가 감싸는 동작이었다. 최초 커밋(241003a) 이후
# 아무도 안 건드린 자리인데, 실기로 검증된 tools/demo_rook_run.py 는 처음부터
# 팔을 내린 뒤 전진했다(2단계 팔 내리기 -> 3단계 미세 전진).
#
# 순서는 문서 세 곳이 이미 옳게 적고 있었는데도 코드만 달랐다. 그래서
# 문자열이 아니라 **실제 호출 순서**로 고정한다.


class _OrderSpy:
    """포트 호출을 한 줄로 엮어 순서를 본다.

    포트가 여럿이라 각자의 호출 목록만 봐서는 서로의 앞뒤를 알 수 없다 —
    바로 그 틈에서 이 결함이 살아남았다."""

    def __init__(self, delegate, name, log):
        self._delegate, self._name, self._log = delegate, name, log

    def __getattr__(self, method):
        attribute = getattr(self._delegate, method)
        if not callable(attribute):
            return attribute

        def recorded(*args, **kwargs):
            if method in ("move_to_floor_pose", "set_gripper",
                          "creep_forward_timed", "remember_target"):
                detail = args[1] if method == "move_to_floor_pose" else None
                self._log.append(f"{method}:{detail}" if detail else method)
            return attribute(*args, **kwargs)

        return recorded


def _grasp_call_order():
    from domain.task.baseline_mission import BaselineGraspState

    log = []
    arm = FakeArm(load_ratio=HOLDING_LOAD)
    base = FakeBase()
    perception = ScriptedPerception()
    ports = BaselinePorts(
        base=_OrderSpy(base, "base", log),
        arm=_OrderSpy(arm, "arm", log),
        perception=_OrderSpy(perception, "perception", log),
        host=FakeHostLink(), lidar=FakeLidar(), estop=threading.Event(),
    )
    BaselineGraspState("queen", 0.030).execute(ports)
    return log


def test_미세_전진은_팔이_내려가_그리퍼가_열린_뒤에_일어난다():
    """전진이 grasp 자세 도달 **뒤**여야 물체가 턱 사이로 들어온다."""
    order = _grasp_call_order()

    assert "creep_forward_timed" in order, "미세 전진이 아예 안 일어났다"
    creep = order.index("creep_forward_timed")
    descended = order.index("move_to_floor_pose:grasp")
    opened = order.index("set_gripper")

    assert opened < descended, "내려가기 전에 열어야 한다(사용자 지시 2026-08-24)"
    assert descended < creep, (
        f"전진이 하강보다 먼저다 — 밀어 넣는 것이 아니라 감싸는 동작이 된다\n"
        f"실제 순서: {order}")


def test_전진은_그리퍼를_닫기_전에_끝난다():
    """턱 사이에 물체가 들어오기 전에 닫으면 빈손으로 물거나 물체를 친다."""
    order = _grasp_call_order()

    creep = order.index("creep_forward_timed")
    closes = [i for i, call in enumerate(order) if call == "set_gripper"]
    assert len(closes) >= 2, f"열기/닫기가 둘 다 있어야 한다: {order}"
    assert creep < closes[1], f"닫은 뒤에 전진했다: {order}"


def test_기준_프레임은_팔이_카메라를_가리기_전에_뜬다():
    """grasp 자세로 내려가면 팔이 뎁스 카메라를 가린다 — confirm_grasp 의
    기준 관측은 그 전에 떠야 한다(tools/demo_rook_run.py 2단계와 같은 이유)."""
    order = _grasp_call_order()

    assert order.index("remember_target") < order.index("move_to_floor_pose:grasp")


def test_전진_구간에_회전이_섞이지_않는다():
    """이 구간에서 그리퍼는 바닥 2.6cm 위에 열린 채 떠 있다. 제자리 회전은
    그것을 바닥과 물체를 가로질러 옆으로 쓴다 — 이 프로젝트의 확립된 안전
    규칙 위반이다(demo_rook_run.py 의 CREEP_KEYMAP 이 회전 키를 뺀 이유)."""
    from domain.task.baseline_mission import BaselineGraspState

    arm = FakeArm(load_ratio=HOLDING_LOAD)
    base = FakeBase()
    ports = BaselinePorts(
        base=base, arm=arm, perception=ScriptedPerception(),
        host=FakeHostLink(), lidar=FakeLidar(), estop=threading.Event(),
    )
    BaselineGraspState("queen", 0.030).execute(ports)

    for linear_x, linear_y, angular_z in base.velocity_calls:
        assert angular_z == 0.0, f"파지 중 회전 명령이 나갔다: {base.velocity_calls}"
        assert linear_y == 0.0, f"파지 중 횡이동 명령이 나갔다: {base.velocity_calls}"
