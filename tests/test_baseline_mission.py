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

    assert (Report.STATE, MissionState.APPROACH, "") in host.reports


def test_Host가_APPROACH_BOX를_부르면_그_이름으로_보고한다():
    host = FakeHostLink([HostCommand(MissionState.APPROACH_BOX)])
    ports = _ports(host=host)

    nxt = BaselineCarryState("queen").execute(ports)

    assert MissionState.APPROACH_BOX in host.reported_states
    assert nxt.reported_as == MissionState.APPROACH_BOX


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
    assert nxt.creep_m == pytest.approx(0.02)   # 관측 전방거리 - 턱 선


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


def test_파지_명령_폭은_ros2_프로파일_공식에서_나온다():
    """도메인과 ros2 프로파일이 갈라져 파지가 헐거워진 2026-08-26 사고 방지."""
    assert plan_for_label("queen").close_width_mm == 7.0
    assert plan_for_label("rook").close_width_mm == 9.5
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


def test_ESTOP이_걸리면_GRASP_조건_판정이_통과하지_않는다():
    estop = threading.Event()
    estop.set()
    host = FakeHostLink([HostCommand(MissionState.GRASP, stop=True)])
    ports = _ports(host=host, estop=estop)

    nxt = BaselineApproachState().execute(ports)

    assert Report.GRASP_BLOCKED in host.reported_kinds
    assert isinstance(nxt, BaselineApproachState)


# ── 정렬 판정 (사용자 지시 2026-08-26) ────────────────────────────────────


def test_영역_안에서_치우치면_Pi가_servo1로_고친다():
    """Host가 아니라 Pi가 고친다 — 차량 제어 원칙의 의도된 예외다."""
    host = FakeHostLink([HostCommand(MissionState.GRASP, stop=True)])
    arm = FakeArm(load_ratio=EMPTY_LOAD)
    perception = ScriptedPerception(
        script=[TargetObservation("queen", JAW_LINE_M + 0.02, 0.040, True)])
    ports = _ports(host=host, arm=arm, perception=perception)

    nxt = BaselineApproachState().execute(ports)

    assert Report.GRASP_CENTERING in host.reported_kinds
    assert len(arm.yaw_offsets) == 1
    assert arm.yaw_offsets[0] > 0.0            # 왼쪽으로 치우쳤으니 왼쪽으로 돈다
    assert isinstance(nxt, BaselineApproachState)   # 보정 후 다시 관측한다


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


def test_servo1이_거부하면_Host에_넘긴다():
    """한계각을 넘는 보정은 차량이 잘못 선 것이다."""
    host = FakeHostLink([HostCommand(MissionState.GRASP, stop=True)])
    arm = FakeArm(load_ratio=EMPTY_LOAD, yaw_offset_ok=False)
    perception = ScriptedPerception(
        script=[TargetObservation("queen", JAW_LINE_M + 0.02, 0.040, True)])
    ports = _ports(host=host, arm=arm, perception=perception)

    BaselineApproachState().execute(ports)

    assert Report.GRASP_BLOCKED in host.reported_kinds
    assert "재회전" in host.reports[-1][2]


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


# ── 두 신호 파지 판정 (사용자 지시 2026-08-26) ─────────────────────────────


def test_부하만_높고_목표가_남아있으면_실패로_본다():
    """턱끼리 물었거나 물체를 쳐 놓은 경우 — 부하 하나로는 못 가른다."""
    host = FakeHostLink()
    perception = ScriptedPerception(grasp_confirmed=False)
    ports = _ports(host=host, arm=FakeArm(load_ratio=HOLDING_LOAD), perception=perception)

    nxt = BaselineGraspState("queen", 0.02).execute(ports)

    assert Report.GRASP_FAILED in host.reported_kinds
    assert isinstance(nxt, BaselineApproachState)


def test_목표는_사라졌는데_부하가_없으면_실패로_본다():
    """내려오는 그리퍼가 물체를 쳐서 밀어낸 경우 — 뎁스 하나로는 못 가른다."""
    host = FakeHostLink()
    ports = _ports(host=host, arm=FakeArm(load_ratio=0.0),
                   perception=ScriptedPerception(grasp_confirmed=True))

    nxt = BaselineGraspState("queen", 0.02).execute(ports)

    assert Report.GRASP_FAILED in host.reported_kinds
    assert isinstance(nxt, BaselineApproachState)


def test_두_신호가_모두_있어야_성공이다():
    host = FakeHostLink()
    ports = _ports(host=host, arm=FakeArm(load_ratio=HOLDING_LOAD),
                   perception=ScriptedPerception(grasp_confirmed=True))

    nxt = BaselineGraspState("queen", 0.02).execute(ports)

    assert Report.GRASP_DONE in host.reported_kinds
    assert isinstance(nxt, BaselineCarryState)
