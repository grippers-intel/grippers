"""Host 명령만으로 미션을 처음부터 끝까지 굴려 본다 (통주행).

`test_baseline_mission.py`는 상태 하나하나의 계약을 따로 고정한다. 여기서
보려는 것은 그 사이의 **이음매**다 — 한 상태가 다음 상태에 넘겨주는 것이
실제로 맞물리는지는 단계별 테스트로는 안 드러난다. 실기 통합 당일에 나오는
사고가 대부분 이 자리에서 나온다.

특히 이 저장소가 실제로 겪은 두 가지를 여기서 잡는다.

  - **CARRY 표본의 이어달리기.** INSERT 판정의 "판독이 흔들리지 않는가"와
    "부하가 안 떨어지는가"는 직전 사이클 표본과 비교해서 나온다. 그 표본은
    `BaselineCarryState`가 자기 다음 인스턴스에 손으로 넘겨주는 값이라,
    전환이 한 번이라도 끊기면 조용히 None이 되어 판정이 미뤄진다.
  - **GRASP가 IDLE이 아니라 CARRY로 접는 것.** 물체를 문 채 IDLE로 접으면
    그리퍼가 라이다 정면을 79% 가려 바구니를 못 본다. 통주행이 아니면
    "접기는 성공했다"까지만 보이고 그 다음이 안 보인다.

Host 명령 대본은 `grippers_topview`의 `MissionFSM`이 실제로 보낼 순서를
그대로 옮긴 것이다 — 상태 이름 대응은 그 저장소 `vehicle_link._STATUS_TO_STATE`
와 이 저장소 `tools/host_link_conformance.py`가 같이 고정하고 있다.
"""

import threading

import pytest

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.fake_host_link import FakeHostLink, FakeLidar
from domain.adapters.fake.scripted_perception import ScriptedPerception
from domain.ports.baseline_ports import (
    BasketFace, HostCommand, MissionState, Report,
)
from domain.task import baseline_constants as bc
from domain.task.motion import AGREED_LINEAR_MPS, AGREED_ROTATION_RAD_S
from domain.values import TargetObservation


LABEL = "queen"
JAW_LINE_M = 0.36          # 실측 자리에 넣어 주는 턱 선
SERVO1_REACH_MM = 240.0

EMPTY = 0.03               # FakeArm.LOAD_EMPTY — 빈 그리퍼 실측 분포 안
HOLDING = 0.14             # FakeArm.LOAD_HOLDING


@pytest.fixture(autouse=True)
def _measured_geometry(monkeypatch):
    """턱 선·좌우 영점·팔 길이를 실측값 자리에 넣어 준다.

    `test_baseline_mission.py`와 같은 값을 쓴다 — 두 파일이 다른 기하를
    쓰면 한쪽만 통과하는 상황이 생기고, 그때 어느 쪽이 옳은지 알 수 없다.
    """
    monkeypatch.setattr(bc, "JAW_LINE_DEPTH_FORWARD_M",
                        {**bc.JAW_LINE_DEPTH_FORWARD_M, LABEL: JAW_LINE_M})
    monkeypatch.setattr(bc, "DEPTH_LATERAL_TO_JAW_CENTER_M",
                        {**bc.DEPTH_LATERAL_TO_JAW_CENTER_M, LABEL: 0.0})
    monkeypatch.setattr(bc, "SERVO1_AXIS_TO_JAW_MM", SERVO1_REACH_MM)


def _good_face():
    """INSERT 조건을 전부 만족하는 바구니 정면 관측.

    거리는 검증 창 [0.130, 0.150]의 목표값을 쓴다. 좌우는 `lateral_known=False`
    인데, 이것이 **정상**이다 — 잘 정렬돼 있을수록 방위각 창 안에 바구니
    양쪽 가장자리가 안 걸려서 구조적으로 못 잰다(오프셋 23mm 미만).
    """
    return BasketFace(
        ok=True,
        distance_m=bc.BASKET_STOP_LIDAR_M,
        yaw_error_rad=0.01,
        reason="",
        point_count=97,          # 2026-08-26 검증 지점 실측
        lateral_offset_m=0.0,
        lateral_known=False,
    )


def _host_script():
    """Host가 한 사이클에 하나씩 보내는 명령.

    소비되는 자리를 주석으로 붙여 둔다 — GRASP와 INSERT 수행은 명령을 읽지
    않으므로(팔이 도는 동안 한 사이클을 통째로 쓴다) 대본이 한 칸씩 밀리기
    쉽다. 그 밀림이 곧 실기에서 "왜 한 상태 늦게 반응하지?"로 나타난다.
    """
    return [
        HostCommand(MissionState.APPROACH, linear_x=AGREED_LINEAR_MPS),   # IDLE  -> APPROACH
        HostCommand(MissionState.GRASP, stop=True),                       # APPROACH -> (판정) -> GRASP
        # GRASP 수행 사이클 — 명령을 안 읽는다
        HostCommand(MissionState.CARRY, linear_x=AGREED_LINEAR_MPS),      # CARRY 표본 1
        HostCommand(MissionState.APPROACH_BOX, linear_x=0.06),            # CARRY 표본 2
        HostCommand(MissionState.INSERT, stop=True),                      # -> (판정) -> INSERT
        # INSERT 수행 사이클 — 명령을 안 읽는다
        HostCommand(MissionState.DONE),                                   # IDLE -> DONE
    ]


def _load_script():
    """`get_load()` 호출 순서대로의 부하 값.

    호출 자리를 세어 둔 것이라, 도메인 쪽 호출이 늘거나 줄면 이 목록이
    어긋나 테스트가 깨진다 — 그것이 의도다. 부하 판정은 빈손 11/256과 파지
    13/256 사이 **2양자**밖에 여유가 없어서, 어느 자리에서 읽는지가 값
    자체만큼 중요하다.

    ⚠️ 2026-09-01 사용자 지시로 check_grasp()가 그리퍼 부하를 더 이상 보지
    않는다(GraspInputs에서 gripper_load를 뺐다 — preconditions.check_grasp
    문서 참고) — 그래서 예전 1번 자리("GRASP 조건 판정")가 사라졌다.
    """
    return [
        HOLDING,    # 1. 닫은 직후
        HOLDING,    # 2. midpoint 유지 확인
        HOLDING,    # 3. CARRY 전환 후 (성공 판정 신호 하나)
        HOLDING,    # 4. CARRY 표본 1
        HOLDING,    # 5. CARRY 표본 2
        HOLDING,    # 6. INSERT 판정 사이클의 표본
        HOLDING,    # 7. 투하 직전
        EMPTY,      # 8. 투하 직후 — 손을 떠났다
    ]


@pytest.fixture
def run_through(make_ports, run_to_completion):
    """통주행 한 번을 돌리고 (상태이름들, 보고들, ports)를 돌려준다."""

    def _run(**overrides):
        host = FakeHostLink(script=_host_script())
        ports = make_ports(
            base=overrides.get("base") or FakeBase(),
            arm=overrides.get("arm") or FakeArm(load_ratio=_load_script()),
            perception=overrides.get("perception") or ScriptedPerception(
                script=[TargetObservation(LABEL, JAW_LINE_M + 0.02, 0.0, True)]),
            host=host,
            lidar=overrides.get("lidar") or FakeLidar(script=[_good_face()]),
            estop=threading.Event(),
        )
        states = run_to_completion(ports)
        return [s.name for s in states], host, ports

    return _run


# --------------------------------------------------------------------------


def test_한_사이클이_IDLE부터_DONE까지_돈다(run_through):
    """이 프로젝트의 성공 기준은 "전부 옮기는 것"이 아니라 "한 사이클이
    끝까지 도는 것"이다(2026-08-23 확정 스펙)."""
    names, _host, _ports = run_through()

    assert names[0] == MissionState.IDLE
    assert names[-1] == MissionState.DONE
    # 여섯 상태를 하나도 건너뛰지 않는다.
    for expected in (MissionState.APPROACH, MissionState.GRASP,
                     MissionState.CARRY, MissionState.INSERT):
        assert expected in names, f"{expected}를 안 거쳤다: {names}"


def test_상태_순서가_설계대로다(run_through):
    """중복을 접은 뒤의 순서가 팀 확정 흐름과 같아야 한다."""
    names, _host, _ports = run_through()
    squashed = [n for i, n in enumerate(names) if i == 0 or n != names[i - 1]]

    assert squashed == [
        MissionState.IDLE,
        MissionState.APPROACH,
        MissionState.GRASP,
        MissionState.CARRY,
        MissionState.INSERT,
        MissionState.IDLE,      # 투하 뒤 복귀
        MissionState.DONE,
    ], squashed


def test_보고가_Host_계약대로_나온다(run_through):
    """Host는 이 보고들로만 진행 여부를 안다 — 하나라도 빠지면 그 자리에서
    영원히 기다린다."""
    _names, host, _ports = run_through()
    kinds = host.reported_kinds

    for expected in (Report.GRASP_READY, Report.GRASP_DONE,
                     Report.INSERT_READY, Report.INSERT_DONE,
                     Report.IDLE_DONE):
        assert expected in kinds, f"{expected}가 안 나왔다: {kinds}"

    # 막힘·거부·실패가 하나도 없어야 한다 — happy path다.
    for unexpected in (Report.GRASP_BLOCKED, Report.GRASP_FAILED,
                       Report.INSERT_BLOCKED, Report.INSERT_FAILED,
                       Report.REJECTED):
        assert unexpected not in kinds, f"{unexpected}가 나왔다: {kinds}"

    # 완료 보고의 순서까지 맞아야 한다.
    order = [k for k in kinds if k in (Report.GRASP_DONE, Report.INSERT_DONE,
                                       Report.IDLE_DONE)]
    assert order == [Report.GRASP_DONE, Report.INSERT_DONE, Report.IDLE_DONE]


def test_GRASP_DONE은_CARRY_상태로_보고된다(run_through):
    """Host 쪽 계약 — "GRASP_DONE을 받으면 상태가 이미 CARRY다."

    Pi가 스스로 넘어간 유일한 자리라 Host가 CARRY 명령을 새로 보내지 않아도
    된다. 여기가 어긋나면 Host가 한 사이클 뒤처진 상태를 들고 다닌다.
    """
    _names, host, _ports = run_through()
    done = [(kind, state) for kind, state, _d, _f in host.reports
            if kind == Report.GRASP_DONE]
    assert done == [(Report.GRASP_DONE, MissionState.CARRY)], done


def test_물체를_문_채_IDLE로_접지_않는다(run_through):
    """CARRY로 접어야 라이다가 바구니를 본다 — IDLE로 접으면 그리퍼가 정면을
    79% 가린다(2026-08-26 실측)."""
    _names, _host, ports = run_through()
    stages = [stage for _profile, stage in ports.arm.floor_pose_calls]

    assert "carry" in stages, stages
    # 파지 뒤 투하 자세로 가기 전까지 idle이 끼면 안 된다.
    carry_at = stages.index("carry")
    drop_at = stages.index("drop")
    assert "idle" not in stages[carry_at:drop_at], stages


def test_투하_후에는_IDLE로_접는다(run_through):
    """투하가 끝나면 팔을 전개한 채 두지 않는다."""
    _names, _host, ports = run_through()
    stages = [stage for _profile, stage in ports.arm.floor_pose_calls]
    assert stages[-1] == "idle", stages


def test_INSERT_판정이_직전_사이클_표본을_쓴다(run_through):
    """표본 이어달리기가 끊기면 판정이 미뤄지고, 그러면 INSERT_READY 대신
    한 사이클 더 도는 흔적이 남는다.

    표본이 살아 있었다는 증거는 "INSERT_BLOCKED 없이 한 번에 READY가 났다"
    이다 — 표본이 None이면 안정성 두 항목을 못 봐서 막힌다.
    """
    _names, host, _ports = run_through()
    kinds = host.reported_kinds
    assert Report.INSERT_BLOCKED not in kinds
    assert kinds.index(Report.INSERT_READY) < kinds.index(Report.INSERT_DONE)


def test_주행_명령이_합의_속도로_바퀴까지_간다(run_through):
    """Host가 보낸 속도가 클램프를 지나 실제로 베이스에 도달하는가."""
    _names, _host, ports = run_through()
    assert ports.base.velocity_calls, "한 번도 안 움직였다"
    for linear_x, linear_y, angular_z in ports.base.velocity_calls:
        assert abs(linear_x) <= AGREED_LINEAR_MPS + 1e-9
        assert abs(linear_y) <= AGREED_LINEAR_MPS + 1e-9
        assert abs(angular_z) <= AGREED_ROTATION_RAD_S + 1e-9
    # 파지 전진은 관측에서 나온 값이라 상한 안이어야 한다.
    assert ports.base.creep_forward_calls
    for creep in ports.base.creep_forward_calls:
        assert 0.0 < creep <= bc.GRASP_CREEP_FORWARD_MM / 1000.0 + 1e-9


def test_파지_성공은_부하_하나만_있어도_된다(run_through):
    """부하와 뎁스(confirm_grasp) 중 하나만 있어도 성공이다(OR, 사용자 지시
    2026-09-01 — AND 였던 이전 계약은 CARRY 자세에서 confirm_grasp() 가
    뎁스 오탐("그대로 있다")을 낸 실기 사고로 바뀌었다).
    """
    _names, host, ports = run_through()
    assert ports.perception.confirm_grasp_calls >= 1
    assert Report.GRASP_DONE in host.reported_kinds

    # 뎁스만 뒤집어도(오탐 흉내) 부하가 정상이면 통주행은 여전히 성공이다.
    _names, host, _ports = run_through(
        perception=ScriptedPerception(
            script=[TargetObservation(LABEL, JAW_LINE_M + 0.02, 0.0, True)],
            grasp_confirmed=False))
    assert Report.GRASP_DONE in host.reported_kinds
    assert Report.GRASP_FAILED not in host.reported_kinds


def test_부하도_뎁스도_없으면_실패다(run_through):
    """둘 다 실패를 가리킬 때만 진짜 실패다 — 유일하게 남은 실패 경로."""
    _names, host, _ports = run_through(
        arm=FakeArm(load_ratio=EMPTY),
        perception=ScriptedPerception(
            script=[TargetObservation(LABEL, JAW_LINE_M + 0.02, 0.0, True)],
            grasp_confirmed=False))
    assert Report.GRASP_DONE not in host.reported_kinds
    failures = [detail for kind, _s, detail, _f in host.reports
                if kind == Report.GRASP_FAILED]
    assert failures, host.reported_kinds


def test_파지_실패_보고에_시도_횟수가_실린다(run_through):
    """Host가 재시도 여부를 정하려면 몇 번째인지 알아야 한다(2026-08-28)."""
    _names, host, _ports = run_through(
        arm=FakeArm(load_ratio=EMPTY),
        perception=ScriptedPerception(
            script=[TargetObservation(LABEL, JAW_LINE_M + 0.02, 0.0, True)],
            grasp_confirmed=False))
    failures = [detail for kind, _s, detail, _f in host.reports
                if kind == Report.GRASP_FAILED]
    assert failures, host.reported_kinds
    assert "1번째 시도 실패" in failures[0], failures[0]


def test_라이다가_바구니를_못_보면_INSERT로_안_넘어간다(run_through):
    """"모르면 실패"가 이 포트의 계약이다 — 관측이 없는데 팔을 펴면 안 된다."""
    blind = BasketFace(ok=False, distance_m=float("inf"),
                       yaw_error_rad=float("inf"), reason="정면 미검출")
    _names, host, _ports = run_through(lidar=FakeLidar(script=[blind]))

    kinds = host.reported_kinds
    assert Report.INSERT_BLOCKED in kinds
    assert Report.INSERT_READY not in kinds
    assert Report.INSERT_DONE not in kinds
