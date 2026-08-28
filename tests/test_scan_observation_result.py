"""이슈 #194 — `scan_floor()` 의 '정상 빈 장면' 과 '관측 실패' 를 가르는 계약.

예전 계약은 둘 다 `[]` 였다. 그래서 perception 이 통째로 죽어도 `SCAN` 이
"남은 대상 없음" 으로 읽고 `DONE` 으로 갔다 — **센서 장애가 정상 완료로
기록됐다.** 이 파일이 그 회귀를 막는다.

전부 Fake 어댑터로 FSM 을 실제로 구동해서 **상태 전이**를 본다. real 어댑터는
`rclpy` 없이 import 할 수 없어 마지막 두 개만 AST 정적 검사다 —
`tests/test_real_adapter_timeouts.py` 와 같은 이유다."""

import ast
import pathlib
import threading
from dataclasses import FrozenInstanceError

import pytest

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.scripted_perception import ScriptedPerception
from domain.task.states import OPEN_MM, GraspState, PerceptionFailedState
from domain.values import (
    Detection,
    MissionContext,
    MissionMode,
    MissionSpec,
    ObjectClass,
    Point3,
    ScanResult,
    ScanStatus,
)

REAL_PERCEPTION = (
    pathlib.Path(__file__).resolve().parent.parent
    / "domain"
    / "adapters"
    / "real"
    / "ros2_perception.py"
)

# 서비스 부재와 타임아웃은 도메인 결과가 같다(둘 다 UNAVAILABLE). 그래도 주입은
# 따로 유지한다 — 실기에서 원인을 가르는 건 이 문자열이고, 한 갈래로 합치면
# "무엇이 끊겼는지" 를 시험이 더 이상 요구하지 않게 된다.
NO_SERVICE = "scan_floor 서비스 없음 (테스트 주입)"
TIMEOUT = "scan_floor 응답 없음 — 3.0s 초과 (테스트 주입)"
NO_FRAME = "카메라 프레임 없음 (테스트 주입)"


def _detection(track_id=1):
    return Detection(
        track_id=track_id,
        cls=ObjectClass.GABE,
        pose_m=Point3(x=0.2, y=0.0, z=0.0),
        dims_m=Point3(x=0.05, y=0.05, z=0.05),
        yaw_rad=0.0,
        confidence=0.9,
    )


def _ctx():
    return MissionContext(
        spec=MissionSpec(
            mode=MissionMode.TIDY,
            target_cls=None,
            placement_rule={ObjectClass.GABE: "RED"},
            raw_text="",
        )
    )


# ── 값 객체 계약 ────────────────────────────────────────────────────────


def test_observed_and_unavailable_are_different_types_of_answer():
    """빈 장면과 실패가 **필드가 아니라 상태로** 갈린다."""
    empty = ScanResult.observed([])
    failed = ScanResult.unavailable(NO_SERVICE)

    assert empty.status is ScanStatus.OBSERVED
    assert failed.status is ScanStatus.UNAVAILABLE
    assert empty != failed


def test_boolean_coercion_raises_instead_of_silently_passing():
    """`if not result` 를 **막지 않고 터뜨린다.**

    `__bool__` 을 정의하지 않으면 파이썬 기본 동작상 둘 다 항상 참이 되어
    `if not result` 가 조용히 거짓 분기로 흐른다 — 차단이 아니라 은폐다.
    실패를 빈 장면처럼 다루던 #194 가 정확히 그 모양이었으므로, 같은 실수를
    다시 하면 그 줄에서 즉시 죽어야 한다."""
    with pytest.raises(TypeError):
        bool(ScanResult.observed([]))
    with pytest.raises(TypeError):
        bool(ScanResult.unavailable("timeout"))


def test_boolean_coercion_raises_in_an_if_statement():
    """`bool()` 직접 호출뿐 아니라 `if` 문맥에서도 터진다."""
    result = ScanResult.observed([])
    with pytest.raises(TypeError):
        if not result:  # noqa: SIM103 — 이 오용 자체가 시험 대상이다
            pass


def test_detections_are_an_immutable_snapshot():
    """`frozen=True` 는 필드 재대입만 막는다 — 내용까지 지키려면 tuple 이어야 한다."""
    source = [_detection(1)]
    result = ScanResult.observed(source)

    assert isinstance(result.detections, tuple)
    source.append(_detection(2))
    assert len(result.detections) == 1, "생성 후 원본 list 변경이 새어 들어왔다"

    with pytest.raises(FrozenInstanceError):
        result.detections = ()


def test_unavailable_requires_a_reason():
    """원인 없는 실패는 실기에서 추적이 불가능하다."""
    with pytest.raises(ValueError):
        ScanResult.unavailable("")


def test_result_rejects_inconsistent_combinations():
    with pytest.raises(ValueError):
        ScanResult(status=ScanStatus.UNAVAILABLE, detections=(_detection(),))
    with pytest.raises(ValueError):
        ScanResult(status=ScanStatus.OBSERVED, reason="있으면 안 된다")


# ── Fake 어댑터 주입 ────────────────────────────────────────────────────


def test_fake_injects_a_normal_empty_scene():
    """`found=False` 는 실패가 아니라 **관측했고 0개** 다."""
    result = ScriptedPerception(found=False).scan_floor()

    assert result.status is ScanStatus.OBSERVED
    assert result.detections == ()


def test_fake_injects_an_observation_failure():
    result = ScriptedPerception(scan_unavailable=NO_SERVICE).scan_floor()

    assert result.status is ScanStatus.UNAVAILABLE
    assert result.reason == NO_SERVICE


def test_fake_failure_wins_over_the_script():
    """실패를 주입해 두고 스크립트가 검출을 돌려주면 시험이 모호해진다."""
    perception = ScriptedPerception(script=[[_detection()]], scan_unavailable=TIMEOUT)

    assert perception.scan_floor().status is ScanStatus.UNAVAILABLE


def test_fake_normal_script_still_advances_per_call():
    """실패 주입이 없을 때 사이클별 스크립트 동작이 그대로다."""
    perception = ScriptedPerception(script=[[_detection(1)], [], [_detection(3)]])

    ids = []
    for _ in range(4):
        ids.append(tuple(d.track_id for d in perception.scan_floor().detections))

    assert ids == [(1,), (), (3,), (3,)]


# ── FSM 전이 ────────────────────────────────────────────────────────────


def test_detections_present_goes_to_select(make_ports, run_to_completion):
    """정상 검출 경로가 그대로다 — SELECT 를 거쳐 DONE 으로 끝난다."""
    ports = make_ports(perception=ScriptedPerception(script=[[_detection()], []]))
    names = [s.name for s in run_to_completion(ports)]

    assert "SELECT" in names
    assert names[-1] == "DONE"
    assert "PERCEPTION_FAILED" not in names


def test_normal_empty_scene_still_rescans_then_done(make_ports, run_to_completion):
    """정상 빈 장면만 기존 재스캔 정책의 대상이다 (MAX_RESCAN 후 DONE)."""
    ports = make_ports(perception=ScriptedPerception(found=False))
    names = [s.name for s in run_to_completion(ports)]

    assert names.count("SCAN") == 4  # 최초 1 + MAX_RESCAN 3
    assert names[-1] == "DONE"
    assert "PERCEPTION_FAILED" not in names


@pytest.mark.parametrize("reason", [NO_SERVICE, TIMEOUT, NO_FRAME], ids=lambda r: r[:20])
def test_observation_failure_never_reaches_done(make_ports, run_to_completion, reason):
    """서비스 없음 · 타임아웃 · 프레임 없음 어느 것도 DONE 으로 가지 않는다."""
    ports = make_ports(perception=ScriptedPerception(scan_unavailable=reason))
    names = [s.name for s in run_to_completion(ports)]

    assert names[-1] == "PERCEPTION_FAILED"
    assert "DONE" not in names


def test_observation_failure_is_not_estop(make_ports, run_to_completion):
    """관측 실패는 물리 E-STOP 이 아니다 — 사람이 누른 인터럽트와 구분한다."""
    ports = make_ports(perception=ScriptedPerception(scan_unavailable=NO_SERVICE))
    names = [s.name for s in run_to_completion(ports)]

    assert "ESTOP" not in names
    assert names[-1] == "PERCEPTION_FAILED"


def test_observation_failure_does_not_retry_forever(make_ports, run_to_completion):
    """재스캔으로 풀리지 않는 실패다 — 한 번 보고 바로 끝난다."""
    ports = make_ports(perception=ScriptedPerception(scan_unavailable=NO_SERVICE))
    names = [s.name for s in run_to_completion(ports)]

    assert names.count("SCAN") == 1


def test_failure_state_is_terminal_and_stops_the_base_without_latching_the_arm(make_ports):
    """terminal 이고 베이스는 세우되, **팔은 래치하지 않는다** — E-STOP 과의 차이다.

    `FakeBase`/`FakeArm` 은 호출 기록을 남기지 않으므로 얇은 스파이로 감싼다.
    Fake 에 기록 필드를 새로 넣는 것은 이 이슈의 범위를 넘는다."""

    class _SpyBase(FakeBase):
        def __init__(self):
            super().__init__()
            self.stop_calls = 0

        def stop(self):
            self.stop_calls += 1
            return super().stop()

    class _SpyArm(FakeArm):
        def __init__(self):
            super().__init__()
            self.hold_calls = 0

        def hold_position(self):
            self.hold_calls += 1
            return super().hold_position()

    base, arm = _SpyBase(), _SpyArm()
    ports = make_ports(base=base, arm=arm)

    assert PerceptionFailedState(_ctx(), NO_SERVICE).execute(ports) is None
    assert base.stop_calls == 1
    assert arm.hold_calls == 0


def test_failure_reason_survives_into_the_terminal_state(make_ports, run_to_completion):
    """상태 이름만으로는 무엇이 끊겼는지 알 수 없다 — 원인이 상태에 실려야 한다."""
    ports = make_ports(perception=ScriptedPerception(scan_unavailable=TIMEOUT))
    last = run_to_completion(ports)[-1]

    assert last.name == "PERCEPTION_FAILED"
    assert last.reason == TIMEOUT


def test_failure_state_name_is_distinct_from_done_and_estop():
    assert PerceptionFailedState.name not in ("DONE", "ESTOP")


# ── 파지 재시도 경로 ────────────────────────────────────────────────────


def _grasp_ports(make_ports, perception):
    """파지가 실패해 `_retry_after_release()` 로 들어가는 상황을 만든다."""
    return make_ports(arm=FakeArm(load_ratio=0.0), perception=perception)


def test_retry_after_release_uses_the_refreshed_pose(make_ports):
    """정상 관측이면 같은 track_id 의 갱신된 detection 으로 재시도한다."""
    moved = Detection(
        track_id=1,
        cls=ObjectClass.GABE,
        pose_m=Point3(x=0.42, y=0.0, z=0.0),
        dims_m=Point3(x=0.05, y=0.05, z=0.05),
        yaw_rad=0.0,
        confidence=0.9,
    )
    ports = _grasp_ports(make_ports, ScriptedPerception(script=[[moved]]))

    nxt = GraspState(_ctx(), _detection(1)).execute(ports)

    assert nxt.name == "GRASP"
    assert nxt.target.pose_m.x == pytest.approx(0.42)


def test_retry_after_release_keeps_old_pose_when_track_id_is_gone(make_ports):
    """관측은 됐는데 그 물체가 없으면 기존 정책 그대로 — 이전 pose 로 한 번 더."""
    ports = _grasp_ports(make_ports, ScriptedPerception(script=[[_detection(99)]]))

    nxt = GraspState(_ctx(), _detection(1)).execute(ports)

    assert nxt.name == "GRASP"
    assert nxt.target.track_id == 1


def test_retry_after_release_never_reuses_stale_pose_on_failure(make_ports):
    """🔴 관측 실패 상태에서 옛 pose 로 팔을 내리면 **보이지도 않는 자리**를 집는다."""
    ports = _grasp_ports(make_ports, ScriptedPerception(scan_unavailable=NO_SERVICE))

    nxt = GraspState(_ctx(), _detection(1)).execute(ports)

    assert nxt.name == "PERCEPTION_FAILED"
    assert nxt.reason == NO_SERVICE


def test_estop_still_preempts_everything(make_ports, run_to_completion):
    """기존 E-STOP 경로가 그대로다 — 관측 실패보다 우선한다."""
    estop = threading.Event()
    estop.set()
    ports = make_ports(perception=ScriptedPerception(scan_unavailable=NO_SERVICE), estop=estop)

    assert [s.name for s in run_to_completion(ports)] == ["ESTOP"]


# ── real 어댑터 (AST 정적 검사 — rclpy 없이 import 불가) ────────────────


def _scan_floor_ast():
    tree = ast.parse(REAL_PERCEPTION.read_text(encoding="utf-8"), filename=str(REAL_PERCEPTION))
    return next(
        n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "scan_floor"
    )


def test_real_adapter_never_returns_a_bare_list_on_failure():
    """실패를 `[]` 로 삼키던 것이 #194 의 직접 원인이다."""
    for node in ast.walk(_scan_floor_ast()):
        if isinstance(node, ast.Return) and isinstance(node.value, ast.List | ast.ListComp):
            pytest.fail("real scan_floor 가 리스트를 그대로 반환한다 — ScanResult 계약 위반")


def test_real_adapter_maps_none_response_to_unavailable():
    """`call_service` 가 None 이면 `ScanResult.unavailable(...)` 로 간다."""
    fn = _scan_floor_ast()
    unavailable_calls = [
        n
        for n in ast.walk(fn)
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute)
        and n.func.attr == "unavailable"
    ]

    assert unavailable_calls, "실패 경로에 ScanResult.unavailable(...) 이 없다"
    assert unavailable_calls[0].args, "unavailable 에 원인이 실리지 않았다"


def test_retry_path_opens_the_gripper_before_it_can_fail(make_ports):
    """`PerceptionFailedState` 가 팔을 래치하지 않는 근거 — 진입 시 손이 비어 있다.

    `_retry_after_release()` 는 첫 줄에서 그리퍼를 연다. 관측 실패로 종료해도
    떨어뜨릴 물체가 없다는 전제가 여기서 성립한다. 순서가 뒤집히면 그 전제가
    깨지므로 호출 순서를 고정한다."""

    class _OrderedArm(FakeArm):
        def __init__(self):
            super().__init__(load_ratio=0.0)
            self.calls = []

        def set_gripper(self, width_mm):
            self.calls.append(("set_gripper", width_mm))
            return super().set_gripper(width_mm)

    class _OrderedPerception(ScriptedPerception):
        def __init__(self, arm):
            super().__init__(scan_unavailable=NO_SERVICE)
            self._arm = arm

        def scan_floor(self):
            self._arm.calls.append(("scan_floor", None))
            return super().scan_floor()

    arm = _OrderedArm()
    ports = make_ports(arm=arm, perception=_OrderedPerception(arm))

    nxt = GraspState(_ctx(), _detection(1)).execute(ports)

    assert nxt.name == "PERCEPTION_FAILED"
    opened = [i for i, (n, w) in enumerate(arm.calls) if n == "set_gripper" and w == OPEN_MM]
    scanned = [i for i, (n, _) in enumerate(arm.calls) if n == "scan_floor"]
    assert opened and scanned, f"호출 순서를 못 잡았다: {arm.calls}"
    assert opened[-1] < scanned[-1], "관측 전에 그리퍼가 열려 있어야 한다"
