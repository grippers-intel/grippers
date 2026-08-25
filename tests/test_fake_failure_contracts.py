"""Fake 어댑터의 실패 표현이 포트 계약과 일치하는지 고정한다.

**Fake와 real이 같은 상황을 다르게 표현하면 "CI 테스트가 실기 동작을 보장한다"는
Fake 어댑터의 존재 이유가 무너진다.** 이 프로젝트에서 이미 두 번 났던 사고다:

- `ScriptedInterpreter.parse()` 는 `ValueError`, `Ros2CommandInterpreter.parse()` 는
  `None` (PR #9 리뷰 B항)
- `FakeArm.get_load()` 는 0~1 정규화, `Ros2ArmDriver.get_load()` 는 서보 원시값 (PR #136)

둘 다 CI는 초록불인데 실기에서만 깨지는 종류다 — 도메인 테스트는 Fake의 표현만 보기
때문이다. 아래 표가 포트별 실패값의 단일 기준이고, 계약이 다시 갈라지면 여기서 잡힌다.

real 쪽은 `rclpy` 가 있어야 import 되므로 여기서 함께 호출해 비교할 수 없다 —
real 어댑터가 이 표와 같은 값을 돌려주는지는 PR #137 의
`tests/test_real_adapter_timeouts.py` 가 AST 정적 검사로 본다."""

import math

import pytest

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.scripted_interpreter import ScriptedInterpreter
from domain.adapters.fake.scripted_perception import ScriptedPerception
from domain.ports.arm_driver import ArmDriver
from domain.ports.base_driver import BaseDriver
from domain.ports.command_interpreter import CommandInterpreter
from domain.ports.perception import Perception
from domain.values import BoxObservation, Destination, Detection, ObjectClass, Point3, Pose2D

_TARGET = Pose2D(x=0.2, y=0.0, theta=0.0)
_POINT = Point3(x=0.2, y=0.0, z=0.0)
_DETECTION = Detection(
    track_id=1,
    cls=ObjectClass.CHESS_PIECE,
    pose_m=Point3(x=0.2, y=0.0, z=0.0),
    dims_m=Point3(x=0.0245, y=0.0245, z=0.04),
    yaw_rad=0.0,
    confidence=0.9,
)
_BOX = BoxObservation(
    dest=Destination.LEFT,
    pose_m=Pose2D(x=0.5, y=0.0, theta=0.0),
    opening_mm=400.0,
    long_axis_rad=0.0,
)

# (포트, 메서드, 실패를 주입한 Fake 호출, 기대 실패값, 포트 docstring에 있어야 할 문구)
#
# docstring 문구가 None인 항목은 **PR #137 이 실패 계약 docstring을 추가하는 자리**다.
# 이 PR은 main에서 분기했으므로 아직 그 문장이 없다 — 아래
# test_undocumented_contracts_are_only_the_ones_pr137_adds 가 그 목록을 고정한다.
FAILURE_CONTRACTS = [
    (BaseDriver, "drive_to", lambda: FakeBase(arrive=False).drive_to(_TARGET), False, None),
    (
        BaseDriver,
        "approach",
        lambda: FakeBase(arrive=False).approach(_DETECTION),
        False,
        "`False`",
    ),
    (
        BaseDriver,
        "align_to_box",
        lambda: FakeBase(align_ok=False).align_to_box(_BOX),
        math.inf,
        "ALIGN_FAILED_YAW_ERROR_RAD",
    ),
    (
        ArmDriver,
        "move_to_cartesian",
        lambda: FakeArm(move_ok=False).move_to_cartesian(_POINT),
        False,
        None,
    ),
    (
        ArmDriver,
        "move_to_floor_pose",
        lambda: FakeArm(move_ok=False).move_to_floor_pose("cube", "safe"),
        False,
        "`False`",
    ),
    (ArmDriver, "get_load", lambda: FakeArm(load_ratio=0.0).get_load(), 0.0, None),
    (ArmDriver, "reorient", lambda: FakeArm(reorient_ok=False).reorient(0.0), False, None),
    (ArmDriver, "fold_to_cradle", lambda: FakeArm(fold_ok=False).fold_to_cradle(), False, None),
    (
        Perception,
        "scan_floor",
        lambda: ScriptedPerception(found=False).scan_floor(),
        [],
        "빈 리스트",
    ),
    (
        Perception,
        "find_box",
        lambda: ScriptedPerception(box_found=False).find_box(Destination.LEFT),
        None,
        "`None`",
    ),
    (
        Perception,
        "measure_opening",
        lambda: ScriptedPerception(opening_mm=None).measure_opening(_BOX),
        None,
        "`None`",
    ),
    (
        # 이 항목만 반환값 전체가 아니라 안전 판정 필드를 본다 — 거리는 시나리오마다
        # 다르지만 "모르면 멈춘다"는 contact_risk 하나로 표현된다.
        Perception,
        "monitor_clearance",
        lambda: ScriptedPerception(contact_risk=True).monitor_clearance().contact_risk,
        True,
        "contact_risk=True",
    ),
    (
        Perception,
        "remember_target",
        # 기준 관측을 못 잡으면 False — 그 뒤 confirm_grasp 는 비교 기준이
        # 없으므로 판정을 포기한다. 테스트 더블은 happy path 가 기본이라
        # 실패 경로를 real 어댑터 계약으로만 표현한다.
        lambda: ScriptedPerception(target_remembered=False).remember_target("rook"),
        False,
        "`False`",
    ),
    (
        Perception,
        "confirm_grasp",
        lambda: ScriptedPerception(grasp_confirmed=False).confirm_grasp(),
        False,
        "`False`",
    ),
    (
        CommandInterpreter,
        "parse",
        lambda: ScriptedInterpreter().parse("등록되지 않은 문형"),
        None,
        "`None`",
    ),
]

# PR #137 이 실패 계약 docstring을 추가하는 메서드들. 그 PR이 머지되면 위 표의
# 마지막 칸을 채우고 이 집합을 비운다.
DOCSTRING_PENDING_IN_PR_137 = {
    "BaseDriver.drive_to",
    "ArmDriver.move_to_cartesian",
    "ArmDriver.get_load",
    "ArmDriver.reorient",
    "ArmDriver.fold_to_cradle",
}


def _row_id(row):
    port, method = row[0], row[1]
    return f"{port.__name__}.{method}"


@pytest.mark.parametrize("row", FAILURE_CONTRACTS, ids=_row_id)
def test_fake_returns_the_contracted_failure_value(row):
    """실패를 주입한 Fake가 포트 계약과 **정확히 같은 값**을 돌려준다."""
    _port, _method, call, expected, _marker = row
    actual = call()

    assert actual == expected, f"{_row_id(row)}: 계약은 {expected!r}인데 Fake는 {actual!r}"
    # bool/float/None은 == 로 서로 통과하는 조합이 있다(False == 0.0, 0.0 == False).
    # 계약이 갈라지는 건 대개 '표현'이 다른 경우라 타입까지 본다.
    assert type(actual) is type(
        expected
    ), f"{_row_id(row)}: 값은 같지만 타입이 다르다 — {type(actual)} vs {type(expected)}"


@pytest.mark.parametrize("row", FAILURE_CONTRACTS, ids=_row_id)
def test_port_docstring_states_the_failure_value(row):
    """계약이 코드 어디에도 적혀 있지 않으면 같은 사고가 반복된다 — 포트
    docstring이 실패값을 실제로 말하는지 본다."""
    port, method, _call, _expected, marker = row
    if marker is None:
        assert _row_id(row) in DOCSTRING_PENDING_IN_PR_137
        return
    doc = getattr(port, method).__doc__ or ""
    assert marker in doc, f"{_row_id(row)}: 포트 docstring에 실패값({marker})이 없다"


def test_undocumented_contracts_are_only_the_ones_pr137_adds():
    """docstring이 비어 있는 항목이 조용히 늘어나지 않게 한다."""
    pending = {_row_id(row) for row in FAILURE_CONTRACTS if row[4] is None}
    assert pending == DOCSTRING_PENDING_IN_PR_137


def test_every_port_method_with_a_failure_value_is_covered():
    """실패값이 있는 포트 메서드가 표에서 빠지면 검사에 구멍이 난다.

    실패를 값으로 표현할 수 없는 메서드(반환 타입이 None이거나 실패 개념이 없는 것)는
    제외 목록에 명시한다 — 목록에 없는 메서드가 새로 생기면 여기서 걸린다."""
    no_failure_value = {
        "BaseDriver.stop",  # E-STOP 경로 — 반환값 없음, 로그만
        "ArmDriver.set_gripper",  # 반환값 없음 — 뒤이은 get_load()가 실패를 드러냄
        "ArmDriver.hold_position",  # E-STOP 경로 — 반환값 없음
        "CommandInterpreter.confirm_phrase",  # 실패해도 빈 문자열, 미션은 계속
    }
    covered = {_row_id(row) for row in FAILURE_CONTRACTS}
    declared = {
        f"{port.__name__}.{name}"
        for port in (BaseDriver, ArmDriver, Perception, CommandInterpreter)
        for name in port.__abstractmethods__
    }

    assert declared == covered | no_failure_value


# ── 새로 열린 실패 경로 ───────────────────────────────────────────────────
#
# 아래 두 개는 PR #138 시점에 **의도적으로 xfail(strict)** 이었다. Fake가 계약대로
# 실패를 표현하게 되면서 결함이 드러났지만, 그 실패를 흡수하는 쪽은
# domain/task/states.py 라 그 PR의 범위 밖이었다. 계약을 테스트로 먼저 적어 두고
# strict xfail이 xpass로 뒤집히면 "이제 마커를 떼라"고 알려 주는 방식이었고,
# 실제로 그렇게 됐다 — 흡수 코드가 들어왔으므로 마커를 떼고 정상 테스트로 둔다.


def _detection(track_id=1):
    from domain.values import Detection, ObjectClass

    return Detection(
        track_id=track_id,
        cls=ObjectClass.GABE,
        pose_m=Point3(x=0.2, y=0.0, z=0.0),
        dims_m=Point3(x=0.05, y=0.05, z=0.05),
        yaw_rad=0.0,
        confidence=0.9,
    )


def test_unparsed_command_keeps_the_mission_in_idle(make_ports):
    """해석하지 못한 명령으로는 미션이 시작되지 않아야 한다 — IDLE 유지.

    Fake가 ValueError를 던지던 동안에는 이 결함이 보이지 않았다. 예외가 IDLE에서
    즉시 터져 나가 SCAN까지 갈 일이 없었기 때문이다. 계약대로 None을 돌려주자
    비로소 real과 같은 경로를 밟게 됐고, 그 경로가 깨져 있다는 게 드러났다 —
    `MissionContext(spec=None)` 이 SCAN까지 흘러가 SELECT에서 AttributeError로
    FSM 스레드가 죽었다."""
    from domain.task.mission_task import MissionTask

    gen = MissionTask(make_ports()).run("알 수 없는 명령")

    assert [next(gen).name for _ in range(5)] == ["IDLE"] * 5


def test_align_failure_holds_the_target(make_ports, run_to_completion):
    """정렬에 실패하면 상자에 넣을 수 없다 — 대상을 보류 등록하고 SCAN으로 복귀한다.

    수정 전에는 `TransportState` 가 `align_to_box()` 반환값을 아예 읽지 않아
    무한대 오차를 돌려줘도 그대로 INSERT까지 진행했다 (hld.md §6.4 #10)."""
    ports = make_ports(
        base=FakeBase(align_ok=False),
        perception=ScriptedPerception(detections=[_detection(track_id=1)]),
    )

    states = run_to_completion(ports)

    assert "INSERT" not in [s.name for s in states]
    assert states[-1].ctx.held_ids == {1}


def test_measure_opening_failure_rejects_before_insert(make_ports, run_to_completion):
    """입구 폭 실측 실패(None)는 투입을 시도하지 않고 REJECT로 전이한다."""
    perception = ScriptedPerception(
        opening_mm=None,
        detections=[_detection(track_id=1)],
    )

    states = run_to_completion(make_ports(perception=perception))
    names = [state.name for state in states]

    assert "REJECT" in names
    assert "INSERT" not in names


def test_measure_opening_failure_does_not_corrupt_box_observation():
    """정밀 실측 실패(None)가 BoxObservation의 float 계약까지 오염시키지 않는다."""
    perception = ScriptedPerception(opening_mm=None)

    box = perception.find_box(Destination.LEFT)

    assert box is not None
    assert isinstance(box.opening_mm, float)
    assert box.opening_mm == 400.0
    assert perception.measure_opening(box) is None
