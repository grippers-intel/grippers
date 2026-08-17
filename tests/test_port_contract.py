"""포트 시그니처 freeze 계약 테스트 (#97).

이 파일을 고치는 diff 가 곧 freeze 를 깨는 diff 다.
변경하려면 `docs/design/port_freeze.md` §6 절차 — 문서 이력 추가 → 3인 합의.

하드웨어·ROS2 없이 돌고, 어댑터 구현에도 의존하지 않는다.
검사 대상은 **이름 · 인자 개수 · 인자 이름과 순서** (Tier-1) 뿐이고,
임계값·동작 의미(Tier-2)는 검사하지 않는다.
"""

import dataclasses
import inspect

import pytest

from domain.ports.arm_driver import ArmDriver
from domain.ports.base_driver import BaseDriver
from domain.ports.command_interpreter import CommandInterpreter
from domain.ports.perception import Perception
from domain.values import (
    BoxColor,
    MissionMode,
    MissionSpec,
    ObjectClass,
    Point3,
    Pose2D,
)

FROZEN_SIGNATURES = {
    BaseDriver: {
        "drive_to": ["self", "target"],
        "align_to_box": ["self", "box"],
        "stop": ["self"],
    },
    ArmDriver: {
        "move_to_cartesian": ["self", "xyz_m", "down"],
        "set_gripper": ["self", "width_mm"],
        "get_load": ["self"],
        "reorient": ["self", "phi_rad"],
        "fold_to_cradle": ["self"],
        "hold_position": ["self"],
    },
    Perception: {
        "scan_floor": ["self"],
        "find_box": ["self", "color"],
        "measure_opening": ["self", "box"],
        "monitor_clearance": ["self"],
    },
    CommandInterpreter: {
        "parse": ["self", "text"],
        "confirm_phrase": ["self", "spec"],
    },
}

_CASES = [
    (port, name, params)
    for port, methods in FROZEN_SIGNATURES.items()
    for name, params in methods.items()
]
_IDS = [f"{port.__name__}.{name}" for port, name, _ in _CASES]


@pytest.mark.parametrize(("port", "name", "params"), _CASES, ids=_IDS)
def test_frozen_method_signature(port, name, params):
    method = getattr(port, name, None)
    assert method is not None, f"{port.__name__}.{name} 이 사라졌습니다 (Tier-1 freeze)"
    assert list(inspect.signature(method).parameters) == params


@pytest.mark.parametrize("port", list(FROZEN_SIGNATURES), ids=lambda p: p.__name__)
def test_no_undeclared_public_method(port):
    """freeze 표에 없는 공개 메서드를 몰래 늘리지 않는다."""
    public = {n for n, _ in inspect.getmembers(port, inspect.isfunction) if not n.startswith("_")}
    assert public == set(FROZEN_SIGNATURES[port])


@pytest.mark.parametrize("port", list(FROZEN_SIGNATURES), ids=lambda p: p.__name__)
def test_every_method_is_abstract(port):
    """포트는 전부 추상이다 — 어댑터가 빠뜨리면 인스턴스 생성 시점에 터져야 한다."""
    assert port.__abstractmethods__ == frozenset(FROZEN_SIGNATURES[port])


def test_domain_layer_does_not_import_ros2():
    """포트·값 객체는 ROS2를 모른다 — 계층 경계의 유일한 자동 검증."""
    import domain.values
    from domain.ports import arm_driver, base_driver, command_interpreter, perception

    for module in (domain.values, base_driver, arm_driver, perception, command_interpreter):
        imported = {n for n in vars(module) if n.startswith(("rclpy", "geometry_msgs"))}
        assert not imported, f"{module.__name__} 이 ROS2 심볼을 들고 있습니다: {imported}"


def test_unit_suffix_on_every_numeric_field():
    """단위 규약 — 숫자 필드는 이름만 보고 단위를 알 수 있어야 한다."""
    assert list(vars(Pose2D("x", "y", "t")).keys()) == ["x_m", "y_m", "theta_rad"]
    assert list(vars(Point3("x", "y", "z")).keys()) == ["x_m", "y_m", "z_m"]


def test_enum_values_are_frozen():
    assert {c.value for c in ObjectClass} == {"toy", "chess"}
    assert {c.value for c in BoxColor} == {"black", "red", "blue", "green"}
    assert {m.value for m in MissionMode} == {"tidy", "fetch"}


def test_mission_context_is_immutable():
    """complete/hold/retry 는 새 인스턴스를 반환한다 — 재시도 추적이 가능한 이유."""
    from domain.values import MissionContext

    spec = MissionSpec(mode=MissionMode.TIDY, placement_rule={}, raw_text="")
    ctx = MissionContext(spec=spec)

    assert ctx.complete(1).done_ids == frozenset({1})
    assert ctx.hold(2).held_ids == frozenset({2})
    assert ctx.retry().grasp_attempts == 1
    assert ctx.done_ids == frozenset() and ctx.held_ids == frozenset()
    assert ctx.grasp_attempts == 0

    with pytest.raises(dataclasses.FrozenInstanceError):
        ctx.grasp_attempts = 5
