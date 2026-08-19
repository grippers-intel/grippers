"""arm_driver_node의 실물 하드웨어 실패 계약 정적 검사 (#156).

arm_driver_node는 rclpy·grippers_interfaces와 실제 SO-ARM101 의존성이 있어
일반 개발 머신에서 직접 import하기 어렵다. 따라서 소스를 AST로 읽어 기동 및
모션 경계의 핵심 안전 계약이 사라지지 않는지 검사한다.

실제 USB 단선·torque OFF 동작은 Pi + SO-ARM101 실기 검증 대상이다.
"""

import ast
import pathlib

ARM_NODE = (
    pathlib.Path(__file__).resolve().parent.parent
    / "ros2_ws"
    / "src"
    / "grippers_arm"
    / "grippers_arm"
    / "arm_driver_node.py"
)


def _parse():
    return ast.parse(
        ARM_NODE.read_text(encoding="utf-8"),
        filename=str(ARM_NODE),
    )


def _function(name):
    return next(
        node
        for node in ast.walk(_parse())
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name
    )


def _calls(node):
    return [n for n in ast.walk(node) if isinstance(n, ast.Call)]


def _called_name(call):
    func = call.func
    if isinstance(func, ast.Attribute):
        return func.attr
    if isinstance(func, ast.Name):
        return func.id
    return None


def test_torque_auto_enable_is_opt_in():
    init = _function("__init__")

    declarations = [
        call
        for call in _calls(init)
        if _called_name(call) == "declare_parameter"
        and call.args
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == "enable_torque_on_start"
    ]

    assert len(declarations) == 1
    call = declarations[0]
    assert len(call.args) >= 2
    assert isinstance(call.args[1], ast.Constant)
    assert call.args[1].value is False


def test_startup_checks_serial_connection_and_torque():
    init = _function("__init__")
    names = [_called_name(call) for call in _calls(init)]

    assert "is_connected" in names
    assert "_check_startup_torque" in names


def test_startup_torque_checks_each_servo():
    fn = _function("_check_startup_torque")
    names = [_called_name(call) for call in _calls(fn)]

    assert "get_torque" in names
    assert "set_torque" in names
    assert "set_all_torque" not in names


def test_move_checks_servos_before_and_after_motion():
    fn = _function("_execute_move")
    names = [_called_name(call) for call in _calls(fn)]

    assert names.count("_require_operational_servos") >= 2
    assert "go" in names


def test_fold_to_cradle_checks_servos_before_and_after_motion():
    fn = _function("_on_fold_to_cradle")
    names = [_called_name(call) for call in _calls(fn)]

    assert names.count("_require_operational_servos") >= 2
    assert "go" in names


def test_gripper_checks_servo_and_position_write_result():
    fn = _function("_on_set_gripper")
    names = [_called_name(call) for call in _calls(fn)]

    assert "_require_operational_servos" in names
    assert "set_position" in names


def test_hold_position_does_not_use_lossy_bulk_torque_helper():
    fn = _function("_on_hold_position")
    names = [_called_name(call) for call in _calls(fn)]

    assert "set_torque" in names
    assert "set_all_torque" not in names


def test_load_read_failure_is_logged():
    fn = _function("_read_load")
    names = [_called_name(call) for call in _calls(fn)]

    assert "get_load" in names
    assert "warn" in names


def test_startup_hardware_failure_is_caught_by_main():
    main = _function("main")

    caught_names = set()
    for handler in [node for node in ast.walk(main) if isinstance(node, ast.ExceptHandler)]:
        typ = handler.type
        if isinstance(typ, ast.Name):
            caught_names.add(typ.id)
        elif isinstance(typ, ast.Tuple):
            caught_names.update(elt.id for elt in typ.elts if isinstance(elt, ast.Name))

    assert "ArmHardwareUnavailableError" in caught_names
