"""Static safety contracts for the attended horizontal grasp harness."""

import ast
import pathlib

SCRIPT = (
    pathlib.Path(__file__).resolve().parent.parent / "tools" / "horizontal_grasp_hardware_test.py"
)


def _function(name):
    tree = ast.parse(SCRIPT.read_text(encoding="utf-8"), filename=str(SCRIPT))
    return next(
        node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == name
    )


def _called_names(function):
    names = []
    for call in (node for node in ast.walk(function) if isinstance(node, ast.Call)):
        if isinstance(call.func, ast.Name):
            names.append(call.func.id)
        elif isinstance(call.func, ast.Attribute):
            names.append(call.func.attr)
    return names


def test_every_motion_stage_is_operator_gated():
    main = _function("main")
    names = _called_names(main)

    assert names.count("confirm") == 10
    assert names.count("glide") == 6
    assert names.count("set_width") == 4


def test_unknown_start_pose_is_rejected_before_motion():
    main = _function("main")
    names = _called_names(main)

    assert "near_pose" in names
    assert "RuntimeError" in names


def test_operator_can_abort_before_next_transition():
    confirm = _function("confirm")
    names = _called_names(confirm)

    assert "input" in names
    assert "KeyboardInterrupt" in names


def test_retry_width_is_clamped_to_calibrated_closed_limit():
    main = _function("main")
    names = _called_names(main)

    assert "max" in names


def test_load_is_checked_again_after_mid_lift():
    main = _function("main")
    names = _called_names(main)

    assert names.count("get_load") >= 2
    assert names.count("RuntimeError") >= 3


def test_hot_servo2_is_rejected_before_motion():
    main = _function("main")
    names = _called_names(main)

    assert "get_temperature" in names
    assert names.index("get_temperature") < names.index("glide")
