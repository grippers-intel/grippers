"""Static safety contracts for the attended empty basket-pose harness."""

import ast
import pathlib

SCRIPT = (
    pathlib.Path(__file__).resolve().parent.parent / "tools" / "basket_drop_pose_hardware_test.py"
)


def _main_calls():
    tree = ast.parse(SCRIPT.read_text(encoding="utf-8"), filename=str(SCRIPT))
    main = next(
        node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == "main"
    )
    return [
        call.func.id if isinstance(call.func, ast.Name) else call.func.attr
        for call in ast.walk(main)
        if isinstance(call, ast.Call)
    ]


def test_each_empty_hand_motion_is_operator_gated():
    calls = _main_calls()

    assert calls.count("confirm") == 4
    assert calls.count("glide_raw") == 2


def test_return_to_idle_waits_for_convergence_and_closes_gripper():
    # Real-hardware regression (2026-08-21), same root cause as
    # horizontal_grasp_hardware_test.py: glide_raw's fixed step schedule can
    # end before present actually reaches goal on a large sweep, and this
    # script also left the gripper open (from the empty-hand 80mm test open)
    # instead of the IDLE convention of CLOSED.
    calls = _main_calls()

    assert calls.count("wait_until_converged") == 2
    assert calls.count("position_from_width") == 2


def test_start_pose_temperature_and_torque_are_checked_before_motion():
    calls = _main_calls()

    assert "near_pose" in calls
    assert "get_temperature" in calls
    assert "get_torque" in calls
    assert calls.index("get_temperature") < calls.index("glide_raw")
