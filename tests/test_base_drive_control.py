"""Issue #148: 도착 근처 drive_to 회전 발산 회귀 테스트."""

import importlib.util
import math
import pathlib

CONTROL_MODULE = (
    pathlib.Path(__file__).resolve().parent.parent
    / "ros2_ws"
    / "src"
    / "grippers_base"
    / "grippers_base"
    / "drive_control.py"
)


def _load_control():
    spec = importlib.util.spec_from_file_location("drive_control", CONTROL_MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_issue_148_reproduction_turns_off_rotation_and_reduces_distance():
    control = _load_control()
    dx, dy, yaw = -0.0249, -0.0168, 1.7199

    command = control.compute_drive_command(dx, dy, yaw)

    assert not command.arrived
    assert command.angular_z == 0.0
    assert command.linear_x < 0.0

    velocity_toward_target = command.linear_x * (dx * math.cos(yaw) + dy * math.sin(yaw))
    assert velocity_toward_target > 0.0


def test_arrival_boundary_is_inclusive():
    control = _load_control()

    command = control.compute_drive_command(control.ARRIVE_XY_TOL, 0.0, 0.0)

    assert command.arrived
    assert command.linear_x == 0.0
    assert command.angular_z == 0.0


def test_deadzone_is_strictly_larger_than_arrival_tolerance():
    control = _load_control()

    assert control.YAW_DEADZONE_XY > control.ARRIVE_XY_TOL


def test_far_target_rotates_before_driving_when_facing_away():
    control = _load_control()

    command = control.compute_drive_command(1.0, 0.0, math.pi)

    assert not command.arrived
    assert command.linear_x == 0.0
    assert abs(command.angular_z) == control.MAX_ANGULAR


def test_far_aligned_target_drives_forward_without_rotation():
    control = _load_control()

    command = control.compute_drive_command(1.0, 0.0, 0.0)

    assert not command.arrived
    assert command.linear_x == control.MAX_LINEAR
    assert command.angular_z == 0.0
