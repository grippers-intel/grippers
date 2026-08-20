"""Contracts for measured and proposed SO-ARM101 floor-grasp profiles."""

import importlib.util
import pathlib

PROFILE_MODULE = (
    pathlib.Path(__file__).resolve().parent.parent
    / "ros2_ws"
    / "src"
    / "grippers_arm"
    / "grippers_arm"
    / "floor_grasp_profiles.py"
)


def _load_profiles():
    spec = importlib.util.spec_from_file_location("floor_grasp_profiles", PROFILE_MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_floor_grasp_profiles_match_measured_object_geometry():
    module = _load_profiles()
    profiles = module.FLOOR_GRASP_PROFILES

    assert (profiles["cube"].object_width_mm, profiles["cube"].grasp_center_height_mm) == (
        40.0,
        20.0,
    )
    assert profiles["cube"].close_width_mm == 30.0
    assert profiles["star_column"].object_width_mm == 45.0
    assert profiles["star_column"].close_width_mm == 35.0
    assert profiles["soccer_polyhedron"].object_width_mm == 46.0
    assert profiles["soccer_polyhedron"].close_width_mm == 35.0
    assert profiles["chess_rook"].close_width_mm == 15.0
    assert profiles["chess_knight"].close_width_mm == 13.0
    assert all(profile.preopen_width_mm == 80.0 for profile in profiles.values())
    assert (
        profiles["chess_knight"].object_width_mm,
        profiles["chess_knight"].grasp_center_height_mm,
    ) == (22.0, 60.0)
    assert (
        profiles["chess_rook"].object_width_mm,
        profiles["chess_rook"].grasp_center_height_mm,
    ) == (24.5, 45.0)
    assert (
        profiles["chess_queen"].object_width_mm,
        profiles["chess_queen"].grasp_center_height_mm,
    ) == (17.0, 50.0)


def test_floor_grasp_commands_are_ordered_and_inside_safe_calibration_range():
    module = _load_profiles()

    for profile in module.FLOOR_GRASP_PROFILES.values():
        assert 9.0 <= profile.close_width_mm < profile.object_width_mm
        assert profile.object_width_mm < profile.preopen_width_mm <= 168.0


def test_hardware_acceptance_contract_records_verified_cube_load():
    module = _load_profiles()

    assert module.MEASURED_CUBE_HOLD_LOAD_RATIO == 0.0704
    assert module.HARDWARE_HOLD_SECONDS == 5.0
    assert module.MIN_GRIPPER_CLEARANCE_MM == 140.0


def test_horizontal_arm_poses_keep_gabe_and_chess_heights_separate():
    module = _load_profiles()

    assert module.HORIZONTAL_SAFE_140_DEG == (-1.67, 45.87, 37.34, -83.70, 84.30)
    assert module.HORIZONTAL_CHESS_MID_40_DEG == (-1.67, 96.57, -9.79, -87.29, 84.30)
    assert module.HORIZONTAL_GABE_LOW_20_DEG == (-1.39, 95.70, -18.16, -68.88, 84.18)
    assert module.HORIZONTAL_CHESS_MID_40_DEG != module.HORIZONTAL_GABE_LOW_20_DEG


def test_every_object_profile_has_a_horizontal_arm_pose():
    module = _load_profiles()

    assert set(module.HORIZONTAL_GRASP_POSES_DEG) == set(module.FLOOR_GRASP_PROFILES)
    assert module.HORIZONTAL_GRASP_POSES_DEG["chess_rook"] == (
        -1.67,
        93.87,
        -6.32,
        -88.06,
        84.30,
    )
    assert module.HORIZONTAL_GRASP_POSES_DEG["chess_queen"][1] == 91.23
    assert module.HORIZONTAL_GRASP_POSES_DEG["chess_knight"][1] == 86.10
