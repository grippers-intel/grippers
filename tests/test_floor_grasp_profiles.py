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
    assert profiles["star_column"].object_width_mm == 45.0
    assert profiles["soccer_polyhedron"].object_width_mm == 46.0
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
