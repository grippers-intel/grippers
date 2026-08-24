"""Hardware-free unit tests for tools/perception/approach_placement_check.py.

Only the pure math (estimate_pose, _format_line) is tested here — the rclpy
node glue (PlacementCheckNode, run()) needs a live ROS graph and isn't
covered, matching the repo's convention for other rclpy CLI tools (e.g.
capture_ros.py, scan_track_return.py)."""

import importlib.util
import math
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "tools" / "perception" / "approach_placement_check.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("approach_placement_check", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


check = _load_module()


class FakeResult:
    def __init__(self, found, x=0.0, h=0.0, w=0.0):
        self.found = found
        self.x = x
        self.h = h
        self.w = w


def test_estimate_pose_matches_perception_node_formula():
    # perception_node.py: z_m = k/sqrt(h*w), y_obj = -(u-cx)*z_m/fx
    z_m, y_obj_m = check.estimate_pose(k_class=37.3992, h_px=100.0, w_px=100.0, u_px=320.0, fx=600.0, cx=320.0)

    assert z_m == 37.3992 / math.sqrt(100.0 * 100.0)
    assert y_obj_m == 0.0  # centered in frame (u == cx) -> zero lateral offset


def test_estimate_pose_object_left_of_center_is_positive_offset():
    # u < cx means the object sits left of the optical center; perception_node's
    # sign convention treats that as a positive (+left) y_obj.
    z_m, y_obj_m = check.estimate_pose(k_class=20.0, h_px=50.0, w_px=50.0, u_px=200.0, fx=600.0, cx=320.0)

    assert y_obj_m > 0


def test_estimate_pose_unmeasured_class_returns_none():
    z_m, y_obj_m = check.estimate_pose(k_class=None, h_px=100.0, w_px=100.0, u_px=320.0, fx=600.0, cx=320.0)

    assert (z_m, y_obj_m) == (None, None)


def test_estimate_pose_missing_camera_info_returns_none():
    z_m, y_obj_m = check.estimate_pose(k_class=20.0, h_px=100.0, w_px=100.0, u_px=320.0, fx=None, cx=None)

    assert (z_m, y_obj_m) == (None, None)


def test_estimate_pose_zero_area_returns_none():
    z_m, y_obj_m = check.estimate_pose(k_class=20.0, h_px=0.0, w_px=50.0, u_px=320.0, fx=600.0, cx=320.0)

    assert (z_m, y_obj_m) == (None, None)


def test_format_line_no_service_response():
    line = check._format_line("rook", None, k_class=37.3992, fx=600.0, cx=320.0)

    assert "응답 없음" in line


def test_format_line_not_found():
    line = check._format_line("rook", FakeResult(found=False), k_class=37.3992, fx=600.0, cx=320.0)

    assert "미검출" in line


def test_format_line_unmeasured_class_shows_raw_only():
    result = FakeResult(found=True, x=310.0, h=40.0, w=35.0)
    line = check._format_line("box", result, k_class=None, fx=600.0, cx=320.0)

    assert "미실측" in line
    assert "h=40" in line and "w=35" in line


def test_format_line_measured_class_shows_distance_and_offset():
    result = FakeResult(found=True, x=320.0, h=100.0, w=100.0)
    line = check._format_line("rook", result, k_class=37.3992, fx=600.0, cx=320.0)

    assert "거리=" in line
    assert "좌우오프셋=" in line
