"""scan_track_control.py 순수 수학 테스트. rclpy/카메라 없이도 돈다.

2026-08-24: SCAN 대상 추적(제자리회전+직진 분리 단계), 원위치 복귀(직선
벡터 방식), 시각 기반 장애물 회피(LiDAR 대신 YOLO) 세 기능의 제어 수학."""

import importlib.util
import math
import pathlib

MODULE = (
    pathlib.Path(__file__).resolve().parent.parent
    / "ros2_ws"
    / "src"
    / "grippers_base"
    / "grippers_base"
    / "scan_track_control.py"
)


def _load():
    spec = importlib.util.spec_from_file_location("scan_track_control", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


control = _load()


# --- establish_target_h / bbox_area_distance_m ----------------------------


def test_establish_target_h_scales_by_distance_ratio():
    # 현재 z_m = K/sqrt(h*w). obs_h=100, obs_w=50, K=37.3992 => area=5000,
    # z_m = 37.3992/sqrt(5000) ≈ 0.5288m. target 0.35m로 스케일하면
    # target_h = 100 * (0.5288/0.35) ≈ 151.1
    target_h = control.establish_target_h(obs_h=100.0, obs_w=50.0, k_class=37.3992, target_distance_m=0.35)
    z_now = 37.3992 / math.sqrt(100.0 * 50.0)
    assert target_h == 100.0 * (z_now / 0.35)


def test_establish_target_h_none_when_k_class_unmeasured():
    assert control.establish_target_h(100.0, 50.0, None, 0.35) is None


def test_z_from_established_h_is_inverse_of_establish_target_h():
    target_h = control.establish_target_h(obs_h=100.0, obs_w=50.0, k_class=37.3992, target_distance_m=0.35)
    # obs_h가 target_h와 정확히 같아지는 순간(목표 지점 도달)에는 거리도
    # 정확히 target_distance_m(0.35)이어야 한다 — 이게 진짜 역함수 관계다.
    z = control.z_from_established_h(obs_h=target_h, target_h=target_h, target_distance_m=0.35)
    assert math.isclose(z, 0.35, rel_tol=1e-9)


def test_z_from_established_h_smaller_h_means_farther():
    z_far = control.z_from_established_h(obs_h=50.0, target_h=150.0, target_distance_m=0.35)
    z_near = control.z_from_established_h(obs_h=300.0, target_h=150.0, target_distance_m=0.35)
    assert z_far > z_near


def test_bbox_area_distance_m_matches_perception_node_formula():
    d = control.bbox_area_distance_m(obs_h=100.0, obs_w=50.0, k_class=37.3992)
    assert d == 37.3992 / math.sqrt(5000.0)


def test_bbox_area_distance_m_none_when_k_class_unmeasured():
    assert control.bbox_area_distance_m(100.0, 50.0, None) is None


# --- h_signal_reliable ------------------------------------------------------


def test_h_signal_reliable_true_when_aspect_matches_reference():
    # ref aspect = 0.5 (w/h), obs aspect = 50/100 = 0.5 -> 편차 0
    assert control.h_signal_reliable(obs_h=100.0, obs_w=50.0, ref_aspect_ratio=0.5) is True


def test_h_signal_reliable_false_when_aspect_deviates_too_much():
    # ref aspect 0.5, obs aspect 50/50=1.0 -> 편차 100% > 40% 임계
    assert control.h_signal_reliable(obs_h=50.0, obs_w=50.0, ref_aspect_ratio=0.5) is False


# --- compute_align_command / compute_drive_command --------------------------


def test_align_command_arrived_within_tolerance():
    cmd = control.compute_align_command(err_x=5.0, tol_x=15.0)
    assert cmd.arrived is True
    assert (cmd.linear_x, cmd.linear_y, cmd.angular_z) == (0.0, 0.0, 0.0)


def test_align_command_is_pure_rotation_never_moves_linearly():
    cmd = control.compute_align_command(err_x=100.0, tol_x=15.0, turn_speed=0.8)
    assert cmd.linear_x == 0.0
    assert cmd.linear_y == 0.0
    assert cmd.angular_z != 0.0
    assert cmd.arrived is False


def test_align_command_object_right_turns_right():
    # +err_x = 물체가 오른쪽 -> 오른쪽으로 돌아야 함 -> angular.z<0 (REP103)
    cmd = control.compute_align_command(err_x=50.0, tol_x=15.0, turn_speed=0.8)
    assert cmd.angular_z < 0.0


def test_align_command_object_left_turns_left():
    cmd = control.compute_align_command(err_x=-50.0, tol_x=15.0, turn_speed=0.8)
    assert cmd.angular_z > 0.0


def test_drive_command_arrived_within_tolerance():
    cmd = control.compute_drive_command(err_dist_m=0.01, tol_dist_m=0.03)
    assert cmd.arrived is True
    assert (cmd.linear_x, cmd.linear_y, cmd.angular_z) == (0.0, 0.0, 0.0)


def test_drive_command_is_pure_linear_never_turns():
    cmd = control.compute_drive_command(err_dist_m=0.5, tol_dist_m=0.03, speed=0.06)
    assert cmd.angular_z == 0.0
    assert cmd.linear_y == 0.0
    assert cmd.linear_x > 0.0
    assert cmd.arrived is False


def test_drive_command_negative_error_backs_up():
    cmd = control.compute_drive_command(err_dist_m=-0.5, tol_dist_m=0.03, speed=0.06)
    assert cmd.linear_x < 0.0


# --- 원위치 복귀 -------------------------------------------------------------


def test_normalize_angle_wraps_to_pi_range():
    assert math.isclose(control.normalize_angle_rad(3 * math.pi), math.pi, abs_tol=1e-9) or \
        math.isclose(control.normalize_angle_rad(3 * math.pi), -math.pi, abs_tol=1e-9)
    assert -math.pi <= control.normalize_angle_rad(10.0) <= math.pi


def test_yaw_from_quaternion_identity_is_zero():
    assert control.yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == 0.0


def test_yaw_from_quaternion_90_degrees():
    # z=sin(45deg), w=cos(45deg) -> yaw=90deg
    half = math.pi / 4
    yaw = control.yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half))
    assert math.isclose(yaw, math.pi / 2, abs_tol=1e-6)


def test_compute_return_vector_straight_ahead_needs_no_turn():
    # 시작점이 현재 위치 기준 정면(+x)에 있고, 현재 yaw도 0(정면) -> 회전 불필요
    heading_error, distance = control.compute_return_vector(
        start_x_m=1.0, start_y_m=0.0, current_x_m=0.0, current_y_m=0.0, current_yaw_rad=0.0,
    )
    assert math.isclose(heading_error, 0.0, abs_tol=1e-9)
    assert math.isclose(distance, 1.0, abs_tol=1e-9)


def test_compute_return_vector_behind_needs_half_turn():
    heading_error, distance = control.compute_return_vector(
        start_x_m=-1.0, start_y_m=0.0, current_x_m=0.0, current_y_m=0.0, current_yaw_rad=0.0,
    )
    assert math.isclose(abs(heading_error), math.pi, abs_tol=1e-9)
    assert math.isclose(distance, 1.0, abs_tol=1e-9)


def test_compute_return_vector_distance_is_euclidean():
    _, distance = control.compute_return_vector(
        start_x_m=3.0, start_y_m=4.0, current_x_m=0.0, current_y_m=0.0, current_yaw_rad=0.0,
    )
    assert math.isclose(distance, 5.0, abs_tol=1e-9)


# --- 시각 기반 장애물 회피 ---------------------------------------------------


def test_lateral_offset_m_matches_pinhole_formula():
    off = control.lateral_offset_m(obs_x=400.0, z_m=0.5, fx_px=600.0, cx_px=320.0)
    assert off == (400.0 - 320.0) * 0.5 / 600.0


def test_find_path_obstacle_ignores_things_outside_corridor():
    obs = [control.ObstacleObservation(cls="knight", forward_m=0.3, lateral_m=0.5)]
    assert control.find_path_obstacle(obs, path_half_width_m=0.15) is None


def test_find_path_obstacle_ignores_things_beyond_target():
    obs = [control.ObstacleObservation(cls="knight", forward_m=0.6, lateral_m=0.0)]
    assert control.find_path_obstacle(obs, path_half_width_m=0.15, max_range_m=0.35) is None


def test_find_path_obstacle_picks_closest_within_corridor():
    far = control.ObstacleObservation(cls="knight", forward_m=0.3, lateral_m=0.05)
    near = control.ObstacleObservation(cls="queen", forward_m=0.15, lateral_m=-0.05)
    result = control.find_path_obstacle([far, near], path_half_width_m=0.15, max_range_m=0.4)
    assert result is near


def test_choose_dodge_side_avoids_obstacle_on_right():
    # 장애물이 우측(+lateral)이면 좌측(+1.0)으로 피한다
    assert control.choose_dodge_side(obstacle_lateral_m=0.1) == 1.0


def test_choose_dodge_side_avoids_obstacle_on_left():
    # 장애물이 좌측(-lateral)이면 우측(-1.0)으로 피한다
    assert control.choose_dodge_side(obstacle_lateral_m=-0.1) == -1.0


def test_choose_dodge_side_defaults_right_when_centered():
    assert control.choose_dodge_side(obstacle_lateral_m=0.0) == -1.0


def test_compute_dodge_command_only_moves_laterally():
    cmd = control.compute_dodge_command(dodge_side=1.0, dodge_speed=0.05)
    assert cmd.linear_x == 0.0
    assert cmd.angular_z == 0.0
    assert cmd.linear_y == 0.05
    assert cmd.arrived is False


def test_compute_dodge_command_negative_side_goes_right():
    cmd = control.compute_dodge_command(dodge_side=-1.0, dodge_speed=0.05)
    assert cmd.linear_y == -0.05
