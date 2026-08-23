"""visual_approach_control.py 순수 수학 테스트. rclpy/카메라 없이도 돈다.

2026-08-23 재설계: 좌우-이동(linear_y) 정렬을 제자리 회전(angular_z) 정렬로
바꿨다(사용자 지적 — 회전 없이 순수 이동만 쓰면 초기 방위각 오차를 전부
좌우 이동으로 상쇄해야 해서 접근 경로가 불필요하게 지그재그였다). 이
회전+전진 조합 자체는 아직 실기 미검증이다 — 여기서는 수학만 검증한다."""

import importlib.util
import pathlib

MODULE = (
    pathlib.Path(__file__).resolve().parent.parent
    / "ros2_ws"
    / "src"
    / "grippers_base"
    / "grippers_base"
    / "visual_approach_control.py"
)


def _load():
    spec = importlib.util.spec_from_file_location("visual_approach_control", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


control = _load()


def test_compute_approach_error_signs():
    """+err_x = 물체가 목표보다 오른쪽, +err_h = 아직 멀다(박스가 작다)."""
    err_x, err_h = control.compute_approach_error(obs_x=180.0, obs_h=90.0, target_x=168.0, target_h=100.0)
    assert err_x == 12.0
    assert err_h == 10.0


def test_arrived_when_both_errors_within_tolerance():
    cmd = control.compute_approach_command(err_x=5.0, err_h=4.0, tol_x=8.0, tol_h=6.0)
    assert cmd.arrived is True
    assert (cmd.linear_x, cmd.linear_y, cmd.angular_z, cmd.burst_s) == (0.0, 0.0, 0.0, 0.0)


def test_not_arrived_when_either_error_outside_tolerance():
    cmd = control.compute_approach_command(err_x=5.0, err_h=20.0, tol_x=8.0, tol_h=6.0)
    assert cmd.arrived is False


def test_linear_y_is_always_zero_in_nominal_approach():
    """정렬은 이제 회전만 담당한다 — 장애물 회피 중이 아니면 linear_y는 항상 0."""
    cmd = control.compute_approach_command(err_x=40.0, err_h=40.0, tol_x=8.0, tol_h=6.0)
    assert cmd.linear_y == 0.0


def test_object_to_the_right_turns_clockwise():
    """err_x>0(물체가 화면 오른쪽) → REP103 관례상 오른쪽으로 돌려면 angular.z<0."""
    cmd = control.compute_approach_command(
        err_x=40.0, err_h=0.0, tol_x=8.0, tol_h=6.0, min_turn=0.0,
    )
    assert cmd.angular_z < 0.0


def test_invert_turn_flips_rotation_sign():
    normal = control.compute_approach_command(
        err_x=40.0, err_h=0.0, tol_x=8.0, tol_h=6.0, min_turn=0.0, invert_turn=False,
    )
    inverted = control.compute_approach_command(
        err_x=40.0, err_h=0.0, tol_x=8.0, tol_h=6.0, min_turn=0.0, invert_turn=True,
    )
    assert inverted.angular_z == -normal.angular_z


def test_align_first_slows_forward_speed_when_bearing_error_is_large():
    """실측 실패 사례(HANDOFF.md) — 방위가 크게 어긋난 채로 전진하면 물체를
    지나쳐버린다. align_first가 켜져 있으면 전진 속도가 1/4로 줄어야 한다."""
    # tol_x=8이므로 align_first=2.0 기준 임계는 16px. err_x=30은 그걸 넘는다.
    without_align = control.compute_approach_command(
        err_x=30.0, err_h=50.0, tol_x=8.0, tol_h=6.0, align_first=0.0,
        min_speed=0.0,  # apply_axis_floor의 증폭을 끄고 비율만 비교한다
    )
    with_align = control.compute_approach_command(
        err_x=30.0, err_h=50.0, tol_x=8.0, tol_h=6.0, align_first=2.0,
        min_speed=0.0,
    )
    assert with_align.linear_x == without_align.linear_x * 0.25


def test_speed_command_is_clamped_to_max_speed():
    cmd = control.compute_approach_command(
        err_x=0.0, err_h=100000.0, tol_x=8.0, tol_h=6.0, max_speed=0.08, min_speed=0.0,
    )
    assert cmd.linear_x == 0.08


def test_turn_command_is_clamped_to_max_turn():
    cmd = control.compute_approach_command(
        err_x=100000.0, err_h=0.0, tol_x=8.0, tol_h=6.0, max_turn=0.5, min_turn=0.0,
    )
    assert abs(cmd.angular_z) == 0.5


def test_apply_axis_floor_scales_up_nonzero_below_deadband():
    assert control.apply_axis_floor(0.01, min_v=0.05, max_v=0.08) == 0.05
    assert control.apply_axis_floor(-0.01, min_v=0.05, max_v=0.08) == -0.05


def test_apply_axis_floor_leaves_zero_untouched():
    assert control.apply_axis_floor(0.0, min_v=0.05, max_v=0.08) == 0.0


def test_apply_axis_floor_clamps_to_max():
    assert control.apply_axis_floor(0.5, min_v=0.05, max_v=0.08) == 0.08


def test_obstacle_ahead_true_when_closer_than_safety_distance():
    assert control.obstacle_ahead(0.2, safety_distance_m=0.35) is True


def test_obstacle_ahead_false_when_farther_than_safety_distance():
    assert control.obstacle_ahead(0.5, safety_distance_m=0.35) is False


def test_obstacle_ahead_false_when_unknown():
    """라이다 데이터가 없으면(None) 막지 않는다 — '모르면 이 기능이 없던 것처럼'
    (모르면 막는다로 하면 라이다 미기동 시 접근이 영원히 회피만 반복한다)."""
    assert control.obstacle_ahead(None) is False


def test_choose_dodge_side_prefers_more_open_side():
    assert control.choose_dodge_side(left_min_m=1.0, right_min_m=0.3) == 1.0
    assert control.choose_dodge_side(left_min_m=0.3, right_min_m=1.0) == -1.0


def test_choose_dodge_side_defaults_when_unknown():
    assert control.choose_dodge_side(None, None) == -1.0
    assert control.choose_dodge_side(None, 0.5) == -1.0
    assert control.choose_dodge_side(0.5, None) == 1.0


def test_compute_dodge_command_only_moves_laterally():
    cmd = control.compute_dodge_command(dodge_side=1.0, dodge_speed=0.06, dodge_burst=0.4)
    assert cmd.linear_x == 0.0
    assert cmd.angular_z == 0.0
    assert cmd.linear_y == 0.06
    assert cmd.burst_s == 0.4
    assert cmd.arrived is False


def test_compute_dodge_command_negative_side_goes_right():
    cmd = control.compute_dodge_command(dodge_side=-1.0, dodge_speed=0.06)
    assert cmd.linear_y == -0.06


def test_min_range_in_arc_finds_closest_valid_return_in_window():
    # angle_min=0, increment=90deg → 인덱스 0,1,2,3 = 0°,90°,180°,270°(=-90°)
    ranges = [5.0, 1.2, float("inf"), 0.9]
    assert control.min_range_in_arc(0.0, 1.5707963267948966, ranges, center_deg=0.0, half_width_deg=10.0) == 5.0
    assert control.min_range_in_arc(0.0, 1.5707963267948966, ranges, center_deg=-90.0, half_width_deg=10.0) == 0.9


def test_min_range_in_arc_ignores_nonfinite_and_nonpositive():
    ranges = [float("inf"), 0.0, -1.0, float("nan")]
    assert control.min_range_in_arc(0.0, 1.5707963267948966, ranges, center_deg=0.0, half_width_deg=180.0) is None


def test_min_range_in_arc_none_when_window_empty():
    ranges = [5.0]
    assert control.min_range_in_arc(0.0, 1.0, ranges, center_deg=170.0, half_width_deg=5.0) is None
