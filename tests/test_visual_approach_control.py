"""visual_approach_control.py 순수 수학 테스트. rclpy/카메라 없이도 돈다 —
HANDOFF.md(2026-08-23)가 실기로 검증한 tools/perception/approach.py의
오차→속도 지령 변환 로직만 옮겨서 본다(drive_control.py 테스트와 같은 이유)."""

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
    assert (cmd.linear_x, cmd.linear_y, cmd.burst_s) == (0.0, 0.0, 0.0)


def test_not_arrived_when_either_error_outside_tolerance():
    cmd = control.compute_approach_command(err_x=5.0, err_h=20.0, tol_x=8.0, tol_h=6.0)
    assert cmd.arrived is False


def test_align_first_slows_forward_speed_when_lateral_error_is_large():
    """실측 실패 사례(HANDOFF.md) — 좌우가 크게 어긋난 채로 전진하면 물체를
    지나쳐버린다. align_first가 켜져 있으면 전진 속도가 1/4로 줄어야 한다."""
    # tol_x=8이므로 align_first=2.0 기준 임계는 16px. err_x=30은 그걸 넘는다.
    without_align = control.compute_approach_command(
        err_x=30.0, err_h=50.0, tol_x=8.0, tol_h=6.0, align_first=0.0,
        min_speed=0.0,  # apply_floor의 증폭을 끄고 비율만 비교한다
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


def test_apply_floor_scales_up_below_deadband():
    """0.017 m/s 아래 지령은 바퀴가 안 돈다(실측) — apply_floor가 min_speed까지
    끌어올리고 그만큼 burst 시간을 줄여 이동 거리를 유지해야 한다."""
    vx, vy, burst = control.apply_floor(
        vx=0.01, vy=0.0, min_speed=0.05, max_speed=0.08, burst=0.35, min_burst=0.15,
    )
    assert vx == 0.05
    assert vy == 0.0
    assert burst < 0.35  # 속도를 5배 키웠으니 시간은 그만큼 줄어야 한다(최소치 하한 있음)


def test_apply_floor_drops_command_still_under_deadband_after_scaling():
    """상한(max_speed)에 걸려도 여전히 데드밴드 아래인 성분은 0으로 버린다 —
    모터만 울릴 뿐 실제로 안 움직이므로."""
    vx, vy, burst = control.apply_floor(
        vx=0.09, vy=0.001, min_speed=0.05, max_speed=0.08, burst=0.35, min_burst=0.15,
    )
    assert vy == 0.0


def test_invert_y_flips_lateral_command_sign():
    normal = control.compute_approach_command(
        err_x=20.0, err_h=0.0, tol_x=8.0, tol_h=6.0, min_speed=0.0, invert_y=False,
    )
    inverted = control.compute_approach_command(
        err_x=20.0, err_h=0.0, tol_x=8.0, tol_h=6.0, min_speed=0.0, invert_y=True,
    )
    assert inverted.linear_y == -normal.linear_y
