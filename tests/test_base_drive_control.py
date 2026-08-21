"""Issue #148 잔여: 도착 근처에서 목표를 등진 채 멀어지는 회귀 테스트.

#166 이 회전 발산을 막았지만 DRIVE 단계 속도가 `KP_LINEAR * dist` 로 **부호가
없어서**, 재정렬을 하지 않는 근접 구간(`REALIGN_MIN_DIST_M` 안)에서 목표가 등
뒤로 넘어가면 그대로 전진해 거리가 늘었다.
"""

import importlib.util
import math
import pathlib

import pytest

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


def _yaw_error(dx, dy, yaw):
    target_yaw = math.atan2(dy, dx)
    return math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))


def test_issue_148_recorded_point_moves_toward_the_target():
    """실기에서 기록된 지점. 예전 식은 여기서 목표에서 멀어졌다."""
    control = _load_control()
    dx, dy, yaw = -0.0249, -0.0168, 1.7199

    speed = control.forward_speed(math.hypot(dx, dy), _yaw_error(dx, dy, yaw))

    assert speed < 0.0, "목표가 뒤에 있으면 후진해야 한다"
    # 명령 속도 벡터를 목표 방향에 투영하면 양수여야 거리가 줄어든다.
    assert speed * (dx * math.cos(yaw) + dy * math.sin(yaw)) > 0.0


def test_unsigned_distance_would_have_driven_away():
    """예전 식(`KP_LINEAR * dist`)이 왜 틀렸는지를 테스트로 남긴다."""
    control = _load_control()
    dx, dy, yaw = -0.0249, -0.0168, 1.7199
    dist = math.hypot(dx, dy)

    old_speed = control.KP_LINEAR * dist

    assert old_speed > 0.0
    assert old_speed * (dx * math.cos(yaw) + dy * math.sin(yaw)) < 0.0


def test_aligned_target_matches_previous_behaviour():
    """정렬된 상태에서는 cos(0)=1 이라 기존 식과 같은 값이 나온다 (클램프 아래 거리)."""
    control = _load_control()

    assert control.forward_speed(0.2, 0.0) == control.KP_LINEAR * 0.2


def test_speed_is_clamped_both_directions():
    control = _load_control()

    assert control.forward_speed(100.0, 0.0) == control.MAX_LINEAR
    assert control.forward_speed(100.0, math.pi) == -control.MAX_LINEAR


def test_perpendicular_target_commands_no_translation():
    control = _load_control()

    assert control.forward_speed(0.5, math.pi / 2) == pytest.approx(0.0, abs=1e-12)
