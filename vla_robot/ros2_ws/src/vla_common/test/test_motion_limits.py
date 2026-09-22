import pytest

from vla_common.motion_limits import MotionLimits, resolve_motion

LIMITS = MotionLimits(max_linear_mps=0.15, max_angular_rad_s=0.5)


def test_stop_wins():
    d = resolve_motion(1.0, 1.0, 1.0, True, LIMITS)
    assert d.ok and d.motion.is_stop


def test_clamps_magnitude_keeps_sign():
    d = resolve_motion(-1.0, 0.0, 0.0, False, LIMITS)
    assert d.ok and d.clamped
    assert d.motion.linear_x == pytest.approx(-0.15)


def test_rejects_mixed_rotation():
    d = resolve_motion(0.1, 0.0, 0.3, False, LIMITS)
    assert not d.ok and d.motion.is_stop


def test_allows_diagonal_translation():
    d = resolve_motion(0.1, 0.1, 0.0, False, LIMITS)
    assert d.ok and not d.clamped


def test_rejects_nan():
    d = resolve_motion(float("nan"), 0.0, 0.0, False, LIMITS)
    assert not d.ok and d.motion.is_stop


def test_float_noise_is_not_rotation():
    d = resolve_motion(0.1, 0.0, 1e-12, False, LIMITS)
    assert d.ok
