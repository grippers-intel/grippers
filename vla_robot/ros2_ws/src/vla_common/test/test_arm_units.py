import json
from pathlib import Path

import pytest

from vla_common.arm_units import ArmCalibration, CalibrationError, with_base_yaw

CALIB = Path(__file__).resolve().parents[2] / "vla_robot_bringup" / "config" / "arm_calibration.json"


@pytest.fixture(scope="module")
def calib():
    return ArmCalibration.load(CALIB)


def test_mid_is_zero_degrees(calib):
    raw = [int(j.mid) for j in calib.joints]
    raw[5] = calib.joints[5].range_min
    pol = calib.raw_to_policy(raw)
    for j, v in zip(calib.joints[:5], pol[:5]):
        assert abs(v) < 360.0 / 4095 * 1.0  # int(mid) 절삭 오차 1틱 이내
    assert pol[5] == pytest.approx(0.0)


def test_gripper_range_maps_to_0_100(calib):
    g = calib.joints[5]
    assert calib.raw_to_policy([2048] * 5 + [g.range_max])[5] == pytest.approx(100.0)
    # 범위 밖은 잘린다(lerobot 과 동일)
    assert calib.raw_to_policy([2048] * 5 + [g.range_max + 500])[5] == pytest.approx(100.0)


def test_degrees_are_not_clamped(calib):
    # wrist_roll 범위는 14틱뿐이지만 바깥 값도 그대로 보고해야 한다
    wr = calib.joints[4]
    far = wr.range_max + 400
    assert calib.raw_to_policy([2048, 2048, 2048, 2048, far, 2000])[4] > 30.0


def test_roundtrip_within_one_tick(calib):
    raw = [2008, 900, 3024, 2868, 2047, 2100]
    back = calib.policy_to_raw(calib.raw_to_policy(raw))
    for a, b in zip(raw, back):
        assert abs(a - b) <= 1


def test_matches_lerobot_int_truncation(calib):
    j = calib.joints[0]
    value = 10.0
    expected = int(value * 4095 / 360 + (j.range_min + j.range_max) / 2)
    assert calib.policy_to_raw([value, 0, 0, 0, 0, 0])[0] == expected


def test_rejects_bad_calibration(tmp_path):
    data = json.loads(CALIB.read_text(encoding="utf-8"))
    data["gripper"]["range_max"] = data["gripper"]["range_min"]
    with pytest.raises(CalibrationError):
        ArmCalibration.from_dict(data)


def test_with_base_yaw_moves_only_servo1():
    pose = [2.51, -2.73, 6.59, 2.15, -0.26, 14.56]
    out = with_base_yaw(pose, -8.0, 15.0)
    assert out[0] == pytest.approx(-5.49)
    assert out[1:] == pose[1:]
    assert pose[0] == 2.51          # 원본을 건드리지 않는다


def test_with_base_yaw_zero_is_identity():
    pose = [2.51, -2.73, 6.59, 2.15, -0.26, 14.56]
    assert with_base_yaw(pose, 0.0, 15.0) == pose


def test_with_base_yaw_rejects_over_limit():
    # 한계를 넘는 회전은 거부한다 — carry -> drop 관절 직선이 어디를 지나는지 모르게 된다
    with pytest.raises(ValueError, match="한계"):
        with_base_yaw([0.0] * 6, 20.0, 15.0)
    with pytest.raises(ValueError, match="한계"):
        with_base_yaw([0.0] * 6, -20.0, 15.0)


def test_with_base_yaw_rejects_nonfinite_and_short():
    with pytest.raises(ValueError):
        with_base_yaw([0.0] * 6, float("nan"), 15.0)
    with pytest.raises(ValueError):
        with_base_yaw([0.0] * 5, 1.0, 15.0)
