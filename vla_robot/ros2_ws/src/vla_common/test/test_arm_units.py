import json
from pathlib import Path

import pytest

from vla_common.arm_units import ArmCalibration, CalibrationError

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
