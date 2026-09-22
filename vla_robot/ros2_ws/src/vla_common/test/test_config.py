from pathlib import Path

import pytest

from vla_common.config import ConfigError, load_poses, load_robot_config, save_pose

ROBOT_YAML = Path(__file__).resolve().parents[2] / "vla_robot_bringup" / "config" / "robot.yaml"


def test_repo_robot_yaml_is_valid():
    cfg = load_robot_config(ROBOT_YAML)
    assert Path(cfg.arm.calibration_file).is_file()
    poses = load_poses(cfg.arm.poses_file)
    assert poses["idle"].measured and len(poses["idle"].values) == 6
    assert not poses["drop"].measured   # 실측 전에는 PLACE 가 거부되어야 한다
    assert cfg.policy.image_size == (0, 0)
    assert cfg.base.cmd_vel_topic == "controller/cmd_vel"


def test_unknown_key_is_rejected(tmp_path):
    p = tmp_path / "robot.yaml"
    p.write_text("arm:\n  prot: /dev/ttyACM0\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="prot"):
        load_robot_config(p)


def test_wrong_type_is_rejected(tmp_path):
    p = tmp_path / "robot.yaml"
    p.write_text("base:\n  max_linear_mps: fast\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="max_linear_mps"):
        load_robot_config(p)


def test_bool_is_not_int(tmp_path):
    p = tmp_path / "robot.yaml"
    p.write_text("arm:\n  baudrate: true\n", encoding="utf-8")
    with pytest.raises(ConfigError):
        load_robot_config(p)


def test_grasp_check_defaults_to_image():
    cfg = load_robot_config(ROBOT_YAML)
    # TPU 턱에서 개구율·부하가 빈손과 겹쳤다 — 기본 판정은 영상이어야 한다
    assert cfg.grasp_check.method == "image"
    assert 0 < cfg.grasp_check.image_changed_percent < 30
    y0, y1, x0, x1 = cfg.grasp_check.image_roi
    assert 0 <= y0 < y1 <= 1 and 0 <= x0 < x1 <= 1


def test_invalid_grasp_check_method(tmp_path):
    p = tmp_path / "robot.yaml"
    p.write_text("grasp_check:" + chr(10) + "  method: vibes" + chr(10), encoding="utf-8")
    with pytest.raises(ConfigError):
        load_robot_config(p)


def test_invalid_policy_source(tmp_path):
    p = tmp_path / "robot.yaml"
    p.write_text("policy:\n  source: cloud\n", encoding="utf-8")
    with pytest.raises(ConfigError):
        load_robot_config(p)


def test_save_pose_roundtrip(tmp_path):
    p = tmp_path / "poses.yaml"
    save_pose(p, "drop", [1, 2, 3, 4, 5, 6], "상자 위")
    save_pose(p, "idle", [0, -100, 90, 70, 0, 7])
    poses = load_poses(p)
    assert poses["drop"].values == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0) and poses["drop"].measured
    assert set(poses) == {"drop", "idle"}
