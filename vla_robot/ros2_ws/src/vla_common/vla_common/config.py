"""설정 파일 로더. **모르는 키가 있으면 기동을 거부한다.**

기존 코드에서 반복된 사고: 런치 인자/파라미터 이름을 틀리게 주면 ROS 가 오류 없이
무시해서, 껐다고 믿은 기능이 켜진 채로 돌았다(use_depth_gate, auto_align_on_first_move).
여기서는 설정을 YAML 한 파일에 모으고, 오타는 즉시 예외로 만든다.
"""
from __future__ import annotations

import math
import typing
from dataclasses import dataclass, field, fields, is_dataclass, replace
from pathlib import Path

import yaml


class ConfigError(ValueError):
    pass


def _coerce(tp, value, key: str):
    origin = typing.get_origin(tp)
    if tp is bool:
        if not isinstance(value, bool):
            raise ConfigError(f"{key}: true/false 여야 한다 ({value!r})")
        return value
    if tp is int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigError(f"{key}: 정수여야 한다 ({value!r})")
        return value
    if tp is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ConfigError(f"{key}: 수치여야 한다 ({value!r})")
        if not math.isfinite(float(value)):
            raise ConfigError(f"{key}: 유한한 수치여야 한다 ({value!r})")
        return float(value)
    if tp is str:
        if not isinstance(value, str):
            raise ConfigError(f"{key}: 문자열이어야 한다 ({value!r})")
        return value
    if origin is tuple:
        args = typing.get_args(tp)
        if not isinstance(value, (list, tuple)):
            raise ConfigError(f"{key}: 목록이어야 한다 ({value!r})")
        if len(args) == 2 and args[1] is Ellipsis:
            return tuple(_coerce(args[0], v, f"{key}[{i}]") for i, v in enumerate(value))
        if len(args) != len(value):
            raise ConfigError(f"{key}: 항목 {len(args)}개가 필요하다 ({len(value)}개)")
        return tuple(_coerce(a, v, f"{key}[{i}]") for i, (a, v) in enumerate(zip(args, value)))
    if origin is dict:
        k_tp, v_tp = typing.get_args(tp)
        if not isinstance(value, dict):
            raise ConfigError(f"{key}: 매핑이어야 한다 ({value!r})")
        return {_coerce(k_tp, k, key): _coerce(v_tp, v, f"{key}.{k}") for k, v in value.items()}
    if is_dataclass(tp):
        return build(tp, value, key)
    raise ConfigError(f"{key}: 지원하지 않는 타입 {tp!r}")


def build(cls, data, where: str = ""):
    """dict -> dataclass. 모르는 키는 ConfigError, 빠진 키는 기본값."""
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise ConfigError(f"{where or cls.__name__}: 매핑이어야 한다 ({data!r})")
    hints = typing.get_type_hints(cls)
    names = {f.name for f in fields(cls) if f.init}
    unknown = sorted(set(data) - names)
    if unknown:
        raise ConfigError(f"{where or cls.__name__}: 모르는 키 {unknown} — 오타가 아닌지 확인")
    kwargs = {}
    for f in fields(cls):
        if not f.init or f.name not in data:
            continue
        key = f"{where}.{f.name}" if where else f.name
        kwargs[f.name] = _coerce(hints[f.name], data[f.name], key)
    return cls(**kwargs)


def load_yaml(path: str | Path) -> dict:
    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigError(f"설정 파일을 읽지 못했다({path}): {exc}") from exc
    return data or {}


# ---------------------------------------------------------------------------
# Pi (robot.yaml)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class ArmConfig:
    port: str = "/dev/soarm"
    baudrate: int = 1_000_000
    calibration_file: str = "arm_calibration.json"
    poses_file: str = "arm_poses.yaml"
    # 기동 시 서보 Homing_Offset 이 캘리브레이션 파일과 같은지 확인한다. 끄지 말 것.
    verify_homing_offsets: bool = True
    torque_on_start: bool = True
    # 3S LiPo. 이보다 낮으면 이동 명령을 거부한다(데드존·탈조 방지).
    min_voltage_v: float = 10.5
    # 서보 온도 상한(°C). 이 이상이면 이동을 거부한다.
    # 그리퍼는 닫힘 끝단을 토크로 계속 누르면 달궈진다(2026-09-22: 35 -> 40°C).
    # STS3215 자체 보호는 70°C 근처라, 그 앞에서 **우리가 먼저 멈추는** 값이다.
    max_servo_temp_c: float = 60.0
    # 청크 재생 시 스텝당 이동 상한(도). lerobot max_relative_target 과 같은 역할.
    max_step_deg: float = 5.0
    max_chunk_steps: int = 500
    # 청크 재생 가속도(raw). 0 = 서보 기본. 기존 실기값 30.
    chunk_acceleration: int = 30
    # 이름 포즈 이동 속도(도/초, 가장 많이 움직이는 관절 기준).
    pose_speed_deg_s: float = 45.0
    pose_rate_hz: float = 30.0
    gripper_speed_pct_s: float = 80.0
    read_retries: int = 3
    state_rate_hz: float = 10.0


@dataclass(frozen=True)
class BaseConfig:
    # 벤더 odom_publisher 가 구독하는 두 토픽 중 속도를 자르지 않는 쪽. 한계는 여기서 집행한다.
    cmd_vel_topic: str = "controller/cmd_vel"
    # 값은 안 보고 도착 시각만 본다 — 끊기면 구동계 노드가 멈춘 것이다.
    feedback_topic: str = "odom_raw"
    feedback_timeout_s: float = 1.0
    max_linear_mps: float = 0.15
    max_angular_rad_s: float = 0.5
    reject_mixed_rotation: bool = True
    # 이 시간 동안 Host 명령이 없으면 정지한다.
    watchdog_s: float = 0.5
    publish_hz: float = 20.0


@dataclass(frozen=True)
class LinkConfig:
    bind_ip: str = "0.0.0.0"
    command_port: int = 5005
    status_port: int = 5006
    status_hz: float = 10.0
    # 비우면 "마지막으로 유효한 명령을 보낸 주소"로 보고한다(권장).
    fixed_host_ip: str = ""


@dataclass(frozen=True)
class GripperCamConfig:
    device: str = "/dev/gripper_cam"
    width: int = 1280
    height: int = 720
    fps: int = 30
    fourcc: str = "MJPG"
    # 그리퍼캠이 거꾸로 달려 있다. 녹화(LeRobot rotation=180)와 같아야 한다.
    rotate_180: bool = True
    publish_hz: float = 10.0
    topic: str = "gripper_cam/image_raw"
    # 발행 크기. 0 이면 캡처 크기 그대로.
    #
    # ⚠️ 1280x720 BGR 한 장이 2.76 MB 다. 2026-09-22 실기: 15 Hz 로 발행하려 했더니 토픽이
    # 실제로 2.3 Hz 밖에 안 나왔고, 구독자가 받는 프레임이 1.5초까지 낡아 파지가 시작조차
    # 못 했다("그리퍼캠 프레임이 1.5s 낡았다"). BEST_EFFORT UDP 에서 큰 메시지는 수천 개로
    # 쪼개져 하나만 유실돼도 통째로 버려진다.
    #
    # 정책은 어차피 180x320 으로 줄여 쓰므로(train_config.json 의 resize) 발행 단계에서
    # 줄인다 — 172 KB 로 16배 작아진다. 캡처는 1280x720 그대로 두어 화질을 잃지 않는다.
    publish_width: int = 320
    publish_height: int = 180
    # 읽기가 이만큼 연속 실패하면 장치를 다시 연다.
    reopen_after_s: float = 2.0


@dataclass(frozen=True)
class PolicyConfig:
    source: str = "local"                       # local | remote
    checkpoint: str = "/shared/act_v5_all_180_120k_120000"
    device: str = "cpu"
    url: str = "http://192.168.0.2:8770"
    remote_timeout_s: float = 5.0
    # 정책에 넣을 색 순서. LeRobot OpenCVCamera 기본값이 RGB 라 녹화 데이터가 RGB 다.
    image_color: str = "rgb"                    # rgb | bgr
    # 정책 입력 해상도 [H, W]. [0, 0] 이면 train_config.json 의 resize 를 읽는다.
    image_size: tuple[int, int] = (0, 0)
    freeze_wrist_roll: bool = True
    wrist_roll_value: float = 0.067
    n_action_steps: int = 0                     # 0 = 체크포인트 값. 줄이지 말 것
    fps: float = 30.0
    timeout_s: float = 45.0
    # 한 번의 RunVlaGrasp 은 **한 번만 시도하고 끝낸다.** 재시도는 Host 몫이다 —
    # 같은 자리에서 다시 돌리면 관측이 거의 같아 같은 실패를 반복한다. 실패를 빨리
    # 확정해야 Host 가 다시 관측하고 차를 다시 세워 접근할 수 있다.
    #
    # 청크 하나 = n_action_steps/fps = 3.33초. 2026-09-22 실기로 확인한 것:
    #   정상 파지  완료 판정이 걸려 **5청크(18.4초)** 에 끝난다 (2회 모두)
    #   실패       정책이 **4청크짜리 시도를 반복한다** — 10 이면 2회+세 번째 도중,
    #              8 이면 정확히 2회를 다 쓴다(29.5초)
    # 학습 통계는 평균 5.1청크 · 최대 7.1청크였다.
    # 실제 중단은 **재시도 감지**(retry_dip_deg)가 맡는다 — 두 번째 시도가 시작되는 순간
    # 끝내므로, 느린 성공을 자르지 않으면서도 낭비를 막는다. 이 상한은 그래도 안 끝날 때를
    # 위한 최후 안전장치이므로 학습 최대치(7.1청크)를 덮는 8 로 둔다.
    #
    # ⚠️ 상한을 5~6 으로 조이면 **평균적인 성공이 잘린다.** 학습 118회차 평균이 506프레임
    # = 5.06청크이고 완료 판정은 청크 경계에서만 보므로, 평균적인 파지가 6청크를 쓴다.
    max_chunks: int = 8
    extended_lift_deg: float = -50.0
    returned_lift_deg: float = -95.0
    # 재시도 감지용. shoulder_lift 가 여기까지 내려왔다가(완전 복귀는 아님) **다시 뻗으면**
    # 정책이 두 번째 시도를 시작한 것이다 — 그 자리에서 끝낸다. 재시도는 Host 몫이다.
    #
    # 2026-09-22 실패 회차의 청크별 lift (물체를 못 잡은 경우):
    #   -102.9(idle) → 13.7 → 73.9 → 2.0 → **-66.4** → 34.9 → 75.5 → -12.0
    #                  \______ 1차 시도 ______/          \___ 2차 시도 ___/
    # 시도 사이의 저점이 -66 이고 성공한 복귀는 -103 이다. 그 사이면 되므로 -60 으로 잡는다
    # (처음 -80 으로 뒀더니 -66 까지만 내려와 안 걸렸다).
    retry_dip_deg: float = -60.0
    max_frame_age_s: float = 1.0
    min_frame_change: float = 0.5


@dataclass(frozen=True)
class GraspCheckConfig:
    enabled: bool = True
    # image | opening | none — 무엇으로 파지를 판정할지.
    #
    # ⚠️ 기본이 image 인 이유(2026-09-22 실측): TPU 턱에서는 개구율도 부하도 빈손과 겹친다.
    #     빈손 6.6~12.2% / 부하 0.13~0.18   ·   룩을 쥔 상태 10.9% / 0.17
    #   TPU 가 무르기 때문이다 — 물체에 닿은 뒤에도 패드가 눌리며 서보가 더 들어가고(위치가 안 멈춤),
    #   같은 변형에 힘이 천천히 올라 부하도 안 튄다. 물린 위치(턱 끝 vs 경첩 쪽 목)에 따라서도 달라진다.
    #   반면 그리퍼캠 근접 영역은 빈손끼리 0.0% vs 파지 30~33% 로 10배 갈린다.
    method: str = "image"
    # image: 파지 전후 근접 ROI 에서 달라진 픽셀 비율(%)이 이 값 이상이면 쥔 것으로 본다.
    image_changed_percent: float = 10.0
    # ROI (y0, y1, x0, x1), 프레임 크기 대비 비율. 화면 아래 중앙 = 턱 바로 앞.
    image_roi: tuple[float, float, float, float] = (0.42, 0.97, 0.33, 0.67)
    # 두 프레임의 픽셀이 "달라졌다"고 볼 채널 최대 차이(0..255).
    image_pixel_threshold: float = 30.0
    # opening 방식일 때만 쓴다. 정책 단위 0..100.
    min_gripper_percent: float = 20.0
    # 0 이면 부하 검사를 하지 않는다. TPU 에서는 분리가 안 되므로 기본 0.
    min_load_ratio: float = 0.0


@dataclass(frozen=True)
class PlaceConfig:
    carry_pose: str = "carry"
    drop_pose: str = "drop"
    # 바구니 앞 정차점에서 drop 포즈의 base(servo 1)를 이만큼(도) 더 튼다.
    # Host 는 상자 앞 0.15 m 지점에 차를 세우고 box_face_yaw_deg 로 방향만 맞춘다 —
    # 그 자리에서 팔이 바구니를 정면으로 보지 않는 나머지를 여기서 메운다.
    # 0 이면 drop 포즈 그대로 간다. 부호는 arm_poses.yaml 첫 번째 값과 같은 축이다.
    # 재는 법: tools/goto_pose.py --pose drop --base-yaw <도> --keep-gripper 로
    #          바구니 위에 설 때까지 몇 도씩 올려 본 값을 여기에 적는다.
    base_yaw_deg: float = 0.0
    # 허용 한계. 교시 자세에서 멀어질수록 carry -> drop 관절 직선 경로가 예측에서
    # 벗어난다. 기존 프로젝트도 ±15도에서 잘랐다(hardware arm_driver_node).
    max_base_yaw_deg: float = 15.0
    release_percent: float = 60.0
    settle_s: float = 0.5
    return_pose: str = "idle"


@dataclass(frozen=True)
class MissionConfig:
    cycle_hz: float = 20.0
    # 파지 시작 전에 팔을 둘 포즈(정책 학습 시작 자세).
    start_pose: str = "idle"
    grasp_timeout_s: float = 90.0
    place_timeout_s: float = 45.0
    # 작업을 시작하기 전에 팔이 idle/carry 중 하나에서 이 각도(도) 안에 있어야 한다.
    # 알 수 없는 자세에서 관절 공간 직선 보간을 하면 그리퍼가 바닥을 쓸 수 있다
    # (기존 실기 2026-08-24). 벗어나 있으면 작업을 거부하고 사람에게 넘긴다.
    known_pose_tolerance_deg: float = 20.0


@dataclass(frozen=True)
class RobotConfig:
    arm: ArmConfig = field(default_factory=ArmConfig)
    base: BaseConfig = field(default_factory=BaseConfig)
    link: LinkConfig = field(default_factory=LinkConfig)
    gripper_cam: GripperCamConfig = field(default_factory=GripperCamConfig)
    policy: PolicyConfig = field(default_factory=PolicyConfig)
    grasp_check: GraspCheckConfig = field(default_factory=GraspCheckConfig)
    place: PlaceConfig = field(default_factory=PlaceConfig)
    mission: MissionConfig = field(default_factory=MissionConfig)


def _resolve(base_dir: Path, value: str) -> str:
    p = Path(value).expanduser()
    return str(p if p.is_absolute() else (base_dir / p).resolve())


def load_robot_config(path: str | Path) -> RobotConfig:
    path = Path(path).resolve()
    cfg = build(RobotConfig, load_yaml(path), "robot")
    if cfg.policy.source not in ("local", "remote"):
        raise ConfigError(f"policy.source 는 local|remote: {cfg.policy.source!r}")
    if cfg.grasp_check.method not in ("image", "opening", "none"):
        raise ConfigError(f"grasp_check.method 는 image|opening|none: {cfg.grasp_check.method!r}")
    if cfg.policy.image_color not in ("rgb", "bgr"):
        raise ConfigError(f"policy.image_color 는 rgb|bgr: {cfg.policy.image_color!r}")
    if cfg.policy.max_chunks < 1:
        raise ConfigError("policy.max_chunks 는 1 이상이어야 한다")
    if cfg.place.max_base_yaw_deg <= 0:
        raise ConfigError("place.max_base_yaw_deg 는 양수여야 한다")
    if abs(cfg.place.base_yaw_deg) > cfg.place.max_base_yaw_deg:
        raise ConfigError(
            f"place.base_yaw_deg {cfg.place.base_yaw_deg:+.1f} 가 "
            f"한계 ±{cfg.place.max_base_yaw_deg:.1f} 를 넘는다")
    for name in ("max_linear_mps", "max_angular_rad_s", "watchdog_s"):
        if getattr(cfg.base, name) <= 0:
            raise ConfigError(f"base.{name} 는 양수여야 한다")
    base_dir = path.parent
    arm = replace(cfg.arm,
                  calibration_file=_resolve(base_dir, cfg.arm.calibration_file),
                  poses_file=_resolve(base_dir, cfg.arm.poses_file))
    return replace(cfg, arm=arm)


# ---------------------------------------------------------------------------
# 이름 포즈 (arm_poses.yaml)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class NamedPose:
    name: str
    values: tuple[float, float, float, float, float, float]
    measured: bool
    note: str = ""


def load_poses(path: str | Path) -> dict[str, NamedPose]:
    data = load_yaml(path)
    poses = data.get("poses")
    if not isinstance(poses, dict):
        raise ConfigError(f"{path}: 최상위에 poses 매핑이 필요하다")
    unknown_top = sorted(set(data) - {"poses", "units"})
    if unknown_top:
        raise ConfigError(f"{path}: 모르는 키 {unknown_top}")
    out = {}
    for name, entry in poses.items():
        if not isinstance(entry, dict):
            raise ConfigError(f"poses.{name}: 매핑이어야 한다")
        unknown = sorted(set(entry) - {"values", "measured", "note"})
        if unknown:
            raise ConfigError(f"poses.{name}: 모르는 키 {unknown}")
        values = _coerce(tuple[float, float, float, float, float, float],
                         entry.get("values"), f"poses.{name}.values")
        measured = _coerce(bool, entry.get("measured", False), f"poses.{name}.measured")
        note = _coerce(str, entry.get("note", ""), f"poses.{name}.note")
        out[str(name)] = NamedPose(str(name), values, measured, note)
    return out


def save_pose(path: str | Path, name: str, values, note: str = "") -> None:
    """teach 도구가 쓴다. 다른 포즈와 주석 없는 필드는 보존한다."""
    path = Path(path)
    data = load_yaml(path) if path.exists() else {}
    data.setdefault("units", "lerobot policy units: servo1-5 degrees, gripper 0-100")
    poses = data.setdefault("poses", {})
    poses[name] = {"values": [round(float(v), 2) for v in values],
                   "measured": True, "note": note}
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")
