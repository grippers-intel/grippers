"""Host(노트북) 설정 — `host/config/host.yaml` 을 엄격하게 읽는다.

기존 코드에는 실측값(aruco/config.py)과 튜닝값(mission_config.py)이 파이썬 상수로
흩어져 있었고, 팀원 파일로 덮어쓰면 한쪽만 바뀌는 일이 잦았다. 여기서는 YAML 한
파일에 모으고 `vla_common.config.build()` 로 읽는다 — **모르는 키는 기동 거부**다.

값의 근거는 host.yaml 주석에 짧게 남기고, 긴 사연은 README 에 둔다.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass, field, replace
from pathlib import Path

HOST_ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = HOST_ROOT / "config" / "host.yaml"
VLA_COMMON_SRC = HOST_ROOT.parent / "ros2_ws" / "src" / "vla_common"


def configure_console() -> None:
    """Windows 한글 콘솔(cp949)은 '—', '⚠️' 를 인코딩하지 못해 print 에서 죽는다.
    인코딩은 그대로 두고 못 그리는 글자만 '?' 로 바꾼다. 진입점에서 한 번 부른다."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass


def ensure_vla_common() -> None:
    """`pip install -e ../ros2_ws/src/vla_common` 을 안 했어도 돌게 한다.

    계약 파일은 Pi 워크스페이스 안에 한 벌뿐이다. 복사본을 만들지 않고 경로만 얹는다."""
    try:
        import vla_common  # noqa: F401
    except ImportError:
        sys.path.insert(0, str(VLA_COMMON_SRC))


ensure_vla_common()

from vla_common.config import ConfigError, build, load_yaml  # noqa: E402

XY = tuple[float, float]


@dataclass(frozen=True)
class ArucoConfig:
    dictionary: str = "DICT_4X4_50"
    robot_marker_id: int = 0
    # 검은 테두리 바깥 변 길이(m). 흰 여백은 뺀다.
    robot_marker_size_m: float = 0.080
    # 바닥에서 로봇 마커 중심까지(m). 10 mm 틀리면 위치가 약 13 mm 밀린다.
    robot_marker_height_m: float = 0.270
    floor_marker_size_m: float = 0.120
    floor_markers: dict[int, tuple[float, float]] = field(default_factory=lambda: {
        1: (0.100, 0.400), 2: (1.700, 0.400), 3: (0.100, 1.400), 4: (1.700, 1.400)})
    # 마커 로컬 +x 와 로봇 전진 방향의 차(도). 2026-09-06 정지 실측 85.7.
    yaw_offset_deg: float = 85.7
    pose_hold_s: float = 1.0
    min_floor_markers: int = 2
    max_extrinsic_reproj_px: float = 3.0
    extrinsic_lock_frames: int = 30
    extrinsic_relock_px: float = 4.0
    extrinsic_relock_frames: int = 15


@dataclass(frozen=True)
class ArenaConfig:
    wall_x: tuple[float, float] = (0.0, 1.8)
    wall_y: tuple[float, float] = (0.0, 1.8)
    # 기물이 놓이는 영역. y 밖(상자 띠)은 이미 옮긴 것으로 본다.
    workspace_x: tuple[float, float] = (0.0, 1.8)
    workspace_y: tuple[float, float] = (0.4, 1.4)
    # 상자 폭(x) x 길이(y) x 높이(z)
    box_size: tuple[float, float, float] = (0.210, 0.350, 0.220)
    # 상자 중심 (x, y, yaw_deg)
    boxes: dict[str, tuple[float, float, float]] = field(default_factory=lambda: {
        "toy": (0.450, 1.625, 180.0), "chess": (1.350, 1.625, 180.0)})


@dataclass(frozen=True)
class CameraConfig:
    indices: tuple[int, ...] = (0, 1)
    width: int = 1280
    height: int = 720
    # 캘리브레이션 파일이 없을 때만 쓰는 근사 화각(C920).
    hfov_deg: float = 70.4
    calib_dir: str = "calib"


@dataclass(frozen=True)
class DetectorConfig:
    kind: str = "geti"                    # geti | none
    deployment_dir: str = "models/geti_deployment"
    device: str = "CPU"
    cache_dir: str = ".ov_cache"
    # 추론이 카메라당 ~0.8 s 라 쉬지 않고 돌리면 메인 루프가 1.7 Hz 로 떨어졌다.
    min_infer_interval_s: float = 0.3
    conf_threshold: float = 0.6
    empty_labels: tuple[str, ...] = ("No object", "Empty")


@dataclass(frozen=True)
class TrackerConfig:
    merge_dist_m: float = 0.08
    hold_s: float = 1.5
    confirm_s: float = 1.2
    label_decay: float = 0.85
    max_per_label: int = 2


@dataclass(frozen=True)
class PlannerConfig:
    cell_m: float = 0.025
    # 벽 여유는 암까지 포함한 반경, 기물 회피는 하단부 반경 — 둘을 뭉개면 하나는 틀린다.
    robot_radius_wall_m: float = 0.20
    robot_radius_piece_m: float = 0.14
    piece_obstacle_radius_m: float = 0.06
    obstacle_margin_m: float = 0.05
    # 마커 중심이 설 수 있는 y 범위. 0.30 = 두 카메라에 다 잡히는 한계 0.27 + 30 mm.
    drive_area_y: tuple[float, float] = (0.30, 1.30)
    waypoint_step_m: float = 0.15
    axis_leg_tolerance_m: float = 0.03
    yaw_tolerance_deg: float = 5.0
    yaw_enter_deg: float = 12.0
    min_heading_dist_m: float = 0.05
    obstacle_hold_cycles: int = 8
    obstacle_match_m: float = 0.06
    # CARRY 중 로봇 근처의 검출은 들고 있는 물체일 가능성이 높아 장애물에서 뺀다.
    carry_ignore_radius_m: float = 0.30


@dataclass(frozen=True)
class DriveConfig:
    linear_mps: float = 0.15
    rotation_rad_s: float = 0.5
    nudge_mps: float = 0.15


@dataclass(frozen=True)
class MissionConfig:
    cycle_hz: float = 10.0
    # 기준점은 ArUco 마커 중심이다(그리퍼는 마커보다 0.15 m 앞).
    grasp_trigger_dist_m: float = 0.35
    place_trigger_dist_m: float = 0.35
    box_approach_margin_m: float = 0.15
    # --- 바구니 투입 목표와 접근 판정 (mission/basket_target.py, 도면 2026-09-05) ---
    # 상자 입구 안쪽의 투입 목표 사각형: 가로 ±3 cm · 안쪽 3 cm.
    insert_half_width_m: float = 0.03
    insert_inset_depth_m: float = 0.03
    # 목표 중심 기준 반경. 이 원의 남쪽 부채꼴 바깥 호가 NUDGE 판정 경계선이다.
    max_approach_dist_m: float = 0.15
    # 접근을 인정하는 부채꼴 각도. 120° = 원을 3등분한 남쪽 한 조각.
    approach_sector_deg: float = 120.0
    # 투입을 시도해도 되는 지향 오차 한계(도). 정렬은 ±yaw_tolerance_deg 로 하므로
    # 평소에는 걸리지 않는다 — pose 가 튀었을 때 투입을 막는 마지막 문이다.
    max_facing_error_deg: float = 50.0
    # NUDGE 에서 앞으로 밀어 볼 수 있는 최대 거리. 여기까지 가도 호를 못 넘으면
    # 정렬이 틀린 것이다 — 다시 접근한다(place_tries 가 오른다).
    nudge_max_m: float = 0.40
    piece_dest_box: dict[str, str] = field(default_factory=lambda: {
        "queen": "chess", "knight": "chess", "rook": "chess",
        "star": "toy", "soccer": "toy", "box": "toy"})
    skip_radius_m: float = 0.10
    skip_expiry_s: float = 90.0
    place_retry_max: int = 2
    grasp_timeout_s: float = 120.0
    place_timeout_s: float = 60.0
    # 경로가 없어 이만큼 서 있으면 APPROACH 는 기물 보류, CARRY 는 HALTED.
    blocked_timeout_s: float = 10.0


@dataclass(frozen=True)
class LinkConfig:
    bind_ip: str = "0.0.0.0"
    command_port: int = 5005
    status_port: int = 5006
    stop_burst: int = 8


@dataclass(frozen=True)
class ViewConfig:
    px_per_m: float = 350.0
    panel_width_px: int = 380


@dataclass(frozen=True)
class HostConfig:
    aruco: ArucoConfig = field(default_factory=ArucoConfig)
    arena: ArenaConfig = field(default_factory=ArenaConfig)
    cameras: CameraConfig = field(default_factory=CameraConfig)
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    tracker: TrackerConfig = field(default_factory=TrackerConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    drive: DriveConfig = field(default_factory=DriveConfig)
    mission: MissionConfig = field(default_factory=MissionConfig)
    link: LinkConfig = field(default_factory=LinkConfig)
    view: ViewConfig = field(default_factory=ViewConfig)


def _resolve(value: str) -> str:
    p = Path(value).expanduser()
    return str(p if p.is_absolute() else (HOST_ROOT / p).resolve())


def load_host_config(path: str | Path | None = None) -> HostConfig:
    path = Path(path) if path else DEFAULT_CONFIG
    cfg = build(HostConfig, load_yaml(path), "host")
    if cfg.detector.kind not in ("geti", "none"):
        raise ConfigError(f"detector.kind 는 geti|none: {cfg.detector.kind!r}")
    for label, box in cfg.mission.piece_dest_box.items():
        if box not in cfg.arena.boxes:
            raise ConfigError(f"mission.piece_dest_box.{label}: 모르는 상자 {box!r}")
    if cfg.planner.cell_m >= cfg.planner.axis_leg_tolerance_m:
        raise ConfigError("planner.cell_m 는 axis_leg_tolerance_m 보다 작아야 한다")
    if cfg.aruco.min_floor_markers < 1:
        raise ConfigError("aruco.min_floor_markers >= 1")
    m = cfg.mission
    if not 0 < m.max_facing_error_deg <= 180:
        raise ConfigError("mission.max_facing_error_deg 는 0 초과 180 이하여야 한다")
    for name in ("insert_half_width_m", "insert_inset_depth_m", "max_approach_dist_m", "nudge_max_m"):
        if getattr(m, name) <= 0:
            raise ConfigError(f"mission.{name} 는 양수여야 한다")
    if not 0 < m.approach_sector_deg <= 360:
        raise ConfigError("mission.approach_sector_deg 는 0 초과 360 이하여야 한다")
    return replace(
        cfg,
        cameras=replace(cfg.cameras, calib_dir=_resolve(cfg.cameras.calib_dir)),
        detector=replace(cfg.detector, deployment_dir=_resolve(cfg.detector.deployment_dir),
                         cache_dir=_resolve(cfg.detector.cache_dir)),
    )


__all__ = ["HostConfig", "load_host_config", "ConfigError", "HOST_ROOT", "ensure_vla_common"]
