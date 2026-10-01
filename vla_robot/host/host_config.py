"""Host(노트북) 설정 — `host/config/host.yaml` 을 엄격하게 읽는다.

기존 코드에는 실측값(aruco/config.py)과 튜닝값(mission_config.py)이 파이썬 상수로
흩어져 있었고, 팀원 파일로 덮어쓰면 한쪽만 바뀌는 일이 잦았다. 여기서는 YAML 한
파일에 모으고 `vla_common.config.build()` 로 읽는다 — **모르는 키는 기동 거부**다.

값의 근거는 host.yaml 주석에 짧게 남기고, 긴 사연은 README 에 둔다.
"""
from __future__ import annotations

import math

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
    robot_marker_height_m: float = 0.282   # 2026-09-30 줄자 28 cm · 교차법 281.7 mm
    floor_marker_size_m: float = 0.120
    # 종이 좌우 변을 장판 좌우 끝에 붙인 자리(배치도 REV.2, 2026-09-29).
    floor_markers: dict[int, tuple[float, float]] = field(default_factory=lambda: {
        1: (0.070, 0.525), 2: (1.910, 0.525), 3: (0.070, 1.305), 4: (1.910, 1.305)})
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
    # 작업 경계 = 장판 가장자리. 원점이 장판 앞·왼 모서리다(2026-09-29 재실측 1.980 × 1.830 m).
    wall_x: tuple[float, float] = (0.0, 1.980)
    wall_y: tuple[float, float] = (0.0, 1.830)
    # 기물이 놓이는 영역 = 바닥 마커 네 장 중심 안쪽. 밖(상자 안 포함)은 대상이 아니다.
    workspace_x: tuple[float, float] = (0.070, 1.910)
    workspace_y: tuple[float, float] = (0.525, 1.305)
    # 상자 폭(x) x 길이(y) x 높이(z)
    box_size: tuple[float, float, float] = (0.210, 0.350, 0.220)
    # 상자 중심 (x, y, yaw_deg). 하나뿐이다 — 뒤쪽 긴 변 가운데, 뒷면이 장판 뒤끝.
    boxes: dict[str, tuple[float, float, float]] = field(default_factory=lambda: {
        "basket": (0.990, 1.655, 180.0)})


@dataclass(frozen=True)
class CameraConfig:
    indices: tuple[int, ...] = (0, 1)
    width: int = 1280
    height: int = 720
    # 캘리브레이션 파일이 없을 때만 쓰는 근사 화각(C920).
    hfov_deg: float = 70.4
    calib_dir: str = "calib"
    # C920 수동 초점, 카메라별 {인덱스: 값}(0 = 먼 곳). 오토포커스를 끄기만 하면 초점이
    # 멈춘 자리에 남는다. 팀이 쓰던 값 {0: 5, 1: 0}. 표에 없거나 음수면 오토포커스만 끈다.
    focus: dict[int, int] = field(default_factory=lambda: {0: 5, 1: 0})


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
    # 기물 회피용 차체 모양(2026-09-30 실측 약 20 x 25 cm). 직진은 반폭, 꺾는 점(제자리 회전)은
    # 대각선 반지름으로 본다 — planning/planner.py 의 safe / turn_safe.
    robot_width_m: float = 0.20
    robot_length_m: float = 0.25
    piece_obstacle_radius_m: float = 0.02     # 기물 40 mm
    obstacle_margin_m: float = 0.02           # 2026-09-30 저녁 0.03 -> 0.02
    # 마커 중심이 설 수 있는 y 범위 = 장판 거의 전체. 앞뒤 끝은 카메라가 로봇 마커를 못 본다.
    drive_area_y: tuple[float, float] = (0.25, 1.575)
    # 상자 둘레 진입 금지. 좌우·뒤는 로봇 반경, 앞은 정차점까지.
    # 주행 구역 상한이 상자를 막아 주던 것을 주행 구역을 넓히면서 이것으로 바꿨다.
    box_keepout_side_m: float = 0.20
    box_keepout_front_m: float = 0.22
    waypoint_step_m: float = 0.15
    axis_leg_tolerance_m: float = 0.03
    yaw_tolerance_deg: float = 5.0
    yaw_enter_deg: float = 12.0
    # 앞길(지금 방향으로 부분목표까지)에 기물·금지 구역·벽이 없으면 이만큼 틀어질 때까지는 멈춰
    # 돌지 않고 계속 직진한다. 부분목표가 no_turn_near_m 안이면(지나치는 중) 뒤로 가지 않는 한
    # 돌지 않는다. 2026-09-30 저녁: 주변이 비었는데도 직진 -> 좌/우 회전을 되풀이해 오래 걸렸다.
    yaw_enter_clear_deg: float = 25.0
    no_turn_near_m: float = 0.15
    min_heading_dist_m: float = 0.05
    obstacle_hold_cycles: int = 8
    obstacle_match_m: float = 0.06
    # CARRY 중 로봇 근처의 검출은 들고 있는 물체일 가능성이 높아 장애물에서 뺀다.
    carry_ignore_radius_m: float = 0.30


@dataclass(frozen=True)
class DriveConfig:
    linear_mps: float = 0.15
    # 제자리 회전 최고 속도. 오차가 rotation_slow_deg 아래면 비례로 줄이되 rotation_min_rad_s
    # 아래로는 안 내린다(데드밴드 아래면 안 돈다).
    # 2026-09-30 실기: 0.5 고정으로는 지연 ~0.35 s 동안 9~11° 더 돌아 ±5° 허용치를 번갈아
    # 넘으며 25초 동안 좌우로 떨었다.
    rotation_rad_s: float = 0.5
    rotation_min_rad_s: float = 0.25
    rotation_slow_deg: float = 30.0
    nudge_mps: float = 0.15
    # 파지·투입 직전, 마지막 움직임이 제자리 회전이면 반대로 이 속도로 이 시간만큼 돈다(0 이면 끔).
    # 2026-10-01: 회전 뒤 멈춰 있으면 바퀴가 크게 울었고, 반대로 0.3 s 돌리자 멎었다(0.3·0.6 s 시험).
    unwind_rad_s: float = 0.3
    unwind_s: float = 0.3
    # 회전 뒤 직진·후진이 이만큼 **이어졌을 때만** 버팀이 풀렸다고 본다(짧은 거리 맞추기로는 안 풀렸다).
    unwind_clear_s: float = 1.0


@dataclass(frozen=True)
class MissionConfig:
    cycle_hz: float = 10.0
    # 기준점은 ArUco 마커 중심이다(그리퍼는 마커보다 0.15 m 앞).
    # 파지는 **거리 범위** [grasp_dist_min_m, grasp_dist_max_m] 안에서만 시작한다.
    # 2026-09-30 실기: 0.27~0.31 m 에서 시작한 6번 모두 성공, 0.33 m 이상 3번·0.25 m 3번 모두 빈손
    # (정면 오차는 모두 1~4°). 트리거 한 점(0.35 -> 0.30)으로는 멈추는 동안 ~5 cm 더 가서 0.25 에
    # 섰다 — 그래서 트리거는 "멈춰서 맞추기 시작"하는 거리이고, 범위 밖이면 천천히 앞뒤로 맞춘다.
    grasp_trigger_dist_m: float = 0.33
    grasp_dist_min_m: float = 0.26
    grasp_dist_max_m: float = 0.31
    # 기물별로 다른 범위 [min, max]. 없으면 위 값을 쓴다. 범위 상한이 트리거보다 크면 그 기물은
    # 그 상한에서 멈춰 맞추기 시작한다.
    # star: 2026-10-01 실기 0.27 m 에서 두 번 다 그리퍼가 기물을 **지나쳤다**(사용자 관찰).
    #       09-30 에는 0.33~0.34 에서 두 번 빈손. 그 사이로 3 cm 물렸다.
    grasp_dist_by_label: dict[str, tuple[float, float]] = field(
        default_factory=lambda: {"star": (0.30, 0.33)})
    # 앞뒤로 맞출 때 멈추라고 하고도 더 가는 만큼을 미리 뺀다(속도 x 이 시간). 멈춘 뒤에는
    # grasp_settle_s 동안 기다렸다가 다시 잰다(흔들리는 위치로 판단하지 않게).
    grasp_creep_lead_s: float = 0.25
    grasp_settle_s: float = 0.5
    # 파지 구역에 들어온 뒤 이만큼 더 멀어져야 구역을 벗어난 것으로 본다. 제자리에서 기물을
    # 향해 도는 동안 마커가 몇 cm 흔들려 들락날락하지 않게.
    grasp_zone_hysteresis_m: float = 0.05
    # 파지 전에 기물을 정면으로 봐야 한다. 2026-09-30 실기: 거리만 보고 파지해 18° · 30° 어긋난
    # 채 시작했고 둘 다 실패했다(같은 날 7° 에서는 성공).
    grasp_face_tol_deg: float = 6.0
    # 파지에 실패하면 바로 다음 기물로 가지 않는다. 탑뷰로 위치를 다시 읽고, 정면을 다시
    # 맞춘 뒤 이 횟수만큼 더 잡아 본다. 그래도 안 되면 보류한다.
    grasp_retry_max: int = 1
    # 접근 중 목표 기물 위치를 탑뷰로 갱신할 때, 같은 라벨의 이 반경 안 검출을 같은 기물로 본다.
    target_track_m: float = 0.10
    place_trigger_dist_m: float = 0.35
    # 상자 앞면에서 정차점(마커)까지. planner.box_keepout_front_m 의 앞쪽 경계도 같은 지점이다.
    # 마커는 차체 중앙쯤이다(마커 -> 차체 앞 0.13 m, 2026-09-23 실측).
    #
    # 2026-09-30 저녁: 0.15 -> 0.22. 0.15 는 차체 앞이 상자에서 2 cm 라 정차점에서 돌다가 모서리로
    # 상자를 쳤고, 그걸 피하려고 넣은 진입점 경유·상자 앞 후진이 오히려 동작을 길게 만들었다.
    # 이제 정차점(허용치만큼 더 붙은 자리까지)에서 제자리 회전해도 차체 반대각(0.16)이 상자에
    # 닿지 않는다 — 로드 시 검사한다. 팔은 그만큼 덜 깊게 넣지만 테두리는 넘는다(_check_reach).
    box_approach_margin_m: float = 0.22
    # --- 바구니 투입 목표 (mission/basket_target.py, 도면 2026-09-05) ---
    # 상자 입구 안쪽의 투입 목표 사각형: 가로 ±3 cm · 안쪽 3 cm.
    insert_half_width_m: float = 0.03
    insert_inset_depth_m: float = 0.03
    # **idle 자세 마커**에서 투하 지점(열린 턱 아래)까지의 수평 거리.
    #
    # ⚠️ 마커가 차체가 아니라 **팔에 붙어 있다**(2026-09-23 확인: 팔을 펴면 마커가 같이
    # 앞으로 가고 그 자세에서는 탑뷰가 검출조차 못 한다). 그래서 기준은 반드시 주행 중
    # 자세인 **idle** 이다. 팔을 편 상태에서 잰 "마커 -> 턱 0.17~0.20" 은 이 값이 아니다.
    #
    # 2026-09-23 실측 합산: 마커(idle) -> 차체 앞 0.13 + 차체 앞 -> 턱 0.19 = 0.32 m.
    arm_reach_m: float = 0.32
    # 그 도달거리 대비 정차 **거리** 허용 오차. 좌우 오차는 팔이 메우지만(max_arm_yaw_deg)
    # 앞뒤 오차는 아무도 못 메운다 — 팔 길이는 고정이다. 그래서 따로, 좁게 잡는다.
    # 정차 판정은 **앞뒤로 비대칭**이다. 제약이 양쪽에서 다르기 때문이다.
    #   덜 붙는 쪽: 팔이 짧아지는 만큼 얕게 떨어진다. 도달거리 0.32 m 에 테두리까지
    #              0.22 m 이므로 4 cm 덜 붙어도 턱이 테두리를 6 cm 넘는다.
    #   더 붙는 쪽: 정차점에서 차체 앞과 상자 사이가 9 cm 다. 5 cm 더 붙어도 제자리 회전
    #              (반대각 0.16)이 상자에 닿지 않는다(0.22 - 0.05 = 0.17).
    place_arrive_tol_m: float = 0.04        # 덜 붙어도 되는 한도
    place_min_gap_m: float = 0.05           # 더 붙어도 되는 한도
    # 판정 목표 중심보다 이만큼 더 안쪽을 겨눈다. 정차 거리 오차가 그대로 앞뒤 오차가
    # 되므로 허용치보다 커야 한다 — 덜 붙어도 테두리 안쪽에 떨어지게 하는 여유다.
    place_aim_margin_m: float = 0.04
    # 팔의 base 로 메울 수 있는 좌우 각도 한계. Pi 의 place.max_base_yaw_deg 와 같은 값.
    # 이 밖이면 그때만 차체를 돌린다.
    max_arm_yaw_deg: float = 15.0
    # 그 한계를 넘어 차체를 돌릴 때는 남는 각도가 이 안이 될 때까지 돈다.
    place_turn_to_deg: float = 12.0
    # 상자 앞에서 앞으로 밀어 볼 수 있는 최대 거리. 여기까지 가도 정면에 못 서면
    # 다시 접근한다(place_tries 가 오른다).
    nudge_max_m: float = 0.40
    # 상자 앞 단계(NUDGE)가 이만큼 걸리면 거기서 맴도는 것이다 — 운반 단계로 돌아가 다시 접근한다.
    nudge_timeout_s: float = 25.0
    piece_dest_box: dict[str, str] = field(default_factory=lambda: {
        label: "basket" for label in ("queen", "knight", "rook", "star", "soccer", "box")})
    skip_radius_m: float = 0.10
    skip_expiry_s: float = 90.0
    place_retry_max: int = 2
    grasp_timeout_s: float = 120.0
    place_timeout_s: float = 60.0
    # 경로가 없어 이만큼 서 있으면 APPROACH 는 기물 보류, CARRY 는 HALTED.
    blocked_timeout_s: float = 10.0
    # 차체 무응답 자동 복구(2026-09-30). 움직임 명령을 base_stall_s 동안 보냈는데 탑뷰상
    # base_stall_move_m · base_stall_turn_deg 도 안 움직이면 Pi 에 컨트롤러 복구를 요청한다.
    # 복구한 뒤에도 연달아 base_recover_max 번 무응답이면 HALTED(그때는 전원·배선 문제).
    base_stall_s: float = 1.5
    base_stall_move_m: float = 0.01
    base_stall_turn_deg: float = 2.0
    base_recover_timeout_s: float = 45.0
    base_recover_max: int = 2
    # 차체 폭주 감지(2026-09-30). 같은 고장이 달리는 중에 나면 마지막 속도로 계속 간다 — 상자 앞에서
    # 회전만 보냈는데 1 m 를 달려 장판 밖으로 나갔다. 병진 명령을 멈춘 지 base_runaway_grace_s 가
    # 지났는데 base_runaway_window_s 동안 base_runaway_move_m 넘게 움직였으면 같은 복구(보드 리셋)를 요청한다.
    base_runaway_move_m: float = 0.05
    base_runaway_window_s: float = 0.5
    base_runaway_grace_s: float = 1.0


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
    pl = cfg.planner
    if pl.robot_width_m <= 0 or pl.robot_length_m <= 0:
        raise ConfigError("planner.robot_width_m / robot_length_m 는 양수여야 한다")
    if pl.box_keepout_side_m < 0 or pl.box_keepout_front_m < 0:
        raise ConfigError("planner.box_keepout_*_m 는 음수일 수 없다")
    if pl.box_keepout_front_m > cfg.mission.box_approach_margin_m:
        # 정차점이 금지 구역 안에 들어가 CARRY 가 영원히 "길 없음"이 된다.
        raise ConfigError("planner.box_keepout_front_m 는 mission.box_approach_margin_m 이하여야 한다")
    m = cfg.mission
    for name in ("insert_half_width_m", "insert_inset_depth_m", "place_arrive_tol_m",
                 "nudge_max_m", "arm_reach_m", "place_aim_margin_m", "place_min_gap_m"):
        if getattr(m, name) <= 0:
            raise ConfigError(f"mission.{name} 는 양수여야 한다")
    if not 0 < m.max_arm_yaw_deg <= 90:
        raise ConfigError("mission.max_arm_yaw_deg 는 0 초과 90 이하여야 한다")
    if not 0 < m.place_turn_to_deg <= m.max_arm_yaw_deg:
        raise ConfigError("mission.place_turn_to_deg 는 0 초과 max_arm_yaw_deg 이하여야 한다")
    half_diag = math.hypot(pl.robot_width_m / 2.0, pl.robot_length_m / 2.0)
    if m.box_approach_margin_m - m.place_min_gap_m < half_diag:
        # 정차점에서 제자리 회전하면 차체 모서리가 상자를 친다(2026-09-30 rook, 정차점 0.15).
        raise ConfigError(f"mission.box_approach_margin_m - place_min_gap_m 는 차체 반대각 "
                          f"{half_diag:.3f} m 이상이어야 한다 — 정차점에서 돌면 상자에 닿는다")
    if m.nudge_timeout_s <= 0:
        raise ConfigError("mission.nudge_timeout_s 는 양수여야 한다")
    if m.grasp_retry_max < 0 or m.grasp_face_tol_deg <= 0:
        raise ConfigError("mission.grasp_retry_max >= 0, grasp_face_tol_deg > 0")
    if not 0 < m.grasp_dist_min_m < m.grasp_dist_max_m <= m.grasp_trigger_dist_m:
        raise ConfigError("mission: 0 < grasp_dist_min_m < grasp_dist_max_m <= grasp_trigger_dist_m")
    for label, (lo, hi) in m.grasp_dist_by_label.items():
        if not 0 < lo < hi:
            raise ConfigError(f"mission.grasp_dist_by_label.{label}: 0 < min < max 여야 한다 ({lo}, {hi})")
    if m.grasp_creep_lead_s < 0 or m.grasp_settle_s < 0:
        raise ConfigError("mission.grasp_creep_lead_s / grasp_settle_s 는 음수일 수 없다")
    d = cfg.drive
    if not 0 < d.rotation_min_rad_s <= d.rotation_rad_s:
        raise ConfigError("drive.rotation_min_rad_s 는 0 초과 rotation_rad_s 이하여야 한다")
    return replace(
        cfg,
        cameras=replace(cfg.cameras, calib_dir=_resolve(cfg.cameras.calib_dir)),
        detector=replace(cfg.detector, deployment_dir=_resolve(cfg.detector.deployment_dir),
                         cache_dir=_resolve(cfg.detector.cache_dir)),
    )


__all__ = ["HostConfig", "load_host_config", "ConfigError", "HOST_ROOT", "ensure_vla_common"]
