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
class HandConfig:
    """탑뷰 손 검출(MediaPipe HandLandmarker). 손은 **가져다줄 곳**이다 — 기물·장애물이 아니다.
    mediapipe 가 없으면 경고만 하고 손 없이 돈다(선택 의존성)."""
    enabled: bool = True
    model_path: str = "models/hand_landmarker.task"
    # full 프레임 ~30 ms/카메라. 손은 기다리는 대상이라 자주 볼 필요가 없다.
    min_infer_interval_s: float = 0.2
    min_conf: float = 0.4
    num_hands: int = 2
    # 두 카메라가 같이 보면 광선 교차로 높이까지 푼다. 광선 사이가 이보다 멀면 다른 손이다.
    pair_max_gap_m: float = 0.08
    max_z_m: float = 1.0
    # 한 카메라만 볼 때 손바닥 중심을 이 높이 평면으로 푼다. 받는 자세(장판 위 30~35 cm).
    hand_z_m: float = 0.32
    # 손을 인정하는 가장자리와 띠 폭. 장판 안쪽(작업 구역)에서 잡힌 것은 버린다.
    edges: tuple[str, ...] = ("front", "left", "right")
    edge_band_m: float = 0.50
    outside_m: float = 0.30               # 장판 밖으로 내민 손도 이만큼은 받는다
    merge_dist_m: float = 0.15
    confirm_s: float = 0.6                # 이만큼 계속 보여야 손으로 인정
    hold_s: float = 2.0                   # 잠깐 놓쳐도 지도에서 지우지 않는다(10-05)
    max_hands: int = 2


@dataclass(frozen=True)
class TrackerConfig:
    merge_dist_m: float = 0.08
    hold_s: float = 1.5
    confirm_s: float = 1.2
    label_decay: float = 0.85
    max_per_label: int = 2
    # 넓은 기물의 바닥 반경(m) — 카메라에 가까운 가장자리를 중심으로 옮긴다(2026-10-05, star).
    piece_radius_by_label: dict[str, float] = field(default_factory=dict)


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
    # 좁은 틈(2026-10-07): 지금 가는 직선이 기물의 제자리 회전 반경(turn_safe) 안을 지나면, 틈 밖에서
    # narrow_yaw_tolerance_deg 까지 맞춘 뒤 들어가고(문턱 narrow_entry_enter_deg), 틈 안에서는
    # narrow_hold_enter_deg 까지 틀어져도 돌지 않고 직진한다 — 틈 안에서 돌면 모서리가 기물을 쓴다.
    narrow_yaw_tolerance_deg: float = 2.5
    narrow_entry_enter_deg: float = 5.0
    narrow_entry_zone_m: float = 0.15      # 틈 기물의 회전 반경 밖 이 거리 안 = 입구 바로 앞(여기서만 직진 중 재정렬)
    narrow_hold_enter_deg: float = 20.0
    no_turn_near_m: float = 0.15
    min_heading_dist_m: float = 0.05
    obstacle_hold_cycles: int = 8
    obstacle_match_m: float = 0.06
    # CARRY 중 로봇 근처의 검출은 들고 있는 물체일 가능성이 높아 장애물에서 뺀다.
    carry_ignore_radius_m: float = 0.30


@dataclass(frozen=True)
class DriveConfig:
    # 제자리 회전이 옆 기물을 쓸 때(틈 안에서 잡은 직후 등) 앞이 비었으면 이만큼까지 앞으로 나간 뒤 돈다(10-07).
    turn_exit_max_m: float = 0.30
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
    # 많이 돌았으면 그만큼 길게: unwind_s 는 unwind_ref_rad(0.3 rad/s x 2 s = 0.6 rad, 10-01 시험)만큼 돈 뒤의 값.
    # 더 돌았으면 비례해서 늘리고 unwind_max_s 에서 자른다(10-05: 상자 앞 54° 회전 뒤 소음).
    unwind_ref_rad: float = 0.6
    unwind_max_s: float = 0.8
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
    # 파지 실패 뒤 다시 접근하기 전에 서서 기다린다(2026-10-05): 그리퍼가 공을 9.5 cm 밀었는데 0.1 s 만에
    # 옛 위치로 다시 잡아 또 실패했다. 지도가 새 위치를 잡을 시간(tracker hold 1.5 s · confirm 1.2 s).
    grasp_retry_settle_s: float = 2.5
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
    # 정차점에서 몸을 돌리면 바구니 옆 기물에 닿을 때, 팔(±max_arm_yaw_deg)로만 넣어도 되는 모자란 각의 상한.
    # 10° 면 팔 0.32 m 끝에서 ~5.6 cm 옆 — 바구니 폭 21 cm 안. 넘으면 다시 접근(그래도 안 되면 HALTED). 10-07
    place_arm_only_max_short_deg: float = 10.0
    # 바구니 정차 구역(2026-10-07): 정차점 한 점 대신, 앞뒤(y)는 그대로 두고 좌우(x)로 ±basket_stop_zone_half_m
    # 안에서 차체가 다른 기물에 닿지 않는 자리 중 가운데에 가장 가까운 곳에 선다. 겨누는 점도 그만큼(최대
    # basket_aim_shift_max_m) 옮겨 팔을 많이 틀지 않게 한다. 구역 어디에도 못 서면 들어가기 전에 HALTED.
    basket_stop_zone_half_m: float = 0.10
    basket_stop_turn_deg: float = 15.0       # 구역 자리 검사에서 바구니 정면 ± 이 각도까지 본다(팔이 메우는 만큼 틀어져 선다)
    basket_stop_pos_err_m: float = 0.02      # 구역 자리 검사에서 좌우로 이만큼 어긋나 서는 것까지 본다
    basket_stop_clear_m: float = 0.015       # 차체 바깥면–기물 가장자리 최소 간격
    basket_stop_zone_retry_s: float = 3.0    # 설 자리가 없으면 운반을 이으며 이만큼 다시 보고 그래도 없으면 HALTED
    # 바구니에는 아래에서 들어간다(2026-10-07): 기물을 정차점보다 basket_low_margin_m 넘게 아래에서 잡았으면
    # 정차점까지 방향이 정면(90°) ± basket_approach_cone_deg 안일 때만 곧장 가는 단계로 넘기고, 아니면 먼저
    # 정차점 바로 아래 basket_pre_stop_m 지점으로 간다. 정차점 높이 근처에서 잡았으면 구역에서 로봇 쪽 자리에 선다.
    basket_low_margin_m: float = 0.12
    basket_approach_cone_deg: float = 15.0
    basket_pre_stop_m: float = 0.30
    basket_stop_step_m: float = 0.02
    basket_aim_shift_max_m: float = 0.06
    # 정차점 근처(place_trigger_dist_m 안)에서 정차점까지 직선이 이만큼 계속 막혀 있으면 HALTED 로 사람을 부른다(10-07).
    carry_line_block_s: float = 5.0
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
    # 회전 폭주 감지(2026-10-05). 같은 고장이 회전 중에 나면 제자리에서 계속 돈다(반시계 ~8°/s, ESTOP 도
    # 안 먹었다). 위치가 거의 안 변해 위 감지로는 못 잡는다. 회전 명령 방향이 바뀐 지 base_spin_grace_s 뒤,
    # base_spin_window_s 동안 회전 명령 없이(또는 반대로) base_spin_turn_deg 넘게 돌면 같은 복구를 요청한다.
    base_spin_turn_deg: float = 6.0
    base_spin_window_s: float = 1.0
    base_spin_grace_s: float = 1.0


@dataclass(frozen=True)
class LinkConfig:
    bind_ip: str = "0.0.0.0"
    command_port: int = 5005
    status_port: int = 5006
    stop_burst: int = 8


@dataclass(frozen=True)
class HandoverConfig:
    """"가져와" 지시 — 기물을 사람 손에 건넨다. 손 위치 9곳마다 [손 x, 손 y, 정차 x, 정차 y](m).
    로봇은 정차점에 가서 손 쪽을 보고(남는 각도는 팔 base) Pi 에 PLACE(place_pose=handover)를 보낸다."""
    spots: dict[str, tuple[float, float, float, float]] = field(default_factory=lambda: {
        "F1": (0.33, 0.12, 0.33, 0.47), "F2": (0.99, 0.12, 0.99, 0.47), "F3": (1.65, 0.12, 1.65, 0.47),
        "L1": (0.075, 0.26, 0.43, 0.40), "L2": (0.075, 0.915, 0.43, 0.915), "L3": (0.075, 1.56, 0.43, 1.45),
        "R1": (1.905, 0.26, 1.55, 0.40), "R2": (1.905, 0.915, 1.55, 0.915), "R3": (1.905, 1.56, 1.55, 1.45)})
    match_radius_m: float = 0.40      # 보이는 손을 이 거리 안의 위치로 본다
    hand_wait_s: float = 20.0         # 정차점에서 손을 기다리는 시간. 넘으면 바구니로
    arrive_tol_m: float = 0.08        # 정차점 도착 판정


@dataclass(frozen=True)
class VoiceConfig:
    """노트북 내장 마이크 -> 한국어 문장(faster-whisper, 오프라인). 시연 UI 마이크 버튼."""
    enabled: bool = True
    model_dir: str = "models/whisper-small"
    language: str = "ko"
    threads: int = 8
    beam_size: int = 1                # 5 와 정확도 차이가 없고 0.2 s 빠르다(2026-10-01)
    # 낱말 힌트. "퀸"을 "균"으로 듣는 일을 줄인다.
    prompt: str = "퀸, 룩, 나이트, 별, 축구공, 공, 상자, 바구니, 체스 말, 가져와, 정리해, 넣어줘, 치워줘"
    calib_s: float = 0.3              # 처음 이만큼으로 주변 소음 측정
    speech_ratio: float = 3.0         # 소음의 몇 배면 말로 보나
    min_rms: float = 0.01
    silence_s: float = 0.9            # 말이 끝나고 이만큼 조용하면 멈춤
    no_speech_s: float = 5.0
    max_s: float = 8.0


@dataclass(frozen=True)
class InstructionConfig:
    """사람 지시(Claude API). 키는 ANTHROPIC_API_KEY 환경변수 — 설정 파일에 두지 않는다."""
    # auto = 보이는 기물을 모두 정리하고, 지시가 오면 그것부터 · instructed = 지시가 있을 때만 움직인다
    mode: str = "auto"
    model: str = "claude-opus-5-5"
    effort: str = "low"
    timeout_s: float = 20.0


@dataclass(frozen=True)
class ViewConfig:
    # web = 팀원 시연 UI(ui/grippers-ui.html, 브라우저 앱 창) · cv = 예전 OpenCV 지도
    kind: str = "web"
    web_port: int = 8765
    window_height: int = 0           # 시연 창 높이(배율 적용 뒤 px). 0 = 화면 작업 영역에 맞춤
    # 배터리 칸(%) 환산: [0 %, 100 %] 전압. 차체 2S(완충 8.4, 실측 8.44→8.06 한 판), 팔 서보 전원(하한 10.5 V).
    veh_v_range: tuple[float, float] = (6.8, 8.4)
    arm_v_range: tuple[float, float] = (10.5, 12.6)
    veh_low_v: float = 7.2           # 이 아래면 화면에 "충전" 알림
    px_per_m: float = 350.0          # cv 지도 전용
    panel_width_px: int = 380        # cv 지도 전용


@dataclass(frozen=True)
class HostConfig:
    aruco: ArucoConfig = field(default_factory=ArucoConfig)
    arena: ArenaConfig = field(default_factory=ArenaConfig)
    cameras: CameraConfig = field(default_factory=CameraConfig)
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    tracker: TrackerConfig = field(default_factory=TrackerConfig)
    hands: HandConfig = field(default_factory=HandConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    drive: DriveConfig = field(default_factory=DriveConfig)
    mission: MissionConfig = field(default_factory=MissionConfig)
    link: LinkConfig = field(default_factory=LinkConfig)
    instruction: InstructionConfig = field(default_factory=InstructionConfig)
    handover: HandoverConfig = field(default_factory=HandoverConfig)
    voice: VoiceConfig = field(default_factory=VoiceConfig)
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
    v = cfg.voice
    if v.threads < 1 or v.beam_size < 1 or min(v.calib_s, v.silence_s, v.no_speech_s, v.max_s) <= 0:
        raise ConfigError("voice: threads, beam_size >= 1 · 시간 값은 양수")
    ho = cfg.handover
    if not ho.spots or ho.match_radius_m <= 0 or ho.hand_wait_s < 0 or ho.arrive_tol_m <= 0:
        raise ConfigError("handover: spots 가 필요하고 match_radius_m · arrive_tol_m > 0, hand_wait_s >= 0")
    for name, v in ho.spots.items():
        if len(v) != 4:
            raise ConfigError(f"handover.spots.{name}: [손 x, 손 y, 정차 x, 정차 y] 네 값")
    ic = cfg.instruction
    if ic.mode not in ("auto", "instructed"):
        raise ConfigError(f"instruction.mode 는 auto|instructed: {ic.mode!r}")
    if ic.effort not in ("low", "medium", "high", "xhigh", "max") or ic.timeout_s <= 0:
        raise ConfigError("instruction.effort 는 low|medium|high|xhigh|max, timeout_s > 0")
    if cfg.view.kind not in ("web", "cv"):
        raise ConfigError(f"view.kind 는 web|cv: {cfg.view.kind!r}")
    h = cfg.hands
    bad = set(h.edges) - {"front", "back", "left", "right"}
    if bad:
        raise ConfigError(f"hands.edges 는 front|back|left|right: {sorted(bad)}")
    for name in ("min_infer_interval_s", "hand_z_m", "edge_band_m", "merge_dist_m", "hold_s",
                 "pair_max_gap_m", "max_z_m"):
        if getattr(h, name) <= 0:
            raise ConfigError(f"hands.{name} 는 양수여야 한다")
    if h.outside_m < 0 or h.confirm_s < 0 or h.num_hands < 1 or h.max_hands < 1:
        raise ConfigError("hands: outside_m, confirm_s >= 0 · num_hands, max_hands >= 1")
    d = cfg.drive
    if not 0 < d.rotation_min_rad_s <= d.rotation_rad_s:
        raise ConfigError("drive.rotation_min_rad_s 는 0 초과 rotation_rad_s 이하여야 한다")
    return replace(
        cfg,
        cameras=replace(cfg.cameras, calib_dir=_resolve(cfg.cameras.calib_dir)),
        detector=replace(cfg.detector, deployment_dir=_resolve(cfg.detector.deployment_dir),
                         cache_dir=_resolve(cfg.detector.cache_dir)),
        hands=replace(cfg.hands, model_path=_resolve(cfg.hands.model_path)),
        voice=replace(cfg.voice, model_dir=_resolve(cfg.voice.model_dir)),
    )


__all__ = ["HostConfig", "load_host_config", "ConfigError", "HOST_ROOT", "ensure_vla_common"]
