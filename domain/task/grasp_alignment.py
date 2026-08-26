"""GRASP 진입 전 좌우·전후 정렬 판정 (사용자 지시, 2026-08-26).

## 왜 "정확히 가운데"를 요구하지 않는가

전진 속도가 충분히 느리고 그리퍼가 완전히 열려 있으면, 물체는 쓰러지지 않고
벌어진 턱 사이로 밀려 들어온다 — 평행 턱의 넓은 목이 좌우 자기정렬 효과를
낸다. 그래서 필요한 것은 "가운데"가 아니라 **턱이 쓸고 지나갈 영역 안에
있는가**다.

그 영역은 직사각형이다.

    가로 = 열린 그리퍼 최대 너비
    세로 = 미세 전진 거리

## 누가 고치는가 — 두 갈래

사용자 지시로 책임이 나뉜다.

    영역 **안**인데 가운데가 아니다  -> **Pi가 스스로 고친다** (servo 1)
    영역 **밖**이다                  -> Host에 수정 요구 (재회전 / 재직진)

Host가 차량 제어를 소유한다는 원칙의 예외다. 예외인 이유는 GRASP의
`creep_forward`가 예외인 이유와 같다 — "턱 사이에 들어올 것인가"는 오버헤드
카메라로 볼 수 없고, 그 판단에 필요한 관측은 Pi의 뎁스 카메라에만 있다.
영역 밖은 이야기가 다르다. 그건 차량이 잘못 선 것이고, 아레나 전체를 보는
쪽이 다시 세워야 한다.

## 왜 servo 1인가 (옆걸음이 아니라)

메카넘 옆걸음에는 속도 데드밴드가 있다 — 실제로 도는 최저 속도 0.06m/s에
최소 버스트 0.25초면 한 번에 **15mm**가 움직인다. 고치려는 오차가 그보다
작은 경우가 대부분이라 옆걸음으로는 오히려 어긋난다. servo 1은 각도로
움직이므로 데드밴드가 없고, 팔 길이 기준 1도가 약 3mm라 훨씬 곱다.

순수 계산만 담아 pytest로 검증한다 — 포트도 ROS도 모른다."""

import math
from dataclasses import dataclass

from domain.task import baseline_constants as bc

# 판정 결과 종류.
READY = "READY"                    # 그대로 내려가도 된다
PI_CENTER = "PI_CENTER"            # Pi가 servo 1로 고친다
HOST_CORRECTION = "HOST_CORRECTION"  # Host가 다시 세워야 한다
UNKNOWN = "UNKNOWN"                # 관측이 부족해 판정 불가 — 진행하지 않는다


@dataclass(frozen=True)
class AlignmentVerdict:
    action: str
    lateral_error_m: float = 0.0
    servo1_offset_rad: float = 0.0
    reason: str = ""


def capture_half_width_m(object_width_mm: float, open_width_mm=None) -> float:
    """물체 중심이 턱 중심선에서 이만큼까지 벗어나도 들어온다.

    턱이 물체를 스치기만 하면 밀려 넘어지므로, 물체 **폭의 절반**만큼은
    양쪽에서 빼 둔다. 열린 폭이 물체보다 좁으면 애초에 들어올 수 없다(0)."""
    if open_width_mm is None:
        open_width_mm = bc.GRIPPER_OPEN_MM
    usable = open_width_mm - object_width_mm
    return max(0.0, usable / 2.0) / 1000.0


def jaw_line_m(label):
    """그 클래스의 턱 선(뎁스 판독값 기준). 아직 안 쟀으면 **None**.

    클래스마다 따로인 이유는 `baseline_constants.JAW_LINE_DEPTH_FORWARD_M`
    주석 참고 — 요약하면 클래스별 거리 보정 K의 배율 오차가 커서, 같은
    클래스로 잰 턱 선을 빼야 그 오차가 상쇄된다."""
    return bc.JAW_LINE_DEPTH_FORWARD_M.get(label)


def capture_depth_range_m(label, creep_mm: float = bc.GRASP_CREEP_FORWARD_MM):
    """물체 중심이 이 전방 구간 안에 있어야 턱이 쓸고 지나간다.

    **뎁스 카메라가 읽는 값의 단위로** 돌려준다 — 비교 상대가 그 값이기
    때문이다. 그 클래스의 턱 선을 아직 안 쟀으면 **None**.

    턱 선 **앞쪽**에 있어야 한다 — 이미 턱 선보다 가까우면 전진해도
    안 들어오고 밀려날 뿐이다. 뒤 끝은 전진 거리가 정한다."""
    jaw = jaw_line_m(label)
    if jaw is None:
        return None
    return jaw, jaw + creep_mm / 1000.0


def servo1_offset_for(lateral_error_m: float, reach_mm=None):
    """좌우 오차를 지우는 servo 1 회전각(rad). 팔 길이를 모르면 **None**.

    턱은 servo 1 축을 중심으로 호를 그리므로 `atan2(오차, 팔 길이)`다.
    작은 각도라 호와 직선의 차이는 무시할 수 있다.

    기본값을 인자 자리에 두지 않는 이유: 파이썬은 기본값을 import 시점에
    한 번만 계산하므로, 상수를 나중에 채워도 반영되지 않는다."""
    if reach_mm is None:
        reach_mm = bc.SERVO1_AXIS_TO_JAW_MM
    if reach_mm is None or reach_mm <= 0.0:
        return None
    return math.atan2(lateral_error_m, reach_mm / 1000.0)


def creep_distance_m(observation, max_creep_mm: float = bc.GRASP_CREEP_FORWARD_MM):
    """이번 파지에 실제로 필요한 미세 전진 거리. 모르면 **None**.

    고정 상수를 쓰지 않는 이유: 전진의 목적은 "물체를 턱 선까지 데려오는
    것"이라 필요한 거리가 매번 다르다. 물체가 이미 턱 선 20mm 앞에 있는데
    상수 100mm를 그대로 밀면 80mm를 더 밀어 물체를 턱 안쪽으로 처박거나
    넘어뜨린다 — 2026-08-26 통주행에서 조작자가 손으로 멈춘 값이 24mm였던
    것도 같은 이야기다.

    이제 뎁스 카메라가 전방 거리를 주므로 그 차이를 그대로 쓰면 된다.
    상한은 남겨 둔다 — 관측이 튀었을 때 크게 밀고 나가지 않게 하는
    안전장치다."""
    if observation is None or not observation.metric_ok:
        return None
    jaw = jaw_line_m(observation.label)
    if jaw is None:
        return None
    needed = observation.forward_m - jaw
    if needed <= 0.0:
        return None
    return min(needed, max_creep_mm / 1000.0)


def judge(observation, object_width_mm: float,
          centering_tolerance_m=None) -> AlignmentVerdict:
    """GRASP로 내려가도 되는지 판정한다.

    `observation`은 Pi 뎁스 카메라의 `TargetObservation`이다. `metric_ok`가
    아니면 **판정하지 않는다** — 물체가 어디 있는지 모르는 채로 팔을 바닥에
    내리는 것이 이 단계에서 가장 비싼 실수다."""
    if centering_tolerance_m is None:
        centering_tolerance_m = bc.GRASP_CENTERING_TOLERANCE_M
    if observation is None or not observation.metric_ok:
        return AlignmentVerdict(
            UNKNOWN, reason="뎁스 카메라가 물체 위치를 미터로 환산하지 못했다")

    depth_range = capture_depth_range_m(observation.label)
    if depth_range is None:
        return AlignmentVerdict(
            UNKNOWN,
            reason=f"'{observation.label}'의 턱 선(JAW_LINE_DEPTH_FORWARD_M) 미실측 "
                   "— 턱 쓸기 구간을 모른다")

    # 좌우 영점을 먼저 뺀다. 클래스별인 이유는 상수 주석 참고 — 겉보기
    # 좌우 값에 그 클래스의 거리 배율 오차가 실려 있어, 같은 클래스로 잰
    # 영점을 빼야 상쇄된다.
    zero = bc.DEPTH_LATERAL_TO_JAW_CENTER_M.get(observation.label)
    if zero is None:
        return AlignmentVerdict(
            UNKNOWN,
            reason=f"'{observation.label}'의 좌우 영점"
                   "(DEPTH_LATERAL_TO_JAW_CENTER_M) 미실측 — 중앙을 모른다")
    lateral = observation.lateral_m - zero
    forward = observation.forward_m
    near_m, far_m = depth_range
    half_width = capture_half_width_m(object_width_mm)

    if forward < near_m:
        return AlignmentVerdict(
            HOST_CORRECTION, lateral, 0.0,
            f"물체가 턱 선보다 가깝다 ({forward * 1000:.0f}mm < {near_m * 1000:.0f}mm) "
            "— 후진 필요")
    if forward > far_m:
        return AlignmentVerdict(
            HOST_CORRECTION, lateral, 0.0,
            f"물체가 전진 거리 밖이다 ({forward * 1000:.0f}mm > {far_m * 1000:.0f}mm) "
            "— 재직진 필요")
    if abs(lateral) > half_width:
        return AlignmentVerdict(
            HOST_CORRECTION, lateral, 0.0,
            f"물체가 턱 폭 밖이다 (좌우 {lateral * 1000:+.0f}mm, "
            f"한계 ±{half_width * 1000:.0f}mm) — 재회전 필요")

    if abs(lateral) <= centering_tolerance_m:
        return AlignmentVerdict(READY, lateral, 0.0, "영역 안 · 중앙")

    offset = servo1_offset_for(lateral)
    if offset is None:
        # 팔 길이 미실측 — 각도를 지어내면 엉뚱한 곳으로 턱을 돌린다.
        # 영역 안이라 그냥 내려가도 대체로 물리지만, 사용자 지시는 "가운데에
        # 있어야 한다"이므로 Host에 넘겨 다시 세우게 한다.
        return AlignmentVerdict(
            HOST_CORRECTION, lateral, 0.0,
            f"좌우 {lateral * 1000:+.0f}mm 치우침 — SERVO1_AXIS_TO_JAW_MM "
            "미실측이라 Pi가 못 고친다")
    return AlignmentVerdict(
        PI_CENTER, lateral, offset,
        f"좌우 {lateral * 1000:+.0f}mm 치우침 — servo 1 {math.degrees(offset):+.1f}도로 보정")
