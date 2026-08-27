"""주행 중 근접 정지 게이트 — 정면 카메라 bbox 면적으로 "너무 가까운가"만 판정한다.

Host PC가 오버헤드 웹캠으로 계산해 준 경로는 이미 물체를 회피하지만, 그
경로는 오버헤드 관측과 주행 오차만큼 어긋날 수 있다. 이 모듈은 그 보완재로,
차량 자기 카메라로 정면을 보며 "지금 뭔가 위험할 만큼 가까운가"를 판정한다
(사용자 지시, 2026-08-25).

## 왜 depth 스트림을 안 쓰는가

depth 카메라(구조광 IR)는 체스말·축구공처럼 작고 광택 있는 물체를 거리와
무관하게 거의 못 본다 — 41cm/75cm 양쪽 실측에서 프레임 전체를 훑어도 물체의
진짜 거리값이 어디에도 없었다(perception_node.py "RGB bbox 면적 기반 거리
추정" 경고 참고). 피해야 할 대상이 정확히 그 센서가 못 보는 부류이므로,
이미 실기 검증된 RGB bbox 면적 방식을 그대로 쓴다.

## 왜 클래스를 안 보는가

거리 추정식 `distance = K_class / (sqrt(area) - PADDING)`의 K는 클래스마다
따로 실측해야 하는데 `box`·`star`는 아직 미실측이고, 파지 거리대에서는
큐브가 `rook`으로 오분류되기까지 한다(2026-08-25 실측).

그런데 **안전 정지에는 그게 무엇인지 알 필요가 없다** — "가까운가"만 알면
된다. 그래서 클래스를 묻지 않고, 실측된 K 중 **가장 작은 값**을 써서
"이 면적이면 아무리 멀어야 이 거리"라는 하한을 계산한다. K가 작을수록 같은
면적에서 더 가까운 거리로 해석되므로, 최소 K는 항상 가장 이른 정지를 낸다.
미실측 클래스가 섞여 있어도 이 하한은 무너지지 않는다.

## 왜 프레임 안이면 전부 위험으로 보는가

카메라 수평 화각은 fx=588.98, 폭 640px에서 약 57도다. 정지 거리 0.25m에서
프레임이 덮는 실제 폭은 약 0.27m인데, 차량 몸체 폭은 반경 0.16m 기준 0.32m다.
**프레임이 차체보다 좁다** — 즉 화면에 잡힌 것은 좌우 어디에 있든 차체 진로
안에 있다고 봐야 한다. 좌/중/우 구간별 거리는 참고용으로 같이 내지만, 위험
판정은 구간을 가리지 않는다.

## 근거리 사각지대

카메라가 높이 8.8cm에 하향 12.75도라 프레임 하단이 보는 바닥이 12.8cm다
(2026-08-25 실측). **그보다 가까운 물체는 프레임 밖으로 사라진다** — 즉
"안 보인다"가 "없다"를 뜻하지 않는다. 직전에 가까웠던 물체가 사라지면
사각지대로 들어갔을 가능성이 있으므로 위험으로 유지한다(`previously_close`).

정지 임계는 반드시 이 사각지대보다 넉넉히 앞에 두어야 한다 — 사각지대에
닿고 나서 멈추면 이미 늦다.

rclpy·카메라 없이 순수 계산만 담아 pytest로 검증한다
(grippers_arena/aruco_localization.py와 같은 이유)."""

import math
from typing import NamedTuple

# perception_node.CLASS_DISTANCE_CALIBRATION_SQRT_PX_M의 실측값 중 최소.
# 새 클래스를 실측해 이보다 작은 K가 나오면 이 값을 반드시 낮춰야 한다 —
# 안 그러면 그 클래스에 대해 게이트가 늦게 걸린다.
#
# 2026-08-27: box(cube)를 처음 실측(--mode k)하니 23.2733으로 soccer의
# 25.8794보다 작았다 — 최소값 자리가 soccer에서 box로 넘어갔다. (soccer도
# 같은 날 --mode scale 재보정으로 18.9592 -> 25.8794로 26.7% 올랐었다.)
MIN_MEASURED_K_SQRT_PX_M = 23.2733

# perception_node.BBOX_PADDING_PX와 같은 값. 검출 bbox가 물체 실루엣보다
# 항상 일정 픽셀만큼 크게 잡히는 검출기 성질이라 클래스와 무관하다.
BBOX_PADDING_PX = 2.5

# 이 면적 미만은 너무 멀거나 오검출로 보고 무시한다
# (perception_node.MIN_BBOX_AREA_PX와 같은 관례).
MIN_BBOX_AREA_PX = 25.0

# 근거리 사각지대 — 이보다 가까우면 물체가 프레임 밖이다(2026-08-25 실측).
NEAR_FIELD_BLIND_M = 0.128

# 기본 정지 거리. 사각지대(0.128m)보다 두 배 가까이 앞이고, GRASP 전제
# 배치(차체 앞 0.19m)보다도 앞이라 파지 진입을 방해하지 않는다.
# ⚠️ 실기 조정 대상 — CPU YOLO 추론 지연을 재서 (지연 x 주행속도)만큼
# 여유를 더해야 한다. 0.06 m/s에서 1초 지연이면 6cm다.
DEFAULT_STOP_DISTANCE_M = 0.25


class Detection(NamedTuple):
    """정면 카메라 한 프레임의 검출 하나. 클래스는 일부러 받지 않는다."""

    area_px: float
    center_x_px: float


class Verdict(NamedTuple):
    contact_risk: bool
    front_m: float
    left_m: float
    right_m: float
    reason: str


def area_threshold_px(
    stop_distance_m: float = DEFAULT_STOP_DISTANCE_M,
    k_min: float = MIN_MEASURED_K_SQRT_PX_M,
    padding_px: float = BBOX_PADDING_PX,
) -> float:
    """정지 거리에 대응하는 bbox 면적 임계값.

    `distance = k / (sqrt(area) - padding)`을 area에 대해 푼 것이다.
    면적이 이 값을 넘으면 물체가 정지 거리보다 가까울 수 있다."""
    if stop_distance_m <= 0.0:
        raise ValueError("stop_distance_m은 양수여야 한다")
    return (k_min / stop_distance_m + padding_px) ** 2


def lower_bound_distance_m(
    area_px: float,
    k_min: float = MIN_MEASURED_K_SQRT_PX_M,
    padding_px: float = BBOX_PADDING_PX,
) -> float | None:
    """이 면적이 나올 수 있는 **가장 가까운** 거리(m). 모르면 None.

    최소 K를 쓰므로 실제 거리는 항상 이 값 이상이다 — 즉 이 값으로 판정하면
    실제보다 이르게 멈출 수는 있어도 늦게 멈추지는 않는다."""
    if area_px < MIN_BBOX_AREA_PX:
        return None
    denom = math.sqrt(area_px) - padding_px
    if denom <= 0.0:
        return None
    return k_min / denom


def evaluate(
    detections: list | None,
    frame_width_px: float,
    stop_distance_m: float = DEFAULT_STOP_DISTANCE_M,
    previously_close: bool = False,
    k_min: float = MIN_MEASURED_K_SQRT_PX_M,
) -> Verdict:
    """한 프레임을 보고 정지해야 하는지 판정한다.

    `detections`가 None이면 **관측 자체가 없다**는 뜻이다(카메라 끊김·서비스
    타임아웃 등) — "모르면 멈춘다"에 따라 위험으로 낸다. 빈 리스트는 "봤는데
    아무것도 없다"라서 다르다.

    `previously_close`는 직전 판정이 위험이었는지다. 근거리 사각지대 때문에
    "가까웠다가 안 보인다"는 "치웠다"와 "발밑으로 들어왔다"를 구분할 수 없어
    위험을 유지한다."""
    if detections is None:
        return Verdict(True, math.inf, math.inf, math.inf, "관측 없음 — 모르면 멈춘다")

    third = frame_width_px / 3.0
    nearest = {"front": math.inf, "left": math.inf, "right": math.inf}
    for det in detections:
        dist = lower_bound_distance_m(det.area_px, k_min=k_min)
        if dist is None:
            continue
        if det.center_x_px < third:
            sector = "left"
        elif det.center_x_px < 2.0 * third:
            sector = "front"
        else:
            sector = "right"
        nearest[sector] = min(nearest[sector], dist)

    closest = min(nearest.values())
    if closest <= stop_distance_m:
        return Verdict(
            True, nearest["front"], nearest["left"], nearest["right"],
            f"{closest:.3f}m — 정지 거리 {stop_distance_m:.3f}m 이내",
        )
    if previously_close:
        return Verdict(
            True, nearest["front"], nearest["left"], nearest["right"],
            f"직전에 가까웠던 물체가 안 보인다 — 근거리 사각지대"
            f"({NEAR_FIELD_BLIND_M:.3f}m) 가능성",
        )
    return Verdict(
        False, nearest["front"], nearest["left"], nearest["right"], "여유 있음"
    )
