"""SO-ARM101 servo 6의 실측 개구 폭(mm) ↔ goal count 변환."""

GRIPPER_CLOSED_MM = 9.0
GRIPPER_OPEN_MM = 168.0

# 2026-08-20, 핑거 안쪽 면 사이 거리. 링크 구조가 비선형이라 endpoint 두 점의
# 단일 선형 보간은 90 mm 요청에서 약 96 mm를 만들었다. 실측 중간점을 보존해
# 구간별 선형 보간한다.
GRIPPER_CALIBRATION_POINTS = (
    (9.0, 1150),
    (96.0, 1578),
    (168.0, 2000),
)


def position_from_width(width_mm: float) -> int:
    """요청 폭을 안전 범위로 clamp하고 piecewise-linear goal count로 바꾼다."""
    width = max(GRIPPER_CLOSED_MM, min(GRIPPER_OPEN_MM, float(width_mm)))

    for (width_lo, raw_lo), (width_hi, raw_hi) in zip(
        GRIPPER_CALIBRATION_POINTS,
        GRIPPER_CALIBRATION_POINTS[1:],
        strict=True,
    ):
        if width <= width_hi:
            fraction = (width - width_lo) / (width_hi - width_lo)
            return round(raw_lo + fraction * (raw_hi - raw_lo))

    return GRIPPER_CALIBRATION_POINTS[-1][1]
