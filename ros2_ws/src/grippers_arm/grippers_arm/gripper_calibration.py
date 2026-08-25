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


def width_from_position(raw_position: int) -> float:
    """position_from_width의 역함수 — 서보 6의 present position을 폭(mm)으로.

    검증 도구가 "명령한 폭"이 아니라 **실제로 도달한 폭**을 읽기 위해 쓴다.
    같은 구간별 보간표를 반대로 탄다. 보정 구간을 벗어난 raw는 양 끝
    폭으로 clamp한다 — 표 밖은 외삽할 근거가 없다.
    """
    raw = float(raw_position)
    raw_first = GRIPPER_CALIBRATION_POINTS[0][1]
    raw_last = GRIPPER_CALIBRATION_POINTS[-1][1]
    if raw <= raw_first:
        return GRIPPER_CLOSED_MM
    if raw >= raw_last:
        return GRIPPER_OPEN_MM

    for (width_lo, raw_lo), (width_hi, raw_hi) in zip(
        GRIPPER_CALIBRATION_POINTS,
        GRIPPER_CALIBRATION_POINTS[1:],
        strict=True,
    ):
        if raw <= raw_hi:
            fraction = (raw - raw_lo) / (raw_hi - raw_lo)
            return round(width_lo + fraction * (width_hi - width_lo), 1)

    return GRIPPER_OPEN_MM
