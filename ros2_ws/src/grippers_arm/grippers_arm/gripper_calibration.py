"""SO-ARM101 servo 6의 실측 개구 폭(mm) ↔ goal count 변환."""

GRIPPER_CLOSED_MM = 9.0
GRIPPER_OPEN_MM = 168.0

# 파지할 때만 쓰는 하한 — 빈 닫힘 폭(GRIPPER_CLOSED_MM)과 **일부러 분리했다**.
#
# 2026-08-25 사용자 지시: "최대한 세게 잡자". servo 6에는 토크 제한
# 레지스터가 없어 파지력은 오직 **명령 폭을 물체보다 얼마나 좁게 잡느냐**로만
# 만들어진다. 그런데 얇은 체스말은 이미 GRIPPER_CLOSED_MM(9.0)을 명령받고
# 있다 — queen(17mm)도 knight(22mm)도 _close_width가 하한에 clamp되므로,
# 하한 자체를 내리는 것 말고는 더 조일 방법이 없다.
#
# 두 하한을 나눈 이유는 안전이다. 물체가 턱 사이에 있으면 그 물체가 턱을
# 멈춰 주므로 더 좁게 명령해도 위치 오차(=힘)만 커진다. 하지만 **빈 채로**
# 턱이 맞닿는 지점보다 좁게 명령하면 턱이 서로를 밀어 서보가 계속 정지
# 토크를 낸다 — IDLE로 접기 전 닫기가 정확히 그 경우다. 그래서 빈 닫힘은
# 계속 GRIPPER_CLOSED_MM을 쓰고, 파지만 이 값을 쓴다.
#
# 2026-08-25 tools/gripper_force_probe.py 실측으로 7.0을 골랐다. 두 스윕이
# 서로 다른 것을 말해 주는데, 둘 다 필요했다.
#
#   빈 턱 (9.0 -> 2.0mm 스윕): 턱은 raw 1144에서 기계적으로 멈춘다. 그보다
#   좁게 명령해도 위치가 안 변하고 **부하도 안 는다**(0.0274 고정). 즉
#   빈 채로 더 좁게 명령하는 것은 서보를 태우지도 않지만 얻는 것도 없다.
#
#   knight을 문 채: 부하가 명령 폭을 따라 오른다 —
#       9.0mm 0.0235 / 8.0mm 0.0430 / 7.0mm 0.0626 / 그 아래 전부 0.0626
#   **7.0mm에서 포화한다.** 9.0 -> 7.0이 부하를 2.7배로 올리고, 6.0 아래로는
#   한 양자도 더 얻지 못한다.
#
# 그래서 7.0이 "최대한 세게"의 실제 답이었다 — **당시 결론**. 더 내려도
# 아무 일도 안 일어난다고 봤다. 영향을 받는 것은 하한에 걸려 있던 둘뿐이다:
#     chess_queen  9.0 -> 7.0      chess_knight  9.0 -> 7.0
# rook(9.5)과 낮은 물체 셋(25.0/30.0/31.0)은 원래 하한 위라 그대로였다.
#
# 빈 닫힘은 여전히 GRIPPER_CLOSED_MM(9.0)을 쓴다 — 얻을 것이 없으므로
# 굳이 바꾸지 않는다.
#
# ⚠️ 2026-09-02 사용자 지시로 재검토: 기어 사이에 이격(백래시)이 있다 —
# 위 스윕이 잰 부하는 모터(servo 6) 축 기준이라, 백래시를 다 흡수해
# 핑거 끝에 실제로 더 세게 물리는 구간이 있어도 축 쪽 부하 판독에는
# 그대로 "포화"로 보일 수 있다. 그래서 "더 내려도 소용없다"는 결론을
# 폐기하고, 서보가 받아들이는 한계까지 더 내린다(baseline_mission이
# 모든 라벨에 이 값을 직접 쓰도록 바뀐 것과 짝이다 — floor_grasp_policy.
# GRIPPER_GRASP_MIN_MM 주석 참고). knight 실측 스윕 자체를 무효로 보는
# 것은 아니다 — 모터 축에서는 여전히 포화해 보일 것이다.
GRIPPER_GRASP_MIN_MM = 0.0

# 2026-08-20, 핑거 안쪽 면 사이 거리. 링크 구조가 비선형이라 endpoint 두 점의
# 단일 선형 보간은 90 mm 요청에서 약 96 mm를 만들었다. 실측 중간점을 보존해
# 구간별 선형 보간한다.
GRIPPER_CALIBRATION_POINTS = (
    (9.0, 1150),
    (96.0, 1578),
    (168.0, 2000),
)


def position_from_width(width_mm: float, min_width_mm: float = GRIPPER_CLOSED_MM) -> int:
    """요청 폭을 안전 범위로 clamp하고 piecewise-linear goal count로 바꾼다.

    min_width_mm는 **파지 전용 하한**을 내려 주기 위한 것이다
    (GRIPPER_GRASP_MIN_MM 주석 참고). 기본값이 기존과 같은
    GRIPPER_CLOSED_MM이라, 인자를 안 주면 동작이 예전 그대로다.

    ⚠️ min_width_mm가 보정표 첫 점(9.0mm)보다 낮으면 첫 구간의 기울기를
    **외삽**한다. 그 아래는 실측점이 없으므로 돌아오는 raw는 "이만큼 좁게
    명령한다"는 뜻일 뿐 실제 개구 폭의 예측이 아니다 — 파지에서는 그것으로
    충분하다. 힘을 만드는 것은 도달할 폭이 아니라 **도달하지 못하는 거리**이기
    때문이다.
    """
    width = max(float(min_width_mm), min(GRIPPER_OPEN_MM, float(width_mm)))

    first_width, first_raw = GRIPPER_CALIBRATION_POINTS[0]
    if width < first_width:
        second_width, second_raw = GRIPPER_CALIBRATION_POINTS[1]
        slope = (second_raw - first_raw) / (second_width - first_width)
        return round(first_raw + (width - first_width) * slope)

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
