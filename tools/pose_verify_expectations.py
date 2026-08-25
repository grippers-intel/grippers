"""pose_verify_cycle이 "무엇이 나와야 하는가"를 계산하는 순수 로직.

ROS도 하드웨어도 참조하지 않는다 — 그래서 개발 머신에서 그대로 import해
단위 테스트할 수 있다(tools/pose_verify_cycle.py 자체는 rclpy를 import해서
불가능하다). 실제 서비스 호출과 출력은 그쪽 파일에 있다.

기대값의 출처는 전부 grippers_arm.floor_grasp_profiles다 — 이 파일에 수치를
베껴 적지 않는다. 검증 도구가 자기 사본을 들고 있으면, 프로파일이 바뀌었을 때
"도구는 통과하는데 팔은 다른 자세로 가는" 조용한 어긋남이 생긴다.
"""

from grippers_arm.floor_grasp_profiles import (
    BASKET_DROP_195_RAW,
    FLOOR_GRASP_PROFILES,
    HORIZONTAL_GRASP_POSES_DEG,
    HORIZONTAL_SAFE_145_RAW,
    IDLE_CRADLE_RAW,
)
from grippers_arm.gripper_calibration import GRIPPER_CLOSED_MM

# driver_sdk.STS3215Driver.degrees_to_position과 **완전히 같은 식**이어야 한다.
# 특히 round가 아니라 int(버림)다 — 1 raw 차이라 판정에는 영향이 없지만,
# 잔차 표를 볼 때 팔이 실제로 받은 목표와 도구가 기대하는 목표가 다르면
# 사람이 그 1을 계속 의심하게 된다.
POS_CENTER = 2048
COUNTS_PER_TURN = 4095


def deg_to_raw(deg: float) -> int:
    return int(POS_CENTER + (deg / 360.0) * COUNTS_PER_TURN)


def expected_poses(profile: str, frozen_servo1: int):
    """이번 회차에서 팔이 가야 할 servo 1..5 raw 자세들.

    ⚠️ safe/grasp/midpoint의 servo1은 등록값이 아니라 **이동을 시작할 때
    읽은 값(frozen_servo1)**이다. arm_driver_node._move_floor_stage가 그
    셋에서 servo1을 얼려 두기 때문이다 — APPROACH가 맞춰 놓은 좌우 정렬을
    등록 절대값으로 되돌리지 않으려는 것. idle/drop은 등록 절대값을 쓴다.
    """
    idle = tuple(IDLE_CRADLE_RAW)
    drop = tuple(BASKET_DROP_195_RAW)
    safe = (frozen_servo1,) + tuple(HORIZONTAL_SAFE_145_RAW[1:])
    grasp = (frozen_servo1,) + tuple(
        deg_to_raw(deg) for deg in HORIZONTAL_GRASP_POSES_DEG[profile][1:]
    )
    midpoint = tuple(round((g + s) / 2.0) for g, s in zip(grasp, safe, strict=True))
    return {"idle": idle, "safe": safe, "grasp": grasp, "midpoint": midpoint, "drop": drop}


# 한 회차의 체크포인트 — (이름, 기대 자세 키, 기대 그리퍼 폭 키).
#
# 그리퍼 폭 키는 FLOOR_GRASP_PROFILES 필드 이름이거나 "closed"(=GRIPPER_CLOSED_MM),
# None(이번 체크포인트에서는 폭을 기대하지 않음)이다.
#
# 순서는 실제 미션 순서 그대로다. 특히 두 가지는 안전 규칙이라 바꾸면 안 된다:
#   - 그리퍼는 **내려가기 전에** 연다(닫힌 손가락이 물체 자리를 통과하지 않게).
#   - 바닥에서 IDLE로 곧장 가지 않고 midpoint -> safe -> idle을 밟는다.
#   - 투하 뒤에는 **닫고 나서** 접는다.
CYCLE_CHECKPOINTS = (
    ("idle_start", "idle", None),
    ("safe_down", "safe", None),
    ("preopen", "safe", "preopen_width_mm"),
    ("grasp", "grasp", "preopen_width_mm"),
    ("closed", "grasp", "close_width_mm"),
    ("midpoint_up", "midpoint", "close_width_mm"),
    ("safe_up", "safe", "close_width_mm"),
    ("carry_idle", "idle", "close_width_mm"),
    ("drop", "drop", "close_width_mm"),
    ("released", "drop", "release_width_mm"),
    ("closed_to_fold", "drop", "closed"),
    ("idle_end", "idle", "closed"),
)

# 자세 판정 허용치 — arm_driver_node.FLOOR_POSE_START_TOLERANCE_RAW와 같은
# 값을 쓴다. 그쪽이 "다음 단계를 시작해도 되는가"의 기준이므로, 이 도구가
# 통과시킨 자세는 정의상 다음 단계가 받아들이는 자세여야 한다.
POSE_TOLERANCE_RAW = 120

# 그리퍼(servo 6)는 다르다. **위치 오차가 곧 파지력**이라(servo 6에는 토크
# 제한 레지스터가 없다) 물체를 물고 있으면 명령 폭에 도달하지 못하는 것이
# 정상이다. 그래서 servo 6 잔차는 실패 판정이 아니라 **측정값**으로 보고한다.
GRIPPER_WIDTH_REPORT_ONLY = True


def expected_gripper_mm(profile: str, width_key):
    if width_key is None:
        return None
    if width_key == "closed":
        return GRIPPER_CLOSED_MM
    return getattr(FLOOR_GRASP_PROFILES[profile], width_key)


def pose_residuals(expected, actual_1_to_5):
    """servo 1..5의 (실측 - 기대) raw. 둘 다 길이 5 시퀀스."""
    return [a - e for e, a in zip(expected, actual_1_to_5, strict=True)]


def pose_ok(residuals, tolerance=POSE_TOLERANCE_RAW):
    return all(abs(r) <= tolerance for r in residuals)


def load_verdict(carry_load, baseline_load, margin):
    """CARRY_IDLE의 load로 본 파지 성공 여부. 못 읽었으면 None.

    baseline_load는 **이번 회차 조건의 빈 기준선**이다 — 하드코딩된 상수가
    아니라 같은 세션의 --empty 회차에서 같은 체크포인트에서 잰 값을 쓴다.
    배터리 전압과 서보 온도에 따라 빈 기준선 자체가 움직이기 때문에, 다른
    날 잰 상수와 비교하는 것보다 같은 세션의 값과 비교하는 쪽이 맞다."""
    if carry_load is None or baseline_load is None:
        return None
    return (carry_load - baseline_load) > margin


def vision_verdict(h_before, found, h_after, ratio):
    """정면에서 목표가 사라졌는가. True=사라짐(성공 쪽), False=아직 있음.

    기준 관측이 없거나 응답이 없으면 None — 판정을 접는다. 두 관측 사이에
    차가 움직이지 않으므로(이 도구는 주행을 전혀 하지 않는다) demo_rook_run과
    달리 h가 커질 이유가 없다. 그래도 같은 비율을 쓴다: 판정 기준이 도구마다
    다르면 두 도구의 결과를 나란히 놓고 볼 수 없다."""
    if h_before is None or found is None:
        return None
    if not found:
        return True
    if h_after is None:
        return None
    return not (h_after >= h_before * ratio)
