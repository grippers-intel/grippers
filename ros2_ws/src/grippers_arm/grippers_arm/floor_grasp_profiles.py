"""Measured floor-grasp targets for the current SO-ARM101 end effector.

Object geometry is kept separate from the named, hardware-tested arm poses so
that a low GABE pose is not accidentally reused for taller chess pieces.
"""

from dataclasses import dataclass

# 절대 import를 쓴다 — align_to_idle.py 등 tools/*.py의 grippers_arm 참조와
# 같은 방식이다. 이 파일은 tests/test_floor_grasp_profiles.py에서
# importlib.util.spec_from_file_location으로 단독 로드되기도 하는데, 그
# 경로에선 패키지 컨텍스트가 없어 상대 import(`from .gripper_calibration`)가
# "attempted relative import with no known parent package"로 깨진다.
from grippers_arm.gripper_calibration import GRIPPER_CLOSED_MM, GRIPPER_OPEN_MM


@dataclass(frozen=True)
class FloorGraspProfile:
    """Geometry and initial gripper commands for one object class."""

    object_width_mm: float
    grasp_center_height_mm: float
    preopen_width_mm: float
    close_width_mm: float


# 2026-08-24: preopen_width_mm을 80.0(임의로 잡았던 절반쯤 열기)에서
# GRIPPER_OPEN_MM(기구적으로 안전하다고 실측된 최대 개구, 168.0)로 올림 —
# 사용자 지시: "무리가 되지 않는 범위 내에서 최대로" 열 것. GRIPPER_OPEN_MM
# 자체가 이미 gripper_calibration.py의 안전 clamp 상한이라 별도 여유값을
# 더 두지 않는다.
#
# 파지력은 **목표 폭을 물체 폭보다 얼마나 더 좁게 명령하느냐**로만 조절된다.
# servo 6에는 토크 제한 레지스터가 없다 — driver_sdk가 노출하는 것은
# set_torque(on/off)뿐이라, "더 세게"는 위치 오차를 키워 정지 토크를 키우는
# 것 말고 방법이 없다. 그래서 물체별로 제각각이던 여유(4.0~11.0mm)를
# GRIPPER_SQUEEZE_MM 하나로 통일한다.
#
# ⚠️ 2026-08-24 실기(사용자 보고: "파지할 때 더 세게 잡아야할 것 같아. 너무
# 흔들흔들거려"). 같은 회차 데이터가 이유를 그대로 보여준다 — 놓친 축구공은
# 닫힘 load 0.0860에서 midpoint 0.0430, safe 0.0391(빈손과 같음)로
# 무너졌고, 성공한 축구공은 0.0978 -> 0.0821로 버텼다. 두 경우의 차이는
# 3양자(0.0117)뿐이라 기존 여유는 성공/실패 경계 위에 놓여 있었다.
GRIPPER_SQUEEZE_MM = 15.0


def _close_width(object_width_mm: float) -> float:
    """물체 폭에서 GRIPPER_SQUEEZE_MM만큼 더 좁힌 목표 폭.

    기구 하한(GRIPPER_CLOSED_MM) 아래로는 내려가지 않는다. 얇은 체스말은
    이 하한에 걸려 여유를 다 못 쓴다 — queen(17.0mm)은 8.0mm, knight
    (22.0mm)은 13.0mm까지만 조일 수 있다. queen의 파지가 늘 가장 약했던
    (2026-08-24 실측 최소 마진 4양자) 진짜 이유가 이것이다: 더 세게 쥐려면
    GRIPPER_CLOSED_MM 자체를 재보정해야 한다.
    """
    return max(GRIPPER_CLOSED_MM, round(object_width_mm - GRIPPER_SQUEEZE_MM, 1))


# 2026-08-24: 낮은 물체 3종(cube/star_column/soccer_polyhedron)의 파지 중심
# 높이를 20.0 -> 26.0mm로 올림. 아래 HORIZONTAL_GABE_LOW_26_DEG 주석 참고.
FLOOR_GRASP_PROFILES = {
    "cube": FloorGraspProfile(40.0, 26.0, GRIPPER_OPEN_MM, _close_width(40.0)),
    "star_column": FloorGraspProfile(45.0, 26.0, GRIPPER_OPEN_MM, _close_width(45.0)),
    "soccer_polyhedron": FloorGraspProfile(46.0, 26.0, GRIPPER_OPEN_MM, _close_width(46.0)),
    "chess_knight": FloorGraspProfile(22.0, 60.0, GRIPPER_OPEN_MM, _close_width(22.0)),
    "chess_rook": FloorGraspProfile(24.5, 45.0, GRIPPER_OPEN_MM, _close_width(24.5)),
    "chess_queen": FloorGraspProfile(17.0, 50.0, GRIPPER_OPEN_MM, _close_width(17.0)),
}

# The smallest successful settled load measured while holding an object was
# 0.0704.  Keep the existing domain threshold lower than that value; load alone
# is not sufficient validation, so hardware tests also require lift and hold.
MEASURED_CUBE_HOLD_LOAD_RATIO = 0.0704
MIN_GRIPPER_CLEARANCE_MM = 140.0

# Servo 1..5 angles in degrees.  These poses are specific to the measured arm
# mounting and floor.  Revalidate them after changing the arm or base mounting.
# 2026-08-20 재실측: raw (2029, 2492, 2513, 1133, 3007)에서 실제 파지
# 중심 높이 145 mm, 차체 전면 기준 전방 185 mm, 중심선 기준 좌측 20 mm.
# 최소 140 mm 계약에 측정 여유 5 mm를 둔다.
HORIZONTAL_SAFE_145_DEG = (-1.67, 39.02, 40.87, -80.42, 84.29)
HORIZONTAL_SAFE_145_RAW = (2029, 2492, 2513, 1133, 3007)
# 2026-08-20 빈손 실측: 중심 높이 195 mm, 테두리 위 약 80 mm,
# 차체 전면 기준 전방 200 mm. SAFE_145와 같은 수평 손가락 방향을 유지한다.
BASKET_DROP_195_RAW = (2029, 2192, 2601, 1345, 3007)
HORIZONTAL_CHESS_MID_40_DEG = (-1.67, 96.57, -9.79, -87.29, 84.30)

# ⚠️ 2026-08-24 폐기. 사용자 보고: "큐브랑 축구공은 파지 높이를 맞추기 위해서
# 내려와 로봇암이 바닥에 약간 닿아."
#
# so101.urdf FK로 확인한 원인(계산은 tests/test_floor_grasp_profiles.py의
# 기하 검사에 그대로 들어 있다):
#
#   - base_link 원점은 바닥에서 98mm 위다. 이 상수는 추정이 아니라 실측된
#     네 자세(SAFE_145 / ROOK_45 / QUEEN_50 / KNIGHT_60)의 문서화된 파지
#     중심 높이와 FK z가 전부 정확히 98mm 차이라는 데서 나온다.
#   - 체스 자세 셋은 접근축(툴 local z)이 모두 수평(+0.51도)인데, 이
#     GABE 자세만 **8.66도 아래를 향한다**. 손가락 판이 파지 중심보다
#     앞·아래로 뻗어 있으므로 이 기울기가 판 끝을 바닥 아래로 밀어넣는다.
#
# 그런데 이 기울기는 잘못 가르친 게 아니라 **불가피하다**: 접근축을 수평으로
# 둔 채 파지 중심을 20mm까지 내리려면 servo2가 107~112도여야 하는데
# shoulder_lift의 URDF 한계는 ±100도다. 즉 팔은 이 높이에서 손가락을 수평으로
# 만들 수 없고, 아래로 기울이는 것이 20mm에 닿는 유일한 방법이었다.
#
# 그래서 기울기를 없애는 대신 **파지 중심을 6mm 올린다**. servo4만 -68.88 ->
# -71.05로 2.17도 움직이는 한 관절 변경이고, 나머지 넷은 그대로다. 결과:
# 파지 중심 20.0 -> 26.0mm, 접근축 -8.66 -> -6.49도, 전방 도달 370.0 ->
# 370.8mm(0.8mm — 물체 배치 위치는 사실상 그대로). 두 효과가 겹쳐 손가락 판
# 최저점이 약 7mm 올라간다.
#
# 파지 자체는 위태롭지 않다 — 이 자세를 쓰는 물체는 폭 40/45/46mm라 실제
# 중심이 20.0/22.5/23.0mm이고, 26mm는 그보다 3~6mm 위일 뿐 손가락 판이
# 물체를 충분히 감싼다. 오히려 star/soccer는 기존 20mm가 중심보다 낮았다.
HORIZONTAL_GABE_LOW_26_DEG = (-1.39, 95.70, -18.16, -71.05, 84.18)
HORIZONTAL_CHESS_ROOK_45_DEG = (-1.67, 93.87, -6.32, -88.06, 84.30)
HORIZONTAL_CHESS_QUEEN_50_DEG = (-1.67, 91.23, -3.04, -88.70, 84.30)
HORIZONTAL_CHESS_KNIGHT_60_DEG = (-1.67, 86.10, 3.06, -89.67, 84.30)

# 2026-08-20 실측 저부하 빈손 이동 자세. servo 1..5 raw를 그대로 보존한다.
# torque를 현재 위치에 latch한 뒤 관절 load가 모두 0인 것을 확인했다.
#
# 2026-08-24: servo 1-5 전체를 reteach_idle_pose.py로 손으로 다시 잡음 —
# torque 해제 후 팔 전체를 원하는 IDLE 자세로 재포즈(그리퍼 정면 정렬 포함).
IDLE_CRADLE_RAW = (2066, 829, 3092, 2751, 3071)

# IDLE_CRADLE과 수평 자세 사이에서 차체 접촉 없이 검증한 중간 waypoint.
VERTICAL_SAFE_OVERHEAD_DEG = (0.0, 9.2, 20.8, 55.3, 0.4)
HORIZONTAL_OVERHEAD_RAW = (2044, 2712, 2380, 1000, 3006)

HORIZONTAL_GRASP_POSES_DEG = {
    "cube": HORIZONTAL_GABE_LOW_26_DEG,
    "star_column": HORIZONTAL_GABE_LOW_26_DEG,
    "soccer_polyhedron": HORIZONTAL_GABE_LOW_26_DEG,
    "chess_rook": HORIZONTAL_CHESS_ROOK_45_DEG,
    "chess_queen": HORIZONTAL_CHESS_QUEEN_50_DEG,
    "chess_knight": HORIZONTAL_CHESS_KNIGHT_60_DEG,
}
