"""Measured floor-grasp targets for the current SO-ARM101 end effector.

Object geometry is kept separate from the named, hardware-tested arm poses so
that a low GABE pose is not accidentally reused for taller chess pieces.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class FloorGraspProfile:
    """Geometry and initial gripper commands for one object class."""

    object_width_mm: float
    grasp_center_height_mm: float
    preopen_width_mm: float
    close_width_mm: float


FLOOR_GRASP_PROFILES = {
    "cube": FloorGraspProfile(40.0, 20.0, 80.0, 30.0),
    "star_column": FloorGraspProfile(45.0, 20.0, 80.0, 35.0),
    "soccer_polyhedron": FloorGraspProfile(46.0, 20.0, 80.0, 35.0),
    "chess_knight": FloorGraspProfile(22.0, 60.0, 80.0, 13.0),
    "chess_rook": FloorGraspProfile(24.5, 45.0, 80.0, 15.0),
    "chess_queen": FloorGraspProfile(17.0, 50.0, 80.0, 13.0),
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
HORIZONTAL_GABE_LOW_20_DEG = (-1.39, 95.70, -18.16, -68.88, 84.18)
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
    "cube": HORIZONTAL_GABE_LOW_20_DEG,
    "star_column": HORIZONTAL_GABE_LOW_20_DEG,
    "soccer_polyhedron": HORIZONTAL_GABE_LOW_20_DEG,
    "chess_rook": HORIZONTAL_CHESS_ROOK_45_DEG,
    "chess_queen": HORIZONTAL_CHESS_QUEEN_50_DEG,
    "chess_knight": HORIZONTAL_CHESS_KNIGHT_60_DEG,
}
