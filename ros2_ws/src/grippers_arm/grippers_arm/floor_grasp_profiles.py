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
    "cube": FloorGraspProfile(40.0, 20.0, 60.0, 30.0),
    "star_column": FloorGraspProfile(45.0, 20.0, 60.0, 40.0),
    "soccer_polyhedron": FloorGraspProfile(46.0, 20.0, 65.0, 40.0),
    "chess_knight": FloorGraspProfile(22.0, 60.0, 35.0, 18.0),
    "chess_rook": FloorGraspProfile(24.5, 45.0, 38.0, 20.0),
    "chess_queen": FloorGraspProfile(17.0, 50.0, 30.0, 13.0),
}

# The smallest successful settled load measured while holding an object was
# 0.0704.  Keep the existing domain threshold lower than that value; load alone
# is not sufficient validation, so hardware tests also require lift and hold.
MEASURED_CUBE_HOLD_LOAD_RATIO = 0.0704
HARDWARE_HOLD_SECONDS = 5.0
MIN_GRIPPER_CLEARANCE_MM = 140.0

# Servo 1..5 angles in degrees.  These poses are specific to the measured arm
# mounting and floor.  Revalidate them after changing the arm or base mounting.
HORIZONTAL_SAFE_140_DEG = (-1.67, 45.87, 37.34, -83.70, 84.30)
HORIZONTAL_CHESS_MID_40_DEG = (-1.67, 96.57, -9.79, -87.29, 84.30)
HORIZONTAL_GABE_LOW_20_DEG = (-1.39, 95.70, -18.16, -68.88, 84.18)
HORIZONTAL_CHESS_ROOK_45_DEG = (-1.67, 93.87, -6.32, -88.06, 84.30)
HORIZONTAL_CHESS_QUEEN_50_DEG = (-1.67, 91.23, -3.04, -88.70, 84.30)
HORIZONTAL_CHESS_KNIGHT_60_DEG = (-1.67, 86.10, 3.06, -89.67, 84.30)

HORIZONTAL_GRASP_POSES_DEG = {
    "cube": HORIZONTAL_GABE_LOW_20_DEG,
    "star_column": HORIZONTAL_GABE_LOW_20_DEG,
    "soccer_polyhedron": HORIZONTAL_GABE_LOW_20_DEG,
    "chess_rook": HORIZONTAL_CHESS_ROOK_45_DEG,
    "chess_queen": HORIZONTAL_CHESS_QUEEN_50_DEG,
    "chess_knight": HORIZONTAL_CHESS_KNIGHT_60_DEG,
}
