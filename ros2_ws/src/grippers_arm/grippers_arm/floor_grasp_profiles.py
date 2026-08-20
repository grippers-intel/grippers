"""Measured floor-grasp targets for the current SO-ARM101 end effector.

These profiles describe object geometry and gripper widths only.  They do not
select an arm orientation: the vertical cube grasp is verified, while the
horizontal side-grasp orientation still needs staged hardware validation.
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
    "cube": FloorGraspProfile(40.0, 20.0, 60.0, 35.0),
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
