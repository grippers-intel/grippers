"""`drive_to`의 ROS 비의존 평면 주행 제어 계산."""

import math
from typing import NamedTuple

ARRIVE_XY_TOL = 0.03  # m
YAW_DEADZONE_XY = 0.10  # m; ARRIVE_XY_TOL보다 커야 회전 발산 띠가 남지 않는다
KP_LINEAR = 0.6
KP_ANGULAR = 1.2
MAX_LINEAR = 0.2
MAX_ANGULAR = 0.5


class DriveCommand(NamedTuple):
    linear_x: float
    angular_z: float
    distance: float
    arrived: bool


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def compute_drive_command(dx: float, dy: float, yaw: float) -> DriveCommand:
    """목표까지의 world-frame 오차로 안전한 body-frame 명령을 계산한다.

    목표 근처에서는 atan2 방위각이 오도메트리 잡음에 크게 흔들리므로 회전을
    금지한다. 대신 잔여 오차를 로봇 전진축에 투영해 전진 또는 후진으로 거리만
    줄인다. deadzone 밖에서도 목표를 등진 상태로 전진하지 않아 궤도 운동을 막는다.
    """
    distance = math.hypot(dx, dy)
    if distance <= ARRIVE_XY_TOL:
        return DriveCommand(0.0, 0.0, distance, True)

    target_yaw = math.atan2(dy, dx)
    yaw_error = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
    heading_projection = math.cos(yaw_error)

    if distance <= YAW_DEADZONE_XY:
        linear_x = _clamp(KP_LINEAR * distance * heading_projection, MAX_LINEAR)
        angular_z = 0.0
    else:
        linear_x = _clamp(KP_LINEAR * distance * max(0.0, heading_projection), MAX_LINEAR)
        angular_z = _clamp(KP_ANGULAR * yaw_error, MAX_ANGULAR)

    return DriveCommand(linear_x, angular_z, distance, False)
