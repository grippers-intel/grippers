"""domain 레이어 전용 값 객체. ROS2 메시지 타입을 여기서 쓰지 않는다 —
adapters/real/*.py가 이걸 grippers_interfaces 메시지로 변환하는 경계선."""

from dataclasses import dataclass


@dataclass
class Pose2D:
    x: float
    y: float
    theta: float


@dataclass
class Point3:
    x: float
    y: float
    z: float
