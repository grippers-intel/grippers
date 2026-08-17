"""BaseDriver 포트 — ROS2를 전혀 모르는 순수 ABC.
grippers_base의 Ros2MecanumBase가 이걸 구현한다."""

from abc import ABC, abstractmethod

from domain.values import BoxObservation, Pose2D


class BaseDriver(ABC):
    @abstractmethod
    def drive_to(self, target: Pose2D) -> bool:
        """target까지 주행한다. 도착하면 True."""

    @abstractmethod
    def align_to_box(self, box: BoxObservation) -> float:
        """box 앞으로 정렬한다. 정렬 후 yaw 오차(rad)를 반환한다."""

    @abstractmethod
    def stop(self) -> None:
        """즉시 정지 (cmd_vel 0)."""
