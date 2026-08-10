# -*- coding: utf-8 -*-
"""BaseDriver 포트 — ROS2를 전혀 모르는 순수 ABC.
grippers_base의 Ros2MecanumBase가 이걸 구현한다."""
from abc import ABC, abstractmethod


class BaseDriver(ABC):
    @abstractmethod
    def drive_to(self, target) -> bool:
        """target: Pose2D 유사 객체(x, y, theta). 도착하면 True."""

    @abstractmethod
    def align_to_centerline(self) -> float:
        """정렬 후 yaw 오차(rad) 반환."""

    @abstractmethod
    def stop(self) -> None:
        """즉시 정지 (cmd_vel 0)."""
