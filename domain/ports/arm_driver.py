# -*- coding: utf-8 -*-
"""ArmDriver 포트 — ROS2를 전혀 모르는 순수 ABC."""
from abc import ABC, abstractmethod


class ArmDriver(ABC):
    @abstractmethod
    def move_to_cartesian(self, xyz, grip=None, down=False) -> bool:
        """손끝을 xyz(m)로. 도달 불가하면 False."""

    @abstractmethod
    def set_gripper(self, deg: float) -> None:
        """그리퍼만 deg(도)로."""

    @abstractmethod
    def get_load(self) -> float:
        """그리퍼(id6) 부하 비율."""
