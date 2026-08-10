# -*- coding: utf-8 -*-
from domain.ports.arm_driver import ArmDriver


class FakeArm(ArmDriver):
    def __init__(self, move_ok: bool = True):
        self._move_ok = move_ok

    def move_to_cartesian(self, xyz, grip=None, down=False) -> bool:
        return self._move_ok

    def set_gripper(self, deg: float) -> None:
        pass

    def get_load(self) -> float:
        return 0.0
