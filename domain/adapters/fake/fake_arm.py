# -*- coding: utf-8 -*-
from domain.ports.arm_driver import ArmDriver


class FakeArm(ArmDriver):
    def move_to_cartesian(self, xyz, grip=None, down=False) -> bool:
        return True

    def set_gripper(self, deg: float) -> None:
        pass

    def get_load(self) -> float:
        return 0.0
