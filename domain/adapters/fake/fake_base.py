# -*- coding: utf-8 -*-
from domain.ports.base_driver import BaseDriver


class FakeBase(BaseDriver):
    def drive_to(self, target) -> bool:
        return True

    def align_to_centerline(self) -> float:
        return 0.0

    def stop(self) -> None:
        pass
