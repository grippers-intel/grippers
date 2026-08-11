from domain.ports.base_driver import BaseDriver


class FakeBase(BaseDriver):
    def __init__(self, arrive: bool = True):
        self._arrive = arrive

    def drive_to(self, target) -> bool:
        return self._arrive

    def align_to_centerline(self) -> float:
        return 0.0

    def stop(self) -> None:
        pass
