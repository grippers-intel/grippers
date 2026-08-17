"""FakeBase — BaseDriver 포트의 테스트 더블. 하드웨어·ROS2 없이 도메인 FSM을 검증한다.
기본값은 전부 '성공'이며, 생성자 인자로 실패 시나리오를 주입한다."""

from domain.ports.base_driver import BaseDriver
from domain.values import BoxObservation, Pose2D


class FakeBase(BaseDriver):
    def __init__(self, arrive: bool = True, align_error_rad: float = 0.0):
        self._arrive = arrive
        self._align_error_rad = align_error_rad

    def drive_to(self, target: Pose2D) -> bool:
        return self._arrive

    def align_to_box(self, box: BoxObservation) -> float:
        return self._align_error_rad

    def stop(self) -> None:
        pass
