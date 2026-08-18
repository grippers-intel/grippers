"""BaseDriver 포트 — ROS2를 전혀 모르는 순수 ABC.
grippers_base의 Ros2MecanumBase가 이걸 구현한다."""

from abc import ABC, abstractmethod

from domain.values import BoxObservation, Pose2D


class BaseDriver(ABC):
    @abstractmethod
    def drive_to(self, target: Pose2D) -> bool:
        """target까지 주행한다. 도착하면 True.

        **실패(도달 불가 · 서버 부재 · 응답 없음)는 예외가 아니라 `False`.**
        `APPROACH`/`TRANSPORT` 가 대상을 보류 등록하고 `SCAN` 으로 복귀한다."""

    @abstractmethod
    def align_to_box(self, box: BoxObservation) -> float:
        """box 앞으로 정렬한다. 정렬 후 yaw 오차(rad)를 반환한다.

        **실패는 `math.inf`.** `0.0` 은 '완벽 정렬'이라 실패 표현으로 쓰면 안 된다 —
        허용 오차와 비교하는 어떤 판정도 통과해 버린다."""

    @abstractmethod
    def stop(self) -> None:
        """즉시 정지 (cmd_vel 0).

        E-STOP 경로다 — **응답을 기다리지 않는다.** 실패해도 돌려줄 값이 없으므로
        로그만 남긴다."""
