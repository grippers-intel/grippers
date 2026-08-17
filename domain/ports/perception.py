"""Perception 포트 — ROS2를 전혀 모르는 순수 ABC.
grippers_perception의 LearnedPerception이 이걸 구현한다."""

from abc import ABC, abstractmethod

from domain.values import BoxColor, BoxObservation, Clearance, Detection


class Perception(ABC):
    @abstractmethod
    def scan_floor(self) -> list[Detection]:
        """바닥을 전역 관측해 검출 목록을 반환한다.
        더 처리할 대상이 없으면(상자 영역 마스킹 포함) **빈 리스트**를 반환해야 한다 —
        `SCAN` 은 빈 리스트를 '남은 대상 없음'으로 해석해 `DONE` 으로 전이한다."""

    @abstractmethod
    def find_box(self, color: BoxColor) -> BoxObservation | None:
        """지정한 색의 상자를 관측한다. 찾지 못하면 **`None`** 을 반환해야 한다 —
        `TRANSPORT` 는 `None` 을 받으면 대상을 보류 등록하고 `SCAN` 으로 복귀한다."""

    @abstractmethod
    def measure_opening(self, box: BoxObservation) -> float:
        """`box` 앞에 정렬한 상태에서 입구 폭(mm)을 정밀 실측한다.
        `POSE_PLAN` 이 이 값으로 φ 해 구간을 계산한다."""

    @abstractmethod
    def monitor_clearance(self) -> Clearance:
        """실제 측정이 되기 전까지는 **항상 `contact_risk=True` (정지)** 를 반환해야 한다 —
        '모르면 멈춘다'가 기본값이다."""
