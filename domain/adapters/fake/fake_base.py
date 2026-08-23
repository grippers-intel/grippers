"""FakeBase — BaseDriver 포트의 테스트 더블. 하드웨어·ROS2 없이 도메인 FSM을 검증한다.
기본값은 전부 '성공'이며, 생성자 인자로 실패 시나리오를 주입한다."""

from domain.ports.base_driver import ALIGN_FAILED_YAW_ERROR_RAD, BaseDriver
from domain.values import BoxObservation, Detection, Pose2D


class FakeBase(BaseDriver):
    def __init__(
        self,
        arrive: bool = True,
        align_ok: bool = True,
        align_error_rad: float = 0.0,
    ):
        self._arrive = arrive
        self._align_ok = align_ok
        self._align_error_rad = align_error_rad

    def drive_to(self, target: Pose2D) -> bool:
        return self._arrive

    def approach(self, target: Detection) -> bool:
        """`arrive`와 같은 플래그를 공유한다 — Fake는 FSM 로직만 검증하면
        되고, real 어댑터가 다루는 raw 클래스 특정 가능 여부 같은 하드웨어
        디테일은 여기서 흉내낼 필요가 없다(tests/test_mission_task.py의
        기존 FakeBase(arrive=False) 시나리오들과 계약을 그대로 공유한다)."""
        return self._arrive

    def align_to_box(self, box: BoxObservation) -> float:
        """`align_ok=False` 로 정렬 실패를 주입하면 포트 계약대로
        `ALIGN_FAILED_YAW_ERROR_RAD`(무한대)를 돌려준다.

        `align_error_rad` 를 크게 주는 것으로는 실패를 표현할 수 없다 — 그건
        '정렬은 됐는데 오차가 이만큼'이지 '정렬하지 못했다'가 아니다. 실패를
        0.0으로 표현하면 안 되는 이유는 포트 docstring 참고."""
        if not self._align_ok:
            return ALIGN_FAILED_YAW_ERROR_RAD
        return self._align_error_rad

    def stop(self) -> None:
        pass
