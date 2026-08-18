"""Perception 포트 — ROS2를 전혀 모르는 순수 ABC.
grippers_perception의 LearnedPerception이 이걸 구현한다."""

from abc import ABC, abstractmethod

from domain.values import BoxColor, BoxObservation, Clearance, Detection


class Perception(ABC):
    @abstractmethod
    def scan_floor(self) -> list[Detection]:
        """바닥을 전역 관측해 검출 목록을 반환한다.
        더 처리할 대상이 없으면(상자 영역 마스킹 포함) **빈 리스트**를 반환해야 한다 —
        `SCAN` 은 빈 리스트를 '남은 대상 없음'으로 해석해 `DONE` 으로 전이한다.

        **실패(서비스 부재 · 응답 없음)도 빈 리스트.** 관측이 안 되는데 계속 도는
        것보다 미션을 끝내고 이유를 로그로 남기는 편이 낫다."""

    @abstractmethod
    def find_box(self, color: BoxColor) -> BoxObservation | None:
        """지정한 색의 상자를 관측한다. 찾지 못하면 **`None`** 을 반환해야 한다 —
        `TRANSPORT` 는 `None` 을 받으면 대상을 보류 등록하고 `SCAN` 으로 복귀한다.
        서비스 부재 · 응답 없음도 같은 `None` 이다."""

    @abstractmethod
    def measure_opening(self, box: BoxObservation) -> float:
        """`box` 앞에 정렬한 상태에서 입구 폭(mm)을 정밀 실측한다.
        `POSE_PLAN` 이 이 값으로 φ 해 구간을 계산한다.

        **실측하지 못하면(서비스 부재 · 응답 없음) `None`** — '해 없음' 취급이라
        `POSE_PLAN` 이 `REJECT` 로 보낸다. 입구 폭을 모르는 채로 투입을 시도하면
        상자 테두리에 물체를 찍는다. (반환 타입 선언은 아직 `float` 이다 —
        `parse()` 와 같은 과도기로, 포트 시그니처를 한꺼번에 맞추는 후속 PR에서
        정리한다.)"""

    @abstractmethod
    def monitor_clearance(self) -> Clearance:
        """여유 공간을 관측한다. **실측 수단이 없거나 응답이 없으면(서비스 부재 ·
        타임아웃 포함) `contact_risk=True`(정지)** — '모르면 멈춘다'가 이 포트의
        기본값이다. 측정 실패를 통과 신호로 두면 실제 장애물을 못 보고 밀고
        지나가는 사고로 직결되므로, 이 메서드만은 실패값이 안전 쪽으로 치우쳐
        있어야 한다.

        ⚠️ '항상 True' 가 아니다. 비전이 미구현인 동안 real 구현이 True를 고정
        반환하는 것은 **그 구현의 현재 상태**이지 포트 계약이 아니다. 시나리오를
        주입받는 테스트 더블은 '모르는' 상태가 아니므로 기본값이 happy path여도
        계약 위반이 아니다 (`ScriptedPerception.monitor_clearance` 참고)."""
