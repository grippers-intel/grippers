"""Perception 포트 — ROS2를 전혀 모르는 순수 ABC.
Real: `LearnedPerception` (모서리 웹캠 + 호모그래피 + Hailo 검출) · Fake: `ScriptedPerception`

⚠️ Tier-1 freeze 대상 (#97).
⛔ **머지 순서 주의** — 이 파일의 변경은 배포판 단일화(#96) 이후에 머지한다.
   `scan_floor()` 가 목록을 반환하므로 `Detection.msg` / `DetectionArray.msg` 가 필요하고,
   Humble(3.10) ↔ Jazzy(3.12) 는 타입 해시가 달라 통신이 되지 않는다.

안전 기본값 — 실측·구현 전 Fake 및 스텁은 다음을 반환한다.
    scan_floor()        → []      (DONE 으로 빠짐)
    find_box()          → None    (보류 등록 유도)
    monitor_clearance() → contact_risk=True  ("모르면 멈춘다")
"""

from abc import ABC, abstractmethod

from domain.values import BoxColor, BoxObservation, Clearance, Detection


class Perception(ABC):
    @abstractmethod
    def scan_floor(self) -> list[Detection]:
        """바닥 전역을 1회 관측해 미처리 후보를 반환. 대상이 없으면 빈 목록.

        상자 내부 영역은 어댑터가 마스킹한다 — 투입한 물체가 다시 검출되면
        루프가 끝나지 않는다 (state_machine.md §4).
        """

    @abstractmethod
    def find_box(self, color: BoxColor) -> BoxObservation | None:
        """색으로 상자를 탐색. 시야에 없으면 None.

        ⚫ BLACK 은 LAB 탐색이 불가능하다 — 밝은 테두리 / ArUco 로 대체 탐색한다.
        """

    @abstractmethod
    def measure_opening(self, box: BoxObservation) -> float:
        """상자 입구 짧은 변을 **실측**해 mm 로 반환. `POSE_PLAN` 의 φ 해 탐색 입력.

        `BoxObservation.opening_mm` 는 탐색 시점의 추정값이고, 이 메서드는
        투입 직전 근거리 재측정이다 — 두 값이 다를 수 있고, 판정은 이쪽을 쓴다.
        """

    @abstractmethod
    def monitor_clearance(self) -> Clearance:
        """주변 여유 거리와 접촉 위험을 반환. 접촉 0회가 성공 기준이므로 폴링 대상."""
