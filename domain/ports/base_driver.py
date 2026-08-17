"""BaseDriver 포트 — ROS2를 전혀 모르는 순수 ABC.
Real: `Ros2MecanumBase` (grippers_base 액션/서비스) · Fake: `FakeBase`

⚠️ Tier-1 freeze 대상 (#97) — 메서드 이름 · 인자 개수 · 단위 접미사는
PR + 3인 합의 없이 바꾸지 않는다.
"""

from abc import ABC, abstractmethod

from domain.values import BoxObservation, Pose2D


class BaseDriver(ABC):
    """메카넘 주행. `/cmd_vel` 을 발행하는 유일한 주체의 도메인 측 얼굴."""

    @abstractmethod
    def drive_to(self, target: Pose2D) -> bool:
        """`target` (map 프레임) 까지 주행. 도착하면 True, 도달 실패면 False.

        실패는 예외가 아니라 False 다 — 루프 FSM에서 도달 실패는 정상 경로
        (`APPROACH` → `SCAN` 보류 등록) 이기 때문.
        """

    @abstractmethod
    def align_to_box(self, box: BoxObservation) -> float:
        """상자 입구 장축에 로봇을 정렬하고 **남은 yaw 오차(rad)** 를 반환.

        반환값이 0.0 이 아니어도 실패가 아니다. 임계 판정은 호출자(`TRANSPORT`)가 한다.
        """

    @abstractmethod
    def stop(self) -> None:
        """즉시 정지 (`/cmd_vel` 0). E-STOP · 접촉 위험 시 호출."""
