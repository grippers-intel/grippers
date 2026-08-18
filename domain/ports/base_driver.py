"""BaseDriver 포트 — ROS2를 전혀 모르는 순수 ABC.
grippers_base의 Ros2MecanumBase가 이걸 구현한다."""

import math
from abc import ABC, abstractmethod

from domain.values import BoxObservation, Pose2D

# 정렬 실패를 나타내는 yaw 오차. `0.0` 은 '완벽 정렬'이라 실패 표현으로 쓰면
# 허용 오차와 비교하는 어떤 판정도 통과해 버린다. 무한대는 어떤 임계값과
# 비교해도 실패로 남는다.
#
# 포트에 두는 이유: 이 값은 Fake와 real **양쪽의** 실패 표현이라, 한쪽 어댑터에만
# 두면 계약이 갈라진다 — 이 프로젝트에서 이미 두 번 사고가 난 부류다
# (ScriptedInterpreter의 ValueError vs None, FakeArm의 0~1 vs 서보 원시값).
ALIGN_FAILED_YAW_ERROR_RAD = math.inf


class BaseDriver(ABC):
    @abstractmethod
    def drive_to(self, target: Pose2D) -> bool:
        """target까지 주행한다. 도착하면 True.

        **실패(도달 불가 · 서버 부재 · 응답 없음)는 예외가 아니라 `False`.**
        `APPROACH`/`TRANSPORT` 가 대상을 보류 등록하고 `SCAN` 으로 복귀한다."""

    @abstractmethod
    def align_to_box(self, box: BoxObservation) -> float:
        """box 앞으로 정렬한다. 정렬 후 yaw 오차(rad)를 반환한다.

        **정렬하지 못하면 `ALIGN_FAILED_YAW_ERROR_RAD`(math.inf).** 오차가 큰 것과
        정렬 자체를 못 한 것은 다르다 — 후자를 `0.0` 으로 표현하면 안 된다.
        `0.0` 은 '완벽 정렬'이라 허용 오차와 비교하는 어떤 판정도 통과해 버린다."""

    @abstractmethod
    def stop(self) -> None:
        """즉시 정지 (cmd_vel 0).

        E-STOP 경로다 — **응답을 기다리지 않는다.** 실패해도 돌려줄 값이 없으므로
        로그만 남긴다."""
