"""BaseDriver 포트 — ROS2를 전혀 모르는 순수 ABC.
grippers_base의 Ros2MecanumBase가 이걸 구현한다."""

import math
from abc import ABC, abstractmethod

from domain.values import BoxObservation, Detection, Pose2D

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
        `TRANSPORT` 가 대상을 보류 등록하고 `SCAN` 으로 복귀한다.

        ⚠️ 2026-08-23: `APPROACH`는 더 이상 이 메서드를 쓰지 않는다 — 아래
        `approach()` 참고. `drive_to`는 오도메트리 기반 절대좌표 점 주행이라
        `TRANSPORT`(상자 앞으로 이동)에는 맞지만, 물체 접근에는 실기 검증
        결과 부적합했다(HANDOFF.md "실패한 접근" — 초기 거리 추정 1회로 계산한
        목표 pose로 오도메트리 주행하면 드리프트가 누적돼 그리퍼가 물체를
        지나쳐버렸다)."""

    @abstractmethod
    def approach(self, target: Detection) -> bool:
        """물체 앞 파지 위치로 **시각 서보 폐루프**로 접근한다. 수렴하면 True.

        `drive_to`와 근본적으로 다르다 — 한 번의 pose 추정치로 목표점을 계산해
        거기로 주행하는 게 아니라, 정지→관측→소이동을 반복하며 **매번 다시
        관측한 오차**로 수렴시킨다(HANDOFF.md "왜 제어 루프인가" — 사람이
        한 번 맞춰도 드리프트가 쌓여 실패했다). 그래서 `target.pose_m`은 이
        메서드 내부에서 쓰이지 않을 수 있다 — real 구현은 라이브 카메라
        관측(화면 x=좌우, 박스 높이=거리, 두 독립 신호)에 다시 수렴시킨다.

        어떤 물체인지 구분은 `target.cls`(domain 대분류)가 아니라 raw YOLO
        클래스별 교시값으로 하는데, domain Detection에는 그 raw 클래스가 없다
        (`domain.task.floor_grasp_policy.approach_target_key` 참고). real
        구현이 폭 휴리스틱으로도 특정할 수 없으면(예: GABE의 star/soccer)
        **`False`**를 돌려준다 — 모르면 실패 관례.

        **실패(수렴 안 됨 · 물체를 놓침 · 특정 불가)는 `False`.**
        `APPROACH`가 대상을 보류 등록하고 `SCAN`으로 복귀한다."""

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
