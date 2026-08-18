"""ArmDriver 포트 — ROS2를 전혀 모르는 순수 ABC."""

from abc import ABC, abstractmethod

from domain.values import Point3


class ArmDriver(ABC):
    @abstractmethod
    def move_to_cartesian(self, xyz_m: Point3, down: bool = False) -> bool:
        """손끝을 xyz_m(m)로 이동한다. 도달 불가하면 False.
        그리퍼 개폐는 이 메서드가 하지 않는다 — `set_gripper()` 를 별도로 호출한다
        (docs/design/state_machine.md §3 `GRASP` 계약).

        **실패(도달 불가 · 서버 부재 · 응답 없음)는 예외가 아니라 `False`.**
        `GRASP` 가 재시도하고, 예산이 소진되면 대상을 보류 등록한다."""

    @abstractmethod
    def set_gripper(self, width_mm: float) -> None:
        """그리퍼 개구 폭을 width_mm(mm)로 맞춘다.
        ⚠️ 단위가 deg(각도)에서 mm(개구 폭)로 바뀌었다. 서보 각도 변환은
        어댑터(FeetechArm) 내부 캘리브레이션 테이블이 담당한다 (미결 #4 결과 반영).

        **돌려줄 값이 없으므로 실패는 로그로만 남는다.** 다만 조용히 삼켜지지는
        않는다 — 그리퍼가 닫히지 않았으면 뒤이은 `get_load()` 가 빈 채 부하를
        읽어 `GRASP` 가 파지 실패로 판정한다."""

    @abstractmethod
    def get_load(self) -> float:
        """그리퍼(id6) 부하 비율.

        **실패(서비스 부재 · 응답 없음 · 서보 읽기 실패)는 `0.0`** — '빈 채'로
        보아 파지 실패로 판정된다. 부하를 모르는 채로 성공 판정을 내려 물체를
        든 줄 알고 이송하는 것보다 안전하다."""

    @abstractmethod
    def reorient(self, phi_rad: float) -> bool:
        """손목을 장축-수평면 각 φ(rad)로 재조정한다. 정착에 실패하면 False.

        **서버 부재 · 응답 없음도 `False`** — `INSERT` 가 `REJECT` 로 넘겨
        물체를 내려놓고 보류 등록한다."""

    @abstractmethod
    def fold_to_cradle(self) -> bool:
        """팔을 이동용 거치 자세로 접는다. **실패는 `False`.**"""

    @abstractmethod
    def hold_position(self) -> None:
        """현재 관절 자세를 그대로 유지한다 (E-STOP 시 파지물 낙하 방지용).

        E-STOP 경로다 — **응답을 기다리지 않는다.** 실패해도 돌려줄 값이 없으므로
        로그만 남긴다."""
