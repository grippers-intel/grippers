"""ArmDriver 포트 — ROS2를 전혀 모르는 순수 ABC."""

from abc import ABC, abstractmethod

from domain.values import Point3


class ArmDriver(ABC):
    @abstractmethod
    def move_to_cartesian(self, xyz_m: Point3, down: bool = False) -> bool:
        """손끝을 xyz_m(m)로 이동한다. 도달 불가하면 False.
        그리퍼 개폐는 이 메서드가 하지 않는다 — `set_gripper()` 를 별도로 호출한다
        (docs/design/state_machine.md §3 `GRASP` 계약)."""

    @abstractmethod
    def set_gripper(self, width_mm: float) -> None:
        """그리퍼 개구 폭을 width_mm(mm)로 맞춘다.
        ⚠️ 단위가 deg(각도)에서 mm(개구 폭)로 바뀌었다. 서보 각도 변환은
        어댑터(FeetechArm) 내부 캘리브레이션 테이블이 담당한다 (미결 #4 결과 반영)."""

    @abstractmethod
    def get_load(self) -> float:
        """그리퍼(id6) 부하 비율 — **0.0~1.0 으로 정규화된 값**이다.

        ⚠️ 서보 원시값(STS3215 PRESENT_LOAD 는 0~1023)을 그대로 돌려주면 안 된다.
        정규화는 어댑터 뒤편(arm_driver_node)의 몫이다 — 도메인은 서보 레지스터
        범위를 알지 못한다. Fake 는 정규화된 값을, real 은 원시값을 주는 식으로
        계약이 갈라지면 CI는 통과하는데 실기에서만 파지 판정이 항상 실패한다."""

    @abstractmethod
    def reorient(self, phi_rad: float) -> bool:
        """손목을 장축-수평면 각 φ(rad)로 재조정한다. 정착에 실패하면 False."""

    @abstractmethod
    def fold_to_cradle(self) -> bool:
        """팔을 이동용 거치 자세로 접는다."""

    @abstractmethod
    def hold_position(self) -> None:
        """현재 관절 자세를 그대로 유지한다 (E-STOP 시 파지물 낙하 방지용)."""
