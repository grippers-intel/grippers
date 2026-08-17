"""ArmDriver 포트 — ROS2를 전혀 모르는 순수 ABC.
Real: `FeetechArm` (SO-ARM101 · STS3215 ×6) · Fake: `FakeArm`

⚠️ Tier-1 freeze 대상 (#97).

`set_gripper` 는 **개구 폭 mm** 를 받는다. 서보 각도 변환은 `FeetechArm` 내부의
캘리브레이션 테이블에서만 한다 (미결 #4 개구 폭 실측 결과가 그 테이블).
도메인 언어는 "몇 도 닫을까"가 아니라 "몇 mm 벌릴까"다.
"""

from abc import ABC, abstractmethod

from domain.values import Point3


class ArmDriver(ABC):
    @abstractmethod
    def move_to_cartesian(self, xyz_m: Point3, down: bool = False) -> bool:
        """손끝(TCP)을 `xyz_m` 으로 이동. 도달 불가(IK 해 없음·리치 초과)면 False.

        `down=True` 는 top-down 접근 자세를 요구한다 (파지 · 투입).
        기준 프레임은 `base_link` 로 고정한다 → freeze 문서 §3 D-2.
        """

    @abstractmethod
    def set_gripper(self, width_mm: float) -> None:
        """엔드이펙터 **개구 폭**(mm)을 지정. 각도가 아니다.

        하드웨어 상·하한을 넘는 값은 어댑터가 클램프한다 (예외를 던지지 않는다).
        """

    @abstractmethod
    def get_load(self) -> float:
        """그리퍼 서보(id6) 부하 비율 0.0~1.0. 파지 성공 판정의 유일한 근거."""

    @abstractmethod
    def reorient(self, phi_rad: float) -> bool:
        """장축과 수평면 사이 각도 `phi_rad` 가 되도록 파지 자세를 전환. 실패하면 False.

        ⏸ `POSE_PLAN` 보류 중이라 현재 호출부는 항상 `phi_rad=0.0` 을 넘긴다.
        구조는 유지한다 (3~4번째 범주 추가 시 재도입).
        """

    @abstractmethod
    def fold_to_cradle(self) -> bool:
        """Transport Pose — 크래들에 팔을 접어 안착. 주행 전 필수."""

    @abstractmethod
    def hold_position(self) -> None:
        """현재 관절 각을 유지 (토크 유지). E-STOP 시 파지물 낙하 방지."""
