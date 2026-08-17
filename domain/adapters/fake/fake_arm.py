"""FakeArm — ArmDriver 포트의 테스트 더블. 하드웨어·ROS2 없이 도메인 FSM을 검증한다.
기본값은 전부 '성공'이며, 생성자 인자로 실패 시나리오를 주입한다."""

from domain.ports.arm_driver import ArmDriver
from domain.values import Point3


class FakeArm(ArmDriver):
    def __init__(
        self,
        move_ok: bool = True,
        reorient_ok: bool = True,
        fold_ok: bool = True,
        load_ratio: float = 1.0,
    ):
        self._move_ok = move_ok
        self._reorient_ok = reorient_ok
        self._fold_ok = fold_ok
        self._load_ratio = load_ratio  # GRASP의 LOAD_THRESHOLD 판정 대상 — 기본값은 '꽉 쥠'

    def move_to_cartesian(self, xyz_m: Point3, down: bool = False) -> bool:
        return self._move_ok

    def set_gripper(self, width_mm: float) -> None:
        pass

    def get_load(self) -> float:
        return self._load_ratio

    def reorient(self, phi_rad: float) -> bool:
        return self._reorient_ok

    def fold_to_cradle(self) -> bool:
        return self._fold_ok

    def hold_position(self) -> None:
        pass
