"""FakeBase — BaseDriver 포트의 테스트 더블. 하드웨어·ROS2 없이 도메인 FSM을 검증한다.
기본값은 전부 '성공'이며, 생성자 인자로 실패 시나리오를 주입한다.

⚠️ 2026-08-26 팀 확정으로 포트에서 좌표가 사라졌다 — 여기에도
`drive_to`/`approach`/`align_to_box`가 없다. 남은 것은 속도와 정지뿐이다."""

from domain.ports.base_driver import BaseDriver


class FakeBase(BaseDriver):
    def __init__(self, creep_ok: bool = True):
        self._creep_ok = creep_ok
        self.velocity_calls: list = []
        self.creep_forward_calls: list = []
        self.creep_forward_timed_calls: list = []
        self.stop_calls = 0

    @property
    def last_velocity(self):
        """마지막으로 낸 속도. 아직 없으면 None."""
        return self.velocity_calls[-1] if self.velocity_calls else None

    def apply_velocity(self, linear_x: float, linear_y: float,
                       angular_z: float) -> None:
        self.velocity_calls.append((linear_x, linear_y, angular_z))

    def creep_forward(self, distance_m: float) -> bool:
        self.creep_forward_calls.append(distance_m)
        return self._creep_ok

    def creep_forward_timed(self, speed_mps: float, duration_s: float) -> bool:
        self.creep_forward_timed_calls.append((speed_mps, duration_s))
        return self._creep_ok

    def stop(self) -> None:
        self.stop_calls += 1
