"""baseline 미션용 Fake — 하드웨어·네트워크 없이 FSM을 끝까지 굴린다."""

from domain.adapters.fake.fake_base import FakeBase
from domain.ports.baseline_ports import BasketFace, HostLink, HostPlan, Lidar


class FakeHostLink(HostLink):
    """`script`에 HostPlan을 차례로 넣어두면 한 번 호출에 하나씩 내준다.

    소진되면 마지막 것을 계속 돌려준다 — ScriptedPerception.scan_floor와 같은
    관례다. 보고는 `reports`에 (status, detail)로 쌓여 테스트에서 확인한다."""

    def __init__(self, script: list | None = None):
        self._script = list(script) if script else [None]
        self._idx = 0
        self.reports: list = []

    def latest_plan(self) -> HostPlan | None:
        plan = self._script[min(self._idx, len(self._script) - 1)]
        self._idx += 1
        return plan

    def report(self, status: str, detail: str = "") -> None:
        self.reports.append((status, detail))

    @property
    def reported_statuses(self) -> list:
        return [status for status, _detail in self.reports]


class FakeLidar(Lidar):
    """`script`에 BasketFace를 차례로 넣어두면 하나씩 내준다.

    기본값은 **관측 실패**다 — "모르면 실패"가 이 포트의 계약이라, 아무것도
    주입하지 않은 테스트가 우연히 INSERT까지 흘러가면 안 된다."""

    def __init__(self, script: list | None = None):
        self._script = list(script) if script else [
            BasketFace(False, float("inf"), float("inf"), "주입 없음")
        ]
        self._idx = 0
        self.calls = 0

    def basket_face(self, bearing_rad: float) -> BasketFace:
        self.calls += 1
        face = self._script[min(self._idx, len(self._script) - 1)]
        self._idx += 1
        return face


class FakeBaselineBase(FakeBase):
    """기존 FakeBase에 baseline이 요구하는 미세 이동 두 가지를 더한다.

    ⚠️ 실제 `Ros2MecanumBase`에는 아직 이 두 메서드가 없다 — baseline을
    실기로 돌리려면 배선해야 한다(BASELINE_MISSION_TODO.md)."""

    def __init__(self, *args, creep_ok: bool = True, **kwargs):
        super().__init__(*args, **kwargs)
        self._creep_ok = creep_ok
        self.creep_forward_calls: list = []
        self.creep_lateral_calls: list = []
        self.drive_calls: list = []
        self.stop_calls = 0

    def drive_to(self, target) -> bool:
        self.drive_calls.append(target)
        return super().drive_to(target)

    def creep_forward(self, distance_m: float) -> bool:
        self.creep_forward_calls.append(distance_m)
        return self._creep_ok

    def creep_lateral(self, distance_m: float) -> bool:
        self.creep_lateral_calls.append(distance_m)
        return self._creep_ok

    def stop(self) -> None:
        self.stop_calls += 1
