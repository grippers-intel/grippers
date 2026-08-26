"""Pi 미션용 Fake — 하드웨어·네트워크 없이 FSM을 끝까지 굴린다."""

from domain.ports.baseline_ports import BasketFace, HostCommand, HostLink, Lidar


class FakeHostLink(HostLink):
    """`script`에 HostCommand를 차례로 넣어두면 한 번 호출에 하나씩 내준다.

    소진되면 마지막 것을 계속 돌려준다 — 이 저장소의 다른 스크립트 더블과
    같은 관례다. `None`을 넣으면 "명령이 안 왔다"를 흉내낸다(워치독 테스트용).

    보고는 `reports`에 (report, state, detail)로 쌓인다."""

    def __init__(self, script: list | None = None):
        self._script = list(script) if script else [None]
        self._idx = 0
        self.reports: list = []

    def latest_command(self) -> HostCommand | None:
        command = self._script[min(self._idx, len(self._script) - 1)]
        self._idx += 1
        return command

    def report(self, report: str, state: str, detail: str = "", fix=None) -> None:
        self.reports.append((report, state, detail, fix))

    @property
    def reported_kinds(self) -> list:
        return [report for report, _state, _detail, _fix in self.reports]

    @property
    def reported_states(self) -> list:
        return [state for _report, state, _detail, _fix in self.reports]

    @property
    def reported_fixes(self) -> list:
        """보정 요구가 실린 보고만. Host가 실제로 행동할 수 있는 것들이다."""
        return [(report, fix) for report, _state, _detail, fix in self.reports
                if fix is not None]


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

    def basket_face(self, bearing_rad: float = 0.0) -> BasketFace:
        self.calls += 1
        face = self._script[min(self._idx, len(self._script) - 1)]
        self._idx += 1
        return face
