"""ScriptedPerception — Perception 포트의 테스트 더블. 하드웨어·ROS2 없이 도메인 FSM을 검증한다.
기본값은 전부 '성공'이며, 생성자 인자로 실패 시나리오를 주입한다.

⚠️ 2026-08-26 팀 확정으로 포트가 크게 줄었다. `scan_floor`(바닥 전역 관측),
`find_box`(바구니 찾기), `measure_opening`(입구 폭 실측)이 전부 사라졌다 —
물체 탐색과 바구니 위치는 Host가 오버헤드로 소유한다. 남은 것은 파지 직전
확인과 여유 공간 감시뿐이다.

이름이 `Fake*` 가 아니라 `Scripted*` 인 이유는 `identify_target` 이 단순
on/off 플래그가 아니라 **사이클별 라벨 스크립트**이기 때문이다."""

from domain.ports.perception import Perception
from domain.values import Clearance


class ScriptedPerception(Perception):
    def __init__(
        self,
        label: str | None = "queen",
        script: list | None = None,
        contact_risk: bool = False,
        grasp_confirmed: bool = True,
        target_remembered: bool = True,
    ):
        # `script`가 있으면 사이클마다 하나씩, 없으면 `label`을 계속 돌려준다.
        self._script = list(script) if script else None
        self._label = label
        self._idx = 0
        self._contact_risk = contact_risk
        self._grasp_confirmed = grasp_confirmed
        self._target_remembered = target_remembered
        self.identify_calls = 0
        self.confirm_grasp_calls = 0
        self.remember_target_calls = 0
        self.remembered_cls: str | None = None

    def identify_target(self) -> str | None:
        """정면 물체의 raw 라벨. `label=None`으로 "못 찾음"을 주입한다."""
        self.identify_calls += 1
        if self._script is None:
            return self._label
        label = self._script[min(self._idx, len(self._script) - 1)]
        self._idx += 1
        return label

    def monitor_clearance(self) -> Clearance:
        """기본값은 `contact_risk=False`(happy path)다. 포트 계약의 "모르면 멈춘다"와
        어긋나 보이지만 그렇지 않다 — 그 계약은 **실측 수단이 없거나 응답이 없을 때**
        안전 쪽으로 기울라는 것이고, 테스트 더블은 시나리오를 주입받는 물건이라
        '모르는' 상태가 아니다. 위험 시나리오는 `contact_risk=True` 로 명시 주입한다."""
        return Clearance(front_m=1.0, left_m=1.0, right_m=1.0, contact_risk=self._contact_risk)

    def remember_target(self, raw_cls: str) -> bool:
        """GRASP 로 내려가기 직전의 기준 관측. 기억한 클래스는
        `remembered_cls` 로 테스트에서 확인할 수 있다."""
        self.remember_target_calls += 1
        self.remembered_cls = raw_cls
        return self._target_remembered

    def confirm_grasp(self) -> bool:
        """기본값 `True`(happy path) — 이 클래스의 다른 기본값들과 같은 관례."""
        self.confirm_grasp_calls += 1
        return self._grasp_confirmed
