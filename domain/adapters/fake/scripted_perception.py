"""ScriptedPerception — Perception 포트의 테스트 더블. 하드웨어·ROS2 없이 도메인 FSM을 검증한다.
기본값은 전부 '성공'이며, 생성자 인자로 실패 시나리오를 주입한다.

이름이 `Fake*` 가 아니라 `Scripted*` 인 이유는 `_script` 가 단순 on/off 플래그가 아니라
**사이클별 검출 목록**이기 때문이다 (docs/design/class_diagram.md §2,
docs/design/architecture.puml). `script` 를 넘기지 않으면 원소 1개짜리 기본 스크립트를
쓰는데, 이러면 매 `scan_floor()` 호출이 '같은 목록'을 반환하게 되어 무한 루프 방지
로직(state_machine.md §4)을 하드웨어 없이 CI에서 검증할 수 있다."""

from domain.ports.perception import Perception
from domain.values import (
    BoxColor,
    BoxObservation,
    Clearance,
    Detection,
    ObjectClass,
    Point3,
    Pose2D,
)

_DEFAULT_DETECTION = Detection(
    track_id=1,
    cls=ObjectClass.GABE,
    pose_m=Point3(x=0.3, y=0.0, z=0.0),
    dims_m=Point3(x=0.05, y=0.05, z=0.05),
    yaw_rad=0.0,
    confidence=0.9,
)


class ScriptedPerception(Perception):
    def __init__(
        self,
        found: bool = True,
        detections: list[Detection] | None = None,
        script: list[list[Detection]] | None = None,
        box_found: bool = True,
        opening_mm: float = 400.0,
        contact_risk: bool = False,
    ):
        if script is not None:
            self._script = script
        elif detections is not None:
            self._script = [detections]
        elif found:
            self._script = [[_DEFAULT_DETECTION]]
        else:
            self._script = [[]]
        self._call_count = 0
        self._box_found = box_found
        self._opening_mm = opening_mm
        self._contact_risk = contact_risk

    def scan_floor(self) -> list[Detection]:
        # 스크립트가 소진되면 마지막 원소를 계속 반환한다 (같은 목록 반복).
        idx = min(self._call_count, len(self._script) - 1)
        self._call_count += 1
        return self._script[idx]

    def find_box(self, color: BoxColor) -> BoxObservation | None:
        if not self._box_found:
            return None
        return BoxObservation(
            color=color,
            pose_m=Pose2D(x=0.5, y=0.0, theta=0.0),
            opening_mm=self._opening_mm,
            long_axis_rad=0.0,
        )

    def measure_opening(self, box: BoxObservation) -> float:
        return self._opening_mm

    def monitor_clearance(self) -> Clearance:
        return Clearance(front_m=1.0, left_m=1.0, right_m=1.0, contact_risk=self._contact_risk)
