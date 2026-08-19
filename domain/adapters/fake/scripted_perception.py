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
        opening_mm: float | None = 400.0,
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

    def measure_opening(self, box: BoxObservation) -> float | None:
        """`opening_mm=None` 을 주면 **실측 실패(None)** 를 주입할 수 있다 —
        포트 계약상 None은 '해 없음' 취급이라 `POSE_PLAN` 이 `REJECT` 로 보낸다.

        ⚠️ 지금은 그 경로에 도달하지 않는다. `PosePlanState._solve_phi` 가 ⏸ 보류
        스텁이라 `opening_mm` 을 아예 보지 않고 항상 0.0(해 있음)을 돌려주기
        때문이다. **POSE_PLAN이 재도입되면 이 주입만으로 REJECT 경로가 열린다** —
        유즈케이스 2(투입 불가 판정 후 거부)를 CI에서 검증할 수 있게 된다.

        같은 값이 `find_box()` 가 만드는 `BoxObservation.opening_mm` 에도 실린다.
        상자 입구를 못 재는 상황이면 관측값도 없는 게 일관되고, 도메인은 정밀
        실측값(이 메서드의 반환값)만 판정에 쓴다."""
        return self._opening_mm

    def monitor_clearance(self) -> Clearance:
        """기본값은 `contact_risk=False`(happy path)다. 포트 계약의 "모르면 멈춘다"와
        어긋나 보이지만 그렇지 않다 — 그 계약은 **실측 수단이 없거나 응답이 없을 때**
        안전 쪽으로 기울라는 것이고, 테스트 더블은 시나리오를 주입받는 물건이라
        '모르는' 상태가 아니다.

        기본값을 True로 두면 `INSERT` 성공 경로를 아예 테스트할 수 없고,
        `found=True`·`box_found=True` 처럼 기본값을 전부 happy path로 두는 이 클래스의
        일관성도 깨진다. 위험 시나리오는 `contact_risk=True` 로 명시 주입한다."""
        return Clearance(front_m=1.0, left_m=1.0, right_m=1.0, contact_risk=self._contact_risk)
