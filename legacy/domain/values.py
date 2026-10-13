"""domain 레이어 전용 값 객체. ROS2 메시지 타입을 여기서 쓰지 않는다 —
adapters/real/*.py가 이걸 grippers_interfaces 메시지로 변환하는 경계선.

단위 규약은 필드명에 박혀 있다 (docs/design/class_diagram.md §1 — Tier-1 freeze 대상):
길이는 `_m`(미터) 또는 `_mm`(밀리미터), 각도는 `_rad`. `_mm`은 길이에만 쓰고 각도에는 쓰지 않는다.
"""

from dataclasses import dataclass, field, replace
from enum import Enum, auto


class ObjectClass(Enum):
    """배정된 상자와 1:1. 상자를 결정하지 않는 정보는 클래스로 만들지 않는다."""

    GABE = auto()
    CHESS_PIECE = auto()


class BoxColor(Enum):
    BLACK = auto()
    RED = auto()
    BLUE = auto()
    GREEN = auto()


class MissionMode(Enum):
    TIDY = auto()
    FETCH = auto()


@dataclass
class Pose2D:
    x: float
    y: float
    theta: float


@dataclass
class Point3:
    x: float
    y: float
    z: float


@dataclass
class Detection:
    track_id: int
    cls: ObjectClass
    pose_m: Point3
    dims_m: Point3
    yaw_rad: float
    confidence: float


@dataclass
class BoxObservation:
    color: BoxColor
    pose_m: Pose2D
    opening_mm: float
    long_axis_rad: float


@dataclass
class Clearance:
    front_m: float
    left_m: float
    right_m: float
    contact_risk: bool


@dataclass
class MissionSpec:
    mode: MissionMode
    target_cls: ObjectClass
    placement_rule: dict
    raw_text: str


@dataclass(frozen=True)
class MissionContext:
    """루프 사이클을 건너 전달되는 불변 상태. `complete()`/`hold()`/`retry()`/
    `reset_attempts()` 는 새 인스턴스를 반환한다 — 재시도 카운터를 가변 필드로 두면
    루프 안에서 누가 언제 증가시켰는지 추적할 수 없다 (docs/design/class_diagram.md §1).

    `last_scan` 은 **연속으로 같게 관측된 `SELECT` 후보 `track_id` 집합의 이력**이다 —
    원소는 `frozenset[int]` 이고, 후보 집합이 바뀌면 길이 1로 되돌아간다. 길이가
    `states.SCAN_NO_CHANGE_LIMIT` 에 닿으면 SCAN 무변화 감지가 발동한다
    (docs/design/state_machine.md §4). 사이클을 건너 비교해야 하므로 여기 있어야
    한다 — `ScanState` 자신에 두면 다른 State를 거쳐 SCAN으로 복귀할 때마다
    초기화돼 버려 비교가 성립하지 않는다.

    필드명은 `scan_floor()` 결과 전체를 담던 시절의 것이고 이름은 freeze 대상이라
    그대로 둔다 — 담는 값의 의미만 바뀌었다(이슈 #131). 검출 목록 전체를 비교하면
    보류된 물체가 바닥에 남아 Fake에서 과잉 발동하고, 실기에서는 `pose_m`·
    `confidence` 가 float이라 아예 발동하지 않았다.

    `complete()`/`hold()`/`retry()`/`reset_attempts()` 는 이 필드를 건드리지 않고
    그대로 넘긴다 (`dataclasses.replace` 기본 동작)."""

    spec: MissionSpec
    done_ids: frozenset = field(default_factory=frozenset)
    held_ids: frozenset = field(default_factory=frozenset)
    grasp_attempts: int = 0
    last_scan: tuple = ()

    def complete(self, track_id: int) -> "MissionContext":
        return replace(self, done_ids=self.done_ids | {track_id})

    def hold(self, track_id: int) -> "MissionContext":
        return replace(self, held_ids=self.held_ids | {track_id})

    def retry(self) -> "MissionContext":
        return replace(self, grasp_attempts=self.grasp_attempts + 1)

    def reset_attempts(self) -> "MissionContext":
        """재시도 예산을 되돌린다 — `SELECT` 가 새 대상을 고를 때만 호출한다.

        `grasp_attempts` 만 스코프가 **대상 1개**다 (state_machine.md §4). 미션
        누적으로 두면 첫 물체가 예산을 소진한 뒤 나머지 물체가 전부 첫 시도에서
        영구 보류된다 (이슈 #139).

        ⚠️ `GraspState` 에서 부르면 안 된다. `GRASP` 는 재시도할 때마다 자기 자신을
        새로 만들므로 거기서 되돌리면 카운터가 영원히 0에 머물러 무한 재시도가
        된다. 되돌리는 자리는 대상이 바뀌는 유일한 지점인 `SELECT` 하나다."""
        return replace(self, grasp_attempts=0)
