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
    """루프 사이클을 건너 전달되는 불변 상태. `complete()`/`hold()`/`retry()`는
    새 인스턴스를 반환한다 — 재시도 카운터를 가변 필드로 두면 루프 안에서
    누가 언제 증가시켰는지 추적할 수 없다 (docs/design/class_diagram.md §1)."""

    spec: MissionSpec
    done_ids: frozenset = field(default_factory=frozenset)
    held_ids: frozenset = field(default_factory=frozenset)
    grasp_attempts: int = 0

    def complete(self, track_id: int) -> "MissionContext":
        return replace(self, done_ids=self.done_ids | {track_id})

    def hold(self, track_id: int) -> "MissionContext":
        return replace(self, held_ids=self.held_ids | {track_id})

    def retry(self) -> "MissionContext":
        return replace(self, grasp_attempts=self.grasp_attempts + 1)
