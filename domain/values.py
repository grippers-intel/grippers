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


class ScanStatus(Enum):
    """`scan_floor()` 가 **관측을 수행했는가**. 검출 개수와는 다른 축이다.

    `OBSERVED` 는 카메라가 바닥을 실제로 봤다는 뜻이고, 그 결과 검출이 0개인
    것은 정상이다 — 치울 것이 남지 않았다는 관측이다. `UNAVAILABLE` 은 아예
    보지 못했다는 뜻이라 "남은 대상 없음" 으로 읽으면 안 된다 (이슈 #194)."""

    OBSERVED = auto()
    UNAVAILABLE = auto()


@dataclass(frozen=True)
class ScanResult:
    """`scan_floor()` 의 반환 계약. **빈 장면과 관측 실패를 타입에서 가른다.**

    예전 계약은 둘 다 `[]` 였다. 그래서 perception 이 통째로 죽어도 `SCAN` 이
    "남은 대상 없음" 으로 읽고 `DONE` 으로 갔다 — 센서 장애가 정상 완료로
    기록됐다 (이슈 #194).

    **truthiness 로 판정할 수 없다.** `__bool__` 이 `TypeError` 를 던진다.
    정의하지 않고 두면 파이썬 기본 동작상 **항상 참**이 되어 `if not result` 가
    조용히 거짓 분기로 흘러 결함을 숨긴다 — 그건 차단이 아니라 은폐다. 그래서
    막지 않고 **터뜨린다.** 호출자는 `status` · `observed_ok` · `detections` 를
    명시적으로 봐야 한다.

    `detections` 는 `tuple` 이다. `frozen=True` 는 **필드 재대입만** 막으므로
    `list` 를 그대로 담으면 만든 쪽이 나중에 원본을 고쳐 내용이 바뀐다.

    `reason` 은 `UNAVAILABLE` 일 때만 의미가 있는 진단 문자열이다. 실패 종류를
    세분한 enum 을 두지 않은 것은 **어댑터가 그만큼 알지 못하기 때문**이다 —
    `_ros_call.call_service()` 가 서비스 부재와 응답 없음을 둘 다 `None` 으로
    돌려주고 구분은 그쪽 경고 로그에만 남는다. 모르는 것을 아는 척하는 타입을
    만드는 대신, 어댑터가 아는 만큼만 문자열로 싣고 상세는 로그를 가리킨다."""

    status: ScanStatus
    detections: tuple[Detection, ...] = ()
    reason: str | None = None

    def __bool__(self):
        raise TypeError(
            "ScanResult 는 참·거짓으로 쓸 수 없다 — status · observed_ok · "
            "detections 를 명시적으로 확인할 것 (이슈 #194)"
        )

    def __post_init__(self):
        if self.status is ScanStatus.OBSERVED and self.reason is not None:
            raise ValueError("OBSERVED 에는 reason 을 싣지 않는다")
        if self.status is ScanStatus.UNAVAILABLE and self.detections:
            raise ValueError("UNAVAILABLE 에는 검출을 싣지 않는다")

    @classmethod
    def observed(cls, detections) -> "ScanResult":
        """관측 성공. `detections` 가 비어 있으면 '치울 것이 없다' 는 정상 관측이다."""
        return cls(status=ScanStatus.OBSERVED, detections=tuple(detections))

    @classmethod
    def unavailable(cls, reason: str) -> "ScanResult":
        """관측 실패. `reason` 은 비워 두지 않는다 — 실패 원인이 없으면 실기에서
        무엇이 끊겼는지 알 수 없다."""
        if not reason:
            raise ValueError("unavailable 에는 reason 이 필요하다")
        return cls(status=ScanStatus.UNAVAILABLE, reason=reason)

    @property
    def observed_ok(self) -> bool:
        """관측 자체가 성공했는가. 검출 유무와는 무관하다."""
        return self.status is ScanStatus.OBSERVED


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
