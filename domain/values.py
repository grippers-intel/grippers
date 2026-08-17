"""domain 레이어 전용 값 객체. ROS2 메시지 타입을 여기서 쓰지 않는다 —
adapters/real/*.py가 이걸 grippers_interfaces 메시지로 변환하는 경계선.

단위 규약 (README §단위 규약)
    길이 m (_m) · 각도 rad (_rad) · 개구 폭 mm (_mm) · 부하 0.0~1.0
    _mm 접미사는 **길이에만** 쓴다. 각도에는 절대 쓰지 않는다.

모든 값 객체는 frozen dataclass다. State가 사이클을 건너 넘기는 값이므로
중간에 누가 바꿨는지 추적 불가능해지는 것을 타입으로 막는다.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, replace
from enum import Enum
from types import MappingProxyType


class ObjectClass(Enum):
    """수거 대상 클래스. **배정된 상자와 1:1** 이다.

    상자를 결정하지 않는 정보는 클래스로 만들지 않는다 —
    `TOY` 안의 원기둥/정육면체/육각기둥은 구분하지 않고,
    `CHESS` 안의 나이트/퀸/룩도 구분하지 않는다.
    """

    TOY = "toy"
    CHESS = "chess"


class BoxColor(Enum):
    """상자 색. 4개 전부 물리적으로 존재하며 투입 가능한 후보다.

    BLACK · RED 는 기본 TIDY 에서 미배정(방해 선택지)이고,
    명령으로 `placement_rule` 이 갱신되면 목적지가 된다.
    """

    BLACK = "black"
    RED = "red"
    BLUE = "blue"
    GREEN = "green"


class MissionMode(Enum):
    TIDY = "tidy"
    FETCH = "fetch"


@dataclass(frozen=True)
class Pose2D:
    x_m: float
    y_m: float
    theta_rad: float


@dataclass(frozen=True)
class Point3:
    x_m: float
    y_m: float
    z_m: float


@dataclass(frozen=True)
class Detection:
    """`scan_floor()` 가 반환하는 목록의 원소.

    `dims_m` 는 단안 추정값이다 — 모서리 웹캠 + 바닥면 호모그래피로 산출하므로
    바닥 평면 치수는 실측이지만 높이(z)는 얻을 수 없다. 세운 물체는 클래스 사전값 폴백.

    `track_id` 는 한 미션 안에서만 유효한 식별자다 (done_ids/held_ids 의 키).
    """

    track_id: int
    cls: ObjectClass
    pose_m: Point3
    dims_m: Point3
    yaw_rad: float
    confidence: float


@dataclass(frozen=True)
class BoxObservation:
    """`find_box()` 의 반환. 못 찾으면 포트가 None 을 반환한다 (이 타입은 '찾은 상태'만 표현)."""

    color: BoxColor
    pose_m: Pose2D
    opening_mm: float
    long_axis_rad: float


@dataclass(frozen=True)
class Clearance:
    """`monitor_clearance()` 의 반환.

    안전 기본값은 `contact_risk=True` 다 — 실제 측정이 되기 전까지 "모르면 멈춘다".
    """

    front_m: float
    left_m: float
    right_m: float
    contact_risk: bool


@dataclass(frozen=True)
class MissionSpec:
    """명령 1건이 결정하는 미션 파라미터. `CommandInterpreter.parse()` 의 반환.

    `placement_rule` 이 비어 있으면 SELECT 가 어떤 대상도 고르지 않는다 (조건 2).
    FETCH 가 아니면 `target_cls` 는 None 이다.
    """

    mode: MissionMode
    placement_rule: Mapping[ObjectClass, BoxColor]
    raw_text: str
    target_cls: ObjectClass | None = None


@dataclass(frozen=True)
class MissionContext:
    """루프를 건너 전달되는 미션 상태. State는 `ctx` + 작업 대상 1개 = 최대 2개 필드.

    `complete()` · `hold()` · `retry()` 는 **새 인스턴스를 반환**한다.
    재시도 카운터를 가변 필드로 두면 루프 안에서 누가 언제 증가시켰는지 추적이 안 된다.
    """

    spec: MissionSpec
    done_ids: frozenset[int] = frozenset()
    held_ids: frozenset[int] = frozenset()
    grasp_attempts: int = 0

    def complete(self, track_id: int) -> MissionContext:
        """처리 완료 등록 (INSERT / HANDOVER 성공). 재시도 카운터도 초기화한다."""
        return replace(self, done_ids=self.done_ids | {track_id}, grasp_attempts=0)

    def hold(self, track_id: int) -> MissionContext:
        """보류 등록 (APPROACH/GRASP/TRANSPORT 실패, REJECT). 이번 미션에서 제외."""
        return replace(self, held_ids=self.held_ids | {track_id}, grasp_attempts=0)

    def retry(self) -> MissionContext:
        """파지 재시도 1회 소진."""
        return replace(self, grasp_attempts=self.grasp_attempts + 1)

    def is_settled(self, track_id: int) -> bool:
        """이미 처리했거나 보류한 대상인가 — SELECT 의 조건 1."""
        return track_id in self.done_ids or track_id in self.held_ids


#: 기본 배치 규칙. TIDY 진입 시 명령에 별도 지정이 없으면 이걸 쓴다.
DEFAULT_PLACEMENT_RULE: Mapping[ObjectClass, BoxColor] = MappingProxyType(
    {
        ObjectClass.TOY: BoxColor.GREEN,
        ObjectClass.CHESS: BoxColor.BLUE,
    }
)
