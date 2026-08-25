"""시연 baseline 미션이 새로 필요로 하는 포트들.

기존 `domain/ports/*`를 고치지 않고 따로 둔다(사용자 지시 — baseline은 기존
FSM과 별개로 굴려 본다). 기존 `BaseDriver`·`ArmDriver`·`Perception`은 그대로
쓰고, 여기에는 **Host 링크**와 **라이다**만 더한다.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field


class Status:
    """Pi -> Host 보고 문자열. `MissionState.msg`의 `string state`에 그대로 실린다."""

    APPROACHING = "APPROACHING"
    AVOIDING = "AVOIDING"                      # 미세 회피함 — 경로 갱신 요청
    GRASP_DONE = "GRASP_DONE"
    GRASP_FAILED_RETRY = "HOST_REPLAN_RETRY"        # 물체 그대로 — 같은 목표
    GRASP_FAILED_RETARGET = "HOST_REPLAN_RETARGET"  # 물체 잃음 — 최근접으로 교체
    APPROACHING_BOX = "APPROACHING_BOX"
    INSERT_DONE = "INSERT_DONE"
    MISSION_DONE = "MISSION_DONE"


@dataclass(frozen=True)
class HostPlan:
    """Host가 오버헤드 웹캠으로 계산해 보내주는 한 사이클치 지시.

    Host가 목표 선정(가장 가까운 '체스말' 고르기)과 경로 계산을 모두 소유하므로
    Pi는 이걸 받아 따라가기만 한다."""

    target_label: str | None = None        # raw YOLO 클래스 — 'knight' 등
    destination: str | None = None         # 'chess' | 'toy'
    waypoints: tuple = ()                  # Host가 준 경로 (Pose2D 유사 객체들)
    grasp_ready: bool = False              # Host가 19cm 정렬을 확인했다 (지시 5)
    basket_bearing_rad: float = 0.0        # 라이다 창 중심 방향
    at_basket: bool = False                # Host가 바구니 앞 도착을 확인했다
    metadata: dict = field(default_factory=dict)


class HostLink(ABC):
    """Host PC와의 양방향 링크."""

    @abstractmethod
    def latest_plan(self) -> HostPlan | None:
        """가장 최근에 받은 지시. 아직 없으면 **None**.

        UDP라 패킷 하나가 빠져도 다음 사이클 것이 곧 온다 — 오래된 것을
        재전송받는 대신 최신만 본다(VEHICLE_LINK_PROTOCOL.md 참고)."""

    @abstractmethod
    def report(self, status: str, detail: str = "") -> None:
        """Pi의 현재 상태를 Host에 알린다. 실패해도 돌려줄 값이 없다 —
        보고가 안 닿으면 Host의 워치독이 알아서 판단한다."""


@dataclass(frozen=True)
class BasketFace:
    """라이다가 본 바구니 정면. 거리는 **라이다 원점 기준**이다."""

    ok: bool
    distance_m: float
    yaw_error_rad: float
    reason: str = ""


class Lidar(ABC):
    """2D 라이다. 바구니 정면 판정에만 쓴다.

    ⚠️ 바닥 물체 회피에는 쓸 수 없다 — 평면이 바닥 위 91mm라 체스말 위를
    지나간다. 라이다가 볼 수 있는 것은 벽과 바구니뿐이다.

    선분 피팅 자체는 도메인이 하지 않는다 — 그 수학은 ROS 패키지
    `grippers_base/basket_lidar_align.py`에 있고, 도메인 계층은 ROS 패키지를
    import하지 않는다(floor_grasp_policy.py의 계층 분리 주석과 같은 이유).
    real 어댑터가 그 모듈을 불러 결과만 이 포트로 넘긴다."""

    @abstractmethod
    def basket_face(self, bearing_rad: float) -> BasketFace:
        """기대 방위각 쪽 바구니 정면을 관측한다.

        **모르면 실패**(`ok=False`) — 점이 모자라거나 평면이 아니면 판정하지
        않는다. INSERT 전환을 막는 쪽이 안전하다."""
