"""Host <-> Pi 링크 포트 (팀 확정, 2026-08-26).

## 역할 분담

팀 논의로 경계가 확정됐다. **Host가 공간에 관한 모든 것을 소유한다** — 물체
좌표, 차량 좌표와 방향, 경로 계산, 그리고 차량 제어 명령까지 전부 Host다.
**Pi는 그 명령을 실행하고 상태를 보고할 뿐이다.**

그래서 이 포트에는 좌표가 하나도 없다. Pi는 "어디로 가라"를 받지 않고
"이 속도로 움직여라"를 받는다. Waypoint·Pose2D·목표 좌표를 이 링크에 다시
얹고 싶어지면, 그건 Host가 해야 할 일이 Pi로 새는 신호다.

## Host -> Pi 로 오는 것 (다섯 가지)

    현 상태(State) · 직진(linear.x) · 수평이동(linear.y) · 제자리회전 · 제자리정지

`HostCommand` 하나에 담긴다. 합의된 속도는 `domain/task/motion.py`에 있다.

## Pi가 하는 일 (네 가지)

1. 현 state를 Host에 보고한다.
2. APPROACH -> GRASP 명령이 오면 **GRASP 조건이 충족됐는지 판단해 보고**한다.
   미충족이면 그 사실을 보고하고 **수정된 명령을 기다린다** — 스스로 고쳐서
   진행하지 않는다.
3. GRASP를 수행하고, CARRY로 전환 가능하다고 판단되면 파지 완료를 보고한다.
4. INSERT를 받으면 조건 충족 여부를 판단해 보고하고, INSERT를 실행해
   성공 여부를 판정한 뒤 IDLE 복귀까지 마치고 보고한다.

Pi가 "판단"하는 것은 **자기 센서로만 알 수 있는 것**뿐이다 — 그리퍼 부하,
팔 자세, 자기 뎁스 카메라가 본 목표, 라이다가 본 바구니 정면. 좌표계가
필요한 판단은 전부 Host 몫이다.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass


class MissionState:
    """Host와 Pi가 같은 이름으로 부르는 상태들.

    문자열인 이유: 이 이름이 그대로 UDP+JSON으로 나가고 `MissionState.msg`의
    `string state`에도 실린다. Enum으로 두면 양쪽에서 직렬화 규약을 따로
    맞춰야 하는데, 그 규약이 어긋나는 사고가 이 프로젝트에서 이미 두 번
    났다(BoxColor -> Destination 개명 때)."""

    IDLE = "IDLE"
    APPROACH = "APPROACH"
    GRASP = "GRASP"
    CARRY = "CARRY"
    APPROACH_BOX = "APPROACH_BOX"
    INSERT = "INSERT"
    DONE = "DONE"
    ESTOP = "ESTOP"

    ALL = (IDLE, APPROACH, GRASP, CARRY, APPROACH_BOX, INSERT, DONE, ESTOP)


class Report:
    """Pi -> Host 보고 종류. 위 docstring의 네 가지 임무에 하나씩 대응한다."""

    # 1번 — 주기 보고. 매 사이클 현재 state를 알린다.
    STATE = "STATE"

    # 2번 — GRASP 조건 판정 결과.
    GRASP_READY = "GRASP_READY"
    GRASP_BLOCKED = "GRASP_BLOCKED"      # 미충족 — 수정된 명령을 기다린다
    GRASP_CENTERING = "GRASP_CENTERING"  # Pi가 좌우 보정 중 — Host는 기다린다

    # 3번 — GRASP 수행 결과.
    GRASP_DONE = "GRASP_DONE"            # 파지 성공 + CARRY 전환 가능
    GRASP_FAILED = "GRASP_FAILED"

    # 4번 — INSERT 조건 판정과 수행 결과.
    INSERT_READY = "INSERT_READY"
    INSERT_BLOCKED = "INSERT_BLOCKED"
    INSERT_DONE = "INSERT_DONE"
    INSERT_FAILED = "INSERT_FAILED"
    IDLE_DONE = "IDLE_DONE"              # IDLE 복귀까지 완료

    # 명령 자체를 실행할 수 없을 때. 모르는 state 이름, 제자리회전과 병진이
    # 섞인 명령 등 — 추측해서 움직이지 않고 되돌려준다.
    REJECTED = "REJECTED"


@dataclass(frozen=True)
class HostCommand:
    """Host가 한 사이클에 보내는 지시 전부.

    좌표가 없다는 점이 이 자료형의 핵심이다. `state`는 "지금 이 상태로
    가라", 나머지 넷은 "그동안 이렇게 움직여라"다.

    속도 값은 Host가 채워 보내지만 Pi가 합의된 크기로 **잘라서** 쓴다
    (`domain/task/motion.py`). 안전 한계는 그 한계를 어길 수 있는 쪽이
    아니라 바퀴를 실제로 돌리는 쪽에 있어야 하기 때문이다."""

    state: str
    linear_x: float = 0.0        # 직진 (m/s, + 앞)
    linear_y: float = 0.0        # 수평이동 (m/s, + 왼쪽)
    angular_z: float = 0.0       # 제자리회전 (rad/s, + 반시계)
    stop: bool = False           # 제자리정지 — 나머지 셋을 무시하고 즉시 정지

    @property
    def wants_motion(self) -> bool:
        return not self.stop and (
            self.linear_x != 0.0 or self.linear_y != 0.0 or self.angular_z != 0.0)


class HostLink(ABC):
    """Host PC와의 양방향 링크."""

    @abstractmethod
    def latest_command(self) -> HostCommand | None:
        """가장 최근에 받은 명령. 아직 없으면 **None**.

        UDP라 패킷 하나가 빠져도 다음 사이클 것이 곧 온다 — 오래된 것을
        재전송받는 대신 최신만 본다(VEHICLE_LINK_PROTOCOL.md 참고).

        **None은 "정지"가 아니라 "모른다"다.** 이 둘을 섞으면 링크가 끊겼는데
        마지막 명령대로 계속 굴러가는 사고가 난다 — 호출하는 쪽이 워치독을
        따로 봐야 한다(`baseline_mission`의 `_motion_for` 참고)."""

    @abstractmethod
    def report(self, report: str, state: str, detail: str = "") -> None:
        """Pi의 상태·판정 결과를 Host에 알린다.

        `report`는 `Report`의 상수, `state`는 `MissionState`의 상수다. 둘을
        함께 보내는 이유: Host는 "무슨 일이 있었나"(report)와 "지금 어디에
        있나"(state)를 둘 다 알아야 다음 명령을 만들 수 있다.

        실패해도 돌려줄 값이 없다 — 보고가 안 닿으면 Host의 워치독이 알아서
        판단한다."""


@dataclass(frozen=True)
class BasketFace:
    """라이다가 본 바구니 정면. 거리는 **라이다 원점 기준**이다."""

    ok: bool
    distance_m: float
    yaw_error_rad: float
    reason: str = ""
    # 피팅에 쓰인 점 개수. 빔이 테두리를 스치기 시작하면 완전히 놓치기
    # **전에** 이 값이 먼저 줄어든다 — 절벽의 조기 신호다.
    point_count: int = 0
    # 차량 중심선에서 바구니 중심까지의 좌우 거리(+ 왼쪽).
    # `lateral_known`이 False면 **모르는 것**이지 0이 아니다.
    lateral_offset_m: float = 0.0
    lateral_known: bool = False


class Lidar(ABC):
    """2D 라이다. 바구니 정면 판정에만 쓴다.

    ⚠️ 바닥 물체 회피에는 쓸 수 없다 — 평면이 바닥 위 140mm에서 정면 아래로
    11.3도 기울어져 있어 체스말 위를 지나간다. 라이다가 볼 수 있는 것은 벽과
    바구니, 그리고 70cm 너머의 바닥뿐이다.

    선분 피팅 자체는 도메인이 하지 않는다 — 그 수학은 ROS 패키지
    `grippers_base/basket_lidar_align.py`에 있고, 도메인 계층은 ROS 패키지를
    import하지 않는다(floor_grasp_policy.py의 계층 분리 주석과 같은 이유).
    real 어댑터가 그 모듈을 불러 결과만 이 포트로 넘긴다."""

    @abstractmethod
    def basket_face(self, bearing_rad: float = 0.0) -> BasketFace:
        """정면 쪽 바구니를 관측한다.

        **모르면 실패**(`ok=False`) — 점이 모자라거나 평면이 아니면 판정하지
        않는다. INSERT 전환을 막는 쪽이 안전하다."""
