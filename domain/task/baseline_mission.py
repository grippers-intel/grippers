"""시연 baseline 미션 — Host 주도 흐름 (사용자 정리, 2026-08-25).

`domain/task/states.py`의 기존 루프 FSM과 **별개**다(사용자 지시). 기존
FSM은 Pi가 스스로 SCAN -> SELECT -> TRANSPORT를 하는 구조였는데, 이제
그 셋을 Host가 가져갔다. 여기서는 그 새 역할 분담을 그대로 표현한다.

흐름
    1. Host가 명령을 받는다 ("체스말 정리해줘")           <- Host
    2. Host가 좌표·경로를 계산하고 목표를 고른다           <- Host
    3. IDLE -> APPROACH. 경로를 따라가며 정면을 감시하고
       위험하면 미세 회피 후 Host에 보고                   <- Pi (여기)
    4. Host가 회피를 반영해 수정된 경로를 다시 보낸다      <- Host
    5. 차체 전면 19cm 정렬을 Host가 판정해 GRASP로 넘긴다  <- Host
    6. GRASP: 미세 전진 -> 파지 -> 판정 -> CARRY_IDLE      <- Pi (여기)
    7. 성공 보고 -> Host가 바구니 경로 -> APPROACH_BOX ->
       라이다로 정지 판정 -> INSERT -> IDLE                <- Pi (여기)

Pi는 "무엇을 할지"를 정하지 않는다. 경로를 따라가고, 팔을 쓰고, 무슨 일이
있었는지 보고할 뿐이다. 목표 선정과 재계획은 전부 Host 몫이다.

⚠️ 실기 미검증이다. 실측이 안 된 수치는 `baseline_constants.py`에서 `None`으로
두었고, 그 값이 필요한 자리는 판정을 포기하고 보고만 한다 — 지어낸 숫자로
도는 것처럼 보이게 하지 않는다.
"""

from dataclasses import dataclass

from domain.ports.baseline_ports import Status
from domain.task import baseline_constants as bc
from domain.task.floor_grasp_policy import (
    GRIPPER_MAX_SAFE_OPEN_MM,
    HorizontalGraspPlan,
    _release_width,
)
from domain.task.state import State

# states.py의 실측 상수를 그대로 쓴다 — 같은 계층이고, 실측값을 복제하면
# 한쪽만 고쳐지는 사고가 난다.
from domain.task.states import CLOSED_MM, GRASP_MIN_MM

# Host가 보내는 raw YOLO 라벨 -> 실측 교시 프로필.
#
# 기존 `select_horizontal_grasp_plan`은 검출 bbox의 바닥면 폭으로 프로필을
# 골랐다 — 그 함수 docstring이 "YOLO subtype이 아직 없으므로"라고 밝힌
# 임시 휴리스틱이다. baseline에서는 Host가 라벨을 직접 주므로 그 추측이
# 필요 없다. 폭 휴리스틱이 못 가르던 star/soccer도 여기서는 갈린다.
_PROFILE_BY_LABEL = {
    "queen": HorizontalGraspPlan("chess_queen", GRIPPER_MAX_SAFE_OPEN_MM, 13.0,
                                 _release_width(17.0)),
    "knight": HorizontalGraspPlan("chess_knight", GRIPPER_MAX_SAFE_OPEN_MM, 13.0,
                                  _release_width(22.0)),
    "rook": HorizontalGraspPlan("chess_rook", GRIPPER_MAX_SAFE_OPEN_MM, 15.0,
                                _release_width(24.5)),
    "box": HorizontalGraspPlan("cube", GRIPPER_MAX_SAFE_OPEN_MM, 30.0,
                               _release_width(40.0)),
    "star": HorizontalGraspPlan("soccer_polyhedron", GRIPPER_MAX_SAFE_OPEN_MM, 35.0,
                                _release_width(45.0)),
    "soccer": HorizontalGraspPlan("soccer_polyhedron", GRIPPER_MAX_SAFE_OPEN_MM, 35.0,
                                  _release_width(46.0)),
}


def plan_for_label(label):
    """raw 라벨에 맞는 교시 파지 계획. 모르는 라벨이면 **None** — 모르면 실패."""
    return _PROFILE_BY_LABEL.get(label)


def basket_stop_distance_m():
    """라이다가 읽을 때 멈춰야 하는 거리(라이다 원점 기준). 모르면 **None**.

    사용자가 정한 정지 거리는 **차체 전면** 기준 5cm인데, 라이다가 재는 것은
    라이다 원점 기준 거리다. 그 둘을 잇는 오프셋이 아직 미실측이라 계산할 수
    없다 — 지어낸 값으로 팔을 전개하느니 판정을 포기한다.

    ⚠️ 실측 후에도 확인할 것: 이 거리가 라이다 최소 측정 거리보다 짧으면
    표면이 아예 안 잡혀 판정이 불가능하다(`LIDAR_MIN_RANGE_M`)."""
    offset = bc.LIDAR_TO_CHASSIS_FRONT_M
    if offset is None:
        return None
    return bc.BASKET_APPROACH_STANDOFF_M + offset


class BaselineDoneState(State):
    """미션 1회 종료. 오케스트레이터가 다음 Host 명령을 기다린다."""

    name = "DONE"

    def __init__(self, ctx=None):
        self.ctx = ctx

    def execute(self, ports):
        return None


class BaselineIdleState(State):
    """Host의 지시를 기다린다. 지시가 오면 곧장 APPROACH로 간다.

    기존 FSM처럼 SCAN·SELECT를 거치지 않는다 — 목표는 Host가 이미 골랐다."""

    name = "IDLE"

    def execute(self, ports):
        plan = ports.host.latest_plan()
        if plan is None or plan.target_label is None:
            return self
        if plan_for_label(plan.target_label) is None:
            ports.host.report(Status.MISSION_DONE, f"모르는 라벨: {plan.target_label}")
            return BaselineDoneState()
        ports.host.report(Status.APPROACHING, plan.target_label)
        return BaselineApproachState(plan)


class BaselineApproachState(State):
    """Host가 준 경로를 따라가며 정면을 감시한다.

    GRASP로 넘어갈지는 **Host가 정한다**(지시 5) — 차체 전면 19cm 정렬을
    오버헤드 이미지로 판정해 `grasp_ready`로 알려준다. Pi는 그 판정을 다시
    하지 않는다. 오버헤드가 차량과 물체를 동시에 보므로 자기 카메라보다
    정확한 자리다."""

    name = "APPROACH"

    def __init__(self, plan, avoided=0):
        self.plan = plan
        self.avoided = avoided

    def execute(self, ports):
        plan = ports.host.latest_plan() or self.plan
        if plan.grasp_ready:
            return BaselineGraspState(plan)

        clearance = ports.perception.monitor_clearance()
        if clearance.contact_risk:
            return self._avoid(ports, plan)

        if plan.waypoints:
            ports.base.drive_to(plan.waypoints[0])
        return BaselineApproachState(plan, self.avoided)

    def _avoid(self, ports, plan):
        """정면이 위험하다 — 멈추고 옆으로 조금 비킨 뒤 Host에 알린다.

        비킨 사실을 알리면 Host가 그걸 반영한 경로를 다시 준다(지시 4).
        Pi가 스스로 경로를 다시 짜지 않는 이유는 Pi가 아레나 전체를 못
        보기 때문이다 — 자기 앞만 보고 크게 돌면 다른 물체로 들어간다."""
        ports.base.stop()
        if self.avoided >= bc.MAX_AVOID_STEPS:
            ports.host.report(Status.AVOIDING, "회피 예산 소진 — 전면 재계획 요청")
            return BaselineIdleState()
        step = bc.AVOID_LATERAL_STEP_M
        if step is None:
            ports.host.report(Status.AVOIDING, "AVOID_LATERAL_STEP_M 미실측 — 정지만 함")
            return BaselineIdleState()
        ports.base.creep_lateral(step)
        ports.host.report(Status.AVOIDING, f"{step:.3f}m 비킴 — 경로 갱신 요청")
        return BaselineApproachState(plan, self.avoided + 1)


class BaselineGraspState(State):
    """미세 전진 -> 파지 -> 부하 판정 -> CARRY_IDLE (지시 6).

    파지 동작 자체는 기존 `states.GraspState`에서 실기로 검증된 순서를 그대로
    따른다 — 벌리고 내려가고, 닫고, midpoint에서 다시 보고, safe를 거쳐
    IDLE로 접는다. 바뀐 것은 앞에 미세 전진이 붙고 결과를 Host에 보고한다는
    점뿐이다."""

    name = "GRASP"

    def __init__(self, plan):
        self.plan = plan

    def execute(self, ports):
        gp = plan_for_label(self.plan.target_label)
        ports.base.stop()

        if not ports.base.creep_forward(bc.GRASP_CREEP_FORWARD_MM / 1000.0):
            return self._failed(ports, "미세 전진 실패")

        if not ports.arm.move_to_floor_pose(gp.profile, "safe"):
            return self._failed(ports, "safe 자세 실패")
        ports.arm.set_gripper(gp.preopen_width_mm)
        ports.perception.remember_target(self.plan.target_label)
        if not ports.arm.move_to_floor_pose(gp.profile, "grasp"):
            return self._failed(ports, "grasp 자세 실패")

        ports.arm.set_gripper(max(GRASP_MIN_MM, gp.close_width_mm))
        load = ports.arm.get_load()
        lifted = load >= bc.LOAD_THRESHOLD and ports.arm.move_to_floor_pose(
            gp.profile, "midpoint")
        held = lifted and ports.arm.get_load() >= bc.LOAD_THRESHOLD
        cleared = held and ports.arm.move_to_floor_pose(gp.profile, "safe")
        if not cleared:
            return self._failed(ports, "들어 올리지 못함")

        if not ports.arm.move_to_floor_pose(gp.profile, "idle"):
            return self._failed(ports, "CARRY_IDLE 복귀 실패")
        if ports.arm.get_load() < bc.LOAD_THRESHOLD:
            return self._failed(ports, "CARRY_IDLE에서 빈손")

        ports.host.report(Status.GRASP_DONE, self.plan.target_label)
        return BaselineApproachBoxState(self.plan)

    def _failed(self, ports, detail):
        """파지 실패 — 정면을 다시 보고 두 갈래로 가른다 (사용자 지시).

        아직 보인다  -> 같은 목표로 경로만 다시 받는다
        사라졌다     -> 같은 클래스 중 최근접으로 교체를 요청한다

        `confirm_grasp()`의 False는 "아직 있다"와 "판정 불가"를 함께 뜻한다.
        둘 다 재시도로 보낸다 — 모를 때 목표를 버리는 것보다 한 번 더 해보는
        쪽이 정보를 덜 잃는다."""
        ports.base.stop()
        ports.arm.hold_position()
        vanished = ports.perception.confirm_grasp()
        status = Status.GRASP_FAILED_RETARGET if vanished else Status.GRASP_FAILED_RETRY
        ports.host.report(status, detail)
        return BaselineIdleState()


class BaselineApproachBoxState(State):
    """바구니 앞으로 이동하고, 라이다로 정지 시점을 판정한다 (지시 7).

    APPROACH와 같은 방식으로 Host 경로를 따라가되, 마지막 정지 판정만
    라이다가 한다 — 오버헤드는 바구니 입구까지의 거리를 cm 단위로 재기에
    각도가 나쁘고, 그 자리에서 팔을 전개하므로 틀리면 비싸다."""

    name = "APPROACH_BOX"

    def __init__(self, plan):
        self.plan = plan

    def execute(self, ports):
        plan = ports.host.latest_plan() or self.plan
        stop_at = basket_stop_distance_m()
        if stop_at is None:
            ports.host.report(
                Status.APPROACHING_BOX,
                "LIDAR_TO_CHASSIS_FRONT_M 미실측 — 정지 판정 불가")
            ports.base.stop()
            return BaselineIdleState()

        face = ports.lidar.basket_face(plan.basket_bearing_rad)
        if face.ok and face.distance_m <= stop_at:
            ports.base.stop()
            return BaselineInsertState(plan)

        if plan.waypoints:
            ports.base.drive_to(plan.waypoints[0])
        return BaselineApproachBoxState(plan)


class BaselineInsertState(State):
    """투하 자세로 전개해 물체를 떨어뜨리고 IDLE로 접는다 (지시 7).

    바닥 파지 높이로 내려가지 않는다 — 실측 DROP_195로 직접 전개한 뒤
    그리퍼를 연다. 활짝 열지 않고 물체가 빠져나올 만큼만 열며, 접기 **전에**
    닫는다(사용자 지시 2026-08-25, 기존 InsertState와 같은 원칙)."""

    name = "INSERT"

    def __init__(self, plan):
        self.plan = plan

    def execute(self, ports):
        ports.base.stop()
        gp = plan_for_label(self.plan.target_label)
        if not ports.arm.move_to_floor_pose(gp.profile, "drop"):
            ports.arm.hold_position()
            ports.host.report(Status.INSERT_DONE, "투하 자세 실패")
            return BaselineIdleState()

        ports.arm.set_gripper(gp.release_width_mm)
        ports.arm.set_gripper(CLOSED_MM)
        ports.arm.move_to_floor_pose(gp.profile, "idle")

        ports.host.report(Status.INSERT_DONE, self.plan.destination or "")
        ports.host.report(Status.MISSION_DONE, "")
        return BaselineDoneState()


@dataclass
class BaselinePorts:
    """baseline이 쓰는 포트 묶음. 기존 `Ports`에 host·lidar가 더해진다."""

    base: object
    arm: object
    perception: object
    host: object
    lidar: object
    estop: object


class BaselineMission:
    """`MissionTask`와 같은 제너레이터 구동 방식."""

    def __init__(self, ports):
        self.ports = ports

    def run(self):
        state = BaselineIdleState()
        while state is not None:
            if self.ports.estop.is_set():
                from domain.task.states import EstopState
                state = EstopState()
            yield state
            state = state.execute(self.ports)
