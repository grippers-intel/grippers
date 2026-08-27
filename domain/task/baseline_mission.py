"""Pi 미션 FSM — Host 명령을 실행하고 상태를 보고한다 (팀 확정, 2026-08-26).

## 이 FSM이 하는 일과 하지 않는 일

Host가 물체 좌표, 차량 좌표와 방향, 경로 계산, 차량 제어 명령을 전부
소유한다. 이 FSM은 **받은 명령을 실행하고, 자기 센서로만 알 수 있는 것을
판단해 보고할 뿐이다.**

그래서 여기에는 목표 선정도, 경로 계산도, 좌표 변환도 없다. 상태 전이는
Host가 보내는 `state`가 정하고, 주행은 Host가 보내는 속도가 정한다. Pi가
자기 판단으로 상태를 바꾸는 경우는 딱 둘이다 — GRASP/INSERT를 **실행한 뒤**
그 결과에 따라 다음 상태로 넘어갈 때, 그리고 조건 미충족으로 **넘어가지 않고
제자리에 머무를** 때.

## 네 가지 임무

1. 현 state를 매 사이클 Host에 보고한다.
2. GRASP 명령이 오면 조건을 판정해 보고한다. 미충족이면 **머무르고
   수정된 명령을 기다린다**(`preconditions.check_grasp`).
3. GRASP를 수행하고, CARRY로 전환 가능하면 파지 완료를 보고한다.
4. INSERT 명령이 오면 조건을 판정해 보고하고, 수행 후 성공 여부와 IDLE
   복귀 완료를 보고한다.

## 상태

    IDLE          대기. Host 지시를 기다린다.
    APPROACH      Host 속도대로 주행. GRASP 판정의 출발점.
    GRASP         파지 수행 (한 번의 execute에서 끝까지 간다).
    CARRY         물체를 든 채 Host 속도대로 주행. INSERT 판정의 출발점.
                  Host가 APPROACH_BOX를 지시하면 그 이름으로 보고한다.
    INSERT        투하 수행 후 IDLE 복귀.
    DONE          Host가 종료를 지시했다.

GRASP와 INSERT만 "한 번의 execute에서 시퀀스 전체를 수행"한다. 나머지는
사이클마다 명령을 받아 속도만 내는 얇은 상태다.

## 링크가 끊기면 멈춘다

`latest_command()`의 None은 "정지"가 아니라 "모른다"다. 이 둘을 섞으면
링크가 끊겼는데 마지막 명령대로 계속 굴러가는 사고가 난다. Host가 차량
제어를 소유한다는 것은 **Host가 말을 멈추면 차량도 멈춘다**는 뜻이기도
하다(`LinkWatchdog`).
"""

from dataclasses import dataclass, field

from domain.ports.baseline_ports import MissionState, Report
from domain.task import baseline_constants as bc
from domain.task import corrections
from domain.task import grasp_alignment as ga
from domain.task import preconditions as pc
from domain.task.floor_grasp_policy import (
    GRIPPER_MAX_SAFE_OPEN_MM,
    HorizontalGraspPlan,
    _close_width,
    _release_width,
)
from domain.task.motion import resolve_motion
from domain.task.state import State

# 그리퍼를 접기 전에 닫아 두는 폭. 벌린 채로 접으면 손가락이 차체에 걸린다
# (2026-08-25 사용자 지시).
CLOSED_MM = 9.0

# Pi 자기 뎁스캠이 내놓는 raw YOLO 라벨 -> 실측 교시 프로필.
#
# Host는 라벨을 보내지 않는다(명령은 state와 속도 넷뿐이다). 무엇을 집을지는
# **Pi가 자기 카메라로 확인한다** — 내려가는 것이 이 팔이므로 자기 눈으로 본
# 것에 맞춰 자세를 고른다. 이것이 Pi가 자기 YOLO를 계속 쓰는 유일한 이유다.
#
# 폭 값은 `floor_grasp_policy`의 실측 공식에서 유도한다. 여기에 숫자를 직접
# 적으면 ros2 프로필과 갈라진다 — 2026-08-26에 실제로 갈라져서 파지가 헐거워진
# 사고가 있었다(도메인 13.0 vs ros2 7.0).
_OBJECT_WIDTH_MM = {
    "queen": ("chess_queen", 17.0),
    "knight": ("chess_knight", 22.0),
    "rook": ("chess_rook", 24.5),
    "box": ("cube", 40.0),
    "star": ("star_column", 45.0),
    "soccer": ("soccer_polyhedron", 46.0),
}

_PROFILE_BY_LABEL = {
    label: HorizontalGraspPlan(profile, GRIPPER_MAX_SAFE_OPEN_MM,
                               _close_width(width_mm), _release_width(width_mm))
    for label, (profile, width_mm) in _OBJECT_WIDTH_MM.items()
}


def plan_for_label(label):
    """raw 라벨에 맞는 교시 파지 계획. 모르는 라벨이면 **None** — 모르면 실패."""
    return _PROFILE_BY_LABEL.get(label)


def object_width_mm(label):
    """그 라벨 물체의 실측 폭(mm). 모르는 라벨이면 **None**.

    턱이 쓸고 갈 영역의 좌우 허용치를 낼 때 쓴다 — 넓은 물체일수록 중심이
    덜 벗어나야 턱에 스치지 않고 들어온다."""
    entry = _OBJECT_WIDTH_MM.get(label)
    return entry[1] if entry else None


class LinkWatchdog:
    """Host 명령이 연속으로 몇 번 빠졌는지 센다.

    상태 객체가 전이마다 새로 만들어지므로 카운터는 여기 한 곳에 둔다."""

    def __init__(self, timeout_cycles: int = bc.HOST_COMMAND_TIMEOUT_CYCLES):
        self.timeout_cycles = timeout_cycles
        self.misses = 0

    def observe(self, command) -> bool:
        """명령을 받았으면 True. 연속 결측이 상한을 넘으면 False(=링크 끊김)."""
        if command is not None:
            self.misses = 0
            return True
        self.misses += 1
        return self.misses < self.timeout_cycles


@dataclass
class BaselinePorts:
    """Pi 미션이 쓰는 포트 묶음."""

    base: object
    arm: object
    perception: object
    host: object
    lidar: object
    estop: object
    watchdog: LinkWatchdog = field(default_factory=LinkWatchdog)


# ── 공통 동작 ──────────────────────────────────────────────────────────────


def _drive(ports, command, state_name) -> bool:
    """Host 속도를 베이스에 낸다. 명령이 부적합하면 정지 + 보고 후 False.

    거부 사유를 그대로 Host에 돌려주는 이유: Pi가 추측해서 둘 중 하나를
    실행하면 Host는 자기가 무엇을 잘못 보냈는지 영영 모른다."""
    decision = resolve_motion(command)
    if not decision.ok:
        ports.base.stop()
        ports.host.report(Report.REJECTED, state_name, decision.reason)
        return False
    if decision.motion.is_stop:
        ports.base.stop()
    else:
        ports.base.apply_velocity(decision.motion.linear_x,
                                  decision.motion.linear_y,
                                  decision.motion.angular_z)
    return True


def _link_ok(ports, state_name, command) -> bool:
    """워치독. 링크가 끊긴 것으로 보이면 정지하고 보고한다."""
    if ports.watchdog.observe(command):
        return True
    ports.base.stop()
    ports.host.report(Report.REJECTED, state_name,
                      f"Host 명령이 {ports.watchdog.misses}사이클 연속 없음 — 정지")
    return False


def _base_stopped(ports, command) -> bool:
    """지금 정지 상태인가. GRASP/INSERT 판정의 입력이다.

    베이스에 물어보지 않고 명령으로 판단하는 이유: 이 시점의 진실은 "Host가
    정지를 지시했는가"다. 바퀴의 실제 속도를 읽을 수단이 없기도 하다 —
    /odom_raw는 명령을 적분할 뿐이라 같은 것을 되돌려준다."""
    return command is None or command.stop or not command.wants_motion


# ── 상태 ──────────────────────────────────────────────────────────────────


class BaselineDoneState(State):
    """Host가 종료를 지시했다. 오케스트레이터가 다음 명령을 기다린다."""

    name = MissionState.DONE

    def execute(self, ports):
        ports.base.stop()
        ports.host.report(Report.STATE, self.name)
        return None


class BaselineIdleState(State):
    """대기. Host가 APPROACH를 지시하면 넘어간다."""

    name = MissionState.IDLE

    def execute(self, ports):
        command = ports.host.latest_command()
        if not _link_ok(ports, self.name, command):
            return self
        ports.host.report(Report.STATE, self.name)
        if command is None:
            return self
        if not _drive(ports, command, self.name):
            return self

        if command.state == MissionState.APPROACH:
            return BaselineApproachState()
        if command.state == MissionState.DONE:
            return BaselineDoneState()
        return self


class BaselineApproachState(State):
    """Host 속도대로 주행하며, GRASP 지시가 오면 조건을 판정한다 (임무 2번).

    조건이 미충족이면 **여기 머무른다.** 스스로 자세를 고치거나 위치를
    바꾸지 않는다 — 무엇을 고쳐야 하는지 Host에 알리고 수정된 명령을
    기다리는 것이 이 상태의 계약이다."""

    name = MissionState.APPROACH

    def __init__(self, retries: int = 0):
        self.retries = retries

    def execute(self, ports):
        command = ports.host.latest_command()
        if not _link_ok(ports, self.name, command):
            return self
        ports.host.report(Report.STATE, self.name)
        if command is None:
            return self

        if command.state == MissionState.GRASP:
            return self._judge_grasp(ports, command)

        if not _drive(ports, command, self.name):
            return self
        if command.state == MissionState.IDLE:
            return BaselineIdleState()
        if command.state == MissionState.DONE:
            return BaselineDoneState()
        return self

    def _judge_grasp(self, ports, command):
        """임무 2번 — 조건 판정 후 보고. 충족이면 GRASP로, 아니면 제자리.

        판정은 두 겹이다. 먼저 기본 전제(E-STOP·정지·빈 그리퍼·식별)를 보고,
        통과하면 **물체가 턱이 쓸고 갈 영역 안에 있는지**를 본다.

        ⚠️ 이 한 번의 판정에 약 1.7초가 든다(2026-08-26 실측). identify_target이
        오검출을 거르려고 5프레임 합의를 쓰고 CPU 추론이 프레임당 0.3초쯤
        걸리기 때문이다. 클래스 6개를 묻지만 표본은 한 번만 뜬다.

        그동안 이 사이클은 Host 명령을 읽지도 보고하지도 않는다. 워치독은
        안 걸린다 — 명령이 **안 온** 것이 아니라 **안 읽은** 것이고, 링크는
        최신 것만 들고 있다가 다음 읽기에 내준다. 다만 **Host 쪽에서는
        약 1.7초 동안 보고가 끊긴다** — Host 워치독을 그보다 넉넉히 잡아야
        한다."""
        ports.base.stop()
        observation = ports.perception.identify_target()
        label = observation.label if observation is not None else None
        report = pc.check_grasp(pc.GraspInputs(
            estop_set=ports.estop.is_set(),
            base_stopped=_base_stopped(ports, command),
            gripper_load=ports.arm.get_load(),
            detected_label=label,
            profile_known=plan_for_label(label) is not None,
        ))
        if not report.ok:
            ports.host.report(Report.GRASP_BLOCKED, self.name, report.detail)
            return self

        return self._judge_alignment(ports, observation, label)

    def _judge_alignment(self, ports, observation, label):
        """좌우·전후 정렬 판정 (사용자 지시 2026-08-26).

        영역 안이면 내려가고, 영역 안인데 치우쳤으면 **Pi가 servo 1로 고친
        뒤 다시 본다**, 영역 밖이면 Host에 다시 세워 달라고 한다.

        보정 직후에 곧장 내려가지 않고 한 사이클 더 관측하는 이유: 이
        저장소의 접근 제어가 전부 "관측 -> 소이동 -> 재관측" 폐루프다.
        한 번 계산한 값으로 열린 루프를 돌면 오차가 쌓인다는 것이 이미
        실기로 확인됐다."""
        verdict = ga.judge(observation, object_width_mm(label))

        if verdict.action == ga.READY:
            creep_m = ga.creep_distance_m(observation)
            ports.host.report(
                Report.GRASP_READY, self.name,
                f"{label} {verdict.reason} · 전진 {creep_m * 1000:.0f}mm")
            return BaselineGraspState(label, creep_m, self.retries)

        if verdict.action == ga.PI_CENTER:
            if ports.arm.offset_base_yaw(verdict.servo1_offset_rad):
                ports.host.report(Report.GRASP_CENTERING, self.name, verdict.reason,
                                  corrections.from_alignment(verdict))
            else:
                # 관절이 거부했다 — 한계각 초과나 범위 밖이다. 팔로 못 고치면
                # 차량이 다시 서야 한다.
                ports.host.report(
                    Report.GRASP_BLOCKED, self.name,
                    f"{verdict.reason} — servo 1이 거부했다, 재회전 필요",
                    corrections.Correction(corrections.ROTATE,
                                           lateral_m=verdict.lateral_error_m))
            return self

        ports.host.report(Report.GRASP_BLOCKED, self.name, verdict.reason,
                          corrections.from_alignment(verdict))
        return self


class BaselineGraspState(State):
    """파지 수행 (임무 3번).

    실기로 검증된 순서를 그대로 따른다 — 벌리고, 내려가고, 물체를 턱 사이로
    밀어 넣고, 닫고, midpoint에서 부하를 다시 보고, safe를 거쳐 CARRY로 접는다.

    ⚠️ 마지막이 IDLE이 아니라 **CARRY**인 것이 중요하다. 물체를 문 채 IDLE로
    접으면 그리퍼가 라이다 정면을 79% 가려 바구니를 못 본다(2026-08-26 실측,
    floor_grasp_profiles.CARRY_RAW 주석)."""

    name = MissionState.GRASP

    def __init__(self, label, creep_m, retries: int = 0):
        self.label = label
        self.creep_m = creep_m
        self.retries = retries

    def execute(self, ports):
        ports.host.report(Report.STATE, self.name)
        gp = plan_for_label(self.label)
        ports.base.stop()

        # 전진 거리는 관측에서 나온다 — 상수를 그대로 밀면 이미 가까운 물체를
        # 턱 안쪽으로 처박는다(grasp_alignment.creep_distance_m 참고).
        if self.creep_m is None:
            return self._failed(ports, "전진 거리를 모른다 — 관측 실패")
        if not ports.base.creep_forward(self.creep_m):
            return self._failed(ports, "미세 전진 실패")

        if not ports.arm.move_to_floor_pose(gp.profile, "safe"):
            return self._failed(ports, "safe 자세 실패")
        ports.arm.set_gripper(gp.preopen_width_mm)
        ports.perception.remember_target(self.label)
        if not ports.arm.move_to_floor_pose(gp.profile, "grasp"):
            return self._failed(ports, "grasp 자세 실패")

        ports.arm.set_gripper(gp.close_width_mm)
        load = ports.arm.get_load()
        lifted = load >= bc.LOAD_THRESHOLD and ports.arm.move_to_floor_pose(
            gp.profile, "midpoint")
        held = lifted and ports.arm.get_load() >= bc.LOAD_THRESHOLD
        cleared = held and ports.arm.move_to_floor_pose(gp.profile, "safe")
        if not cleared:
            return self._failed(ports, f"들어 올리지 못함 (부하 {load:.4f})")

        if not ports.arm.move_to_floor_pose(gp.profile, "carry"):
            return self._failed(ports, "CARRY 전환 실패")

        # 성공 판정은 **독립적인 두 신호가 모두** 있어야 한다(사용자 지시
        # 2026-08-26). 부하는 "무언가를 쥐고 있다"를, 뎁스 카메라는 "있던
        # 물체가 그 자리에서 사라졌다"를 말한다 — 실패 양상이 겹치지 않는다.
        #
        # 부하만 보면 물체 모서리를 살짝 물었거나 턱이 서로를 문 경우도
        # 통과한다. 뎁스만 보면 내려오는 그리퍼가 물체를 **쳐서 시야 밖으로
        # 밀어낸** 경우도 "사라짐"으로 읽힌다. 둘 다 요구하면 각자의 오검출이
        # 서로를 막는다.
        carried = ports.arm.get_load()
        vanished = ports.perception.confirm_grasp()
        if carried < bc.LOAD_THRESHOLD and not vanished:
            return self._failed(ports, f"부하도 낮고 물체도 그대로다 (부하 {carried:.4f})")
        if carried < bc.LOAD_THRESHOLD:
            return self._failed(ports, f"CARRY에서 빈손 (부하 {carried:.4f})")
        if not vanished:
            # 쥐고는 있는데 목표는 제자리다 — 엉뚱한 것을 물었거나 물체를
            # 쳐 놓고 턱끼리 문 경우다.
            return self._failed(
                ports, f"부하는 {carried:.4f}인데 목표가 그 자리에 남아 있다")

        ports.host.report(Report.GRASP_DONE, MissionState.CARRY,
                          f"{self.label} 부하 {carried:.4f} · 목표 사라짐 확인")
        return BaselineCarryState(self.label)

    def _failed(self, ports, detail):
        """파지 실패 — 팔을 붙잡고 APPROACH로 되돌아가 Host의 판단을 기다린다.

        Pi가 스스로 재시도하지 않는다. 다시 시도할지, 다른 물체로 바꿀지,
        어디로 옮겨 설지는 아레나 전체를 보는 Host가 정한다 — 그래서 여기엔
        재시도 상한이 없다(예전엔 baseline_constants.MAX_GRASP_RETRY라는
        미사용 상수가 있었지만, 이 설계 원칙과 어긋나 2026-08-28에 지웠다).
        다만 몇 번째 시도가 실패했는지는 Host가 판단을 내리는 데 필요한
        정보라 detail에 실어 보낸다(2026-08-28)."""
        attempt = self.retries + 1
        ports.base.stop()
        ports.arm.hold_position()
        ports.host.report(Report.GRASP_FAILED, MissionState.APPROACH,
                          f"{attempt}번째 시도 실패 — {detail}")
        return BaselineApproachState(self.retries + 1)


class BaselineCarryState(State):
    """물체를 든 채 Host 속도대로 주행하고, INSERT 지시가 오면 판정한다 (임무 4번).

    Host가 `CARRY`를 보내든 `APPROACH_BOX`를 보내든 하는 일은 같다 — 받은
    속도를 낸다. 보고하는 이름만 Host가 부른 이름을 따른다."""

    name = MissionState.CARRY

    def __init__(self, label, reported_as: str = MissionState.CARRY,
                 previous=None):
        self.label = label
        self.reported_as = reported_as
        # 직전 사이클의 (라이다 거리, 그리퍼 부하). INSERT 판정의 "흔들리지
        # 않는가"·"미끄러지지 않는가"가 이 표본과 비교해서 나온다.
        self.previous = previous
        self.sample = None

    def execute(self, ports):
        command = ports.host.latest_command()
        if not _link_ok(ports, self.reported_as, command):
            return self

        # 이번 사이클에 Host가 부른 이름으로 보고한다. 직전 사이클의 이름을
        # 쓰면 Host가 APPROACH_BOX로 넘긴 첫 사이클이 CARRY로 보고돼, Host의
        # 상태 추적이 한 사이클씩 뒤처진다.
        if command is not None and command.state in (
                MissionState.CARRY, MissionState.APPROACH_BOX):
            self.reported_as = command.state
        ports.host.report(Report.STATE, self.reported_as)
        if command is None:
            return self

        # 라이다와 부하를 **매 사이클** 떠 둔다. INSERT 명령이 왔을 때
        # 비교할 직전 표본이 이미 있어야 왕복이 한 번 줄고, 주행 중에 뜬
        # 표본은 자연히 현재와 어긋나므로 "아직 안 멈췄다"가 그대로 드러난다.
        face = ports.lidar.basket_face()
        self.sample = (face, ports.arm.get_load())

        if command.state == MissionState.INSERT:
            return self._judge_insert(ports, command, face)

        if not _drive(ports, command, self.reported_as):
            return self
        if command.state in (MissionState.CARRY, MissionState.APPROACH_BOX):
            return BaselineCarryState(self.label, self.reported_as, self.sample)
        if command.state == MissionState.DONE:
            return BaselineDoneState()
        return self

    def _judge_insert(self, ports, command, face):
        """임무 4번 앞단 — 조건 판정 후 보고. 충족이면 INSERT로, 아니면 제자리.

        직전 사이클 표본과 비교하는 항목이 둘 있다(판독 안정성·부하 안정성).
        표본이 없으면 판정하지 않고 한 사이클 더 본다 — Host는 INSERT를
        계속 보내므로 다음 사이클에 자연히 채워진다."""
        ports.base.stop()
        gp = plan_for_label(self.label)
        load = self.sample[1]

        distance_change = load_change = None
        if self.previous is not None:
            previous_face, previous_load = self.previous
            if previous_face.ok and face.ok:
                distance_change = face.distance_m - previous_face.distance_m
            load_change = load - previous_load

        insert_inputs = pc.InsertInputs(
            estop_set=ports.estop.is_set(),
            base_stopped=_base_stopped(ports, command),
            gripper_load=load,
            face_ok=face.ok,
            face_distance_m=face.distance_m,
            face_yaw_error_rad=face.yaw_error_rad,
            face_reason=face.reason,
            profile=gp.profile if gp else None,
            face_point_count=face.point_count,
            face_lateral_offset_m=face.lateral_offset_m,
            face_lateral_known=face.lateral_known,
            distance_change_m=distance_change,
            load_change=load_change,
        )
        report = pc.check_insert(insert_inputs)
        if not report.ok:
            # 보정 요구를 같이 실어 보낸다. 남은 미충족이 Host가 고칠 수 있는
            # 것이 아니면(점 개수·안정성·부하) from_insert가 None을 준다 —
            # 지어낸 보정을 주면 Host가 엉뚱하게 움직인다.
            ports.host.report(Report.INSERT_BLOCKED, self.reported_as, report.detail,
                              corrections.from_insert(insert_inputs))
            return BaselineCarryState(self.label, self.reported_as, self.sample)
        ports.host.report(
            Report.INSERT_READY, self.reported_as,
            f"라이다 {face.distance_m:.3f}m yaw {face.yaw_error_rad:+.3f}rad "
            f"점 {face.point_count} 좌우 "
            + (f"{face.lateral_offset_m * 1000:+.0f}mm"
               if face.lateral_known else "창 안(중앙)"))
        return BaselineInsertState(self.label)


class BaselineInsertState(State):
    """투하 후 IDLE 복귀 (임무 4번 뒷단).

    바닥 파지 높이로 내려가지 않는다 — 실측 DROP 자세로 직접 전개한 뒤
    그리퍼를 연다. 활짝 열지 않고 물체가 빠져나올 만큼만 열며, 접기 **전에**
    닫는다(사용자 지시 2026-08-25).

    성공 판정은 **부하 변화**로 한다. 놓기 전후를 비교해 유의하게 줄었으면
    물체가 손을 떠난 것이다 — 2026-08-26 실기에서 0.0626 -> 0.0313이었다.
    이것으로 "바구니 안에 들어갔는가"까지는 알 수 없다. 그건 오버헤드로
    보는 Host의 판단이고, Pi는 자기가 아는 것만 보고한다."""

    name = MissionState.INSERT

    # 놓임으로 볼 부하 감소량. 2026-08-26 실기 감소폭이 0.0274였다.
    RELEASE_LOAD_DROP = 0.015

    def __init__(self, label):
        self.label = label

    def execute(self, ports):
        ports.host.report(Report.STATE, self.name)
        ports.base.stop()
        gp = plan_for_label(self.label)

        if not ports.arm.move_to_floor_pose(gp.profile, "drop"):
            ports.arm.hold_position()
            ports.host.report(Report.INSERT_FAILED, self.name, "투하 자세 실패")
            return BaselineCarryState(self.label)  # 표본은 버린다 — 팔이 움직였다

        before = ports.arm.get_load()
        ports.arm.set_gripper(gp.release_width_mm)
        after = ports.arm.get_load()
        released = after <= before - self.RELEASE_LOAD_DROP

        ports.arm.set_gripper(CLOSED_MM)
        folded = ports.arm.move_to_floor_pose(gp.profile, "idle")

        if released:
            ports.host.report(Report.INSERT_DONE, self.name,
                              f"{self.label} 부하 {before:.4f} -> {after:.4f}")
        else:
            # 놓이지 않았는데 IDLE로 접으면 물체를 문 채 라이다를 가린다.
            # 그래도 접기는 한다 — 팔을 전개한 채 두는 편이 더 위험하다.
            ports.host.report(Report.INSERT_FAILED, self.name,
                              f"부하가 안 줄었다 ({before:.4f} -> {after:.4f})")

        ports.host.report(Report.IDLE_DONE, MissionState.IDLE,
                          "복귀 완료" if folded else "IDLE 복귀 실패")
        return BaselineIdleState()


class BaselineEstopState(State):
    """E-STOP. 정지하고 팔을 붙잡는다 — 파지물이 떨어지지 않도록."""

    name = MissionState.ESTOP

    def execute(self, ports):
        ports.base.stop()
        ports.arm.hold_position()
        ports.host.report(Report.STATE, self.name)
        return None


class BaselineMission:
    """`MissionTask`와 같은 제너레이터 구동 방식."""

    def __init__(self, ports):
        self.ports = ports

    def run(self):
        state = BaselineIdleState()
        while state is not None:
            if self.ports.estop.is_set():
                state = BaselineEstopState()
            yield state
            state = state.execute(self.ports)
