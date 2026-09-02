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
    GRIPPER_GRASP_MIN_MM,
    GRIPPER_MAX_SAFE_OPEN_MM,
    HorizontalGraspPlan,
    _close_width,
    _release_width,
)
from domain.task.base_liveness import LivenessLatch
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

# rook만 파지 폭을 GRIPPER_GRASP_MIN_MM(2026-08-25 knight 실측 포화점,
# floor_grasp_policy 주석 참고)까지 강제로 더 좁힌다 (2026-09-02 실기 지시).
#
# _close_width(24.5) = max(7.0, 24.5-15.0) = **9.5** — 이미 하한(7.0)보다
# 위라 08-25에 queen/knight을 7.0으로 내린 조정(9.0 -> 7.0)의 혜택을 rook은
# 못 받았다. 09-02 실기에서 8회 연속 "들어 올리지 못함"이 났고, 그때 부하
# (0.0235 x3 / 0.0274 / 0.0469)가 knight 실측 스윕의 "9.0mm 명령 -> 0.0235"
# 대와 같은 대역이었다 — 09-01 이전까지는 헐겁지 않았다는 사용자 관찰과
# 별개로, 이 폭 자체가 애초에 그 헐거운 대역에 있었다는 뜻이다.
#
# GRIPPER_SQUEEZE_MM(공용 상수)을 올리면 box/star/soccer(이미 검증된 25.0/
# 30.0/31.0)까지 같이 좁아진다 — 그건 요청받지 않았으니 rook 하나만 덮어쓴다.
_CLOSE_WIDTH_OVERRIDE_MM = {
    "rook": GRIPPER_GRASP_MIN_MM,
}

_PROFILE_BY_LABEL = {
    label: HorizontalGraspPlan(
        profile, GRIPPER_MAX_SAFE_OPEN_MM,
        _CLOSE_WIDTH_OVERRIDE_MM.get(label, _close_width(width_mm)),
        _release_width(width_mm))
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
    # 구동계 생존 판정의 래치. 워치독과 같은 이유로 여기 한 곳에 둔다 —
    # 상태 객체는 전이마다 새로 만들어지므로 상태를 들고 있을 수 없다.
    base_liveness: LivenessLatch = field(default_factory=LivenessLatch)


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


def _report_base_liveness(ports, state_name) -> None:
    """구동계가 명령을 받아 갈 상태인지 보고한다 (2026-08-28 정지 실패 사고).

    상태가 **바뀔 때만** 나간다(발생 1회, 복구 1회). 매 사이클 부르는 이유는
    이 신호가 가장 필요한 순간이 정지를 지시하는 순간이기 때문이다 — 그때
    조용하면 Host는 차가 섰다고 믿는다.

    `liveness()`가 없는 어댑터(테스트 더블)는 그냥 지나간다. 모르는 것과
    고장난 것은 다르고, 모를 때 경보를 울리면 아무도 경보를 안 보게 된다."""
    probe = getattr(ports.base, "liveness", None)
    if probe is None:
        return
    message = ports.base_liveness.observe(probe())
    if message is not None:
        ports.host.report(Report.BASE_UNRESPONSIVE, state_name, message)


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


def _load_holds(ports) -> bool:
    """부하가 threshold를 넘기는가 — 못 넘기면 한 번 더 재서 확인한다.

    LOAD_THRESHOLD 자체가 "빈손 실측 11/256"과 "파지 실측 13/256"의
    중점이다(baseline_constants 주석) — 두 계단 사이 틈이 코드 값 하나
    (약 0.0039)뿐이라, 방금 막 멈춘 팔의 잔진동 하나로도 판독이 계단
    하나만큼 튈 수 있다. 09-02 10:41 실기가 그 예다: 그리퍼를 막 닫고
    잰 값(13/256 대역, 0.0508)은 정상적으로 threshold를 넘겼는데, 곧바로
    이어진 midpoint 이동 직후 다시 잰 값(11/256 대역, 0.0430)만 그 틈
    하나만큼 밑돌아 "들어 올리지 못함"으로 실패 처리됐다 — 사용자가 직접
    옆에서 파지 성공을 확인한 자리였다.

    한 번 더 재서 그중 하나라도 넘기면 쥐고 있는 것으로 본다 — 진짜로
    놓쳤다면 재확인에서도 낮게 나온다."""
    if ports.arm.get_load() >= bc.LOAD_THRESHOLD:
        return True
    return ports.arm.get_load() >= bc.LOAD_THRESHOLD


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
        if command.state == MissionState.GRASP_FORCE:
            return self._judge_grasp(ports, command, force=True)

        if not _drive(ports, command, self.name):
            return self
        if command.state == MissionState.IDLE:
            return BaselineIdleState()
        if command.state == MissionState.DONE:
            return BaselineDoneState()
        return self

    def _judge_grasp(self, ports, command, force: bool = False):
        """임무 2번 — 조건 판정 후 보고. 충족이면 GRASP로, 아니면 제자리.

        판정은 두 겹이다. 먼저 기본 전제(정지·식별, 2026-09-01 사용자 지시로
        E-STOP·빈 그리퍼·교시 자세 확인을 뺐다 — preconditions.check_grasp
        문서 참고)를 보고, 통과하면 **물체가 턱이 쓸고 갈 영역 안에 있는지**를
        본다. `force`
        는 이 중 두 번째 겹(정렬 창)만 건너뛴다 — 첫 겹(기본 전제)은
        force 여도 그대로 지킨다(2026-08-31, MissionState.GRASP_FORCE 참고).

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
        inputs = pc.GraspInputs(
            base_stopped=_base_stopped(ports, command),
            detected_label=label,
        )
        report = pc.check_grasp(inputs)
        if not report.ok:
            # 보정을 같이 실어 보낸다. 안 보내면 Host가 이 실패를 고칠 수
            # 없는 것으로 읽고 기물을 포기한다 — 2026-08-28 run6이 그랬다.
            # force 는 이 전제를 건너뛰지 않는다 — 아직 안 멈춘 상태에서는
            # 강제로도 안 내려간다.
            ports.host.report(Report.GRASP_BLOCKED, self.name, report.detail,
                              corrections.from_grasp_precondition(inputs))
            return self

        return self._judge_alignment(ports, observation, label, force=force)

    def _judge_alignment(self, ports, observation, label, force: bool = False):
        """좌우·전후 정렬 판정 (사용자 지시 2026-08-26).

        영역 안이면 내려가고, 영역 밖이면 Host에 다시 세워 달라고 한다.

        ⚠️ 2026-09-01까지는 영역 안인데 가운데가 아니면 Pi가 servo 1로
        미세 보정한 뒤 다시 봤다(PI_CENTER). 사용자 지시로 그 경로를
        없앴다 — 실기에서 servo 1이 첫 보정 때 반대 방향으로 도는 사례가
        나왔고, 그 보정각이 offset_base_yaw 의 ±15도(교시 정면 기준) 예산을
        갉아먹어 다음 보정이 "servo 1이 거부했다"로 막히는 일이 반복됐다
        (2026-08-28 run1/run6도 같은 계열 — test_grasp_centering_loop.py
        참고). 이제는 턱 폭 안이면 그대로 READY다 — grasp_alignment 모듈
        docstring의 설계 원칙(평행 턱의 자기정렬 효과)에 맡긴다.

        `force=True` 면 HOST_CORRECTION(영역 밖) 이라도 READY 처럼 내려간다
        — **UNKNOWN(뎁스캠이 아예 못 잰 경우)은 건너뛰지 않는다.** Host 가
        재정렬을 충분히 반복했다는 건 매번 유효한 관측이 있었다는 뜻이라
        HOST_CORRECTION 만 대상이다. 어디 있는지조차 모르는 상태를 강제로
        내려보내는 것과는 다르다."""
        verdict = ga.judge(observation, object_width_mm(label))

        if verdict.action == ga.READY or (force and verdict.action == ga.HOST_CORRECTION):
            # creep_m 자체는 이제 미는 양을 정하지 않는다(2026-09-02, 아래
            # BaselineGraspState.execute 참고) — 여기서는 "정면에서 유효한
            # 관측이 있었는가"만 본다. None이면 물체가 이미 턱 선 안쪽이라
            # 전진 자체가 필요 없다는 뜻이거나 관측 실패다.
            creep_m = ga.creep_distance_m(observation)
            reason = (verdict.reason if verdict.action == ga.READY
                      else f"Host 지시로 강제 진행 — {verdict.reason}")
            detail = (f"{label} {reason} · 전진 "
                      f"{bc.GRASP_CREEP_OPEN_LOOP_SEC:.1f}s@"
                      f"{bc.GRASP_CREEP_OPEN_LOOP_SPEED_MPS:.2f}m/s"
                      if creep_m is not None else f"{label} {reason}")
            ports.host.report(Report.GRASP_READY, self.name, detail)
            return BaselineGraspState(label, creep_m, self.retries)

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
        #
        # 거리를 **팔을 내리기 전에** 확인한다. 모르는 채로 내려가 봐야 그
        # 자리에서 실패하고 팔만 바닥에 남는다.
        if self.creep_m is None:
            return self._failed(ports, "전진 거리를 모른다 — 관측 실패")

        # 정면을 볼 수 있는 마지막 순간이다 — grasp 자세로 내려가면 팔이
        # 뎁스 카메라를 가린다(tools/demo_rook_run.py 2단계와 같은 이유).
        ports.perception.remember_target(self.label)

        if not ports.arm.move_to_floor_pose(gp.profile, "safe"):
            return self._failed(ports, "safe 자세 실패")
        # 내려가기 전에 연다 — 닫힌 손가락이 물체가 있는 공간을 통과해
        # 내려가면 물체를 밀어낸다(사용자 지시 2026-08-24).
        ports.arm.set_gripper(gp.preopen_width_mm)
        if not ports.arm.move_to_floor_pose(gp.profile, "grasp"):
            return self._failed(ports, "grasp 자세 실패")

        # ⚠️ 전진은 **팔이 내려가 그리퍼가 열린 뒤**다 (사용자 지시 2026-08-24,
        # 재확인 2026-08-29). 이 전진의 목적은 "물체 가까이 가는 것"이 아니라
        # **물체를 벌어진 턱 사이로 밀어 넣는 것**이고, 그래야 평행 턱의 넓은
        # 목이 좌우 자기정렬 효과를 낸다(grasp_alignment 모듈 docstring).
        #
        # 2026-08-29까지 이 호출이 `safe` 앞에 있었다 — 차체가 먼저 가고 팔이
        # 나중에 내려오는 순서라, 밀어 넣는 것이 아니라 물체 위로 내려가
        # 감싸는 동작이었고 자기정렬 효과가 없었다. 최초 커밋(241003a) 이후
        # 아무도 안 건드린 자리인데, 실기로 검증된 tools/demo_rook_run.py 는
        # 처음부터 이 순서였다(2단계 팔 내리기 -> 3단계 미세 전진).
        #
        # ⚠️ 이 구간에서는 **회전이 절대 금지**다. 그리퍼가 바닥에서 2.6cm
        # 위에 열린 채 떠 있어서, 제자리 회전은 그것을 바닥과 물체를 가로질러
        # 옆으로 쓴다. `creep_forward_timed` 도 직진만 내므로 계약상
        # 지켜진다 — 여기에 회전을 섞는 구현으로 바꾸면 안 된다
        # (demo_rook_run.py 의 CREEP_KEYMAP 이 회전 키를 일부러 뺀 것과
        # 같은 이유).
        #
        # 2026-09-02: 관측 거리(self.creep_m, ga.creep_distance_m)로 미는
        # 양을 계산하던 방식을 버렸다 — 300→500mm 상한, +300mm 보너스까지
        # 조정해도 실기에서 16~70mm 수준의 미세 전진만 나왔고(원인은 이
        # 계산 자체가 아니라 배포 지연이었던 것으로 나중에 드러났지만),
        # 사용자가 신뢰성이 불투명한 관측 기반 계산 대신 결정론적인
        # 시간·속도 개방루프를 지시했다("거리 단위가 아니라 1.5초간 0.1의
        # 속도로 전진"). `self.creep_m is None` 게이트(위)는 그대로 둔다 —
        # 그건 "전진량이 얼마인가"가 아니라 "물체가 애초에 유효하게 관측
        # 됐는가"를 보는 것이라 여기와 무관하다.
        if not ports.base.creep_forward_timed(bc.GRASP_CREEP_OPEN_LOOP_SPEED_MPS,
                                              bc.GRASP_CREEP_OPEN_LOOP_SEC):
            return self._failed(ports, "미세 전진 실패")

        ports.arm.set_gripper(gp.close_width_mm)
        load = ports.arm.get_load()
        lifted = load >= bc.LOAD_THRESHOLD and ports.arm.move_to_floor_pose(
            gp.profile, "midpoint")
        # 2026-09-02 10:41 실기: 여기서 한 번만 재고 threshold를 못 넘기면
        # 바로 실패 처리했다 — _load_holds() 주석 참고.
        held = lifted and _load_holds(ports)
        cleared = held and ports.arm.move_to_floor_pose(gp.profile, "safe")
        if not cleared:
            return self._failed(ports, f"들어 올리지 못함 (부하 {load:.4f})")

        if not ports.arm.move_to_floor_pose(gp.profile, "carry"):
            return self._failed(ports, "CARRY 전환 실패")

        # 2026-08-26 ~ 2026-09-01까지는 성공 판정에 **독립적인 두 신호가
        # 모두** 있어야 했다(AND). 부하는 "무언가를 쥐고 있다"를, 뎁스
        # 카메라는 "있던 물체가 그 자리에서 사라졌다"를 말한다는 전제였다.
        #
        # ⚠️ 2026-09-01 실기: CARRY 자세에서는 팔·기물이 카메라 프레임
        # 밖에 있는 게 맞다(floor_grasp_profiles.CARRY_RAW 로 확인된 설계
        # 그대로) — 그런데도 confirm_grasp() 가 "그대로 있다"를 반환했다.
        # 즉 눈앞에 없는 rook 을 있다고 오검출한 것이다(뎁스 카메라 쪽의
        # 오탐). 그 사이 부하(0.0547)는 LOAD_THRESHOLD 를 정상적으로
        # 넘겼고, 사용자가 직접 파지 성공을 눈으로 확인했다. 뎁스 오탐
        # 하나가 정상적인 부하 신호를 막는 것을 사용자 지시로 없앤다 —
        # AND 를 **OR** 로 바꾼다. 부하가 낮고 뎁스도 "그대로 있다"고
        # 하는, **둘 다 실패를 가리키는 경우만** 진짜 실패로 본다.
        carried = ports.arm.get_load()
        vanished = ports.perception.confirm_grasp()
        if carried < bc.LOAD_THRESHOLD and not vanished:
            return self._failed(ports, f"부하도 낮고 물체도 그대로다 (부하 {carried:.4f})")

        detail = f"{self.label} 부하 {carried:.4f}"
        detail += " · 목표 사라짐 확인" if vanished else " · 부하로 확인(뎁스 오탐 무시)"
        ports.host.report(Report.GRASP_DONE, MissionState.CARRY, detail)
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

        # ⚠️ 팔을 바닥에 둔 채 Host 에 돌려주면 안 된다 (2026-08-29).
        #
        # 이 함수는 APPROACH 로 돌아가고, 거기서 Host 는 곧바로 주행을
        # 지시한다. 그런데 파지 경로의 실패는 대부분 팔이 **이미 내려간 뒤**
        # 난다(전진 실패·닫기 실패·들어올리기 실패). 그 상태로 차가 움직이면
        # 바닥 2.6cm 위에 열려 있는 그리퍼가 바닥과 물체를 가로질러 쓸린다 —
        # "팔이 바닥 높이에서 옆으로 쓸리는 움직임은 절대 안 된다"가 이
        # 프로젝트의 확립된 안전 규칙이다(사용자 지시 2026-08-24).
        #
        # 실기로 검증된 도구들은 전부 실패 시 recover_idle 로 팔을 올린다
        # (tools/grasp_test_console.recover_to_idle). FSM 만 안 하고 있었다.
        #
        # "idle" 이 아니라 "recover_idle" 인 이유: 이동이 실패하면 팔은 정의상
        # 등록된 자세들 **사이**에 멈춰 서는데, 그 상태가 "idle" 의 시작 자세
        # 게이트에 걸려 거부된다 — 정작 복구가 필요한 순간에만 복구가 막힌다.
        #
        # 복구가 실패해도 원래 실패를 덮지 않는다. 팔을 붙잡아 두고, 무슨 일이
        # 있었는지 둘 다 Host 에 보낸다 — 여기서 예외를 올리면 진짜 원인이
        # 로그에서 묻힌다.
        gp = plan_for_label(self.label)
        recovered = False
        if gp is not None:
            recovered = ports.arm.move_to_floor_pose(gp.profile, "recover_idle")
        if not recovered:
            ports.arm.hold_position()

        note = "" if recovered else " · ⚠️ 팔이 중간 자세에 멈춰 있다(수동 정렬 필요)"
        ports.host.report(Report.GRASP_FAILED, MissionState.APPROACH,
                          f"{attempt}번째 시도 실패 — {detail}{note}")
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

        if command.state == MissionState.APPROACH_BOX and face.ok:
            # 09-02 실기(2건): NUDGE_BOX가 Host 계획 거리(want_m)를 다 밀
            # 때까지 라이다를 안 보다가, PLACE에 들어가서야 확인해서는 늦었다
            # — ArUco 데드레커닝이 틀리면 그사이 이미 바구니에 닿는다. 접근
            # 중에도 매 사이클 확인해서, 이미 너무 가까우면 Host 계획을
            # 무시하고 더 밀지 않는다(바퀴를 실제로 돌리는 쪽이 최종
            # 안전판이라는 이 파일의 기존 원칙 그대로 — encode()/motion.py의
            # 속도 클램프와 같은 계층).
            too_close = corrections.retreat_if_too_close(face.distance_m)
            if too_close is not None:
                ports.base.stop()
                ports.host.report(
                    Report.INSERT_BLOCKED, self.reported_as,
                    f"라이다 판독이 하한보다 가깝다 ({face.distance_m:.3f}m < "
                    f"{bc.BASKET_MIN_LIDAR_M:.3f}m) — 접근 중 감지, 더 밀지 않는다",
                    too_close)
                return BaselineCarryState(self.label, self.reported_as, self.sample)
            if corrections.within_stop_window(face.distance_m):
                # 이미 알맞은 거리다 — 계획한 거리를 마저 채우면 창을 넘겨
                # 버린다. 요·좌우·안정성·부하는 아직 안 본다 — PLACE에서
                # check_insert가 평소대로 마저 본다.
                ports.base.stop()
                ports.host.report(
                    Report.APPROACH_BOX_READY, self.reported_as,
                    f"라이다 {face.distance_m:.3f}m — 목표창 안, 그만 밀어도 된다")
                return BaselineCarryState(self.label, self.reported_as, self.sample)

        if not _drive(ports, command, self.reported_as):
            return self
        if command.state in (MissionState.CARRY, MissionState.APPROACH_BOX):
            return BaselineCarryState(self.label, self.reported_as, self.sample)
        if command.state == MissionState.DONE:
            return BaselineDoneState()
        if command.state == MissionState.IDLE:
            # 2026-09-02 실기로 발견: 여기만 IDLE을 안 받고 있었다 —
            # BaselineIdleState/BaselineApproachState는 둘 다 IDLE을 받아
            # IdleState로 돌아가는데, CarryState만 빠져 있었다. Host가
            # 미션을 중간에 멈출 때(run_mission.py 종료 처리, 사용자가
            # Enter/q로 끌 때) 보내는 것은 DONE이 아니라 "stop"+
            # SEARCH_TARGET(-> 여기서는 IDLE)이다. 그 순간 Pi가 CARRY나
            # APPROACH_BOX(바구니 접근) 어딘가에 있었으면, 이 분기가 없어서
            # `return self`로 떨어져 그 자리에 그대로 갇혔다 — 다음에 새
            # 미션을 시작해도 Host가 APPROACH/GRASP를 보내는데 Pi는 여전히
            # CarryState라 못 알아듣고(APPROACH_BOX만 받는다) GRASP가
            # 영원히 대기하는 락업이 됐다(10:06 실기).
            return BaselineIdleState()
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
            # 상태와 무관하게 매 사이클 본다 — 이 신호가 가장 필요한 순간이
            # 정지를 지시하는 순간이라, 특정 상태에만 걸면 놓친다.
            _report_base_liveness(self.ports, state.name)
            yield state
            state = state.execute(self.ports)
