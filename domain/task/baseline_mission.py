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

import math
import time
from dataclasses import dataclass, field

from domain.ports.baseline_ports import MissionState, Report
from domain.task import baseline_constants as bc
from domain.task import corrections
from domain.task import grasp_alignment as ga
from domain.task import preconditions as pc
from domain.task.floor_grasp_policy import (
    HorizontalGraspPlan,
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
# 폭은 더 이상 여기서 안 정한다 — 파지 폭 정책은 2026-09-07 에 들어냈고
# (floor_grasp_policy 의 그 주석), 남은 것은 투하 때 여는 폭뿐이다.
# 물체 폭은 정렬 판정(grasp_alignment.judge)이 아직 쓴다.
_OBJECT_WIDTH_MM = {
    "queen": ("chess_queen", 17.0),
    "knight": ("chess_knight", 22.0),
    "rook": ("chess_rook", 24.5),
    "box": ("cube", 40.0),
    "star": ("star_column", 45.0),
    "soccer": ("soccer_polyhedron", 46.0),
}

_PROFILE_BY_LABEL = {
    label: HorizontalGraspPlan(profile, _release_width(width_mm))
    for label, (profile, width_mm) in _OBJECT_WIDTH_MM.items()
}


def plan_for_label(label):
    """raw 라벨에 맞는 교시 파지 계획. 모르는 라벨이면 **None** — 모르면 실패."""
    return _PROFILE_BY_LABEL.get(label)


# MissionState.DEBUG_FORCE_CARRY로 CARRY에 바로 들어갈 때 쓸 라벨(2026-09-05).
# 실제 파지가 없어 Host/Pi 어느 쪽도 진짜 라벨을 모르므로 하나 고정해 둔다 —
# INSERT의 drop 자세/그리퍼 개방폭이 이 라벨의 교시 계획을 그대로 쓴다.
# 다른 물체로 시험하려면 이 상수만 바꾸면 된다.
DEBUG_FORCE_CARRY_LABEL = "rook"


def object_width_mm(label):
    """그 라벨 물체의 실측 폭(mm). 모르는 라벨이면 **None**.

    턱이 쓸고 갈 영역의 좌우 허용치를 낼 때 쓴다 — 넓은 물체일수록 중심이
    덜 벗어나야 턱에 스치지 않고 들어온다."""
    entry = _OBJECT_WIDTH_MM.get(label)
    return entry[1] if entry else None


class ArmParkLatch:
    """팔이 접힌 상태인가. 아니면 **주행을 막는다.**

    2026-09-06 실기: VLA 파지가 실패해 팔이 미등록 자세에 남았는데 Host 가
    RETURN_HOME 으로 넘어가 팔을 뻗은 채 주행했다. Host 는 팔 상태를 모르고
    알 이유도 없다(역할 분담) — 아는 쪽인 Pi 가 막는다.

    상태 객체는 전이마다 새로 만들어지므로 포트에 둔다. LinkWatchdog 과 같다.
    """

    def __init__(self) -> None:
        self.parked = True
        self.reason = ""

    def mark_parked(self) -> None:
        self.parked = True
        self.reason = ""

    def mark_unparked(self, reason: str) -> None:
        self.parked = False
        self.reason = reason


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

    # 팔이 접혀 있는가. 안 접혔으면 _drive 가 주행을 거부한다.
    arm_parked: ArmParkLatch = field(default_factory=ArmParkLatch)

    # ── 파지 포트 ──────────────────────────────────────────────────────────
    #
    # 파지는 **정책이 통째로 한다.** 예전에는 grasp_backend 로 classic/vla 를
    # 골랐지만, classic 쪽은 팀원 브랜치(kica927)에서 온 임시 구현이었고
    # 2026-09-07 사용자 지시로 들어냈다 — 파지는 사용자가 직접 만들 부분이다.
    #
    # 그래서 이 포트는 **필수**다. None 이면 GRASP 가 바로 실패한다.
    vla: object = None

    # ── 정책만 돌려 본다 (2026-09-07 사용자 지시) ─────────────────────────
    #
    # "파지 시퀀스에 팀원의 하드코딩 부분이 계속 들어가는 거 같은데 우선 잠시
    # 배제해줘 — 그냥 vla 로만 동작하는 것을 확인하고 싶어."
    #
    # 그때 정책이 **안 시킨 팔 동작**이 셋 남아 있었다: creep 거리 관문,
    # remember_target(뎁스 관측), CARRY 전환. 앞의 둘은 classic 파지 시퀀스와
    # 함께 아예 지웠으므로(2026-09-07), 이 스위치에 남은 것은 CARRY 전환
    # 하나뿐이다 — 정책이 끝낸 자세에서 손목만 올리는 별도 동작이다.
    #
    # True 면 그 전환을 건너뛴다. ⚠️ **켜면 운반·투하가 깨진다** — 물체를 문 채
    # IDLE 에 있으면 그리퍼가 라이다 정면을 79% 가려 바구니를 못 본다
    # (2026-08-26 실측, floor_grasp_profiles.CARRY_RAW 주석). 파지 하나만
    # 눈으로 확인하려는 진단용이다.
    #
    # 시작 자세(fold_to_cradle)는 **안 건드린다** — 학습 회차 118개의 첫
    # 관측이 전부 IDLE 크래들이었다. 그것까지 빼면 정책이 분포 밖에서
    # 시작하므로, 이건 팀원 하드코딩이 아니라 정책을 돌리기 위한 조건이다.
    vla_only: bool = False

    # ── 뎁스 관문 ──────────────────────────────────────────────────────────
    #
    # 기본은 켜짐이다. 끄면 **뎁스 카메라를 한 번도 안 본다** — 물체 식별,
    # 정렬 판정, 파지 성공의 두 번째 신호가 전부 빠진다.
    #
    # 왜 끄는 선택지가 있는가: 주행이 탑뷰 ArUco 로 물체 앞에 세워 주고 파지를
    # VLA 가 하는 구성에서는 Pi 의 뎁스캠이 경로에 없다. 그런데 이 FSM 은 뎁스
    # 관측이 없으면 GRASP 로 넘어가지 못한다(metric_ok 가 아니면 UNKNOWN).
    #
    # ⚠️ 끄면 잃는 것이 분명하다. 성공 판정이 **서보 부하 하나**로 줄어서,
    # 물체 모서리를 살짝 물었거나 턱끼리 문 경우도 통과한다. 사람이 눈으로
    # 지켜보는 시연에서나 감수할 만한 거래다.
    use_depth_gate: bool = True
    # 뎁스 관문을 껐을 때 쓸 라벨. HostCommand 에는 라벨이 없고(상태와 속도뿐)
    # 뎁스캠 식별이 빠지므로 어딘가에서 와야 한다. ACT 는 이 문자열을 정책
    # 입력으로 받지 않는다 — 쓰이는 곳은 파지 프로파일 하나다.
    default_grasp_label: str = "queen"


# ── 공통 동작 ──────────────────────────────────────────────────────────────


def _drive(ports, command, state_name) -> bool:
    """Host 속도를 베이스에 낸다. 명령이 부적합하면 정지 + 보고 후 False.

    거부 사유를 그대로 Host에 돌려주는 이유: Pi가 추측해서 둘 중 하나를
    실행하면 Host는 자기가 무엇을 잘못 보냈는지 영영 모른다."""
    # ⚠️ 팔이 안 접혔으면 안 달린다. 2026-09-06 실기에서 VLA 파지 실패 뒤
    # 팔을 뻗은 채 RETURN_HOME 주행을 했다 — 부딪히면 팔이 부서지고 라이다
    # 시야도 가린다. Host 는 팔 상태를 모르므로(HostCommand 에 없다) 아는
    # 쪽인 Pi 가 막는다. 정지 명령은 통과시킨다 — 멈추는 것은 언제나 안전하다.
    latch = getattr(ports, "arm_parked", None)
    if latch is not None and not latch.parked:
        pre = resolve_motion(command)
        if pre.ok and not pre.motion.is_stop:
            ports.base.stop()
            ports.host.report(
                Report.REJECTED, state_name,
                f"팔이 안 접혀 주행을 막는다 — {latch.reason}. "
                "tools/align_to_idle.py 로 정렬한 뒤 다시 시도하십시오")
            return False

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
        if command.state == MissionState.DEBUG_FORCE_CARRY:
            # 테스트 전용 우회로 — MissionState.DEBUG_FORCE_CARRY 정의 참고.
            # 실제 파지 없이 CARRY 에 바로 들어간다.
            ports.host.report(Report.STATE, MissionState.CARRY,
                               "DEBUG_FORCE_CARRY — 실제 파지 아님, 시험 전용")
            return BaselineCarryState(DEBUG_FORCE_CARRY_LABEL)
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
        if not ports.use_depth_gate:
            # 뎁스캠을 안 본다. 식별도 정렬 판정도 건너뛰고 바로 GRASP 다 —
            # "물체가 팔 앞에 있다"는 판단을 탑뷰를 보는 Host 가 이미 했고,
            # 그 판단으로 GRASP 명령을 보낸 것이기 때문이다.
            #
            label = ports.default_grasp_label
            ports.host.report(
                Report.GRASP_READY, self.name,
                f"{label} 뎁스 관문 꺼짐 — 정렬 판정 없이 진행")
            return BaselineGraspState(label, self.retries)

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
            # ⚠️ 여기서 전진 거리(ga.creep_distance_m)를 내던 것을 지웠다
            # (2026-09-07). 그 값은 classic 파지 시퀀스의 미세 전진에만
            # 쓰였는데 그 시퀀스가 없어졌다 — 정책은 차체를 밀지 않고
            # 스스로 뻗는다(_grasp_vla 주석).
            reason = (verdict.reason if verdict.action == ga.READY
                      else f"Host 지시로 강제 진행 — {verdict.reason}")
            ports.host.report(Report.GRASP_READY, self.name,
                              f"{label} {reason}")
            return BaselineGraspState(label, self.retries)

        ports.host.report(Report.GRASP_BLOCKED, self.name, verdict.reason,
                          corrections.from_alignment(verdict))
        return self


#: 버스가 잠깐 나갔다 돌아오는 데 실기에서 4.5초쯤 걸렸다(2026-09-07).
#: 간격 x 횟수가 그보다 넉넉해야 한다. 도메인에서 유일하게 자는 자리라
#: 상수로 빼 뒀다 — 시험은 interval_s=0 으로 부른다.
#: 닫기 명령을 내린 뒤 턱이 자리를 잡을 때까지 기다리는 시간.
#: VLA_GRIPPER_SPEED_RAW(600 raw/s)로 최대 900 raw 를 움직여야 1.5초다.
GRIP_SETTLE_SEC = 1.5

# ── 파지와 CARRY 사이에 그리퍼 명령을 내지 않는다 ────────────────────────
#
# 여기 JUDGE_CLOSE_WIDTH_MM 이 있었다 — 성공 판정 직전에 "확실히 닫아라"로
# 보내던 폭이다. 판정을 통째로 들어내면서 읽는 곳이 없어졌고, 2026-09-08 에
# 상수도 지웠다. 다만 **왜 되살리면 안 되는지**는 남겨 둔다.
#
# 그 닫기가 오히려 쥐는 힘을 **풀고 있었다.** 위치제어에서 힘은
# P x (목표 - 현재) 이고, 판정용 닫기가 명령한 자리가 정책이 잡아 둔 자리보다
# 넓었다:
#
#     정책이 명령한 자리        1007      오차 183 raw
#     판정용 닫기가 명령한 자리  1106      오차  84 raw   <- 힘이 절반
#
# 그리고 그 직후가 주행이었다. 2026-09-07 에 "주행 자세에서 기물을
# 떨어뜨린다"고 본 것의 원인이 이쪽일 가능성이 크다 — 그날 CARRY 손목 각도를
# 고쳤지만, 사용자는 2026-09-08 에 "VLA 기반에서는 물체를 딱히 떨어뜨린 적이
# 없다"고 정정했다(그 사이에 판정 시퀀스가 사라졌다).
#
# 그래서 지금 계약은 이것이다 — **정책이 만든 쥔 상태를 아무도 건드리지
# 않는다.** 그리퍼로 가는 명령은 투하 때 release_until_open 이 처음이자
# 마지막이다. 시험이 이 계약을 지킨다
# (test_baseline_mission.test_파지부터_CARRY까지_그리퍼_명령이_없다).

#: 마커 정면 축과 그리퍼가 실제로 겨누는 방향의 **고정 각도 차이**(도).
#:
#: 마커가 servo 1 회전축 위에 있으므로(2026-09-08 실측) 이 둘은 같은 회전부에
#: 붙어 있고 각도 차이는 상수다. 탑뷰 θ 를 0 으로 만드는 것만으로는 마커가
#: 겨눠질 뿐이라, 그리퍼를 겨누려면 이 값을 더해야 한다.
#:
#: ⚠️ 아직 **안 쟀다.** 재려면 로봇 자세가 정확해야 하는데, 2026-09-08 진단에서
#: 두 탑뷰 카메라가 같은 마커를 158mm·yaw 4.4도 다르게 보고 있는 것이
#: 드러났다(각 카메라 안에서는 σ 0.3mm 로 안정적). 그 위에서 이 값을 1도
#: 정밀도로 잡는 것은 불가능하다.
#:
#: 그래서 0 으로 두고 실기에서 잡는다(사용자 지시: "실행했을 때 틀어지면
#: 그때 다시 수정하자"). 기물이 그리퍼 기준 **왼쪽**에 남으면 양수를 키운다.
VLA_PAN_TRIM_DEG = 0.0

RELEASE_RETRIES = 4
RELEASE_RETRY_SEC = 1.5


def release_until_open(ports, width_mm, retries=None, interval_s=None) -> bool:
    """그리퍼를 열고 **위치로 확인**한다. 안 열렸으면 다시 시도한다.

    ⚠️ `ArmDriver.set_gripper` 은 반환값이 없다(포트 계약) — 서보 통신이
    실패해도 호출한 쪽은 모른다. 2026-09-07 실기에서 그게 두 번 물렸다:

      파지 실패 뒤   정책이 servo 6 write 실패로 끝나고, 이어진 놓기도 실패해
                     **별을 문 채** 다음 기물로 갔다.
      투하 순간      팀원 보고 — "상자나 별을 정리상자에 넣는 순간 servo 6
                     오류로 그리퍼를 안 푼다".

    그래서 명령이 아니라 위치를 읽어 판정한다(bc.GRIPPER_RELEASED_MIN_RAW).
    읽기 실패(-1)는 문턱보다 작으므로 자연히 "안 열렸다"가 된다 — 모르는
    것을 열렸다고 치면 물건을 문 채 다음으로 간다."""
    # ⚠️ 기본값을 인자 자리에 박지 않는다 — 그러면 def 시점에 굳어서 시험이
    # 모듈 상수를 낮춰도 안 먹고, 실패 시늉 하나마다 4.5초를 실제로 잔다.
    retries = RELEASE_RETRIES if retries is None else retries
    interval_s = RELEASE_RETRY_SEC if interval_s is None else interval_s
    for attempt in range(1, int(retries) + 1):
        ports.arm.set_gripper(width_mm)
        if ports.arm.gripper_position_raw() >= bc.GRIPPER_RELEASED_MIN_RAW:
            return True
        if attempt < retries:
            time.sleep(interval_s)
    return False


class BaselineGraspState(State):
    """파지 수행 (임무 3번).

    실기로 검증된 순서를 그대로 따른다 — 벌리고, 내려가고, 물체를 턱 사이로
    밀어 넣고, 닫고, midpoint에서 부하를 다시 보고, safe를 거쳐 CARRY로 접는다.

    ⚠️ 마지막이 IDLE이 아니라 **CARRY**인 것이 중요하다. 물체를 문 채 IDLE로
    접으면 그리퍼가 라이다 정면을 79% 가려 바구니를 못 본다(2026-08-26 실측,
    floor_grasp_profiles.CARRY_RAW 주석)."""

    name = MissionState.GRASP

    def __init__(self, label, retries: int = 0):
        self.label = label
        self.retries = retries

    def execute(self, ports):
        ports.host.report(Report.STATE, self.name)
        gp = plan_for_label(self.label)
        ports.base.stop()

        # ── 파지는 정책이 통째로 한다 ──────────────────────────────────
        #
        # ⚠️ 2026-09-07 사용자 지시로 classic 파지 시퀀스를 **들어냈다**.
        # 그 코드는 팀원 브랜치(kica927)에서 온 임시 구현이었고, 파지는
        # 사용자가 직접 만들 부분이다 — 남겨 두면 어느 쪽이 도는지 계속
        # 헷갈린다.
        #
        # 지운 것: safe 자세 -> 벌리기 -> grasp 자세 -> 미세 전진 ->
        # 닫기 -> midpoint 들어올리기 -> safe 복귀. 그리고 그 앞의
        # creep 거리 관문도 같이 지웠다 — 그 시퀀스의 미세 전진에만 쓰던
        # 값이다.
        #
        # ⚠️ remember_target 은 **안 지웠다.** 저것은 파지 동작이 아니라
        # 뎁스 관문의 준비다 — 아래 성공 판정의 confirm_grasp() 가 여기서
        # 잡아 둔 기준 프레임과 비교한다. 지우면 use_depth_gate=true 구성이
        # 통째로 못 쓰게 된다(실기 기본 구성은 false 라 안 부른다).
        if ports.use_depth_gate:
            # 정면을 볼 수 있는 마지막 순간이다 — 정책이 팔을 내리면
            # 뎁스 카메라를 가린다(tools/demo_rook_run.py 2단계와 같은 이유).
            ports.perception.remember_target(self.label)

        if not self._grasp_vla(ports, gp):
            # ⚠️ 여기까지 오는 경우는 **팔을 못 움직였을 때뿐**이다
            # (VLA 포트 없음, 시작 자세 정렬 실패). 정책이 물체를 집었는지
            # 여부로는 실패를 만들지 않는다 — _grasp_vla 의 그 주석 참고.
            #
            # ── servo 6 만 죽었으면 상자로 간다 ──────────────────────────
            #
            # 사용자 지시(2026-09-08): "오히려 6번서보모터가 오류가 났을때
            # 상자(toy, chess)로 가게끔 하는 것이 좋을 거 같아."
            #
            # 맞는 판단이다. servo 6 이 죽은 채로 제자리에서 재시도해 봐야
            # 파지가 될 리가 없고, 그 사이 물체를 문 채로 있을 수도 있다.
            # 상자로 가면 셋 다 나아진다:
            #
            #   문 게 있으면   목적지 근처까지는 옮겨 놓는다
            #   시간이 번다    주행하는 몇 초가 곧 서보 회복 시간이다
            #                  (2026-09-07 실측: 버스가 약 4초 뒤 돌아왔다)
            #   투하가 재시도   INSERT 의 release_until_open 이 4회 x 1.5초
            #                  로 다시 열어 본다 — 여기서 살아날 수 있다
            #
            # ⚠️ 그래도 **팔은 움직일 수 있어야** 한다. CARRY 전환이 되면
            # 팔이 알려진 자세에 있다는 뜻이라 주행이 안전하다. 그것마저
            # 안 되면 진짜로 팔이 갇힌 것이니 예전대로 실패다.
            if ports.arm.gripper_position_raw() < 0:
                ports.host.report(
                    Report.STATE, self.name,
                    "servo 6 이 응답하지 않는다 — 제자리 재시도 대신 상자로 "
                    "간다(주행 중 회복을 노리고, 물었으면 목적지 근처까지 옮긴다)")
                if ports.arm.move_to_floor_pose(gp.profile, "carry"):
                    ports.host.report(
                        Report.GRASP_DONE, MissionState.CARRY,
                        f"{self.label} servo 6 고장으로 상자행 — 그리퍼 상태는 "
                        f"모른다(투하 직전에 다시 확인한다)")
                    return BaselineCarryState(self.label)
                ports.host.report(
                    Report.STATE, self.name,
                    "CARRY 전환도 실패했다 — 팔이 갇혔다")
            return self._failed(ports, "팔을 움직이지 못했다")

        # ⚠️ vla_only 면 CARRY 로 안 옮긴다 — 정책이 끝낸 자세 그대로 둔다
        # (BaselinePorts.vla_only 주석). 그 대가로 운반·투하가 깨진다.
        if ports.vla_only:
            ports.host.report(
                Report.STATE, self.name,
                "vla_only — CARRY 전환을 건너뛴다. 정책이 끝낸 자세 그대로다 "
                "(물체를 물었으면 라이다가 가려 바구니를 못 찾는다)")
        elif not ports.arm.move_to_floor_pose(gp.profile, "carry"):
            # ── 여기서 실패로 접지 않는다 ────────────────────────────────
            #
            # 사용자 지시(2026-09-08): "vla 동작 후에 아예 성공 실패를
            # 따지지 말고 바로 바구니쪽으로 가는 것으로 간단하게 수정해줘."
            #
            # 시연에서 본 "다시 파지 종료 동작으로 이어지던" 것이 정확히
            # 이 자리였다 — 로그에 `파지 실패 — CARRY 전환 실패` 로 찍히고
            # _failed 가 recover_idle 을 돌린 뒤 APPROACH 로 돌아갔다.
            #
            # 운반 자세를 못 잡은 것은 **파지의 성패와 무관**하다. 정책은
            # 이미 끝났고, 물었으면 문 채다. 되돌릴 이유가 없다.
            #
            # 대신 팔은 어떻게든 안전한 자세로 두려고 한 번 더 시도한다.
            # 그것마저 실패하면 arm_parked 가 주행을 거부하므로(그 래치의
            # 원래 역할) 팔을 끌고 다니는 일은 안 생긴다.
            ports.host.report(
                Report.STATE, self.name,
                "CARRY 자세를 못 잡았다 — 그래도 바구니로 간다(파지와 무관)")
            if not ports.arm.fold_to_cradle():
                ports.arm_parked.mark_unparked("CARRY·접기 둘 다 실패")
                ports.host.report(
                    Report.STATE, self.name,
                    "접기도 실패 — 팔이 알려진 자세에 없어 주행이 막힌다")

        # ── 파지 성공/실패 판정은 하지 않는다 ──────────────────────────
        #
        # ⚠️ 2026-09-07 사용자 지시로 **통째로 들어냈다.**
        #
        #   "파지 성공 실패 시퀀스 자체를 아예 없애줘. 실패하게 되면 어차피
        #    그 상태에서 다시 시작하게 될텐데 왜 굳이 실패 성공을 만들어
        #    놓은건지 이해가 안돼"
        #
        # 맞는 지적이다. 실패로 판정해서 얻는 것이 없었다 — Host 는 실패를
        # 받으면 상태를 통째로 리셋하고 SEARCH_TARGET 부터 다시 하는데,
        # 그것은 파지가 성공하지 못했을 때 어차피 일어나는 일이다. 판정은
        # **틀릴 기회만** 만들었고, 실제로 계속 틀렸다:
        #
        #   문턱이 옛 하한 기준이라 1107 을 실패로 읽음      (09-07 낮)
        #   루프 미완료(run_grasp=False)를 못 잡았다로 읽음  (09-07 저녁)
        #   읽기 실패(-1)를 빈손으로 읽음                     (09-07 밤)
        #
        # 세 번 다 **성공한 파지를 버렸고**, 버리는 과정에서 물체를 놓으려다
        # 죽은 서보에 4회씩 재시도하며 시간을 태웠다.
        #
        # 그래서 지운다. 정책이 돌고 나면 CARRY 로 간다. 정말 못 집었으면
        # 빈 그리퍼로 바구니까지 갔다가 아무것도 안 놓고 IDLE 로 돌아오고,
        # 그 다음 사이클이 다시 집으러 간다 — 잃는 것은 한 바퀴이고, 얻는
        # 것은 "성공한 파지를 버리지 않는다"이다.
        #
        # 남겨 둔 신호가 하나 있다: 투하 **직전**의 재확인(BaselineInsertState).
        # 그것은 파지 판정이 아니라 "운반 도중에 흘렸는가"이고, 헛투하를
        # 막는 자리라 성격이 다르다.
        held_raw = ports.arm.gripper_position_raw()
        carried = ports.arm.get_load()
        ports.host.report(
            Report.GRASP_DONE, MissionState.CARRY,
            f"{self.label} 파지 완료 — 그리퍼 {held_raw} · 부하 {carried:.4f} "
            f"(기록용, 판정에는 안 씀)")
        return BaselineCarryState(self.label)


    def _grasp_vla(self, ports, gp) -> bool:
        """정책이 파지를 대신한다. 성공했다고 **주장**하면 True.

        진짜 성공 판정은 여기서 하지 않는다 — `execute` 꼬리가 그리퍼 위치
        (와 구성에 따라 뎁스)로 판정한다. 이 함수는 "정책 루프가 끝까지
        돌았는가"만 본다.

        ⚠️ `creep_forward` 로 차체를 밀지 않는다. 학습 때 차체는
        **정지해 있었고** 정책이 스스로 뻗어서 물체를 집었다. 여기서 차체를
        밀면 정책이 본 적 없는 조건이 된다.

        ⚠️ 시작 자세를 IDLE 로 맞춘다. 학습 회차 118개의 첫 관측이 전부
        IDLE 크래들이었다(2026-09-04 측정: pan -4.75, lift -103.21,
        elbow 95.93, wrist 73.33 / 교시 IDLE 과 2도 이내). 다른 자세에서
        시작하면 분포 밖이다.
        """
        if ports.vla is None:
            ports.host.report(Report.GRASP_BLOCKED, self.name,
                              "VLA 포트가 없다 — 파지는 정책이 한다")
            return False
        # ⚠️ move_to_floor_pose(idle) 를 쓰면 안 된다. 그 경로는 **등록된
        # 자세에서 출발할 때만** 허용된다(safe/drop/carry). 정책은 학습한 대로
        # 자유롭게 뻗으므로 끝 자세가 등록 자세와 다르고, 그러면 다음 시도의
        # 시작 자세조차 못 잡아 팔이 갇힌다 — 2026-09-06 실기에서 15회가 전부
        # "등록된 자세 어디에도 가깝지 않다(safe 로 630, 허용 500)"로 죽었다.
        #
        # fold_to_cradle 은 _auto_align_to_idle 에 위임하고, 그 함수는 어디서
        # 시작하든 안전한 경로를 고른다(바닥 높이면 safe 를 경유해 들어 올린다).
        # 자세 게이트가 없다.
        if not ports.arm.fold_to_cradle():
            # ── 서보가 죽었는데 초당 열 번씩 다시 덤비지 않는다 ──────────
            #
            # 2026-09-07 실기: servo 6 이 과전류 보호로 버스에서 떨어지자
            # 이 fold 가 매번 실패했고, Host 는 실패를 받자마자 상태를
            # 리셋해 곧바로 GRASP 를 다시 지시했다. 0.35초 주기로 돌아
            # **37번째 시도**까지 갔다 — 사용자: "파지실패라는 말이 나오는데
            # 그냥 그런걸 없애줘".
            #
            # 재시도 자체가 틀린 게 아니라 **간격이 없는 것**이 틀렸다.
            # 같은 로그에서 버스는 4초쯤 뒤에 스스로 돌아왔다. 그 사이를
            # 쉬지 않고 두드리면 회복을 돕지도 않으면서 로그만 덮는다.
            #
            # 하드웨어가 죽은 것인지 자세를 못 잡은 것인지는 그리퍼 위치를
            # 읽어 보면 갈린다 — 읽기 실패(-1)면 버스가 나간 것이다.
            if ports.arm.gripper_position_raw() < 0:
                ports.host.report(
                    Report.GRASP_BLOCKED, self.name,
                    f"servo 6 이 응답하지 않는다 — 버스가 돌아올 때까지 "
                    f"{self.HARDWARE_FAULT_DWELL_SEC:.0f}초 기다린다 "
                    f"(과전류 보호로 떨어졌을 때 실기 회복 시간 약 4초)")
                time.sleep(self.HARDWARE_FAULT_DWELL_SEC)
            else:
                ports.host.report(Report.GRASP_BLOCKED, self.name,
                                  "VLA 시작 자세(IDLE) 실패")
            return False
        # ── 좌우 조준: 탑뷰 방위각을 servo 1 로 지운다 ──────────────────
        #
        # 2026-09-07 에 부호를 몰라 통째로 뺐다가, 2026-09-08 에 **실측으로
        # 확정하고** 되살렸다.
        #
        # ── 마커가 팔에 붙어 있다 (2026-09-08 실측) ──
        #
        # servo 1 을 -12~+12도 스윕하며 탑뷰 마커를 읽었다:
        #
        #     yaw = -0.975 * servo1 + 8.65도
        #     잔차 RMS 0.18도 · 위치 산포 x 2.2mm / y 6.7mm
        #
        # 위치는 제자리인데 yaw 만 1:1 로 돈다 — 마커가 servo 1 회전축 위에
        # 있다는 뜻이다. 그래서 그리퍼와 마커는 **같은 회전부**에 있고, 탑뷰가
        # "마커와 기물이 일직선"이라고 하면 그리퍼도 같이 겨눠진다.
        #
        # ── 부호 ──
        #
        # θ = B - marker_yaw 이고 marker_yaw 는 servo1 에 -1 로 붙으므로
        # dθ/d(servo1) = +1 이다. 즉 θ 를 지우려면 servo 1 을 **-θ** 만큼
        # 돌린다 — 이 저장소가 여태 가정만 하던 `-yaw_correction_deg` 가
        # 실측으로 맞았다.
        #
        # ── 한계를 넘으면 자른다, 버리지 않는다 ──
        #
        # 2026-09-07 실기에서 세 번 연속 0.0 이 나갔다. 예전 코드는 한계를
        # 넘으면 보정을 통째로 버렸는데, 그러면 조준이 차체가 우연히 멈춘
        # 각도에 그대로 맡겨진다. ±8도가 0도보다 항상 가깝다.
        #
        # ⚠️ vla_only 면 아예 안 넣는다 — 정책만 돌려 보는 진단 모드다.
        command = None if ports.vla_only else ports.host.last_command()
        theta = 0.0
        if command is not None and command.yaw_correction_deg:
            theta = float(command.yaw_correction_deg)
        wanted = -theta + VLA_PAN_TRIM_DEG
        limit = ga.VLA_PAN_LIMIT_DEG
        pan_bias_deg = max(-limit, min(limit, wanted))
        if theta or VLA_PAN_TRIM_DEG:
            clipped = "" if pan_bias_deg == wanted else f" (한계 ±{limit:.0f}도로 자름)"
            ports.host.report(
                Report.STATE, self.name,
                f"servo 1 조준 θ={theta:+.1f}도 -> pan {pan_bias_deg:+.1f}도"
                f"{clipped}")
        ok = bool(ports.vla.run_grasp(self.label, pan_bias_deg))
        # ⚠️ 정책이 끝난 **직후** 한 번 재 둔다. 최종 판정은 CARRY 뒤에
        # 하는데, 그것만으로는 "정책이 애초에 못 잡았다"와 "잡았다가 CARRY
        # 로 옮기다 놓쳤다"를 구분할 수 없다 — 고칠 곳이 완전히 다른데도.
        #
        # 2026-09-06 실기: 사용자가 "파지를 성공했는데 드는 과정에서
        # 놓쳤다"고 보고했고, CARRY 뒤 판정은 1147(빈 턱 기계정지)이었다.
        # 그 한 숫자로는 어느 쪽인지 알 수가 없었다.
        held_after_policy = ports.arm.gripper_position_raw()
        ports.host.report(
            Report.STATE, self.name,
            f"정책 직후 그리퍼 {held_after_policy} "
            f"({'물고 있음' if held_after_policy >= bc.GRIPPER_HELD_POSITION_RAW else '비었음'}"
            f", 문턱 {bc.GRIPPER_HELD_POSITION_RAW})")

        # ── 정책 결과로 성공/실패를 가르지 않는다 ──────────────────────
        #
        # ⚠️ 2026-09-07 사용자 지시. 여기 있던 것들을 전부 지웠다:
        #
        #   run_grasp 결과로 실패 판정
        #   "루프는 실패했지만 턱은 물고 있다" 구제
        #   실패하면 물체를 놓고 접기(_release_and_fold)
        #
        # run_grasp 가 False 라고 "못 잡았다"는 뜻이 아니다 —
        # RunVlaGrasp.action 이 "True 가 집었다는 뜻이 아니다"라고 경고하는
        # 것의 반대편이고, False 는 **루프가 끝을 못 봤다**는 뜻일 뿐이다.
        # 실기에서 16청크를 다 쓰고도 복귀를 못 본 회차가 실제로는 물체를
        # 물고 있었다.
        #
        # 그걸 실패로 접으면 놓기 경로가 돌고, 놓기는 되돌릴 수 없다 —
        # 성공한 파지를 버린다. 그래서 결과는 **기록만** 하고 그대로 CARRY 로
        # 간다. 못 집었으면 빈 그리퍼로 한 바퀴 돌고 다시 온다.
        ports.host.report(
            Report.STATE, self.name,
            f"정책 루프 {'완료' if ok else '미완료'} · 그리퍼 {held_after_policy} "
            f"(기록용 — 여기서 성공/실패를 가르지 않는다)")
        return True

    #: 버스가 잠깐 나갔다 돌아오는 데 실기에서 4초쯤 걸렸다(2026-09-07).
    #: 간격 x 횟수가 그보다 넉넉해야 한다. 도메인에서 유일하게 자는
    #: 자리라 클래스 속성으로 뺐다 — 시험은 0 으로 두고 부른다.
    RELEASE_RETRIES = RELEASE_RETRIES
    RELEASE_RETRY_SEC = RELEASE_RETRY_SEC

    #: 서보가 버스에서 떨어졌을 때 다음 시도까지 쉬는 시간(초).
    #: 실기 회복이 약 4초였으므로(2026-09-07) 그 언저리로 잡는다. 이것도
    #: 도메인이 자는 자리라 클래스 속성이다 — 시험은 0 으로 두고 부른다.
    HARDWARE_FAULT_DWELL_SEC = 4.0

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
            # recover_idle 은 등록 자세에서만 출발할 수 있다. VLA 가 끝낸
            # 자세는 등록 자세가 아니라 여기서 거의 항상 실패한다 —
            # fold_to_cradle 은 자세 게이트가 없으니 한 번 더 시도한다.
            recovered = ports.arm.fold_to_cradle()
        if not recovered:
            ports.arm.hold_position()
            ports.arm_parked.mark_unparked("파지 실패 뒤 팔을 접지 못했다")
        else:
            ports.arm_parked.mark_parked()

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
        # BaselineGraspState가 CARRY 도달 시점에 이미 끝낸 "정말 물었는가"
        # 판정(부하 OR 뎁스 "사라짐", 2026-09-03). CARRY에 들어왔다는 것
        # 자체가 그 판정을 통과했다는 뜻이라 기본값이 True다 — check_insert가
        # 이 값을 쓰고, 매 사이클 다시 잰 raw 부하로 "비어 있다"를 재판정하지
        # 않는다(box처럼 부하가 계속 낮게 읽히는 물체에서 그 재판정이 영원히
        # 막히는 문제가 있었다).

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

        if command.state == MissionState.DEBUG_FORCE_INSERT:
            # 테스트 전용 우회로 — 라이다 게이트(check_insert)를 건너뛰고
            # 곧장 투하로 들어간다(DEBUG_FORCE_INSERT 정의부 주석 참고).
            ports.host.report(Report.STATE, MissionState.INSERT,
                              "DEBUG_FORCE_INSERT — 라이다 게이트 우회, 시험 전용")
            return BaselineInsertState(self.label)

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
            # ⚠️ 2026-09-05: 둘 중 하나라도 부하 읽기 실패(-1.0, get_load()
            # 문서 참고)면 차분을 내지 않는다. 예전엔 그대로 뺐는데, 실패
            # 신호가 0.0이든 -1.0이든 직전 실측 부하와의 차가 항상 큰
            # 음수로 나와 진짜 미끄러짐(GRIPPER_SLIP_LOAD_DROP=0.010)처럼
            # 오판됐다 — 통신 글리치 한 번마다 INSERT가 "미끄러진다"로
            # 걸렸을 수 있다. 표본 하나가 무효면 이번 사이클은 안정성
            # 판정을 보류하고(None) 다음 사이클에서 다시 본다.
            if load >= 0.0 and previous_load >= 0.0:
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
    보는 Host의 판단이고, Pi는 자기가 아는 것만 보고한다.

    ⚠️ 2026-09-03 실기(queen)에서 이 문턱이 실제 성공을 실패로 오판했다 —
    부하가 0.0469 -> 0.0352(감소폭 0.0117)로 줄었는데, 당시 문턱 0.015보다
    작아서 INSERT_FAILED 로 보고됐다. 하지만 사용자가 바구니 안에 들어간
    걸 육안으로 확인했다 — 실물체 놓임인데도 문턱이 못 넘은 진짜 오탐이다.
    그 여파로 Host 쪽 PLACE 가 FAILED 를 못 받아 넘기고 영구히 얼어붙는
    별개 버그도 같이 드러나서 그건 host/mission.py 에서 고쳤다(FAILED 를
    명시적으로 다음 기물로 넘어가는 분기로 처리)."""

    name = MissionState.INSERT

    # 놓임으로 볼 부하 감소량. 실측 2건(둘 다 실제 성공): 2026-08-26 감소폭
    # 0.0313, 2026-09-03(queen) 감소폭 0.0117 — 둘 다 성공이었는데 옛 문턱
    # 0.015는 두 번째를 실패로 오판했다. 실패 사례가 아직 실측된 적이 없어
    # (기물이 진짜 안 떨어진 경우의 감소폭을 모른다) 정확한 경계는 여전히
    # 미실측이다 — 0.0117보다 여유 있게 낮춰서 두 성공 사례를 다 통과시키는
    # 임시치로 잡았다. 다음에 진짜 실패(안 떨어짐) 사례가 나오면 그 감소폭과
    # 비교해서 다시 조정할 것.
    RELEASE_LOAD_DROP = 0.008

    def __init__(self, label):
        self.label = label
        # CARRY에서 넘어온 판정을 그대로 들고 있다가, 투하 자세 실패로
        # CARRY로 되돌아갈 때(아래) 다시 넘긴다 — 팔만 움직이다 실패한
        # 것이지 그리퍼가 놓친 게 아니므로 판정이 리셋될 이유가 없다.

    # servo 1 보정을 편도로 요청했는데 도착 못 미치는 등 응답이 없을 때(포트
    # 계약상 correct_drop_yaw는 도달 실패도 항상 bool을 준다 — 이 값은 순수
    # 방어용, 실제로는 안 쓰일 것으로 본다).
    _NO_CORRECTION_DEG = 0.0

    def execute(self, ports):
        ports.host.report(Report.STATE, self.name)
        ports.base.stop()
        gp = plan_for_label(self.label)

        # ── 쥐었는지 확인하지 않는다 ─────────────────────────────────────
        #
        # ⚠️ 2026-09-08 사용자 지시로 투하 직전 재확인을 들어냈다:
        #
        #   "진짜 쥐었는지 확인하는 단계와 로봇과 노트북이 집은 것과 못 집은
        #    것에 대해 양방향 소통하는 것이 필요없는 거 같아"
        #
        # 맞는 판단이다. 그 확인이 막으려던 것은 "빈 그리퍼로 투하 동작을
        # 하는 것"인데, 그건 물리적으로 무해하다 — 상자 위에서 손을 폈다
        # 접을 뿐이다.
        #
        # 그리고 **정보가 중복이다.** 물체가 아직 바닥에 있는지는 탑뷰가
        # 이미 안다. Pi 가 서보 위치로 추측해 봐야 Host 가 모르는 것을
        # 더해 주지 못하면서, 틀릴 기회만 만든다 — 2026-09-07 하루에 세 번
        # 틀려서 성공한 파지를 버렸다(문턱이 옛 하한 기준, 루프 미완료를
        # 실패로 읽음, 읽기 실패를 빈손으로 읽음).
        #
        # 못 놓았으면 다음 사이클의 SEARCH_TARGET 이 그 기물을 다시 찾는다.
        # 그게 원래 이 계통의 진실 공급원이다.
        if not ports.arm.move_to_floor_pose(gp.profile, "drop"):
            ports.arm.hold_position()
            ports.host.report(Report.INSERT_FAILED, self.name, "투하 자세 실패")
            # 표본(라이다·부하)은 버린다 — 팔
            # 자세만 실패했지 그리퍼가 놓친 게 아니다.
            return BaselineCarryState(self.label)

        # safe_300 — "drop" 자세(300mm)에 도달했지만 아직 그리퍼는 열지
        # 않은 상태다. Host가 차량을 NUDGE 경계선에서 방향 그대로(방향에
        # 상관없이) 세우고 남은 지향 오차를 yaw_correction_deg로 실어 보내면,
        # 여기서 그리퍼를 열기 **전에** servo 1을 그만큼 돌려 흡수한다
        # (사용자 지시, 2026-09-05 — 차량을 다시 회전시키는 대신 팔로
        # 보정한다). 값이 0이면 이 단계 자체가 보고 없이 통째로 건너뛰어진다
        # — 기존 경로(차량이 이미 FACE_BOX로 정렬해 오는 경우)와 100% 동일하게
        # 동작한다.
        #
        # ⚠️ correct_drop_yaw는 servo 1 한계각(교시 정면 기준, 사용자 지시로
        # ±60도 — GRASP 좌우보정의 ±15도와 별개다. arm_driver_node의
        # MAX_DROP_YAW_OFFSET_RAD 주석 참고)을 넘으면 그 자리에서 거부하고
        # False를 준다 — 그런 경우도 투하 자체는 포기하지 않는다. 팔로 다
        # 못 흡수한 오차를 안고 여는 것이 물체를 든 채 무한정 멈춰 있는
        # 것보다 낫다는 판단이다(다른 실패들과 같은 원칙 — BaselineInsertState
        # 클래스 docstring 참고). 대신 보고에 실패 사실을 남겨 Host가 다음
        # 기물부터 반영할 수 있게 한다.
        command = ports.host.latest_command()
        yaw_correction_deg = (
            command.yaw_correction_deg if command is not None else self._NO_CORRECTION_DEG)
        applied_rad = 0.0
        if yaw_correction_deg != 0.0:
            ports.host.report(
                Report.STATE, MissionState.SAFE_300,
                f"servo 1 요 보정 {yaw_correction_deg:+.1f}도 적용 시도")
            # ⚠️ 2026-09-05 실기 확인: facing_error_deg 부호를 그대로 넘기면
            # servo 1이 오차를 줄이는 게 아니라 반대쪽으로 돈다(사용자 보고
            # — "servo1이 돌았는데, 반대방향으로 돌았어"). facing_error_deg는
            # 차량 좌표계 기준, servo 1의 +방향은 팔 베이스 좌표계 기준이라
            # 둘의 부호축이 반대인 것으로 실측됐다 — 여기서 부호를 뒤집어
            # 흡수한다(manual_insert_probe.py 상단 docstring에 이미 예견해
            # 둔 대응).
            correction_rad = -math.radians(yaw_correction_deg)
            if ports.arm.correct_drop_yaw(correction_rad):
                applied_rad = correction_rad
            else:
                ports.host.report(
                    Report.STATE, MissionState.SAFE_300,
                    f"servo 1 요 보정 {yaw_correction_deg:+.1f}도 거부됨 — "
                    "보정 없이 투하를 계속한다")

        before = ports.arm.get_load()
        # ── 놓기: 명령이 아니라 **위치로** 확인하고, 안 되면 다시 한다 ────
        #
        # ⚠️ 2026-09-07 팀원 보고: "상자나 별을 정리상자에 넣는 순간 servo 6
        # 오류로 그리퍼를 안 푼다." 여기 코드가 정확히 그 모양이었다.
        #
        #   1) set_gripper 을 **한 번만** 부르고, 그 함수는 반환값이 없어
        #      통신 실패가 안 보였다.
        #   2) 놓았는지를 **부하 차이**로 봤는데, 부하는 못 쓴다는 것이 이미
        #      실측으로 확정됐다(퀸을 문 것 10/256 대 빈손 9/256 — 한 양자화
        #      단위 차이다. GRIPPER_HELD_POSITION_RAW 주석의 표).
        #   3) 부하를 **못 읽으면 released=True** 로 단정했다. 서보가 맛이
        #      갔을 때가 바로 못 읽는 때다 — 실패를 성공으로 읽는 조합이다.
        #   4) 그리고 곧바로 CLOSED_MM 으로 닫았다. 물체가 안 떨어졌으면
        #      **도로 물어 버린다.**
        #
        # 이제 위치로 본다. 열렸으면 raw 가 1600 을 넘고(투하 폭은 약 1989),
        # 못 읽으면 -1 이라 자연히 "안 열렸다"가 된다.
        released = release_until_open(ports, gp.release_width_mm)
        after = ports.arm.get_load()
        # ⚠️ 2026-09-05: before/after 둘 중 하나라도 부하 읽기 실패(-1.0)면
        # 차분 비교 자체를 하지 않는다 — before가 -1.0이면 `after - before`가
        # 항상 거대한 음수가 되어 진짜로는 놓였어도 "부하가 안 줄었다"로
        # 오판되는 게 아니라(부호가 반대라 이 경우엔 오히려 실제로 안 놓여도
        # "줄었다"로 오판될 수 있다), 어느 쪽이든 이 비교가 무의미해진다.
        # 그리퍼를 열라는 명령 자체는 정상적으로 내려갔으니, 읽기가 실패한
        # 경우는 놓인 것으로 본다(release_width_mm 명령이 실행됐다는 사실을
        # 신뢰) — 다만 보고 문구에 판독 실패였다는 걸 남긴다.
        load_read_failed = before < 0.0 or after < 0.0

        # ⚠️ 놓은 것이 확인됐을 때만 닫는다. 예전에는 무조건 닫았는데, 안
        # 떨어진 물체를 도로 무는 동작이었다(위 4번). 못 놓았으면 턱을 벌린
        # 채 둔다 — 접기 전에 닫는 것은 팔이 차체를 안 긁게 하려는 조치이고,
        # 물건을 물고 가는 것보다는 그 위험이 작다.
        if released:
            ports.arm.set_gripper(CLOSED_MM)
        # safe_300에서 servo 1을 돌렸으면, idle로 접기 전에 먼저 그 각도를
        # 되돌린다(사용자 지시) — idle 자체도 servo 1을 교시 절대값으로
        # 되돌리긴 하지만(_move_floor_stage 참고), 큰 보정각을 그대로 안고
        # 5관절 글라이드를 한 번에 타는 대신 servo 1만 먼저 원위치시켜
        # 시작 자세를 always drop pose 그대로로 맞춘다.
        if applied_rad != 0.0:
            if not ports.arm.correct_drop_yaw(-applied_rad):
                ports.host.report(
                    Report.STATE, MissionState.SAFE_300,
                    "servo 1 원위치 복귀 실패 — idle 글라이드가 대신 정렬한다")
        folded = ports.arm.move_to_floor_pose(gp.profile, "idle")

        if released:
            detail = (f"{self.label} 그리퍼가 열린 것을 위치로 확인했다 "
                      f"(문턱 {bc.GRIPPER_RELEASED_MIN_RAW}) · "
                      f"부하 {before:.4f} -> {after:.4f}")
            if load_read_failed:
                detail += " (부하는 판독 실패 — 판정에는 안 쓴다)"
            ports.host.report(Report.INSERT_DONE, self.name, detail)
        else:
            # 놓이지 않았는데 IDLE로 접으면 물체를 문 채 라이다를 가린다.
            # 그래도 접기는 한다 — 팔을 전개한 채 두는 편이 더 위험하다.
            ports.host.report(
                Report.INSERT_FAILED, self.name,
                f"{RELEASE_RETRIES}회 시도했는데 그리퍼가 안 열렸다 "
                f"(위치가 {bc.GRIPPER_RELEASED_MIN_RAW} 미만) — 물체를 문 채 "
                f"접는다. servo 6 통신을 볼 것")

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
