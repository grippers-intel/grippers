"""구동계 생존 감시 (2026-08-28 정지 실패 사고 대응).

## 무엇을 고정하는가

그날 마지막 시험에서 **정지 명령은 전부 정상적으로 나갔는데 차량이 멈추지
않았다.** Pi 워치독이 828사이클 연속으로 `base.stop()`을 불렀고, 그동안
바퀴는 0.092 m/s로 등속 직진했다. `cmd_vel`을 모터 보드로 옮기는 노드가
물려 있었기 때문이고, STM32는 마지막 속도를 무기한 계속 집행한다.

여기서 고정하는 성질은 하나다 — **명령이 닿지 않는 상태를 소프트웨어가
알아채고 사람에게 말한다.** 자동으로 고치지는 못한다(고칠 수단이 없다).

같은 버그가 그날 오후에는 반대 얼굴로 나왔다는 것도 같이 고정한다. `go`가
155회 나가는 동안 로봇이 1mm도 안 움직였던 것과, 정지가 836회 나갔는데 안
멈춘 것은 **같은 물림**이다 — 걸려 있던 값이 0이냐 아니냐만 달랐다.
"""

import threading

import pytest

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.fake_host_link import FakeHostLink, FakeLidar
from domain.adapters.fake.scripted_perception import ScriptedPerception
from domain.ports.baseline_ports import HostCommand, MissionState, Report
from domain.task import base_liveness as bl
from domain.task.baseline_mission import BaselineMission, BaselinePorts


# ── 판정 자체 ──────────────────────────────────────────────────────────────


def test_기동_직후에는_판정하지_않는다():
    """디스커버리가 끝나기 전에 경보를 울리면 매 기동마다 거짓 경보가 뜬다."""
    verdict = bl.judge(subscriber_count=0, feedback_age_s=None,
                       commanding_for_s=0.5)
    assert verdict.state == bl.UNKNOWN
    assert verdict.alive is True


def test_cmd_vel_구독자가_없으면_명령이_닿지_않는다():
    """내가 ros_robot_controller 를 죽였던 순간이 정확히 이 상태다."""
    verdict = bl.judge(subscriber_count=0, feedback_age_s=0.0,
                       commanding_for_s=10.0)
    assert verdict.state == bl.NO_CONSUMER
    assert verdict.alive is False


def test_피드백이_끊기면_컨트롤러가_물린_것으로_본다():
    """그날 실제로 일어난 쪽. 노드는 떠 있어서 프로세스 목록으로는 멀쩡하다."""
    verdict = bl.judge(subscriber_count=1, feedback_age_s=5.0,
                       commanding_for_s=10.0)
    assert verdict.state == bl.STALE_FEEDBACK
    assert verdict.alive is False


def test_구독자도_있고_피드백도_신선하면_살아_있다():
    verdict = bl.judge(subscriber_count=1, feedback_age_s=0.05,
                       commanding_for_s=10.0)
    assert verdict.state == bl.ALIVE
    assert verdict.alive is True


def test_구독자_0이_낡은_피드백보다_먼저_보고된다():
    """둘 다 어긋나 있으면 원인을 말해야 한다.

    아무도 안 듣고 있으면 피드백이 낡은 것은 **결과**다. 사람에게 결과를
    말하면 엉뚱한 곳을 고치러 간다."""
    verdict = bl.judge(subscriber_count=0, feedback_age_s=99.0,
                       commanding_for_s=10.0)
    assert verdict.state == bl.NO_CONSUMER


def test_피드백을_한_번도_못_받으면_물린_것으로_본다():
    verdict = bl.judge(subscriber_count=1, feedback_age_s=None,
                       commanding_for_s=10.0)
    assert verdict.state == bl.STALE_FEEDBACK


# ── 래치 ──────────────────────────────────────────────────────────────────


def test_같은_상태가_이어지면_한_번만_말한다():
    """10Hz 루프에서 매 사이클 경고하면 로그가 그것으로 가득 찬다."""
    latch = bl.LivenessLatch()
    dead = bl.LivenessVerdict(bl.NO_CONSUMER, "아무도 안 듣는다")

    first = latch.observe(dead)
    rest = [latch.observe(dead) for _ in range(50)]

    assert first is not None
    assert rest == [None] * 50


def test_복구도_한_번_말한다():
    """고장만 말하고 복구를 안 말하면, 사람이 아직 고장 중인 줄 안다."""
    latch = bl.LivenessLatch()
    latch.observe(bl.LivenessVerdict(bl.NO_CONSUMER, "x"))

    message = latch.observe(bl.LivenessVerdict(bl.ALIVE))

    assert message is not None and "복구" in message


def test_처음부터_정상이면_아무_말도_안_한다():
    """정상 기동에서 '복구됐다'가 나오면 그 말의 값어치가 사라진다."""
    latch = bl.LivenessLatch()
    assert latch.observe(bl.LivenessVerdict(bl.ALIVE)) is None


def test_liveness_를_모르는_어댑터는_그냥_지나간다():
    """모르는 것과 고장난 것은 다르다."""
    latch = bl.LivenessLatch()
    assert latch.observe(None) is None
    assert latch.state is None


# ── 미션 루프와의 결합 ──────────────────────────────────────────────────────


class WedgedBase(FakeBase):
    """`cmd_vel` 아래가 물린 차량.

    2026-08-28의 그 상태다 — `stop()`은 정상적으로 받아들이고 아무 예외도
    내지 않는다. 소프트웨어 쪽에서 보면 모든 것이 성공한다."""

    def liveness(self):
        return bl.LivenessVerdict(bl.STALE_FEEDBACK, "컨트롤러가 물렸다")


# FakeArm 기본값은 LOAD_HOLDING(0.14) = **물체를 쥔 상태**다. GRASP 전제는
# 빈 그리퍼를 요구하므로, 파지 판정을 보는 테스트는 빈손 실측값을 넣어야 한다.
EMPTY_LOAD = 0.03


def _ports(base, script, arm=None):
    return BaselinePorts(
        base=base,
        arm=arm if arm is not None else FakeArm(),
        perception=ScriptedPerception(),
        host=FakeHostLink(script),
        lidar=FakeLidar(),
        estop=threading.Event(),
    )


def _run(ports, cycles):
    """미션을 `cycles` 사이클만 돌린다."""
    for index, _state in enumerate(BaselineMission(ports).run()):
        if index >= cycles:
            break


def test_구동계가_물리면_Host에_보고한다():
    """이 보고가 없으면 Host는 차가 섰다고 믿는다 — 그날의 실패 그대로다."""
    ports = _ports(WedgedBase(), [None])

    _run(ports, cycles=20)

    assert Report.BASE_UNRESPONSIVE in ports.host.reported_kinds


def test_물린_상태가_이어져도_보고는_한_번뿐이다():
    """워치독 거부는 매 사이클 나온다 — 여기까지 매 사이클이면 로그가 죽는다."""
    ports = _ports(WedgedBase(), [None])

    _run(ports, cycles=50)

    unresponsive = [k for k in ports.host.reported_kinds
                    if k == Report.BASE_UNRESPONSIVE]
    assert len(unresponsive) == 1


def test_정상_차량은_구동계_경보를_내지_않는다():
    """`FakeBase`는 liveness를 모른다 — 모를 때 경보하면 아무도 안 본다."""
    ports = _ports(FakeBase(), [None])

    _run(ports, cycles=30)

    assert Report.BASE_UNRESPONSIVE not in ports.host.reported_kinds


def test_정지를_지시받는_동안에도_감시가_돈다():
    """이 신호가 가장 필요한 순간이 정지를 지시하는 순간이다.

    2026-08-28에 놓친 자리가 정확히 여기다 — Host가 정지를 보내고 Pi가
    그것을 성실히 실행하는 동안, 아무도 그 명령이 닿는지 보지 않았다."""
    stop_command = HostCommand(state=MissionState.APPROACH, stop=True)
    ports = _ports(WedgedBase(), [stop_command])

    _run(ports, cycles=20)

    assert ports.base.stop_calls > 0            # 정지는 성실히 나갔고
    assert Report.BASE_UNRESPONSIVE in ports.host.reported_kinds   # 안 닿았다


def test_한_물림이_두_얼굴로_나타난다():
    """`go`가 안 먹는 것과 `stop`이 안 먹는 것은 같은 고장이다.

    2026-08-28 오후에는 `go` 155회에 1mm도 안 움직였고(래치된 값 0), 저녁에는
    정지 836회에 안 멈췄다(래치된 값 go). 감시는 명령의 종류와 무관하게
    같은 신호를 내야 한다 — 종류별로 갈라 놓으면 한쪽만 잡는다."""
    driving = HostCommand(state=MissionState.APPROACH, linear_x=0.1)
    stopping = HostCommand(state=MissionState.APPROACH, stop=True)

    kinds = []
    for command in (driving, stopping):
        ports = _ports(WedgedBase(), [command])
        _run(ports, cycles=20)
        kinds.append(Report.BASE_UNRESPONSIVE in ports.host.reported_kinds)

    assert kinds == [True, True]


def test_감시는_미션_상태와_무관하게_돈다():
    """특정 상태에만 걸면 그 상태를 안 지나는 실행에서 통째로 놓친다."""
    ports = _ports(WedgedBase(), [HostCommand(state=MissionState.IDLE)])

    _run(ports, cycles=10)

    reported = [state for kind, state, _d, _f in ports.host.reports
                if kind == Report.BASE_UNRESPONSIVE]
    assert reported == [MissionState.IDLE]


# ── 목표를 못 봤을 때 후진을 요구한다 (run6 회귀) ──────────────────────────


def test_목표를_못_보면_GRASP_BLOCKED_에_보정이_실린다():
    """2026-08-28 run6이 여기서 죽었다.

    Pi가 보정 없이 `뎁스 카메라가 정면에서 목표를 찾지 못했다`만 보냈고,
    Host는 그것을 고칠 수 없는 것으로 읽어 `rook 보류: 고칠 수 없음`으로
    기물을 통째로 포기했다. 보정이 실려 있어야 Host가 물러났다 다시 본다."""
    from domain.task import corrections as cx
    from domain.task.baseline_mission import BaselineApproachState

    ports = _ports(FakeBase(), [HostCommand(state=MissionState.GRASP)],
                   arm=FakeArm(load_ratio=EMPTY_LOAD))
    ports.perception = ScriptedPerception(label=None)

    BaselineApproachState().execute(ports)

    blocked = [(kind, fix) for kind, _s, _d, fix in ports.host.reports
               if kind == Report.GRASP_BLOCKED]
    assert blocked, "GRASP_BLOCKED 가 나와야 한다"
    _kind, fix = blocked[-1]
    assert fix is not None, "보정이 없으면 Host 가 기물을 포기한다"
    assert fix.action == cx.RETREAT
