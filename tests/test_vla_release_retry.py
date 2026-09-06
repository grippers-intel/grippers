"""정책 실패 뒤의 놓기·접기는 **끈질기게** 다시 시도한다 (2026-09-07).

## 사고

실기 로그가 4초 안에 이렇게 흘렀다.

    185.95  arm_driver   관절 청크 하드웨어 오류: servo 6 write 실패 — step 63/63
    185.96  vla          청크 재생 실패
    185.96  mission      RETURN vla.run_grasp result=False
    186.09  mission      RETURN arm.gripper_position_raw result=-1
    186.19  arm_driver   set_gripper 실패: SO-ARM101 servo 통신 실패 — servo IDs: [6]
    186.22  arm_driver   fold_to_cradle 실패: servo 6 present position 읽기 실패
    190.79  mission      RETURN arm.move_to_floor_pose result=True     <- 멀쩡해졌다

버스가 잠깐 나갔던 것뿐이라 4.5초 뒤에는 정상이었다. 그런데 놓기는 그
100ms 창 안에서 **한 번만** 시도하고 포기했고, 그때 그리퍼에는 별이 물려
있었다. 사용자 보고:

    "실제로 파지도 되었는데 물건을 따로 놓으러가지는 않고 3번째 물건을
     잡으러가는 행동을 취했어"

놓기는 한 번 실패해도 물러설 수 있는 동작이 아니다 — 실패한 파지가 물건을
들고 다음 기물로 가는 것이 최악이다.

## set_gripper 로는 실패를 알 수 없다

`ArmDriver.set_gripper` 는 반환값이 없다(포트 계약). 위 로그에서 실패한
그 호출도 호출한 쪽에서는 성공과 구분되지 않는다. 그래서 놓았는지는
명령이 아니라 **위치를 읽어서** 판정한다(bc.GRIPPER_RELEASED_MIN_RAW).
"""

import pytest

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.fake_host_link import FakeHostLink
from domain.ports.baseline_ports import Report
from domain.task import baseline_constants as bc
from domain.task.baseline_mission import BaselineGraspState, BaselinePorts

OPEN_RAW = 2000          # 투하 폭(약 168mm)에서의 실측 위치
STUCK_RAW = 1181         # 별을 문 채 멈춘 위치 — 2026-09-07 실기값


class _Profile:
    release_width_mm = 168.0
    profile = "queen"


class _FlakyArm(FakeArm):
    """`n_bad` 번째 호출까지는 버스가 나가 있다."""

    def __init__(self, bad_calls: int, stuck_raw: int = STUCK_RAW):
        super().__init__()
        self.bad_calls = bad_calls
        self.stuck_raw = stuck_raw
        self.set_gripper_calls = 0
        self.fold_calls = 0

    def _bus_down(self) -> bool:
        return self.set_gripper_calls <= self.bad_calls

    def set_gripper(self, width_mm):
        self.set_gripper_calls += 1

    def gripper_position_raw(self) -> int:
        # 버스가 나가 있으면 읽기 자체가 실패한다(-1). 살아나면 열려 있다.
        return -1 if self._bus_down() else OPEN_RAW

    def fold_to_cradle(self) -> bool:
        self.fold_calls += 1
        return not self._bus_down()


def _state():
    state = BaselineGraspState("queen", creep_m=0.0)
    state.RELEASE_RETRY_SEC = 0.0        # 시험은 안 잔다
    return state


def _ports(arm):
    return BaselinePorts(base=FakeBase(), arm=arm, perception=None,
                         host=FakeHostLink(), lidar=None, estop=None,
                         grasp_backend="vla")


# ── 재시도 ─────────────────────────────────────────────────────────────────


def test_한_번_실패해도_다시_시도해_놓는다():
    """⚠️ 이것이 2026-09-07 사고다 — 예전 코드는 여기서 끝났다."""
    arm = _FlakyArm(bad_calls=1)
    state = _state()

    assert state._release_and_fold(_ports(arm), _Profile())
    assert arm.set_gripper_calls == 2


def test_버스가_4초_나가_있어도_끝내_놓는다():
    """실기에서 4.5초였다 — 재시도 예산이 그보다 넉넉해야 한다."""
    state = _state()
    budget_s = (state.RELEASE_RETRIES - 1) * BaselineGraspState.RELEASE_RETRY_SEC
    assert budget_s >= 4.5, f"재시도 예산 {budget_s}s 로는 실기 4.5초를 못 넘긴다"


def test_끝내_못_놓으면_False_와_함께_알린다():
    """조용히 넘어가면 물건을 문 채 다음 기물로 간다 — 그게 사고였다."""
    arm = _FlakyArm(bad_calls=99)
    ports = _ports(arm)

    assert not _state()._release_and_fold(ports, _Profile())
    assert Report.GRASP_BLOCKED in ports.host.reported_kinds
    assert "물건이 남아 있을 수 있다" in " ".join(
        detail for _r, _s, detail, _f in ports.host.reports)


# ── 판정을 명령이 아니라 위치로 한다 ──────────────────────────────────────


def test_명령만_보내고_안_열렸으면_놓은_게_아니다():
    """set_gripper 는 반환값이 없어 실패해도 조용하다 — 위치로 확인한다."""

    class SilentlyStuckArm(FakeArm):
        def set_gripper(self, width_mm):
            pass                              # 명령은 받되 서보는 안 움직인다

        def gripper_position_raw(self) -> int:
            return STUCK_RAW                  # 별을 문 그대로

        def fold_to_cradle(self) -> bool:
            return True

    ports = _ports(SilentlyStuckArm())
    assert not _state()._release_and_fold(ports, _Profile())


def test_문턱은_어떤_기물보다도_넓다():
    """1600(약 96mm) 위라면 턱 사이에 무엇이 끼어 있을 수가 없다."""
    assert bc.GRIPPER_RELEASED_MIN_RAW > STUCK_RAW
    assert bc.GRIPPER_RELEASED_MIN_RAW > bc.GRIPPER_HELD_POSITION_RAW
    assert bc.GRIPPER_RELEASED_MIN_RAW < OPEN_RAW, (
        "명령값과 같으면 서보가 끝까지 못 간 정상 상황을 실패로 읽는다")


def test_위치_읽기_실패는_놓은_것으로_안_친다():
    """-1 은 '모른다'다. 실기에서 놓기 실패 직후 실제로 -1 이 나왔다."""

    class UnreadableArm(FakeArm):
        def set_gripper(self, width_mm):
            pass

        def gripper_position_raw(self) -> int:
            return -1

        def fold_to_cradle(self) -> bool:
            return True

    assert not _state()._release_and_fold(_ports(UnreadableArm()), _Profile())


# ── 놓기와 접기를 따로 센다 ────────────────────────────────────────────────


def test_놓기_성공_뒤_접기만_실패하면_놓기를_또_안_부른다():
    """둘은 다른 서보를 건드린다 — 이미 된 것을 다시 할 이유가 없다."""

    class FoldOnlyFailsArm(FakeArm):
        def __init__(self):
            super().__init__()
            self.set_gripper_calls = 0
            self.fold_calls = 0

        def set_gripper(self, width_mm):
            self.set_gripper_calls += 1

        def gripper_position_raw(self) -> int:
            return OPEN_RAW

        def fold_to_cradle(self) -> bool:
            self.fold_calls += 1
            return self.fold_calls >= 3

    arm = FoldOnlyFailsArm()
    assert _state()._release_and_fold(_ports(arm), _Profile())
    assert arm.set_gripper_calls == 1
    assert arm.fold_calls == 3
