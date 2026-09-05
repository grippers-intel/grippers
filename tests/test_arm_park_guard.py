"""팔이 안 접혔으면 주행을 막는다 (2026-09-06 실기 사고 대응).

## 왜 필요한가

VLA 파지가 실패해 팔이 미등록 자세에 남았는데, Host 가 RETURN_HOME 으로
넘어가 **팔을 뻗은 채 주행했다.** 부딪히면 팔이 부서지고 라이다 시야도 가린다.

Host 는 팔 상태를 모른다 — HostCommand 에 그 정보가 없고, 알 이유도 없다
(역할 분담: Host 는 아레나를 보고 명령하고, Pi 는 자기 하드웨어를 안다).
그래서 아는 쪽인 Pi 가 막는다.

## 정지는 왜 통과시키는가

멈추는 것은 언제나 안전하다. 팔이 이상하다고 정지 명령까지 막으면, 정작
세워야 할 때 못 세운다 — 2026-08-28 에 "정지 828회가 안 먹은" 사고를 겪은
저장소에서 그 방향의 실수는 하지 않는다.
"""

import pytest

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.fake_host_link import FakeHostLink
from domain.ports.baseline_ports import HostCommand, MissionState, Report
from domain.task.baseline_mission import ArmParkLatch, BaselinePorts, _drive


def _ports(**kw):
    return BaselinePorts(base=FakeBase(), arm=FakeArm(), perception=None,
                         host=FakeHostLink(), lidar=None, estop=None, **kw)


def test_기본값은_접힘이다():
    """기동 직후를 위험 상태로 보면 매번 못 움직인다 — 모르는 것과 고장은 다르다."""
    assert ArmParkLatch().parked is True


def test_안_접혔으면_전진을_막는다():
    ports = _ports()
    ports.arm_parked.mark_unparked("파지 실패 뒤 팔을 접지 못했다")

    ok = _drive(ports, HostCommand(state=MissionState.APPROACH, linear_x=0.15),
                "APPROACH")

    assert ok is False
    assert ports.base.last_velocity in (None, (0.0, 0.0, 0.0))
    kinds = [r[0] for r in ports.host.reports]
    assert Report.REJECTED in kinds


def test_안_접혔어도_정지는_통과한다():
    """멈추는 것은 언제나 안전하다."""
    ports = _ports()
    ports.arm_parked.mark_unparked("무언가 잘못됐다")

    ok = _drive(ports, HostCommand(state=MissionState.APPROACH, stop=True),
                "APPROACH")

    assert ok is True


def test_접혀_있으면_평소대로_간다():
    ports = _ports()

    ok = _drive(ports, HostCommand(state=MissionState.APPROACH, linear_x=0.15),
                "APPROACH")

    assert ok is True
    assert ports.base.last_velocity is not None


def test_거부_사유에_풀_방법이_들어간다():
    """사람이 무엇을 해야 하는지 모르면 경보는 소음이다."""
    ports = _ports()
    ports.arm_parked.mark_unparked("파지 실패 뒤 팔을 접지 못했다")

    _drive(ports, HostCommand(state=MissionState.APPROACH, linear_x=0.15), "APPROACH")

    detail = " ".join(str(r) for r in ports.host.reports)
    assert "align_to_idle" in detail
