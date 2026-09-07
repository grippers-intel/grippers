"""공용 테스트 픽스처 — 하드웨어·ROS2 없이 Fake 어댑터로 Pi 미션 FSM을 구동한다."""

import threading

import pytest

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.fake_vla import FakeVla
from domain.adapters.fake.fake_host_link import FakeHostLink, FakeLidar
from domain.adapters.fake.scripted_perception import ScriptedPerception
from domain.task.baseline_mission import BaselineMission, BaselinePorts, LinkWatchdog

# 무한 루프 방지가 이 프로젝트의 최대 리스크다(docs/design/state_machine.md §4).
# 테스트 쪽도 같은 원칙을 지킨다 — "느리게라도 끝났다"가 아니라 "상한 안에 못
# 끝나면 그 자체가 실패"로 취급한다.
MAX_STEPS = 200


@pytest.fixture
def make_ports():
    def _make(base=None, arm=None, perception=None, host=None, lidar=None,
              estop=None, watchdog=None, vla=None):
        return BaselinePorts(
            # 파지 경로가 정책 하나뿐이라(2026-09-07) 이 포트가 없으면 GRASP 가
            # 그 자리에서 죽는다. 기본은 "정책이 루프를 끝까지 돌았다"이다 —
            # 진짜 파지 성공은 FakeArm.jaw_blocked_raw 가 정한다.
            vla=vla or FakeVla(),
            base=base or FakeBase(),
            arm=arm or FakeArm(),
            perception=perception or ScriptedPerception(),
            host=host or FakeHostLink(),
            lidar=lidar or FakeLidar(),
            estop=estop or threading.Event(),
            watchdog=watchdog or LinkWatchdog(),
        )

    return _make


@pytest.fixture
def run_to_completion():
    def _run(ports, max_steps=MAX_STEPS):
        """`BaselineMission.run()`을 상한 안에서 끝까지 구동해 yield된 State
        목록을 반환한다. 상한에 닿으면 즉시 실패시킨다.

        Host 주도 FSM은 Host가 DONE을 보내야 끝난다 — `FakeHostLink`의
        스크립트 마지막이 DONE이 아니면 이 상한에 걸린다."""
        states = []
        gen = BaselineMission(ports).run()
        for _ in range(max_steps):
            try:
                states.append(next(gen))
            except StopIteration:
                return states
        pytest.fail(
            f"{max_steps}스텝 안에 종료되지 않음 — 무한 루프 의심. "
            f"마지막 10개 상태: {[s.name for s in states[-10:]]}"
        )

    return _run


# ── 실기용 대기 시간은 시험에서 0 으로 ────────────────────────────────────
#
# 파지 판정 전 그리퍼 정착 대기(GRIP_SETTLE_SEC 1.5초)와 놓기 재시도 간격
# (RELEASE_RETRY_SEC 1.5초)은 실기 값이다. 시험이 실제로 자면 전체가 22초에서
# 82초로 늘어난다 — 그 시간에 확인되는 것은 아무것도 없다.
@pytest.fixture(autouse=True)
def _no_hardware_waits(monkeypatch):
    from domain.task import baseline_mission as bm
    monkeypatch.setattr(bm, "GRIP_SETTLE_SEC", 0.0)
    monkeypatch.setattr(bm, "RELEASE_RETRY_SEC", 0.0)
