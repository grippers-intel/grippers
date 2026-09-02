"""NUDGE_BOX가 바구니에 막혀도 오래 밀어붙이지 않는다 (11:47 실기, 2026-09-02).

## 처음 진단이 틀렸던 부분

이 로그를 처음 봤을 때는 "라이다 0.155m(도착) → 0.139m(더 들어감)"을 오버슈트로
읽었다. 하지만 `mission_config.py`의 목표/데드밴드 설계 주석을 다시 보면, 목표는
Pi 수용창의 **중심**(0.140m)이고 데드밴드는 그 절반(수렴점이 창 가장자리가
아니라 한가운데 오게)이다 — 즉 0.155m(창 상한, "그만 밀어도 된다")에서
0.139m(중심 근처)로 더 들어간 것 자체는 **설계대로 동작한 것**이다.

## 진짜 문제

문제는 그 16mm를 옮기는 데 걸린 시간이다. `NUDGE_BOX`는 이 접근 구간에서
0.06 m/s로 움직인다(`BASKET_APPROACH_MPS`) — 16mm면 0.27초면 끝나야 한다.
그런데 로그 타임스탬프(73.0s cmd=go 시작 → 82.6s 도착)로는 **9.6초**가 걸렸다.
그 사이 ArUco 위치는 거의 그대로였다(1.328,1.357 → 1.330,1.363 → 1.329,1.360)
— 차가 바구니에 막혀 헛돌면서(기어 백래시/바퀴 미끄러짐) 아주 조금씩만
전진한 것이다.

`BASKET_NUDGE_PROGRESS_M`(10mm)을 넘는 진전이 있으면 정체 타이머
(`BASKET_NUDGE_STALL_SEC`)가 그대로 리셋된다 — 이 자체는 "모터 전원이 아예
안 들어온" 경우(2026-08-28 실기, 62초 동안 0mm)를 잡기 위한 설계라 옳다.
그런데 리셋 문턱이 10mm 뿐이라, 바구니에 막혀 조금씩 미끄러지는 경우에도
계속 리셋되면서 원래 여유(10.0초 = 문서가 밝힌 필요시간 5초의 2배)를
꽉 채워 쓴다 — 그 10초 내내 차는 바구니를 계속 밀고 있다. 이게 사용자가
말한 "돌진"이다.

이 파일은 로봇 없이 "막혀서 거의 안 움직이는" NUDGE_BOX가 옛 10초가 아니라
새 정체시간 안에 멈추는지 확인한다."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import pytest

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

import mission                                     # noqa: E402
import mission_config as mcfg                      # noqa: E402
from mission import DriveMode, MissionFSM, State   # noqa: E402
from vehicle_link import MissionCommand            # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from conftest import PiSim                          # noqa: E402

_TICK = 1.0 / 14.0   # 실측 Host 루프 주기(PiSim.dt와 동일)


class FakeClock:
    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, dt: float) -> None:
        self.now += dt


@dataclass
class BlockedPi(PiSim):
    """전진/후진 명령을 받아도 실제로는 (거의) 안 움직이는 Pi.

    이미 바구니에 닿아 막힌 상태 — 기어 백래시나 바퀴 미끄러짐으로 아주
    조금씩만 실제 위치가 바뀐다고 가정해 최악(=제일 안 멈추기 쉬운) 경우를
    시늉한다. 정확히 0mm면 옛 코드도 바로 정체로 잡으니 의미가 없다."""

    def _move(self, cmd: MissionCommand) -> None:
        if cmd.cmd in ("go", "back"):
            return
        super()._move(cmd)


@pytest.fixture
def fake_clock(monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr(mission.time, "monotonic", clock)
    return clock


def _nudge_fsm() -> tuple[MissionFSM, BlockedPi]:
    fsm = MissionFSM()
    fsm.state = State.NUDGE_BOX
    fsm.target_label = "rook"
    fsm.dest_xy = (1.271, 1.30)
    link = BlockedPi(x=1.27, y=1.25, yaw_deg=mcfg.BOX_FACE_YAW_DEG)
    return fsm, link


def test_필요시간_문서값보다는_넉넉하다():
    """`BASKET_NUDGE_STALL_SEC` 주석이 밝힌 필요시간(최대 5초)보다는 넉넉해야
    정상적인 접근까지 정체로 오탐하지 않는다."""
    assert mcfg.BASKET_NUDGE_STALL_SEC >= 5.5


def test_10초씩_밀어붙이지_않는다():
    """2026-08-28 실기(모터 전원 없음)를 잡으려던 옛 10초 여유가, 09-02
    11:47 실기(바구니에 막힘)에서는 그대로 "10초 동안 계속 미는" 시간이
    됐다. 새 여유는 그보다 뚜렷하게 짧아야 한다."""
    assert mcfg.BASKET_NUDGE_STALL_SEC < 10.0


def test_막혀도_새_정체시간_안에_멈춘다(fake_clock):
    fsm, link = _nudge_fsm()
    # 도착까지 필요한 것보다 훨씬 큰 목표를 줘서, 정체가 아니면 계속
    # "go"를 냈을 상황을 만든다.
    fsm._nudge_plan = (0.20, "forward")

    max_ticks = int((mcfg.BASKET_NUDGE_STALL_SEC + 2.0) / _TICK) + 5
    for _ in range(max_ticks):
        fake_clock.advance(_TICK)
        fsm.step(link.pose(), {}, link)
        if fsm.last_cmd == "stop":
            break
    else:
        pytest.fail("정체 상태인데도 멈추지 않았다")

    elapsed = fake_clock.now - 1000.0
    assert elapsed <= mcfg.BASKET_NUDGE_STALL_SEC + 1.0, (
        f"멈추기까지 {elapsed:.1f}초 걸렸다 — 정체시간({mcfg.BASKET_NUDGE_STALL_SEC}"
        f"초)보다 한참 늦게 멈췄다")
    # 옛 설정(10초)이었다면 이 시점엔 아직 안 멈췄어야 한다는 것도 같이 못박는다.
    assert elapsed < 10.0
