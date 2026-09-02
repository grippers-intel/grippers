"""PLACE가 바구니를 아예 못 찾으면(REACQUIRE) 오버헤드로 다시 접근한다
(10:41 실기, 2026-09-02).

## 왜 이 기능이 생겼나

10:41 실기: 차가 바구니가 아니라 옆의 **벽**을 보고 서 있었다. Pi의
`corrections.from_insert`는 라이다 평면 자체를 못 찾으면(`face_ok=False`)
방향 없는 `REACQUIRE`를 보낸다 — "무엇이 얼마나 어긋났는지"가 없으니
Host의 `_plan_basket_fix`(전후/좌우/회전 국소 보정)는 애초에 계획을 세울
수가 없다. 그런데 그전까지는 이 신호를 받고도 아무것도 안 하고 그냥
버렸다 — PLACE가 INSERT_BLOCKED만 반복하며 영원히 갇혔다.

사용자 지적: "그때는 Host에서 복구를 해야 하지 않나?" — GRASP_REPLAN과
같은 이유(Pi의 좁은 정면 센서 대신 Host가 아는 좌표로 되돌아간다)로,
REACQUIRE가 일정 횟수 연속 오면 CARRY_TO_DEST로 되돌아가 오버헤드
카메라 기준 dest_xy를 향해 크게 다시 접근한다."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

import mission_config as mcfg                                    # noqa: E402
from mission import MissionFSM, State                             # noqa: E402
from vehicle_link import BasketFix                                 # noqa: E402

from conftest import PiSim                                         # noqa: E402


@dataclass
class LostPi(PiSim):
    """바구니 정면을 아예 못 찾는 Pi 시늉.

    `PiSim._judge_insert()`는 "거리가 멀다"/"좌우로 밀려 있다"만 시늉하고
    "정면 자체를 못 찾음"(REACQUIRE, face_ok=False)은 시늉할 수 없다 —
    그래서 `send()`를 직접 오버라이드해 PLACE 명령마다 `lost` 표시만
    구조화된 fix로 낸다. 방향이 있는 보정으로 회복되는 시나리오
    (test_중간에_한_번이라도...)를 위해 `recovers_after`를 두면 그 이후엔
    평소 PiSim._judge_insert()로 넘어간다."""

    recovers_after: int | None = None
    _place_calls: int = 0

    def send(self, cmd):
        self.sent.append((cmd.cmd, cmd.status))
        self._move(cmd)
        if cmd.status != "PLACE":
            return
        self._place_calls += 1
        if self.recovers_after is not None and self._place_calls > self.recovers_after:
            self._judge_insert()
            return
        self.last_basket_fix = BasketFix(lost=True)
        self._pending = "BUSY"


def _place_fsm(**link_kwargs) -> tuple[MissionFSM, LostPi]:
    fsm = MissionFSM()
    fsm.state = State.PLACE
    fsm.target_label = "rook"
    fsm.dest_xy = (1.271, 1.30)
    link = LostPi(x=1.27, y=1.25, yaw_deg=mcfg.BOX_FACE_YAW_DEG, **link_kwargs)
    return fsm, link


def test_한두_번은_그냥_기다린다():
    """회귀 방지 — 잠깐의 흔들림까지 매번 크게 재접근하면 안 된다."""
    fsm, link = _place_fsm()

    for _ in range(mcfg.BASKET_LOST_REPLAN_AFTER_TRIES - 1):
        link.last_basket_fix = BasketFix(lost=True)
        fsm.step(link.pose(), {}, link)
        assert fsm.state == State.PLACE


def test_문턱을_넘기면_CARRY_TO_DEST로_다시_접근한다():
    fsm, link = _place_fsm()

    for _ in range(mcfg.BASKET_LOST_REPLAN_AFTER_TRIES):
        link.last_basket_fix = BasketFix(lost=True)
        fsm.step(link.pose(), {}, link)

    assert fsm.state == State.CARRY_TO_DEST
    # 재접근 예산은 깨끗하게 다시 시작한다 — 벽을 보고 있었을 때 쓴
    # 국소 보정 예산이 진짜 접근에 넘어가면 안 된다.
    assert fsm._basket_creep_used == 0.0
    assert fsm._basket_yaw_used == 0.0
    assert fsm._basket_lost_tries == 0


def test_중간에_한_번이라도_방향이_있는_보정을_받으면_문턱이_리셋된다():
    """진짜로 못 찾는 게 아니라 잠깐 흔들린 것이었다면, Pi가 스스로 다시
    찾아 방향 있는 보정(예: 거리)을 보내는 순간 처음부터 다시 세야 한다."""
    fsm, link = _place_fsm(recovers_after=1)

    fsm.step(link.pose(), {}, link)
    assert fsm._basket_lost_tries == 1

    fsm.step(link.pose(), {}, link)
    assert fsm._basket_lost_tries == 0
    assert fsm.state != State.CARRY_TO_DEST


def test_재접근은_dest_xy를_향해_다시_주행한다():
    fsm, link = _place_fsm()

    for _ in range(mcfg.BASKET_LOST_REPLAN_AFTER_TRIES):
        link.last_basket_fix = BasketFix(lost=True)
        fsm.step(link.pose(), {}, link)
    assert fsm.state == State.CARRY_TO_DEST

    fsm.step(link.pose(), {}, link)
    assert fsm.nav_goal == fsm.dest_xy
