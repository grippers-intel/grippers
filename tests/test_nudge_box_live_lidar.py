"""NUDGE_BOX가 실시간 라이다 신호로 계획 거리 전에 멈춘다 (2026-09-02 실기).

## 왜 이 기능이 생겼나

09-02 실기 2건: NUDGE_BOX가 ArUco 데드레커닝으로 계산한 계획 거리
(want_m)를 다 채울 때까지 Pi 보고를 하나도 안 읽었다 — PLACE에 들어가서야
`link.poll_status()`를 처음 불러 라이다로 확인했다. 계획이 실제와 어긋나면
(ArUco 오차) 확인 시점엔 이미 바구니에 닿아 있었다.

Pi는 이제 APPROACH_BOX 접근 중에도 매 사이클 라이다를 보고, 이미 목표창
안이면 `APPROACH_BOX_READY`를, 이미 너무 가까우면(하한 아래) `INSERT_BLOCKED`
+ retreat 보정을 실시간으로 보낸다(baseline_mission.BaselineCarryState 참고).
이 파일은 Host의 NUDGE_BOX가 그 신호를 받으면 `moved >= want_m`을 기다리지
않고 즉시 PLACE로 넘어가는지만 검증한다 — 실제 라이다/파싱은 Pi 쪽 도메인
테스트가 따로 본다."""

from __future__ import annotations

import sys
from pathlib import Path

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

import mission_config as mcfg                                    # noqa: E402
from mission import MissionFSM, State                             # noqa: E402
from vehicle_link import BasketFix                                 # noqa: E402

from conftest import PiSim                                         # noqa: E402


def _nudge_fsm() -> tuple[MissionFSM, PiSim]:
    fsm = MissionFSM()
    fsm.state = State.NUDGE_BOX
    fsm.target_label = "rook"
    link = PiSim(x=1.0, y=1.0, yaw_deg=mcfg.BOX_FACE_YAW_DEG)
    return fsm, link


def test_신호가_없으면_계획_거리를_채울_때까지_NUDGE_BOX에_머문다():
    """회귀 방지 — 이 기능이 평소 동작(거리 다 채우고 넘어가기)을 건드리면
    안 된다."""
    fsm, link = _nudge_fsm()

    fsm.step(link.pose(), {}, link)

    assert fsm.state == State.NUDGE_BOX


def test_목표창_안이라는_신호를_받으면_계획_거리_전에_바로_PLACE로_넘어간다():
    fsm, link = _nudge_fsm()
    link.basket_ready_early = True

    fsm.step(link.pose(), {}, link)

    assert fsm.state == State.PLACE
    # 소비됐으니 다음 판정에 또 쓰이면 안 된다.
    assert link.basket_ready_early is False


def test_너무_가깝다는_보정을_받으면_계획_거리_전에_바로_PLACE로_넘어간다():
    fsm, link = _nudge_fsm()
    link.last_basket_fix = BasketFix(forward_m=-0.02)

    fsm.step(link.pose(), {}, link)

    assert fsm.state == State.PLACE
    # PLACE 가 이걸 소비해서 다음 보정을 계산해야 한다 — NUDGE_BOX 는
    # 넘어가는 신호로만 쓰고 지우지 않는다.
    assert link.last_basket_fix is not None


def test_너무_멀다는_보정은_PLACE의_잔여_보고로_보고_무시한다():
    """09-02 실기 3번째 재발 재현 — 회귀 방지.

    라이브 점검(baseline_mission.BaselineCarryState)이 접근 중에 실제로
    보내는 것은 `retreat_if_too_close` 뿐이라 forward_m 이 항상 음수다.
    양수 forward_m 은 직전 PLACE(INSERT 판정)에서 아직 도착 중이던
    오래된 보고일 수 있다 — 그걸 "지금 온 라이브 신호"로 오인해 멈추면
    실제로는 안 움직였는데 `_basket_creep_used` 예산만 청구돼, 몇 번
    반복으로 예산이 바닥나 이후 INSERT_BLOCKED 에서 영영 못 빠져나온다
    (09-02 09:45 실기가 정확히 이랬다)."""
    fsm, link = _nudge_fsm()
    link.last_basket_fix = BasketFix(forward_m=0.025)

    fsm.step(link.pose(), {}, link)

    assert fsm.state == State.NUDGE_BOX
    # 소비하지 않았으니 PLACE 가 도착하면 그대로 봐야 한다.
    assert link.last_basket_fix is not None
