"""PLACE 완료 후 SEARCH_TARGET이 아니라 RETURN_HOME으로 간다 (시연용, 2026-09-02).

## 왜

바구니 바로 앞은 매번 접근 각도·거리가 조금씩 다른 자리다. PLACE가 끝나자마자
그 자리에서 SEARCH_TARGET을 시작하면 매 라운드가 다른 자리에서 시작돼
시연이 매번 다르게 보인다. `_skip_target`(기물을 포기했을 때)이 이미 쓰던
"RETURN_HOME을 한 번 거쳐 항상 같은 자리에서 다음 라운드를 시작한다"는
원칙을 PLACE 완료 경로에도 그대로 적용한다.
"""

from __future__ import annotations

import sys
from pathlib import Path

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

import mission_config as mcfg               # noqa: E402
from mission import MissionFSM, State        # noqa: E402

from conftest import PiSim                    # noqa: E402

MAX_STEPS = 900

# 2026-09-02, AWAIT_CONTINUE 신설로 이 테스트들의 전제가 바뀌었다: PLACE
# 완료 시 방금 옮긴 라벨(rook, chess 그룹)과 **같은 그룹의 다른 개체가
# 화면에 하나도 안 남으면** RETURN_HOME 대신 AWAIT_CONTINUE 로 간다(사용자
# 확인 대기). 이 파일의 테스트는 "PLACE -> RETURN_HOME -> SEARCH_TARGET"
# 루프 메커니즘 자체를 보는 것이지 그룹 소진 판정을 보는 게 아니므로,
# knight(같은 chess 그룹)가 하나 더 남아 있는 피지도를 줘서 그룹이 아직
# 안 끝난 것으로 만든다 — AWAIT_CONTINUE 자체는 test_await_continue.py 가
# 따로 검증한다.
_OTHER_CHESS_PIECE_REMAINS = {"knight": [(0.9, 0.9)]}


def test_PLACE_완료는_SEARCH_TARGET이_아니라_RETURN_HOME으로_간다():
    sim = PiSim()
    fsm = MissionFSM()
    assert fsm.begin_carrying("rook")

    # PLACE -> NUDGE_BOX 보정 왕복(정상 동작)과 진짜 완료를 구분해야 한다 —
    # "직전이 PLACE였고 지금이 RETURN_HOME"인 순간만 완료로 본다
    # (tests/test_basket_close_loop.py 의 _run_to_place_done 과 같은 이유).
    was_place = False
    for _ in range(MAX_STEPS):
        was_place = fsm.state == State.PLACE
        fsm.step(sim.pose(), _OTHER_CHESS_PIECE_REMAINS, sim)
        if was_place and fsm.state == State.RETURN_HOME:
            break
        if was_place and fsm.state not in (State.PLACE, State.NUDGE_BOX):
            raise AssertionError(
                f"PLACE에서 예상 밖의 상태({fsm.state.name})로 넘어갔다")
    else:
        raise AssertionError("PLACE가 RETURN_HOME으로 끝나지 않았다")


def test_RETURN_HOME을_거쳐_결국_SEARCH_TARGET에_도착한다():
    """중간에 한 번 쉬어 가는 것뿐, 다음 라운드로 계속 이어져야 한다."""
    sim = PiSim()
    fsm = MissionFSM()
    assert fsm.begin_carrying("rook")

    for n in range(1, MAX_STEPS + 1):
        fsm.step(sim.pose(), _OTHER_CHESS_PIECE_REMAINS, sim)
        if fsm.state == State.SEARCH_TARGET:
            break
    else:
        raise AssertionError(f"{MAX_STEPS} 사이클 안에 SEARCH_TARGET에 못 갔다 — "
                              f"상태 {fsm.state.name}")


def test_RETURN_HOME_도착지는_DEFAULT_HOME_XY다():
    """다음 라운드가 항상 같은 자리에서 시작된다는 것을 좌표로 고정한다."""
    sim = PiSim()
    fsm = MissionFSM()
    assert fsm.begin_carrying("rook")

    for _ in range(MAX_STEPS):
        fsm.step(sim.pose(), _OTHER_CHESS_PIECE_REMAINS, sim)
        if fsm.state == State.SEARCH_TARGET:
            break
    else:
        raise AssertionError("SEARCH_TARGET에 못 갔다")

    dist = ((sim.x - mcfg.DEFAULT_HOME_XY[0]) ** 2
            + (sim.y - mcfg.DEFAULT_HOME_XY[1]) ** 2) ** 0.5
    assert dist <= mcfg.HOME_ARRIVE_TOL_M
