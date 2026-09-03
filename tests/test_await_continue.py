"""AWAIT_CONTINUE/AWAIT_COMMAND/IDLE — 그룹(chess/toy) 소진 시 계속/정지 확인
(2026-09-02, 사용자 지시).

PLACE 완료마다 묻지 않는다 — 방금 옮긴 라벨과 **같은 그룹**(chess: queen/
knight/rook, toy: star/soccer/box)의 다른 개체가 화면에 하나도 안 남았을
때만 AWAIT_CONTINUE 로 가서 "계속할지/정지할지" 사용자 응답을 기다린다.
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


def _run_until(fsm: MissionFSM, sim: PiSim, piece_map, states, max_steps=MAX_STEPS):
    """`states` 중 하나에 도달할 때까지 돌린다. 실패하면 예외."""
    for _ in range(max_steps):
        fsm.step(sim.pose(), piece_map, sim)
        if fsm.state in states:
            return fsm
    raise AssertionError(
        f"{max_steps} 사이클 안에 {[s.name for s in states]} 에 못 갔다 — "
        f"현재 {fsm.state.name}")


def test_그룹이_소진되면_RETURN_HOME_대신_AWAIT_CONTINUE로_간다():
    """rook(chess)을 넣었는데 화면에 다른 기물이 하나도 없다 — chess 그룹이
    이걸로 끝났으니 곧장 RETURN_HOME 으로 못 간다."""
    sim = PiSim()
    fsm = MissionFSM()
    assert fsm.begin_carrying("rook")
    _run_until(fsm, sim, {}, {State.AWAIT_CONTINUE})


def test_같은_그룹이_남아있으면_기존처럼_RETURN_HOME으로_간다():
    """rook(chess)을 넣었는데 knight(같은 chess 그룹)가 아직 화면에 있다 —
    그룹이 안 끝났으니 지금처럼 곧장 RETURN_HOME 으로 간다(질문 없음)."""
    sim = PiSim()
    fsm = MissionFSM()
    assert fsm.begin_carrying("rook")
    _run_until(fsm, sim, {"knight": [(0.9, 0.9)]}, {State.AWAIT_CONTINUE, State.RETURN_HOME})
    assert fsm.state == State.RETURN_HOME, (
        "같은 그룹이 남아 있는데도 AWAIT_CONTINUE 로 갔다 — 그룹마다 한 번만 "
        "물어야 하는데 개체마다 묻게 된 것")


def test_다른_그룹만_남아있어도_AWAIT_CONTINUE로_간다():
    """rook(chess)을 넣었을 때 star(toy, 다른 그룹)가 화면에 있어도 chess
    그룹 자체는 끝난 것 — 그룹 판정은 라벨 그룹별로 독립이어야 한다."""
    sim = PiSim()
    fsm = MissionFSM()
    assert fsm.begin_carrying("rook")
    _run_until(fsm, sim, {"star": [(0.9, 0.9)]}, {State.AWAIT_CONTINUE, State.RETURN_HOME})
    assert fsm.state == State.AWAIT_CONTINUE, (
        "다른 그룹(toy)이 남아 있다고 chess 그룹 소진 판정을 건너뛰었다")


def test_on_continue로_AWAIT_COMMAND로_넘어간다():
    sim = PiSim()
    fsm = MissionFSM()
    assert fsm.begin_carrying("rook")
    _run_until(fsm, sim, {}, {State.AWAIT_CONTINUE})
    fsm.on_continue()
    assert fsm.state == State.AWAIT_COMMAND


def test_submit_next_command은_RETURN_HOME을_생략하고_곧장_SEARCH_TARGET으로():
    """AWAIT_CONTINUE 도착 시점에 이미 기본 위치 근처이므로, 다음 명령은
    RETURN_HOME 을 다시 거치지 않고 바로 SEARCH_TARGET 으로 간다(사용자 결정)."""
    sim = PiSim()
    fsm = MissionFSM()
    assert fsm.begin_carrying("rook")
    _run_until(fsm, sim, {}, {State.AWAIT_CONTINUE})
    fsm.on_continue()
    assert fsm.state == State.AWAIT_COMMAND

    fsm.submit_next_command("soccer", dest_xy=None)
    assert fsm.state == State.SEARCH_TARGET
    assert fsm._instructed_label == "soccer"
    assert fsm._instructed_dest_xy is None


def test_submit_next_command은_fetch_의도의_목적지도_그대로_넘긴다():
    sim = PiSim()
    fsm = MissionFSM()
    assert fsm.begin_carrying("rook")
    _run_until(fsm, sim, {}, {State.AWAIT_CONTINUE})
    fsm.on_continue()

    fsm.submit_next_command("soccer", dest_xy=mcfg.DELIVER_HERE_XY)
    assert fsm.state == State.SEARCH_TARGET
    assert fsm._instructed_dest_xy == mcfg.DELIVER_HERE_XY


def test_on_stop은_RETURN_HOME을_거쳐_결국_IDLE로_간다():
    sim = PiSim()
    fsm = MissionFSM()
    assert fsm.begin_carrying("rook")
    _run_until(fsm, sim, {}, {State.AWAIT_CONTINUE})
    fsm.on_stop()
    assert fsm.state == State.RETURN_HOME

    _run_until(fsm, sim, {}, {State.IDLE})
    dist = ((sim.x - mcfg.DEFAULT_HOME_XY[0]) ** 2
            + (sim.y - mcfg.DEFAULT_HOME_XY[1]) ** 2) ** 0.5
    assert dist <= mcfg.HOME_ARRIVE_TOL_M


def test_IDLE은_영구_정지다_계속_돌려도_안_빠져나온다():
    """빠져나가는 경로가 없다는 게 이 상태의 정의다(사용자 결정)."""
    sim = PiSim()
    fsm = MissionFSM()
    assert fsm.begin_carrying("rook")
    _run_until(fsm, sim, {}, {State.AWAIT_CONTINUE})
    fsm.on_stop()
    _run_until(fsm, sim, {}, {State.IDLE})

    for _ in range(50):
        fsm.step(sim.pose(), {}, sim)
        assert fsm.state == State.IDLE
        assert fsm.last_cmd == "stop"


def test_엉뚱한_상태에서_부르면_아무_효과_없다():
    """on_continue/on_stop/submit_next_command 는 자기 상태가 아니면 무시한다
    — 방어적 가드가 실제로 막아 주는지 확인."""
    fsm = MissionFSM()   # 기본 SEARCH_TARGET
    fsm.on_continue()
    assert fsm.state == State.SEARCH_TARGET
    fsm.on_stop()
    assert fsm.state == State.SEARCH_TARGET
    fsm.submit_next_command("rook")
    assert fsm.state == State.SEARCH_TARGET
    assert fsm._instructed_label is None
