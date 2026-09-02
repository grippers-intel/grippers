"""사용자 지시(자연어 -> 라벨)로 "가장 가까운 기물" 규칙을 덮어쓰는 경로
(사용자 지시, 2026-09-01 — 팀원의 2026-08-31 handoff 델타
`set_instruction()`을 이 저장소의 현재 구조(GridPathPlanner·GRASP_ALIGN·
RETURN_HOME 등, 그 델타 이후에 추가된 것들)에 맞춰 이식).

instruction_resolver.py(Claude API 호출)는 여기서 검증하지 않는다 — 여기는
"라벨(+목적지 오버라이드)이 이미 정해졌을 때 mission.py 가 옳게 반응하는가"
만 본다. API 해석 자체는 이 프로세스 밖(수동 확인)이다.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

import mission_config as mcfg                       # noqa: E402
from mission import MissionFSM, State                # noqa: E402

from conftest import PiSim                            # noqa: E402

# ⚠️ 2026-09-02, 시연용으로 PLACE 완료 뒤 SEARCH_TARGET 으로 곧장 가지 않고
# RETURN_HOME 을 한 번 거치도록 바뀌어 사이클이 늘었다(test_basket_close_
# loop.py 의 MAX_STEPS 주석 참고) — 여기는 그 위에 두 번째 기물까지
# 오가므로 더 넉넉히 둔다.
MAX_STEPS = 900


class AutoDonePi(PiSim):
    """GRASP 요청마다 즉시 완료로 답하는 Pi — 지시 라우팅만 보고 싶을 때
    파지 세부는 신경 쓰지 않기 위함. PLACE 는 그대로 PiSim 의 실제 바구니
    판정(_judge_insert)에 맡긴다 — 그래야 실제로 도착해서 내려놓는다."""

    def poll_status(self) -> str:
        last_status = self.sent[-1][1] if self.sent else None
        if last_status in ("GRASP", "GRASP_FORCE"):
            return "GRASP_DONE"
        return super().poll_status()


def _run_until(fsm, link, pmap, predicate, max_steps=MAX_STEPS):
    for n in range(1, max_steps + 1):
        fsm.step(link.pose(), pmap, link)
        if predicate(fsm):
            return n
    pytest.fail(f"{max_steps} 사이클 안에 조건에 도달하지 못했다 — 상태 {fsm.state.name}")


def _two_piece_map():
    """rook 이 로봇(1.0, 1.0)에 더 가깝고 queen 이 더 멀다 — 아무 지시도
    없으면 rook 이 최근접으로 뽑힌다는 것을 지시가 뒤집는지 확인하는 용도."""
    return {"rook": [(1.05, 1.05)], "queen": [(0.3, 1.2)]}


# ── 즉시 적용 (손이 비어 있음) ───────────────────────────────────────────


def test_라벨_지시가_최근접_규칙을_덮어쓴다():
    fsm = MissionFSM()
    link = AutoDonePi(x=1.0, y=1.0)
    pmap = _two_piece_map()

    applied = fsm.set_instruction("queen")
    assert applied is True   # SEARCH_TARGET 은 손이 빈 상태 — 즉시 적용

    fsm.step(link.pose(), pmap, link)
    assert fsm.target_label == "queen"   # rook 이 더 가까운데도 지시를 따른다


def test_organize_기본값이면_기존_라벨별_상자로_간다():
    fsm = MissionFSM()
    link = AutoDonePi(x=1.0, y=1.0)
    pmap = _two_piece_map()

    fsm.set_instruction("queen")   # dest_xy 안 줌 = organize
    fsm.step(link.pose(), pmap, link)

    dest_box = mcfg.PIECE_DEST_BOX["queen"]
    expected = fsm.dest_xy
    # _box_front_xy 를 다시 계산해도 같은 값이어야 한다(= 오버라이드가 아님).
    from mission import _box_front_xy
    assert expected == _box_front_xy(dest_box)


def test_fetch_의도면_목적지가_DELIVER_HERE_XY_다():
    fsm = MissionFSM()
    link = AutoDonePi(x=1.0, y=1.0)
    pmap = _two_piece_map()

    fsm.set_instruction("queen", dest_xy=mcfg.DELIVER_HERE_XY)
    fsm.step(link.pose(), pmap, link)

    assert fsm.dest_xy == mcfg.DELIVER_HERE_XY


def test_안_보이는_라벨을_지시하면_그_라벨이_보일_때까지_기다린다():
    fsm = MissionFSM()
    link = AutoDonePi(x=1.0, y=1.0)
    pmap = _two_piece_map()   # "box" 라벨은 없음

    fsm.set_instruction("box")
    for _ in range(5):
        fsm.step(link.pose(), pmap, link)
        assert fsm.state == State.SEARCH_TARGET
        assert not fsm.ready_to_advance


# ── 손이 안 비었을 때는 큐에 쌓인다 ──────────────────────────────────────


def test_손이_안_비었으면_큐에_쌓이고_즉시_적용되지_않는다():
    fsm = MissionFSM()
    fsm.state = State.CARRY_TO_DEST   # 이미 뭔가 든 것으로 시늉
    fsm.target_label = "rook"
    fsm._target_xy = (1.0, 1.0)
    fsm.dest_xy = (1.271, 1.30)

    applied = fsm.set_instruction("queen")

    assert applied is False
    assert fsm.target_label == "rook"   # 지금 하던 일은 안 바뀐다
    assert fsm._instructed_label is None
    assert fsm._queued_instruction_label == "queen"


def test_큐에_쌓인_지시는_PLACE_완료_후_적용된다():
    fsm = MissionFSM()
    link = AutoDonePi(x=1.271, y=1.0, yaw_deg=90.0)
    pmap = {"rook": [(1.0, 1.0)], "queen": [(0.3, 1.2)]}

    # rook 을 먼저 잡아 나르는 중.
    _run_until(fsm, link, pmap, lambda f: f.state == State.CARRY_TO_DEST)
    assert fsm.target_label == "rook"

    # 그 도중 새 지시 — 손이 안 비었으니 큐에 쌓인다.
    applied = fsm.set_instruction("queen", dest_xy=mcfg.DELIVER_HERE_XY)
    assert applied is False

    # rook 을 마저 상자에 넣고 SEARCH_TARGET 으로 돌아올 때까지 진행.
    _run_until(fsm, link, pmap,
              lambda f: f.state == State.APPROACH_PIECE and f.target_label == "queen")

    assert fsm.dest_xy == mcfg.DELIVER_HERE_XY


# ── 뒤로가기는 즉시 적용된 지시를 취소한다 ─────────────────────────────


def test_뒤로가기로_SEARCH_TARGET까지_가면_지시도_취소된다():
    fsm = MissionFSM()
    link = AutoDonePi(x=1.0, y=1.0)
    pmap = _two_piece_map()

    fsm.set_instruction("queen")
    fsm.step(link.pose(), pmap, link)   # APPROACH_PIECE(queen) 로 즉시 적용
    assert fsm.state == State.APPROACH_PIECE
    assert fsm.target_label == "queen"

    fsm.request_back()
    fsm.step(link.pose(), pmap, link)   # -> SEARCH_TARGET

    assert fsm.state == State.SEARCH_TARGET
    assert fsm._instructed_label is None
    assert fsm._instructed_dest_xy is None
