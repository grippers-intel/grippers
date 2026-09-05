"""CARRY_TO_DEST가 방금 파지한 기물 좌표를 장애물에서 빼는지 (2026-09-06).

## 배경

사용자가 실기 관찰을 보고했다: 이미 파지에 성공한 물체를 오버헤드
웹캠이 간헐적으로(잔상·오검출) 원래 판 위 자리에서 다시 감지하면,
GridPathPlanner가 그 유령 좌표를 피하려고 경로를 다시 잡아 불필요하게
버벅이며 회전했다가 다시 정상화되는 것처럼 보였다.

`_other_pieces()`는 `exclude_xy`를 주면 그 근처 한 점을 장애물 후보에서
뺀다 — APPROACH_PIECE(자기 목표를 스스로의 장애물로 보지 않으려고,
mission.py 892/903행)는 이미 그렇게 쓰고 있었다. 그런데 CARRY_TO_DEST만
`_other_pieces(piece_map)`을 exclude_xy 없이 불러서, 이미 그리퍼로 들고
있어 판 위에 없어야 할 `self._target_xy`가 매 사이클 장애물 후보에
그대로 다시 섞여 들어갔다.

이 파일은 그 인자가 실제로 넘어가는지, `_other_pieces` 호출을 가로채
확인한다."""

from __future__ import annotations

import sys
from pathlib import Path

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))

from mission import MissionFSM, State   # noqa: E402
import mission                          # noqa: E402

from conftest import PiSim              # noqa: E402


def test_CARRY_TO_DEST는_방금_집은_기물_좌표를_장애물에서_제외한다(monkeypatch):
    calls: list = []
    real_other_pieces = mission._other_pieces

    def _spy(piece_map, exclude_xy=None, tol=0.05):
        calls.append(exclude_xy)
        return real_other_pieces(piece_map, exclude_xy=exclude_xy, tol=tol)

    monkeypatch.setattr(mission, "_other_pieces", _spy)

    fsm = MissionFSM()
    fsm.state = State.CARRY_TO_DEST
    fsm.target_label = "queen"
    fsm._target_xy = (0.5, 0.5)          # 방금 집은 기물의 원래(판 위) 좌표
    fsm.dest_xy = (1.271, 1.30)
    fsm.dest_box_name = None

    link = PiSim(x=0.6, y=0.6, yaw_deg=90.0)
    # 웹캠이 방금 집은 그 좌표에서 유령을 다시 본 상황을 흉내낸다.
    ghost_piece_map = {"queen": [(0.5, 0.5)]}

    fsm.step(link.pose(), ghost_piece_map, link)

    assert calls, "CARRY_TO_DEST에서 _other_pieces가 호출되지 않았다"
    assert calls[0] == (0.5, 0.5), (
        f"exclude_xy로 방금 집은 기물 좌표(0.5, 0.5)가 안 넘어갔다 — {calls[0]!r} "
        "— APPROACH_PIECE처럼 CARRY_TO_DEST도 self._target_xy를 exclude_xy로 "
        "넘겨야 한다")


def test_CARRY_TO_DEST는_다른_기물은_여전히_장애물로_본다(monkeypatch):
    """exclude_xy는 좌표 하나만 빼야 한다 — 같은 라벨의 다른 개체나
    무관한 다른 기물까지 통째로 사라지면 실제 충돌 회피가 깨진다."""
    calls: list = []
    real_other_pieces = mission._other_pieces

    def _spy(piece_map, exclude_xy=None, tol=0.05):
        result = real_other_pieces(piece_map, exclude_xy=exclude_xy, tol=tol)
        calls.append(result)
        return result

    monkeypatch.setattr(mission, "_other_pieces", _spy)

    fsm = MissionFSM()
    fsm.state = State.CARRY_TO_DEST
    fsm.target_label = "queen"
    fsm._target_xy = (0.5, 0.5)
    fsm.dest_xy = (1.271, 1.30)
    fsm.dest_box_name = None

    link = PiSim(x=0.6, y=0.6, yaw_deg=90.0)
    piece_map = {"queen": [(0.5, 0.5)], "rook": [(0.9, 0.9)]}

    fsm.step(link.pose(), piece_map, link)

    assert calls
    assert (0.9, 0.9) in calls[0], "무관한 다른 기물(rook)까지 장애물에서 빠졌다"
    assert (0.5, 0.5) not in calls[0], "방금 집은 기물(queen) 유령이 여전히 장애물에 남아 있다"
