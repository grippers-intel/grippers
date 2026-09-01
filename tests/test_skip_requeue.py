"""포기한 기물(MissionFSM.skipped)을 다시 후보로 돌리는 조건 — 사용자 지시,
2026-09-01.

## 배경

`_skip_target()`이 실패한 기물 좌표를 `skipped`에 영구히 남겨서, 원래는
그 좌표가 미션이 끝날 때까지(reset() 전까지) 다시는 후보에 안 뽑혔다 —
같은 기물 앞에서 재정렬/재시도만 무한 반복하는 것을 막기 위해서였다.

그런데 그 기물 말고 후보가 아예 없어지면(다른 기물이 전부 처리됐거나,
애초에 그것 하나뿐이었으면) 영원히 손을 놓게 된다. 그래서 "다른 후보가
남아 있는 동안은 skip을 존중하고, 그것 말고 후보가 하나도 없을 때만
skip을 무시하고 다시 뽑는다"로 바꿨다(_nearest_piece/_find_label의
2단계 탐색).

여기는 `mission.py` 를 통째로 돌리지 않고 그 두 순수 함수만 직접
검증한다 — 이 판단 자체는 FSM 상태 전이와 무관한 선택 로직이라서다.
"""

from __future__ import annotations

import sys
from pathlib import Path

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

import mission_config as mcfg                          # noqa: E402
from mission import _nearest_piece, _find_label         # noqa: E402

ROBOT = (1.0, 1.0)


# ── _nearest_piece ──────────────────────────────────────────────────────


def test_스킵된_기물_말고_다른_후보가_있으면_스킵을_그대로_존중한다():
    """rook 이 더 가깝지만 skip 대상이다 — 더 먼 queen 이 남아 있으니
    queen 을 골라야 한다(rook 을 또 들이밀면 안 된다)."""
    piece_map = {"rook": [(1.02, 1.02)], "queen": [(0.3, 1.2)]}
    found = _nearest_piece(piece_map, ROBOT, skip=[(1.02, 1.02)])
    assert found == ("queen", (0.3, 1.2))


def test_스킵되지_않은_후보가_하나도_없으면_스킵을_무시하고_다시_뽑는다():
    """필드에 rook 하나뿐이고 그마저 skip 대상이다 — 그래도 손을 놓지
    않고 그 rook 을 다시 후보로 준다."""
    piece_map = {"rook": [(1.02, 1.02)]}
    found = _nearest_piece(piece_map, ROBOT, skip=[(1.02, 1.02)])
    assert found == ("rook", (1.02, 1.02))


def test_스킵이_없으면_평소처럼_최근접을_고른다():
    piece_map = {"rook": [(1.02, 1.02)], "queen": [(0.3, 1.2)]}
    found = _nearest_piece(piece_map, ROBOT, skip=None)
    assert found == ("rook", (1.02, 1.02))


# ── _find_label ──────────────────────────────────────────────────────────


def test_라벨의_다른_개체가_남아_있으면_스킵된_개체는_안_돌려준다():
    """폰이 두 개고 하나는 skip 대상이다 — 남은 개체를 골라야 한다."""
    piece_map = {"rook": [(1.02, 1.02), (0.5, 0.9)]}
    found = _find_label(piece_map, "rook", ROBOT, skip=[(1.02, 1.02)])
    assert found == ("rook", (0.5, 0.9))


def test_라벨의_유일한_개체가_스킵됐으면_그것을_다시_돌려준다():
    """지시받은 라벨의 유일한 개체가 한 번 포기됐다 — 그것 말고는 이
    라벨에 후보가 없으니, 영원히 기다리지 않고 그것을 다시 준다."""
    piece_map = {"queen": [(0.3, 1.2)]}
    found = _find_label(piece_map, "queen", ROBOT, skip=[(0.3, 1.2)])
    assert found == ("queen", (0.3, 1.2))


def test_다른_라벨이_남아있어도_이_라벨_기준으로만_판단한다():
    """rook 지시 중인데 rook 은 유일하고 skip 대상, queen 은 멀쩡히
    남아 있다 — _find_label 은 rook 만 본다(다른 라벨은 SEARCH_TARGET
    이 지시를 소비하기 전까지 애초에 후보가 아니다), 그러니 skip 을
    무시하고 그 rook 을 다시 돌려줘야 한다."""
    piece_map = {"rook": [(1.02, 1.02)], "queen": [(0.3, 1.2)]}
    found = _find_label(piece_map, "rook", ROBOT, skip=[(1.02, 1.02)])
    assert found == ("rook", (1.02, 1.02))


def test_라벨이_안_보이면_스킵과_무관하게_None이다():
    found = _find_label({}, "queen", ROBOT, skip=[(0.3, 1.2)])
    assert found is None
