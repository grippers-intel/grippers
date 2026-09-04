"""PieceTracker.suppress_at() — 파지/투입에 성공한 "바로 그 개체"만 지도
출력에서 숨기고, 같은 라벨의 다른 개체는 그대로 남기는지 (2026-09-05,
사용자 지시).

같은 라벨의 기물이 여러 개 있을 때(rook 두 개 등), GRASP_DONE/INSERT_DONE이
뜨면 그 대상이었던 개체만 다음부터 후보/LiveMap에서 빠져야 한다 — 라벨
전체를 지우면 안 된다. PieceTracker는 트랙을 위치로 구분하므로(모듈
docstring 참고), 백지 상태에서 시간 경과 없이 내부 `_Track`을 직접 심어
`update()`의 확정 대기 시간(PIECE_CONFIRM_SEC)에 얽매이지 않고 검증한다."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "host"))

from piece_map import PieceTracker, _Track   # noqa: E402


def _tracker_with(*tracks: _Track) -> PieceTracker:
    t = PieceTracker()
    t._tracks = list(tracks)
    return t


def _track(label: str, x: float, y: float, now: float = 100.0) -> _Track:
    tr = _Track(x, y, first_seen=now - 10.0, last_seen=now, n_obs=5)
    tr.label_scores[label] = 1.0
    return tr


def test_같은_라벨_두_개_중_가까운_것_하나만_숨긴다():
    near = _track("rook", 1.00, 1.00)
    far = _track("rook", 1.50, 1.00)
    tracker = _tracker_with(near, far)

    hit = tracker.suppress_at("rook", (1.01, 1.00))

    assert hit is True
    assert near.suppressed is True
    assert far.suppressed is False


def test_반경_밖이면_아무것도_안_숨기고_False를_돌려준다():
    far = _track("rook", 5.0, 5.0)
    tracker = _tracker_with(far)

    hit = tracker.suppress_at("rook", (0.0, 0.0))

    assert hit is False
    assert far.suppressed is False


def test_다른_라벨은_후보에서_안_빠진다():
    rook = _track("rook", 1.00, 1.00)
    queen = _track("queen", 1.00, 1.00)   # 같은 자리, 다른 라벨
    tracker = _tracker_with(rook, queen)

    tracker.suppress_at("rook", (1.00, 1.00))

    assert rook.suppressed is True
    assert queen.suppressed is False


def test_숨긴_트랙은_update의_출력에서_빠지지만_다른_개체는_남는다(monkeypatch):
    import time as time_mod
    now = 1000.0
    monkeypatch.setattr(time_mod, "monotonic", lambda: now)

    near = _track("rook", 1.00, 1.00, now=now)
    far = _track("rook", 1.50, 1.00, now=now)
    tracker = _tracker_with(near, far)
    tracker.suppress_at("rook", (1.00, 1.00))

    result = tracker.update([])

    xs = [xy[0] for xy in result.get("rook", [])]
    assert 1.00 not in xs, "숨긴 트랙이 출력에 다시 나타났다"
    assert 1.50 in xs, "숨기지 않은 다른 rook까지 같이 사라졌다"


def test_숨긴_뒤에도_관측이_계속_들어오면_트랙은_갱신되지만_계속_숨겨진다(monkeypatch):
    """트랙 자체는 지우지 않는다 — 옮겨지는 동안 잠깐 다시 잡혀도 숨김이
    풀리면 안 된다(클래스 docstring과 같은 철학)."""
    import time as time_mod
    now = 1000.0
    monkeypatch.setattr(time_mod, "monotonic", lambda: now)

    tracked = _track("rook", 1.00, 1.00, now=now)
    tracker = _tracker_with(tracked)
    tracker.suppress_at("rook", (1.00, 1.00))
    assert tracked.suppressed is True

    from piece_map import PieceObs
    result = tracker.update([[PieceObs("rook", 1.00, 1.00, 0.9, "cam0")]])

    assert tracked.suppressed is True
    assert result.get("rook", []) == []
