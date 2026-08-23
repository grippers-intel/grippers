"""floor_consensus.py 테스트 — tools/perception/consensus.py(팀원 실기
튜닝본)를 그대로 불러와 HANDOFF.md 동작점 게이트(순도·산포·거리)만 검증한다.
numpy가 있어야 돈다(consensus.py 의존) — rclpy는 필요 없다."""

import pytest

np = pytest.importorskip("numpy")

from grippers_perception.floor_consensus import confirmed_tracks, track_bbox_xyxy  # noqa: E402


def _bbox(cx, bottom_y, w, h):
    return (cx - w / 2.0, bottom_y - h, cx + w / 2.0, bottom_y)


def test_consistent_high_confidence_object_is_confirmed():
    """10프레임 내내 같은 자리에 나오는 물체 — HANDOFF.md 검증 케이스와
    같은 모양(작은 산포, 높은 순도, 화면 하단)."""
    frames = [[("rook", 0.9, _bbox(320, 400, 40, 60))] for _ in range(10)]

    tracks = confirmed_tracks(frames, n_frames=10)

    assert len(tracks) == 1
    assert tracks[0].label == "rook"


def test_low_purity_track_is_rejected():
    """같은 위치에서 클래스가 절반씩 갈리면(순도 0.5) 헷갈리는 검출이라 뺀다."""
    frames = []
    for i in range(10):
        cls = "rook" if i % 2 == 0 else "queen"
        frames.append([(cls, 0.9, _bbox(320, 400, 40, 60))])

    assert confirmed_tracks(frames, n_frames=10) == []


def test_high_spread_track_is_rejected():
    """위치가 프레임마다 크게 흔들리면(산포 > 40px) 신뢰하지 않는다."""
    frames = [
        [("rook", 0.9, _bbox(320 + (i % 2) * 100, 400, 40, 60))] for i in range(10)
    ]

    assert confirmed_tracks(frames, n_frames=10) == []


def test_track_above_distance_gate_is_rejected():
    """HANDOFF.md 거리 게이트 — 바닥 접점 y가 290 미만이면(너무 멀다) 뺀다.
    빈 바닥 대조군 오탐 최대 y=277, 진짜 물체 최소 y=293 실측."""
    frames = [[("rook", 0.9, _bbox(320, 277, 40, 60))] for _ in range(10)]

    assert confirmed_tracks(frames, n_frames=10) == []


def test_track_below_k_of_n_ratio_is_rejected():
    """10프레임 중 5프레임에서만 보이면(k-of-n 0.6 미만) 오탐 취급."""
    frames = [[("rook", 0.9, _bbox(320, 400, 40, 60))] for _ in range(5)]
    frames += [[] for _ in range(5)]

    assert confirmed_tracks(frames, n_frames=10) == []


def test_low_mean_confidence_track_is_rejected():
    """순도가 만점이어도(전부 같은 클래스) 평균 신뢰도가 낮으면(<0.35) 뺀다 —
    floor_observer.py의 min_support_conf 게이트."""
    frames = [[("rook", 0.36, _bbox(320, 400, 40, 60))] for _ in range(10)]
    assert len(confirmed_tracks(frames, n_frames=10)) == 1  # 경계 확인용(0.36 > 0.35)

    frames_below = [[("rook", 0.34, _bbox(320, 400, 40, 60))] for _ in range(10)]
    assert confirmed_tracks(frames_below, n_frames=10) == []


@pytest.mark.parametrize("unreliable_cls", ["box", "star"])
def test_unreliable_classes_are_excluded_even_when_otherwise_confirmed(unreliable_cls):
    """2026-08-23 실측: box(60프레임 중 0회 검출)·star(신뢰도 0.31)는
    허용목록 밖이다 — 다른 게이트를 전부 통과해도 제외한다."""
    frames = [[(unreliable_cls, 0.9, _bbox(320, 400, 40, 60))] for _ in range(10)]
    assert confirmed_tracks(frames, n_frames=10) == []


def test_track_bbox_xyxy_round_trips_through_center_and_size():
    frames = [[("rook", 0.9, _bbox(320, 400, 40, 60))] for _ in range(10)]
    (track,) = confirmed_tracks(frames, n_frames=10)

    assert track_bbox_xyxy(track) == pytest.approx((300.0, 340.0, 340.0, 400.0))
