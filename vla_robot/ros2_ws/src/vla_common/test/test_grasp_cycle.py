"""scan_cycle — 2026-09-22 실기에서 본 궤적으로 고정한다."""
import numpy as np
import pytest

from vla_common.grasp_cycle import scan_cycle

EXT, DIP, RET = -50.0, -60.0, -95.0
START = (False, False, None, 0)


def ramp(*points, n=20):
    """구간별 직선으로 궤적을 만든다."""
    out = []
    for a, b in zip(points, points[1:]):
        out.append(np.linspace(a, b, n))
    return np.concatenate(out)


def run(seq, state=START):
    return scan_cycle(seq, state, EXT, DIP, RET)


def test_idle_only_is_not_a_cycle():
    # 첫 청크: 시작 자세에서 가만히 있는다 (실측 -103 ~ -104)
    state, stop, reason = run(ramp(-103, -104, -103))
    assert stop is None and state[1] is False


def test_reach_then_return_ends_the_cycle():
    state, stop, reason = run(ramp(-103, 70, 0, -103))
    assert reason == "복귀" and stop is not None
    # 멈추는 지점의 명령값이 복귀 문턱 아래여야 한다
    assert ramp(-103, 70, 0, -103)[stop] < RET


def test_shallow_dip_then_rise_stops_at_the_bottom():
    # 실측 실패 회차: 골짜기 -74 까지만 내려왔다가 다시 오른다
    seq = ramp(-9, -74, -39)
    state, stop, reason = run(seq, state=(True, True, None, 0))
    assert reason == "재상승"
    assert seq[stop] == pytest.approx(seq.min(), abs=1.0), "바닥에서 멈춰야 다시 뻗지 않는다"


def test_up_down_up_inside_one_chunk_is_caught():
    # 오탐을 냈던 실측 패턴: 한 청크 안에서 -104 -> 상승 -> -104 -> 29
    state, stop, reason = run(ramp(-104, 20, -104, 29))
    assert reason == "복귀", "한 청크 안에서 끝나는 사이클도 잡아야 한다"


def test_state_carries_across_chunks():
    # 청크 1: 뻗기만 한다
    state, stop, _ = run(ramp(-103, 29))
    assert stop is None and state[1] is True
    # 청크 2: 계속 오른다 — 사이클은 아직 안 끝났다 (여기서 오탐이 났었다)
    state, stop, _ = run(ramp(42, 74), state=state)
    assert stop is None, "단조 상승을 새 시도로 세면 안 된다"
    # 청크 3: 복귀
    state, stop, reason = run(ramp(74, -10, -103), state=state)
    assert reason == "복귀"
