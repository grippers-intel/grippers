"""scan_cycle — 실기에서 본 궤적(2026-09-22 · 09-23 · 09-30)으로 고정한다."""
import numpy as np
import pytest

from vla_common.grasp_cycle import CYCLE_START, scan_cycle

EXT, DROP, RISE, RET = -50.0, 60.0, 30.0, -95.0


def ramp(*points, n=20):
    """구간별 직선으로 궤적을 만든다."""
    out = []
    for a, b in zip(points, points[1:]):
        out.append(np.linspace(a, b, n))
    return np.concatenate(out)


def run(seq, state=CYCLE_START):
    return scan_cycle(seq, state, EXT, DROP, RISE, RET)


def run_chunks(*chunks):
    """청크를 차례로 넣고 (몇 번째 청크, stop_at, reason) 을 돌려준다."""
    state = CYCLE_START
    for k, c in enumerate(chunks):
        state, stop, reason = run(np.asarray(c, dtype=float), state)
        if stop is not None:
            return k, stop, reason
    return None, None, None


def test_idle_only_is_not_a_cycle():
    # 첫 청크: 시작 자세에서 가만히 있는다 (실측 -103 ~ -104)
    state, stop, reason = run(ramp(-103, -104, -103))
    assert stop is None and state[1] is False


def test_reach_then_return_ends_the_cycle():
    state, stop, reason = run(ramp(-103, 70, 0, -103))
    assert reason == "복귀" and stop is not None
    assert ramp(-103, 70, 0, -103)[stop] < RET


def test_up_down_up_inside_one_chunk_is_caught():
    # 오탐을 냈던 실측 패턴: 한 청크 안에서 -104 -> 상승 -> -104 -> 29
    state, stop, reason = run(ramp(-104, 20, -104, 29))
    assert reason == "복귀", "한 청크 안에서 끝나는 사이클도 잡아야 한다"


def test_state_carries_across_chunks():
    k, stop, reason = run_chunks(ramp(-103, 29), ramp(42, 74), ramp(74, -10, -103))
    assert (k, reason) == (2, "복귀"), "단조 상승을 새 시도로 세면 안 된다"


def test_retry_2026_09_22_dip_minus_66():
    # 실패 회차 청크별: -103(idle) → 14 → 74 → 2 → -66 → 35 → 75
    k, stop, reason = run_chunks(ramp(-104, 14), ramp(14, 74), ramp(74, 2),
                                 ramp(2, -66), ramp(-66, 35), ramp(35, 75))
    assert reason == "재상승" and k in (3, 4)


def test_retry_2026_09_30_dip_minus_58_is_caught():
    """절대값(-60) 기준으로는 놓친 회차 — 정책이 세 번째 시도까지 갔다."""
    chunks = [ramp(-103, -104), ramp(-104, 16), ramp(21, 64), ramp(72, 3),
              ramp(14, -39), ramp(-51, -58, 35), ramp(46, 73)]
    k, stop, reason = run_chunks(*chunks)
    assert reason == "재상승"
    assert k == 5, "다시 뻗는 청크(6번째)에서 멈춰야 한다"
    assert chunks[5][stop] == pytest.approx(-58, abs=1.0), "골짜기 바닥에서 끊어야 다시 뻗지 않는다"


def test_chunk_boundary_bump_is_not_a_retry():
    # 09-30: 청크 끝 3 -> 다음 청크 첫 값 14 (+11). 내려가는 중의 경계 튐이다.
    k, stop, reason = run_chunks(ramp(-104, 16), ramp(21, 72), ramp(72, 3), ramp(14, -39),
                                 ramp(-40, -103))
    assert reason == "복귀"


def test_successful_grasps_from_the_logs_all_end_by_returning():
    # Pi 로그의 성공 회차들(청크별 처음/최소/최대/끝을 이은 궤적)
    logs = [
        [(-104, 53), (59, 74, 73), (69, -103)],
        [(-104, -33), (-36, 66), (75, 78), (76, -22), (-40, -103, -69)],
        [(-104, 35), (41, 63), (69, -25), (-38, -103, -88)],
        [(-104, 30), (31, 73), (79, 72), (78, -24), (-30, -100, -65)],
        [(-104, 14), (17, 70), (78, 40), (54, -98)],
    ]
    for chunks in logs:
        k, stop, reason = run_chunks(*[ramp(*c) for c in chunks])
        assert reason == "복귀", chunks


def test_retry_from_the_previous_chunk_plays_nothing():
    """골짜기가 이전 청크 끝에 있고 새 청크가 오르며 시작하면, 새 청크는 재생하지 않는다."""
    k, stop, reason = run_chunks(ramp(-104, 70), ramp(70, -60), ramp(-20, 40))
    assert (k, reason) == (2, "재상승")
    assert stop == 0
