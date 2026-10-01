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
    seq = ramp(-103, 70, 0, -103)
    state, stop, reason = run(seq)
    assert reason == "복귀" and stop is not None
    assert seq[stop - 1] < RET


# ---------------------------------------------------------------------------
# 복귀는 바닥까지 (2026-10-01: 문턱에서 끊으면 idle 이동이 남은 몫을 메우며 그리퍼를 "살짝 든다")
# ---------------------------------------------------------------------------
def test_return_plays_down_to_the_bottom_not_just_past_the_threshold():
    seq = np.concatenate([ramp(-103, 70, -60), np.linspace(-60, -104, 40), np.full(30, -104.0)])
    first_below = int(np.argmax(seq < RET))
    _state, stop, reason = run(seq)
    assert reason == "복귀"
    assert stop > first_below + 1, "문턱을 처음 넘는 곳에서 끊으면 안 된다"
    assert seq[stop - 1] == pytest.approx(-104.0, abs=0.6), "바닥(-104)까지 재생"
    assert stop < len(seq), "바닥에서 평평하면 청크 끝까지 기다리지 않는다"


def test_return_stops_at_the_bottom_before_the_next_attempt_rises():
    """10-01 soccer: 청크 6 이 -96 까지 내려갔다가 -88 로 다시 오른다 — 바닥 -96 에서 끊는다."""
    seq = np.concatenate([ramp(-35, -96, n=40), ramp(-96, -88, n=20)])
    state = (True, True, 66.0, None, 0)                    # 이미 뻗었다 내려오는 중
    _state, stop, reason = scan_cycle(seq, state, EXT, DROP, RISE, RET)
    assert reason == "복귀"
    assert seq[stop - 1] == pytest.approx(-96.0, abs=0.1)
    assert max(seq[:stop]) <= -35.0 and seq[stop - 1] == min(seq)


def test_settling_stops_before_the_policy_opens_the_gripper():
    """10-01 soccer: 접힌 자세에서 새로 받은 청크(-96 -> -102 -> -77)를 바닥까지 틀었더니 정책이
    그리퍼를 열어 공을 놓쳤다. 열기 시작하는 스텝 전에 멈춘다."""
    lift = np.concatenate([np.linspace(-96, -102, 20), np.linspace(-102, -77, 80)])
    grip = np.concatenate([np.full(5, 8.0), np.linspace(8, 45, 15), np.full(80, 45.0)])
    state = (True, True, 66.0, None, 0)
    _s, stop, reason = scan_cycle(lift, state, EXT, DROP, RISE, RET, gripper_cmd=grip, gripper_now=24.0)
    assert reason == "복귀"
    assert stop <= 6, f"그리퍼가 열리기 전에 멈춰야 한다 (stop={stop})"
    assert max(grip[:stop]) <= 8.0 + 2.0


def test_settling_with_a_closed_gripper_still_reaches_the_bottom():
    lift = np.concatenate([np.linspace(-96, -102, 20), np.linspace(-102, -77, 80)])
    grip = np.full(100, 6.0)
    state = (True, True, 66.0, None, 0)
    _s, stop, reason = scan_cycle(lift, state, EXT, DROP, RISE, RET, gripper_cmd=grip, gripper_now=24.0)
    assert reason == "복귀" and lift[stop - 1] == pytest.approx(-102.0, abs=0.4)


def test_new_chunk_that_already_opens_at_the_threshold_plays_nothing_more():
    """청크를 받을 때 실측 10% 인데 새 청크 첫 명령이 이미 30% — 한 스텝도 틀지 않는다."""
    lift = np.linspace(-96, -102, 30)
    grip = np.full(30, 30.0)
    state = (True, True, 66.0, None, 0)
    _s, stop, reason = scan_cycle(lift, state, EXT, DROP, RISE, RET, gripper_cmd=grip, gripper_now=10.0)
    assert (reason, stop) == ("복귀", 0)


def test_return_that_is_still_falling_at_the_chunk_end_plays_the_whole_chunk():
    seq = np.concatenate([ramp(-103, 70, -60), np.linspace(-60, -100, 30)])
    _state, stop, reason = run(seq)
    assert reason == "복귀" and stop == len(seq)


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
