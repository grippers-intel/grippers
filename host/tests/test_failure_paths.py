"""실패 경로 테스트 — 통주행으로는 절대 안 밟히는 자리들.

`run_sim.py` 통주행은 **모든 것이 잘 되는 경우**만 지나간다. Pi 가 실패나
차단을 보고했을 때 Host 가 무엇을 하는지는 여기서만 검증된다.

pytest 로도, 단독으로도 돌아간다:

    python tests/test_failure_paths.py
    pytest tests/test_failure_paths.py        # pytest 가 있으면
"""
from __future__ import annotations

import sys
from pathlib import Path

HOST = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HOST / "aruco"))
sys.path.insert(0, str(HOST))

import mission_config as mcfg
from localizer import Pose
from mission import MissionFSM, State
from vehicle_link import (BACK_OFF, CREEP_IN, RE_AIM, SHIFT, UNFIXABLE, WAIT,
                          GraspCorrection, MissionCommand, MissionState,
                          VehicleLink, classify_insert_correction,
                          correction_from_fix, encode)

POSE = Pose(x=1.0, y=0.8, yaw_deg=90.0, ok=True, n_cams=2, fresh=True)


class FakeLink(VehicleLink):
    """보낸 것을 모아두고, 미리 정해둔 보고를 돌려준다."""

    def __init__(self, status="IDLE", correction=None, insert_correction=None):
        self.sent: list[MissionCommand] = []
        self._status = status
        self.last_correction = correction
        self.last_insert_correction = insert_correction

    def send(self, cmd: MissionCommand) -> None:
        self.sent.append(cmd)

    def poll_status(self) -> str:
        status, self._status = self._status, "IDLE"   # 한 번만 준다
        return status


def _fsm_at(state: State, **kw) -> MissionFSM:
    fsm = MissionFSM()
    fsm.state = state
    fsm.target_label = "queen"
    fsm._target_xy = (1.2, 1.0)
    fsm.dest_xy = (0.4, 1.6)
    for k, v in kw.items():
        setattr(fsm, k, v)
    return fsm


# --- INSERT 사유 분류 ------------------------------------------------------

def test_insert_reasons_are_classified():
    """Pi `check_insert()` 의 실제 문구를 그대로 넣어 본다."""
    cases = [
        ("바구니가 멀다 (라이다 0.185m > 0.155m)", CREEP_IN),
        ("라이다 판독이 하한보다 가깝다 (0.120m < 0.128m)", BACK_OFF),
        ("정면 점이 부족하다 (18개 < 40개)", BACK_OFF),
        ("정렬이 틀어졌다 (yaw +0.142rad > 0.087rad)", RE_AIM),
        ("좌우로 밀려 있다 (+85mm > ±70mm)", SHIFT),
        ("E-STOP이 걸려 있다", UNFIXABLE),
        ("그리퍼가 비어 있다 (부하 0.0102 < 0.0469)", UNFIXABLE),
    ]
    for detail, want in cases:
        got = classify_insert_correction(detail).kind
        assert got == want, f"{detail!r} -> {got}, 기대 {want}"


def test_transient_reasons_do_not_move_the_robot():
    """기다리면 풀리는 사유에 대고 움직이면 판독이 또 흔들려 영영 안 맞는다."""
    for detail in ("차체가 아직 정지하지 않았다",
                   "직전 판독이 없다 — 한 사이클 더 확인해야 한다",
                   "판독이 흔들린다 (+12mm) — 아직 움직이는 중이거나 관측이 불안정하다"):
        c = classify_insert_correction(detail)
        assert c.kind == WAIT, detail
        assert c.transient is True
        assert c.actionable is False


def test_wait_wins_when_reasons_are_mixed():
    """사유가 여러 개 오면 '기다려라'가 이긴다 — 그때의 거리·yaw 값은 못 믿는다."""
    c = classify_insert_correction(
        "차체가 아직 정지하지 않았다; 바구니가 멀다 (라이다 0.185m > 0.155m)")
    assert c.kind == WAIT


def test_yaw_sign_is_read():
    assert classify_insert_correction("정렬이 틀어졌다 (yaw +0.142rad)").lateral_mm > 0
    assert classify_insert_correction("정렬이 틀어졌다 (yaw -0.142rad)").lateral_mm < 0


def test_reaim_without_direction_is_not_actionable():
    """어느 쪽으로 돌지 모르면 안 돈다 — 반대로 돌면 더 나빠진다."""
    assert classify_insert_correction("정렬이 틀어졌다").kind == UNFIXABLE


# --- 보정 슬롯 분리 (누수 버그) --------------------------------------------

def test_insert_correction_does_not_leak_into_grasp():
    """INSERT_BLOCKED 를 GRASP 가 집어가면 바구니 얘기로 기물을 움직인다."""
    link = FakeLink(insert_correction=GraspCorrection(BACK_OFF, "바구니 얘기"))
    fsm = _fsm_at(State.GRASP)
    fsm.step(POSE, {}, link)
    assert fsm.state == State.GRASP, "INSERT 보정으로 GRASP 가 움직였다"
    assert link.last_insert_correction is not None, "GRASP 가 소비해 버렸다"


def test_each_slot_is_consumed_once():
    link = FakeLink(correction=GraspCorrection(CREEP_IN, "재직진 필요"))
    assert link.take_correction() is not None
    assert link.take_correction() is None
    link.last_insert_correction = GraspCorrection(SHIFT, "좌우", 85.0)
    assert link.take_insert_correction() is not None
    assert link.take_insert_correction() is None


# --- GRASP 실패 ------------------------------------------------------------

def test_grasp_failed_skips_the_piece():
    link = FakeLink(status="FAILED")
    fsm = _fsm_at(State.GRASP)
    fsm.step(POSE, {}, link)
    assert fsm.state == State.SEARCH_TARGET
    assert (1.2, 1.0) in fsm.skipped, "보류 목록에 안 넣으면 같은 기물을 또 고른다"


def test_grasp_done_still_advances():
    """실패 분기를 넣다가 성공 경로를 깨지 않았는지."""
    link = FakeLink(status="GRASP_DONE")
    fsm = _fsm_at(State.GRASP)
    fsm.step(POSE, {}, link)
    assert fsm.state == State.CARRY_TO_DEST


# --- INSERT 실패 -----------------------------------------------------------

def test_insert_failed_retries_then_halts():
    """물체를 든 채라 보류할 수 없다 — 재시도하고, 예산을 다 쓰면 멈춘다."""
    fsm = _fsm_at(State.PLACE)
    for i in range(mcfg.INSERT_RETRY_MAX):
        fsm.state = State.PLACE
        fsm.step(POSE, {}, FakeLink(status="FAILED"))
        assert fsm.state == State.FACE_BOX, f"{i + 1}번째는 재시도해야 한다"
    fsm.state = State.PLACE
    fsm.step(POSE, {}, FakeLink(status="FAILED"))
    assert fsm.state == State.HALTED
    assert fsm.halt_reason is not None


def test_halted_keeps_sending_stop():
    """워치독에 걸려 Pi 가 멋대로 판단하지 않도록 명령은 계속 보낸다."""
    link = FakeLink()
    fsm = _fsm_at(State.HALTED)
    fsm.step(POSE, {}, link)
    assert fsm.state == State.HALTED
    assert link.sent and link.sent[-1].cmd == "stop"
    assert encode(link.sent[-1]).stop is True


def test_insert_blocked_goes_to_align():
    link = FakeLink(insert_correction=GraspCorrection(CREEP_IN, "바구니가 멀다"))
    fsm = _fsm_at(State.PLACE)
    fsm.step(POSE, {}, link)
    assert fsm.state == State.INSERT_ALIGN


def test_insert_blocked_transient_stays_in_place():
    link = FakeLink(insert_correction=GraspCorrection(WAIT, "차체가 아직 정지하지 않았다"))
    fsm = _fsm_at(State.PLACE)
    fsm.step(POSE, {}, link)
    assert fsm.state == State.PLACE, "기다려야 하는데 움직였다"


def test_insert_align_budget_halts():
    link = FakeLink(insert_correction=GraspCorrection(CREEP_IN, "바구니가 멀다"))
    fsm = _fsm_at(State.PLACE, _insert_align_tries=mcfg.INSERT_ALIGN_MAX_TRIES)
    fsm.step(POSE, {}, link)
    assert fsm.state == State.HALTED


def test_insert_unfixable_halts_not_skips():
    """손이 차 있으므로 SEARCH_TARGET 으로 돌아가면 안 된다."""
    link = FakeLink(insert_correction=GraspCorrection(UNFIXABLE, "E-STOP이 걸려 있다"))
    fsm = _fsm_at(State.PLACE)
    fsm.step(POSE, {}, link)
    assert fsm.state == State.HALTED


# --- 인코딩 규약 -----------------------------------------------------------

def test_encoder_never_mixes_rotation_and_translation():
    """Pi 는 섞인 명령을 REJECTED 로 되돌리고 정지한다."""
    for cmd in ("go", "back", "left", "right", "yaw+", "yaw-", "stop"):
        h = encode(MissionCommand(cmd, "INSERT_ALIGN", 0, 0, 0))
        moving = (h.linear_x != 0) or (h.linear_y != 0)
        assert not (moving and h.angular_z != 0), cmd


def test_new_states_map_to_pi_states():
    """모르는 상태면 encode 가 IDLE+stop 으로 떨어진다 — 그러면 안 움직인다."""
    assert encode(MissionCommand("go", "INSERT_ALIGN", 0, 0, 0)).state != MissionState.IDLE
    assert encode(MissionCommand("stop", "HALTED", 0, 0, 0)).state != MissionState.IDLE


# --- Pi 의 fix 필드 (정식 경로) ---------------------------------------------

def test_fix_actions_map_to_kinds():
    """Pi 가 판정 자리에서 만든 수치를 그대로 받는다 — 문장 파싱보다 우선한다."""
    cases = [
        ({"action": "wait", "lateral_m": 0.012}, False, WAIT),
        ({"action": "advance", "forward_m": 0.031}, False, CREEP_IN),
        ({"action": "retreat", "forward_m": -0.018}, True, BACK_OFF),
        ({"action": "reacquire"}, True, UNFIXABLE),
    ]
    for fix, insert, want in cases:
        got = correction_from_fix(fix, insert=insert)
        assert got is not None and got.kind == want, f"{fix} -> {got}"


def test_rotate_becomes_shift_only_at_the_basket():
    """같은 lateral_m 이라도 없애는 경로가 다르다.

    기물 앞은 회전(턱을 물체 쪽으로), 바구니 앞은 횡이동 — 회전하면 거리와
    yaw 가 같이 틀어져 여섯 조건을 동시에 흔들기 때문이다."""
    fix = {"action": "rotate", "lateral_m": 0.085}
    assert correction_from_fix(fix, insert=False).kind == RE_AIM
    assert correction_from_fix(fix, insert=True).kind == SHIFT


def test_rotate_with_yaw_is_always_reaim():
    c = correction_from_fix({"action": "rotate", "yaw_rad": 0.142}, insert=True)
    assert c.kind == RE_AIM and c.lateral_mm > 0     # 부호가 회전 방향이 된다


def test_unknown_fix_action_falls_back():
    """모르는 action 을 추측해서 움직이지 않는다 — None 이면 문장 파싱으로 간다."""
    assert correction_from_fix({"action": "허튼값"}, insert=True) is None


def test_numbers_survive_the_conversion():
    c = correction_from_fix({"action": "advance", "forward_m": 0.031}, insert=False)
    assert abs(c.forward_mm - 31.0) < 0.01


# --- APPROACH_BOX 속도 상한 --------------------------------------------------

def test_basket_states_use_the_slower_cap():
    """Pi 가 APPROACH_BOX 에서 0.06 으로 자른다. 같은 값을 보내야 도착 예측이 맞다."""
    from domain.task.motion import AGREED_LINEAR_MPS, BASKET_APPROACH_MPS
    for state in ("NUDGE_BOX", "INSERT_ALIGN"):
        h = encode(MissionCommand("go", state, 0, 0, 0))
        assert h.state == MissionState.APPROACH_BOX
        assert abs(h.linear_x - BASKET_APPROACH_MPS) < 1e-9, state
        assert abs(encode(MissionCommand("left", state, 0, 0, 0)).linear_y
                   - BASKET_APPROACH_MPS) < 1e-9, state
    for state in ("APPROACH_PIECE", "GRASP_ALIGN"):
        h = encode(MissionCommand("go", state, 0, 0, 0))
        assert abs(h.linear_x - AGREED_LINEAR_MPS) < 1e-9, state


if __name__ == "__main__":
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in fns:
        try:
            fn()
            print(f"  PASS  {name}")
        except AssertionError as exc:
            failed += 1
            print(f"  FAIL  {name}: {exc}")
        except Exception as exc:
            failed += 1
            print(f"  ERROR {name}: {type(exc).__name__}: {exc}")
    print(f"\n{len(fns) - failed}/{len(fns)} 통과")
    sys.exit(1 if failed else 0)
