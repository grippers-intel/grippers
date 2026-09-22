import json

import pytest

from vla_common.protocol import (HostCommand, JobResult, JobTracker, PiStatus,
                                 ProtocolError, State)


def test_host_command_roundtrip():
    cmd = HostCommand(State.APPROACH, linear_x=0.15, angular_z=0.0, label="queen", seq=7)
    assert HostCommand.from_bytes(cmd.to_bytes()) == cmd


@pytest.mark.parametrize("patch", [
    {"v": 99},
    {"state": "FLY"},
    {"linear_x": "fast"},
    {"linear_x": float("nan")},
    {"stop": "yes"},
    {"label": "x" * 100},
])
def test_host_command_rejects_bad_packets(patch):
    body = json.loads(HostCommand(State.IDLE).to_bytes())
    body.update(patch)
    raw = json.dumps(body).encode()
    with pytest.raises(ProtocolError):
        HostCommand.from_bytes(raw)


def test_garbage_is_rejected():
    with pytest.raises(ProtocolError):
        HostCommand.from_bytes(b"\xff\x00not json")


def _status(job_id=0, result=None, boot="A", busy=False):
    return PiStatus(boot_id=boot, state=State.IDLE, busy=busy, job_id=job_id,
                    result=result, base_ok=True, watchdog=False)


def test_status_roundtrip():
    st = _status(3, JobResult(2, State.GRASP, True, "ok"))
    assert PiStatus.from_bytes(st.to_bytes()) == st


def test_job_tracker_ignores_old_result_and_accepts_new():
    tracker = JobTracker()
    before = _status(job_id=2, result=JobResult(2, State.GRASP, True))
    tracker.arm(State.GRASP, before)
    # 같은 옛 결과가 반복돼도 완료가 아니다
    assert tracker.poll(before) is None
    assert tracker.poll(_status(job_id=3, busy=True, result=JobResult(2, State.GRASP, True))) is None
    done = tracker.poll(_status(job_id=3, result=JobResult(3, State.GRASP, False, "empty")))
    assert done is not None and done.job_id == 3 and not done.ok
    # 한 번 넘겨준 뒤에는 무장 해제
    assert not tracker.armed


def test_job_tracker_ignores_other_action():
    tracker = JobTracker()
    tracker.arm(State.PLACE, _status())
    assert tracker.poll(_status(job_id=1, result=JobResult(1, State.GRASP, True))) is None


def test_job_tracker_armed_before_any_status_ignores_stale_result():
    tracker = JobTracker()
    tracker.arm(State.GRASP, None)
    stale = _status(job_id=4, result=JobResult(4, State.GRASP, True))
    assert tracker.poll(stale) is None           # 옛 결과는 기준이 된다
    assert tracker.poll(_status(job_id=5, busy=True, result=JobResult(4, State.GRASP, True))) is None
    done = tracker.poll(_status(job_id=5, result=JobResult(5, State.GRASP, True)))
    assert done is not None and done.job_id == 5


def test_status_rejects_non_integer_ack_seq():
    body = json.loads(_status().to_bytes())
    body["ack_seq"] = "x"
    with pytest.raises(ProtocolError):
        PiStatus.from_bytes(json.dumps(body).encode())


def test_job_tracker_handles_pi_restart():
    tracker = JobTracker()
    tracker.arm(State.GRASP, _status(job_id=9, result=JobResult(9, State.GRASP, True), boot="A"))
    # Pi 재시작: 번호가 1 부터 다시
    done = tracker.poll(_status(job_id=1, result=JobResult(1, State.GRASP, True), boot="B"))
    assert done is not None and done.job_id == 1
