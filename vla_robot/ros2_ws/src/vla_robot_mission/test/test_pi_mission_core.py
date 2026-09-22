from vla_common.motion_limits import MotionLimits
from vla_common.protocol import HostCommand, State
from vla_robot_mission.pi_mission_core import PiMissionCore


class FakeJobs:
    def __init__(self, fail_start=False):
        self.busy = False
        self.started = []
        self.cancelled = 0
        self._finished = None
        self.fail_start = fail_start

    def start(self, job_id, action, label, arm_yaw_deg=0.0):
        if self.fail_start:
            raise RuntimeError("arm offline")
        self.started.append((job_id, action, label, arm_yaw_deg))
        self.busy = True

    def finish(self, ok=True, detail="done"):
        job_id = self.started[-1][0]
        self.busy = False
        self._finished = (job_id, ok, detail)

    def poll(self):
        done, self._finished = self._finished, None
        return done

    def cancel(self):
        self.cancelled += 1


def make(fail_start=False):
    jobs = FakeJobs(fail_start)
    core = PiMissionCore(MotionLimits(0.15, 0.5), watchdog_s=0.5, boot_id="b", jobs=jobs)
    return core, jobs


def test_no_command_means_stop_and_watchdog():
    core, _ = make()
    out = core.step(0.0, True)
    assert out.motion.is_stop and out.status.watchdog


def test_drive_is_clamped():
    core, _ = make()
    core.on_command(HostCommand(State.APPROACH, linear_x=1.0), 0.0)
    out = core.step(0.1, True)
    assert out.motion.linear_x == 0.15 and not out.status.watchdog


def test_watchdog_stops_after_silence():
    core, _ = make()
    core.on_command(HostCommand(State.APPROACH, linear_x=0.1), 0.0)
    assert not core.step(0.4, True).motion.is_stop
    out = core.step(0.6, True)
    assert out.motion.is_stop and out.status.watchdog


def test_grasp_starts_once_on_transition_and_result_repeats():
    core, jobs = make()
    core.on_command(HostCommand(State.APPROACH, linear_x=0.1), 0.0)
    core.step(0.0, True)
    core.on_command(HostCommand(State.GRASP, label="queen"), 0.1)
    out = core.step(0.1, True)
    assert jobs.started == [(1, State.GRASP, "queen", 0.0)]
    assert out.status.busy and out.motion.is_stop
    # 작업 중에는 주행 명령이 무시된다
    core.on_command(HostCommand(State.APPROACH, linear_x=0.1), 0.2)
    out = core.step(0.2, True)
    assert out.motion.is_stop and out.status.state == State.GRASP
    jobs.finish(ok=True)
    core.on_command(HostCommand(State.GRASP), 0.3)
    out = core.step(0.3, True)
    assert out.status.result.job_id == 1 and out.status.result.ok
    # 같은 상태를 계속 보내도 다시 시작하지 않고 결과는 반복된다
    for t in (0.4, 0.5):
        core.on_command(HostCommand(State.GRASP), t)
        out = core.step(t, True)
    assert len(jobs.started) == 1 and out.status.result.job_id == 1


def test_start_failure_becomes_failed_result():
    core, _ = make(fail_start=True)
    core.on_command(HostCommand(State.GRASP), 0.0)
    out = core.step(0.0, True)
    assert out.status.result is not None and not out.status.result.ok
    assert out.status.result.job_id == 1 and not out.status.busy


def test_estop_cancels_running_job():
    core, jobs = make()
    core.on_command(HostCommand(State.PLACE), 0.0)
    core.step(0.0, True)
    core.on_command(HostCommand(State.ESTOP), 0.1)
    out = core.step(0.1, True)
    assert jobs.cancelled == 1 and out.status.state == State.ESTOP


def test_latched_estop_overrides_everything_until_reset():
    core, _ = make()
    core.latch_estop()
    core.on_command(HostCommand(State.APPROACH, linear_x=0.1), 0.0)
    assert core.step(0.0, True).motion.is_stop
    core.reset_estop()
    core.on_command(HostCommand(State.APPROACH, linear_x=0.1), 0.1)
    assert not core.step(0.1, True).motion.is_stop


def test_mixed_rotation_rejected_with_detail():
    core, _ = make()
    core.on_command(HostCommand(State.APPROACH, linear_x=0.1, angular_z=0.3), 0.0)
    out = core.step(0.0, True)
    assert out.motion.is_stop and "거부" in out.status.detail


def test_job_runs_through_link_loss():
    core, jobs = make()
    core.on_command(HostCommand(State.GRASP), 0.0)
    core.step(0.0, True)
    out = core.step(5.0, True)          # 명령이 끊겨도
    assert jobs.cancelled == 0 and out.status.busy


def test_place_carries_the_arm_yaw_from_the_host():
    """PLACE 명령에 실린 각도가 작업 시작까지 그대로 간다 — 팔이 그만큼 base 를 튼다."""
    core, jobs = make()
    core.on_command(HostCommand(State.PLACE, arm_yaw_deg=-12.5), 0.0)
    core.step(0.0, True)
    assert jobs.started == [(1, State.PLACE, "", -12.5)]
