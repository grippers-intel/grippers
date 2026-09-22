"""Pi 미션 코어 — 순수 로직(ROS·소켓 없음). 단위 테스트로 고정한다.

매 사이클 `step(now, base_alive)` 가 "지금 바퀴에 낼 속도"와 "Host 에 보낼 상태"를 낸다.

## 규칙 (우선순위 순)

1. **E-STOP** (ROS 토픽으로 래치됐거나 Host 가 ESTOP 을 보내는 중):
   진행 중인 팔 작업을 취소하고 정지.
2. **팔 작업 중**: 바퀴는 정지. Host 가 무엇을 보내든(ESTOP 제외) 작업을 끊지 않는다.
   GRASP 도중 차가 움직이면 정책 입력이 학습과 달라진다.
3. **Host 명령이 watchdog_s 넘게 끊김**: 정지. 마지막 명령대로 굴러가지 않는다.
   ("명령 없음"은 "정지하라"가 아니라 "모른다"이므로 워치독이 따로 처리한다.)
4. **GRASP / PLACE 로 새로 들어온 순간**에만 작업을 시작한다. 작업이 끝난 뒤 Host 가
   같은 상태를 계속 보내도 다시 시작하지 않는다 — 다시 시키려면 다른 상태를 거쳐야 한다.
   시작에 실패하면 즉시 실패 결과를 만든다(결과 경로가 하나뿐이도록).
5. **주행 상태**: 속도를 합의된 한계로 자른다. 회전+병진 혼합은 거부(정지 + 사유).

작업 결과는 다음 작업이 끝날 때까지 매 상태 패킷에 반복된다(protocol.JobTracker 참고).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol

from vla_common.motion_limits import STOP, Motion, MotionLimits, resolve_motion
from vla_common.protocol import HostCommand, JobResult, PiStatus, State


class JobRunner(Protocol):
    @property
    def busy(self) -> bool: ...

    def start(self, job_id: int, action: str, label: str) -> None:
        """작업을 비동기로 시작한다. 시작할 수 없으면 예외."""

    def poll(self) -> Optional[tuple[int, bool, str]]:
        """끝난 작업의 (job_id, ok, detail). 한 번만 돌려준다."""

    def cancel(self) -> None: ...


@dataclass(frozen=True)
class StepOutput:
    motion: Motion
    status: PiStatus


class PiMissionCore:
    def __init__(self, limits: MotionLimits, watchdog_s: float, boot_id: str, jobs: JobRunner) -> None:
        self._limits = limits
        self._watchdog_s = float(watchdog_s)
        self._boot_id = boot_id
        self._jobs = jobs

        self._cmd: Optional[HostCommand] = None
        self._cmd_time = 0.0
        self._prev_cmd_state: Optional[str] = None
        self._last_state = State.IDLE

        self._job_id = 0
        self._job_actions: dict[int, str] = {}
        self._running: Optional[tuple[int, str]] = None
        self._result: Optional[JobResult] = None
        self._estop_latched = False

    # -- 입력 ---------------------------------------------------------------
    def on_command(self, cmd: HostCommand, now: float) -> None:
        self._cmd, self._cmd_time = cmd, now

    def latch_estop(self) -> None:
        self._estop_latched = True

    def reset_estop(self) -> None:
        self._estop_latched = False

    @property
    def estop_latched(self) -> bool:
        return self._estop_latched

    @property
    def last_result(self) -> Optional[JobResult]:
        return self._result

    # -- 한 사이클 -----------------------------------------------------------
    def _collect_finished(self) -> None:
        finished = self._jobs.poll()
        if finished is None:
            return
        job_id, ok, detail = finished
        action = self._job_actions.pop(job_id, State.GRASP)
        self._result = JobResult(job_id, action, bool(ok), str(detail))
        if self._running and self._running[0] == job_id:
            self._running = None

    def _start_job(self, action: str, label: str) -> None:
        self._job_id += 1
        job_id = self._job_id
        self._job_actions[job_id] = action
        try:
            self._jobs.start(job_id, action, label)
            self._running = (job_id, action)
        except Exception as exc:  # noqa: BLE001 — 결과 경로를 하나로 모은다
            self._job_actions.pop(job_id, None)
            self._result = JobResult(job_id, action, False, f"작업 시작 실패: {exc}")

    def step(self, now: float, base_alive: bool) -> StepOutput:
        self._collect_finished()
        cmd = self._cmd
        fresh = cmd is not None and (now - self._cmd_time) <= self._watchdog_s
        motion: Motion = STOP
        detail = ""

        if self._estop_latched or (fresh and cmd.state == State.ESTOP):
            if self._jobs.busy:
                self._jobs.cancel()
            state = State.ESTOP
            detail = "E-STOP 래치" if self._estop_latched else "Host E-STOP"
            if fresh:
                self._prev_cmd_state = cmd.state
        elif self._jobs.busy:
            state = self._running[1] if self._running else self._last_state
            detail = "팔 작업 중 — 주행 명령 무시"
        elif not fresh:
            state = self._last_state
            detail = "Host 명령 없음 — 정지" if cmd is None else "Host 명령 끊김(워치독) — 정지"
        elif cmd.state in State.JOB_STATES:
            if cmd.state != self._prev_cmd_state:
                self._start_job(cmd.state, cmd.label)
            self._prev_cmd_state = cmd.state
            state = cmd.state
        else:
            decision = resolve_motion(cmd.linear_x, cmd.linear_y, cmd.angular_z, cmd.stop, self._limits)
            motion = decision.motion
            state = cmd.state
            if not decision.ok:
                detail = f"명령 거부: {decision.reason}"
            elif decision.clamped:
                detail = decision.reason
            self._prev_cmd_state = cmd.state

        if not base_alive and not motion.is_stop:
            detail = (detail + " / " if detail else "") + "구동계 응답 없음 — 차체 전원 확인"
        self._last_state = state
        status = PiStatus(
            boot_id=self._boot_id,
            state=state,
            busy=self._jobs.busy,
            job_id=self._job_id,
            result=self._result,
            base_ok=bool(base_alive),
            watchdog=not fresh,
            ack_seq=cmd.seq if cmd is not None else 0,
            detail=detail,
        )
        return StepOutput(motion=motion, status=status)
