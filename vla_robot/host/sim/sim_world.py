"""차량·카메라 없이 Host 전체 흐름을 돌리는 시뮬레이터.

`SimWorld` 가 가짜 pose·기물 지도를 주고, `SimWorld.link` 는 UdpVehicleLink 와 같은
모양(send / latest_status / status_age_s / close)으로 **Pi 계약을 흉내낸다**:

- 작업은 GRASP/PLACE 로 **전이하는 순간** 시작한다(같은 상태가 반복돼도 재시작 안 함).
- 작업 중에는 ESTOP 외 명령을 무시한다(바퀴는 정지).
- 마지막 결과를 매 상태 패킷에 반복한다. boot_id 는 인스턴스마다 다르다.
- watchdog_s 동안 명령이 없으면 정지한다.
- 속도는 vla_common.motion_limits 로 Pi 와 같은 규칙으로 자른다.

시계는 주입한다 — 테스트는 가짜 시계로 수천 사이클을 순식간에 돌린다.
"""
from __future__ import annotations

import math
import random
import time
import uuid
from dataclasses import dataclass
from typing import Callable, Optional

from localization.pose import Pose
from vla_common.motion_limits import MotionLimits, resolve_motion
from vla_common.protocol import HostCommand, JobResult, PiStatus, State

XY = tuple[float, float]


@dataclass
class SimPiece:
    label: str
    x: float
    y: float
    held: bool = False
    in_box: Optional[str] = None


DEFAULT_PIECES = (("queen", 0.55, 0.95), ("star", 1.25, 0.80))


class SimWorld:
    def __init__(self, cfg, pieces=DEFAULT_PIECES, start=(0.90, 0.45, 90.0),
                 clock: Callable[[], float] = time.monotonic, grasp_s: float = 3.0,
                 place_s: float = 2.0, grasp_reach_m: float = 0.40,
                 pos_noise_m: float = 0.002, yaw_noise_deg: float = 0.3,
                 watchdog_s: float = 0.5, seed: int = 0) -> None:
        self.cfg = cfg
        self.clock = clock
        self.pieces = [SimPiece(l, x, y) for l, x, y in pieces]
        self.x, self.y, self.yaw_deg = start
        self.grasp_s, self.place_s, self.grasp_reach_m = grasp_s, place_s, grasp_reach_m
        self.pos_noise_m, self.yaw_noise_deg = pos_noise_m, yaw_noise_deg
        self.watchdog_s = watchdog_s
        self.limits = MotionLimits(max_linear_mps=0.15, max_angular_rad_s=0.5)
        self._rng = random.Random(seed)
        self._t = clock()
        # Pi 흉내 상태
        self.boot_id = uuid.uuid4().hex[:8]
        self.pi_state = State.IDLE
        self._vel = (0.0, 0.0, 0.0)
        self._last_cmd_t: Optional[float] = None
        self._seq = 0
        self._job_id = 0
        self._job: Optional[tuple[str, float]] = None       # (action, 끝나는 시각)
        self._result: Optional[JobResult] = None
        self._detail = ""
        self.link = _SimLink(self)

    # -- 세계 --------------------------------------------------------------
    def update(self) -> None:
        now = self.clock()
        dt = max(0.0, now - self._t)
        self._t = now
        if self._job is not None and now >= self._job[1]:
            self._finish_job()
        watchdog = self._last_cmd_t is None or now - self._last_cmd_t > self.watchdog_s
        vx, vy, wz = (0.0, 0.0, 0.0) if (watchdog or self._job is not None) else self._vel
        th = math.radians(self.yaw_deg)
        self.x += (vx * math.cos(th) - vy * math.sin(th)) * dt
        self.y += (vx * math.sin(th) + vy * math.cos(th)) * dt
        self.yaw_deg = (self.yaw_deg + math.degrees(wz) * dt + 180.0) % 360.0 - 180.0
        for p in self.pieces:
            if p.held:
                p.x, p.y = self.x, self.y

    def pose(self) -> Pose:
        n = self._rng.gauss
        return Pose(self.x + n(0, self.pos_noise_m), self.y + n(0, self.pos_noise_m),
                    self.yaw_deg + n(0, self.yaw_noise_deg), ok=True, n_cams=2, fresh=True)

    def piece_map(self) -> dict[str, list[XY]]:
        out: dict[str, list[XY]] = {}
        for p in self.pieces:
            if not p.held:
                out.setdefault(p.label, []).append((p.x, p.y))
        return out

    def pieces_in_box(self, box: str) -> list[str]:
        return [p.label for p in self.pieces if p.in_box == box]

    # -- Pi 흉내 ----------------------------------------------------------
    def _receive(self, cmd: HostCommand) -> None:
        now = self.clock()
        self._last_cmd_t = now
        self._seq = cmd.seq
        if cmd.state == State.ESTOP:
            if self._job is not None:
                self._result = JobResult(self._job_id, self._job[0], False, "cancelled by ESTOP")
                self._job = None
            self._vel = (0.0, 0.0, 0.0)
            self.pi_state = State.ESTOP
            return
        if self._job is not None:
            return  # 작업 중에는 무시
        entering = cmd.state in State.JOB_STATES and cmd.state != self.pi_state
        self.pi_state = cmd.state
        if entering:
            self._job_id += 1
            self._job = (cmd.state, now + (self.grasp_s if cmd.state == State.GRASP else self.place_s))
            self._vel = (0.0, 0.0, 0.0)
            return
        if cmd.state in State.JOB_STATES:
            self._vel = (0.0, 0.0, 0.0)
            return
        d = resolve_motion(cmd.linear_x, cmd.linear_y, cmd.angular_z, cmd.stop, self.limits)
        self._detail = d.reason
        self._vel = (d.motion.linear_x, d.motion.linear_y, d.motion.angular_z)

    def _finish_job(self) -> None:
        action, _ = self._job
        self._job = None
        if action == State.GRASP:
            if any(p.held for p in self.pieces):
                self._result = JobResult(self._job_id, action, False, "already holding")
                return
            free = [p for p in self.pieces if not p.held and p.in_box is None]
            near = min(free, key=lambda p: math.hypot(p.x - self.x, p.y - self.y), default=None)
            if near is None or math.hypot(near.x - self.x, near.y - self.y) > self.grasp_reach_m:
                self._result = JobResult(self._job_id, action, False, "nothing within reach")
                return
            near.held = True
            self._result = JobResult(self._job_id, action, True, f"holding {near.label}")
        else:
            held = next((p for p in self.pieces if p.held), None)
            if held is None:
                self._result = JobResult(self._job_id, action, False, "gripper empty")
                return
            name, (bx, by, _yaw) = min(self.cfg.arena.boxes.items(),
                                       key=lambda kv: math.hypot(kv[1][0] - self.x, kv[1][1] - self.y))
            held.held, held.in_box, held.x, held.y = False, name, bx, by
            self._result = JobResult(self._job_id, action, True, f"dropped {held.label} in {name}")

    def status(self) -> PiStatus:
        now = self.clock()
        watchdog = self._last_cmd_t is None or now - self._last_cmd_t > self.watchdog_s
        return PiStatus(boot_id=self.boot_id, state=self.pi_state, busy=self._job is not None,
                        job_id=self._job_id, result=self._result, base_ok=True,
                        watchdog=watchdog, ack_seq=self._seq, detail=self._detail)


class _SimLink:
    def __init__(self, world: SimWorld) -> None:
        self._w = world
        self._seq = 0

    def send(self, cmd: HostCommand) -> None:
        from dataclasses import replace
        self._seq += 1
        self._w._receive(replace(cmd, seq=self._seq))

    def latest_status(self) -> PiStatus:
        return self._w.status()

    def status_age_s(self) -> float:
        return 0.0

    def close(self) -> None:
        pass
