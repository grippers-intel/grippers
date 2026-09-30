"""차체가 명령을 따르는지 탑뷰로 본다 — "움직이라고 했는데 안 움직인다".

보드가 Pi 의 쓰기를 **오류 없이 무시**하는 고장이 있다(2026-09-23 · 09-30). Pi 쪽에서는
아무것도 실패하지 않으므로 Pi 는 모른다. 2026-09-30 로그: 반시계 0.5 rad/s 를 10 초 보냈는데
yaw 가 89.6° 에서 0.3° 도 안 변했다. 이런 판단은 로봇 위치를 실제로 재는 Host 만 할 수 있다.
"""
from __future__ import annotations

import math
from typing import Optional

from localization.pose import Pose
from planning.planner import wrap_deg
from vla_common.protocol import HostCommand, State


class BaseStallMonitor:
    """움직임 명령이 stall_s 동안 이어졌는데 위치·방향이 거의 그대로면 True."""

    def __init__(self, stall_s: float, move_m: float, turn_deg: float) -> None:
        self.stall_s = stall_s
        self.move_m = move_m
        self.turn_deg = turn_deg
        self.reset()

    def reset(self) -> None:
        self._since: Optional[float] = None
        self._ref: Optional[tuple[float, float, float]] = None
        self.moved_since_reset = False

    @staticmethod
    def _commands_motion(cmd: HostCommand) -> bool:
        if cmd.stop or cmd.state not in State.DRIVE_STATES:
            return False
        return abs(cmd.linear_x) > 1e-6 or abs(cmd.linear_y) > 1e-6 or abs(cmd.angular_z) > 1e-6

    def update(self, now: float, cmd: HostCommand, pose: Pose) -> bool:
        if not self._commands_motion(cmd) or not (pose.ok and pose.fresh):
            # 정지 명령·pose 없음이면 판단하지 않는다(창을 새로 연다)
            self._since, self._ref = None, None
            return False
        cur = (pose.x, pose.y, pose.yaw_deg)
        if self._since is None:
            self._since, self._ref = now, cur
            return False
        moved = math.hypot(cur[0] - self._ref[0], cur[1] - self._ref[1])
        turned = abs(wrap_deg(cur[2] - self._ref[2]))
        if moved > self.move_m or turned > self.turn_deg:
            # 움직이고 있다. 창을 여기서 다시 연다.
            self._since, self._ref = now, cur
            self.moved_since_reset = True
            return False
        return now - self._since >= self.stall_s
