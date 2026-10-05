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


class BaseRunawayMonitor:
    """직진을 시키지 않았는데 로봇이 달리고 있으면 True — "서라/돌라고 했는데 계속 간다".

    같은 고장(보드가 쓰기를 무시)이 **달리는 중에** 나면 마지막 속도를 그대로 유지한다.
    2026-09-30: 상자 앞에서 제자리 회전만 보냈는데 8초 동안 1 m 를 곧장 달려 장판 밖으로
    나갔고, 사람이 잡은 뒤에도 바퀴가 계속 돌았다. 정지 명령도 먹지 않으므로 컨트롤러를
    다시 띄워(보드 리셋) 세우는 수밖에 없다 — 무응답 감지와 같은 복구를 쓴다.

    마지막 병진 명령에서 grace_s 가 지났는데 최근 window_s 동안 move_m 넘게 움직였으면 폭주다.
    grace_s 는 멈추라고 한 뒤 관성·지연으로 더 가는 몫이다. 팔 작업(GRASP/PLACE) 중에는 보지
    않는다 — 마커가 팔에 붙어 있어 팔을 펴면 마커가 30 cm 가까이 움직인다.
    """

    def __init__(self, move_m: float, window_s: float, grace_s: float) -> None:
        self.move_m = move_m
        self.window_s = window_s
        self.grace_s = grace_s
        self.reset()

    def reset(self) -> None:
        self._last_linear_at: Optional[float] = None
        self._poses: list[tuple[float, float, float]] = []

    @staticmethod
    def _commands_translation(cmd: HostCommand) -> bool:
        return not cmd.stop and (abs(cmd.linear_x) > 1e-6 or abs(cmd.linear_y) > 1e-6)

    def update(self, now: float, cmd: HostCommand, pose: Pose, watching: bool) -> bool:
        """watching: 지금 폭주를 볼 상태인가(주행 단계·ESTOP). 아니면 창을 비운다."""
        if not watching or cmd.state in State.JOB_STATES or not (pose.ok and pose.fresh):
            self._poses.clear()
            if self._commands_translation(cmd):
                self._last_linear_at = now
            return False
        if self._commands_translation(cmd) or self._last_linear_at is None:
            # 병진 중이거나 막 시작했다. 기준 시각만 적는다(처음 보는 경우도 방금 병진한 것으로 친다).
            self._last_linear_at = now
            self._poses.clear()
            return False
        self._poses.append((now, pose.x, pose.y))
        self._poses = [p for p in self._poses if now - p[0] <= self.window_s]
        if now - self._last_linear_at < self.grace_s:
            return False
        t0, x0, y0 = self._poses[0]
        if now - t0 < self.window_s * 0.8:
            return False                       # 창이 아직 덜 찼다
        return math.hypot(pose.x - x0, pose.y - y0) > self.move_m


class BaseSpinMonitor:
    """돌라고 하지 않았는데(또는 반대로 돌라고 했는데) 로봇이 돌고 있으면 True — 회전 폭주.

    같은 고장이 **회전 중에** 나면 마지막 회전 속도를 그대로 유지한다. 2026-10-05: 투하 뒤 다음
    기물로 돌던 중 반시계 약 8°/s 로 굳었다. Host 는 yaw- / yaw+ 를 번갈아 보냈고 ESTOP 도
    걸었지만 10 s 마다 +80° 로 계속 돌았다. 위치는 몇 cm 밖에 안 변해 BaseRunawayMonitor
    (위치만 본다)가 못 잡았고, 보드 리셋 없이 사람이 전원을 내렸다.

    회전 명령의 방향(없음 · + · -)이 바뀐 지 grace_s 가 지난 뒤, 최근 window_s 동안
      - 회전 명령이 없는데(정지·ESTOP) turn_deg 넘게 돌았거나
      - 명령과 **반대 방향으로** turn_deg 넘게 돌았으면
    폭주다. 병진 중에는 보지 않는다(옆걸음·직진 중 방향 보정과 섞인다). 팔 작업 중에도 보지
    않는다 — 마커가 팔에 붙어 있어 팔 base 가 돌면 마커 방향도 돈다.
    """

    def __init__(self, turn_deg: float, window_s: float, grace_s: float) -> None:
        self.turn_deg = turn_deg
        self.window_s = window_s
        self.grace_s = grace_s
        self.reset()

    def reset(self) -> None:
        self._sign: Optional[int] = None       # 지금 회전 명령 방향(0 = 없음). None = 새로 본다
        self._since = 0.0
        self._yaws: list[tuple[float, float]] = []

    @staticmethod
    def _rot_sign(cmd: HostCommand) -> int:
        if cmd.stop or abs(cmd.angular_z) <= 1e-6:
            return 0
        return 1 if cmd.angular_z > 0 else -1

    def update(self, now: float, cmd: HostCommand, pose: Pose, watching: bool) -> bool:
        """watching: 지금 폭주를 볼 상태인가(주행 단계·ESTOP). 아니면 창을 비운다."""
        translating = not cmd.stop and (abs(cmd.linear_x) > 1e-6 or abs(cmd.linear_y) > 1e-6)
        if (not watching or cmd.state in State.JOB_STATES or translating
                or not (pose.ok and pose.fresh)):
            self._sign = None
            self._yaws.clear()
            return False
        sign = self._rot_sign(cmd)
        if sign != self._sign:
            # 명령 방향이 바뀌었다(처음 보는 경우 포함) — 관성으로 이전 방향으로 더 도는 몫을 grace 로 둔다
            self._sign, self._since = sign, now
            self._yaws.clear()
        self._yaws.append((now, pose.yaw_deg))
        self._yaws = [p for p in self._yaws if now - p[0] <= self.window_s]
        if now - self._since < self.grace_s:
            return False
        t0, yaw0 = self._yaws[0]
        if now - t0 < self.window_s * 0.8:
            return False                       # 창이 아직 덜 찼다
        turned = wrap_deg(pose.yaw_deg - yaw0)  # + = 반시계
        if sign == 0:
            return abs(turned) > self.turn_deg
        return sign * turned < -self.turn_deg
