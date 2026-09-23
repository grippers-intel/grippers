"""Host 미션 상태머신 — 탑뷰 pose + 기물 지도 -> 매 사이클 HostCommand 하나.

**순수 계산이다.** 소켓도 카메라도 모른다. `step()` 에 pose, 지도, 마지막 PiStatus,
현재 시각을 넣으면 이번 사이클에 보낼 명령을 돌려준다. 그래서 차량 없이 테스트된다.

## 매 사이클 반드시 명령 하나를 낸다

기존 코드는 pose 를 잃으면 명령을 아예 안 보냈다 — Pi 워치독이 세우긴 하지만 화면에는
아무것도 안 떠서 "Pi 문제"로 오해해 한참 헤맸다(2026-09-05). 여기서는
- 주행 상태에서 pose 를 잃으면: 그 상태의 전선 이름 + stop
- GRASP / PLACE 에서는 pose 와 무관하게 계속 GRASP / PLACE 를 보낸다. 팔이 마커를
  가리는 것은 정상이고, 상태가 바뀌면 Pi 쪽 작업 전이가 흔들린다.

## 팔 작업 완료 판정 — JobTracker

Pi 는 GRASP/PLACE 로 **전이하는 순간** 작업을 시작하고, 마지막 결과를 매 상태 패킷에
반복한다. 그래서 GRASP 를 처음 보내기 직전에 `JobTracker.arm()` 으로 기준 job_id 를
잡고, 그보다 새 결과가 보이면 완료로 본다. UDP 패킷이 빠져도 사건을 잃지 않는다.

## 기존 설계에서 뺀 것
GRASP_ALIGN / INSERT_ALIGN (Pi 가 뎁스·라이다로 "다시 세워 달라"고 요청하던 경로).
VLA 전용 파지로 바뀌며 Pi 는 보정 요청을 하지 않는다. 실패는 결과(ok=false)로만 온다.
"""
from __future__ import annotations

import math
from collections import deque
from enum import Enum, auto
from typing import Optional

from host_config import HostConfig
from localization.pose import Pose
from mission.basket_target import BasketTarget, basket_target, facing_error_deg
from planning.planner import DriveMode, DriveSequencer, GridPathPlanner, ObstacleHold, wrap_deg
from vla_common.protocol import HostCommand, JobResult, JobTracker, PiStatus, State

XY = tuple[float, float]
PieceMap = dict[str, list[XY]]


class HostState(Enum):
    SEARCH_TARGET = auto()     # 다음 기물 고르기
    APPROACH_PIECE = auto()    # 기물 앞까지 주행
    GRASP = auto()             # Pi 가 VLA 로 집는 동안 대기
    CARRY_TO_DEST = auto()     # 상자 앞까지 주행
    NUDGE_BOX = auto()         # 상자 정면(dest_xy)까지 직진해 붙기
    PLACE = auto()             # Pi 가 내려놓는 동안 대기
    HALTED = auto()            # 물체를 든 채 갈 곳이 없다 — 사람이 개입


WIRE_STATE = {
    HostState.SEARCH_TARGET: State.IDLE,
    HostState.APPROACH_PIECE: State.APPROACH,
    HostState.GRASP: State.GRASP,
    HostState.CARRY_TO_DEST: State.CARRY,
    HostState.NUDGE_BOX: State.APPROACH_BOX,
    HostState.PLACE: State.PLACE,
    # 물체를 든 채 서 있기. PLACE 로 두면 Pi 가 투하를 다시 시도할 수 있다.
    HostState.HALTED: State.IDLE,
}

_PREV = {
    HostState.APPROACH_PIECE: HostState.SEARCH_TARGET,
    HostState.GRASP: HostState.APPROACH_PIECE,
    HostState.CARRY_TO_DEST: HostState.APPROACH_PIECE,
    HostState.NUDGE_BOX: HostState.CARRY_TO_DEST,
    HostState.PLACE: HostState.NUDGE_BOX,
    # HALTED 에서 "이전"은 사람이 복구했다는 뜻 — 상자 앞 접근부터 다시.
    HostState.HALTED: HostState.NUDGE_BOX,
}


def _dist(a: XY, b: XY) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class MissionFSM:
    def __init__(self, cfg: HostConfig, manual_mode: bool = False) -> None:
        self.cfg = cfg
        self.manual_mode = manual_mode
        m = cfg.mission
        self._planner = GridPathPlanner(cfg.planner, cfg.arena,
                                        arrive_tol=min(m.grasp_trigger_dist_m, m.place_trigger_dist_m))
        self._drive = DriveSequencer(cfg.planner)
        self._obstacles = ObstacleHold(cfg.planner.obstacle_hold_cycles, cfg.planner.obstacle_match_m)
        self.events: deque[str] = deque(maxlen=10)
        self.reset()
        self._check_reach()

    def _check_reach(self) -> None:
        """정차점에서 조준점까지의 거리 = **팔이 뻗어야 하는 거리**. 설정과 맞는지 본다.

        차는 상자 앞 dest_xy 까지만 간다(그 이상은 차체가 상자에 닿는다). 그래서 남은
        거리는 팔의 몫이고, 팔이 그만큼 못 뻗으면 기물이 상자 앞에 떨어진다 — 2026-09-23
        시뮬레이터가 그 상황을 재현했다("missed toy (+2, -21) mm"). 값이 어긋나면 주행이
        아니라 **팔을 재야 한다**(tools/goto_pose.py --pose drop 으로 투하 지점 측정).
        """
        m = self.cfg.mission
        for name in self.cfg.arena.boxes:
            need = self._basket(name).distance(self._box_front_xy(name))
            if abs(need - m.arm_reach_m) > m.place_arrive_tol_m:
                self._log(f"⚠️ {name}: 정차점에서 조준점까지 {need:.3f} m 인데 "
                          f"mission.arm_reach_m 은 {m.arm_reach_m:.3f} m — 팔 도달거리를 재서 맞출 것")

    # ------------------------------------------------------------------ 조작
    def reset(self) -> None:
        """처음부터. ESTOP 래치를 푸는 유일한 방법이다(실수로 재개되지 않게)."""
        self.state = HostState.SEARCH_TARGET
        self.estop = False
        self.target_label: Optional[str] = None
        self.target_xy: Optional[XY] = None
        self.dest_box: Optional[str] = None
        self.dest_xy: Optional[XY] = None
        self.halt_reason: Optional[str] = None
        self.search_reason: Optional[str] = None
        self.ready_to_advance = False
        self.place_tries = 0
        self.skipped: list[tuple[XY, float]] = []
        self.last_result: Optional[JobResult] = None
        self._advance_requested = False
        self._back_requested = False
        self._tracker = JobTracker()
        self._job_armed = False
        self._job_started = 0.0
        self._job_result: Optional[JobResult] = None
        self._nudge_from: Optional[XY] = None
        # 상자 앞에 선 자리에서 팔이 메워야 할 좌우 각도. PLACE 명령에 실린다.
        self.place_arm_yaw_deg = 0.0
        self._now = 0.0
        # 화면 표시용
        self.nav_goal: Optional[XY] = None
        self.nav_path: Optional[list[XY]] = None
        self.blocked_by: Optional[str] = None
        self.last_cmd_text = ""
        self._reset_motion()

    def request_estop(self) -> None:
        self.estop = True
        self._log("ESTOP 래치 — r(reset) 으로만 풀린다")

    def request_advance(self) -> None:
        self._advance_requested = True

    def request_back(self) -> None:
        self._back_requested = True

    def set_manual_mode(self, manual: bool) -> None:
        # 도중에 모드만 바꾸면 "이 상태로 계속 자동 진행?"이 애매하다. 항상 reset 과 묶는다.
        self.manual_mode = manual
        self.reset()

    # ------------------------------------------------------------------ 본체
    def step(self, pose: Pose, piece_map: PieceMap, pi_status: Optional[PiStatus],
             now: float) -> HostCommand:
        self._now = now
        if self.estop:
            self._clear_nav()
            self.last_cmd_text = "ESTOP"
            return HostCommand(State.ESTOP, stop=True)

        if self._back_requested:
            self._back_requested = False
            self._go_back()

        if self.state == HostState.GRASP:
            return self._step_grasp(pi_status)
        if self.state == HostState.PLACE:
            return self._step_place(pi_status)
        if self.state == HostState.HALTED:
            self._clear_nav()
            return self._stop("halted")

        if not pose.ok:
            self._clear_nav()
            return self._stop("pose lost")

        if self.state == HostState.SEARCH_TARGET:
            return self._step_search(pose, piece_map, pi_status)
        if self.state == HostState.APPROACH_PIECE:
            return self._step_approach(pose, piece_map, pi_status)
        if self.state == HostState.CARRY_TO_DEST:
            return self._step_carry(pose, piece_map)
        if self.state == HostState.NUDGE_BOX:
            return self._step_nudge(pose, pi_status)
        raise AssertionError(self.state)

    # ------------------------------------------------------------------ 상태별
    def _step_search(self, pose: Pose, pmap: PieceMap, pi_status) -> HostCommand:
        self._clear_nav()
        skips = self._active_skips()
        found = self._nearest_piece(pmap, pose.xy, skips)
        self.ready_to_advance = found is not None
        if found is None:
            in_ws = [p for pts in pmap.values() for p in pts if self._in_workspace(p)]
            if not pmap:
                self.search_reason = "no pieces on map"
            elif in_ws and skips:
                self.search_reason = f"{len(skips)} skipped (retry in {self.cfg.mission.skip_expiry_s:.0f}s)"
            elif in_ws:
                self.search_reason = "no destination box for visible labels"
            else:
                self.search_reason = "pieces outside workspace"
            return self._stop("search")
        self.search_reason = None
        if self._should_advance():
            label, xy = found
            self.target_label, self.target_xy = label, xy
            self.dest_box = self.cfg.mission.piece_dest_box[label]
            self.dest_xy = self._box_front_xy(self.dest_box)
            self.place_tries = 0
            self._log(f"target {label} @ ({xy[0]:.2f},{xy[1]:.2f}) -> {self.dest_box}")
            self._enter(HostState.APPROACH_PIECE)
            return self._step_approach(pose, pmap, pi_status)
        return self._stop("search (ready)")

    def _step_approach(self, pose: Pose, pmap: PieceMap, pi_status) -> HostCommand:
        assert self.target_xy is not None
        obstacles = [p for pts in pmap.values() for p in pts if _dist(p, self.target_xy) > 0.05]
        dist = _dist(pose.xy, self.target_xy)
        self.ready_to_advance = dist <= self.cfg.mission.grasp_trigger_dist_m
        if self.ready_to_advance:
            self._clear_nav()
            if self._should_advance():
                self._enter(HostState.GRASP)
                return self._step_grasp(pi_status)
            return self._stop("approach (ready)")
        cmd = self._drive_to(pose, self.target_xy, obstacles)
        if self._blocked_too_long():
            # 손이 비었으니 이 기물은 보류하고 다른 기물을 치우면 길이 열릴 수 있다.
            self._skip_target(f"no path for {self.cfg.mission.blocked_timeout_s:.0f}s")
            return self._stop("approach blocked")
        return cmd

    def _step_grasp(self, pi_status: Optional[PiStatus]) -> HostCommand:
        self._clear_nav()
        m = self.cfg.mission
        if not self._job_armed:
            self._tracker.arm(State.GRASP, pi_status)
            self._job_armed = True
            self._job_started = self._now
            self._job_result = None
        if self._job_result is None:
            result = self._tracker.poll(pi_status)
            if result is not None:
                self._job_result = self.last_result = result
                self._log(f"GRASP {'ok' if result.ok else 'FAILED'}: {result.detail}")
            elif self._now - self._job_started > m.grasp_timeout_s:
                self._skip_target(f"grasp timeout {m.grasp_timeout_s:.0f}s")
                return self._stop("grasp timeout")
        if self._job_result is not None:
            if not self._job_result.ok:
                # 손이 비어 있으니 보류하고 다음 기물로 간다. 같은 자리 재시도는 같은 결과일 공산이 크다.
                self._skip_target(f"grasp failed: {self._job_result.detail}")
                return self._stop("grasp failed")
            self.ready_to_advance = True
            if self._should_advance():
                self._enter(HostState.CARRY_TO_DEST)
                return self._stop("grasp done")
        self.last_cmd_text = "GRASP (wait)"
        return HostCommand(State.GRASP, stop=True, label=self.target_label or "")

    def _step_carry(self, pose: Pose, pmap: PieceMap) -> HostCommand:
        assert self.dest_xy is not None
        r = self.cfg.planner.carry_ignore_radius_m
        obstacles = [p for pts in pmap.values() for p in pts if _dist(p, pose.xy) > r]
        dist = _dist(pose.xy, self.dest_xy)
        self.ready_to_advance = dist <= self.cfg.mission.place_trigger_dist_m
        if self.ready_to_advance:
            self._clear_nav()
            if self._should_advance():
                self._enter(HostState.NUDGE_BOX)
                return self._step_nudge(pose, None)
            return self._stop("carry (ready)")
        cmd = self._drive_to(pose, self.dest_xy, obstacles)
        if self._blocked_too_long():
            # 물체를 든 채라 보류할 곳이 없다. 사람을 부른다.
            self._halt(f"no path to {self.dest_box} for {self.cfg.mission.blocked_timeout_s:.0f}s")
            return self._stop("halted")
        return cmd

    def _step_nudge(self, pose: Pose, pi_status) -> HostCommand:
        """상자 정면(dest_xy)까지 **직진해서** 붙는다. 제자리 정렬은 하지 않는다.

        투입 방향을 차체로 맞추지 않는 이유: 차체는 0.5 rad/s 에 데드밴드까지 있어
        몇 도짜리 회전을 못 내고, 제자리 회전이 ArUco 위치추정만 흔든다. 그래서
        **남는 좌우 각도는 팔의 base(servo 1)가 맡는다** — 여기서 그 각도를 재서
        `HostCommand.arm_yaw_deg` 로 보낸다.

        계획기를 쓰지 않는다 — 상자 앞은 회피구역과 겹쳐 "길 없음"이 나온다.
        """
        self._clear_nav()
        m = self.cfg.mission
        target = self._basket()
        if self._nudge_from is None:
            self._nudge_from = pose.xy
        moved = _dist(pose.xy, self._nudge_from)
        # 정차점은 dest_xy 다 — 상자 앞 0.15 m, 주행 구역 안이고 차체가 상자에 닿지 않는
        # 자리다. **더 붙지 않는다**: 팔이 모자라면 차를 밀어 넣는 게 아니라 팔을 재야 한다
        # (2026-09-23 시뮬에서 조준점까지 붙였더니 차가 주행 구역 밖으로 나가 다음 경로를
        # 못 찾았다). 거리 허용치를 좁게 두는 이유는 앞뒤 오차를 아무도 못 메우기 때문이다.
        dist = _dist(pose.xy, self.dest_xy)
        residual = facing_error_deg(target, pose.xy, pose.yaw_deg)
        close = dist <= m.place_arrive_tol_m
        self.place_arm_yaw_deg = residual
        self.ready_to_advance = close and abs(residual) <= m.max_arm_yaw_deg
        if self.ready_to_advance:
            if self._should_advance():
                self._enter(HostState.PLACE)
                return self._step_place(pi_status)
            return self._stop(f"at box (arm {residual:+.1f}도)")
        if close:
            # 팔이 못 메우는 각도다. 이때만 차체를 돌린다 — 한계 안으로만 넣는다.
            self._log(f"arm cannot cover {residual:+.1f}도 (limit ±{m.max_arm_yaw_deg:.0f}) — turning the body")
            return self._rotate(residual)
        if moved >= m.nudge_max_m:
            # 이만큼 밀고도 정면에 못 섰다. 더 밀면 상자를 친다.
            self.place_tries += 1
            if self.place_tries > m.place_retry_max:
                self._halt(f"could not stand in front of {self.dest_box} ({self.place_tries} tries)")
                return self._stop("halted")
            self._log(f"nudge {moved:.2f}m and still {dist:.2f}m off dest_xy — "
                      f"approach again ({self.place_tries}/{m.place_retry_max})")
            self._enter(HostState.CARRY_TO_DEST)
            return self._stop("nudge missed")
        nav = self._drive.update(pose.xy, pose.yaw_deg, self.dest_xy)
        if nav.mode == DriveMode.ROTATE:
            return self._rotate(nav.yaw_error_deg)
        if nav.mode == DriveMode.STOP:
            # 시퀀서가 직진<->회전 사이에 한 사이클 세운다. 그 한 박자를 지킨다.
            return self._stop("nudge (settle)")
        self.last_cmd_text = "nudge"
        return HostCommand(WIRE_STATE[self.state], linear_x=self.cfg.drive.nudge_mps)

    def _step_place(self, pi_status: Optional[PiStatus]) -> HostCommand:
        self._clear_nav()
        m = self.cfg.mission
        if not self._job_armed:
            self._tracker.arm(State.PLACE, pi_status)
            self._job_armed = True
            self._job_started = self._now
            self._job_result = None
        if self._job_result is None:
            result = self._tracker.poll(pi_status)
            if result is not None:
                self._job_result = self.last_result = result
                self._log(f"PLACE {'ok' if result.ok else 'FAILED'}: {result.detail}")
            elif self._now - self._job_started > m.place_timeout_s:
                self._job_result = JobResult(0, State.PLACE, False, f"host timeout {m.place_timeout_s:.0f}s")
                self._log("PLACE timeout")
        if self._job_result is not None:
            if not self._job_result.ok:
                # 물체를 든 채라 보류할 수 없다. 상자 앞에서 다시 세우고, 예산을 넘으면 사람을 부른다.
                self.place_tries += 1
                if self.place_tries > m.place_retry_max:
                    self._halt(f"place failed {self.place_tries} times: {self._job_result.detail}")
                    return self._stop("halted")
                self._log(f"place retry {self.place_tries}/{m.place_retry_max}")
                self._enter(HostState.NUDGE_BOX)
                return self._stop("place retry")
            self.ready_to_advance = True
            if self._should_advance():
                self._log(f"{self.target_label} delivered to {self.dest_box}")
                self._clear_target()
                self._enter(HostState.SEARCH_TARGET)
                return self._stop("place done")
        self.last_cmd_text = f"PLACE (wait, arm {self.place_arm_yaw_deg:+.1f}도)"
        return HostCommand(State.PLACE, stop=True, label=self.target_label or "",
                           arm_yaw_deg=self.place_arm_yaw_deg)

    # ------------------------------------------------------------------ 도우미
    def _drive_to(self, pose: Pose, goal: XY, obstacles: list[XY]) -> HostCommand:
        held = self._obstacles.update(obstacles)
        sub_goal, _corner, blocked = self._planner.update(pose.xy, goal, held)
        nav = self._drive.update(pose.xy, pose.yaw_deg, sub_goal)
        self.nav_goal = goal
        self.nav_path = self._planner.last_path
        self.blocked_by = blocked
        if nav.mode == DriveMode.FORWARD:
            self.last_cmd_text = "forward" + (f" ({blocked})" if blocked else "")
            return HostCommand(WIRE_STATE[self.state], linear_x=self.cfg.drive.linear_mps)
        if nav.mode == DriveMode.ROTATE:
            return self._rotate(nav.yaw_error_deg)
        return self._stop("stop" + (f" ({blocked})" if blocked else ""))

    def _blocked_too_long(self) -> bool:
        """길이 없어 제자리에 선 채(blocked) blocked_timeout_s 가 지났는가.
        조용히 맴돌지 않고 결정(보류/HALT)으로 넘기기 위한 것이다."""
        if self.blocked_by != "blocked":
            self._blocked_since = None
            return False
        if self._blocked_since is None:
            self._blocked_since = self._now
            return False
        return self._now - self._blocked_since > self.cfg.mission.blocked_timeout_s

    def _rotate(self, yaw_error_deg: float) -> HostCommand:
        # yaw_error = 목표 - 현재, 반시계가 +. 부호가 곧 회전 방향이다.
        sign = 1.0 if yaw_error_deg >= 0 else -1.0
        self.last_cmd_text = "yaw+" if sign > 0 else "yaw-"
        return HostCommand(WIRE_STATE[self.state], angular_z=sign * self.cfg.drive.rotation_rad_s)

    def _stop(self, why: str) -> HostCommand:
        self.last_cmd_text = f"stop: {why}"
        return HostCommand(WIRE_STATE[self.state], stop=True, label=self.target_label or "")

    def _enter(self, state: HostState) -> None:
        self.state = state
        self.ready_to_advance = False
        if state in (HostState.GRASP, HostState.PLACE):
            self._job_armed = False
            self._job_result = None
        if state == HostState.NUDGE_BOX:
            self._nudge_from = None
        self._reset_motion()

    def _go_back(self) -> None:
        prev = _PREV.get(self.state)
        if prev is None:
            return
        if prev == HostState.SEARCH_TARGET:
            self._clear_target()
        self.halt_reason = None
        self._log(f"back: {self.state.name} -> {prev.name}")
        self._enter(prev)

    def _should_advance(self) -> bool:
        if not self.manual_mode:
            return True
        if self._advance_requested:
            self._advance_requested = False
            return True
        return False

    def _reset_motion(self) -> None:
        self._blocked_since: Optional[float] = None
        self._planner.reset()
        self._drive.reset()

    def _clear_nav(self) -> None:
        self.nav_goal = None
        self.nav_path = None
        self.blocked_by = None

    def _clear_target(self) -> None:
        self.target_label = None
        self.target_xy = None
        self.dest_box = None
        self.dest_xy = None
        self.place_tries = 0

    def _skip_target(self, why: str) -> None:
        """좌표를 보류 목록에 남긴다. 안 남기면 같은 기물을 또 "가장 가까운 것"으로 골라
        무한 반복한다. 시효(skip_expiry_s)가 있는 이유: 2026-09-05 USB 끊김으로 생긴
        일시적 실패가 세션 내내 영구 보류가 됐다."""
        if self.target_xy is not None:
            self.skipped.append((self.target_xy, self._now))
        self._log(f"skip {self.target_label}: {why}")
        self._clear_target()
        self._enter(HostState.SEARCH_TARGET)

    def _halt(self, why: str) -> None:
        self.halt_reason = why
        self._log(f"HALTED: {why}")
        self._enter(HostState.HALTED)

    def _active_skips(self) -> list[XY]:
        expiry = self.cfg.mission.skip_expiry_s
        self.skipped = [(xy, t) for xy, t in self.skipped if self._now - t < expiry]
        return [xy for xy, _ in self.skipped]

    def _in_workspace(self, p: XY) -> bool:
        a = self.cfg.arena
        return a.workspace_x[0] <= p[0] <= a.workspace_x[1] and a.workspace_y[0] <= p[1] <= a.workspace_y[1]

    def _nearest_piece(self, pmap: PieceMap, robot_xy: XY, skips: list[XY]) -> Optional[tuple[str, XY]]:
        """작업영역 안 + 목적지 상자가 있는 라벨 + 보류되지 않은 것 중 최근접.
        y 가 작업영역 밖이면 상자 자리(이미 옮긴 것)라 뺀다."""
        best, best_d = None, math.inf
        r = self.cfg.mission.skip_radius_m
        for label, pts in pmap.items():
            if label not in self.cfg.mission.piece_dest_box:
                continue
            for p in pts:
                if not self._in_workspace(p) or any(_dist(p, s) <= r for s in skips):
                    continue
                d = _dist(p, robot_xy)
                if d < best_d:
                    best, best_d = (label, p), d
        return best

    def _basket(self, box: Optional[str] = None) -> BasketTarget:
        """목적지 상자의 투입 목표. 팔이 겨누는 점은 상자 중심이 아니다."""
        name = box or self.dest_box
        assert name is not None
        m = self.cfg.mission
        bx, by, _yaw = self.cfg.arena.boxes[name]
        return basket_target(name, (bx, by), self.cfg.arena.box_size,
                             m.insert_half_width_m, m.insert_inset_depth_m,
                             m.place_aim_margin_m)

    def _box_front_xy(self, box: str) -> XY:
        """상자 중심이 아니라 상자 앞(작업영역 쪽). 상자들은 뒤쪽 벽에 붙어 있다."""
        bx, by, _yaw = self.cfg.arena.boxes[box]
        return (bx, by - (self.cfg.arena.box_size[1] / 2.0 + self.cfg.mission.box_approach_margin_m))

    def _log(self, msg: str) -> None:
        line = f"[{self._now:8.1f}] {msg}"
        self.events.append(line)
        print(f"[mission] {line}")

    @property
    def wire_state(self) -> str:
        return State.ESTOP if self.estop else WIRE_STATE[self.state]
