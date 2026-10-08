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
from dataclasses import dataclass, replace
from enum import Enum, auto
from typing import Optional

from host_config import HostConfig
from localization.pose import Pose
from mission.base_monitor import BaseRunawayMonitor, BaseSpinMonitor, BaseStallMonitor
from mission.trajectory_log import body_gap
from mission.basket_target import BasketTarget, basket_target, facing_error_deg
from planning.planner import (DriveMode, DriveSequencer, GridPathPlanner, ObstacleHold,
                              segment_circle_clearance, segment_hits_rect, wrap_deg)
from vla_common.protocol import HostCommand, JobResult, JobTracker, PiStatus, State

XY = tuple[float, float]
PieceMap = dict[str, list[XY]]


@dataclass
class Order:
    """사람의 지시 하나(Claude 가 해석). 이게 있는 동안은 이 라벨들만 고른다.
    quantity "one" = 하나 옮기면 끝, "all" = 보이는 것이 없어질 때까지."""
    labels: tuple[str, ...]
    quantity: str = "one"
    intent: str = "organize"         # organize | fetch (fetch 는 손 전달이 붙기 전까지 바구니로)
    text: str = ""
    done: int = 0
    handed: int = 0                  # 그중 손에 건넨 개수(나머지는 손이 없어 바구니로)


class HostState(Enum):
    SEARCH_TARGET = auto()     # 다음 기물 고르기
    APPROACH_PIECE = auto()    # 기물 앞까지 주행
    GRASP = auto()             # Pi 가 VLA 로 집는 동안 대기
    CARRY_TO_DEST = auto()     # 상자 앞까지 주행
    NUDGE_BOX = auto()         # 상자 정면(dest_xy)까지 직진해 붙기
    FACE_HAND = auto()         # 손 앞 정차점에서 손 쪽을 보기("가져와")
    PLACE = auto()             # Pi 가 내려놓는(건네는) 동안 대기
    HALTED = auto()            # 물체를 든 채 갈 곳이 없다 — 사람이 개입


WIRE_STATE = {
    HostState.SEARCH_TARGET: State.IDLE,
    HostState.APPROACH_PIECE: State.APPROACH,
    HostState.GRASP: State.GRASP,
    HostState.CARRY_TO_DEST: State.CARRY,
    HostState.NUDGE_BOX: State.APPROACH_BOX,
    HostState.FACE_HAND: State.APPROACH_BOX,
    HostState.PLACE: State.PLACE,
    # 물체를 든 채 서 있기. PLACE 로 두면 Pi 가 투하를 다시 시도할 수 있다.
    HostState.HALTED: State.IDLE,
}

#: Host 가 바퀴를 움직이는 단계. 폭주는 여기서만 본다 — SEARCH·HALTED 에서는 사람이 로봇을
#: 옮기는 일이 잦고, GRASP·PLACE 에서는 팔(마커)이 움직인다.
_DRIVE_HOST_STATES = (HostState.APPROACH_PIECE, HostState.CARRY_TO_DEST, HostState.NUDGE_BOX,
                      HostState.FACE_HAND)

_PREV = {
    HostState.APPROACH_PIECE: HostState.SEARCH_TARGET,
    HostState.GRASP: HostState.APPROACH_PIECE,
    HostState.CARRY_TO_DEST: HostState.APPROACH_PIECE,
    HostState.NUDGE_BOX: HostState.CARRY_TO_DEST,
    HostState.FACE_HAND: HostState.CARRY_TO_DEST,
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
        # auto = 보이는 기물을 모두 정리(지시가 오면 그것부터) · instructed = 지시가 있을 때만 움직인다
        self.command_mode = cfg.instruction.mode
        self.finished_order: Optional[tuple[Order, str]] = None     # (지시, "done" | "absent"), 화면용
        self.hands: list[XY] = []        # 탑뷰에서 확인된 손(가져다줄 곳). run_host 가 매 사이클 넣는다
        m = cfg.mission
        self._planner = GridPathPlanner(cfg.planner, cfg.arena,
                                        arrive_tol=min(m.grasp_trigger_dist_m, m.place_trigger_dist_m))
        self._drive = DriveSequencer(cfg.planner)
        self._obstacles = ObstacleHold(cfg.planner.obstacle_hold_cycles, cfg.planner.obstacle_match_m)
        self.events: deque[str] = deque(maxlen=10)
        self.reset()
        self._check_reach()

    def _check_reach(self) -> None:
        """팔이 **상자 테두리를 넘는가**. 기동할 때 한 번 본다.

        차는 상자 앞 dest_xy 까지만 간다 — 그 이상은 차체가 상자에 닿고 상자 금지 구역
        (planner.box_keepout_front_m)을 넘는다. 그래서 테두리까지 남은 거리는 팔의 몫이다.

        기준은 조준점이 아니라 **테두리**다. 조준점은 여유를 두려고 일부러 더 깊게 잡은
        점이라, 거기까지 못 닿아도 테두리만 넘으면 기물은 상자 안에 들어간다. 정차 오차만큼
        덜 붙을 수 있으므로 그만큼을 더해서 본다.

        ⚠️ 2026-09-23: 이걸 조준점 기준으로 두었더니 2 mm 차이로 HALT 가 났다. 시뮬레이터가
        그 자리에서 잡았다.
        """
        m = self.cfg.mission
        for name, (_bx, by, _yaw) in self.cfg.arena.boxes.items():
            rim_gap = (by - self.cfg.arena.box_size[1] / 2.0) - self._box_front_xy(name)[1]
            need = rim_gap + m.place_arrive_tol_m       # 가장 덜 붙어 선 경우까지
            # 팔 base 를 틀면 앞으로 나가는 거리가 cos 만큼 준다(최대 max_arm_yaw_deg).
            reach = m.arm_reach_m * math.cos(math.radians(m.max_arm_yaw_deg))
            if reach < need:
                self._log(f"⚠️ {name}: 테두리까지 {rim_gap:.3f} m + 정차 오차 "
                          f"{m.place_arrive_tol_m:.3f} m = {need:.3f} m 가 필요한데 "
                          f"팔 {m.arm_reach_m:.3f} m 은 base ±{m.max_arm_yaw_deg:.0f}° 에서 "
                          f"{reach:.3f} m — 기물이 상자 앞에 떨어진다")

    # ------------------------------------------------------------------ 조작
    @property
    def idle(self) -> bool:
        """지시를 기다리거나(지시 모드) 멈춰 서 있다 — 기물 검출을 느리게 돌려도 된다."""
        if self.estop or self.state == HostState.HALTED:
            return True
        return (self.state == HostState.SEARCH_TARGET and self.order is None
                and self.command_mode == "instructed")

    def reset(self) -> None:
        """처음부터. ESTOP 래치를 푸는 유일한 방법이다(실수로 재개되지 않게)."""
        self.state = HostState.SEARCH_TARGET
        self.estop = False
        self.order: Optional[Order] = None              # reset = 지시 취소
        self.target_label: Optional[str] = None
        self.target_xy: Optional[XY] = None
        self.dest_box: Optional[str] = None
        self.dest_xy: Optional[XY] = None
        # 목적지 종류: box = 바구니 투입 · hand = 사람 손에 건네기("가져와" 지시)
        self.dest_kind = "box"
        self.hand_spot: Optional[str] = None
        self.hand_xy: Optional[XY] = None
        self._hand_wait_since: Optional[float] = None
        self._face_turning = False
        self._face_arrived = False
        self._last_pose_xy: Optional[XY] = None
        self._pose_hist: deque[tuple[float, XY]] = deque(maxlen=20)
        self.halt_reason: Optional[str] = None
        self.search_reason: Optional[str] = None
        self.ready_to_advance = False
        self.place_tries = 0
        self.grasp_tries = 0
        self.grasp_face_err_deg: Optional[float] = None
        self._in_grasp_zone = False
        self._creep = 0                 # 파지 거리 맞추기: +1 앞으로, -1 뒤로, 0 서 있음
        self._creep_stopped_at: Optional[float] = None
        self._creep_tries = 0
        self._creep_cmds = 0
        self._retry_settle_until: Optional[float] = None   # 파지 실패 뒤 다시 보기 전 대기
        self._narrow_logged = False
        self._exit_logged = False
        self._carry_line_blocked_since: Optional[float] = None
        self._zone_wait_logged = False
        self._box_stop_chosen = False
        self._zone_blocked_since: Optional[float] = None
        self._turn_latch: Optional[tuple[float, float]] = None   # (방향, 마지막 시각) — 거의 정반대 회전 고정
        self._aim_shift = 0.0                # 바구니 정차 구역에서 비켜 선 만큼 겨누는 점도 옮긴다(m)
        self._pre_stop: Optional[tuple[XY, XY]] = None   # (정차점, 거쳐 갈 점) — 정차점이 바뀌면 다시 정한다
        self._other_way_logged = False
        self._halted_from: Optional[HostState] = None
        self._prev_step_now: Optional[float] = None
        self._last_turn_at: Optional[float] = None
        self._along_turning = False
        self._along_logged = False
        self._along_rot: Optional[tuple[float, float]] = None
        self._nudge_drove = False
        self._tight_turn_logged = False
        self._pose_now: Optional[Pose] = None
        self._pmap_now: Optional[PieceMap] = None
        self.skipped: list[tuple[XY, float]] = []
        self.last_result: Optional[JobResult] = None
        self._advance_requested = False
        self._back_requested = False
        self._tracker = JobTracker()
        self._job_armed = False
        self._job_started = 0.0
        self._job_result: Optional[JobResult] = None
        self._nudge_from: Optional[XY] = None
        self._nudge_start_dist = 0.0
        self._nudge_started = 0.0
        self._nudge_turn_logged = False
        self._arm_only_place = False        # 정차점에서 몸을 돌리면 옆 기물에 닿아 팔(±max)로만 넣는다
        # 차체 무응답 자동 복구
        m = self.cfg.mission
        self._stall = BaseStallMonitor(m.base_stall_s, m.base_stall_move_m, m.base_stall_turn_deg)
        self._runaway = BaseRunawayMonitor(m.base_runaway_move_m, m.base_runaway_window_s,
                                           m.base_runaway_grace_s)
        self._spin = BaseSpinMonitor(m.base_spin_turn_deg, m.base_spin_window_s, m.base_spin_grace_s)
        self._recover_started: Optional[float] = None
        self._recover_baseline: Optional[int] = None
        self._recover_in_a_row = 0
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

    @property
    def place_released(self) -> bool:
        """PLACE 작업이 성공으로 끝나 기물을 놓았다 — MANUAL 에서 Next 를 기다리는 동안에도 참(화면용)."""
        r = self._job_result
        return self.state == HostState.PLACE and r is not None and r.ok

    def set_hands(self, hands) -> None:
        """확인된 손 위치(지도 좌표). 손 검출이 없으면 빈 목록."""
        self.hands = [tuple(h) for h in hands]

    def set_order(self, order: Order) -> None:
        """새 지시. 진행 중인 기물(집는 중·운반 중)은 끝까지 하고, 다음 대상부터 지시를 따른다."""
        self.order = order
        self.finished_order = None
        self._log(f"order {'/'.join(order.labels)} x{order.quantity} ({order.intent}): {order.text}")

    def cancel_order(self) -> None:
        if self.order is not None:
            self._log("order cancelled")
        self.order = None

    def _finish_order(self, outcome: str) -> None:
        assert self.order is not None
        self._log(f"order {outcome}: {'/'.join(self.order.labels)} done={self.order.done}")
        self.finished_order = (self.order, outcome)
        self.order = None

    def set_manual_mode(self, manual: bool) -> None:
        # 도중에 모드만 바꾸면 "이 상태로 계속 자동 진행?"이 애매하다. 항상 reset 과 묶는다.
        self.manual_mode = manual
        self.reset()

    # ------------------------------------------------------------------ 본체
    def step(self, pose: Pose, piece_map: PieceMap, pi_status: Optional[PiStatus],
             now: float) -> HostCommand:
        """한 사이클. 차체가 명령을 무시하면(탑뷰로 판단) Pi 에 컨트롤러 복구를 맡기고 기다린다."""
        dt = 0.0 if self._prev_step_now is None else max(0.0, now - self._prev_step_now)
        self._prev_step_now = now
        self._now = now
        if pose.ok:
            self._last_pose_xy = pose.xy
            self._pose_hist.append((now, pose.xy))
        self._pose_now, self._pmap_now = pose, piece_map      # 직선이 비었는지(_line_clear)용
        if self._recover_started is not None and not self.estop:
            waiting = self._wait_base_recovery(pi_status)
            if waiting is not None:
                self._pause_timers(dt)
                return waiting
        if (pi_status is not None and pi_status.base_recovering and not self.estop
                and self.state in _DRIVE_HOST_STATES):
            # Pi 가 스스로 컨트롤러를 다시 띄우는 중(기동 직후 · 소음 정리, 10-05) — 그동안 바퀴는 명령을
            # 안 받는다. 움직이라고 내면 무응답으로 오판하므로 끝날 때까지 정지만 낸다.
            self._stall.reset()
            self._runaway.reset()
            self._spin.reset()
            self._clear_nav()
            self._pause_timers(dt)
            self.last_cmd_text = "base reset (wait)"
            return HostCommand(WIRE_STATE[self.state], stop=True, label=self.target_label or "")
        cmd = self._settle_after_turn(self._step_states(pose, piece_map, pi_status, now), now)
        watching = self.estop or self.state in _DRIVE_HOST_STATES
        ran = self._runaway.update(now, cmd, pose, watching)
        spun = self._spin.update(now, cmd, pose, watching)      # 제자리 회전 폭주(10-05)
        if ran or spun:
            self._runaway.reset()
            self._spin.reset()
            m = self.cfg.mission
            if self.estop:
                # ESTOP 인데 달린다/돈다 — 정지 명령이 안 먹는 것이다. 보드 리셋만 요청하고 ESTOP 은 유지한다.
                self._log(f"차체 폭주 — ESTOP 인데 {'움직인다' if ran else '돈다'}. Pi 에 컨트롤러 복구(보드 리셋) 요청")
                return replace(cmd, recover_base=True)
            if ran:
                why = (f"차체 폭주 — 직진을 멈췄는데 {m.base_runaway_window_s:.1f}s 에 "
                       f"{m.base_runaway_move_m * 100:.0f} cm 넘게 움직인다")
            else:
                why = (f"차체 회전 폭주 — 돌라고 하지 않았는데(또는 반대로) {m.base_spin_window_s:.1f}s 에 "
                       f"{m.base_spin_turn_deg:.0f}° 넘게 돈다")
            return self._start_base_recovery(pi_status, why)
        if self._stall.update(now, cmd, pose):
            return self._start_base_recovery(pi_status)
        if self._stall.moved_since_reset:
            # 복구 뒤 실제로 움직였다 — 연속 실패 횟수를 되돌린다.
            self._recover_in_a_row = 0
        return cmd

    def _settle_after_turn(self, cmd: HostCommand, now: float) -> HostCommand:
        """제자리 회전을 마친 뒤 drive.turn_settle_s 동안은 직진·옆걸음 대신 선다(10-08: 회전 직후 직진하면 처음
        0.4~0.5 s 동안 저절로 돌며 미끄러졌다). 어느 단계의 직진이든(운반·바구니 앞·겨누는 선·거리 맞추기) 같다."""
        if cmd.stop:
            return cmd
        moving = abs(cmd.linear_x) > 1e-6 or abs(cmd.linear_y) > 1e-6
        if abs(cmd.angular_z) > 1e-6 and not moving:
            self._last_turn_at = now
            return cmd
        settle = self.cfg.drive.turn_settle_s
        if moving and settle > 0 and self._last_turn_at is not None and now - self._last_turn_at < settle:
            self.last_cmd_text = f"settle after turn ({self.last_cmd_text})"
            return replace(cmd, linear_x=0.0, linear_y=0.0, angular_z=0.0, stop=True)
        return cmd

    def _start_base_recovery(self, pi_status: Optional[PiStatus], why: Optional[str] = None) -> HostCommand:
        m = self.cfg.mission
        if self._recover_in_a_row >= m.base_recover_max:
            self._halt(f"차체가 명령을 따르지 않는다 — 자동 복구 {self._recover_in_a_row}번 뒤에도. "
                       f"차체 전원·배선을 확인할 것")
            # HALTED 도 멈추라는 명령일 뿐이다 — 폭주 중이면 그것도 안 먹으니 보드 리셋은 계속 요청한다.
            return replace(self._stop("halted"), recover_base=why is not None)
        self._recover_started = self._now
        self._recover_baseline = pi_status.base_recoveries if pi_status is not None else None
        why = why or f"차체 무응답 — 움직임 명령 {m.base_stall_s:.1f}s 동안 위치가 그대로다"
        self._log(f"{why}. Pi 에 컨트롤러 복구 요청({self._recover_in_a_row + 1}/{m.base_recover_max})")
        self._clear_nav()
        self.last_cmd_text = "base recovery"
        return HostCommand(WIRE_STATE.get(self.state, State.IDLE), stop=True,
                           label=self.target_label or "", recover_base=True)

    def _wait_base_recovery(self, pi_status: Optional[PiStatus]) -> Optional[HostCommand]:
        """복구 중이면 정지 + 요청을 계속 싣는다. 끝났으면 None(하던 일로 돌아간다)."""
        m = self.cfg.mission
        if pi_status is not None:
            if self._recover_baseline is None:
                self._recover_baseline = pi_status.base_recoveries
            elif pi_status.base_recoveries > self._recover_baseline and not pi_status.base_recovering:
                self._recover_started = None
                self._recover_in_a_row += 1
                self._stall.reset()
                self._runaway.reset()
                self._spin.reset()
                self._reset_motion()
                self._log(f"차체 컨트롤러 복구 완료 — 하던 {self.state.name} 을 이어간다")
                return None
        if self._now - self._recover_started > m.base_recover_timeout_s:
            self._recover_started = None
            self._halt(f"차체 복구가 {m.base_recover_timeout_s:.0f}s 안에 끝나지 않았다")
            return self._stop("halted")
        self._clear_nav()
        self.last_cmd_text = "base recovery (wait)"
        return HostCommand(WIRE_STATE.get(self.state, State.IDLE), stop=True,
                           label=self.target_label or "", recover_base=True)

    def _step_states(self, pose: Pose, piece_map: PieceMap, pi_status: Optional[PiStatus],
                     now: float) -> HostCommand:
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
        if self.state == HostState.FACE_HAND:
            return self._step_face_hand(pose)
        raise AssertionError(self.state)

    # ------------------------------------------------------------------ 상태별
    def _step_search(self, pose: Pose, pmap: PieceMap, pi_status) -> HostCommand:
        self._clear_nav()
        if self.order is None and self.command_mode == "instructed":
            self.ready_to_advance = False
            self.search_reason = "waiting for command"
            return self._stop("waiting for command")
        skips = self._active_skips()
        labels = self.order.labels if self.order else None
        found = self._nearest_piece(pmap, pose.xy, skips, labels)
        self.ready_to_advance = found is not None
        if found is None and self.order is not None:
            pending = [p for lb in self.order.labels for p in pmap.get(lb, []) if self._in_workspace(p)]
            if not pending:
                # 지시한 기물이 더 없다 — 하나라도 옮겼으면 완료, 아니면 "대상 없음"
                self._finish_order("done" if self.order.done else "absent")
                return self._stop("order finished")
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
            self.grasp_tries = 0
            dest = "hand (파지 뒤 보이는 손)" if (self.order and self.order.intent == "fetch") else self.dest_box
            self._log(f"target {label} @ ({xy[0]:.2f},{xy[1]:.2f}) -> {dest}")
            self._enter(HostState.APPROACH_PIECE)
            return self._step_approach(pose, pmap, pi_status)
        return self._stop("search (ready)")

    def _step_approach(self, pose: Pose, pmap: PieceMap, pi_status) -> HostCommand:
        assert self.target_xy is not None
        m = self.cfg.mission
        self._refresh_target(pmap)
        if self._retry_settle_until is not None:
            if self._now < self._retry_settle_until:
                self.ready_to_advance = False
                return self._stop("grasp retry (settle — 기물 새 위치 기다림)")
            self._retry_settle_until = None
        obstacles = [p for pts in pmap.values() for p in pts if _dist(p, self.target_xy) > 0.05]
        dist = _dist(pose.xy, self.target_xy)
        lo, hi = self._grasp_range()
        # 파지 구역: 들어갈 때는 트리거 거리(기물 범위 상한이 더 크면 그것), 나갈 때는 히스테리시스를 더한다.
        trigger = max(m.grasp_trigger_dist_m, hi)
        limit = trigger + (m.grasp_zone_hysteresis_m if self._in_grasp_zone else 0.0)
        entering = dist <= limit and not self._in_grasp_zone
        if entering:
            # 우회하다 옆구리로 들어오면 여기서 크게 돌아야 하고, 그 자리는 우회하게 만든 기물 바로 옆이다
            # (10-07: 룩을 끼고 돌아 0.33 m 안에 들어와 90° 돌다 룩을 6 cm 밀었다). 그 회전이 옆 기물을
            # 쓸면 아직 구역에 들어가지 않고 경로 주행을 잇는다 — 앞으로 빠져나간 뒤 공을 보고 다가온다.
            bearing = math.degrees(math.atan2(self.target_xy[1] - pose.y, self.target_xy[0] - pose.x))
            face = wrap_deg(bearing - pose.yaw_deg)
            pl = self._planner
            sweep = [o for o in obstacles
                     if _dist(pose.xy, o) < pl.turn_safe
                     and pl._sweep_hits(pose.xy, pose.yaw_deg, face, o, slack_deg=self.SWEEP_SLACK_DEG)]
            if abs(face) > m.grasp_face_tol_deg and sweep:
                entering = False
                if not self._zone_wait_logged:
                    self._zone_wait_logged = True
                    self._log(f"grasp zone: 여기서 {face:+.0f}° 돌면 옆 기물에 닿는다 — 경로로 더 가서 돈다")
        self._in_grasp_zone = self._in_grasp_zone and dist <= limit or entering
        if self._in_grasp_zone:
            self._zone_wait_logged = False
        if self._in_grasp_zone:
            self._clear_nav()
            bearing = math.degrees(math.atan2(self.target_xy[1] - pose.y, self.target_xy[0] - pose.x))
            self.grasp_face_err_deg = wrap_deg(bearing - pose.yaw_deg)
            # 앞뒤로 맞추는 중이면 그것부터 끝낸다(정면을 본 채 곧장 움직이므로 방향은 거의 그대로다).
            creep = self._creep_to_range(dist)
            if creep is not None:
                return creep
            # 정면으로 볼 때까지 제자리에서 돈다. 거리만 보고 잡으면 정책이 옆에 있는 기물을
            # 못 잡는다(2026-09-30: 18° · 30° 어긋난 채 시작해 둘 다 실패, 7° 는 성공).
            if abs(self.grasp_face_err_deg) > m.grasp_face_tol_deg:
                self.ready_to_advance = False
                cmd = self._rotate(self.grasp_face_err_deg)
                self.last_cmd_text = f"face piece {self.grasp_face_err_deg:+.0f}도"
                return cmd
            # 정면을 봤으면 거리를 범위 안으로. 앞뒤로 움직였으면 선 뒤 잠깐 기다렸다 다시 잰다.
            if self._creep_stopped_at is not None and self._now - self._creep_stopped_at < m.grasp_settle_s:
                self.ready_to_advance = False
                return self._stop(f"grasp range (settle, {dist:.3f} m)")
            in_range = lo <= dist <= hi
            if not in_range and self._creep_tries < self.CREEP_TRIES:
                # 범위까지 앞뒤로 갈 자리가 탑뷰로 없으면(차체가 다른 기물에 닿는다) 여기서 잡지 않는다 —
                # 너무 가까우면 그리퍼 아래에 걸려 빈손이다. 다른 기물부터 치우고 다시 온다(10-08 사용자).
                blocker = self._creep_blocker(pose, dist)
                if blocker is not None:
                    return self._no_room_to_grasp(pmap, pose, dist, blocker)
                self._creep = 1 if dist > hi else -1
                self._creep_tries += 1
                self._creep_cmds = 0
                creep = self._creep_to_range(dist)
                if creep is not None:
                    return creep
            self.ready_to_advance = True
            if self._should_advance():
                # 실제로 몇 cm 에서 잡기 시작했는지 매번 남긴다(10-05: "파지를 멀리서 시도하는 것 같다" —
                # 범위 밖일 때만 적어서 숫자로 확인할 수 없었다). 거리 = 마커 중심 -> 기물.
                out = "" if in_range else f" · 범위 밖(맞추기 {self.CREEP_TRIES}번 뒤 그대로)"
                self._log(f"grasp start {self.target_label}: {dist:.3f} m (범위 {lo:.2f}-{hi:.2f}) · "
                          f"정면 {self.grasp_face_err_deg:+.0f}° · 거리 맞추기 {self._creep_tries}번{out}")
                self._enter(HostState.GRASP)
                return self._step_grasp(pi_status)
            return self._stop(f"approach (ready, {self.grasp_face_err_deg:+.0f}도)")
        self.ready_to_advance = False
        self._creep, self._creep_stopped_at, self._creep_tries = 0, None, 0
        cmd = self._drive_to(pose, self.target_xy, obstacles)
        if self._blocked_too_long():
            # 손이 비었으니 이 기물은 보류하고 다른 기물을 치우면 길이 열릴 수 있다.
            self._skip_target(f"no path for {self.cfg.mission.blocked_timeout_s:.0f}s")
            return self._stop("approach blocked")
        return cmd

    #: 지금 서 있는 자리에서 실제로 돌 각도를 볼 때 앞뒤로 더 보는 여유. 계획기의 꺾는 점 검사(12°)는 도착 방향
    #: 오차까지 넣지만, 여기서는 지금 방향을 재고 있다 — 12° 를 붙이면 5° 보정에도 "닿는다"가 나왔다(10-07).
    SWEEP_SLACK_DEG = 3.0

    #: 파지 거리 맞추기 최대 횟수. 넘으면 그 자리에서 잡는다(맴돌지 않게).
    CREEP_TRIES = 4

    #: 거리 맞추기 끝 자리를 볼 때 더 보는 거리 — 멈추라고 한 뒤 지연 동안 더 간다(09-30: 0.30 에서 멈췄는데 0.25).
    CREEP_OVERRUN_M = 0.01

    def _creep_blocker(self, pose: Pose, dist: float) -> Optional[XY]:
        """파지 범위 가운데까지 지금 방향으로 앞뒤로 가면 차체가 다른 기물(목표 제외)에 basket_stop_clear_m 보다
        붙는가. 붙으면 그 기물, 아니면 None. 이미 붙어 있던 기물에서 멀어지는 쪽은 막지 않는다.

        10-08 1차: 정차점에서 154° 돌아 상자를 본 뒤 0.249 m 라 바구니 쪽으로 후진 — 뒤의 퀸과 계산상 −1.2 cm."""
        c, clear = self.cfg.planner, self.cfg.mission.basket_stop_clear_m
        move = dist - sum(self._grasp_range()) / 2.0              # + 앞으로, − 뒤로
        move += math.copysign(self.CREEP_OVERRUN_M, move)
        th = math.radians(pose.yaw_deg)
        steps = max(1, int(math.ceil(abs(move) / 0.005)))
        worst: Optional[tuple[float, XY]] = None
        for o in self._other_pieces(pose):
            gap0 = body_gap(pose.x, pose.y, pose.yaw_deg, o, c.robot_length_m, c.robot_width_m, c.piece_obstacle_radius_m)
            for k in range(1, steps + 1):
                d = move * k / steps
                g = body_gap(pose.x + d * math.cos(th), pose.y + d * math.sin(th), pose.yaw_deg, o,
                             c.robot_length_m, c.robot_width_m, c.piece_obstacle_radius_m)
                if g < clear and g < gap0 - 1e-4 and (worst is None or g < worst[0]):
                    worst = (g, o)
        return worst[1] if worst else None

    def _no_room_to_grasp(self, pmap: PieceMap, pose: Pose, dist: float, blocker: XY) -> HostCommand:
        """거리 맞추기 자리가 없다 — 다른 기물이 있으면 이것은 보류하고 그것부터, 없으면 사람을 부른다."""
        lo, hi = self._grasp_range()
        name = self._piece_label_at(blocker)
        way = "앞" if dist > hi else "뒤"
        why = (f"{self.target_label} 를 잡을 거리({lo:.2f}-{hi:.2f} m, 지금 {dist:.3f})로 {way}로 가면 "
               f"{name} 에 닿는다")
        labels = self.order.labels if self.order else None
        others = self._nearest_piece(pmap, pose.xy, self._active_skips() + [self.target_xy], labels)
        if others is None:
            self._halt(f"{why} — 치워 주세요")
            return self._stop("halted")
        self._skip_target(f"{why} — 다른 기물부터")
        return self._stop("no room to grasp")

    def _grasp_range(self) -> tuple[float, float]:
        """지금 목표 기물의 파지 시작 거리 범위 [min, max]. 기물별 값이 없으면 기본 범위."""
        m = self.cfg.mission
        return m.grasp_dist_by_label.get(self.target_label or "", (m.grasp_dist_min_m, m.grasp_dist_max_m))

    def _creep_to_range(self, dist: float) -> Optional[HostCommand]:
        """파지 거리 범위의 가운데로 천천히 앞(+1)·뒤(-1)로 간다. 끝났거나 할 일이 없으면 None.

        멈추라고 해도 명령 지연 동안 더 가므로(09-30: 트리거 0.30 에서 멈췄는데 0.25 에 섰다)
        속도 x grasp_creep_lead_s 만큼 미리 멈춘다. 멈춘 시각을 적어 두고 grasp_settle_s 뒤에 다시 잰다.
        """
        if self._creep == 0:
            return None
        m = self.cfg.mission
        v = self.cfg.drive.nudge_mps
        mid = sum(self._grasp_range()) / 2.0
        lead = v * m.grasp_creep_lead_s
        done = (dist - lead <= mid) if self._creep > 0 else (dist + lead >= mid)
        # 범위를 몇 mm 만 벗어났으면 "미리 멈추기"가 곧바로 참이라 한 번도 안 움직이고 멈춤·대기만
        # 되풀이했다(시뮬 700 s). 시작했으면 적어도 한 사이클은 움직인다.
        if done and self._creep_cmds > 0:
            self._creep = 0
            self._creep_stopped_at = self._now
            self.ready_to_advance = False
            return self._stop(f"grasp range (settle, {dist:.3f} m)")
        self.ready_to_advance = False
        self._creep_cmds += 1
        self.last_cmd_text = f"grasp range {'fwd' if self._creep > 0 else 'back'} ({dist:.3f} m)"
        return HostCommand(WIRE_STATE[self.state], linear_x=self._creep * v, label=self.target_label or "")

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
                if self.grasp_tries < m.grasp_retry_max:
                    # 바로 다시 잡지 않는다 — 같은 관측이면 같은 결과다. 접근 단계로 돌아가
                    # 탑뷰로 기물 위치를 다시 읽고(건드려 밀렸을 수 있다) 정면을 다시 맞춘다.
                    self.grasp_tries += 1
                    self._log(f"GRASP retry {self.grasp_tries}/{m.grasp_retry_max}: "
                              f"위치를 다시 보고 정면을 맞춘 뒤 다시 잡는다")
                    self._enter(HostState.APPROACH_PIECE)
                    # 바로 다시 보지 않는다 — 밀린 기물의 새 위치가 지도에 잡힐 때까지 선다(10-05 soccer)
                    self._retry_settle_until = self._now + m.grasp_retry_settle_s
                    return self._stop("grasp failed — re-approach")
                # 재시도까지 실패했다. 보류하고 다음 기물로 간다.
                self._skip_target(f"grasp failed: {self._job_result.detail}")
                return self._stop("grasp failed")
            self.ready_to_advance = True
            if self._should_advance():
                self._choose_destination(self._last_pose_xy)
                self._enter(HostState.CARRY_TO_DEST)
                return self._stop("grasp done")
        self.last_cmd_text = "GRASP (wait)"
        return HostCommand(State.GRASP, stop=True, label=self.target_label or "")

    def _step_carry(self, pose: Pose, pmap: PieceMap) -> HostCommand:
        """집은 자리에서 상자 정차점(dest_xy)으로 곧장 간다.

        2026-09-30 저녁: 한동안 정차점 앞 진입점(가운데 지점)을 거쳐 올라가게 했는데, 상자 옆에서
        집어도 가운데로 내려갔다가 다시 올라와 동작이 길었다. 정차점을 상자에서 0.22 m 로 물려
        거기서 돌아도 상자에 닿지 않게 했으므로 진입점 없이 곧장 간다.
        """
        assert self.dest_xy is not None
        m = self.cfg.mission
        r = self.cfg.planner.carry_ignore_radius_m
        if self.dest_kind != "hand" and self.dest_box and not self._box_stop_chosen and pose.ok:
            who = self._choose_box_stop(pose)
            if who is None:
                self._box_stop_chosen = True
                self._zone_blocked_since = None
            else:
                # 잡은 직후 지도는 흔들린다(팔이 돌아오며 카메라를 가린다) — 10-07: 지금 지도로는 −4 cm 에 서는데
                # 잡은 순간의 지도로 막혀 HALTED. 운반을 이으며 basket_stop_zone_retry_s 동안 다시 보고 그래도 없으면 멈춘다.
                if self._zone_blocked_since is None:
                    self._zone_blocked_since = self._now
                    self._log(f"basket stop zone: 지금은 설 자리가 없다({who}) — {m.basket_stop_zone_retry_s:.0f}s 동안 다시 본다")
                elif self._now - self._zone_blocked_since > m.basket_stop_zone_retry_s:
                    self._box_stop_chosen = True
                    self._halt(f"{self.dest_box} 앞 정차 구역(가운데 ±{m.basket_stop_zone_half_m * 100:.0f} cm)에 "
                               f"{who} 이(가) 있어 설 자리가 없다 — 치워 주세요")
                    return self._stop("halted")
        # 쥐고 있는 기물은 로봇 옆에서 계속 검출된다 — 그것만 뺀다. **라벨이 같은 것만**이다.
        # 2026-09-30: 반경만으로 빼다가 box 기물에 30 cm 안으로 다가가자 그것까지 빠져,
        # 계획기가 box 를 뚫고 가는 길을 내고 그대로 밀고 갔다.
        obstacles = [p for label, pts in pmap.items() for p in pts
                     if not (label == self.target_label and _dist(p, pose.xy) <= r)]
        # 상자(손) 앞 맞추기는 계획기 없이 정차점까지 곧장 간다 — 그 직선이 기물을 스치면 넘기지 않고
        # 계획기로 더 다가간다(2026-10-05: 0.35 m 에서 넘겨 곧장 가다 정차점 옆 나이트를 쳤다).
        dist = _dist(pose.xy, self.dest_xy)
        near = dist <= m.place_trigger_dist_m
        at_stop = dist <= m.place_arrive_tol_m      # 이미 정차점에 서 있다 — 곧장 갈 거리가 없다
        # 바구니에는 아래에서 들어간다(10-07): 옆에서 비스듬히 들어가면 정차점에서 크게 돌아야 하고, 그 회전이
        # 옆 기물을 쓴다. 정차점보다 충분히 아래에 있으면 정면 ±cone 안에서만 넘기고, 아니면 바로 아래 지점으로 간다.
        # 10-07 마지막 상자: 주변에 기물이 없는데도 가운데(0.93, 0.96)로 갔다 올라갔다 — 곧장 가서 정차점에서
        # 도는 회전이 기물을 쓸 때만 아래에서 들어간다.
        # 옆에서(정차점 높이) 올 때도 같다 — 10-07 상자: 오른쪽에서 퀸 바로 아래로 옆으로 들어가 1.3 cm, 거기서 돌다 −2 cm.
        low = self.dest_kind != "hand" and bool(self.dest_box) and self._straight_arrival_sweeps(pose)
        bearing = math.degrees(math.atan2(self.dest_xy[1] - pose.y, self.dest_xy[0] - pose.x))
        in_cone = (not low) or abs(wrap_deg(bearing - 90.0)) <= m.basket_approach_cone_deg
        line_ok = self._line_clear(pose.xy, self.dest_xy)
        # 10-07: 정차점 2 cm 안에 서 있는데 정차점 자체가 나이트에서 13 cm 라 "직선 막힘"으로 영원히 서 있었다.
        # 넘기면 곧장 가는 단계가 먼저 정차점 쪽으로 제자리에서 돈다(검사 없음). 그 회전이 옆 기물을 쓸면
        # 넘기지 않고 운반(앞으로 빠져나간 뒤 돌기)을 잇는다 — 10-07 궤적: 넘긴 직후 +20° 돌며 상자 기물과 −1.9 cm.
        turn_ok = not self._approach_turn_sweeps(pose)
        # 직선과 도착 회전이 비면 멀리서도 곧장 간다(경로 계획의 중간점을 거치면 목표가 바뀌며 한 번 더 돈다).
        # 멀리서 넘길 때는 정차점 쪽 첫 회전을 경로 주행과 같은 기준(계획기 회전 반경 + 차체)으로 본다 — 쓸리면 경로
        # 주행이 앞으로 빠져나간 뒤 돈다(틈 안에서 잡은 경우, 10-07).
        direct = (self.dest_kind != "hand" and dist <= m.place_direct_max_m
                  and line_ok and turn_ok and in_cone and not self._direct_turn_blocked(pose, obstacles))
        self.ready_to_advance = (near and ((line_ok and turn_ok and in_cone) or at_stop)) or direct
        if near and not line_ok and not at_stop:
            if self._carry_line_blocked_since is None:
                self._carry_line_blocked_since = self._now
            elif self._now - self._carry_line_blocked_since > m.carry_line_block_s:
                who = self._piece_label_near(self.dest_xy)
                where = self.dest_box or f"hand {self.hand_spot}"
                self._halt(f"{where} 정차점 옆 {who} 이(가) 길을 막는다 — 치워 주세요 "
                           f"({m.carry_line_block_s:.0f}s 동안 정차점까지 직선이 막힘)")
                return self._stop("halted")
        else:
            self._carry_line_blocked_since = None
        if self.ready_to_advance:
            # 정차점 근처에서 상자 앞(손 앞) 맞추기로 Next 없이 넘어간다 — MANUAL 에서도 "운반" 한 단계로 본다
            # (2026-10-05: 운반 중 / 상자 앞 진입 중이 따로 Next 를 받는 게 구분이 안 된다).
            self._clear_nav()
            if self.dest_kind == "hand":
                self._enter(HostState.FACE_HAND)
                return self._step_face_hand(pose)
            self._enter(HostState.NUDGE_BOX)
            return self._step_nudge(pose, None)
        goal = self.dest_xy
        if low and not in_cone:
            # 정차점 바로 아래 — 거기서 위로 곧장. 이미 그보다 위에 있으면 내려가지 않고 지금 높이에서 옆으로 간다
            # (10-07 별: y 1.06 에서 잡고 0.96 까지 내려갔다 올라오며 직진·회전이 여러 번 섞였다). 한 번 정하면 고정.
            if self._pre_stop is None or self._pre_stop[0] != self.dest_xy:
                self._pre_stop = (self.dest_xy, self._pick_pre_stop(pose))
            if self._pre_stop[1] is not None:
                goal = self._pre_stop[1]
        cmd = self._drive_to(pose, goal, obstacles)
        if self._blocked_too_long():
            # 물체를 든 채라 보류할 곳이 없다. 사람을 부른다.
            where = self.dest_box or f"hand {self.hand_spot}"
            self._halt(f"no path to {where} for {self.cfg.mission.blocked_timeout_s:.0f}s")
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
            self._nudge_start_dist = _dist(pose.xy, self.dest_xy)
        moved = _dist(pose.xy, self._nudge_from)
        if _dist(pose.xy, self.dest_xy) > m.place_arrive_tol_m and not self._line_clear(
                pose.xy, self.dest_xy, slack=0.01):
            # 정차점까지 곧장 가면 기물을 스친다 — 밀고 들어가지 않고 계획기로 돌아가 돌아서 온다(10-05 나이트)
            self._log("nudge: 정차점까지 직선에 기물이 있다 — 운반(경로 계획)으로 돌아가 다시 다가간다")
            self._enter(HostState.CARRY_TO_DEST)
            return self._stop("nudge blocked by a piece")
        # 멀리서 곧장 오면(place_direct_max_m) 오는 시간만큼 더 준다.
        if self._now - self._nudge_started > m.nudge_timeout_s + self._nudge_start_dist / self.cfg.drive.nudge_mps:
            # 2026-09-30: 상자 앞에서 "물러나기 <-> 밀기"를 끝없이 되풀이한 적이 있다. 여기서 더
            # 버티지 않고 운반 단계로 돌아가 다시 접근한다.
            return self._nudge_missed(f"no stand-off after {m.nudge_timeout_s:.0f}s at the box")
        # 정차점은 dest_xy 다 — 상자 앞 box_approach_margin_m(0.22), 금지 구역 바로 밖이고
        # 거기서 제자리 회전해도 차체가 상자에 닿지 않는 자리다. **더 붙지 않는다**: 팔이 모자라면 차를 밀어 넣는 게 아니라 팔을 재야 한다
        # (2026-09-23 시뮬에서 조준점까지 붙였더니 차가 주행 구역 밖으로 나가 다음 경로를
        # 못 찾았다). 거리 허용치를 좁게 두는 이유는 앞뒤 오차를 아무도 못 메우기 때문이다.
        # 앞뒤는 **비대칭**으로 본다 — 덜 붙는 쪽은 팔 길이(0.32 m)가 메워 주지만,
        # 더 붙는 쪽은 place_min_gap_m 을 넘으면 돌 때 차체가 상자에 닿는다.
        gap = target.distance(pose.xy) - target.distance(self.dest_xy)   # + 면 덜 붙었다
        dist = _dist(pose.xy, self.dest_xy)
        residual = facing_error_deg(target, pose.xy, pose.yaw_deg)
        # 바구니까지 거리만 보면 정차점 높이에서 옆으로 13 cm 떨어져도 "도착"이었다(10-07 별, 팔 +12.6° 로 멀리서 넣음).
        # 바구니 쪽 몸 회전을 시작했으면 경계를 2 cm 넓힌다 — 정차점 옆 8 cm 언저리에서 위치 흔들림으로 "바구니 쪽 돌기"와
        # "정차점 쪽 돌기"가 사이클마다 번갈아 나왔다(10-08 시뮬, 회전 뒤 기다리기와 겹쳐 25 s 동안 제자리).
        # 바구니 쪽 몸 회전을 시작했거나 달려와 섰으면 경계를 2 cm 넓힌다(떨림 · 4.5 cm 에 서서 0.5 cm 더 붙으려 돌고 기기).
        hyst = 0.02 if (self._nudge_turn_logged or self._nudge_drove) else 0.0
        rim_ok = dist <= m.place_here_max_m + hyst and self._drop_clears_rim(pose, target)
        # 곧장 달려오는 중이면 허용치(4 cm) 끝에서 서지 않고 정차점 거리(+1 cm)까지 와서 선다 — 거기서 바구니 쪽으로
        # 돌고 바로 넣는다. 4 cm 앞에서 돌면 돌며 마커가 밀려 "덜 붙었다"가 되고, 겨누는 선으로 다시 붙었다
        # (10-08 별·공: 돌고 → 앞으로 2~3 cm). 모든 기물이 같은 순서: 직진 → 도착 → yaw → 투입.
        driving = self._nudge_drove and not self._nudge_turn_logged
        # 정차점 5 cm 안(시퀀서가 방향을 못 재는 거리)에 들어오면 달리기는 끝 — 허용치로 본다(더 붙으려고 돌고 기지 않는다).
        gap_hi = (m.place_here_gap_m if driving and dist > max(self.cfg.planner.axis_leg_tolerance_m,
                                                               self.cfg.planner.min_heading_dist_m)
                  else m.place_arrive_tol_m + hyst / 2.0)
        close = (-m.place_min_gap_m <= gap <= gap_hi
                 and (dist <= m.place_lateral_tol_m + hyst or rim_ok))
        # 정차점 가까이에서 바구니까지만 조금 멀다 — 정차점으로 돌아가지 않고 겨누는 점을 보고 그 선을 따라 붙는다.
        # 바구니까지 place_arrive_tol_m 넘게 멀 때 시작하고, 시작했으면 place_here_gap_m 까지 붙는다.
        # 서 있는 채 넘어왔을 때(잡은 자리가 정차점 가까이), 또는 달려오다 그대로 가면 정차점 옆 place_lateral_tol_m 밖에
        # 닿을 만큼 틀어졌을 때만 — 그 안이면 끝까지 달려 정차점 거리에 선다(위 driving). 각이 아니라 옆 거리로 본다:
        # 정차점 12 cm 앞에서 17° 는 옆 3.4 cm 다(10-08 시뮬: 각으로 보다 서서 돌고 다시 붙었다).
        to_stop = math.degrees(math.atan2(self.dest_xy[1] - pose.y, self.dest_xy[0] - pose.x))
        off_line = dist * abs(math.sin(math.radians(wrap_deg(to_stop - pose.yaw_deg)))) > m.place_lateral_tol_m
        along = (rim_ok and not self._arm_only_place and gap <= m.place_here_max_gap_m
                 and (self._along_turning or not self._nudge_drove or (gap > m.place_arrive_tol_m and off_line))
                 and gap > (m.place_here_gap_m if self._along_turning else m.place_arrive_tol_m))
        # 팔로만 넣기로 했으면 팔 한계까지만 튼다(남는 몇 도만큼 떨어지는 점이 옆으로 간다).
        arm = (max(-m.max_arm_yaw_deg, min(m.max_arm_yaw_deg, residual)) if self._arm_only_place
               else residual)
        self.place_arm_yaw_deg = arm
        # 차체를 돌리기 시작했으면 팔 한계 바로 안(14.x°)이 아니라 place_turn_to_deg(12°)까지 돈다.
        # 정면(0°)까지는 맞추지 않는다 — 나머지는 팔 base 가 맡는다(2026-09-30 저녁 요청).
        limit = m.place_turn_to_deg if self._nudge_turn_logged else m.max_arm_yaw_deg
        if along:
            # 붙기 시작한 뒤에는 팔이 메우는 만큼(place_turn_to_deg)까지 다시 돌지 않는다 — 회전 뒤 직진 시작 때 더 도는 몫이
            # 덜 나와 몇 도 남아도 서서 다시 돌지 않게(10-08 시뮬: 돌고 1.5 cm 가고 다시 돌고).
            limit = self.cfg.planner.yaw_tolerance_deg if not self._along_turning else m.place_turn_to_deg
            if not self._along_turning:
                # 돌고 나서 직진하면 돈 방향으로 turn_lead_deg 더 돈다 — 그만큼 남기고 멈춘다(시퀀서와 같다).
                if self._along_rot is None and abs(residual) > limit:
                    self._along_rot = (1.0 if residual >= 0 else -1.0, self._now)
                if self._along_rot is not None and self._now - self._along_rot[1] >= 0.5 \
                        and residual * self._along_rot[0] > 0:
                    residual = residual - math.copysign(min(self.cfg.planner.turn_lead_deg, abs(residual)),
                                                        self._along_rot[0])
        self.ready_to_advance = close and not along and (abs(residual) <= limit or self._arm_only_place)
        if self.ready_to_advance and not self._standing_still():
            # 넣기 전에 차체가 정말 섰는지 본다 — 곧장 길게 달려오면(place_direct_max_m) 중간에 서는 일이 없어, 굳은
            # 직진(09-30 폭주)을 "멈춰라" 뒤에야 알아챈다. 투입 중에는 지켜보지 않으므로 여기서 거른다.
            self.ready_to_advance = False
            return self._stop("at box (settle)")
        if self.ready_to_advance:
            if self._should_advance():
                self._enter(HostState.PLACE)
                return self._step_place(pi_status)
            return self._stop(f"at box (arm {residual:+.1f}도)")
        if gap < -m.place_min_gap_m and dist <= m.place_trigger_dist_m:
            # 정차점을 크게 지나쳤다(드문 경우). 멀리서 곧장 오는 중(place_direct_max_m)이면 해당 없다. 여기서 돌면 차체가 상자에 닿고, 시퀀서는 뒤에 있는
            # 정차점을 보려고 180° 돌려 한다 — 돌지 않고 상자에서 곧장 물러난다.
            return self._back_away_from_box(pose, "overshoot — back off")
        if along and abs(residual) <= limit:
            ahead = (pose.x + (gap + 0.02) * math.cos(math.radians(pose.yaw_deg)),
                     pose.y + (gap + 0.02) * math.sin(math.radians(pose.yaw_deg)))
            if self._line_clear(pose.xy, ahead):
                self._along_turning = True              # 붙는 동안 조금 흘러도 6° 까지는 다시 돌지 않는다
                if not self._along_logged:
                    self._along_logged = True
                    self._log(f"nudge: 정차점 {dist * 100:.0f} cm 옆 · 바구니까지 {gap * 100:+.0f} cm — "
                              f"겨누는 점을 보고 곧장 붙는다")
                self.last_cmd_text = "nudge (aim line)"
                return HostCommand(WIRE_STATE[self.state], linear_x=self.cfg.drive.nudge_mps,
                                   label=self.target_label or "")
            along = False                               # 앞에 기물 — 정차점으로 간다
        if (close or along) and abs(residual) > limit:
            # 팔이 못 메우는 각도다. 이때만 정차점에서 차체를 돌린다(돌아도 상자에 닿지 않는 자리다).
            # 단, 그 회전이 바구니 옆 기물을 쓸면 돌지 않는다(10-05 별 · 10-07 나이트를 쳤다).
            turn = residual - math.copysign(limit if along else m.place_turn_to_deg, residual)
            if self._turn_sweeps(pose, turn):         # 정차 구역·상자 앞 맞추기와 같은 간격 기준
                # 반대로 돌면 비는가(10-08 상자: 정차점 위에서 −144° 를 보고 잡아 +127° 쪽은 뒷모서리가 퀸을 쓸고
                # 시계 방향은 뒷면이 퀸에서 멀어지는데, 짧은 쪽만 보고 다시 접근 3번 → HALTED).
                other = turn - math.copysign(360.0, turn)
                if not self._turn_sweeps(pose, other):
                    self._nudge_turn_logged = True
                    return self._rotate_other_way(residual, residual - math.copysign(360.0, residual))
                short = abs(residual) - m.max_arm_yaw_deg
                if short <= m.place_arm_only_max_short_deg:
                    self._arm_only_place = True
                    self._log(f"몸을 돌리면 바구니 옆 기물에 닿는다 — 팔로만 넣음 "
                              f"(팔 {math.copysign(m.max_arm_yaw_deg, residual):+.0f}°, 모자란 각 {short:.1f}°)")
                    return self._stop("arm-only place")
                # 양쪽 다 1.5 cm 안이지만 닿지는 않으면 덜 붙는 쪽으로 돈다 — 운반 중 바구니 앞 회전과 같은 규칙(10-07).
                # 10-08 상자: 잡은 자리가 고른 정차점에서 3 cm 어긋나 퀸과 시계 1.1 · 반시계 0.8 cm.
                g_turn, g_other = self._turn_min_gap(pose, turn), self._turn_min_gap(pose, other)
                if max(g_turn, g_other) >= 0.0:
                    self._nudge_turn_logged = True
                    if not self._tight_turn_logged:
                        self._tight_turn_logged = True
                        self._log(f"바구니 앞 몸 회전: 양쪽 다 옆 기물과 빠듯하다(짧은 쪽 {g_turn * 100:.1f} · "
                                  f"반대 {g_other * 100:.1f} cm) — 덜 붙는 쪽으로")
                    if g_other > g_turn + 0.005:
                        return self._rotate_other_way(residual, residual - math.copysign(360.0, residual))
                    return self._rotate(residual)
                return self._nudge_missed(f"바구니 옆 기물 — 몸을 돌리면 닿고 팔로는 {short:.1f}° 모자란다")
            if not self._nudge_turn_logged:
                self._nudge_turn_logged = True
                self._log(f"arm cannot cover {residual:+.1f}도 (limit ±{m.max_arm_yaw_deg:.0f}) — "
                          f"turn the body at the stop point")
            return self._rotate(residual)
        # 멀리서 시작했으면 올라가는 거리도 길다 — 그만큼은 허용한다.
        if moved >= max(m.nudge_max_m, self._nudge_start_dist + 0.15):
            # 이만큼 밀고도 정면에 못 섰다. 더 밀면 상자를 친다.
            return self._nudge_missed(f"nudge {moved:.2f}m and still {gap:+.2f}m off the stand-off")
        # 앞길이 비었으면 25° 까지, 정차점 15 cm 안이면 다시 돌지 않는다 — 운반과 같은 규칙(10-08: 30 cm 맞추기 중
        # 12° 넘을 때마다 서서 다시 돌았다). 정차점이 5 cm 안이면 방향을 못 재 지금 방향으로 곧장 갔다(퀸, 바구니 반대로
        # 6 cm) — 그때는 정차점이 아니라 겨누는 점을 본다.
        goal = self.dest_xy
        if dist < self.cfg.planner.min_heading_dist_m:
            return self._rotate(residual) if abs(residual) > self.cfg.planner.yaw_tolerance_deg else \
                HostCommand(WIRE_STATE[self.state], linear_x=self.cfg.drive.nudge_mps, label=self.target_label or "")
        others = self._other_pieces(pose)
        # 다시 돌기 시작하는 문턱. 바구니 앞은 상자 금지 구역 바로 앞이라 운반용 판단(_enter_deg, 앞길이 금지 구역에 닿으면
        # 기본 12°)을 쓰지 않는다. 틀어진 길이 기물의 회전 반경(turn_safe) 밖이면:
        #   - 지금 흐름대로 가도 정차점 옆 place_lateral_tol_m 안에 닿는 각까지는 그대로 간다 — 도착해서 바구니 쪽으로
        #     돌며 맞춘다(10-08 사용자: 정차점까지 직선, 도착 뒤 yaw). 가까울수록 같은 옆 거리에 큰 각이다.
        #   - 정차점 no_turn_near_m 안에서는 다시 돌지 않는다.
        # 기물 옆을 지나면 넓히지 않는다(10-08 공: 8~9° 틀어진 채 퀸 옆 2.9 cm 에 도착, 어느 쪽으로도 못 돌아 HALTED).
        th = math.radians(pose.yaw_deg)
        ahead = (pose.x + dist * math.cos(th), pose.y + dist * math.sin(th))
        enter = None
        if dist > 1e-6 and all(segment_circle_clearance(pose.xy, ahead, o)[0] >= self._planner.turn_safe for o in others):
            c = self.cfg.planner
            enter = 90.0 if dist < c.no_turn_near_m else                 max(c.yaw_enter_deg, math.degrees(math.asin(min(1.0, m.place_lateral_tol_m / dist))))
        nav = self._drive.update(pose.xy, pose.yaw_deg, goal, enter_deg=enter)
        if nav.mode == DriveMode.ROTATE:
            if self._turn_sweeps(pose, nav.yaw_error_deg):
                # 정차점 쪽으로 돌면 옆 기물을 쓴다 — 밀고 돌지 않고 운반(경로 계획)으로 돌아가 다시 다가온다.
                self._log(f"nudge: 정차점 쪽 {nav.yaw_error_deg:+.0f}° 회전이 옆 기물을 쓴다 — 운반으로 돌아간다")
                self._enter(HostState.CARRY_TO_DEST)
                return self._stop("nudge turn blocked by a piece")
            return self._rotate(nav.yaw_error_deg)
        if nav.mode == DriveMode.STOP:
            # 시퀀서가 직진<->회전 사이에 한 사이클 세운다. 그 한 박자를 지킨다.
            return self._stop("nudge (settle)")
        self._nudge_drove = True
        self.last_cmd_text = "nudge"
        return HostCommand(WIRE_STATE[self.state], linear_x=self.cfg.drive.nudge_mps)

    #: 넣기 전 차체가 섰다고 보는 기준 — 최근 STILL_S 동안 마커가 STILL_M 안에서만 움직였다.
    STILL_S = 0.3
    STILL_M = 0.015

    def _standing_still(self) -> bool:
        recent = [xy for t, xy in self._pose_hist if self._now - t <= self.STILL_S + 1e-6]
        if len(recent) < 2 or self._now - min(t for t, _ in self._pose_hist) < self.STILL_S - 1e-6:
            return False
        return max(_dist(a, b) for a in recent for b in recent) <= self.STILL_M

    def _drop_clears_rim(self, pose: Pose, target: BasketTarget) -> bool:
        """여기서 겨누는 점을 보고 넣으면 그 선이 바구니 입구(앞면)를 양쪽 벽에서 basket_rim_margin_m 안쪽으로 지나는가."""
        a, m = self.cfg.arena, self.cfg.mission
        bx, by, _yaw = a.boxes[self.dest_box]
        w, l = a.box_size[0] / 2.0, a.box_size[1] / 2.0
        edge = by - l
        ax, ay = target.aim
        if ay - pose.y < 1e-6 or pose.y >= edge:
            return False
        k = (edge - pose.y) / (ay - pose.y)
        cx = pose.x + k * (ax - pose.x)
        return bx - w + m.basket_rim_margin_m <= cx <= bx + w - m.basket_rim_margin_m

    def _pause_timers(self, dt: float) -> None:
        """차체 컨트롤러 복구로 서 있던 시간은 바구니 앞 맞추기 제한 시간에 넣지 않는다(10-08 룩: 회전 폭주 복구
        13 s 가 25 s 에 들어가 복구 직후 "제자리에 못 섰다" HALTED)."""
        if self.state == HostState.NUDGE_BOX:
            self._nudge_started += dt

    def _nudge_missed(self, why: str) -> HostCommand:
        m = self.cfg.mission
        pose = self._pose_now
        if pose is not None and self.dest_xy is not None and _dist(pose.xy, self.dest_xy) <= m.place_arrive_tol_m:
            # 이미 정차점에 서 있다 — 운반으로 돌아가도 곧바로 여기로 다시 넘어와 같은 결과다(10-08: 0.2 s 에 3번).
            self._halt(f"could not stand in front of {self.dest_box}: {why} — 치워 주세요")
            return self._stop("halted")
        self.place_tries += 1
        if self.place_tries > m.place_retry_max:
            self._halt(f"could not stand in front of {self.dest_box} ({self.place_tries} tries): {why}")
            return self._stop("halted")
        self._log(f"{why} — approach again ({self.place_tries}/{m.place_retry_max})")
        self._enter(HostState.CARRY_TO_DEST)
        return self._stop("nudge missed")

    def _back_away_from_box(self, pose: Pose, why: str) -> HostCommand:
        """상자에서 **곧장 멀어지는 쪽(-y)**으로 옆걸음 섞어 물러난다 — 차체가 어디를 보든.
        정차점을 지나쳤을 때만 쓴다.

        2026-09-30: 158°(옆)로 선 채 차체 방향으로 후진했더니 상자와의 간격이 거의 안 늘어
        "물러나기 <-> 밀기"를 되풀이했다. 메카넘이라 차체를 돌리지 않고 -y 로 갈 수 있다.
        월드 (0, -v) 를 차체 좌표로: vx = -v·sinθ, vy = -v·cosθ.
        """
        v = self.cfg.drive.nudge_mps
        th = math.radians(pose.yaw_deg)
        self.last_cmd_text = why
        return HostCommand(WIRE_STATE[self.state], linear_x=-v * math.sin(th), linear_y=-v * math.cos(th))

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
                self._enter(HostState.FACE_HAND if self.dest_kind == "hand" else HostState.NUDGE_BOX)
                return self._stop("place retry")
            self.ready_to_advance = True
            if self._should_advance():
                where = f"hand {self.hand_spot}" if self.dest_kind == "hand" else self.dest_box
                self._log(f"{self.target_label} delivered to {where}")
                if self.order is not None:
                    self.order.done += 1
                    self.order.handed += int(self.dest_kind == "hand")
                    if self.order.quantity == "one":
                        self._finish_order("done")
                self._clear_target()
                self._enter(HostState.SEARCH_TARGET)
                return self._stop("place done")
        hand = self.dest_kind == "hand"
        self.last_cmd_text = f"{'HANDOVER' if hand else 'PLACE'} (wait, arm {self.place_arm_yaw_deg:+.1f}도)"
        return HostCommand(State.PLACE, stop=True, label=self.target_label or "",
                           arm_yaw_deg=self.place_arm_yaw_deg, place_pose="handover" if hand else "")

    # ------------------------------------------------------------------ 도우미
    def _drive_to(self, pose: Pose, goal: XY, obstacles: list[XY]) -> HostCommand:
        held = self._obstacles.update(obstacles)
        sub_goal, corner, blocked = self._planner.update(pose.xy, goal, held, now=self._now)
        c = self.cfg.planner
        if _dist(pose.xy, sub_goal) < c.min_heading_dist_m and _dist(sub_goal, goal) > 1e-6:
            # 경로 중간 점이 5 cm 안이면 방향을 잴 수 없어(노이즈로 봄) 보던 방향 그대로 직진했다 — 10-07 별을 잡은 직후
            # 바구니 반대쪽으로 5 cm. 도착 판정이 아니라 지나는 점이니 그다음 점을 보고, 없으면 그 구간 방향으로 늘린다.
            nxt = corner
            path = self._planner.last_path or []
            if nxt is None and sub_goal in path and path.index(sub_goal) >= 1:
                prev = path[path.index(sub_goal) - 1]
                d = _dist(prev, sub_goal)
                if d > 1e-6:
                    ext = 2.0 * c.min_heading_dist_m
                    nxt = (sub_goal[0] + (sub_goal[0] - prev[0]) / d * ext, sub_goal[1] + (sub_goal[1] - prev[1]) / d * ext)
            if nxt is not None:
                sub_goal = nxt
        enter, tol = self._enter_deg(pose, sub_goal, held), None
        narrow, inside, at_entry = self._narrow_gap(pose.xy, sub_goal, held)
        if inside:
            # 틈 안 — 여기서 돌면 차체 모서리가 옆 기물을 쓴다. 웬만큼 틀어져도 직진으로 빠져나간다.
            enter = max(enter if enter is not None else c.yaw_enter_deg, c.narrow_hold_enter_deg)
        elif narrow:
            # 틈으로 가는 구간 — 어차피 돌 때(출발·꺾는 점)는 정확히 맞춘다. 직진 중 멈춰 다시 맞추는 건
            # 입구 바로 앞에서만(10-07: 구간 전체에 걸었더니 조금 가고 돌기를 되풀이했다).
            tol = c.narrow_yaw_tolerance_deg
            if at_entry:
                enter = c.narrow_entry_enter_deg
        if narrow and not self._narrow_logged:
            self._narrow_logged = True
            self._log(f"narrow gap ahead — 틈 밖에서 {c.narrow_yaw_tolerance_deg:.1f}° 까지 맞추고 들어간다"
                      if not inside else "narrow gap — 틈 안에서는 돌지 않고 직진")
        elif not narrow:
            self._narrow_logged = False
        nav = self._drive.update(pose.xy, pose.yaw_deg, sub_goal, enter, tol)
        self.nav_goal = goal
        self.nav_path = self._planner.last_path
        self.blocked_by = blocked
        if nav.mode == DriveMode.FORWARD:
            self.last_cmd_text = "forward" + (f" ({blocked})" if blocked else "")
            return HostCommand(WIRE_STATE[self.state], linear_x=self.cfg.drive.linear_mps)
        if nav.mode == DriveMode.ROTATE:
            turn = nav.yaw_error_deg
            if self._turn_blocked(pose, turn, held):
                other = turn - 360.0 if turn > 0 else turn + 360.0
                at_basket = self._near_basket(pose)
                # 바구니 앞에서는 앞으로 나가면 바구니 쪽이다 — 반대 방향이 비었으면 그쪽으로 돈다(10-07 사용자:
                # 바구니 옆 기물 반대쪽으로 돌기). 그 밖에서는 앞으로 빠져나가기가 먼저(작은 회전이 자연스럽다).
                if at_basket and not self._turn_blocked(pose, other, held):
                    return self._rotate_other_way(turn, other)
                ahead = self._forward_exit(pose, turn, held)
                if ahead is not None:
                    return ahead
                if not at_basket and not self._turn_blocked(pose, other, held):
                    return self._rotate_other_way(turn, other)
                if at_basket:
                    g_turn, g_other = self._turn_min_gap(pose, turn), self._turn_min_gap(pose, other)
                    if max(g_turn, g_other) < 0.0:
                        # 어느 쪽으로 돌아도 닿는다(10-07 상자: 퀸 옆에서 돌다 −2 cm). 밀고 돌지 않고 사람을 부른다.
                        who = min(self._other_pieces(pose), key=lambda o: _dist(o, pose.xy), default=None)
                        name = self._piece_label_at(who) if who is not None else "기물"
                        self._halt(f"바구니 앞에서 어느 쪽으로 돌아도 {name} 에 닿는다 — 치워 주세요")
                        return self._stop("halted")
                    if g_other > g_turn + 0.005:
                        # 양쪽 다 쓸고 앞은 바구니 — 덜 붙는 쪽으로(후진 없음). 정차 구역이 돌 수 있는 자리를 먼저 고르니 드물다.
                        return self._rotate_other_way(turn, other)
            else:
                self._other_way_logged = False
            self._exit_logged = False
            return self._rotate(turn)
        return self._stop("stop" + (f" ({blocked})" if blocked else ""))

    def _turn_blocked(self, pose: Pose, turn_deg: float, obstacles) -> bool:
        """제자리에서 turn_deg 돌면 기물을 쓰는가 — 계획기 모서리 검사 또는 차체 사각형 간격(basket_stop_clear_m).

        10-07 6기물: 바구니에서 떠나며 계획기 검사는 통과했는데 차체 사각형으로는 나이트와 −0.8 · −1.1 cm."""
        pl = self._planner
        if any(_dist(pose.xy, o) < pl.turn_safe
               and pl._sweep_hits(pose.xy, pose.yaw_deg, turn_deg, o, slack_deg=self.SWEEP_SLACK_DEG)
               for o in obstacles):
            return True
        return self._turn_sweeps(pose, turn_deg)

    def _turn_min_gap(self, pose: Pose, turn_deg: float) -> float:
        """제자리에서 turn_deg 도는 동안 차체–다른 기물 최소 간격(m)."""
        c = self.cfg.planner
        sgn = 1.0 if turn_deg >= 0 else -1.0
        steps = max(1, int(math.ceil(abs(turn_deg))))
        others = [o for o in self._other_pieces(pose) if _dist(pose.xy, o) < self._planner.turn_safe + 0.05]
        return min((body_gap(pose.x, pose.y, pose.yaw_deg + sgn * abs(turn_deg) * k / steps, o, c.robot_length_m,
                             c.robot_width_m, c.piece_obstacle_radius_m) for o in others for k in range(steps + 1)),
                   default=1.0)

    def _rotate_other_way(self, turn: float, other: float) -> HostCommand:
        if not self._other_way_logged:
            self._other_way_logged = True
            self._log(f"turn other way: {turn:+.0f}° 쪽은 옆 기물을 쓴다 — 반대로 {other:+.0f}° 돈다 ({self.state.name})")
        self._exit_logged = False
        return self._rotate(other)

    def _near_basket(self, pose: Pose) -> bool:
        a, r = self.cfg.arena, self.cfg.mission.basket_near_m
        w, l = a.box_size[0] / 2.0, a.box_size[1] / 2.0
        return any(bx - w - r <= pose.x <= bx + w + r and by - l - r <= pose.y <= by + l + r
                   for bx, by, _yaw in a.boxes.values())

    def _front_hits_basket(self, at: XY, fwd: XY) -> bool:
        """at 에 섰을 때 차체 앞면이 바구니 사각형(+ basket_front_clear_m) 안에 드는가."""
        a, c = self.cfg.arena, self.cfg.planner
        m = self.cfg.mission.basket_front_clear_m
        w, l = a.box_size[0] / 2.0 + m, a.box_size[1] / 2.0 + m
        hl, hw = c.robot_length_m / 2.0, c.robot_width_m / 2.0
        side = (-fwd[1], fwd[0])
        pts = [(at[0] + fwd[0] * hl + side[0] * hw * k, at[1] + fwd[1] * hl + side[1] * hw * k)
               for k in (-1.0, -0.5, 0.0, 0.5, 1.0)]
        return any(abs(px - bx) <= w and abs(py - by) <= l
                   for bx, by, _yaw in a.boxes.values() for px, py in pts)

    def _forward_exit(self, pose: Pose, turn_deg: float, obstacles) -> Optional[HostCommand]:
        """여기서 turn_deg 만큼 돌면 차체가 옆 기물을 쓸 때, 앞이 비었으면 앞으로 빠져나간 뒤 돈다.

        2026-10-07: 틈 바로 너머의 공을 틈 한가운데서 잡고, 바구니 쪽으로 그 자리에서 돌다 양옆 상자·룩을
        10 cm 씩 밀었다. 잡은 직후라 앞(기물이 있던 자리)은 비어 있다 — "직선으로 가고 도착해서 yaw" 대로
        앞으로 조금 나간 뒤 돈다. 후진은 하지 않는다(10-07 사용자 결정). 앞도 막혔으면 None(그 자리에서 돈다).
        """
        pl, c, d = self._planner, self.cfg.planner, self.cfg.drive
        hits = lambda at: self._turn_blocked(Pose(at[0], at[1], pose.yaw_deg, True), turn_deg, obstacles)  # noqa: E731
        if not hits(pose.xy):
            return None
        pad = c.piece_obstacle_radius_m + c.obstacle_margin_m
        hl = c.robot_length_m / 2.0
        th = math.radians(pose.yaw_deg)
        fwd = (math.cos(th), math.sin(th))
        for k in range(1, int(round(d.turn_exit_max_m / 0.01)) + 1):
            s = 0.01 * k
            end = (pose.x + s * fwd[0], pose.y + s * fwd[1])
            if not (pl.x0 <= end[0] <= pl.x1 and pl.y0 <= end[1] <= pl.y1):
                return None
            if any(segment_hits_rect(pose.xy, end, r) for r in pl._active_keepouts(pose.xy)):
                return None
            if self._front_hits_basket(end, fwd):
                return None                     # 바구니 근처에선 금지 구역이 꺼져 있다 — 앞면으로 직접 본다(10-07 돌진)
            for o in obstacles:                 # 앞면에 닿는 기물이 있으면 더 못 간다
                dx, dy = o[0] - pose.x, o[1] - pose.y
                lx, ly = dx * fwd[0] + dy * fwd[1], -dx * fwd[1] + dy * fwd[0]
                if lx > 0 and abs(ly) < pl.safe and lx - s < hl + pad:
                    return None
            if not hits(end):
                if not self._exit_logged:
                    self._exit_logged = True
                    self._log(f"turn exit: 여기서 {turn_deg:+.0f}° 돌면 옆 기물에 닿는다 — "
                              f"앞으로 {s * 100:.0f} cm 나간 뒤 돈다 ({self.state.name})")
                self.last_cmd_text = f"exit forward ({s * 100:.0f} cm)"
                return HostCommand(WIRE_STATE[self.state], linear_x=d.linear_mps,
                                   label=self.target_label or "")
        return None

    def _narrow_gap(self, at: XY, sub_goal: XY, obstacles) -> tuple[bool, bool, bool]:
        """(지금 가는 직선이 좁은 틈을 지나는가, 지금 그 틈 안에 있는가, 틈 입구 바로 앞인가).

        좁은 틈 = 직진으로는 지나가지만(차체 반폭 여유 safe 이상) 제자리 회전 반경(turn_safe) 안을
        지나는 곳. 그 안에서 돌면 모서리가 기물을 쓴다(10-05 공 · 별). 후진 없이 피하려면 틈 밖에서
        방향을 정확히 맞추고 들어가 안에서는 돌지 않는다(10-07 사용자 결정).
        """
        pl = self._planner
        dx, dy = sub_goal[0] - at[0], sub_goal[1] - at[1]
        side = lambda o: (dx * (o[1] - at[1]) - dy * (o[0] - at[0])) > 0      # noqa: E731  왼쪽이면 True
        near = [o for o in obstacles if segment_circle_clearance(at, sub_goal, o)[0] < pl.turn_safe]
        # 틈 = 직선 **양쪽**에 기물이 있을 때만. 한쪽 옆을 붙어 지나가는 우회(10-07 상자 바깥)는 틈이 아니다 —
        # 그걸 틈으로 봐서 5° 재정렬이 우회 내내 걸려 좌우로 왔다갔다 했다.
        if not ({side(o) for o in near} >= {True, False}):
            return False, False, False
        inside = any(_dist(at, o) < pl.turn_safe for o in near)
        entry = pl.turn_safe + self.cfg.planner.narrow_entry_zone_m
        at_entry = any(_dist(at, o) < entry for o in near)
        return True, inside, at_entry

    def _enter_deg(self, pose: Pose, sub_goal: XY, obstacles) -> Optional[float]:
        """직진 중 다시 돌기 시작하는 문턱. 앞길이 비어 있으면 넓힌다(None = 기본 12°).

        앞길 = 지금 방향으로 부분목표까지의 거리만큼 곧장 간 선. 그 선이 기물(직진 여유)·상자
        금지 구역·주행 구역 밖에 닿지 않으면, 조금 틀어진 채 가도 부딪힐 것이 없다 — 어긋난 만큼은
        계획기가 지금 자리에서 다시 짠 경로가 메운다. 부분목표 바로 앞(지나치는 중)에서는 뒤로
        가는 게 아니면 돌지 않는다.
        """
        c = self.cfg.planner
        pl = self._planner
        dist = _dist(pose.xy, sub_goal)
        th = math.radians(pose.yaw_deg)
        ahead = (pose.x + dist * math.cos(th), pose.y + dist * math.sin(th))
        if not (pl.x0 <= ahead[0] <= pl.x1 and pl.y0 <= ahead[1] <= pl.y1):
            return None
        if any(segment_hits_rect(pose.xy, ahead, r) for r in pl._active_keepouts(pose.xy)):
            return None
        if any(segment_circle_clearance(pose.xy, ahead, o)[0] < pl.safe for o in obstacles):
            return None
        if dist < c.no_turn_near_m:
            return 90.0
        return c.yaw_enter_clear_deg

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

    def _other_pieces(self, pose: Pose) -> list[XY]:
        """지도 위 기물 중 목표(잡으러 가는 것)·쥔 기물을 뺀 것."""
        r_hold = self.cfg.planner.carry_ignore_radius_m
        holding = self.state in (HostState.CARRY_TO_DEST, HostState.NUDGE_BOX, HostState.FACE_HAND,
                                 HostState.PLACE)
        out = []
        for label, pts in (self._pmap_now or {}).items():
            for p in pts:
                if self.target_xy is not None and not holding and _dist(p, self.target_xy) <= 0.05:
                    continue                    # 잡으러 가는 기물(정면에 0.26 m 이상 떨어져 있다)
                if holding and label == self.target_label and _dist(p, pose.xy) <= r_hold:
                    continue                    # 쥐고 있는 기물(그리퍼 안에서 계속 보인다)
                out.append(p)
        return out

    def _turn_sweeps(self, pose: Pose, turn_deg: float) -> bool:
        """여기서 turn_deg 만큼 제자리에서 돌면(실제 각 + 3°) 차체–다른 기물 간격이 basket_stop_clear_m 아래로 가는가.

        바구니·손 앞에서 쓰는 검사 — 정차 구역 자리 고르기와 **같은 기준**이다(10-07: 구역은 1.5 cm 로 골랐는데
        여기선 계획기 여유 4 cm 로 봐서 "들어갔다 돌아가기"를 되풀이하다 HALTED)."""
        c, clear = self.cfg.planner, self.cfg.mission.basket_stop_clear_m
        sgn = 1.0 if turn_deg >= 0 else -1.0
        span = abs(turn_deg) + self.SWEEP_SLACK_DEG
        steps = max(1, int(math.ceil(span)))
        angles = [pose.yaw_deg - sgn * self.SWEEP_SLACK_DEG + sgn * span * k / steps for k in range(steps + 1)]
        others = [o for o in self._other_pieces(pose) if _dist(pose.xy, o) < self._planner.turn_safe + 0.05]
        return any(body_gap(pose.x, pose.y, a, o, c.robot_length_m, c.robot_width_m, c.piece_obstacle_radius_m) < clear
                   for o in others for a in angles)

    def _straight_arrival_sweeps(self, pose: Pose) -> bool:
        """지금 자리에서 정차점으로 곧장 가면(도착 방향 = 그 직선) 정차점에서 몸을 돌릴 때 기물을 쓰는가.

        팔이 ±max_arm_yaw_deg 를 메우니 그 밖만 몸으로 돈다(place_turn_to_deg 까지). 직선 자체가 막혔어도 True."""
        m, pl = self.cfg.mission, self._planner
        if not self._line_clear(pose.xy, self.dest_xy):
            return True
        # 직선이 기물의 회전 반경(turn_safe) 안을 지나면 메카넘 흐름(15 cm 에 ~10°)만으로 닿는다 — 아래로 돌아간다.
        if _dist(pose.xy, self.dest_xy) > m.place_arrive_tol_m and any(
                segment_circle_clearance(pose.xy, self.dest_xy, o)[0] < pl.turn_safe
                and _dist(o, self.dest_xy) >= pl.turn_safe for o in self._other_pieces(pose)):
            return True
        b = math.degrees(math.atan2(self.dest_xy[1] - pose.y, self.dest_xy[0] - pose.x))
        residual = facing_error_deg(self._basket(), self.dest_xy, b)
        if abs(residual) <= m.max_arm_yaw_deg:
            return False
        turn = residual - math.copysign(m.place_turn_to_deg, residual)
        return self._turn_sweeps(Pose(self.dest_xy[0], self.dest_xy[1], b, True), turn)

    def _pick_pre_stop(self, pose: Pose) -> Optional[XY]:
        """정차점 바로 아래 거쳐 갈 점. 기물에서 직진 여유(safe) + 2 cm 밖이고 거기서 정차점까지 직선이 빈 곳.

        10-07: 정차 구역이 −10 cm 로 옮겨지자 그 아래 30 cm 점이 퀸에서 5 cm 였다 — 계획기가 그 점으로 곧장 가며
        퀸을 7.7 cm 밀었다. 아래에서부터(지금 높이보다 낮게는 안 감) 위로 2.5 cm 씩 보고, 없으면 None(정차점으로 바로).
        """
        m = self.cfg.mission
        sx, sy = self.dest_xy
        # 아래에서 왔으면 지금 높이보다 낮게는 안 간다. 옆(정차점 높이)에서 왔으면 정차점 30 cm 아래부터 본다.
        lo = max(sy - m.basket_pre_stop_m, pose.y) if pose.y < sy - m.basket_low_margin_m else sy - m.basket_pre_stop_m
        hi = sy - m.basket_low_margin_m
        others = self._other_pieces(pose)
        clear = self._planner.safe + 0.02
        y = lo
        while y <= hi + 1e-9:
            pt = (sx, y)
            if all(_dist(pt, o) >= clear for o in others) and self._line_clear(pt, self.dest_xy):
                return pt
            y += 0.025
        self._log(f"pre-stop: 정차점 아래에 기물 없는 점이 없다 — 정차점으로 바로 간다")
        return None

    def _approach_turn_sweeps(self, pose: Pose) -> bool:
        """곧장 가는 단계로 넘기면 먼저 정차점 쪽으로 돌 각도(정렬 허용치 넘을 때만)가 옆 기물을 쓰는가."""
        if self.dest_xy is None or _dist(pose.xy, self.dest_xy) < self.cfg.planner.min_heading_dist_m:
            return False
        bearing = math.degrees(math.atan2(self.dest_xy[1] - pose.y, self.dest_xy[0] - pose.x))
        turn = wrap_deg(bearing - pose.yaw_deg)
        if abs(turn) <= self.cfg.planner.yaw_tolerance_deg:
            return False
        return self._turn_sweeps(pose, turn)

    def _direct_turn_blocked(self, pose: Pose, obstacles) -> bool:
        """정차점 쪽으로 제자리에서 돌 회전이 경로 주행 기준(_turn_blocked)으로 막히는가."""
        if self.dest_xy is None or _dist(pose.xy, self.dest_xy) < self.cfg.planner.min_heading_dist_m:
            return False
        bearing = math.degrees(math.atan2(self.dest_xy[1] - pose.y, self.dest_xy[0] - pose.x))
        turn = wrap_deg(bearing - pose.yaw_deg)
        if abs(turn) <= self.cfg.planner.yaw_tolerance_deg:
            return False
        return self._turn_blocked(pose, turn, obstacles)

    def _piece_label_near(self, xy: XY) -> str:
        """xy 에 가장 가까운 기물(목표·쥔 것 제외)의 라벨 — 사람에게 알릴 때."""
        pose = self._pose_now
        others = set(self._other_pieces(pose)) if pose is not None else set()
        best = min(((lb, p) for lb, pts in (self._pmap_now or {}).items() for p in pts if p in others),
                   key=lambda t: _dist(t[1], xy), default=None)
        return best[0] if best else "기물"

    def _line_clear(self, a: XY, b: XY, slack: float = 0.0) -> bool:
        """a -> b 직선을 차체가 곧장 지나가도 기물(목표·쥔 것 제외)을 스치지 않는가."""
        pose = self._pose_now
        if pose is None:
            return True
        return all(segment_circle_clearance(a, b, o)[0] >= self._planner.safe - slack
                   for o in self._other_pieces(pose))

    def _rotate(self, yaw_error_deg: float) -> HostCommand:
        # yaw_error = 목표 - 현재, 반시계가 +. 부호가 곧 회전 방향이다.
        # 오차가 작을수록 느리게 돈다 — 한 속도(0.5 rad/s)로는 지연 동안 허용치를 넘어가
        # 좌우로 떨었다(2026-09-30). 데드밴드 아래로는 내리지 않는다.
        d = self.cfg.drive
        # 거의 정반대(≥ 150°)면 처음 고른 방향을 끝까지 — 175° 와 185° 사이에서 위치 흔들림(회전 중 마커 3–5 cm)으로
        # 짧은 쪽이 바뀌어 돌다 뒤집히는 것을 막는다(10-07 "큰 쪽으로 도는 것처럼 보인다").
        if self._turn_latch is not None and self._now - self._turn_latch[1] <= 0.5 and abs(yaw_error_deg) > 90.0 \
                and (yaw_error_deg >= 0) != (self._turn_latch[0] > 0):
            yaw_error_deg = yaw_error_deg - 360.0 if yaw_error_deg > 0 else yaw_error_deg + 360.0
        if abs(yaw_error_deg) >= 150.0 and (self._turn_latch is None or self._now - self._turn_latch[1] > 0.5):
            self._turn_latch = (1.0 if yaw_error_deg >= 0 else -1.0, self._now)
        elif self._turn_latch is not None:
            self._turn_latch = (self._turn_latch[0], self._now) if abs(yaw_error_deg) > 90.0 else None
        sign = 1.0 if yaw_error_deg >= 0 else -1.0
        scale = min(1.0, abs(yaw_error_deg) / max(d.rotation_slow_deg, 1e-6))
        speed = max(d.rotation_min_rad_s, d.rotation_rad_s * scale)
        self.last_cmd_text = "yaw+" if sign > 0 else "yaw-"
        return HostCommand(WIRE_STATE[self.state], angular_z=sign * speed)

    def _refresh_target(self, pmap: PieceMap) -> None:
        """목표 기물 위치를 탑뷰로 갱신한다. 같은 라벨이 target_track_m 안에 있으면 그것이다.
        안 보이면 마지막 위치를 그대로 쓴다(팔이나 차체가 가렸을 수 있다)."""
        if self.target_xy is None or not self.target_label:
            return
        near = [p for p in pmap.get(self.target_label, [])
                if _dist(p, self.target_xy) <= self.cfg.mission.target_track_m]
        if near:
            self.target_xy = min(near, key=lambda p: _dist(p, self.target_xy))

    def _stop(self, why: str) -> HostCommand:
        self.last_cmd_text = f"stop: {why}"
        return HostCommand(WIRE_STATE[self.state], stop=True, label=self.target_label or "")

    def _enter(self, state: HostState) -> None:
        self.state = state
        self.ready_to_advance = False
        self._in_grasp_zone = False
        self._creep = 0
        self._creep_stopped_at = None
        self._creep_tries = 0
        self._creep_cmds = 0
        if state in (HostState.GRASP, HostState.PLACE):
            self._job_armed = False
            self._job_result = None
        if state == HostState.FACE_HAND:
            self._face_turning = False
            self._face_arrived = False
        if state == HostState.CARRY_TO_DEST:
            self._box_stop_chosen = False
            self._zone_blocked_since = None
            self._aim_shift = 0.0
            self._pre_stop = None
        if state == HostState.NUDGE_BOX:
            self._nudge_from = None
            self._along_turning = False
            self._along_logged = False
            self._along_rot = None
            self._nudge_drove = False
            self._tight_turn_logged = False
            self._nudge_turn_logged = False
            self._arm_only_place = False
            self._nudge_started = self._now
        self._reset_motion()

    def _go_back(self) -> None:
        prev = _PREV.get(self.state)
        if prev is None:
            return
        if self.state == HostState.HALTED:
            # 사람이 치웠다 — 쥔 채 멈췄으면 운반부터(정차 자리를 다시 고른다. 10-08: 상자 앞 맞추기로 곧장 가면
            # 막혔던 예전 자리를 그대로 썼다), 잡기 전에 멈췄으면 다가가기부터.
            held = (HostState.CARRY_TO_DEST, HostState.NUDGE_BOX, HostState.FACE_HAND, HostState.PLACE)
            if self._halted_from in held:
                prev = HostState.CARRY_TO_DEST
            elif self._halted_from in (HostState.APPROACH_PIECE, HostState.GRASP) and self.target_xy is not None:
                prev = HostState.APPROACH_PIECE
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

    # ------------------------------------------------------------------ 손에 건네기
    def _hand_near(self, xy: XY) -> Optional[XY]:
        r = self.cfg.handover.match_radius_m
        near = [h for h in self.hands if _dist(h, xy) <= r]
        return min(near, key=lambda h: _dist(h, xy)) if near else None

    def _choose_destination(self, pose_xy: Optional[XY]) -> None:
        """파지 직후. "가져와" 지시면 보이는 손(로봇에서 가장 가까운)의 위치 정차점, 아니면 바구니.
        손이 하나도 안 보이면 바구니로 간다(지시 접수 때 손을 확인하므로 드물다)."""
        if self.order is None or self.order.intent != "fetch":
            return
        spots = self.cfg.handover.spots
        if not self.hands:
            self._log("fetch: 손이 안 보인다 — 바구니로")
            return
        ref = pose_xy or self.target_xy or (0.0, 0.0)
        hand = min(self.hands, key=lambda h: _dist(h, ref))
        spot = min(spots, key=lambda n: _dist(hand, spots[n][:2]))
        self.dest_kind, self.hand_spot, self.hand_xy = "hand", spot, hand
        self.dest_box = None
        self.dest_xy = tuple(spots[spot][2:])
        self._log(f"fetch: 손 {spot} ({hand[0]:.2f},{hand[1]:.2f}) -> 정차 "
                  f"({self.dest_xy[0]:.2f},{self.dest_xy[1]:.2f})")

    def _to_basket(self, why: str) -> None:
        self._log(f"{why} — 바구니로")
        self.dest_kind, self.hand_spot, self.hand_xy = "box", None, None
        self._hand_wait_since = None
        self.dest_box = self.cfg.mission.piece_dest_box.get(self.target_label or "",
                                                            next(iter(self.cfg.arena.boxes)))
        self.dest_xy = self._box_front_xy(self.dest_box)
        self._enter(HostState.CARRY_TO_DEST)

    def _step_face_hand(self, pose: Pose) -> HostCommand:
        """손 앞 정차점에서 손 쪽을 본다. 남는 각도(±max_arm_yaw_deg)는 팔 base 가 메운다 — 상자 투입과 같다.
        손이 안 보이면 hand_wait_s 동안 그 자리에서 기다리고, 넘으면 바구니로 간다."""
        self._clear_nav()
        m, hc = self.cfg.mission, self.cfg.handover
        assert self.hand_spot is not None and self.dest_xy is not None
        # 운반은 정차점 place_trigger_dist_m 안에서 끝난다 — 남은 거리는 상자 앞처럼 곧장 붙는다.
        # 한 번 붙은 뒤에는 조금(돌며 생기는 흔들림) 벗어나도 다시 움직이지 않는다.
        dist = _dist(pose.xy, self.dest_xy)
        if dist > (2.5 * hc.arrive_tol_m if self._face_arrived else hc.arrive_tol_m):
            self._face_arrived = False
            self.ready_to_advance = False
            nav = self._drive.update(pose.xy, pose.yaw_deg, self.dest_xy)
            if nav.mode == DriveMode.ROTATE:
                if self._turn_sweeps(pose, nav.yaw_error_deg):
                    self._log(f"to hand: 정차점 쪽 {nav.yaw_error_deg:+.0f}° 회전이 옆 기물을 쓴다 — 운반으로 돌아간다")
                    self._enter(HostState.CARRY_TO_DEST)
                    return self._stop("hand turn blocked by a piece")
                return self._rotate(nav.yaw_error_deg)
            if nav.mode == DriveMode.STOP:
                return self._stop("to hand stop (settle)")
            self.last_cmd_text = f"to hand stop ({dist:.2f} m)"
            return HostCommand(WIRE_STATE[self.state], linear_x=self.cfg.drive.nudge_mps)
        if not self._face_arrived:
            self._face_arrived = True
            self._reset_motion()
        hand = self._hand_near(hc.spots[self.hand_spot][:2])
        if hand is None:
            if self._hand_wait_since is None:
                self._hand_wait_since = self._now
                self._log(f"handover: 손을 기다린다 ({self.hand_spot}, 최대 {hc.hand_wait_s:.0f}s)")
            if self._now - self._hand_wait_since > hc.hand_wait_s:
                self._to_basket(f"손이 {hc.hand_wait_s:.0f}s 동안 안 보였다")
                return self._stop("no hand")
            self.ready_to_advance = False
            return self._stop(f"waiting for hand ({self.hand_spot})")
        self._hand_wait_since = None
        self.hand_xy = hand
        heading = math.degrees(math.atan2(hand[1] - pose.y, hand[0] - pose.x))
        residual = wrap_deg(heading - pose.yaw_deg)
        self.place_arm_yaw_deg = residual
        limit = m.place_turn_to_deg if self._face_turning else m.max_arm_yaw_deg
        self.ready_to_advance = abs(residual) <= limit
        if self.ready_to_advance:
            if self._should_advance():
                self._enter(HostState.PLACE)
                return self._step_place(None)
            return self._stop(f"at hand (arm {residual:+.1f}도)")
        self._face_turning = True
        return self._rotate(residual)

    def _clear_target(self) -> None:
        self.target_label = None
        self.target_xy = None
        self.dest_box = None
        self.dest_xy = None
        self.dest_kind = "box"
        self.hand_spot = None
        self.hand_xy = None
        self._hand_wait_since = None
        self.place_tries = 0
        self.grasp_tries = 0
        self.grasp_face_err_deg = None

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
        self._halted_from = self.state
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

    def _nearest_piece(self, pmap: PieceMap, robot_xy: XY, skips: list[XY],
                       labels: Optional[tuple[str, ...]] = None) -> Optional[tuple[str, XY]]:
        """작업영역 안 + 목적지 상자가 있는 라벨 + 보류되지 않은 것 중 최근접.
        y 가 작업영역 밖이면 상자 자리(이미 옮긴 것)라 뺀다. labels 가 있으면 그 라벨만(지시)."""
        best, best_d = None, math.inf
        r = self.cfg.mission.skip_radius_m
        for label, pts in pmap.items():
            if label not in self.cfg.mission.piece_dest_box:
                continue
            if labels is not None and label not in labels:
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
        t = basket_target(name, (bx, by), self.cfg.arena.box_size,
                          m.insert_half_width_m, m.insert_inset_depth_m,
                          m.place_aim_margin_m)
        if self._aim_shift and box is None:
            # 정차 구역에서 좌우로 비켜 섰다 — 겨누는 점·판정 사각형도 같이 옮긴다(입구 안).
            d = self._aim_shift
            x0, x1, y0, y1 = t.rect
            t = replace(t, center=(t.center[0] + d, t.center[1]), rect=(x0 + d, x1 + d, y0, y1),
                        aim=(t.aim[0] + d, t.aim[1]))
        return t

    def _choose_box_stop(self, pose: Pose) -> Optional[str]:
        """바구니 정차 구역에서 설 자리를 고른다. 못 고르면 막고 있는 기물 라벨, 골랐으면 None.

        y 는 정차점 그대로(더 붙으면 돌 때 상자에 닿고, 덜 붙으면 팔이 테두리를 못 넘는다), x 만
        가운데 ±basket_stop_zone_half_m 에서 basket_stop_step_m 간격으로 가운데부터 본다. 후보마다
          - 바구니를 보고(90° ± basket_stop_turn_deg) 섰을 때 차체와 다른 기물 사이가 basket_stop_clear_m 이상인가
        를 본다(10-07: 정차점 한 점이라 바로 옆 나이트와 바퀴가 1 cm 였다). 들어오는 길은 여기서 보지 않는다 —
        어느 쪽에서든 직선이 비면 넘기는 운반 단계 규칙(직선 검사 · 5 s HALTED)이 맡는다.
        """
        m, pl = self.cfg.mission, self._planner
        cx, cy = self._box_front_xy(self.dest_box)
        others = self._other_pieces(pose)
        n = int(round(m.basket_stop_zone_half_m / m.basket_stop_step_m))
        offsets = [0.0] + [s * k * m.basket_stop_step_m for k in range(1, n + 1) for s in (-1.0, 1.0)]
        blocker: Optional[XY] = None
        c = self.cfg.planner
        # 넣고 떠날 때 그 자리에서 어느 쪽으로든 돌 수 있는 자리를 먼저 고른다(모서리 반경 + 기물 반경 + 간격).
        # 10-07 6기물: −6 cm 에 서서 넣고 떠나며 도는 동안 나이트와 −0.8 · −1.1 cm(앞은 바구니, 후진 없음).
        corner_r = math.hypot(c.robot_length_m / 2.0, c.robot_width_m / 2.0)
        # 실제로 서는 자리는 고른 자리에서 basket_stop_pos_err_m 어긋난다(별: 0.93 고르고 0.958 에 섬) — 그만큼 더.
        free_r = corner_r + c.piece_obstacle_radius_m + m.basket_stop_clear_m + m.basket_stop_pos_err_m
        free = {dx for dx in offsets if all(_dist((cx + dx, cy), o) >= free_r for o in others)}
        # 로봇에서 가장 가까운 자리부터(10-08 사용자: 정해 둔 한 점이 아니라 가장 가까운 정차 지역으로 가서 바로 넣기).
        # 떠날 때 어느 쪽으로든 돌 수 있는 자리는 basket_stop_free_bonus_m 만큼 가깝게 친다(10-07 나이트 −0.8 cm).
        offsets.sort(key=lambda dx: _dist(pose.xy, (cx + dx, cy)) - (m.basket_stop_free_bonus_m if dx in free else 0.0))
        for dx in offsets:
            stop = (cx + dx, cy)
            if not (pl.x0 <= stop[0] <= pl.x1):
                continue
            # 바구니를 보고(90° ± basket_stop_turn_deg) 섰을 때 차체 바깥면–기물 가장자리가 basket_stop_clear_m 이상.
            # 10-07: ±15° 회전 · 여유 4 cm 로 보다가 "그렇게 여유가 없진 않았다" — 정차점 몸 회전은 팔로만 넣기 규칙이 따로 본다.
            # 실제로 서는 방식대로 본다: 팔이 ±max_arm_yaw_deg 를 메우니 몸은 그만큼 틀어진 채 서고(10-07 실기 79°),
            # 정차 위치도 좌우로 basket_stop_pos_err_m 어긋난다(0.95 고르고 0.964 에 섰다 → 나이트와 1.2 cm).
            turn, err = int(m.basket_stop_turn_deg), m.basket_stop_pos_err_m
            swept = [o for o in others
                     if any(body_gap(stop[0] + e, stop[1], 90.0 + a, o, c.robot_length_m, c.robot_width_m,
                                     c.piece_obstacle_radius_m) < m.basket_stop_clear_m
                            for a in range(-turn, turn + 1) for e in (-err, 0.0, err))]
            if swept:
                blocker = blocker or swept[0]
                continue
            self.dest_xy = stop
            self._aim_shift = max(-m.basket_aim_shift_max_m, min(m.basket_aim_shift_max_m, dx))
            if dx:
                self._log(f"basket stop zone: 좌우 {dx * 100:+.0f} cm 에 선다 — 로봇에서 가장 가까운 자리 "
                          f"(겨누는 점 {self._aim_shift * 100:+.0f} cm)")
            return None
        if blocker is None:
            return "기물"
        return f"{self._piece_label_at(blocker)} ({blocker[0]:.2f}, {blocker[1]:.2f})"

    def _piece_label_at(self, xy: XY) -> str:
        for lb, pts in (self._pmap_now or {}).items():
            if any(_dist(p, xy) < 1e-6 for p in pts):
                return lb
        return "기물"

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
