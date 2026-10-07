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
        # 정차 소음 줄이기: 마지막 움직임이 회전이었는가, 그 방향, 반대 회전 진행 상태
        self._rot_since_unwind = False
        self._rot_accum = 0.0           # 반대 회전 뒤 돈 양(rad) — 반대 회전 길이를 여기에 맞춘다
        self._last_rot_sign = 1.0
        self._unwind_until: Optional[float] = None
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
        self._pose_now: Optional[Pose] = None
        self._pmap_now: Optional[PieceMap] = None
        self._unwind_sign = -1.0
        self._unwound = False
        self._translating_since: Optional[float] = None
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
        self._now = now
        if pose.ok:
            self._last_pose_xy = pose.xy
        self._pose_now, self._pmap_now = pose, piece_map      # 직선이 비었는지(_line_clear)용
        if self._recover_started is not None and not self.estop:
            waiting = self._wait_base_recovery(pi_status)
            if waiting is not None:
                return waiting
        if (pi_status is not None and pi_status.base_recovering and not self.estop
                and self.state in _DRIVE_HOST_STATES):
            # Pi 가 스스로 컨트롤러를 다시 띄우는 중(기동 직후 · 소음 정리, 10-05) — 그동안 바퀴는 명령을
            # 안 받는다. 움직이라고 내면 무응답으로 오판하므로 끝날 때까지 정지만 낸다.
            self._stall.reset()
            self._runaway.reset()
            self._spin.reset()
            self._clear_nav()
            self.last_cmd_text = "base reset (wait)"
            return HostCommand(WIRE_STATE[self.state], stop=True, label=self.target_label or "")
        cmd = self._step_states(pose, piece_map, pi_status, now)
        # 직진·옆걸음으로 바퀴가 **충분히** 구르면 회전 뒤 버팀이 풀린다(2 s·30 cm 직진 뒤에는 조용했다).
        # 거리 맞추기 같은 짧은 움직임은 안 된다 — 10-01 star: 회전 → 1~2 cm 후진 → 파지 동안 다시 울었다.
        if not cmd.stop and (abs(cmd.linear_x) > 1e-6 or abs(cmd.linear_y) > 1e-6):
            if self._translating_since is None:
                self._translating_since = now
            if now - self._translating_since >= self.cfg.drive.unwind_clear_s:
                self._rot_since_unwind = False
                self._rot_accum = 0.0
        else:
            self._translating_since = None
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
            # 반대 회전(_unwind)을 마친 뒤에는 다시 돌지 않는다 — 1~2° 되돌아가도 그대로 잡는다.
            unwinding = self._unwound or self._unwind_until is not None
            if abs(self.grasp_face_err_deg) > m.grasp_face_tol_deg and not unwinding:
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
                self._creep = 1 if dist > hi else -1
                self._creep_tries += 1
                self._creep_cmds = 0
                creep = self._creep_to_range(dist)
                if creep is not None:
                    return creep
            # 파지 동안 오래 서 있는다 — 회전으로 멈췄으면 정차 소음이 나지 않게 반대로 짧게 돈다.
            unwind = self._unwind()
            if unwind is not None:
                return unwind
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
        self._unwind_until, self._unwound = None, False
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
        low = self._carry_from_below(pose)
        bearing = math.degrees(math.atan2(self.dest_xy[1] - pose.y, self.dest_xy[0] - pose.x))
        in_cone = (not low) or abs(wrap_deg(bearing - 90.0)) <= m.basket_approach_cone_deg
        line_ok = self._line_clear(pose.xy, self.dest_xy)
        # 10-07: 정차점 2 cm 안에 서 있는데 정차점 자체가 나이트에서 13 cm 라 "직선 막힘"으로 영원히 서 있었다.
        # 넘기면 곧장 가는 단계가 먼저 정차점 쪽으로 제자리에서 돈다(검사 없음). 그 회전이 옆 기물을 쓸면
        # 넘기지 않고 운반(앞으로 빠져나간 뒤 돌기)을 잇는다 — 10-07 궤적: 넘긴 직후 +20° 돌며 상자 기물과 −1.9 cm.
        turn_ok = not self._approach_turn_sweeps(pose)
        self.ready_to_advance = near and ((line_ok and turn_ok and in_cone) or at_stop)
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
        if self._now - self._nudge_started > m.nudge_timeout_s:
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
        close = -m.place_min_gap_m <= gap <= m.place_arrive_tol_m
        # 팔로만 넣기로 했으면 팔 한계까지만 튼다(남는 몇 도만큼 떨어지는 점이 옆으로 간다).
        arm = (max(-m.max_arm_yaw_deg, min(m.max_arm_yaw_deg, residual)) if self._arm_only_place
               else residual)
        self.place_arm_yaw_deg = arm
        # 차체를 돌리기 시작했으면 팔 한계 바로 안(14.x°)이 아니라 place_turn_to_deg(12°)까지 돈다.
        # 정면(0°)까지는 맞추지 않는다 — 나머지는 팔 base 가 맡는다(2026-09-30 저녁 요청).
        limit = m.place_turn_to_deg if self._nudge_turn_logged else m.max_arm_yaw_deg
        if self._unwound or self._unwind_until is not None:
            limit = m.max_arm_yaw_deg               # 반대 회전으로 1~2° 되돌아가도 다시 돌지 않는다
        self.ready_to_advance = close and (abs(residual) <= limit or self._arm_only_place)
        if self.ready_to_advance:
            # 투입 동안 서 있는다 — 회전으로 멈췄으면 반대로 짧게 돌아 정차 소음을 없앤다.
            unwind = self._unwind()
            if unwind is not None:
                self.place_arm_yaw_deg = arm
                return unwind
            if self._should_advance():
                self._enter(HostState.PLACE)
                return self._step_place(pi_status)
            return self._stop(f"at box (arm {residual:+.1f}도)")
        if gap < -m.place_min_gap_m:
            # 정차점을 크게 지나쳤다(드문 경우). 여기서 돌면 차체가 상자에 닿고, 시퀀서는 뒤에 있는
            # 정차점을 보려고 180° 돌려 한다 — 돌지 않고 상자에서 곧장 물러난다.
            return self._back_away_from_box(pose, "overshoot — back off")
        if close and abs(residual) > limit:
            # 팔이 못 메우는 각도다. 이때만 정차점에서 차체를 돌린다(돌아도 상자에 닿지 않는 자리다).
            # 단, 그 회전이 바구니 옆 기물을 쓸면 돌지 않는다(10-05 별 · 10-07 나이트를 쳤다).
            turn = residual - math.copysign(m.place_turn_to_deg, residual)
            pl = self._planner
            if self._turn_sweeps(pose, turn):         # 정차 구역·상자 앞 맞추기와 같은 간격 기준
                short = abs(residual) - m.max_arm_yaw_deg
                if short <= m.place_arm_only_max_short_deg:
                    self._arm_only_place = True
                    self._log(f"몸을 돌리면 바구니 옆 기물에 닿는다 — 팔로만 넣음 "
                              f"(팔 {math.copysign(m.max_arm_yaw_deg, residual):+.0f}°, 모자란 각 {short:.1f}°)")
                    return self._stop("arm-only place")
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
        nav = self._drive.update(pose.xy, pose.yaw_deg, self.dest_xy)
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
        self.last_cmd_text = "nudge"
        return HostCommand(WIRE_STATE[self.state], linear_x=self.cfg.drive.nudge_mps)

    def _nudge_missed(self, why: str) -> HostCommand:
        m = self.cfg.mission
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
                if at_basket and self._turn_min_gap(pose, other) > self._turn_min_gap(pose, turn) + 0.005:
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

    def _pick_pre_stop(self, pose: Pose) -> Optional[XY]:
        """정차점 바로 아래 거쳐 갈 점. 기물에서 직진 여유(safe) + 2 cm 밖이고 거기서 정차점까지 직선이 빈 곳.

        10-07: 정차 구역이 −10 cm 로 옮겨지자 그 아래 30 cm 점이 퀸에서 5 cm 였다 — 계획기가 그 점으로 곧장 가며
        퀸을 7.7 cm 밀었다. 아래에서부터(지금 높이보다 낮게는 안 감) 위로 2.5 cm 씩 보고, 없으면 None(정차점으로 바로).
        """
        m = self.cfg.mission
        sx, sy = self.dest_xy
        lo = max(sy - m.basket_pre_stop_m, pose.y)
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

    def _carry_from_below(self, pose: Pose) -> bool:
        """바구니로 가는데 정차점보다 basket_low_margin_m 넘게 아래에 있는가(아래에서 똑바로 들어갈 수 있다)."""
        if self.dest_kind == "hand" or not self.dest_box or self.dest_xy is None:
            return False
        return pose.y < self.dest_xy[1] - self.cfg.mission.basket_low_margin_m

    def _approach_turn_sweeps(self, pose: Pose) -> bool:
        """곧장 가는 단계로 넘기면 먼저 정차점 쪽으로 돌 각도(정렬 허용치 넘을 때만)가 옆 기물을 쓰는가."""
        if self.dest_xy is None or _dist(pose.xy, self.dest_xy) < self.cfg.planner.min_heading_dist_m:
            return False
        bearing = math.degrees(math.atan2(self.dest_xy[1] - pose.y, self.dest_xy[0] - pose.x))
        turn = wrap_deg(bearing - pose.yaw_deg)
        if abs(turn) <= self.cfg.planner.yaw_tolerance_deg:
            return False
        return self._turn_sweeps(pose, turn)

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
        self._rot_since_unwind, self._last_rot_sign = True, sign
        self._rot_accum += speed / max(self.cfg.mission.cycle_hz, 1.0)
        return HostCommand(WIRE_STATE[self.state], angular_z=sign * speed)

    def _unwind(self) -> Optional[HostCommand]:
        """오래 서 있기 전(파지·투입)에, 마지막 움직임이 제자리 회전이었으면 반대로 아주 짧게 돈다.

        2026-10-01 실기: 제자리 회전 뒤 멈춰 있으면 바퀴가 "지잉" 하고 크게 울었다(파지 중 내내).
        직진 뒤에는 조용했다. 회전 뒤 반대로 0.3 s 돌리자 소리가 멎었다 — 미끄러지며 돈 동안
        바퀴 속도 제어에 쌓인 보정이 남아 네 바퀴가 서로 버티는 것으로 본다(보드 안쪽 일이라 추정).
        끝나면 None. 이 뒤로는 정면·각도 재확인으로 다시 돌지 않는다(_unwound) — 반복하지 않게.
        """
        d = self.cfg.drive
        if d.unwind_s <= 0:
            return None
        if self._unwind_until is None:
            if not self._rot_since_unwind:
                return None
            dur = min(d.unwind_max_s, max(d.unwind_s, d.unwind_s * self._rot_accum / max(d.unwind_ref_rad, 1e-6)))
            self._unwind_until = self._now + dur
            self._unwind_sign = -self._last_rot_sign
            self._log(f"unwind {'+' if self._unwind_sign > 0 else '-'} {dur:.2f}s "
                      f"(돈 양 {math.degrees(self._rot_accum):.0f}°, {self.state.name})")
        self.ready_to_advance = False
        if self._now < self._unwind_until:
            self.last_cmd_text = "unwind"
            return HostCommand(WIRE_STATE[self.state], angular_z=self._unwind_sign * d.unwind_rad_s,
                               label=self.target_label or "")
        self._unwind_until = None
        self._rot_since_unwind = False
        self._rot_accum = 0.0
        self._unwound = True
        return self._stop("unwind (settle)")

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
        self._unwind_until = None
        self._unwound = False
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
            self._nudge_turn_logged = False
            self._arm_only_place = False
            self._nudge_started = self._now
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
        if self._unwound or self._unwind_until is not None:
            limit = m.max_arm_yaw_deg
        self.ready_to_advance = abs(residual) <= limit
        if self.ready_to_advance:
            unwind = self._unwind()
            if unwind is not None:
                return unwind
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
        if not self._carry_from_below(pose):
            # 정차점 높이 근처(옆)에서 왔다 — 옆으로 가는 거리와 첫 회전을 줄이게 로봇 쪽 자리부터 본다(10-07).
            offsets.sort(key=lambda dx: abs(cx + dx - pose.x))
        blocker: Optional[XY] = None
        c = self.cfg.planner
        # 넣고 떠날 때 그 자리에서 어느 쪽으로든 돌 수 있는 자리를 먼저 고른다(모서리 반경 + 기물 반경 + 간격).
        # 10-07 6기물: −6 cm 에 서서 넣고 떠나며 도는 동안 나이트와 −0.8 · −1.1 cm(앞은 바구니, 후진 없음).
        corner_r = math.hypot(c.robot_length_m / 2.0, c.robot_width_m / 2.0)
        # 실제로 서는 자리는 고른 자리에서 basket_stop_pos_err_m 어긋난다(별: 0.93 고르고 0.958 에 섬) — 그만큼 더.
        free_r = corner_r + c.piece_obstacle_radius_m + m.basket_stop_clear_m + m.basket_stop_pos_err_m
        free = [dx for dx in offsets if all(_dist((cx + dx, cy), o) >= free_r for o in others)]
        offsets = free + [dx for dx in offsets if dx not in free]
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
                self._log(f"basket stop zone: 가운데 막힘 — 좌우 {dx * 100:+.0f} cm 에 선다 "
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
