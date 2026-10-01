"""미션 FSM -> 팀원 시연 UI(ui/grippers-ui.html)가 받는 state(dict).

UI 의 app.js 는 "파이썬이 만든 state 를 DOM 에 반영만 한다" — 한국어 문구·상태 판정은 여기 있다.
모양은 UI 목업(mock.js build())과 같다: screen · run · status · map · pieces · robot · path ·
tray · notice · card ... 를 채운다.

입력창의 문장은 CommandDesk(mission/commands.py)가 Claude 로 해석해 FSM 지시(Order)로 넘긴다.
- instruction.mode auto: 보이는 기물을 모두 정리하고, 지시가 오면 그것부터
- instruction.mode instructed: 지시가 있을 때만 움직인다(없으면 대기 화면)
마이크(음성)는 아직 연결 전 — 알림만 띄운다.
"""
from __future__ import annotations

import math
import time
from typing import Optional

from mission.host_fsm import HostState
from perception.hands import nearest_spot

PIECE_KO = {"rook": "룩", "soccer": "공", "queen": "퀸", "knight": "나이트", "star": "별", "box": "상자"}
LEGEND_ORDER = ["box", "soccer", "star", "queen", "knight", "rook"]
DEST_KO = "바구니"
AUTO_QUOTE = "“보이는 기물을 모두 바구니에 정리합니다”"
HINTS = ["예: 퀸을 바구니에 넣어줘", "체스 말만 전부 정리해줘", "공 하나 치워줘", "자유롭게 움직이는 말 정리해"]

# 상태 -> (한국어, 영어, 단계, 톤). 문구는 목업 mock.js 의 T 표를 따른다.
PHASE = {
    HostState.SEARCH_TARGET: ("대상 탐색 중", "SCANNING", "기물 탐색", "accent"),
    HostState.APPROACH_PIECE: ("타깃으로 접근 중", "APPROACH_PIECE", "1 / 4 · 접근", "active"),
    HostState.GRASP: ("집는 중", "GRASP", "2 / 4 · 집기", "active"),
    HostState.CARRY_TO_DEST: (f"{DEST_KO}로 운반 중", "TRANSPORT", "3 / 4 · 운반", "active"),
    HostState.NUDGE_BOX: ("상자 앞 진입 중", "NUDGE_BOX", "3 / 4 · 운반", "active"),
    HostState.PLACE: ("내려놓는 중", "RELEASE", "4 / 4 · 놓기", "active"),
    HostState.HALTED: ("멈춤 · 확인 필요", "HALTED", "사람 개입", "error"),
}
HELD_STATES = (HostState.CARRY_TO_DEST, HostState.NUDGE_BOX, HostState.PLACE)
NOTICE_S = 3.5


def josa(word: str, kind: str) -> str:
    """받침에 맞는 조사 — 목업 josa() 와 같은 규칙."""
    if not word:
        return word
    c = ord(word[-1])
    jong = (c - 0xAC00) % 28 if 0xAC00 <= c <= 0xD7A3 else 0
    if kind == "eun":
        return word + ("는" if jong == 0 else "은")
    if kind == "i":
        return word + ("가" if jong == 0 else "이")
    return word + ("를" if jong == 0 else "을")


def search_reason_ko(reason: Optional[str]) -> Optional[str]:
    """FSM 탐색 사유(영어, 콘솔·로그용) -> 화면 문구."""
    if not reason:
        return None
    if reason.startswith("no pieces"):
        return "작업 구역에 기물이 없습니다"
    if "skipped" in reason:
        n = reason.split(" ", 1)[0]
        sec = reason.split("retry in ", 1)[-1].rstrip("s)")
        return f"집지 못한 기물 {n}개는 {sec}초 뒤 다시 시도합니다"
    if reason.startswith("no destination"):
        return "보이는 기물을 넣을 상자가 정해져 있지 않습니다"
    if reason.startswith("pieces outside"):
        return "기물이 작업 구역 밖에 있습니다"
    return reason


def _dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class UiState:
    """매 사이클 build() 로 state 를 만든다. 완료 개수·알림처럼 사이클을 넘는 것만 여기 남는다."""

    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self.done = 0
        self._seen_place_jobs: set[int] = set()
        self._started = time.monotonic()
        self._notice: Optional[dict] = None
        self._notice_until = 0.0
        self._skipped_n = 0
        self._leg_state: Optional[HostState] = None
        self._leg_start = 0.0

    def reset(self) -> None:
        self.__init__(self.cfg)

    def notify(self, text: str, code: str, tone: str = "accent", seconds: float = NOTICE_S) -> None:
        self._notice = {"text": text, "code": code, "tone": tone}
        self._notice_until = time.monotonic() + seconds

    # ------------------------------------------------------------------
    def _workspace_pieces(self, pmap) -> list[dict]:
        a = self.cfg.arena
        out = []
        for label, pts in sorted(pmap.items()):
            for i, (x, y) in enumerate(pts):
                inside = a.workspace_x[0] <= x <= a.workspace_x[1] and a.workspace_y[0] <= y <= a.workspace_y[1]
                out.append({"id": f"{label}#{i}", "label": label, "x": round(x, 3), "y": round(y, 3),
                            "state": "idle" if inside else "done"})
        return out

    def _progress(self, fsm, pose, target_xy) -> tuple[str, float]:
        """(metric, progress 0~1). 거리 진행률은 그 단계에 들어갈 때의 거리를 기준으로 본다."""
        st = fsm.state
        if st != self._leg_state:
            self._leg_state = st
            goal = target_xy if st == HostState.APPROACH_PIECE else fsm.dest_xy
            self._leg_start = _dist(pose.xy, goal) if (pose.ok and goal) else 0.0
        frac = 0.0
        goal = target_xy if st == HostState.APPROACH_PIECE else fsm.dest_xy
        remain = _dist(pose.xy, goal) if (pose.ok and goal) else None
        if remain is not None and self._leg_start > 1e-3:
            frac = max(0.0, min(1.0, 1.0 - remain / self._leg_start))
        label = fsm.target_label or ""
        if st == HostState.APPROACH_PIECE:
            return (f"남은 거리 {remain:.2f} m" if remain is not None else ""), frac / 4
        if st == HostState.GRASP:
            return "그리퍼로 집는 중", 0.375
        if st == HostState.CARRY_TO_DEST:
            return f"{label} 적재됨", (2 + frac) / 4
        if st == HostState.NUDGE_BOX:
            return f"{label} 적재됨", 0.75
        if st == HostState.PLACE:
            d = fsm.dest_xy
            return (f"{DEST_KO} · {d[0]:.2f}, {d[1]:.2f} m" if d else DEST_KO), 0.875
        return "", 0.0

    # ------------------------------------------------------------------
    def _idle(self, now: float, waiting: bool) -> dict:
        k = int(now / 2.4) % len(HINTS)
        if waiting:
            hints = [HINTS[(k + i) % len(HINTS)] for i in range(3)]
        else:
            hints = ["기물을 작업 구역에 놓으면 바로 정리를 시작합니다",
                     HINTS[k], "Esc 비상 정지 · d 디버그 · l 범례"]
        return {"label": "READY", "placeholder": "무엇을 시킬까요?", "hints": hints,
                "dots": len(HINTS), "dot": k}

    @staticmethod
    def _command(desk) -> dict:
        if desk is None or desk.phase != "interpreting":
            return {}
        return {"text": f"“{desk.text}”", "interp_text": "해석 중 · 대상 탐색…", "interp_code": "Interpreting",
                "echo": desk.text, "action": "mic", "partial": desk.text, "hint": ""}

    def _done(self, finished, elapsed: int, waiting: bool) -> dict:
        sub = f"소요 {elapsed}초 · " + ("다음 명령을 말해 주세요" if waiting else "다음 기물을 놓으면 이어서 정리합니다")
        if finished:
            order, outcome = finished
            names = "·".join(PIECE_KO.get(lb, lb) for lb in order.labels)
            if outcome == "absent":
                return {"title": f"{josa(names, 'i')} 보이지 않아 끝냈습니다", "sub": sub}
            if order.done == 1:
                return {"title": f"{josa(names, 'eul')} {DEST_KO}에 넣었습니다", "sub": sub}
            return {"title": f"{names} {order.done}개를 {DEST_KO}에 넣었습니다", "sub": sub}
        return {"title": f"기물 {self.done}개를 모두 옮겼습니다", "sub": sub}

    def build(self, pose, pmap, fsm, pi_status, link_age_s: float, hz: float, hands=(), desk=None) -> dict:
        cfg = self.cfg
        now = time.monotonic()
        st = fsm.state
        if desk is not None and desk.note is not None:
            self.notify(*desk.note)
            desk.note = None

        # 완료 개수 — 성공한 PLACE 작업 번호를 한 번씩만 센다.
        r = fsm.last_result
        if r is not None and r.action == "PLACE" and r.ok and r.job_id not in self._seen_place_jobs:
            self._seen_place_jobs.add(r.job_id)
            self.done += 1
        if len(fsm.skipped) > self._skipped_n:
            self.notify("이 기물은 건너뛰고 다음 기물로 갑니다", "E-314", "caution")
        self._skipped_n = len(fsm.skipped)

        pieces = self._workspace_pieces(pmap)
        live = [p for p in pieces if p["state"] == "idle"]
        target_id = None
        if fsm.target_xy is not None and st not in HELD_STATES:
            near = [p for p in pieces if p["label"] == fsm.target_label]
            if near:
                best = min(near, key=lambda p: _dist((p["x"], p["y"]), fsm.target_xy))
                if _dist((best["x"], best["y"]), fsm.target_xy) < 0.10:
                    target_id = best["id"]
        held = {"id": "held", "label": fsm.target_label} if (st in HELD_STATES and fsm.target_label) else None

        # 화면: 해석 중 -> 접수 · 움직이는 중/지시 있음 -> 실행 · 할 일 없음 -> 완료(옮긴 뒤) 또는 대기
        searching = st == HostState.SEARCH_TARGET
        order = getattr(fsm, "order", None)
        finished = getattr(fsm, "finished_order", None)
        waiting = searching and order is None and getattr(fsm, "command_mode", "auto") == "instructed"
        if desk is not None and desk.phase == "interpreting":
            screen = "command"
        elif fsm.estop or not searching or order is not None:
            screen = "run"
        elif waiting or not live:
            screen = "done" if (finished or self.done) else "idle"
        else:
            screen = "run"
        ko, en, step, tone = PHASE.get(st, ("", st.name, "", "accent"))
        if fsm.estop:
            ko, en, step, tone = "비상 정지", "E_STOP", "정지됨", "error"
        metric, progress = self._progress(fsm, pose, fsm.target_xy)

        target_ko = PIECE_KO.get(fsm.target_label or "", fsm.target_label or "기물")
        if target_id or held:
            tgt = {"label": fsm.target_label or "", "title": f"대상 · {fsm.target_label}",
                   "reason": (f"명령에 지정된 기물 · {target_ko}" if order else f"가장 가까운 기물 · {target_ko}"),
                   "distance": (f"{_dist(pose.xy, fsm.target_xy):.2f} m"
                                if pose.ok and fsm.target_xy else "—")}
        else:
            tgt = {"label": "", "title": "대상 탐색 중",
                   "reason": search_reason_ko(fsm.search_reason) or "작업 영역을 훑는 중입니다", "distance": "—"}

        # 알림 배너: 일회성(notify) > 차체 복구 > Pi 연결 > 탐색 사유
        notice = self._notice if (self._notice and now < self._notice_until) else None
        if notice is None:
            if pi_status is not None and pi_status.base_recovering:
                notice = {"text": "차체 컨트롤러를 다시 띄우는 중입니다", "code": "BASE", "tone": "caution"}
            elif pi_status is None or link_age_s > 2.0:
                notice = {"text": "Pi 상태가 오지 않습니다 — 명령은 콘솔/UDP 로만 나갑니다",
                          "code": "LINK", "tone": "caution"}
            elif searching and fsm.search_reason and live and not waiting:
                notice = {"text": search_reason_ko(fsm.search_reason), "code": "SEARCH", "tone": "accent"}

        card = None
        if fsm.estop:
            card = {"code": "E-000 EMERGENCY_STOP", "next": "→ 초기화 후 재개", "tone": "error", "icon": "!",
                    "title": "비상 정지되었습니다",
                    "detail": (f"{josa(target_ko, 'eun')} 그립에 유지되어 있습니다. 주변을 확인한 뒤 초기화하세요."
                               if held else "모든 구동이 멈췄습니다. 주변을 확인한 뒤 초기화하세요."),
                    "rows": [], "actions": [{"id": "reset", "label": "초기화 후 재개", "primary": True}]}
        elif st == HostState.HALTED:
            card = {"code": "HALTED", "next": "→ 사람 개입", "tone": "error", "icon": "!",
                    "title": "멈췄습니다 — 확인이 필요합니다",
                    "detail": fsm.halt_reason or "", "rows": [],
                    "actions": [{"id": "reset", "label": "초기화", "primary": True}]}
        elif desk is not None:
            card = desk.card()

        grip_state = "closed" if held else ("closing" if st == HostState.GRASP else "open")
        cnt: dict[str, int] = {}
        for p in live:
            cnt[p["label"]] = cnt.get(p["label"], 0) + 1
        running = st not in (HostState.SEARCH_TARGET, HostState.HALTED) and not fsm.estop
        a = cfg.arena
        bw, bl, _ = a.box_size
        boxes = [{"name": name, "x0": bx - bw / 2, "x1": bx + bw / 2, "y0": by - bl / 2, "y1": by + bl / 2,
                  "active": held is not None or (target_id is not None and fsm.dest_box == name)}
                 for name, (bx, by, _yaw) in a.boxes.items()]
        pl = cfg.planner
        robot_r = pl.robot_width_m / 2.0
        elapsed = max(1, round(now - self._started))
        if not pose.ok:
            led = "lost"
        elif st == HostState.HALTED or fsm.estop:
            led = "halt"
        else:
            led = "busy" if running else "ready"

        return {
            "screen": screen, "mode": en, "tone": tone, "recording": False, "level": 0.0,
            "idle": self._idle(now, waiting),
            "command": self._command(desk),
            "run": {"quote": f"“{order.text}”" if order else AUTO_QUOTE,
                    "mode": "target" if searching else "status"},
            "target": tgt,
            "status": {"ko": ko, "en": en, "step": step, "metric": metric, "progress": progress,
                       "grip": st in (HostState.GRASP, HostState.PLACE)},
            "done": self._done(finished, elapsed, waiting),
            "detail": {"x": f"{pose.x:.3f}" if pose.ok else None, "y": f"{pose.y:.3f}" if pose.ok else None,
                       "yaw": f"{pose.yaw_deg:.1f}" if pose.ok else None,
                       "cmd": (fsm.last_cmd_text or "—")[:14], "target": fsm.target_label,
                       "grip": grip_state, "veh": None, "arm": None},
            "legend": [{"label": lb, "ko": PIECE_KO[lb], "n": cnt.get(lb, 0)} for lb in LEGEND_ORDER],
            "map": {"boxes": boxes,
                    "markers": [[x, y, mid] for mid, (x, y) in sorted(cfg.aruco.floor_markers.items())],
                    "robot_r_m": robot_r,
                    "safe_r_m": robot_r + pl.piece_obstacle_radius_m + pl.obstacle_margin_m},
            "pieces": pieces,
            "target_id": target_id,
            "held": held,
            "robot": {"x": pose.x, "y": pose.y, "yaw": pose.yaw_deg, "ok": bool(pose.ok),
                      "fresh": bool(getattr(pose, "fresh", True))} if pose.ok else {"ok": False},
            "path": {"pts": [list(p) for p in fsm.nav_path], "blocked": bool(fsm.blocked_by), "t": 0}
                    if fsm.nav_path and len(fsm.nav_path) >= 2 else {"pts": []},
            "grip": 1.0 if st == HostState.GRASP else 0.4,
            "show_grip": st in (HostState.GRASP, HostState.PLACE) and target_id is not None,
            "scanning": searching and bool(live) and pose.ok,
            "obstacle": None,
            "estop": bool(fsm.estop), "pickable": False,
            "hands": [{"x": round(x, 3), "y": round(y, 3), "spot": nearest_spot((x, y))} for x, y in hands],
            "notice": notice, "card": card,
            "tray": {"manual": bool(fsm.manual_mode),
                     "auto": (not fsm.manual_mode) and running,
                     "ready": bool(fsm.manual_mode and fsm.ready_to_advance),
                     "led": led, "estop_armed": bool(fsm.estop)},
            "hz": round(hz, 1),
        }
