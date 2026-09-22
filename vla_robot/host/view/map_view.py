"""OpenCV 로 그리는 탑뷰 지도 + 조작 키.

⚠️ GUI 툴킷은 OpenCV HighGUI 하나만 쓴다. 기존 코드는 matplotlib(Tk) 지도와 cv2
카메라 창을 한 프로세스에 띄웠다가 GIL 크래시(PyEval_RestoreThread)로 미션이 통째로
죽었다. 카메라 디버그 창도 같은 HighGUI 로 띄우므로 섞일 일이 없다.

⚠️ cv2.putText 의 Hershey 폰트는 한글을 못 그린다 — 화면 문구는 영어로 둔다.

키: q 종료 · space ESTOP 래치(해제는 r 만) · r reset · n next(수동) · p prev · m 수동/자동
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from mission.basket_target import basket_target

WINDOW = "vla_robot host"
KEYMAP = {ord("q"): "quit", 27: "quit", ord(" "): "estop", ord("r"): "reset",
          ord("n"): "next", ord("p"): "prev", ord("m"): "toggle_manual"}


class MapView:
    def __init__(self, cfg) -> None:
        self.cfg = cfg
        a = cfg.arena
        self.s = cfg.view.px_per_m
        self.margin = 20
        self.map_w = int((a.wall_x[1] - a.wall_x[0]) * self.s) + 2 * self.margin
        self.map_h = int((a.wall_y[1] - a.wall_y[0]) * self.s) + 2 * self.margin
        self.w = self.map_w + cfg.view.panel_width_px
        self.h = max(self.map_h, 520)
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        self._closed = False

    def _px(self, x: float, y: float) -> tuple[int, int]:
        a = self.cfg.arena
        # +y 가 화면 위쪽이 되게 뒤집는다(상자 띠가 위).
        return (int(self.margin + (x - a.wall_x[0]) * self.s),
                int(self.margin + (a.wall_y[1] - y) * self.s))

    def update(self, pose, piece_map, fsm, pi_status, link_age_s: float, hz: float) -> str | None:
        cfg = self.cfg
        a = cfg.arena
        img = np.full((self.h, self.w, 3), 30, np.uint8)
        cv2.rectangle(img, self._px(a.wall_x[0], a.wall_y[1]), self._px(a.wall_x[1], a.wall_y[0]),
                      (90, 90, 90), 2)
        cv2.rectangle(img, self._px(a.workspace_x[0], a.workspace_y[1]),
                      self._px(a.workspace_x[1], a.workspace_y[0]), (60, 80, 60), 1)
        bw, bl, _ = a.box_size
        m = cfg.mission
        for name, (bx, by, _yaw) in a.boxes.items():
            cv2.rectangle(img, self._px(bx - bw / 2, by + bl / 2), self._px(bx + bw / 2, by - bl / 2),
                          (40, 120, 200), 2)
            cv2.putText(img, name, self._px(bx - bw / 2, by), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (40, 160, 230), 1)
            # 투입 목표 사각형 — 팔이 겨누는 점이다(상자 중심이 아니다).
            target = basket_target(name, (bx, by), a.box_size,
                                   m.insert_half_width_m, m.insert_inset_depth_m)
            x0, x1, y0, y1 = target.rect
            cv2.rectangle(img, self._px(x0, y1), self._px(x1, y0), (60, 200, 230), 1)
            # 정차 판정 반경 — 이 안에 들면 나머지 각도는 팔의 base 가 맡는다.
            stop_xy = (bx, by - (a.box_size[1] / 2.0 + m.box_approach_margin_m))
            cv2.circle(img, self._px(*stop_xy), int(m.place_arrive_tol_m * self.s),
                       (60, 200, 230), 1)

        for xy, _t in fsm.skipped:
            cv2.circle(img, self._px(*xy), int(cfg.mission.skip_radius_m * self.s), (80, 80, 160), 1)
        for label, pts in piece_map.items():
            for x, y in pts:
                target = fsm.target_xy is not None and math.hypot(x - fsm.target_xy[0], y - fsm.target_xy[1]) < 0.05
                color = (0, 220, 255) if target else (200, 200, 200)
                cv2.circle(img, self._px(x, y), 7, color, -1)
                cv2.putText(img, label, (self._px(x, y)[0] + 9, self._px(x, y)[1] - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        if fsm.nav_path:
            pts = np.array([self._px(*p) for p in fsm.nav_path], np.int32)
            cv2.polylines(img, [pts], False, (0, 200, 0), 2)
        if fsm.nav_goal:
            cv2.drawMarker(img, self._px(*fsm.nav_goal), (0, 255, 0), cv2.MARKER_CROSS, 16, 2)
        if fsm.dest_xy:
            cv2.drawMarker(img, self._px(*fsm.dest_xy), (40, 160, 230), cv2.MARKER_TILTED_CROSS, 14, 2)

        if pose.ok:
            c = self._px(pose.x, pose.y)
            color = (255, 160, 0) if pose.fresh else (120, 120, 255)
            cv2.circle(img, c, int(cfg.planner.robot_radius_piece_m * self.s), color, 2)
            th = math.radians(pose.yaw_deg)
            tip = self._px(pose.x + 0.2 * math.cos(th), pose.y + 0.2 * math.sin(th))
            cv2.arrowedLine(img, c, tip, color, 2, tipLength=0.3)

        lines = [
            f"state: {fsm.state.name}{' [MANUAL]' if fsm.manual_mode else ''}",
            f"wire : {fsm.wire_state}",
            f"cmd  : {fsm.last_cmd_text}",
            f"ready: {fsm.ready_to_advance}",
            f"target: {fsm.target_label} -> {fsm.dest_box}",
            f"pose : {'LOST' if not pose.ok else f'{pose.x:.3f},{pose.y:.3f} {pose.yaw_deg:.1f}deg cams={pose.n_cams}'}",
            f"loop : {hz:.1f} Hz   link age: {link_age_s:.2f}s",
        ]
        if pi_status is not None:
            r = pi_status.result
            lines += [
                f"pi   : {pi_status.state} busy={pi_status.busy} job={pi_status.job_id}",
                f"pi   : base_ok={pi_status.base_ok} watchdog={pi_status.watchdog}",
                f"last : {'-' if r is None else f'#{r.job_id} {r.action} ok={r.ok}'}",
            ]
        else:
            lines.append("pi   : no status")
        if fsm.search_reason:
            lines.append(f"search: {fsm.search_reason}")
        if fsm.halt_reason:
            lines.append("HALT: " + fsm.halt_reason[:40])
        if fsm.estop:
            lines.append("*** ESTOP LATCHED (r = reset) ***")
        lines.append("")
        lines += [e[-48:] for e in list(fsm.events)[-6:]]
        lines += ["", "q quit  space ESTOP  r reset", "n next  p prev  m manual"]
        x0 = self.map_w + 10
        for i, text in enumerate(lines):
            color = (0, 0, 255) if text.startswith("***") else (230, 230, 230)
            cv2.putText(img, text, (x0, 24 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        cv2.imshow(WINDOW, img)
        key = cv2.waitKey(1) & 0xFF
        try:
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                return "quit"
        except cv2.error:
            return "quit"
        return KEYMAP.get(key)

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            cv2.destroyAllWindows()
