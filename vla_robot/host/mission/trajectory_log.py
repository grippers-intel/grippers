"""주행 궤적 기록 — 사이클마다 한 줄 CSV (2026-10-07).

상자 바깥으로 우회하다 모서리에 아주 붙어 지나갔는데, 로그는 10 s 마다 위치만 있어서 어느 동작
(경로 직진 · 앞으로 빠져나가기 · 틈 안 직진 유지 …)에서 파고들었는지 알 수 없었다. 그래서 사이클마다
상태 · 명령 · 위치와 **가장 가까운 기물까지 차체 간격**을 남긴다. 동작은 바꾸지 않는다.

차체 간격 = 차체 사각형(마커 중심 기준 길이 robot_length_m · 폭 robot_width_m)에서 기물 중심까지의
거리 - 기물 반경(piece_obstacle_radius_m). 0 이하면 닿았다고 본다(상자 기물 모서리는 0.8 cm 더 나온다).
쥔 기물·잡으러 가는 기물은 빼고 따로 적는다(target_m).

veh_v · arm_v = Pi 가 보내는 차체·팔 배터리 전압(V, 1 Hz 갱신 · 모르면 빈칸, 10-08 추가). 대기·직진·회전별 전압 하락,
전압과 무응답·회전 뒤 흐름·파지 빈손의 관계를 나중에 보려고 남긴다.
"""
from __future__ import annotations

import csv
import math
import time
from pathlib import Path
from typing import Optional

COLUMNS = ["t", "state", "cmd", "linear_x", "angular_z", "stop", "pose_ok", "cams",
           "x", "y", "yaw", "target", "target_m", "nearest", "nearest_center_m", "nearest_gap_m",
           "sub_goal_x", "sub_goal_y", "veh_v", "arm_v"]


def body_gap(x: float, y: float, yaw_deg: float, o: tuple[float, float], length: float, width: float,
             piece_r: float) -> float:
    """차체 사각형 바깥면에서 기물 가장자리까지(m). 음수면 겹친다."""
    th = math.radians(yaw_deg)
    dx, dy = o[0] - x, o[1] - y
    lx, ly = dx * math.cos(th) + dy * math.sin(th), -dx * math.sin(th) + dy * math.cos(th)
    ex, ey = max(abs(lx) - length / 2.0, 0.0), max(abs(ly) - width / 2.0, 0.0)
    return math.hypot(ex, ey) - piece_r


def _volts(v) -> str:
    """전압 칸. 0 이하(Pi 가 모른다고 보냄)·없음은 빈칸."""
    try:
        return f"{float(v):.2f}" if v and float(v) > 0 else ""
    except (TypeError, ValueError):
        return ""


class TrajectoryLog:
    def __init__(self, path: Path, planner_cfg) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._f = open(path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(COLUMNS)
        self._c = planner_cfg
        self._last_flush = time.monotonic()

    @classmethod
    def new(cls, logs_dir: Path, planner_cfg) -> "TrajectoryLog":
        return cls(logs_dir / time.strftime("traj_%Y%m%d_%H%M%S.csv"), planner_cfg)

    def row(self, t: float, fsm, pose, cmd, pmap, status=None) -> None:
        c = self._c
        ok = bool(pose.ok)
        others: list[tuple[str, tuple[float, float]]] = []
        try:
            keep = set(fsm._other_pieces(pose)) if ok else set()      # 목표·쥔 기물 뺀 것
        except Exception:  # noqa: BLE001 — 기록이 주행을 멈추면 안 된다
            keep = set()
        for lb, pts in (pmap or {}).items():
            for p in pts:
                if p in keep:
                    others.append((lb, p))
        near_lb, near_c, near_gap = "", "", ""
        if ok and others:
            lb, p = min(others, key=lambda q: body_gap(pose.x, pose.y, pose.yaw_deg, q[1], c.robot_length_m,
                                                       c.robot_width_m, c.piece_obstacle_radius_m))
            near_lb = lb
            near_c = f"{math.hypot(p[0] - pose.x, p[1] - pose.y):.3f}"
            near_gap = f"{body_gap(pose.x, pose.y, pose.yaw_deg, p, c.robot_length_m, c.robot_width_m, c.piece_obstacle_radius_m):.3f}"
        tgt = getattr(fsm, "target_xy", None)
        tgt_m = f"{math.hypot(tgt[0] - pose.x, tgt[1] - pose.y):.3f}" if (ok and tgt) else ""
        path = getattr(fsm, "nav_path", None) or []
        sg: Optional[tuple[float, float]] = path[1] if len(path) > 1 else None
        self._w.writerow([
            f"{t:.2f}", fsm.state.name, getattr(fsm, "last_cmd_text", ""),
            f"{cmd.linear_x:.3f}", f"{cmd.angular_z:.3f}", int(bool(cmd.stop)), int(ok),
            getattr(pose, "n_cams", ""),
            f"{pose.x:.3f}" if ok else "", f"{pose.y:.3f}" if ok else "", f"{pose.yaw_deg:.1f}" if ok else "",
            getattr(fsm, "target_label", "") or "", tgt_m, near_lb, near_c, near_gap,
            f"{sg[0]:.3f}" if sg else "", f"{sg[1]:.3f}" if sg else "",
            _volts(getattr(status, "battery_v", 0.0)), _volts(getattr(status, "arm_v", 0.0)),
        ])
        now = time.monotonic()
        if now - self._last_flush >= 1.0:            # 강제 종료돼도 1 s 이상은 잃지 않게
            self._f.flush()
            self._last_flush = now

    def close(self) -> None:
        try:
            self._f.close()
        except Exception:  # noqa: BLE001
            pass
