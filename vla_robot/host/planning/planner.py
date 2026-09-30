"""경로 계획(격자 탐색 + 직선화)과 직진/정지/회전 시퀀서.

## 왜 매 사이클 전체 경로를 다시 짜는가
기물 지도가 바뀌면(늦은 검출, 사람이 건드림) 낡은 경로를 밀고 가게 된다. 매 사이클
다시 짜고 첫 구간만 실행한다.

## 왜 장애물 입력에 이력을 거는가 (ObstacleHold)
2026-08-29 실기: 로봇이 목표 앞에서 제자리 왕복만 했다. 옆 기물 검출이 흔들리자
장애물 집합이 사이클마다 바뀌었고, 두 경로의 방위 차가 50~60° 라 영원히 수렴하지
않았다. 사라진 장애물을 잠시 붙들어 두는 쪽이 안전하기도 하다.

## 왜 회전 문턱이 둘인가 (DriveSequencer)
하나로 쓰면 목표 방위 주변에서 정지<->회전을 반복하며 떤다(2026-09-05). 멈추라고
해도 지연 0.3 s 동안 ~6° 더 돌므로, 나올 때 5°, 들어갈 때 12° 로 벌린다.

## 왜 기물을 차체 모양으로 피하는가 (safe / turn_safe)
차는 직진과 제자리 회전만 한다. 직진할 때 기물을 스칠 수 있는 건 차체 옆면(반폭)이고,
제자리에서 돌 때는 대각선 반지름의 원 전체다. 예전에는 로봇을 한 원으로 보고 기물마다
큰 쪽(회전)을 칠해 두었는데, 2026-09-30 실기에서 51 cm 떨어진 기물 사이가 막혀 HALTED 가
났다. 지금은 직진 여유로 길을 찾고, **꺾이는 점**만 회전 여유를 확인해 모자라면 그 기물만
넓혀 다시 찾는다.

## 왜 상자 둘레를 막는가 (box_keepout)
2026-09-29 배치도 REV.2 에서 주행 구역을 장판 거의 전체로 넓혔다. 그전에는 주행 구역
상한(y 1.30)이 곧 상자 앞이라 계획기가 상자로 들어갈 수 없었는데, 넓히면서 그 역할이
사라졌다. 그래서 상자 둘레 사각형을 **고정 장애물**로 둔다 — 앞쪽 경계가 정차점이다.

## 이전 구현에서 고친 것
- 막혀서 부분목표가 로봇 자신이면, 방위 계산이 "이미 정렬"로 나와 FORWARD 가
  나갔다. 여기서는 부분목표까지 거리가 허용치 이하면 STOP 을 낸다.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import numpy as np

XY = tuple[float, float]


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def segment_circle_clearance(p0: XY, p1: XY, c: XY) -> tuple[float, float]:
    """선분 p0->p1 과 점 c 의 최근접 거리와 그 지점의 진행률 t(0~1)."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-12:
        return math.hypot(p0[0] - c[0], p0[1] - c[1]), 0.0
    t = ((c[0] - p0[0]) * dx + (c[1] - p0[1]) * dy) / length_sq
    t = min(1.0, max(0.0, t))
    return math.hypot(p0[0] + t * dx - c[0], p0[1] + t * dy - c[1]), t


Rect = tuple[float, float, float, float]      # x0, x1, y0, y1


def box_keepout_rects(arena_cfg, side_m: float, front_m: float) -> list[Rect]:
    """상자마다 진입 금지 사각형. 입구 쪽(-y, yaw 180)은 front_m, 나머지는 side_m 만큼 넓힌다.

    상자는 모두 입구가 -y 를 향한다는 전제다(basket_target 과 같은 전제).
    """
    bw, bl = arena_cfg.box_size[0], arena_cfg.box_size[1]
    return [(bx - bw / 2.0 - side_m, bx + bw / 2.0 + side_m,
             by - bl / 2.0 - front_m, by + bl / 2.0 + side_m)
            for bx, by, _yaw in arena_cfg.boxes.values()]


def point_in_rect(p: XY, r: Rect) -> bool:
    """경계 위는 밖으로 본다 — 정차점이 앞쪽 경계 위에 있다."""
    return r[0] < p[0] < r[1] and r[2] < p[1] < r[3]


def segment_hits_rect(p0: XY, p1: XY, r: Rect) -> bool:
    """선분이 사각형 **내부**를 지나는가(Liang-Barsky). 경계를 스치는 것은 통과로 본다."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    t0, t1 = 0.0, 1.0
    for d, lo, hi, q in ((dx, r[0], r[1], p0[0]), (dy, r[2], r[3], p0[1])):
        if abs(d) < 1e-12:
            if not lo < q < hi:
                return False
            continue
        ta, tb = (lo - q) / d, (hi - q) / d
        if ta > tb:
            ta, tb = tb, ta
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 >= t1:
            return False
    return True


# ---------------------------------------------------------------------------
class ObstacleHold:
    def __init__(self, hold_cycles: int, match_m: float) -> None:
        self.hold_cycles = hold_cycles
        self.match_m = match_m
        self._held: list[list] = []     # [(x, y), 남은 사이클]

    def reset(self) -> None:
        self._held.clear()

    def update(self, seen: list[XY]) -> list[XY]:
        for entry in self._held:
            entry[1] -= 1
        for sx, sy in seen:
            for entry in self._held:
                hx, hy = entry[0]
                if math.hypot(hx - sx, hy - sy) <= self.match_m:
                    entry[0] = (sx, sy)
                    entry[1] = self.hold_cycles
                    break
            else:
                self._held.append([(sx, sy), self.hold_cycles])
        self._held = [e for e in self._held if e[1] > 0]
        return [e[0] for e in self._held]


# ---------------------------------------------------------------------------
class DriveMode(Enum):
    FORWARD = auto()
    STOP = auto()
    ROTATE = auto()


@dataclass
class DriveCommand:
    mode: DriveMode
    waypoint: XY
    target_yaw_deg: float
    yaw_error_deg: float
    dist_to_waypoint: float


class DriveSequencer:
    """차량을 항상 정면으로만 달리게 한다(메카넘이지만 제어 단순화를 위해 직진 전용).

    FORWARD -(오차 > enter)-> STOP 1사이클 -> ROTATE -(오차 <= tol)-> STOP 1사이클 -> FORWARD
    """

    def __init__(self, cfg) -> None:
        self.tol = cfg.yaw_tolerance_deg
        self.enter = max(cfg.yaw_enter_deg, cfg.yaw_tolerance_deg)
        self.min_heading = cfg.min_heading_dist_m
        self.step = cfg.waypoint_step_m
        self.arrive = cfg.axis_leg_tolerance_m
        self._mode: Optional[DriveMode] = None
        self._after_stop = DriveMode.FORWARD

    def reset(self) -> None:
        self._mode = None
        self._after_stop = DriveMode.FORWARD

    def update(self, robot_xy: XY, robot_yaw_deg: float, target_xy: XY) -> DriveCommand:
        dx, dy = target_xy[0] - robot_xy[0], target_xy[1] - robot_xy[1]
        dist = math.hypot(dx, dy)
        if dist <= self.step:
            waypoint = target_xy
        else:
            waypoint = (robot_xy[0] + dx / dist * self.step, robot_xy[1] + dy / dist * self.step)

        if dist < self.min_heading:
            # 잔여 거리가 위치 노이즈 수준이면 atan2 는 방향이 아니라 노이즈의 방향이다
            # (2026-08-30: 1.8 cm 앞에서 ±3 mm 흔들자 80 사이클 중 6번 회전 방향이 뒤집힘).
            target_yaw = robot_yaw_deg
        else:
            target_yaw = math.degrees(math.atan2(dy, dx))
        err = wrap_deg(target_yaw - robot_yaw_deg)

        if dist <= self.arrive:
            self._mode = DriveMode.STOP
            self._after_stop = DriveMode.FORWARD
            return DriveCommand(DriveMode.STOP, waypoint, target_yaw, err, dist)

        aligned = abs(err) <= self.tol
        drifted = abs(err) > self.enter
        if self._mode is None:
            self._mode = DriveMode.FORWARD if aligned else DriveMode.ROTATE
        out = self._mode
        if self._mode == DriveMode.FORWARD and drifted:
            self._mode, self._after_stop = DriveMode.STOP, DriveMode.ROTATE
        elif self._mode == DriveMode.ROTATE and aligned:
            self._mode, self._after_stop = DriveMode.STOP, DriveMode.FORWARD
        elif self._mode == DriveMode.STOP:
            self._mode = self._after_stop
        return DriveCommand(out, waypoint, target_yaw, err, dist)


# ---------------------------------------------------------------------------
_DIRS = ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1))
_STEP = tuple(141 if dx and dy else 100 for dx, dy in _DIRS)


class GridPathPlanner:
    """주행영역 격자에서 최단 경로를 찾고 시선 검사로 편다.

    update() -> (부분목표, 그다음 꺾이는 점, blocked_by)
      blocked_by: None(직진) | "piece"(회피 중) | "blocked"(목표까지 길 없음)
    """

    def __init__(self, planner_cfg, arena_cfg, arrive_tol: float) -> None:
        c = planner_cfg
        self.cfg = c
        self.cell = c.cell_m
        self.arrive_tol = arrive_tol
        # 도착 칸은 트리거 거리보다 두 칸 안쪽에서 고른다. 칸 중심에 서도(±허용치)
        # 실제 로봇 위치가 FSM 트리거 거리 안에 들어오게 하려는 것이다.
        self.goal_radius = max(arrive_tol - 2.0 * c.cell_m, c.cell_m)
        # 기물 회피는 **차체 모양**으로 본다(2026-09-30). 차는 직진과 제자리 회전만 하므로
        #   직진 중: 차체가 옆으로 차지하는 폭(반폭)만 기물을 스친다     -> safe
        #   꺾을 때: 제자리 회전이 대각선 반지름의 원을 쓸고 지나간다     -> turn_safe
        # 둘을 하나(큰 쪽)로 뭉치면 직진으로 충분히 지나갈 틈까지 막힌다 — 실기에서 그랬다.
        pad = c.piece_obstacle_radius_m + c.obstacle_margin_m
        self.safe = c.robot_width_m / 2.0 + pad
        self.turn_safe = math.hypot(c.robot_width_m / 2.0, c.robot_length_m / 2.0) + pad
        # 마커 중심이 설 수 있는 범위다. 좌우는 경계에서 암 휩쓸림 반경만큼 물러난다
        # (이걸 빼먹으면 계획기가 벽을 파고드는 경로를 낸다 — 실제로 겪었다).
        self.x0 = arena_cfg.wall_x[0] + c.robot_radius_wall_m
        self.x1 = arena_cfg.wall_x[1] - c.robot_radius_wall_m
        self.y0, self.y1 = c.drive_area_y
        self.nx = int(round((self.x1 - self.x0) / self.cell)) + 1
        self.ny = int(round((self.y1 - self.y0) / self.cell)) + 1
        self._gx, self._gy = np.meshgrid(self.x0 + np.arange(self.nx) * self.cell,
                                         self.y0 + np.arange(self.ny) * self.cell)
        self.keepouts = box_keepout_rects(arena_cfg, c.box_keepout_side_m, c.box_keepout_front_m)
        self.last_path: Optional[list[XY]] = None

    def reset(self) -> None:
        self.last_path = None

    def _pos(self, i: int, j: int) -> XY:
        return (self.x0 + i * self.cell, self.y0 + j * self.cell)

    def _cell(self, p: XY) -> tuple[int, int]:
        i = int(round((p[0] - self.x0) / self.cell))
        j = int(round((p[1] - self.y0) / self.cell))
        return (min(max(i, 0), self.nx - 1), min(max(j, 0), self.ny - 1))

    def update(self, robot_xy: XY, target_xy: XY, obstacles=()) -> tuple[XY, Optional[XY], Optional[str]]:
        self.last_path = None
        if math.hypot(target_xy[0] - robot_xy[0], target_xy[1] - robot_xy[1]) <= self.cfg.axis_leg_tolerance_m:
            return target_xy, None, None
        obstacles = list(obstacles)
        # 먼저 직진 여유(safe)로 길을 찾는다. 그 길의 꺾이는 점이 어떤 기물에 회전 여유
        # (turn_safe)보다 가까우면 **그 기물만** 넓혀서 다시 찾는다.
        wide: set[int] = set()
        for _ in range(self.TURN_PASSES):
            planned = self._plan(robot_xy, target_xy, obstacles, wide)
            if planned[0] == "done":
                return planned[1]
            _, pts, unreachable = planned
            bad = self._turn_conflicts(pts, obstacles, robot_xy) - wide
            if not bad:
                return self._emit(pts, robot_xy, target_xy, unreachable)
            wide |= bad
        # 넓혀도 꺾을 자리가 안 나온다 — 그 길로 가면 돌다가 기물을 친다.
        return robot_xy, None, "blocked"

    #: 꺾이는 점 검사 후 다시 찾는 횟수. 기물마다 한 번씩 넓히므로 몇 번이면 수렴한다.
    TURN_PASSES = 5

    def _radii(self, n: int, wide: set[int]) -> list[float]:
        return [self.turn_safe if i in wide else self.safe for i in range(n)]

    def _plan(self, robot_xy: XY, target_xy: XY, obstacles: list[XY], wide: set[int]):
        """("done", 결과) 또는 ("path", 편 경로, unreachable)."""
        radii = self._radii(len(obstacles), wide)
        free = self._free_grid(obstacles, robot_xy, radii)
        start = self._cell(robot_xy)
        # 출발 칸이 회피구역 안일 수 있다(기물을 막 집은 직후). 빠져나갈 수는 있어야 한다.
        free[start[1], start[0]] = True
        reach = self._reachable(start, free)
        goal_mask, unreachable = self._goal_mask(target_xy, reach)
        if goal_mask is None:
            return "done", (robot_xy, None, "blocked")
        cells = self._search(start, goal_mask, free)
        if cells is None or len(cells) < 2:
            # 이미 도착 칸 안이다. 격자 칸 중심은 부분목표로 쓰지 않는다.
            return "done", ((robot_xy, None, "blocked") if unreachable else (target_xy, None, None))
        # 펴는 선분은 직진 여유(safe)만 본다 — 넓힌 기물 옆도 직진으로는 지나갈 수 있다.
        # 꺾이는 점은 격자 칸이라 넓힌 기물에서는 이미 turn_safe 밖이다.
        return "path", self._smooth([self._pos(*c) for c in cells], obstacles, robot_xy), unreachable

    def _turn_conflicts(self, pts: list[XY], obstacles: list[XY], robot_xy: XY) -> set[int]:
        """꺾이는 점(출발점 제외)에서 제자리 회전하면 닿는 기물의 번호.

        출발점은 뺀다 — 이미 거기 서 있고, 막 집은 기물 옆처럼 피할 수 없는 경우가 있다.
        """
        bad = set()
        for v in pts[1:]:
            for i, o in enumerate(obstacles):
                if math.hypot(o[0] - robot_xy[0], o[1] - robot_xy[1]) <= self.safe:
                    continue            # 출발할 때 이미 안에 있던 기물(집은 직후)
                if math.hypot(v[0] - o[0], v[1] - o[1]) < self.turn_safe - 1e-9:
                    bad.add(i)
        return bad

    def _emit(self, pts: list[XY], robot_xy: XY, target_xy: XY, unreachable: bool):
        self.last_path = pts
        tol = self.cfg.axis_leg_tolerance_m
        k = 1
        while k < len(pts) - 1 and math.hypot(pts[k][0] - robot_xy[0], pts[k][1] - robot_xy[1]) <= tol:
            k += 1
        sub_goal = pts[k]
        if math.hypot(sub_goal[0] - robot_xy[0], sub_goal[1] - robot_xy[1]) <= tol:
            # 이미 도착 칸 중심 위에 서 있다. 도착 판정은 칸 중심 기준인데 FSM 트리거는
            # 실제 로봇 위치 기준이라 그 틈(수 cm)에서 STOP 만 영원히 나갔다(시뮬 재현).
            # 남은 몇 cm 는 실제 목표로 곧장 간다 — FSM 이 트리거 거리에서 세운다.
            if unreachable:
                return robot_xy, None, "blocked"
            return target_xy, None, None
        corner = pts[k + 1] if len(pts) > k + 1 else None
        if unreachable:
            return sub_goal, corner, "blocked"
        return sub_goal, corner, ("piece" if len(pts) - k > 1 else None)

    def _active_keepouts(self, robot_xy: XY) -> list[Rect]:
        """로봇이 이미 들어가 있는 금지 구역은 뺀다 — 막으면 빠져나오지도 못한다
        (정차 허용치만큼 더 붙어 선 직후가 그렇다)."""
        return [r for r in self.keepouts if not point_in_rect(robot_xy, r)]

    def _free_grid(self, obstacles, robot_xy: XY, radii=None) -> np.ndarray:
        radii = radii if radii is not None else [self.safe] * len(obstacles)
        free = np.ones((self.ny, self.nx), dtype=bool)
        for x0, x1, y0, y1 in self._active_keepouts(robot_xy):
            free &= ~((self._gx > x0) & (self._gx < x1) & (self._gy > y0) & (self._gy < y1))
        for (ox, oy), r in zip(obstacles, radii):
            if math.hypot(ox - robot_xy[0], oy - robot_xy[1]) <= 1e-6:
                continue
            free &= ((self._gx - ox) ** 2 + (self._gy - oy) ** 2) >= r * r
        return free

    def _passable(self, flat, i, j, di, dj, s_idx) -> bool:
        x2, y2 = i + di, j + dj
        if not (0 <= x2 < self.nx and 0 <= y2 < self.ny):
            return False
        c2 = y2 * self.nx + x2
        if not flat[c2] and c2 != s_idx:
            return False
        # 대각 이동 시 옆 두 칸 중 하나라도 막혀 있으면 모서리를 자르는 셈이라 막는다.
        if di and dj and (not flat[j * self.nx + i + di] or not flat[(j + dj) * self.nx + i]):
            return False
        return True

    def _reachable(self, start, free) -> np.ndarray:
        flat = free.ravel()
        seen = np.zeros(self.nx * self.ny, dtype=bool)
        s_idx = start[1] * self.nx + start[0]
        seen[s_idx] = True
        stack = [s_idx]
        while stack:
            c = stack.pop()
            i, j = c % self.nx, c // self.nx
            for di, dj in _DIRS:
                if not self._passable(flat, i, j, di, dj, s_idx):
                    continue
                c2 = (j + dj) * self.nx + i + di
                if not seen[c2]:
                    seen[c2] = True
                    stack.append(c2)
        return seen.reshape(self.ny, self.nx)

    def _goal_mask(self, target_xy: XY, reach):
        """도착 허용 거리 안의 도달 가능한 칸 전부. "가장 가까운 칸 하나"로 잡으면
        마지막 10 cm 를 위해 좁은 틈을 계단으로 넘는다(꺾기 3번 -> 11번 실측)."""
        d2 = (self._gx - target_xy[0]) ** 2 + (self._gy - target_xy[1]) ** 2
        mask = reach & (d2 <= self.goal_radius * self.goal_radius)
        if mask.any():
            return mask, False
        far = np.where(reach, d2, np.inf)
        if not np.isfinite(far).any():
            return None, True
        mask = np.zeros_like(reach)
        jj, ii = np.unravel_index(int(np.argmin(far)), far.shape)
        mask[jj, ii] = True
        return mask, True

    def _search(self, start, goal_mask, free):
        flat = free.ravel()
        goal = goal_mask.ravel()
        s_idx = start[1] * self.nx + start[0]
        if goal[s_idx]:
            return None
        best = [math.inf] * (self.nx * self.ny)
        parent = [-1] * (self.nx * self.ny)
        best[s_idx] = 0
        pq = [(0, s_idx)]
        found = -1
        while pq:
            cost, cell = heapq.heappop(pq)
            if cost > best[cell]:
                continue
            if goal[cell]:
                found = cell
                break
            i, j = cell % self.nx, cell // self.nx
            for k, (di, dj) in enumerate(_DIRS):
                if not self._passable(flat, i, j, di, dj, s_idx):
                    continue
                c2 = (j + dj) * self.nx + i + di
                nc = cost + _STEP[k]
                if nc < best[c2]:
                    best[c2] = nc
                    parent[c2] = cell
                    heapq.heappush(pq, (nc, c2))
        if found < 0:
            return None
        cells = []
        c = found
        while c >= 0:
            cells.append((c % self.nx, c // self.nx))
            c = parent[c]
        cells.reverse()
        return cells

    def _smooth(self, pts: list[XY], obstacles, robot_xy: XY) -> list[XY]:
        """string pulling. 출발점이 이미 회피구역 안인 장애물은 시선 검사에서 뺀다."""
        obs = [o for o in obstacles if math.hypot(o[0] - robot_xy[0], o[1] - robot_xy[1]) > self.safe]
        rects = self._active_keepouts(robot_xy)
        if not obs and not any(segment_hits_rect(pts[0], pts[-1], r) for r in rects):
            return [pts[0], pts[-1]]

        def visible(a: XY, b: XY) -> bool:
            return (all(segment_circle_clearance(a, b, c)[0] >= self.safe for c in obs)
                    and not any(segment_hits_rect(a, b, r) for r in rects))

        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not visible(pts[i], pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        return out
