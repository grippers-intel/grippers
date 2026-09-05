"""탑뷰+ArUco 로 "지금 이 순간 향해야 할 다음 좌표"를 계산한다.

차량은 이 좌표(map 좌표계 x, y)와 로봇의 현재 pose 만 받아서, 거기까지 가는
모터값은 스스로 계산한다 — 여기서는 그 앞단, 즉 "어디로 향할지"만 정한다.

next_waypoint() 자체은 직선(최단거리) 하나만 안다 — 로봇 → 목표를 잇는
직선상의 점을 목표 방향으로 최대 WAYPOINT_STEP_M 만큼만 내밀어서 낸다.

실제 "목표"는 매 사이클 GridPathPlanner 가 격자 탐색으로 골라주는
부분목표(sub-goal)다 — 회전량이 가장 적은 경로의 첫 구간 끝점이다.
next_waypoint() 는 이 부분목표까지의 직선만 책임지고, 경로 전체는 모른다.

회피: 그 직선이 다른 기물(원형 회피구역)에 너무 가깝게 지나가면, 경로를
전부 다시 짜는 대신 그 장애물을 살짝 비켜가는 점 하나만 끼워 넣는다. 매
사이클 로봇 위치가 바뀔 때마다 다시 계산하므로, 이렇게 한 점씩만 내밀어도
결과적으로 장애물을 부드럽게 돌아가는 경로가 된다. 바닥이 트여 있고
장애물이 몇 개뿐인 상황이라 A*/RRT 같은 전역 경로계획은 과하다고 보고 이
방식을 택했다.

ArUco 마커는 차량 "중심"에 있다. 여기서 다루는 robot_xy 는 그 마커 점이라
반지름이 없는데, 실제로 안 부딪히려면 차량 몸체 반경(mission_config.
ROBOT_RADIUS_PIECE_M)만큼 더 떨어져 있어야 한다. 그래서 안전거리는
obstacle_radius + robot_radius + margin 으로 잡는다 — 기물 자체의 회피반경에
차량 몸통 크기를 더한 것이다.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import numpy as np

import mission_config as mcfg

XY = tuple[float, float]


@dataclass
class NavResult:
    waypoint: XY
    dist_to_target: float
    blocked_by: Optional[str] = None   # 회피 중이면 "piece", 아니면 None


def _segment_circle_clearance(p0: XY, p1: XY, c: XY) -> tuple[float, float]:
    """선분 p0->p1 이 중심 c 원에 얼마나 가까운지(최근접 거리)와, 그 지점의
    선분상 진행률 t(0~1)를 돌려준다."""
    p0v, p1v, cv = np.asarray(p0, float), np.asarray(p1, float), np.asarray(c, float)
    d = p1v - p0v
    length_sq = float(d @ d)
    if length_sq < 1e-9:
        return float(np.hypot(*(p0v - cv))), 0.0
    t = float(np.clip(((cv - p0v) @ d) / length_sq, 0.0, 1.0))
    closest = p0v + t * d
    return float(np.hypot(*(closest - cv))), t


def next_waypoint(
    robot_xy: XY,
    target_xy: XY,
    obstacles: list[XY],
    obstacle_radius: float = mcfg.PIECE_OBSTACLE_RADIUS_M,
    robot_radius: float = mcfg.ROBOT_RADIUS_PIECE_M,
    margin: float = mcfg.OBSTACLE_MARGIN_M,
    step: float = mcfg.WAYPOINT_STEP_M,
) -> NavResult:
    rx, ry = robot_xy
    tx, ty = target_xy
    dist = float(np.hypot(tx - rx, ty - ry))

    # 직선 경로를 가장 심하게 막는 장애물 하나를 찾는다.
    # robot_xy 는 차량 "중심"(ArUco 마커) 이므로, 몸체가 안 부딪히려면
    # 기물 회피반경 + 차량 반경 + 여유만큼은 떨어져 있어야 한다. 여유까지
    # 포함한 안전거리를 기준으로 판단해야 한다 — 반지름만 기준으로 삼으면
    # 이미 여유 구간 안쪽까지 들어온 뒤에야 피하기 시작해서 margin 을 못
    # 지킨다.
    safe_dist = obstacle_radius + robot_radius + margin
    worst: Optional[tuple[float, XY]] = None   # (clearance, center)
    for ox, oy in obstacles:
        if np.hypot(ox - rx, oy - ry) < 1e-6:
            continue  # 로봇 자기 위치와 겹치는 관측(들고 있는 기물 오검출 등)은 무시
        clearance, t = _segment_circle_clearance((rx, ry), (tx, ty), (ox, oy))
        if t <= 1e-3:
            # 최근접점이 바로 지금 위치다 — 장애물이 옆/뒤에 있다는 뜻이라
            # 전진하면 오히려 멀어진다. 이걸 "막혔다"고 보면 접선점 근처에서
            # 제자리를 맴돈다(로컬 미니멈). 진짜 앞을 막을 때만 카운트한다.
            continue
        if clearance < safe_dist and (worst is None or clearance < worst[0]):
            worst = (clearance, (ox, oy))

    if worst is None:
        if dist <= step:
            return NavResult((tx, ty), dist)
        ux, uy = (tx - rx) / dist, (ty - ry) / dist
        return NavResult((rx + ux * step, ry + uy * step), dist)

    # 막혔으면: 장애물을 경로에 수직 방향으로 안전거리(safe_dist)만큼 비켜간 점을 우회점으로 삼는다
    _clearance, (ox, oy) = worst
    dx, dy = tx - rx, ty - ry
    seg_len = max(float(np.hypot(dx, dy)), 1e-6)
    ux, uy = dx / seg_len, dy / seg_len
    nx, ny = -uy, ux                       # 경로에 수직인 두 방향 중
    # 장애물이 로봇 기준 어느 쪽에 있는지 보고, 그 반대편(더 크게 도는 쪽)으로 비킨다
    side = 1.0 if ((ox - rx) * nx + (oy - ry) * ny) < 0 else -1.0
    dtx, dty = ox + nx * side * safe_dist, oy + ny * side * safe_dist

    ddx, ddy = dtx - rx, dty - ry
    ddist = max(float(np.hypot(ddx, ddy)), 1e-6)
    if ddist <= step:
        return NavResult((dtx, dty), dist, blocked_by="piece")
    return NavResult((rx + ddx / ddist * step, ry + ddy / ddist * step), dist, blocked_by="piece")


# ---------------------------------------------------------------------------
# 직진/정지/회전 시퀀서
#
# 차량을 항상 정면으로만 달리게 한다(메카넘휠이라 옆으로도 갈 수 있지만 차량
# 쪽 제어를 단순하게 하려고 일부러 직진 전용으로 씀). next_waypoint() 는
# "다음 좌표"만 내는데, 여기서는 그걸 감싸서 매 사이클 로봇이 실제로 뭘 해야
# 하는지(FORWARD/STOP/ROTATE)를 결정한다.
#
# 흐름: FORWARD 로 달리다 목표 방위각 오차가 허용치를 넘으면 STOP 한 사이클 ->
# ROTATE(오차가 줄 때까지 반복) -> 다시 오차가 허용치 안으로 들어오면 STOP 한
# 사이클 -> FORWARD. STOP 은 전환 신호 한 번으로만 쓴다(그 자체로 오래 멈춰
# 있진 않음 — 실제로 얼마나 정지할지는 차량 쪽이 정한다).
# ---------------------------------------------------------------------------
class DriveMode(Enum):
    FORWARD = auto()
    STOP = auto()
    ROTATE = auto()
    # yaw+/yaw- 가 계속 방향을 바꾸며 제자리 헌팅할 때(아래 DriveSequencer
    # 토글 워치독 참고) 강제로 끼워 넣는, 정렬을 무시한 짧은 전진. FORWARD와
    # cmd 상으로는 똑같이 "go"지만, 목표 방향으로 가고 있다는 보장이 없다는
    # 걸 호출부가 구분할 수 있게 별도 값으로 둔다.
    ESCAPE = auto()


@dataclass
class DriveCommand:
    mode: DriveMode
    waypoint: XY            # FORWARD 일 때 향할 지점(장애물 회피 반영됨)
    target_yaw_deg: float   # 로봇이 맞춰야 할 절대 방위각(도) — ROTATE 참고용
    yaw_error_deg: float    # 지금 로봇 yaw 와 target_yaw_deg 의 차 (-180~180)
    dist_to_target: float
    blocked_by: Optional[str] = None


class DriveSequencer:
    """FORWARD/STOP/ROTATE 상태를 들고 있다가 매 사이클 하나씩 낸다."""

    def __init__(self, yaw_tolerance_deg: float = mcfg.DRIVE_YAW_TOLERANCE_DEG) -> None:
        self.yaw_tolerance_deg = yaw_tolerance_deg
        self._mode: Optional[DriveMode] = None   # None = 이번이 구간의 첫 update()
        self._next_after_stop = DriveMode.FORWARD
        # ROTATE 진입 시점에 한 번 정한 목표각(2026-09-02, 아래 update() 참고).
        self._rotate_target_yaw: Optional[float] = None
        # ROTATE 방향 토글 워치독 상태(2026-09-03, mission_config 주석 참고).
        # 시간이 아니라 사이클 수로 잰다 — 이유는 ROTATE_OSCILLATION_ESCAPE_
        # CYCLES 정의부(mission_config.py) 주석 참고.
        self._last_rotate_sign: Optional[float] = None
        self._toggle_count = 0
        self._escape_remaining = 0   # 0이면 ESCAPE 중이 아니다.
        # ESCAPE 로 실제로 넘어간 누적 횟수(2026-09-05, 사용자가 "yaw 진동
        # 때문에 시간이 지체된다"고 보고해서 추가) — 이 모듈은 순수 상태기계라
        # 여기서 직접 로그를 찍지 않는다(reset() 코멘트 참고, 실시간
        # sleep 없이 호출만 반복하는 테스트도 있다). 호출부(mission.py
        # _approach())가 이 값이 늘었는지 보고 필요하면 사용자에게 보고한다
        # — DRIVE_YAW_TOLERANCE_DEG(현재 12도)를 더 넓혀야 할지는 실기에서
        # 이게 얼마나 자주 뛰는지를 보고 판단해야 한다(mission_config.py의
        # ROTATE_OSCILLATION_TOGGLE_LIMIT 정의부 코멘트 참고).
        self.escape_count = 0

    def reset(self) -> None:
        """새 구간(다른 기물/상자로 향할 때)을 시작할 때 부른다.

        모드를 바로 FORWARD로 정하지 않는다 — 새 구간 시작 시점에 이미 정면이
        아니면(예: 상자 쪽으로 막 돌아선 뒤 다음 기물이 대각선에 있는 경우)
        굳이 FORWARD를 한 사이클 내보내고서야 STOP/ROTATE로 넘어가는 게 아니라,
        첫 update() 에서 정렬 여부를 보고 바로 알맞은 모드로 시작한다.

        토글 워치독도 같이 초기화한다 — 다른 목표로 향하는 새 구간의 회전을
        이전 구간의 방향 이력과 섞으면 안 된다.

        `escape_count` 는 여기서 초기화하지 않는다 — 구간이 아니라 이
        인스턴스가 살아 있는 동안(대략 한 번의 run_mission.py 실행) 전체
        오실레이션 빈도를 세려는 값이다."""
        self._mode = None
        self._next_after_stop = DriveMode.FORWARD
        self._rotate_target_yaw = None
        self._last_rotate_sign = None
        self._toggle_count = 0
        self._escape_remaining = 0

    def _enter_rotate(self, yaw_err: float) -> DriveMode:
        """ROTATE 로 들어가려는 시점에 부른다. 방향이 바로 전 ROTATE 와
        반대면 토글로 센다(같은 방향이면 0으로 되돌린다) — 다다르면 ROTATE
        대신 ESCAPE 로 들어가게 한다."""
        sign = math.copysign(1.0, yaw_err) if yaw_err != 0.0 else 1.0
        if self._last_rotate_sign is not None and sign != self._last_rotate_sign:
            self._toggle_count += 1
        else:
            self._toggle_count = 0
        self._last_rotate_sign = sign
        if self._toggle_count >= mcfg.ROTATE_OSCILLATION_TOGGLE_LIMIT:
            self._toggle_count = 0
            self._last_rotate_sign = None
            self._escape_remaining = mcfg.ROTATE_OSCILLATION_ESCAPE_CYCLES
            self.escape_count += 1
            return DriveMode.ESCAPE
        return DriveMode.ROTATE

    def update(
        self,
        robot_xy: XY,
        robot_yaw_deg: float,
        target_xy: XY,
        obstacles: list[XY],
        **kwargs,
    ) -> DriveCommand:
        nav = next_waypoint(robot_xy, target_xy, obstacles, **kwargs)

        dx = nav.waypoint[0] - robot_xy[0]
        dy = nav.waypoint[1] - robot_xy[1]
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            fresh_target_yaw = robot_yaw_deg   # 이미 도착 — 방향 계산 의미 없음
        else:
            fresh_target_yaw = float(np.degrees(np.arctan2(dy, dx)))

        if self._mode == DriveMode.ESCAPE:
            # 정렬 무시하고 잠깐 전진하는 중 — 사이클이 다 찰 때까지는 판단
            # 로직 전체를 건너뛴다(회전 이력도 안 쌓는다). 다 차면 처음
            # 사이클(mode=None)처럼 처음부터 다시 판단한다.
            self._escape_remaining -= 1
            if self._escape_remaining > 0:
                yaw_err = (fresh_target_yaw - robot_yaw_deg + 180.0) % 360.0 - 180.0
                return DriveCommand(
                    mode=DriveMode.ESCAPE, waypoint=nav.waypoint,
                    target_yaw_deg=fresh_target_yaw, yaw_error_deg=yaw_err,
                    dist_to_target=nav.dist_to_target, blocked_by=nav.blocked_by,
                )
            self._mode = None

        if self._mode is None:
            # 구간의 첫 사이클 — STOP 전이 신호 없이 바로 알맞은 모드로 시작.
            yaw_err = (fresh_target_yaw - robot_yaw_deg + 180.0) % 360.0 - 180.0
            self._mode = (DriveMode.FORWARD
                          if abs(yaw_err) <= self.yaw_tolerance_deg
                          else self._enter_rotate(yaw_err))
            if self._mode == DriveMode.ROTATE:
                self._rotate_target_yaw = fresh_target_yaw
            elif self._mode == DriveMode.ESCAPE:
                return DriveCommand(
                    mode=DriveMode.ESCAPE, waypoint=nav.waypoint,
                    target_yaw_deg=fresh_target_yaw, yaw_error_deg=yaw_err,
                    dist_to_target=nav.dist_to_target, blocked_by=nav.blocked_by,
                )

        # 회전 중엔 매 사이클 다시 잰 목표각을 쫓지 않는다(2026-09-02 실기
        # — GRASP_REPLAN/RETURN_HOME에서 76도로 시작해 같은 방향으로
        # 250도+ 돈 사례). 부분목표(GridPathPlanner sub-goal)가 로봇 시작
        # 칸 근처에서 살짝 흔들리면 목표각이 사이클마다 바뀌고, 도는
        # 도중에 그걸 계속 쫓으면 방향이 안정적으로 안 잡힌다. ROTATE에
        # 들어간 시점에 한 번 정한 목표각을 그대로 쓰고, 정렬이 끝나야
        # (STOP 을 거쳐) 다시 잰다.
        target_yaw = (self._rotate_target_yaw
                     if self._mode == DriveMode.ROTATE and self._rotate_target_yaw is not None
                     else fresh_target_yaw)
        yaw_err = (target_yaw - robot_yaw_deg + 180.0) % 360.0 - 180.0
        aligned = abs(yaw_err) <= self.yaw_tolerance_deg

        out_mode = self._mode   # 이번 사이클에 내보낼 모드(전이 전 값)

        if self._mode == DriveMode.FORWARD and not aligned:
            self._mode, self._next_after_stop = DriveMode.STOP, DriveMode.ROTATE
        elif self._mode == DriveMode.ROTATE and aligned:
            self._mode, self._next_after_stop = DriveMode.STOP, DriveMode.FORWARD
            self._rotate_target_yaw = None
        elif self._mode == DriveMode.STOP:
            self._mode = self._next_after_stop
            if self._mode == DriveMode.ROTATE:
                self._rotate_target_yaw = fresh_target_yaw
                self._mode = self._enter_rotate(yaw_err)
                if self._mode == DriveMode.ESCAPE:
                    self._rotate_target_yaw = None

        return DriveCommand(
            mode=out_mode, waypoint=nav.waypoint, target_yaw_deg=target_yaw,
            yaw_error_deg=yaw_err, dist_to_target=nav.dist_to_target,
            blocked_by=nav.blocked_by,
        )


# ---------------------------------------------------------------------------
# 경로 계획 (격자 탐색 + 시선 직선화, 임의 각도)
#
# 주행 가능 영역(mission_config.DRIVE_AREA_*)을 격자로 깔아 최단 경로를 찾은
# 뒤, 그 경로를 시선(line of sight) 검사로 펴서 **임의 각도** 경로로 만든다.
#
# 왜 격자 방향에 안 묶이는가
#     차량은 "go"로 지금 보는 방향으로 전진하고 "yaw+/-"로 제자리 회전한다.
#     Host 가 탑뷰로 매 사이클 방위 오차를 보고 멈추라고 하므로 **어떤
#     방위각이든 만들 수 있다** — 격자의 8방향은 탐색 수단일 뿐이고 결과를
#     거기 맞출 이유가 없다. 회전이 모자라면 다음 사이클에 다시 틀면 된다
#     (DriveSequencer 가 FORWARD 중에도 오차가 커지면 STOP -> ROTATE 로 되돌린다).
#     격자 8방향 그대로 내보내면 45도 단위 계단이 남는데, 직선화하면 사라진다.
#
# 무엇을 최소화하는가 — 거리다.
#     한때 회전량을 최소화했었다(회전 한 번이 1.1초로 비싸서). 직선화를 넣은
#     뒤로는 그럴 필요가 없다 — 계단이 알아서 한 줄로 펴지면서 꺾임이 사라지고,
#     오히려 회전량으로 최적화하면 "곧게 가려고 멀리 도는" 경로가 남는다.
#
# 왜 매 사이클 다시 짜는가
#     구간 시작 때 한 번만 짜고 따라가면 그동안 기물 지도가 바뀔 때(geti 가
#     늦게 발견/재검출, 사람이 건드림) 낡은 경로를 그대로 밀고 간다. 매 사이클
#     전체를 다시 짜되 첫 구간만 실행하면 전역 최적성과 반응성을 같이 얻는다 —
#     이 프로젝트의 "최신 것만 믿는다" 원칙과도 맞다.
#
# 반경이 둘인 점에 주의 (mission_config 참고)
#     기물 회피는 하단부 반경(ROBOT_RADIUS_PIECE_M), 벽 여유는 암까지 포함한
#     반경(ROBOT_RADIUS_WALL_M)으로 잡는다. 벽 쪽은 DRIVE_AREA_* 가 이미
#     물러난 값이라 여기서는 격자 경계가 곧 벽 여유다.
#
# 이전 구현(가장 가까운 장애물 하나만 보고 옆으로 비켜가는 우회점 1개)은 서로
# 밀어내는 장애물 두 개 사이에서 영원히 왕복했다 — 그렇게 옮긴 차선이 다음
# 장애물에 막히는지 몰랐기 때문이다. 전체 경로를 한 번에 풀면 그 상황이 생기지
# 않고, 길이 없으면 조용히 맴도는 대신 blocked_by="blocked" 로 알린다.
# ---------------------------------------------------------------------------
_DIRS = ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1))

# 한 칸 이동 거리(정수). 대각은 sqrt(2) 배.
_STEP = tuple(141 if dx and dy else 100 for dx, dy in _DIRS)


class GridPathPlanner:
    """주행영역 격자에서 회전량이 가장 적은 경로를 찾는다.

    update() 는 (부분목표, 그다음 꺾이는 점, 회피중 여부)를 낸다. 부분목표는
    "지금 향할 곧은 구간의 끝"이라 DriveSequencer 가 그대로 쓸 수 있다.
    """

    def __init__(self, cell: float = mcfg.PLAN_CELL_M) -> None:
        # 직선화까지 끝낸 이번 사이클의 전체 경로. LiveMap 이 이걸 그린다 —
        # 부분목표(첫 구간 끝)만 넘기면 화면에서는 그 뒤가 목표까지 직선으로
        # 이어져 기물을 뚫고 가는 것처럼 보인다.
        self.last_path: Optional[list[XY]] = None
        self.cell = cell
        self.x0, self.x1 = mcfg.DRIVE_AREA_X
        self.y0, self.y1 = mcfg.DRIVE_AREA_Y
        self.nx = int(round((self.x1 - self.x0) / cell)) + 1
        self.ny = int(round((self.y1 - self.y0) / cell)) + 1
        # 칸 중심 좌표를 미리 구워 둔다 — 매 사이클 장애물 검사에 쓴다.
        self._gx, self._gy = np.meshgrid(
            self.x0 + np.arange(self.nx) * cell,
            self.y0 + np.arange(self.ny) * cell)
        # 마지막으로 실제 계산했을 때의 결과와 그때의 로봇 위치 — 아래
        # update()의 "왜 위치로 얼리는가" 주석 참고.
        self._frozen_result: Optional[tuple[XY, Optional[XY], Optional[str]]] = None
        self._frozen_robot_xy: Optional[XY] = None

    def reset(self) -> None:
        """구간이 바뀔 때 부른다. 경로는 매 사이클 처음부터 다시 짜므로
        지울 상태는 화면 표시용 last_path 뿐이다 — 얼려 둔 결과도 새
        구간과 섞이면 안 되니 같이 지운다."""
        self.last_path = None
        self._frozen_result = None
        self._frozen_robot_xy = None

    # -- 격자 -------------------------------------------------------------
    def _pos(self, i: int, j: int) -> XY:
        return self.x0 + i * self.cell, self.y0 + j * self.cell

    def _cell(self, p: XY) -> tuple[int, int]:
        i = int(round((p[0] - self.x0) / self.cell))
        j = int(round((p[1] - self.y0) / self.cell))
        return (min(max(i, 0), self.nx - 1), min(max(j, 0), self.ny - 1))

    # -- 본체 -------------------------------------------------------------
    def update(
        self,
        robot_xy: XY,
        robot_yaw_deg: float,
        target_xy: XY,
        obstacles: list[XY] = (),
        obstacle_radius: float = mcfg.PIECE_OBSTACLE_RADIUS_M,
        robot_radius: float = mcfg.ROBOT_RADIUS_PIECE_M,
        margin: float = mcfg.OBSTACLE_MARGIN_M,
        arrive_tol: float = min(mcfg.GRASP_TRIGGER_DIST_M, mcfg.PLACE_TRIGGER_DIST_M),
    ) -> tuple[XY, Optional[XY], Optional[str]]:
        """로봇이 마지막 실제 계산 때 위치에서 PATH_REPLAN_MIN_MOVE_M 이내로
        밖에 안 움직였으면 다시 계산하지 않고 그 결과를 그대로 돌려준다
        (2026-09-06, 사용자 지시로 근본 수정 — "제자리에서 이상하게 돈다").

        원인: 이 경로는 매 사이클 처음부터 다시 짜고(클래스 docstring
        "왜 매 사이클 다시 짜는가") 회전량은 비용에 안 넣는다("무엇을
        최소화하는가 — 거리다"). 그래서 로봇이 ROTATE 중이라 실제로는
        거의 안 움직였는데도, 근처 장애물을 사이에 둔 좌/우 우회 비용이
        비슷하면 mm 단위 위치 흔들림만으로 탐색 결과(sub_goal)가 뒤집힐
        수 있다. DriveSequencer는 회전 관성(coast)으로 ROTATE가 끝난 뒤에도
        STOP<->ROTATE를 다시 오갈 수 있는데, 그때마다 이 흔들리는
        sub_goal로 목표각을 다시 잠그면 실제 필요한 것보다 훨씬 큰 호를
        그리며 뱅뱅 도는 것처럼 보였다(실기 로그 — CARRY_TO_DEST, 위치는
        5cm 이내인데 yaw는 135도 넘게 돌았다 되돌아옴).

        DriveSequencer의 모드(ROTATE/STOP/FORWARD)를 직접 물어서 얼리지
        않는 이유: 그 모드 이름과 "실제로 유의미하게 움직였는가"가
        정확히 대응하지 않는다 — 예를 들어 FORWARD로 막 전환된 한
        사이클은 아직 실제로는 안 움직였을 수 있고, 반대로 새 부분목표에
        막 도착한 순간(ROTATE 진입 직전)은 이미 그 전 사이클까지 충분히
        움직인 뒤다. 실제로 물어야 할 것은 "그때보다 지금 위치가 정말
        달라졌는가" 그 자체이므로, 이 클래스가 직접 자신의 마지막 계산
        위치와 비교해 판단한다 — 회전은 정의상 제자리라 이 값이 저절로
        얼어붙고, 실제 전진은 한 사이클 이동량(초당 약 0.1~0.2m, 14Hz
        기준 7~14mm)이 이 문턱보다 훨씬 커서 사실상 매 사이클 다시
        계산된다."""
        if (self._frozen_result is not None and self._frozen_robot_xy is not None
                and math.hypot(robot_xy[0] - self._frozen_robot_xy[0],
                               robot_xy[1] - self._frozen_robot_xy[1])
                <= mcfg.PATH_REPLAN_MIN_MOVE_M):
            return self._frozen_result

        def _freeze(result: tuple[XY, Optional[XY], Optional[str]]):
            self._frozen_result = result
            self._frozen_robot_xy = robot_xy
            return result

        self.last_path = None
        if math.hypot(target_xy[0] - robot_xy[0],
                      target_xy[1] - robot_xy[1]) <= mcfg.AXIS_LEG_TOLERANCE_M:
            return _freeze((target_xy, None, None))

        free = self._free_grid(obstacles, robot_xy,
                               obstacle_radius + robot_radius + margin)
        start = self._cell(robot_xy)
        # 출발 칸이 회피구역 안일 수 있다(기물을 막 집은 직후 등) — 거기서
        # 빠져나갈 수는 있어야 하므로 그 칸만 예외로 연다.
        free[start[1], start[0]] = True

        reach = self._reachable(start, free)
        goal_mask, unreachable = self._goal_mask(target_xy, reach, arrive_tol)
        if goal_mask is None:
            # 갈 수 있는 칸이 하나도 없다 — 제자리에 선다. 목표로 직진시키면
            # 기물을 뚫고 간다.
            return _freeze((robot_xy, None, "blocked"))

        cells = self._search(start, goal_mask, free)
        if cells is None or len(cells) < 2:
            return _freeze((target_xy, None, None))     # 이미 도착 거리 안

        pts = self._smooth([self._pos(*c) for c in cells], obstacles, robot_xy,
                           obstacle_radius + robot_radius + margin)
        self.last_path = pts
        sub_goal = pts[1]
        corner = pts[2] if len(pts) > 2 else None
        if unreachable:
            # 목표까지는 못 간다(기물이 자유공간을 갈라놨다). 갈 수 있는 데까지
            # 가 두면 기물을 하나씩 치우면서 길이 열린다. 뚫고 가지는 않는다.
            return _freeze((sub_goal, corner, "blocked"))
        # 한 구간이면 곧장 간 것. 그보다 많으면 뭔가 피하고 있다는 뜻 —
        # LiveMap 이 그때만 회피 표시를 한다.
        return _freeze((sub_goal, corner, ("piece" if len(pts) > 2 else None)))

    def _smooth(self, pts: list[XY], obstacles, robot_xy: XY,
                safe: float) -> list[XY]:
        """격자 경로를 시선 검사로 편다(string pulling).

        앞에서부터 갈 수 있는 가장 먼 점까지 직선으로 잇는다. 격자의 45도
        계단이 한 줄로 합쳐지면서 임의 각도 경로가 된다.

        출발 지점이 이미 회피구역 안일 수 있으므로(기물을 막 집은 직후 등)
        그런 장애물은 시선 검사에서 뺀다 — 안 그러면 첫 구간이 통째로 막혀
        직선화가 하나도 안 된다.
        """
        obs = [(ox, oy) for ox, oy in obstacles
               if math.hypot(ox - robot_xy[0], oy - robot_xy[1]) > safe]
        if not obs:
            return [pts[0], pts[-1]]

        def visible(a: XY, b: XY) -> bool:
            for c in obs:
                clearance, _t = _segment_circle_clearance(a, b, c)
                if clearance < safe:
                    return False
            return True

        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not visible(pts[i], pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        return out

    def _free_grid(self, obstacles, robot_xy: XY, safe: float) -> np.ndarray:
        """칸마다 장애물 루프를 도는 대신 한 번에 굽는다. (ny, nx) bool."""
        free = np.ones((self.ny, self.nx), dtype=bool)
        for ox, oy in obstacles:
            # 로봇 자기 위치와 겹치는 관측(들고 있는 기물 오검출 등)은 무시
            if math.hypot(ox - robot_xy[0], oy - robot_xy[1]) <= 1e-6:
                continue
            free &= ((self._gx - ox) ** 2 + (self._gy - oy) ** 2) >= safe * safe
        return free

    def _passable(self, flat, nx, ny, i, j, di, dj, s_idx) -> bool:
        """(i,j) 에서 (i+di, j+dj) 로 갈 수 있는가.

        대각으로 갈 때 옆 두 칸 중 하나라도 막혀 있으면 모서리를 잘라 지나가는
        셈이 되므로 막는다 — 그러면 차량이 기물 회피원을 스치고 지나간다.
        """
        x2, y2 = i + di, j + dj
        if not (0 <= x2 < nx and 0 <= y2 < ny):
            return False
        c2 = y2 * nx + x2
        if not flat[c2] and c2 != s_idx:
            return False
        if di and dj:
            if not flat[j * nx + (i + di)] or not flat[(j + dj) * nx + i]:
                return False
        return True

    def _reachable(self, start, free: np.ndarray) -> np.ndarray:
        """출발 칸에서 실제 갈 수 있는 칸들. (ny, nx) bool.

        자유공간이 기물로 갈라져 있을 수 있어서 "비어 있다"와 "갈 수 있다"는
        다르다. 이걸 안 보면 건너편 칸을 목표로 잡고 경로 없음이 나온다.
        """
        nx, ny = self.nx, self.ny
        flat = free.ravel()
        seen = np.zeros(nx * ny, dtype=bool)
        s_idx = start[1] * nx + start[0]
        seen[s_idx] = True
        stack = [s_idx]
        while stack:
            c = stack.pop()
            i, j = c % nx, c // nx
            for di, dj in _DIRS:
                if not self._passable(flat, nx, ny, i, j, di, dj, s_idx):
                    continue
                c2 = (j + dj) * nx + (i + di)
                if seen[c2]:
                    continue
                seen[c2] = True
                stack.append(c2)
        return seen.reshape(ny, nx)

    def _goal_mask(self, target_xy: XY, reach: np.ndarray, arrive_tol: float):
        """"도착"으로 볼 칸들과, 목표에 못 닿는지 여부를 돌려준다.

        mission 이 GRASP/PLACE 로 넘기는 거리 안에 들어가기만 하면 되므로
        목표 점까지 끝까지 파고들 이유가 없다.

        ⚠️ 이걸 "목표에 가장 가까운 칸 하나"로 잡으면 안 된다. 상자 앞이나
           기물 옆의 마지막 10여 cm 를 위해 좁은 틈을 계단으로 넘게 돼서,
           실측에서 꺾기 3번이면 될 경로가 11번으로 늘어났다.
        """
        d2 = (self._gx - target_xy[0]) ** 2 + (self._gy - target_xy[1]) ** 2
        mask = reach & (d2 <= arrive_tol * arrive_tol)
        if mask.any():
            return mask, False
        far = np.where(reach, d2, np.inf)
        if not np.isfinite(far).any():
            return None, True
        mask = np.zeros_like(reach)
        jj, ii = np.unravel_index(int(np.argmin(far)), far.shape)
        mask[jj, ii] = True
        return mask, True

    def _search(self, start, goal_mask: np.ndarray, free: np.ndarray):
        """최단거리 경로를 칸 목록으로 돌려준다(8방향, 대각은 sqrt(2)).

        여기서 나온 계단 경로는 호출부에서 _smooth() 가 직선으로 편다.
        상태를 (칸번호 * 8 + 방향) 이 아니라 칸번호만으로 다뤄도 되는 이유는
        회전 비용을 안 쓰기 때문이다 — 상태 수가 8배 줄어 그만큼 빠르다.
        """
        nx, ny = self.nx, self.ny
        flat = free.ravel()
        goal_flat = goal_mask.ravel()
        s_idx = start[1] * nx + start[0]
        if goal_flat[s_idx]:
            return None                            # 이미 도착 거리 안

        INF = float("inf")
        best = [INF] * (nx * ny)
        parent = [-1] * (nx * ny)
        best[s_idx] = 0
        pq = [(0, s_idx)]
        found = -1
        while pq:
            cost, cell = heapq.heappop(pq)
            if cost > best[cell]:
                continue
            if goal_flat[cell]:
                found = cell
                break
            i, j = cell % nx, cell // nx
            for ni in range(8):
                dx, dy = _DIRS[ni]
                if not self._passable(flat, nx, ny, i, j, dx, dy, s_idx):
                    continue
                c2 = (j + dy) * nx + (i + dx)
                nc = cost + _STEP[ni]
                if nc < best[c2]:
                    best[c2] = nc
                    parent[c2] = cell
                    heapq.heappush(pq, (nc, c2))
        if found < 0:
            return None

        cells = []
        c = found
        while c >= 0:
            cells.append((c % nx, c // nx))
            c = parent[c]
        cells.reverse()
        return cells
