"""바구니(목적지 상자) 투입 목표와 접근 판정 — 순수 계산.

`hardware/host-mac` 의 `host/basket_target.py` 와 그 도면(`docs/workshop_floorplan.html`,
2026-09-05)의 기하를 이 프로젝트로 옮긴 것이다. 좌표 규약은 host.yaml 과 같다:
map 원점 = 가벽 앞쪽 왼쪽 바닥 모서리, +x 오른쪽, +y 뒤쪽, 각도는 +x 기준 반시계(도).

## 무엇이 목표인가

상자는 `yaw 180°` = 입구가 작업구역(-y) 쪽으로 열려 있다. 그래서 투입 목표는 상자
중심이 아니라 **입구 안쪽의 작은 사각형**이다(가로 ±3 cm · 안쪽 3 cm). 도면의 호박색
사각형이 그것이고, chess 기준 x[1.320, 1.380] · y[1.450, 1.480] 이다.

## 왜 고정 90° 로 서면 안 되는가

예전 코드는 상자 앞 한 점(`dest_xy`)을 조준하고 항상 yaw 90°(정북)로 섰다. 사선으로
들어오면 그 자세에서 팔이 입구를 비껴본다. 도면이 정한 규칙은 이렇다.

    목표 중심에서 반경 0.15 m 원을 120°씩 3등분하고, 접근 방향(남쪽·6시) 부채꼴의
    바깥 호를 판정 경계선으로 쓴다. 그 호 위 어디로 들어오든 **목표 중심을 향한
    방위각**으로 제자리 정렬한다 — 좌 30° · 중앙 90° · 우 150°.

정중앙으로 들어올 때만 옛 고정 90° 와 같아진다.

⚠️ 남은 오차(정렬 허용치 ±5° 와 팔의 좌우 장착 오프셋)는 차체로 메우지 않는다.
차체는 0.5 rad/s 에 데드밴드까지 있어 몇 도짜리 회전을 못 낸다. 그 몫은 Pi 쪽
`place.base_yaw_deg` 가 팔의 base 를 틀어서 맡는다(그리퍼 턱이 마커보다 약 0.20 m
앞이라 ±15° 가 좌우 ±0.05 m 에 해당한다).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from planning.planner import wrap_deg

XY = tuple[float, float]

#: 접근 부채꼴의 중심 방위 — 로봇은 상자의 남쪽(작업구역 쪽)에서 들어온다.
SOUTH_DEG = -90.0


@dataclass(frozen=True)
class BasketTarget:
    """한 상자의 투입 목표. 좌표는 전부 map 프레임 m."""

    name: str
    center: XY                                  # 목표 사각형의 중심 = 정렬이 겨누는 점
    rect: tuple[float, float, float, float]     # x0, x1, y0, y1

    def distance(self, p: XY) -> float:
        """목표 **중심**까지의 거리. 호(경계선) 판정은 이 값으로 한다."""
        return math.hypot(p[0] - self.center[0], p[1] - self.center[1])

    def rect_distance(self, p: XY) -> float:
        """목표 **사각형**까지의 거리(안에 있으면 0). Host 1차 승인용이다."""
        x0, x1, y0, y1 = self.rect
        dx = max(x0 - p[0], 0.0, p[0] - x1)
        dy = max(y0 - p[1], 0.0, p[1] - y1)
        return math.hypot(dx, dy)

    def entry_angle_deg(self, p: XY) -> float:
        """목표 중심에서 본 로봇의 방위. 남쪽(-90°) 부채꼴 안이어야 한다."""
        return math.degrees(math.atan2(p[1] - self.center[1], p[0] - self.center[0]))

    def heading_deg(self, p: XY) -> float:
        """그 자리에서 로봇이 향해야 할 방위각. 정중앙 진입이면 90° 가 된다."""
        return math.degrees(math.atan2(self.center[1] - p[1], self.center[0] - p[0]))


def basket_target(name: str, box_xy: XY, box_size, half_width_m: float,
                  inset_depth_m: float) -> BasketTarget:
    """상자 중심·치수에서 투입 목표를 만든다.

    상자는 뒤쪽 벽에 붙어 입구가 -y 를 향한다(arena.boxes 의 yaw 180°). 그래서 입구
    모서리는 `by - 길이/2` 이고, 목표는 거기서 안쪽으로 `inset_depth_m` 만큼이다.
    """
    if half_width_m <= 0 or inset_depth_m <= 0:
        raise ValueError("목표 사각형의 반폭·깊이는 양수여야 한다")
    bx, by = box_xy[0], box_xy[1]
    edge_y = by - box_size[1] / 2.0
    return BasketTarget(
        name=name,
        center=(bx, edge_y + inset_depth_m / 2.0),
        rect=(bx - half_width_m, bx + half_width_m, edge_y, edge_y + inset_depth_m),
    )


def in_approach_sector(target: BasketTarget, p: XY, sector_deg: float) -> bool:
    """접근 부채꼴(남쪽 120°) 안에서 들어오고 있는가.

    옆이나 뒤에서 붙으면 팔이 상자 벽을 넘어야 해서 투입이 성립하지 않는다.
    """
    return abs(wrap_deg(target.entry_angle_deg(p) - SOUTH_DEG)) <= sector_deg / 2.0


def approach_ready(target: BasketTarget, p: XY, max_dist_m: float, sector_deg: float) -> bool:
    """도면의 "Host 1차 승인" — 목표 사각형에서 max_dist_m 이내 + 접근 부채꼴 안."""
    return target.rect_distance(p) <= max_dist_m and in_approach_sector(target, p, sector_deg)


def crossed_arc(target: BasketTarget, p: XY, radius_m: float, sector_deg: float) -> bool:
    """NUDGE 판정 경계호를 넘었는가 — 중심에서 radius_m 안, 그리고 부채꼴 안.

    부채꼴 조건을 같이 보는 이유: 거리만 보면 옆에서 스쳐 지나가도 참이 된다.
    """
    return target.distance(p) <= radius_m and in_approach_sector(target, p, sector_deg)
