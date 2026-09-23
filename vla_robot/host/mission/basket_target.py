"""바구니(목적지 상자) 투입 목표와 접근 판정 — 순수 계산.

`hardware/host-mac` 의 `host/basket_target.py` 와 그 도면(`docs/workshop_floorplan.html`,
2026-09-05)의 기하를 이 프로젝트로 옮긴 것이다. 좌표 규약은 host.yaml 과 같다:
map 원점 = 가벽 앞쪽 왼쪽 바닥 모서리, +x 오른쪽, +y 뒤쪽, 각도는 +x 기준 반시계(도).

## 무엇이 목표인가

상자는 `yaw 180°` = 입구가 작업구역(-y) 쪽으로 열려 있다. 그래서 투입 목표는 상자
중심이 아니라 **입구 안쪽의 작은 사각형**이다(가로 ±3 cm · 안쪽 3 cm). 도면의 호박색
사각형이 그것이고, chess 기준 x[1.320, 1.380] · y[1.450, 1.480] 이다.

## 방향은 차체가 아니라 팔이 맞춘다

차는 상자 정면(`dest_xy` = 상자 중심 x, 상자 앞 0.15 m)에 서기만 한다. 그 자리에서
투입 목표 중심을 정확히 보고 있을 리 없고, **그 차이를 차체로 메우지 않는다** —
차체는 0.5 rad/s 에 데드밴드까지 있어 몇 도짜리 회전을 못 내고, 제자리 회전이
ArUco 위치추정만 흔든다.

그래서 Host 는 그 잔차(`facing_error_deg`)를 재서 `HostCommand.arm_yaw_deg` 로 보내고,
Pi 가 팔의 base(servo 1)를 그만큼 튼다. 그리퍼 턱이 마커보다 약 0.20 m 앞이라
±15° 가 좌우 ±0.05 m 에 해당한다. 그 밖으로 벗어나면 그때만 차체를 돌린다.

> 도면(2026-09-05)에는 "목표 중심 반경 0.15 m 원의 남쪽 120° 부채꼴 바깥 호에서
> 목표 중심을 향한 방위각으로 정렬" 하는 규칙이 있었고 한때 구현도 했다. 제자리
> 정렬을 빼기로 하면서(2026-09-22) 호 판정은 걷어냈다 — 목표 사각형·중심과 잔차
> 각도만 남는다. 호 좌표는 `docs/layout/workspace_layout.md` 에 남아 있다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from planning.planner import wrap_deg

XY = tuple[float, float]

@dataclass(frozen=True)
class BasketTarget:
    """한 상자의 투입 목표. 좌표는 전부 map 프레임 m."""

    name: str
    center: XY                                  # 판정용 목표 사각형의 중심(도면 값)
    rect: tuple[float, float, float, float]     # x0, x1, y0, y1
    aim: XY                                     # **겨누는 점** — 중심보다 안쪽이다

    def distance(self, p: XY) -> float:
        """조준점까지의 거리. 차가 서야 할 거리(팔 도달거리)를 이 값으로 잰다."""
        return math.hypot(p[0] - self.aim[0], p[1] - self.aim[1])

    def rect_distance(self, p: XY) -> float:
        """목표 사각형까지의 거리(안에 있으면 0)."""
        x0, x1, y0, y1 = self.rect
        dx = max(x0 - p[0], 0.0, p[0] - x1)
        dy = max(y0 - p[1], 0.0, p[1] - y1)
        return math.hypot(dx, dy)

    def heading_deg(self, p: XY) -> float:
        """그 자리에서 조준점을 보려면 향해야 할 방위각. 정면이면 90° 다."""
        return math.degrees(math.atan2(self.aim[1] - p[1], self.aim[0] - p[0]))


def basket_target(name: str, box_xy: XY, box_size, half_width_m: float,
                  inset_depth_m: float, aim_margin_m: float = 0.0) -> BasketTarget:
    """상자 중심·치수에서 투입 목표를 만든다.

    상자는 뒤쪽 벽에 붙어 입구가 -y 를 향한다(arena.boxes 의 yaw 180°). 그래서 입구
    모서리는 `by - 길이/2` 이고, 판정 목표는 거기서 안쪽으로 `inset_depth_m` 만큼이다.

    ⚠️ **겨누는 점은 그보다 더 안쪽이다.** 판정 목표 중심은 테두리에서 15 mm 밖에 안
    들어가 있어서, 차가 2 cm 만 덜 붙어도 기물이 상자 **앞**에 떨어진다(2026-09-23
    시뮬레이터에서 실제로 그렇게 나왔다: "missed toy (+2, -21) mm"). 그래서 정차 거리
    허용 오차보다 조금 더 깊은 곳을 겨눈다 — 덜 붙으면 테두리 안쪽, 더 붙으면 더 깊이
    떨어질 뿐이다. 상자 깊이가 0.35 m 라 깊은 쪽은 여유가 많다.
    """
    if half_width_m <= 0 or inset_depth_m <= 0:
        raise ValueError("목표 사각형의 반폭·깊이는 양수여야 한다")
    if aim_margin_m < 0:
        raise ValueError("조준 여유는 음수일 수 없다")
    bx, by = box_xy[0], box_xy[1]
    edge_y = by - box_size[1] / 2.0
    center = (bx, edge_y + inset_depth_m / 2.0)
    return BasketTarget(
        name=name,
        center=center,
        rect=(bx - half_width_m, bx + half_width_m, edge_y, edge_y + inset_depth_m),
        aim=(bx, center[1] + aim_margin_m),
    )


def facing_error_deg(target: BasketTarget, p: XY, yaw_deg: float) -> float:
    """지금 향한 방향과 목표 중심 방위각의 차(도). 이 값이 팔의 base 가 틀 각도다.

    부호는 map 규약 그대로 **반시계가 +** 다. servo 1 은 반대로 음수가 반시계라
    (2026-09-22 실기 확인) Pi 가 `place.host_yaw_sign: -1` 로 뒤집어 쓴다.
    """
    return wrap_deg(target.heading_deg(p) - yaw_deg)
