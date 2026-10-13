"""그리퍼캠을 아껴도 되는가(2026-10-12, 배터리) — gripper_cam_node 가 쓰는 판단만 따로 둔다(ROS 없이 시험).

팔 작업이 없으면 프레임을 받기만 하고 idle_period_s 마다 한 장만 푼다. 모르는 상태에서는 아끼지 않는다.
"""
from __future__ import annotations

from typing import Optional

#: 작업이 끝난 뒤에도 이만큼은 다 푼다(작업 끝 무렵 사진 · 상태 패킷 지연)
BUSY_HOLD_S = 3.0
#: 상태가 이보다 오래 안 오면 모르는 것으로 보고 다 푼다
STATUS_STALE_S = 2.0


def camera_idle(idle_period_s: float, status_at: Optional[float], busy_until: float, now: float) -> bool:
    """설정이 켜졌고(idle_period_s > 0), 상태(/vla_robot/status)가 살아 있고, 팔 작업(busy) 뒤 BUSY_HOLD_S 가
    지났으면 True."""
    if idle_period_s <= 0 or status_at is None or now - status_at > STATUS_STALE_S:
        return False
    return now >= busy_until
