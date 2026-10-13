"""그리퍼캠 아끼기 판단(2026-10-12, 배터리)."""
from vla_common.gripper_idle import BUSY_HOLD_S, STATUS_STALE_S, camera_idle


def test_off_by_default_or_without_status():
    assert not camera_idle(0.0, 10.0, 0.0, 10.0)               # 설정 꺼짐(0) — 늘 다 푼다
    assert not camera_idle(0.5, None, 0.0, 10.0)               # 상태를 한 번도 못 받았다
    assert not camera_idle(0.5, 10.0, 0.0, 10.0 + STATUS_STALE_S + 0.1)   # 상태가 끊겼다


def test_idle_only_outside_arm_jobs():
    now = 100.0
    assert camera_idle(0.5, now, 0.0, now)                     # 팔 작업 없음 — 아낀다
    busy_until = now + BUSY_HOLD_S                             # 방금 busy 를 봤다
    assert not camera_idle(0.5, now, busy_until, now + 1.0)    # 작업 중·직후는 다 푼다
    assert camera_idle(0.5, now + BUSY_HOLD_S, busy_until, now + BUSY_HOLD_S)
