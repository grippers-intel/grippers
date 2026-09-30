from localization.pose import Pose
from mission.base_monitor import BaseRunawayMonitor
from vla_common.protocol import HostCommand, State


def P(x, y, yaw=90.0):
    return Pose(x, y, yaw, ok=True, n_cams=2, fresh=True)


def _mon():
    return BaseRunawayMonitor(move_m=0.05, window_s=0.5, grace_s=1.0)


FWD = HostCommand(State.CARRY, linear_x=0.15)
ROT = HostCommand(State.APPROACH_BOX, angular_z=0.5)
STOP = HostCommand(State.CARRY, stop=True)


def test_coasting_after_a_stop_is_not_a_runaway():
    m = _mon()
    t, y = 0.0, 0.40
    for _ in range(10):                       # 1 s 직진
        assert not m.update(t, FWD, P(0.9, y), True)
        t, y = t + 0.1, y + 0.015
    for k in range(30):                       # 멈춘 뒤 0.3 s 동안 4 cm 더 가고 선다
        y += 0.013 if k < 3 else 0.0
        assert not m.update(t, STOP, P(0.9, y), True)
        t += 0.1


def test_driving_on_while_told_to_turn_is_a_runaway():
    m = _mon()
    t, y = 0.0, 0.40
    for _ in range(10):
        m.update(t, FWD, P(0.9, y), True)
        t, y = t + 0.1, y + 0.015
    hit = None
    for k in range(40):                       # 돌라고 하는데 0.13 m/s 로 곧장 간다
        if m.update(t, ROT, P(0.9, y), True):
            hit = k
            break
        t, y = t + 0.1, y + 0.013
    assert hit is not None and hit <= 17      # grace 1 s + 창 0.5 s 안팎


def test_turning_in_place_is_not_a_runaway():
    m = _mon()
    t = 0.0
    m.update(t, FWD, P(0.9, 0.4), True)
    for k in range(50):                       # 제자리 회전 — 마커가 중심에서 조금 떨어져 작은 원을 그린다
        t += 0.1
        import math
        a = 0.05 * k
        assert not m.update(t, ROT, P(0.9 + 0.03 * math.cos(a), 0.4 + 0.03 * math.sin(a), 90 + math.degrees(a)), True)


def test_not_watched_during_arm_jobs_or_outside_drive_states():
    m = _mon()
    t = 0.0
    m.update(t, FWD, P(0.9, 0.4), True)
    grasp = HostCommand(State.GRASP, stop=True)
    for k in range(40):                       # 팔을 펴면 마커가 30 cm 가까이 움직인다
        t += 0.1
        assert not m.update(t, grasp, P(0.9, 0.4 + 0.01 * k), True)
    for k in range(40):                       # 사람이 옮기는 중(감시 안 함)
        t += 0.1
        assert not m.update(t, STOP, P(0.9 + 0.02 * k, 0.4), False)
