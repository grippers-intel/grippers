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


# ---------------------------------------------------------------- 회전 폭주(2026-10-05)
from mission.base_monitor import BaseSpinMonitor  # noqa: E402

ROT_P = HostCommand(State.APPROACH, angular_z=0.3)
ROT_N = HostCommand(State.APPROACH, angular_z=-0.3)
HALT = HostCommand(State.APPROACH, stop=True)


def _spin():
    return BaseSpinMonitor(turn_deg=6.0, window_s=1.0, grace_s=1.0)


def _first_hit(m, cmds, rate_deg_s, yaw=0.0, t=0.0, watching=True):
    """cmds[k] 를 보내는 동안 로봇이 rate_deg_s 로 돈다. 처음 True 가 된 k(없으면 None)."""
    for k, c in enumerate(cmds):
        if m.update(t, c, P(0.95, 1.25, yaw), watching):
            return k
        t, yaw = t + 0.1, yaw + rate_deg_s * 0.1
    return None


def test_spinning_on_after_estop_is_a_spin_runaway():
    """10-05 실기: ESTOP 뒤에도 반시계 ~8°/s 로 계속 돌았다."""
    k = _first_hit(_spin(), [HALT] * 60, 8.0)
    assert k is not None and k <= 22            # grace 1 s + 창 1 s 안팎


def test_spinning_against_the_command_is_a_spin_runaway():
    """yaw- 를 보내는데 반시계로 돈다."""
    k = _first_hit(_spin(), [ROT_N] * 60, 8.0)
    assert k is not None and k <= 22


def test_normal_turns_overshoot_and_reversals_are_not_a_spin_runaway():
    m = _spin()
    t, yaw = 0.0, 0.0
    script = ([(ROT_P, 17.0)] * 20 + [(HALT, 6.0)] * 2 + [(HALT, 0.0)] * 20      # 돌고 서면서 조금 더 돈다
              + [(ROT_N, -17.0)] * 10 + [(ROT_P, -10.0), (ROT_P, 0.0)]           # 반대로 돌다 뒤집기
              + [(ROT_P, 17.0)] * 10 + [(ROT_N, 5.0)] * 3 + [(ROT_N, -17.0)] * 10  # 반대 회전(unwind)
              + [(HALT, 0.0)] * 30)
    for c, rate in script:
        noise = 0.8 if int(t * 10) % 2 else -0.8                                # 탑뷰 방향 떨림 ±0.8°
        assert not m.update(t, c, P(0.95, 1.25, yaw + noise), True), (t, c)
        t, yaw = t + 0.1, yaw + rate * 0.1


def test_spin_not_watched_while_translating_in_arm_jobs_or_unwatched():
    fwd = HostCommand(State.APPROACH, linear_x=0.1)
    assert _first_hit(_spin(), [fwd] * 40, 10.0) is None                       # 직진 중 방향 보정
    assert _first_hit(_spin(), [HostCommand(State.PLACE, stop=True)] * 40, 10.0) is None  # 팔 base 가 돈다
    assert _first_hit(_spin(), [HALT] * 40, 10.0, watching=False) is None      # 사람이 돌리는 중
