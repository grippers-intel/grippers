import math

import pytest
"""바구니 정차 구역(2026-10-07): y 는 정차점 그대로, x 는 가운데 ±8 cm 안에서 기물에 안 닿는 가장 가운데 자리."""
from localization.pose import Pose
from mission.host_fsm import HostState, MissionFSM, Order
from vla_common.protocol import PiStatus, State


def P(x, y, yaw=90.0):
    return Pose(x, y, yaw, ok=True, n_cams=2, fresh=True)


def S():
    return PiStatus(boot_id="B", state=State.IDLE, busy=False, job_id=0, result=None,
                    base_ok=True, watchdog=False)


def _cfg():
    from host_config import load_host_config
    return load_host_config(None)


def _carrying(extra):
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("queen",)))
    fsm.step(P(0.99, 0.50), {"queen": [(0.99, 0.80)]}, S(), 0.0)
    centre = fsm.dest_xy
    fsm._enter(HostState.CARRY_TO_DEST)
    pmap = {"queen": [(0.99, 0.82)], **extra}                                  # 쥔 퀸(그리퍼 안)
    fsm.step(P(0.99, 0.56), pmap, S(), 0.1)
    return fsm, centre


def test_centre_is_used_when_clear():
    fsm, centre = _carrying({})
    assert fsm.dest_xy == centre and fsm._aim_shift == 0.0
    assert not any("stop zone" in e for e in fsm.events)


def test_steps_aside_from_the_knight_by_the_basket():
    """10-07 실기: 나이트가 정차점 오른쪽 위 17 cm — 바퀴와 1 cm. 왼쪽으로 비켜 선다."""
    fsm, centre = _carrying({"knight": [(1.103, 1.395)]})
    assert fsm.state in (HostState.CARRY_TO_DEST, HostState.NUDGE_BOX)   # 비면 곧장 가는 단계로(10-08)
    dx = fsm.dest_xy[0] - centre[0]
    assert fsm.dest_xy[1] == centre[1] and -0.08 - 1e-9 <= dx < 0               # y 그대로, 왼쪽으로
    assert abs(fsm._aim_shift - max(-0.06, dx)) < 1e-9                        # 겨누는 점도 같이(최대 6 cm)
    assert abs((fsm._basket().aim[0] - fsm._basket("basket").aim[0]) - fsm._aim_shift) < 1e-9
    assert any("stop zone" in e for e in fsm.events)


def test_whole_zone_blocked_halts_after_retrying_and_names_the_piece():
    cx, cy = 0.99, 1.26                                                         # 바구니 정차점(가운데)
    extra = {"rook": [(cx - 0.06, cy + 0.12)], "knight": [(cx + 0.06, cy + 0.12)]}
    fsm, _ = _carrying(extra)
    assert fsm.state == HostState.CARRY_TO_DEST                                 # 바로 멈추지 않는다
    fsm._stall.update = lambda *a, **k: False                                   # 시험 로봇은 안 움직인다
    pmap = {"queen": [(0.99, 0.82)], **extra}
    t = 0.2
    while fsm.state == HostState.CARRY_TO_DEST and t < 6.0:
        fsm.step(P(0.99, 0.56), pmap, S(), t)
        t += 0.1
    assert fsm.state == HostState.HALTED and t > 3.0                            # 3 s 다시 본 뒤
    assert "정차 구역" in fsm.halt_reason and "치워" in fsm.halt_reason and "(" in fsm.halt_reason


def test_a_brief_blocked_map_right_after_the_grasp_does_not_halt():
    """10-07: 잡은 순간 지도로 막혀 HALTED — 다음 사이클 지도로는 −4 cm 에 섰다."""
    cx, cy = 0.99, 1.26
    fsm, _ = _carrying({"rook": [(cx - 0.06, cy + 0.12)], "knight": [(cx + 0.06, cy + 0.12)]})
    assert fsm.state in (HostState.CARRY_TO_DEST, HostState.NUDGE_BOX)   # 비면 곧장 가는 단계로(10-08)
    fsm.step(P(0.99, 0.56), {"queen": [(0.99, 0.82)], "knight": [(1.106, 1.307)]}, S(), 0.3)   # 룩 유령 사라짐
    assert fsm.state in (HostState.CARRY_TO_DEST, HostState.NUDGE_BOX) and fsm.dest_xy[0] < cx


def test_assumes_the_robot_stands_tilted_and_off_by_a_little():
    """10-07 실기: 0.95(−4 cm)를 골랐는데 0.964 · 79° 로 서서 나이트와 1.2 cm. 자리 검사는 ±15° · 좌우 ±2 cm 까지 본다."""
    fsm, centre = _carrying({"knight": [(1.106, 1.307)]})
    assert fsm.state in (HostState.CARRY_TO_DEST, HostState.NUDGE_BOX)   # 비면 곧장 가는 단계로(10-08)
    dx = fsm.dest_xy[0] - centre[0]
    assert dx <= -0.06 + 1e-9                                                  # −6 cm 이상 비켜 선다


def test_from_the_side_at_stop_height_stands_on_the_robots_side():
    """정차점 높이 근처(옆)에서 잡았으면 구역에서 로봇 쪽 자리에 선다 — 옆으로 가는 거리·첫 회전을 줄인다(10-07)."""
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("queen",)))
    fsm.step(P(0.60, 1.00), {"queen": [(0.60, 1.20)]}, S(), 0.0)
    centre = fsm.dest_xy
    fsm._enter(HostState.CARRY_TO_DEST)
    fsm.step(P(0.70, 1.22, yaw=150.0), {"queen": [(0.48, 1.30)]}, S(), 0.1)   # 정차점 높이, 왼쪽
    assert fsm.dest_xy[1] == centre[1] and fsm.dest_xy[0] < centre[0] - 0.09   # 왼쪽 끝(−10 cm)


def _leaving_fsm(pose, target=(1.20, 0.90)):
    """바구니에 넣고 다음 기물(target)로 떠나는 순간."""
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("queen",)))
    fsm.step(pose, {"queen": [target], "knight": [(1.106, 1.307)]}, S(), 0.0)
    return fsm


def test_leaving_the_basket_never_drives_forward_into_it():
    """10-07 6기물: 상자를 넣고 공으로 떠나며 앞으로 빠져나가기가 바구니 쪽으로 10.7 cm — 앞면이 바구니를 넘었다."""
    pose = P(0.927, 1.262, yaw=98.5)
    fsm = _leaving_fsm(pose, target=(0.53, 1.15))
    for k in range(5):
        cmd = fsm.step(pose, {"queen": [(0.53, 1.15)], "knight": [(1.106, 1.307)]}, S(), 0.1 * (k + 1))
        assert not fsm.last_cmd_text.startswith("exit forward"), fsm.last_cmd_text
        assert cmd.linear_x == 0.0


def test_forward_exit_stops_short_of_the_basket_front():
    fsm = MissionFSM(_cfg())
    fwd = (0.0, 1.0)
    assert fsm._front_hits_basket((0.99, 1.36), fwd)          # 앞면 1.485 > 바구니 1.48 − 2 cm
    assert not fsm._front_hits_basket((0.99, 1.30), fwd)      # 앞면 1.425


def test_stop_zone_prefers_a_spot_where_it_can_turn_away():
    """나이트 옆에서 넣고 떠날 때 그 자리에서 돌 수 있는 자리(모서리 반경 밖)를 먼저 고른다."""
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("queen",)))
    fsm.step(P(0.60, 1.00), {"queen": [(0.60, 1.20)], "knight": [(1.106, 1.307)]}, S(), 0.0)
    fsm._enter(HostState.CARRY_TO_DEST)
    fsm.step(P(0.90, 0.80, yaw=90.0), {"queen": [(0.90, 1.06)], "knight": [(1.106, 1.307)]}, S(), 0.1)
    c = fsm.cfg.planner
    free_r = math.hypot(c.robot_length_m / 2, c.robot_width_m / 2) + c.piece_obstacle_radius_m \
        + fsm.cfg.mission.basket_stop_clear_m
    assert math.dist(fsm.dest_xy, (1.106, 1.307)) >= free_r


def test_near_waypoint_does_not_drive_straight_in_the_wrong_direction():
    """10-07 별: 경로 다음 점이 4 cm 라 방향을 안 재고 보던 쪽(바구니 반대)으로 5 cm 직진했다."""
    fsm = MissionFSM(_cfg())
    fsm._now = 1.0
    pose = P(0.635, 1.062, yaw=-143.0)
    cmd = fsm._drive_to(pose, (0.93, 0.96), [(1.106, 1.307)])
    assert cmd.linear_x == 0.0                                # 직진 아님(서거나 돈다)


def test_pre_stop_is_never_next_to_a_piece():
    """10-07: 구역 −10 cm 아래 30 cm 점이 퀸에서 5 cm — 계획기가 그 점으로 가며 퀸을 7.7 cm 밀었다."""
    fsm = MissionFSM(_cfg())
    fsm.set_order(Order(labels=("rook",)))
    pm = {"rook": [(1.20, 0.86)], "queen": [(0.90, 0.91)], "knight": [(1.106, 1.307)]}
    fsm.step(P(1.19, 0.58, yaw=90.0), pm, S(), 0.0)
    fsm._enter(HostState.CARRY_TO_DEST)
    pm["rook"] = [(1.19, 0.86)]
    fsm.step(P(1.19, 0.58, yaw=131.0), pm, S(), 0.1)
    goal = fsm.nav_goal
    assert goal is not None
    assert math.dist(goal, (0.90, 0.91)) >= fsm._planner.safe


def test_nudge_does_not_place_from_beside_the_stop():
    """10-07 별: 정차점 높이에서 13 cm 옆인데 바구니까지 거리만 보고 "도착" — 팔 +12.6° 로 멀리서 넣었다."""
    fsm = MissionFSM(_cfg())
    fsm.target_label, fsm.target_xy, fsm.dest_box = "star", (0.5, 1.19), "basket"
    fsm.dest_xy = (0.89, 1.26)
    fsm._enter(HostState.NUDGE_BOX)
    fsm.step(P(0.76, 1.267, yaw=45.0), {"queen": [(1.125, 1.355)]}, S(), 0.0)
    assert fsm.state == HostState.NUDGE_BOX
    assert not fsm.ready_to_advance                           # 옆으로 더 가야 한다


def test_nudge_places_from_just_beside_the_stop_with_one_turn():
    """10-08 상자: 정차점(−10 cm 자리) 바로 옆 6.5 cm 에서 바구니 반대쪽을 보고 잡았다. 옆 허용 6 cm 에 걸려
    정차점 쪽으로 +120° 돌고 5 cm 간 뒤 다시 +83° — 약 200°. 8 cm 안이면 그 자리에서 바구니 쪽으로 한 번만 돈다."""
    fsm = MissionFSM(_cfg())
    fsm.target_label, fsm.target_xy, fsm.dest_box = "box", (0.573, 1.202), "basket"
    fsm.dest_xy, fsm._aim_shift = (0.89, 1.26), -0.06
    fsm._enter(HostState.NUDGE_BOX)
    pmap = {"queen": [(1.118, 1.334)]}
    cmd = fsm.step(P(0.840, 1.301, yaw=-159.5), pmap, S(), 0.0)
    assert fsm.state == HostState.NUDGE_BOX
    assert cmd.linear_x == 0.0 and cmd.angular_z < 0          # 정차점으로 가지 않고 바구니 쪽으로(짧은 쪽) 돈다
    heading = fsm._basket().heading_deg((0.840, 1.301))
    t = 0.1
    while fsm.state == HostState.NUDGE_BOX and t < 1.0:
        cmd = fsm.step(P(0.840, 1.301, yaw=heading - 5.0), pmap, S(), t)
        assert cmd.linear_x == 0.0
        t += 0.1
    assert fsm.state == HostState.PLACE


def _nudge_sim(pose, stop, shift, drift=0.0):
    """바구니 앞 맞추기만 단순 적분으로 돌린다. (끝 상태, 돈 각도 합, 바구니에서 멀어진 최대 거리, 사건)."""
    fsm = MissionFSM(_cfg())
    fsm.target_label, fsm.target_xy, fsm.dest_box = "star", (0.5, 0.5), "basket"
    fsm.dest_xy, fsm._aim_shift = stop, shift
    fsm._enter(HostState.NUDGE_BOX)
    x, y, yaw = pose
    t, rot, worst = 0.0, 0.0, 0.0
    aim = fsm._basket().aim
    d0 = math.dist((x, y), aim)
    while fsm.state == HostState.NUDGE_BOX and t < 20.0:
        th = math.radians(yaw)
        cmd = fsm.step(P(x, y, yaw), {"star": [(x + 0.2 * math.cos(th), y + 0.2 * math.sin(th))]}, S(), t)
        yaw += math.degrees(cmd.angular_z) * 0.1
        rot += abs(math.degrees(cmd.angular_z)) * 0.1
        x += cmd.linear_x * 0.1 * math.cos(th) + (drift * 0.1 if cmd.linear_x else 0.0)
        y += cmd.linear_x * 0.1 * math.sin(th)
        worst = max(worst, math.dist((x, y), aim) - d0)
        t += 0.1
    return fsm.state, rot, worst, list(fsm.events)


def test_beside_the_stop_faces_the_aim_and_creeps_in():
    """10-08 6기물 별: 정차점 8.7 cm 옆 · 바구니까지 +4 cm. 정차점 쪽으로 185° 돌아 5 cm 간 뒤 다시 46° 돌았다.
    겨누는 점으로 가는 선이 입구 안(벽에서 2.2 cm)이면 그 선을 보고 서서 곧장 붙는다 — 한 번만 돈다."""
    state, rot, _worst, events = _nudge_sim((1.175, 1.242, -18.4), (1.09, 1.26), 0.06)
    assert state == HostState.PLACE
    assert rot < 150.0, rot                                   # 예전 ~230°
    assert any("겨누는 점을 보고" in e for e in events)


def test_close_to_the_stop_never_drives_away_from_the_basket():
    """10-08 6기물 퀸: 정차점 4.7 cm(5 cm 안) — 방향을 못 재 지금 방향(−64°, 바구니 반대)으로 6 cm 직진했다."""
    state, _rot, worst, _events = _nudge_sim((1.047, 1.213, -63.6), (1.05, 1.26), 0.06)
    assert state == HostState.PLACE
    assert worst < 0.01, worst


@pytest.mark.parametrize("drift", [0.01, -0.01])
def test_thirty_cm_nudge_does_not_stop_to_realign(drift):
    """10-08 공·나이트: 30 cm 맞추기 중 12° 넘을 때마다 서서 다시 돌았다(2번씩). 앞길이 비면 계속 간다."""
    state, rot, _worst, _events = _nudge_sim((1.110, 0.937, 134.7), (0.99, 1.26), 0.0, drift=drift)
    assert state == HostState.PLACE and rot < 60.0, rot


def _nudging(pose_xy=(0.99, 0.95)):
    fsm = MissionFSM(_cfg())
    fsm.target_label, fsm.target_xy, fsm.dest_box = "star", (0.5, 0.5), "basket"
    fsm.dest_xy, fsm._aim_shift = (0.99, 1.26), 0.0
    fsm._enter(HostState.NUDGE_BOX)
    return fsm


def test_nudge_realigns_when_drifting_past_a_piece():
    """10-08 공: 바구니 앞 직진을 25° 까지 그냥 가게 했더니 8~9° 틀어진 채 퀸 옆 2.9 cm 에 도착해 HALTED.
    틀어진 길이 기물의 회전 반경(0.20 m) 안을 지나면 예전처럼 12° 에서 다시 맞춘다."""
    pose = P(0.99, 0.95, 90.0 + 15.0)                          # 정차점 쪽에서 15° 틀어짐
    near = {"star": [(0.99 - 0.05, 1.14)], "queen": [(0.80, 1.18)]}
    far = {"star": [(0.99 - 0.05, 1.14)], "queen": [(0.50, 1.18)]}
    for pm, keeps_going in ((near, False), (far, True)):
        fsm = _nudging()
        for k in range(3):                                     # 똑바로 보고 직진을 시작한 뒤
            fsm.step(P(0.99, 0.95), pm, S(), 0.1 * k)
        fsm.step(pose, pm, S(), 0.3)                           # 15° 흐름 — 시퀀서는 한 사이클 뒤에 바꾼다
        cmd = fsm.step(pose, pm, S(), 0.4)
        assert (cmd.linear_x > 0.0) == keeps_going, (pm["queen"], cmd)


def test_base_recovery_time_does_not_count_against_the_nudge_timeout():
    """10-08 룩: 회전 폭주 복구 13 s 가 바구니 앞 맞추기 25 s 에 들어가 복구 직후 HALTED."""
    from vla_common.protocol import PiStatus
    fsm = _nudging()
    pm = {"star": [(0.94, 1.14)]}
    fsm.step(P(0.99, 0.95), pm, S(), 0.0)
    rec = PiStatus(boot_id="B", state=State.IDLE, busy=False, job_id=0, result=None,
                   base_ok=True, watchdog=False, base_recovering=True)
    t = 0.1
    while t < fsm.cfg.mission.nudge_timeout_s + 5.0:
        fsm.step(P(0.99, 0.95), pm, rec, t)
        t += 0.1
    fsm.step(P(0.99, 0.95), pm, S(), t)
    assert fsm.state == HostState.NUDGE_BOX
