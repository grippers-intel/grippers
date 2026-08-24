#!/usr/bin/env python3
"""자동 GRASP 시퀀스 — 정렬 → GRASP 진입 → 면적 기반 미세 전진 → 파지.

2026-08-24 사용자 지시로 grasp_test_console.py의 수동 단계를 자동화한다.
수동 콘솔은 진단용으로 그대로 남는다 — 이 파일은 "확정된 절차"를 담는다.

    1단계 정렬  물체가 카메라 정중앙에 오도록 제자리 회전하고,
                카메라 기준 전방 TARGET_FORWARD_M에 오도록 전후진한다.
    2단계 진입  safe -> grasp 자세로 내려가고 그리퍼를 최대로 연다.
    3단계 접근  느리게 직진하면서 그리퍼 캠 면적이 GRIPPER_AREA_TARGET_PX2를
                넘으면 정지하고 파지한다.
    4단계 복귀  midpoint -> safe -> idle (CARRY_IDLE).

⚠️ 이 도구가 **오도메트리로 위치를 정하지 않는 이유**를 반드시 알아둘 것.
`/odom_raw`는 엔코더가 아니라 **명령으로 받은 속도를 그대로 적분**한다 —
바퀴가 멈춰 있어도 완벽하게 이동했다고 보고한다. 실제로 2026-08-24 실기
로그 두 건에서 "정지 시 잔여거리 x 그리퍼캠 면적"이 물리적으로 일정해야
하는데(면적은 거리의 제곱에 반비례한다) 1830 대 996으로 1.8배 어긋났다.
그래서 두 단계 모두 **눈으로 보는 값**으로 닫는다:

    1단계는 depth 카메라의 관측 거리/좌우 오프셋으로 닫는다
    3단계는 그리퍼 캠의 컨투어 면적으로 닫는다

오도메트리는 "너무 멀리 갔다"를 막는 **안전 상한**으로만 쓰고, 도달 판정에는
절대 쓰지 않는다.

사전 준비(grasp_test_console.py와 동일):
    odom_publisher / depth_camera / depth_cam_rotate_node / perception_node /
    arm_driver 가 모두 떠 있어야 한다.

실행:
    python3 /grippers/tools/auto_grasp_sequence.py --raw-cls rook
    python3 /grippers/tools/auto_grasp_sequence.py --raw-cls rook --dry-run-align

--dry-run-align은 1단계 정렬까지만 하고 멈춘다 — 팔을 안 움직이므로 정렬
파라미터를 안전하게 조정할 때 쓴다.
"""
from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions

from grasp_test_console import (
    CLASS_TO_PROFILE,
    GraspTestNode,
    GripperCam,
    RunLog,
    estimate_position,
    odom_distance_m,
    restart_perception_node,
    save_yolo_annotated,
    start_stream_server,
)
from grippers_arm.floor_grasp_profiles import FLOOR_GRASP_PROFILES

# --- 1단계 정렬 목표 -------------------------------------------------------

# 사용자 지정(2026-08-24): "카메라 기준 25cm 정면".
TARGET_FORWARD_M = 0.25
# 전방 거리 허용 오차. 거리 모델 자체의 실측 오차가 40/70/104cm에서 ±0.4cm이므로
# (perception_node.py BBOX_PADDING_PX 주석) 그보다 넉넉하게 잡는다 — 모델
# 정확도보다 타이트하게 잡으면 영원히 수렴하지 못한다.
FORWARD_TOL_M = 0.015
# 좌우 허용 오차. 그리퍼 개구(168mm)의 절반보다 훨씬 작아야 물체가 손가락
# 사이로 들어온다.
LATERAL_TOL_M = 0.010

# 사용자 지정(2026-08-24): 회전 속도 0.25 rad/s.
# ⚠️ 실측으로 회전이 확인된 가장 낮은 값은 0.3이다(1.2~0.3 전부 회전,
# tools/inplace_rotation_test.py). 0.25는 그 아래라 **아직 미검증**이다 —
# 정지마찰을 못 이기면 명령만 나가고 안 도는데, 오도메트리는 돌았다고
# 보고하므로 로그로는 알 수 없다. 그래서 아래 _align은 회전 명령을 낸 뒤
# **관측 x가 실제로 움직였는지**를 확인하고, 안 움직이면 경고한다(눈으로
# 보는 신호라 이건 못 속인다).
ALIGN_TURN_RAD_S = 0.25
ALIGN_DRIVE_MPS = 0.06  # 직진 데드밴드(0.05) 바로 위 — 실측으로 움직임 확인

# ⚠️ 2026-08-24 실기: 회전 버스트를 0.30s 고정으로 냈다가 **40회 내내 진동만
# 하고 수렴하지 못했다**. 원인은 단순하다 —
#
#     버스트 1회 = 0.25 rad/s x 0.30s = 0.075 rad = 4.30도
#     48cm에서 그만큼 돌면 물체가 화면에서 41px(3.4cm) 움직인다(실측 평균
#     41.2px, 이론 44.3px — 즉 명령대로 정확히 돌고 있었다)
#     그런데 허용 오차 +-1cm는 12.3px다
#
# 한 걸음이 허용폭의 3.4배라 원리적으로 절대 들어갈 수 없다. 로봇은 정확히
# 시킨 대로 움직였고 틀린 건 제어 법칙이었다.
#
# 그래서 버스트 시간을 **오차에 비례**시킨다: 남은 각도를 각속도로 나눈
# 시간만큼만 돈다. 큰 오차는 크게, 작은 오차는 짧게.
TURN_BURST_GAIN = 0.7  # 목표를 지나치지 않도록 계산값의 이만큼만 간다
TURN_BURST_MIN_S = 0.08  # 이보다 짧으면 가속 구간뿐이라 안 움직인다
TURN_BURST_MAX_S = 0.50
DRIVE_BURST_S = 0.30
ALIGN_SETTLE_S = 0.40  # 명령 후 관측 전에 기다리는 시간(흔들림 가라앉히기)

ALIGN_MAX_ITERATIONS = 40
ALIGN_MAX_LOST_FRAMES = 5
# 회전 명령을 냈는데 관측 x가 이만큼(px)도 안 움직이면 "안 도는 것"으로 본다.
TURN_PROGRESS_PX = 3.0
TURN_STALL_LIMIT = 3
# 한 반복 사이에 관측 x가 이만큼 튀면 다른 물체를 잡은 오검출로 보고 버린다.
# 2026-08-24 실기 40번째 반복에서 x가 300 -> 616으로 튀며 좌우 오차가 갑자기
# +25.5cm로 보고됐다 — 그 값으로 회전 명령을 내면 로봇이 엉뚱하게 돈다.
# 정상 회전 1회의 최대 이동량(약 41px)보다 충분히 크게 잡는다.
OBSERVATION_JUMP_PX = 150.0

# --- 3단계 미세 전진 -------------------------------------------------------

# 사용자 지정(2026-08-24): "적어도 12만 이상이면 정지하고 파지".
#
# 근거가 되는 실측(그리퍼캠 컨투어 면적 -> 파지 결과):
#     141293 -> load 0.0899 성공 (가장 확실했던 파지)
#      82619 -> load 0.0899 성공
#      44365 -> load 0.0665 성공하긴 했으나 여유가 적었다
# 즉 4만대에서도 물리긴 하지만 12만은 확실히 물리는 영역이다.
GRIPPER_AREA_TARGET_PX2 = 120000.0
APPROACH_SPEED_MPS = 0.06
APPROACH_POLL_S = 0.2
# 안전 상한 — **도달 판정이 아니라 폭주 방지용**이다(모듈 docstring 참고).
# 1단계가 25cm에 맞춰 놨으므로 그보다 한참 더 가면 뭔가 잘못된 것이다.
APPROACH_MAX_TRAVEL_M = 0.35
APPROACH_MAX_SEC = 40.0
# 이 시간 동안 면적이 한 번도 안 잡히면 중단한다 — 물체가 시야 밖이다.
APPROACH_MAX_BLIND_SEC = 6.0

LOAD_THRESHOLD = 0.04  # domain/task/states.py GraspState.LOAD_THRESHOLD과 동일


class AutoGraspNode(GraspTestNode):
    """GraspTestNode에 클램프 없는 회전용 발행자를 더한다.

    `odom_publisher_node`는 `cmd_vel`의 angular.z를 ±0.5 rad/s로 자른다.
    지금 쓰는 0.25는 안 잘리지만, 나중에 이 값을 올릴 때 조용히 잘리는 걸
    막으려고 회전은 처음부터 클램프 없는 토픽으로 낸다(scan_track_control.py의
    같은 날짜 주석과 같은 이유)."""

    def __init__(self):
        super().__init__()
        self.turn_pub = self.create_publisher(Twist, "controller/cmd_vel", 10)

    def _burst(self, publisher, twist, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            publisher.publish(twist)
            self.pump()
            time.sleep(0.05)
        publisher.publish(Twist())
        self.pump()

    def turn_burst(self, angular_z, seconds):
        twist = Twist()
        twist.angular.z = angular_z
        self._burst(self.turn_pub, twist, seconds)

    def drive_burst(self, linear_x, seconds=DRIVE_BURST_S):
        twist = Twist()
        twist.linear.x = linear_x
        self._burst(self.cmd_pub, twist, seconds)


def _observe(node, raw_cls):
    """(forward_m, lateral_m, obs_x) 또는 (None, None, None)."""
    obs = node.observe(raw_cls)
    if obs is None or not obs.found:
        return None, None, None
    forward_m, lateral_m = estimate_position(obs, raw_cls)
    if forward_m is None:
        return None, None, None
    return forward_m, lateral_m, obs.x


def turn_burst_seconds(lateral_m: float, forward_m: float) -> float:
    """남은 좌우 오차를 없애는 데 필요한 회전 시간(초).

    물체까지의 거리 forward_m에서 좌우로 lateral_m 떨어져 있으면 필요한
    회전각은 atan(lateral/forward)다. 그 각도를 각속도로 나눈 시간만큼 돈다.
    TURN_BURST_GAIN(<1)을 곱해 목표를 지나치지 않고 접근하게 한다.

    고정 시간 버스트로는 왜 안 되는지는 TURN_BURST_GAIN 위 주석 참고 —
    2026-08-24 실기에서 40회 내내 진동만 했다."""
    if forward_m <= 0.0:
        return TURN_BURST_MIN_S
    theta = math.atan2(abs(lateral_m), forward_m)
    seconds = TURN_BURST_GAIN * theta / ALIGN_TURN_RAD_S
    return max(TURN_BURST_MIN_S, min(TURN_BURST_MAX_S, seconds))


def align(node, raw_cls, log, turn_only=False) -> bool:
    """물체를 카메라 정면(그리고 turn_only가 아니면 TARGET_FORWARD_M 거리)에
    놓는다. 성공하면 True.

    회전과 전후진을 **한 번에 하나씩만** 낸다 — 2026-08-23 실기에서 둘을
    섞어 냈다가 로봇이 좌측으로 90도 돌아 목표를 이탈한 적이 있다
    (scan_track_control.py 모듈 docstring). 좌우가 먼저 맞아야 전진이
    의미가 있으므로 좌우를 우선한다."""
    goal = "좌우 0cm" if turn_only else f"전방 {TARGET_FORWARD_M*100:.0f}cm, 좌우 0cm"
    print(f"\n[1단계] 정렬 — 목표: {goal}")
    lost = 0
    turn_stall = 0
    last_x = None
    last_turn_sign = 0

    for iteration in range(1, ALIGN_MAX_ITERATIONS + 1):
        forward_m, lateral_m, obs_x = _observe(node, raw_cls)
        if forward_m is None:
            lost += 1
            print(f"  [{iteration:2d}] 물체를 못 찾음 ({lost}/{ALIGN_MAX_LOST_FRAMES})")
            if lost >= ALIGN_MAX_LOST_FRAMES:
                print("  정렬 실패 — 물체를 연속으로 놓쳤습니다")
                log.log("align_failed", reason="lost")
                return False
            time.sleep(ALIGN_SETTLE_S)
            continue

        # 오검출로 x가 튄 프레임은 버린다(OBSERVATION_JUMP_PX 주석 참고).
        if last_x is not None and abs(obs_x - last_x) > OBSERVATION_JUMP_PX:
            print(
                f"  [{iteration:2d}] x가 {last_x:.0f} -> {obs_x:.0f}로 튀었습니다 "
                "— 오검출로 보고 이 프레임을 버립니다"
            )
            log.log("align_outlier", iteration=iteration, x=obs_x, last_x=last_x)
            lost += 1
            if lost >= ALIGN_MAX_LOST_FRAMES:
                print("  정렬 실패 — 관측이 계속 튑니다")
                log.log("align_failed", reason="jumpy")
                return False
            time.sleep(ALIGN_SETTLE_S)
            continue
        lost = 0

        forward_err = forward_m - TARGET_FORWARD_M
        print(
            f"  [{iteration:2d}] 전방 {forward_m*100:5.1f}cm (오차 {forward_err*100:+5.1f}) "
            f"좌우 {lateral_m*100:+5.1f}cm  x={obs_x:.1f}"
        )
        log.log(
            "align_step", iteration=iteration, forward_m=forward_m,
            lateral_m=lateral_m, x=obs_x,
        )

        if abs(lateral_m) > LATERAL_TOL_M:
            # lateral_m은 +면 우측(estimate_position 규약) → 오른쪽으로 돌아야
            # 하고, REP103에서 오른쪽 회전은 angular.z<0이다.
            turn_sign = -1.0 if lateral_m > 0 else 1.0
            reversed_direction = last_turn_sign != 0 and turn_sign != last_turn_sign
            if last_x is not None and not reversed_direction and abs(obs_x - last_x) < TURN_PROGRESS_PX:
                turn_stall += 1
                if turn_stall >= TURN_STALL_LIMIT:
                    print(
                        f"  ⚠️ 회전 명령을 {turn_stall}번 냈는데 화면상 물체가 "
                        f"{TURN_PROGRESS_PX}px도 안 움직였습니다 — "
                        f"{ALIGN_TURN_RAD_S} rad/s 또는 최소 버스트 "
                        f"{TURN_BURST_MIN_S}s가 정지마찰을 못 이기는 것으로 "
                        "보입니다. ALIGN_TURN_RAD_S를 0.3 이상으로 올리거나 "
                        "TURN_BURST_MIN_S를 늘리세요."
                    )
                    log.log("align_failed", reason="turn_stall", turn_rad_s=ALIGN_TURN_RAD_S)
                    return False
            else:
                # 방향을 바꾼 직후 한 번은 백래시/관성 때문에 거의 안 움직인다 —
                # 2026-08-24 실기 로그에서 반전 직후 x 변화가 0.3px였다. 그걸
                # 끼임으로 오판하지 않는다.
                turn_stall = 0
            last_x = obs_x
            last_turn_sign = turn_sign
            seconds = turn_burst_seconds(lateral_m, forward_m)
            print(f"       회전 {turn_sign*ALIGN_TURN_RAD_S:+.2f} rad/s x {seconds:.3f}s")
            node.turn_burst(turn_sign * ALIGN_TURN_RAD_S, seconds)
            time.sleep(ALIGN_SETTLE_S)
            continue

        last_turn_sign = 0
        turn_stall = 0
        if turn_only:
            print(f"  회전 정렬 완료 — 좌우 {lateral_m*100:+.1f}cm (전방 {forward_m*100:.1f}cm, 미보정)")
            log.log("align_done", forward_m=forward_m, lateral_m=lateral_m,
                    iterations=iteration, turn_only=True)
            return True

        last_x = None
        if abs(forward_err) > FORWARD_TOL_M:
            node.drive_burst(-ALIGN_DRIVE_MPS if forward_err < 0 else ALIGN_DRIVE_MPS)
            time.sleep(ALIGN_SETTLE_S)
            continue

        print(f"  정렬 완료 — 전방 {forward_m*100:.1f}cm, 좌우 {lateral_m*100:+.1f}cm")
        log.log("align_done", forward_m=forward_m, lateral_m=lateral_m, iterations=iteration)
        return True

    print(f"  정렬 실패 — {ALIGN_MAX_ITERATIONS}회 안에 수렴하지 못했습니다")
    log.log("align_failed", reason="max_iterations")
    return False


def approach_until_area(node, cam, log) -> float | None:
    """그리퍼 캠 면적이 목표를 넘을 때까지 느리게 직진한다. 정지 시 면적 반환.

    ⚠️ 정지 판정은 **면적으로만** 한다. 이동거리는 오도메트리 값이라 실제
    이동의 증거가 못 되고(모듈 docstring), 여기서는 폭주 방지 상한으로만
    쓴다."""
    print(f"\n[3단계] 미세 전진 — 그리퍼캠 면적 {GRIPPER_AREA_TARGET_PX2:.0f}px² 도달 시 정지")
    start_pose = node._pose
    started = time.monotonic()
    last_seen = started
    twist = Twist()
    twist.linear.x = APPROACH_SPEED_MPS

    try:
        while True:
            node.cmd_pub.publish(twist)
            node.pump()
            time.sleep(APPROACH_POLL_S)

            area = cam.measure_area_px2()
            now = time.monotonic()
            travelled = odom_distance_m(start_pose, node._pose)

            if area is not None:
                last_seen = now
                print(f"    면적 {area:8.0f}px²  (odom {travelled:.3f}m — 참고용)")
                log.log("approach_sample", area_px2=area, odom_m=travelled)
                if area >= GRIPPER_AREA_TARGET_PX2:
                    print(f"  목표 면적 도달 ({area:.0f} ≥ {GRIPPER_AREA_TARGET_PX2:.0f}) — 정지")
                    log.log("approach_done", area_px2=area, odom_m=travelled)
                    return area
            else:
                print(f"    면적 검출 안 됨  (odom {travelled:.3f}m — 참고용)")
                if now - last_seen > APPROACH_MAX_BLIND_SEC:
                    print(f"  중단 — {APPROACH_MAX_BLIND_SEC}s 동안 물체가 그리퍼캠에 안 잡힘")
                    log.log("approach_failed", reason="blind", odom_m=travelled)
                    return None

            if travelled is not None and travelled > APPROACH_MAX_TRAVEL_M:
                print(
                    f"  중단 — 안전 상한 {APPROACH_MAX_TRAVEL_M}m 초과(odom 기준). "
                    "1단계가 25cm에 맞춰 놨는데 이만큼 갔다면 뭔가 잘못됐습니다"
                )
                log.log("approach_failed", reason="max_travel", odom_m=travelled)
                return None
            if now - started > APPROACH_MAX_SEC:
                print(f"  중단 — 시간 상한 {APPROACH_MAX_SEC}s 초과")
                log.log("approach_failed", reason="timeout", odom_m=travelled)
                return None
    finally:
        node.cmd_pub.publish(Twist())
        node.pump()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raw-cls", default="rook", choices=sorted(CLASS_TO_PROFILE))
    ap.add_argument("--profile", default=None)
    ap.add_argument("--dry-run-align", action="store_true",
                    help="1단계 정렬까지만 하고 멈춘다(팔을 안 움직인다)")
    ap.add_argument("--turn-only", action="store_true",
                    help="정렬에서 전후진을 빼고 회전만 한다 — 회전 수렴만 따로 볼 때")
    args = ap.parse_args()

    profile = args.profile or CLASS_TO_PROFILE[args.raw_cls]
    close_width_mm = FLOOR_GRASP_PROFILES[profile].close_width_mm
    preopen_mm = FLOOR_GRASP_PROFILES[profile].preopen_width_mm

    log = RunLog(args.raw_cls, profile)
    print(f"대상: raw_cls={args.raw_cls}  profile={profile}  close_width={close_width_mm}mm")
    print(f"분석용 로그: {log.path}")

    if subprocess.run(
        ["pgrep", "-f", "grippers_perception/perception_node"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    ).returncode != 0:
        print("\n⚠️  perception_node가 없습니다 — 1단계 정렬이 불가능합니다.")
        return 1

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = AutoGraspNode()
    cam = None
    perception_was_killed = False
    try:
        capture = save_yolo_annotated(node, args.raw_cls)
        if capture is not None:
            log.log("start_yolo_capture", **capture)

        if not align(node, args.raw_cls, log, turn_only=args.turn_only):
            return 2
        if args.dry_run_align:
            print("\n--dry-run-align — 정렬까지만 하고 종료합니다.")
            return 0

        # 2단계 -------------------------------------------------------
        print("\n[2단계] GRASP 진입")
        if not (node.move_floor_pose(profile, "safe") and node.move_floor_pose(profile, "grasp")):
            print("  GRASP 진입 실패 — arm.log 확인")
            log.log("grasp_entry", ok=False)
            node.move_floor_pose(profile, "recover_idle")
            return 3
        log.log("grasp_entry", ok=True)
        node.set_gripper(preopen_mm)
        print(f"  그리퍼 열림({preopen_mm}mm)")

        # perception_node가 /dev/gripper_cam을 쥐고 있어 넘겨받아야 한다
        # (grasp_test_console.py 3단계와 같은 이유). 끝나면 되살린다.
        subprocess.run(["pkill", "-f", "grippers_perception/perception_node"],
                       stdin=subprocess.DEVNULL)
        perception_was_killed = True
        time.sleep(1.0)
        cam = GripperCam()
        print(f"  그리퍼캠 스트림: {start_stream_server(cam)}")

        # 3단계 -------------------------------------------------------
        area = approach_until_area(node, cam, log)
        if area is None:
            node.move_floor_pose(profile, "recover_idle")
            return 4

        # 4단계 -------------------------------------------------------
        print("\n[4단계] 파지 후 CARRY_IDLE 복귀")
        resp = node.set_gripper(close_width_mm)
        if resp is None or not resp.ok:
            print("  그리퍼 닫기 실패")
            log.log("close", ok=False)
            node.move_floor_pose(profile, "recover_idle")
            return 5
        print(f"  닫힘({close_width_mm}mm). load_ratio={resp.load_ratio:.4f} (기준 {LOAD_THRESHOLD})")
        log.log("close", ok=True, load_ratio=resp.load_ratio)
        if resp.load_ratio < LOAD_THRESHOLD:
            print("  [경고] 닫힘 부하가 기준 미만 — 빈 채로 닫혔을 수 있습니다")

        for stage in ("midpoint", "safe", "idle"):
            if not node.move_floor_pose(profile, stage):
                print(f"  {stage} 실패 — arm.log 확인")
                log.log("return_failed", stage=stage)
                node.move_floor_pose(profile, "recover_idle")
                return 6
        load = node.get_load()
        print(f"  CARRY_IDLE 도달. load_ratio={load:.4f}" if load is not None else "  CARRY_IDLE 도달")
        log.log("carry_idle", ok=True, load_ratio=load)
        if load is not None and load < LOAD_THRESHOLD:
            print("  [경고] 복귀 후 부하가 기준 미만 — 운반 중 놓쳤을 수 있습니다")
        print(f"\n완료 — 물체를 물고 CARRY_IDLE에 있습니다(그리퍼 {close_width_mm}mm).")
        return 0

    except KeyboardInterrupt:
        print("\n[중단] 정지 명령 발행 중...")
        node.stop()
        print("정지 완료. 팔 상태는 직접 확인하세요(자동 복구 없음).")
        log.log("aborted")
        return 130
    finally:
        log.log("run_end")
        log.close()
        print(f"\n분석용 로그: {log.path}")
        if cam is not None:
            cam.close()
        if perception_was_killed:
            restart_perception_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
