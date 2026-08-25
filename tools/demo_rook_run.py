#!/usr/bin/env python3
"""시연용 1회 완주 — 수동 APPROACH -> GRASP -> 바구니 투하 -> IDLE.

2026-08-24 사용자 지시(시연 영상 촬영용): 물체 하나(기본 rook)를 대상으로
사람이 키보드로 몰고, 파지와 투하만 자동으로 한다.

    [1] APPROACH   space=전진, s=후진, a/d=제자리 회전, x=정지, c=단계 종료
    [2] g          팔 내리기 — safe -> 그리퍼 열기 -> grasp
    [3] 미세 전진  space=전진, s=후진, x=정지, c=단계 종료
        g          파지 -> midpoint -> safe -> CARRY_IDLE
    [4] 운반       w=전진, s=후진, a/d=제자리 회전, x=정지, c=단계 종료
    [5] g          바구니 투하 후 IDLE 복귀

x와 c를 나눠 둔 것이 요점이다(사용자 지시, 2026-08-24). x는 그 자리에
멈추기만 하고 단계 안에 머무르므로 다시 몰 수 있고, c는 그 단계를 끝낸다 —
"멈춰서 눈으로 확인한 뒤 조금 더 간다"가 시연에서 제일 자주 하는 동작이다.

자동 정렬·자동 전진이 없다는 점에서 auto_grasp_sequence.py와 다르고, 차를
움직인다는 점에서 grasp_cycle.py와 다르다. 이 도구의 목적은 데이터 수집이
아니라 **한 번에 끊김 없이 끝까지 가는 그림**을 찍는 것이다 — 그래서 중간에
Enter로 멈춰 세우는 확인 절차를 두지 않고, 대신 사람이 c로 멈춘 자리에서
g를 누를 때까지 기다린다.

그리퍼 캠은 쓰지 않는다. 열려면 perception_node를 죽여야 하는데(장치를
독점한다 — grasp_test_console.GripperCam 참고), 그러면 APPROACH 중에 거리를
읽어 주는 관측이 함께 죽는다. 시연에서는 거리 표시가 훨씬 쓸모 있고, 파지
성공 여부는 어차피 load 쪽이 신뢰할 수 있는 신호다 — 2026-08-24 6종 수집에서
그리퍼캠 면적은 빈 그리퍼(닫힘 165990px²)가 룩을 문 상태(70384px²)보다 커서
파지 판정에 쓸 수 없다는 것이 확인됐다.

사전 준비: depth_camera · depth_cam_rotate_node · perception_node ·
arm_driver · odom_publisher(controller)가 떠 있어야 한다. 팔은 IDLE에서
시작해야 한다(안 그러면 safe 이동이 거부된다).
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions

from grasp_test_console import (
    CLASS_TO_PROFILE,
    GRIPPER_CLOSED_MM,
    FINE_SPEED_MPS,
    GraspTestNode,
    KeyReader,
    RunLog,
    drive_phase,
    estimate_position,
    odom_distance_m,
    recover_to_idle,
)
from grippers_arm.floor_grasp_profiles import FLOOR_GRASP_PROFILES, GRIPPER_RELEASE_MM

# 제자리 회전 각속도(사용자 지시, 2026-08-24: "제자리에서 0.3으로").
#
# ⚠️ 이 값을 더 낮추면 안 된다. 순수 제자리 회전에는 정지마찰 문턱이 있어
# 명령이 나가도 바퀴가 안 도는 구간이 있는데, 2026-08-24 제자리 회전
# 시험에서 1.2부터 0.3까지는 전부 실제로 돌았고 사용자가 0.3~0.4를
# 적당하다고 판단했다. 즉 0.3은 실측으로 확인된 하한 근처다. 그리고
# /odom_raw는 명령을 적분할 뿐이라 안 돌아도 돌았다고 보고하므로, 안 도는
# 것을 로그로는 알아챌 수 없다.
#
# odom_publisher_node가 cmd_vel의 angular.z를 ±0.5로 자르지만 0.3은 그
# 아래라 그대로 나간다 — 회전만 따로 controller/cmd_vel로 우회할 필요가 없다.
TURN_IN_PLACE_RAD_S = 0.3

# grasp_test_console의 SPACE_KEYMAP/WASD_KEYMAP을 그대로 쓰지 않는다. 그쪽
# a/d는 "전진 + 약한 회전 바이어스"(TURN_BIAS_RAD_S=0.15)로, 보정 주행에서
# 곡선을 그리려고 일부러 그렇게 만든 것이다. 시연에서는 제자리에서 방향만
# 맞추고 직선으로 들어가는 편이 자연스럽고 거리 판단도 쉽다.
APPROACH_KEYMAP = {
    " ": lambda v: (v, 0.0),
    "s": lambda v: (-v, 0.0),
    "a": lambda v: (0.0, TURN_IN_PLACE_RAD_S),
    "d": lambda v: (0.0, -TURN_IN_PLACE_RAD_S),
    "x": lambda v: (0.0, 0.0),
}

CARRY_KEYMAP = {
    "w": lambda v: (v, 0.0),
    "s": lambda v: (-v, 0.0),
    "a": lambda v: (0.0, TURN_IN_PLACE_RAD_S),
    "d": lambda v: (0.0, -TURN_IN_PLACE_RAD_S),
    "x": lambda v: (0.0, 0.0),
}

# 팔이 바닥 높이까지 내려간 뒤의 미세 전진(사용자 지시, 2026-08-24).
#
# ⚠️ 회전 키를 **일부러** 빼 놓았다. 이 단계에서는 그리퍼가 바닥에서 2.6cm
# 위에 열린 채 떠 있는데, 제자리 회전은 그 그리퍼를 바닥과 물체를 가로질러
# 옆으로 쓸고 간다 — 팔이 바닥 높이에서 옆으로 쓸리는 움직임은 절대 안 된다는
# 것이 이 프로젝트의 확립된 안전 규칙이다(arm_driver_node의
# RETURN_TO_IDLE_DEFERRED_JOINTS 주석 참고). 좌우 정렬은 팔을 내리기 전
# 1단계에서 끝내야 한다.
#
# 후진(s)은 남겨 둔다 — 너무 밀고 들어가 물체가 그리퍼 목에 끼었을 때
# 빠져나올 유일한 수단이고, 열린 턱 사이에서 뒤로 빠지는 것 자체는 물체를
# 건드리지 않는다.
CREEP_KEYMAP = {
    " ": lambda v: (v, 0.0),
    "s": lambda v: (-v, 0.0),
    "x": lambda v: (0.0, 0.0),
}

APPROACH_LEGEND = ("  [space]전진 [s]후진 [a]좌회전 [d]우회전(제자리) "
                   "[x]정지 [c]단계 종료")
CARRY_LEGEND = ("  [w]전진 [s]후진 [a]좌회전 [d]우회전(제자리) "
                "[x]정지 [c]단계 종료")
CREEP_LEGEND = ("  [space]전진 [s]후진 [x]정지 [c]단계 종료 "
                "— 팔이 내려가 있어 회전 키는 없습니다")

# 시각 파지 확인 — "그 자리에 있던 것이 사라졌는가"
#
# 원리: 파지에 성공했으면 물체는 바닥에서 사라져 그리퍼에 있다. 실패했으면
# 여전히 바닥에 있다. 그리퍼캠으로 "손끝에 물려 있는가"를 보려던 방식은
# 실측으로 무효였다(빈 그리퍼 닫힘 165990px²가 룩을 문 상태 70384px²보다
# 컸다) — 물체가 있던 자리를 보는 쪽이 훨씬 다루기 쉬운 신호다.
#
# CARRY_IDLE에서 팔이 depth 카메라를 가리지 않는다는 것은 2026-08-25 실기로
# 확인했다: 팔은 프레임 밖이고 바닥이 그대로 보인다.
#
# 두 관측 사이에 로봇은 미세 전진으로 몇 cm 앞으로 간다 — 물체가 그대로라면
# 더 **가까워져** bbox 높이가 오히려 커진다. 그래서 "아직 있다"는
# h_after >= h_before * 이 비율로 잡는다. 1.0이 아니라 0.8인 것은 관측 잡음과
# 살짝 밀린 경우까지 실패로 보기 위해서다 — 이 방향의 오판(성공인데 실패라고
# 함)은 사람이 눈으로 확인하게 만들 뿐이지만, 반대 방향은 빈 그리퍼로 미션을
# 계속하게 한다.
#
# ⚠️ 이것만으로 성공을 단정하면 안 된다. 내려오는 그리퍼가 물체를 쳐서 시야
# 밖으로 밀어낸 경우에도 "사라짐"으로 보인다. load와 **독립적인 두 번째
# 근거**로 쓴다 — load는 "무언가를 쥐었다"를, 이쪽은 "목표가 그 자리에서
# 없어졌다"를 말하므로 실패 양상이 서로 겹치지 않는다.
STILL_THERE_H_RATIO = 0.8


def remember_target(node, raw_cls, log):
    """내려가기 직전의 기준 관측. 실패하면 None — 확인 단계가 판정을 접는다."""
    obs = node.observe(raw_cls, timeout_sec=1.5)
    if obs is None or not obs.found or obs.h <= 0.0:
        print("  [시각확인] 기준 관측 실패 — CARRY_IDLE 확인을 건너뜁니다")
        log.log("remember_target", found=False)
        return None
    print(f"  [시각확인] 기준 관측: {raw_cls} h={obs.h:.1f}px x={obs.x:.1f}px")
    log.log("remember_target", found=True, h=float(obs.h), x=float(obs.x))
    return float(obs.h)


def confirm_by_absence(node, raw_cls, h_before, log):
    """CARRY_IDLE에서 정면을 다시 본다. True=사라짐(성공 쪽), False=아직 있음."""
    if h_before is None:
        return None
    obs = node.observe(raw_cls, timeout_sec=1.5)
    if obs is None:
        print("  [시각확인] 관측 응답 없음 — 판정 불가")
        log.log("confirm_by_absence", result="no_response")
        return None
    if not obs.found:
        print(f"  [시각확인] ★ {raw_cls}가 정면에서 사라졌습니다 (기준 h={h_before:.1f}px)")
        log.log("confirm_by_absence", result="gone", h_before=h_before)
        return True
    still = obs.h >= h_before * STILL_THERE_H_RATIO
    print(f"  [시각확인] {raw_cls} h={obs.h:.1f}px (기준 {h_before:.1f}px) → "
          + ("⚠️ 아직 그 자리에 있습니다" if still else "멀리 있는 다른 개체로 보입니다"))
    log.log("confirm_by_absence", result="still_there" if still else "other_instance",
            h_before=h_before, h_after=float(obs.h))
    return not still


# 닫힘/이동 뒤 load가 정착할 때까지의 여유(grasp_cycle.LOAD_SETTLE_S와 동일 근거).
LOAD_SETTLE_S = 1.2

# 빈 그리퍼로 닫았을 때의 load. 2026-08-24 --empty 기준선 실측값이다.
# 파지 판정은 이 값과의 차이로 한다 — load는 4/1023 = 0.00391 단위로
# 양자화돼 있어 한 단위 차이는 잡음과 구분이 안 되므로 두 단위를 요구한다.
EMPTY_CARRY_LOAD = 0.0352
LOAD_MARGIN = 0.0078

# APPROACH를 멈출 거리. 팔이 그 자리에서 바로 잡을 수 있었던 **실측** 배치
# 거리다 — 2026-08-24 grasp_cycle 성공 기록의 rook 16.0cm와 18.6cm.
# 계산으로 고른 값이 아니라서, 다른 클래스에는 그대로 적용되지 않는다.
STOP_BAND_M = {"rook": (0.155, 0.190)}
DEFAULT_STOP_BAND_M = (0.150, 0.200)
LATERAL_OK_M = 0.02


def stop_band(raw_cls):
    return STOP_BAND_M.get(raw_cls, DEFAULT_STOP_BAND_M)


def measure_load(node, label, log):
    time.sleep(LOAD_SETTLE_S)
    load = node.get_load()
    if load is None:
        print(f"  [{label}] load: 읽기 실패")
    else:
        print(f"  [{label}] load_ratio: {load:.4f}")
    log.log("load", where=label, load_ratio=load)
    return load


def approach_report(node, raw_cls, log, band):
    """APPROACH 중 1초에 한 번, 지금 멈춰야 하는지를 알려준다.

    사용자의 보정 방식 선호(WASD로 사람이 몰고, 멈출 조건은 실시간으로
    알려준다)를 그대로 따른다 — 여기서 자동으로 브레이크를 잡지는 않는다.
    시연 중 자동 개입은 영상에서 무슨 일이 일어났는지 알아보기 어렵게 만든다."""
    lo, hi = band

    def report():
        obs = node.observe(raw_cls, timeout_sec=0.6)
        if obs is None or not obs.found:
            print("    [관측] 물체 안 보임")
            log.log("approach_sample", found=False)
            return
        forward_m, lateral_m = estimate_position(obs, raw_cls)
        if forward_m is None:
            print(f"    [관측] x={obs.x:.0f} (거리 보정값 없음)")
            log.log("approach_sample", found=True, x=obs.x, forward_m=None)
            return
        if forward_m < lo:
            verdict = "⚠️ 너무 가까움 — 후진"
        elif forward_m > hi:
            verdict = "계속 전진"
        elif abs(lateral_m) > LATERAL_OK_M:
            verdict = f"거리 OK, 좌우 {lateral_m * 100:+.1f}cm — {'d' if lateral_m > 0 else 'a'}로 보정"
        else:
            verdict = "★ 지금 c로 정지"
        print(f"    [관측] 전방 {forward_m * 100:5.1f}cm · 좌우 {lateral_m * 100:+5.1f}cm  → {verdict}")
        log.log("approach_sample", found=True, x=obs.x,
                forward_m=forward_m, lateral_m=lateral_m)

    return report


def wait_for_key(kr, key, prompt):
    """`key`가 눌릴 때까지 기다린다. q면 KeyboardInterrupt로 중단."""
    kr.ensure_cbreak()
    print(prompt)
    while True:
        pressed = kr.getch_nonblocking()
        if pressed is None:
            time.sleep(0.05)
            continue
        pressed = pressed.lower()
        if pressed == key:
            return
        if pressed == "q":
            raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raw-cls", default="rook", choices=sorted(CLASS_TO_PROFILE))
    ap.add_argument("--profile", default=None)
    args = ap.parse_args()

    raw_cls = args.raw_cls
    profile = args.profile or CLASS_TO_PROFILE[raw_cls]
    close_width_mm = FLOOR_GRASP_PROFILES[profile].close_width_mm
    preopen_mm = FLOOR_GRASP_PROFILES[profile].preopen_width_mm
    release_mm = FLOOR_GRASP_PROFILES[profile].release_width_mm
    band = stop_band(raw_cls)

    log = RunLog(raw_cls, profile)
    print(f"=== 시연 1회 — {raw_cls} ===")
    print(f"profile={profile}  preopen={preopen_mm}mm  close={close_width_mm}mm")
    print(f"목표 정지 거리: 전방 {band[0] * 100:.0f}~{band[1] * 100:.0f}cm (실측 성공 범위)")
    print(f"상세 로그: {log.path}")
    print("⚠️ 팔이 IDLE에 있어야 시작할 수 있습니다. 언제든 q로 중단(주행은 즉시 정지).")

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GraspTestNode()
    try:
        with KeyReader() as kr:
            # --- [1] APPROACH ------------------------------------------
            print("\n[1] APPROACH — 물체 앞까지 몰고 가세요")
            start, end = drive_phase(
                node, kr, keymap=APPROACH_KEYMAP, speed=FINE_SPEED_MPS,
                legend=APPROACH_LEGEND,
                report=approach_report(node, raw_cls, log, band),
            )
            travelled = odom_distance_m(start, end)
            if travelled is not None:
                print(f"  정지 — odom 이동 {travelled:.3f}m "
                      "(명령 적분값이라 실제 이동과 다를 수 있습니다)")
            log.log("approach_done", odom_m=travelled)

            # --- [2] 팔 내리기 -------------------------------------------
            wait_for_key(kr, "g", "\n[2] 준비되면 [g] — 팔을 내립니다")
            # 지금이 정면을 볼 수 있는 마지막 순간이다 — grasp 자세로 내려가면
            # 팔이 depth 카메라를 가린다.
            h_before = remember_target(node, raw_cls, log)
            print("  safe → 그리퍼 열기 → grasp")
            if not node.move_floor_pose(profile, "safe"):
                recover_to_idle(node, profile, log, "safe 이동 실패")
                return 2
            # 내려가기 전에 연다 — 닫힌 손가락이 물체가 있는 공간을 통과해
            # 내려가면 물체를 밀어낸다(사용자 지시, 2026-08-24).
            node.set_gripper(preopen_mm)
            print(f"  그리퍼 열림({preopen_mm}mm) — 내려가기 전")
            if not node.move_floor_pose(profile, "grasp"):
                recover_to_idle(node, profile, log, "grasp 이동 실패")
                return 2

            # --- [3] 미세 전진 -------------------------------------------
            # 사용자 지시(2026-08-24): 닫기 전에 사람이 조금씩 밀어 넣는다.
            # 팔을 내린 뒤에는 depth 카메라가 팔에 가려 거리 표시가 없다 —
            # 눈으로 보고 판단해야 한다.
            print("\n[3] 물체가 그리퍼 안에 들어오도록 전진하세요")
            start, end = drive_phase(
                node, kr, keymap=CREEP_KEYMAP, speed=FINE_SPEED_MPS,
                legend=CREEP_LEGEND)
            crept = odom_distance_m(start, end)
            if crept is not None:
                print(f"  정지 — odom 이동 {crept:.3f}m")
            log.log("creep_done", odom_m=crept)

            wait_for_key(kr, "g", "  [g] — 파지하고 CARRY_IDLE까지 갑니다")
            print("  그리퍼 닫기")
            resp = node.set_gripper(close_width_mm)
            if resp is None or not resp.ok:
                recover_to_idle(node, profile, log, "그리퍼 닫기 실패")
                return 3
            measure_load(node, "닫힘", log)

            # 바닥에서 IDLE로 곧장 가면 그리퍼가 바닥을 쓸어간다 — 검증된
            # 상승 체인(midpoint → safe → idle)을 그대로 밟는다.
            print("  midpoint → safe → idle (CARRY_IDLE)")
            for stage in ("midpoint", "safe", "idle"):
                if not node.move_floor_pose(profile, stage):
                    recover_to_idle(node, profile, log, f"{stage} 이동 실패")
                    return 4
            carry_load = measure_load(node, "CARRY_IDLE", log)

            if carry_load is None:
                print("  판정 불가 — load를 못 읽었습니다")
            elif carry_load - EMPTY_CARRY_LOAD > LOAD_MARGIN:
                print(f"  ★ 파지 성공 — 빈 상태보다 {carry_load - EMPTY_CARRY_LOAD:+.4f} "
                      f"({(carry_load - EMPTY_CARRY_LOAD) / 0.00391:+.1f}단위)")
            else:
                print(f"  ⚠️ 빈 상태와 구분이 안 됩니다(load {carry_load:.4f}) — "
                      "물체를 놓쳤을 수 있습니다. 계속할지 눈으로 확인하세요")
            log.log("grasp_verdict", carry_load=carry_load, empty=EMPTY_CARRY_LOAD)

            # load와 독립적인 두 번째 근거. 둘이 엇갈리면 사람이 눈으로 가른다.
            vanished = confirm_by_absence(node, raw_cls, h_before, log)
            if carry_load is not None and vanished is not None:
                load_ok = carry_load - EMPTY_CARRY_LOAD > LOAD_MARGIN
                if load_ok and vanished:
                    print("  ★★ 두 신호 모두 성공 — load 있음 + 물체 사라짐")
                elif not load_ok and not vanished:
                    print("  ⚠️⚠️ 두 신호 모두 실패 — load 없음 + 물체 그대로")
                elif load_ok and not vanished:
                    print("  ❓ 엇갈림: load는 잡혔다는데 물체가 그 자리에 있습니다 —\n"
                          "     다른 것을 쥐었거나 같은 클래스가 하나 더 있을 수 있습니다")
                else:
                    print("  ❓ 엇갈림: 물체는 사라졌는데 load가 없습니다 —\n"
                          "     그리퍼가 물체를 쳐서 밀어냈을 수 있습니다")
                log.log("combined_verdict", load_ok=load_ok, vanished=vanished)

            # --- [4] 운반 ----------------------------------------------
            print("\n[4] 바구니 앞까지 운반하세요")
            start, end = drive_phase(
                node, kr, keymap=CARRY_KEYMAP, speed=FINE_SPEED_MPS,
                legend=CARRY_LEGEND)
            travelled = odom_distance_m(start, end)
            if travelled is not None:
                print(f"  정지 — odom 이동 {travelled:.3f}m")
            log.log("carry_done", odom_m=travelled)

            # --- [5] 투하 ----------------------------------------------
            wait_for_key(kr, "g", "\n[5] 바구니가 팔 아래 오면 [g] — 투하 후 IDLE 복귀")
            if not node.move_floor_pose(profile, "drop"):
                recover_to_idle(node, profile, log, "drop 이동 실패")
                return 5
            measure_load(node, "투하 직전", log)
            # 활짝 열지 않는다(사용자 지시, 2026-08-25) — 물체가 턱 사이에서
            # 빠져나올 만큼만 벌린다. 손가락 판이 바구니 위로 넓게 쓸리는 것을
            # 막는다.
            print(f"  그리퍼 열기 {release_mm}mm (투하용 — 물체 폭 +{GRIPPER_RELEASE_MM}mm)")
            node.set_gripper(release_mm)
            measure_load(node, "놓은 뒤", log)
            # IDLE로 접기 **전에** 닫는다(사용자 지시, 2026-08-25). 닫힌
            # 그리퍼가 접기에 알맞은 형상이고, "내려가기 전에 연다"와 같은
            # 원칙이다 — 다음 동작이 요구하는 형상을 그 동작 전에 만든다.
            print("  그리퍼 닫기 — IDLE 복귀 전")
            node.set_gripper(GRIPPER_CLOSED_MM)
            if not node.move_floor_pose(profile, "idle"):
                recover_to_idle(node, profile, log, "투하 후 idle 복귀 실패")
                return 5

        print("\n완료 — IDLE 복귀")
        log.log("run_ok")
        return 0

    except KeyboardInterrupt:
        print("\n[중단] 주행을 멈춥니다. 팔 상태는 직접 확인하세요(자동 복구 없음).")
        log.log("aborted")
        return 130
    finally:
        # 어떤 경로로 끝나든 바퀴부터 세운다 — 팔은 자세 게이트가 지켜 주지만
        # cmd_vel은 마지막 값이 그대로 유지된다.
        node.stop()
        log.log("run_end")
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        print(f"상세 로그: {log.path}")


if __name__ == "__main__":
    sys.exit(main())
