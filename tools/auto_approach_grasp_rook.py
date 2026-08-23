#!/usr/bin/env python3
"""룩 전용 완전 자동 접근+파지 실기 테스트 — 키 입력 없이 끝까지 진행한다.

`grasp_test_console.py`(대화식, 사람이 매 단계 키를 누름)의 다음 실험 —
같은 순서를 이번엔 사람 개입 없이 닫힌 루프로 자동 수행한다:

    1) `perception/observe_target`으로 룩을 계속 관측하며, **직진+좌우회전
       결합 이동만으로**(오늘 실기로 확인된 순수 제자리회전 고장 회피 — 아래
       "왜 pure rotation을 안 쓰는가" 참고) 그리퍼 정렬 x(170.1px)·약 40cm 지점으로
       수렴시킨다.
    2) GRASP 진입(safe→grasp, 그리퍼 예열림) — `grasp_test_console.py`와
       동일한 절차·동일한 이유로 여기서도 `perception_node`를 죽이고
       그리퍼캠을 직접 연다.
    3) 약 20cm 전진(오차 허용) — 매 스텝 그리퍼캠 컨투어 면적을 재서
       기준치(오늘 룩 실측값, GRASP_AREA_THRESHOLD_PX2) 이상이 되는
       시점에 정지한다. 안전 상한(MAX_FINE_ADVANCE_M)을 넘기면 포기하고
       보고한다(계속 밀어붙이지 않는다 — 오늘 세션에 실제로 물체를 밀어버린
       적이 있다).
    4) 파지(닫기)→load 확인→midpoint 들어올리기→load 재확인→CARRY_IDLE.
       바구니 투하·WASD 자유주행은 안 한다 — CARRY_IDLE 도달로 끝난다.

**왜 pure rotation을 안 쓰는가**: 2026-08-23 실기에서 제자리 회전만
단독으로 냈을 때(0.3~0.6 rad/s) 모터가 소리만 내고 실제로 전혀 안
돌아갔다(정지마찰 추정, 원인 미해결). `visual_approach_control.
compute_approach_command`는 거리 오차가 이미 허용치 안이면 `vx=0`으로
두고 회전만 낼 수 있는 설계라 이 상황에 걸릴 위험이 있다 — 그래서 여기서는
그 함수를 그대로 쓰지 않고, **회전이 걸릴 때는 항상 최소 전진 성분을
강제로 얹는** 별도의 단순 제어를 쓴다. `compute_approach_error`(오차 계산)
만 재사용한다.

⚠️ 이 자동 접근 게인(APPROACH_GAIN_TURN 등)은 오늘 처음 자동으로 도는
것이라 실기 미검증 자리 표시자다 — 사람이 옆에서 지켜보다가 필요하면
아무 때나 `q`+Enter 또는 Ctrl+C로 즉시 정지시킬 수 있게 했다.

실행 (grasp_test_console.py와 동일한 노드 구성 필요 — perception_node는
이 스크립트가 3단계 진입 시 알아서 죽이므로, 시작 전엔 켜져 있어야 한다):

    scp tools/auto_approach_grasp_rook.py pi@10.82.133.189:/home/pi/docker/shared/grippers/tools/auto_approach_grasp_rook.py

컨테이너 안(zsh)에서, perception_node가 안 떠 있으면 먼저:

    ros2 run grippers_perception perception_node > /tmp/perception.log 2>&1 &
    sleep 2

그 다음:

    python3 /grippers/tools/auto_approach_grasp_rook.py
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist

sys.path.insert(0, "/grippers/tools")
from grasp_test_console import (  # noqa: E402
    FLOOR_GRASP_PROFILES,
    GRIPPER_CLOSED_MM,
    LOAD_THRESHOLD,
    CLASS_TO_PROFILE,
    GRASP_AREA_THRESHOLD_PX2,
    GraspTestNode,
    GripperCam,
    KeyReader,
    RunLog,
)
from grippers_base.visual_approach_control import compute_approach_error  # noqa: E402

# --- 자동 접근(1단계) 목표·게인 — 전부 실기 미검증 자리 표시자 -------------
# 2026-08-23 실기 3연속: 화면 "정가운데"(320)는 그리퍼 기준 너무 오른쪽,
# 옛 교시값(170.1)은 반대로 너무 왼쪽이었다(사용자 직접 관찰). 사용자 지시로
# "화면 정가운데에서 왼쪽으로 10px만" — 카메라 프레임은 640폭(camera_info
# 실측 확인)이라 정가운데=320, 그 왼쪽 10px.
TARGET_X_PX = 310.0
# 목표 거리 40cm. 오늘 네 지점 실측(h*z_m가 전부 ~50.0으로 일정 — h는
# 선형 치수라 거리에 반비례하므로 h*z_m=상수가 맞다)으로 회귀:
#   (h=222.26,z=0.230) (h=196.4,z=0.255) (h=146.25,z=0.339) (h=71.68,z=0.689)
#   → h*z_m ≈ 49.4~51.1, 평균 50.0 → target_h = 50.0/0.40 ≈ 125
TARGET_H_PX = 125.0
APPROACH_TOL_X_PX = 15.0
APPROACH_TOL_H_PX = 10.0
APPROACH_GAIN_TURN = 0.0015
APPROACH_GAIN_H = 0.0016
APPROACH_MAX_TURN = 0.15  # 오늘 수동 a/d 테스트에서 쓴 값과 동일
APPROACH_MAX_SPEED = 0.06
APPROACH_MIN_SPEED = 0.05  # 데드밴드 경계 — 이 아래 속도는 안 움직인다(오늘 실기 확인)
APPROACH_BURST_S = 0.4
APPROACH_SETTLE_S = 0.3
MAX_APPROACH_ITERS = 50
MAX_CONSECUTIVE_MISSES = 10  # 이만큼 연속으로 못 찾으면 포기

FINE_SPEED_MPS = 0.05
FINE_BURST_S = 0.4
FINE_SETTLE_S = 0.3
# 2026-08-23 실기 3연속 모두 load=0.0352(파지 기준 0.04 미달, 게다가 세 번
# 다 정확히 같은 값 — 우연이라기보단 매번 같은 방식으로 살짝 헛집었다는
# 뜻일 가능성이 크다)로 파지가 부실했다. 사용자가 직접 보고 "아직도 멀리
# 떨어져있다"고 지시 — 기존 20cm 목표를 40cm로 올린다(이론상 과잉전진
# 우려가 있었지만, 실물을 보고 있는 사용자 판단을 우선한다).
MIN_FINE_ADVANCE_M = 0.30  # 이만큼 가기 전엔 면적 기준으로 조기 정지하지 않는다
TARGET_FINE_ADVANCE_M = 0.40  # 최소량을 넘긴 뒤에도 기준 미달이면 여기서 무조건 정지
MAX_FINE_ADVANCE_M = 0.50  # 안전 상한 — 넘기면 포기(계속 밀어붙이지 않는다)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def auto_approach(node: GraspTestNode, kr: KeyReader, raw_cls: str, log: RunLog) -> bool:
    """직진+좌우회전 결합만으로 그리퍼 정렬 x·목표 거리(40cm)까지 수렴시킨다.
    수렴하면 True, 실패(연속 미검출/반복 초과/사용자 중단)하면 False."""
    misses = 0
    for i in range(MAX_APPROACH_ITERS):
        if kr.getch_nonblocking() == "q":
            print("  [중단] 사용자 요청")
            node.stop()
            return False
        node.pump()
        obs = node.observe(raw_cls)
        if obs is None or not obs.found:
            misses += 1
            print(f"  [{i}] 물체 못 찾음 ({misses}/{MAX_CONSECUTIVE_MISSES})")
            if misses >= MAX_CONSECUTIVE_MISSES:
                print("  [실패] 연속 미검출 상한 도달")
                return False
            node.stop()
            time.sleep(APPROACH_SETTLE_S)
            continue
        misses = 0

        err_x, err_h = compute_approach_error(obs.x, obs.h, TARGET_X_PX, TARGET_H_PX)
        done_x = abs(err_x) <= APPROACH_TOL_X_PX
        done_h = abs(err_h) <= APPROACH_TOL_H_PX
        print(f"  [{i}] x={obs.x:.1f} h={obs.h:.1f} err_x={err_x:+.1f} err_h={err_h:+.1f}")
        log.log("auto_approach_step", i=i, x=obs.x, h=obs.h, err_x=err_x, err_h=err_h)

        if done_x and done_h:
            node.stop()
            print("  수렴 완료(그리퍼 정렬 x·목표 거리 도달)")
            log.log("auto_approach_done", iters=i)
            return True

        wz = 0.0 if done_x else _clamp(-err_x * APPROACH_GAIN_TURN, -APPROACH_MAX_TURN, APPROACH_MAX_TURN)
        vx = 0.0 if done_h else _clamp(err_h * APPROACH_GAIN_H, -APPROACH_MAX_SPEED, APPROACH_MAX_SPEED)

        # align_first — 좌우가 크게 어긋난 채로 전진하면 원근 때문에 픽셀
        # 오차가 더 벌어진다(HANDOFF.md "전진과 좌우 보정은 결합돼 있다").
        # 1차 실기(x=320 목표)는 오른쪽으로, 2차(x=170.1 목표)는 왼쪽으로
        # 엇갈리게 못 맞춘 게 이 결합 때문일 가능성이 커서, 좌우가 아직 크게
        # 어긋난 동안은 전진을 줄여 회전이 먼저 따라잡게 한다.
        if abs(err_x) > 2.0 * APPROACH_TOL_X_PX:
            vx *= 0.25

        # 순수 회전 금지 — wz를 낼 거면 vx도 최소 데드밴드 이상으로 강제한다
        # (오늘 실기: 순수 회전은 모터가 소리만 내고 안 움직였다).
        if wz != 0.0 and abs(vx) < APPROACH_MIN_SPEED:
            vx = APPROACH_MIN_SPEED if err_h >= 0 else -APPROACH_MIN_SPEED
        elif vx != 0.0:
            vx = (APPROACH_MIN_SPEED if abs(vx) < APPROACH_MIN_SPEED else abs(vx)) * (1 if vx > 0 else -1)

        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        node.cmd_pub.publish(t)
        time.sleep(APPROACH_BURST_S)
        node.stop()
        time.sleep(APPROACH_SETTLE_S)

    print("  [실패] 반복 상한 도달 — 수렴 못 함")
    return False


def fine_approach_and_close(
    node: GraspTestNode, kr: KeyReader, cam: GripperCam, area_threshold: float | None, log: RunLog,
) -> bool:
    """그리퍼캠 면적이 기준치를 넘을 때까지 소량씩 전진한다. 넘으면 True."""
    node.pump()
    start_pose = node._pose
    while True:
        if kr.getch_nonblocking() == "q":
            print("  [중단] 사용자 요청")
            node.stop()
            return False
        node.pump()
        moved = None
        if start_pose is not None and node._pose is not None:
            dx = node._pose[0] - start_pose[0]
            dy = node._pose[1] - start_pose[1]
            moved = (dx * dx + dy * dy) ** 0.5

        area = cam.measure_area_px2()
        area_str = f"{area:.0f}px²" if area is not None else "검출 안 됨"
        moved_str = f"{moved*100:.1f}cm" if moved is not None else "?"
        print(f"    면적: {area_str}  누적 전진: {moved_str}")
        log.log("fine_approach_step", area_px2=area, moved_m=moved)

        past_min = moved is not None and moved >= MIN_FINE_ADVANCE_M
        if past_min and area_threshold is not None and area is not None and area >= area_threshold:
            node.stop()
            print(f"  기준치({area_threshold:.0f}px²) 도달({moved*100:.1f}cm 전진 후) — 정지")
            return True

        if moved is not None and moved >= TARGET_FINE_ADVANCE_M:
            node.stop()
            print(f"  목표 전진량({TARGET_FINE_ADVANCE_M*100:.0f}cm) 도달 — 기준 면적 미달이어도 정지")
            return True

        if moved is not None and moved >= MAX_FINE_ADVANCE_M:
            node.stop()
            print(f"  [실패] 안전 상한({MAX_FINE_ADVANCE_M*100:.0f}cm) 도달 — 기준치 미달, 포기")
            return False

        t = Twist()
        t.linear.x = FINE_SPEED_MPS
        node.cmd_pub.publish(t)
        time.sleep(FINE_BURST_S)
        node.stop()
        time.sleep(FINE_SETTLE_S)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raw-cls", default="rook", choices=sorted(CLASS_TO_PROFILE))
    args = ap.parse_args()
    profile = CLASS_TO_PROFILE[args.raw_cls]
    close_width_mm = FLOOR_GRASP_PROFILES[profile].close_width_mm
    preopen_mm = FLOOR_GRASP_PROFILES[profile].preopen_width_mm
    area_threshold = GRASP_AREA_THRESHOLD_PX2.get(args.raw_cls)

    log = RunLog(args.raw_cls, profile)
    print(f"대상: {args.raw_cls}/{profile}  분석용 로그: {log.path}")

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GraspTestNode()
    cam = None
    try:
        with KeyReader() as kr:
            kr.wait_enter("\n자동 시퀀스를 시작합니다. Enter로 진행 (q+Enter로 취소): ")

            print("\n[1/4] 자동 접근 — 직진+좌우회전 결합으로 그리퍼 정렬 x·약 40cm 수렴")
            if not auto_approach(node, kr, args.raw_cls, log):
                print("접근 실패 — 중단합니다.")
                return

            print("\n[2/4] GRASP 진입")
            if not (node.move_floor_pose(profile, "safe") and node.move_floor_pose(profile, "grasp")):
                print("GRASP 진입 실패 — arm.log 확인할 것")
                log.log("grasp_entry", ok=False)
                return
            node.set_gripper(preopen_mm)
            log.log("grasp_entry", ok=True, preopen_mm=preopen_mm)
            subprocess.run(["pkill", "-f", "grippers_perception/perception_node"])
            time.sleep(1.0)
            cam = GripperCam()

            print("\n[3/4] 미세 전진 — 그리퍼캠 면적 기준 도달까지")
            if not fine_approach_and_close(node, kr, cam, area_threshold, log):
                print("미세 전진 실패 — 중단합니다(팔은 grasp 자세에 그대로 있음, 수동 확인 요망).")
                return

            print("\n[4/4] 파지 → 들어올리기 → CARRY_IDLE")
            resp = node.set_gripper(close_width_mm)
            if resp is None or not resp.ok:
                print("그리퍼 닫기 실패")
                log.log("close", ok=False)
                return
            print(f"  닫힘. load_ratio={resp.load_ratio:.4f} (기준 {LOAD_THRESHOLD})")
            log.log("close", ok=True, load_ratio=resp.load_ratio)
            if not node.move_floor_pose(profile, "midpoint"):
                print("들어올리기 실패")
                log.log("midpoint", ok=False)
                return
            mid_load = node.get_load()
            print(f"  midpoint load_ratio={mid_load:.4f}" if mid_load is not None else "  load 확인 실패")
            log.log("midpoint", ok=True, load_ratio=mid_load)
            if mid_load is not None and mid_load < LOAD_THRESHOLD:
                print("  [경고] 부하가 기준 미만 — 파지 실패(미끄러짐) 가능성")

            ok = node.move_floor_pose(profile, "safe") and node.move_floor_pose(profile, "idle")
            log.log("carry_idle", ok=ok)
            print("\n완료 — CARRY_IDLE 도달." if ok else "\nCARRY_IDLE 복귀 실패 — 수동 확인할 것.")

    except KeyboardInterrupt:
        print("\n[중단] 정지 명령 발행 중...")
        node.stop()
        log.log("aborted")
    finally:
        log.log("run_end")
        log.close()
        print(f"\n분석용 로그: {log.path}")
        if cam is not None:
            cam.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
