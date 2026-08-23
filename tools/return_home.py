#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""팔을 검증된 경로로 IDLE 까지 되돌린다 — 조작자 승인형.

align_to_idle.py 는 **IDLE 근처의 미세 정렬용**이라 허용 편차가 800 카운트다.
파지 자세처럼 멀리 있으면(실측 편차 2296) 거부한다. 그건 옳은 설계다 —
어디 있는지 모르는 팔을 곧장 IDLE 로 끌어당기면 차체에 부딪힐 수 있다.

이 스크립트는 그 빈 자리를 메운다. **horizontal_grasp_hardware_test.py 가 쓰는
것과 같은 검증된 자세와 순서**만 사용한다:

    현재 → HORIZONTAL_SAFE_145 → IDLE_CRADLE

파지 자세에서 곧장 IDLE 로 가지 않는 이유가 여기 있다. 먼저 145mm 로 들어올려
바닥·물체에서 떨어뜨린 뒤에 접는다. 시험 스크립트도 같은 순서를 쓴다.

모든 물리적 이동 전에 Enter 를 요구한다. q 는 이동 없이 중단한다.
"""
from __future__ import annotations

import argparse
import time

import soarm_lab  # noqa: F401  — driver_sdk 를 flat import 가능하게 만든다
from driver_sdk import STS3215Driver
from grippers_arm.floor_grasp_profiles import (
    HORIZONTAL_SAFE_145_RAW,
    IDLE_CRADLE_RAW,
)
from grippers_arm.gripper_calibration import GRIPPER_CLOSED_MM, position_from_width

SERVO_IDS = range(1, 6)


def confirm(message: str) -> None:
    answer = input(f"\n{message}\nEnter=계속, q=중단 > ").strip().lower()
    if answer == "q":
        raise SystemExit("조작자가 중단했습니다. 팔은 현재 자세에 그대로 있습니다.")
    if answer != "":
        raise RuntimeError("Enter 또는 q만 입력하세요")


def read_arm(driver) -> dict:
    return {sid: driver.get_position(sid) for sid in SERVO_IDS}


def glide(driver, label: str, goal_raw, steps: int = 90, delay: float = 0.034) -> None:
    start = read_arm(driver)
    goal = {sid: goal_raw[sid - 1] for sid in SERVO_IDS}
    print(f"\n[{label}] start={start}")
    print(f"[{label}] goal={goal}")
    for i in range(1, steps + 1):
        r = i / steps
        for sid in SERVO_IDS:
            pos = round(start[sid] + r * (goal[sid] - start[sid]))
            if not driver.set_position(sid, pos):
                raise RuntimeError(f"servo {sid} 쓰기 실패 (step {i})")
        time.sleep(delay)
        if i % max(1, steps // 4) == 0:
            print(f"[{label}] step={i}/{steps} present={read_arm(driver)}")
    time.sleep(1.0)
    end = read_arm(driver)
    off = {sid: end[sid] - goal[sid] for sid in SERVO_IDS}
    print(f"[{label}] 도착 offsets={off}")


def main():
    ap = argparse.ArgumentParser(description="팔을 검증된 경로로 IDLE 까지 복귀")
    ap.add_argument("--port", default="/dev/soarm")
    ap.add_argument("--accel", type=int, default=None,
                    help="서보 1~5 가속도(0~254). 생략하면 건드리지 않는다")
    ap.add_argument("--skip-safe", action="store_true",
                    help="145mm 경유를 건너뛰고 곧장 IDLE 로. **팔이 이미 높이 있을 때만**")
    ap.add_argument("--close-gripper", action="store_true",
                    help="복귀 후 그리퍼를 IDLE 관례대로 닫는다")
    args = ap.parse_args()

    driver = STS3215Driver(args.port)
    if not driver.connect():
        raise SystemExit(f"{args.port} 열기 실패")

    if args.accel is not None:
        for sid in SERVO_IDS:
            driver.set_acceleration(sid, args.accel)
        print(f"[setup] servo 1~5 가속도={args.accel}")

    print(f"[start] arm={read_arm(driver)}")
    print(f"[start] temp={{{', '.join(f'{s}: {driver.get_temperature(s)}' for s in SERVO_IDS)}}}")

    if not args.skip_safe:
        confirm("팔 주변과 이동 경로가 비어 있습니다. 145mm 안전 자세로 들어올리기")
        glide(driver, "safe-145", HORIZONTAL_SAFE_145_RAW)

    confirm("팔이 바닥·물체에서 떨어졌습니다. IDLE_CRADLE 로 접기")
    glide(driver, "idle-cradle", IDLE_CRADLE_RAW)

    if args.close_gripper:
        confirm("그리퍼 주변이 비어 있습니다. IDLE 관례대로 그리퍼 닫기")
        driver.set_position(6, position_from_width(GRIPPER_CLOSED_MM))
        time.sleep(1.0)
        print(f"[gripper] present={driver.get_position(6)}")

    print(f"\n[complete] arm={read_arm(driver)}")
    print("복귀 완료")
    driver.disconnect()


if __name__ == "__main__":
    main()
