#!/usr/bin/env python3
"""Interactive, operator-gated SO-ARM101 horizontal grasp hardware test.

Every physical transition requires Enter.  Entering ``q`` stops before the
next transition; Ctrl-C also leaves the arm at its latest commanded waypoint.
Run only with the robot attended and the base stationary.
"""

import argparse
import time

from driver_sdk import STS3215Driver
from grippers_arm.floor_grasp_profiles import (
    FLOOR_GRASP_PROFILES,
    HORIZONTAL_GRASP_POSES_DEG,
    HORIZONTAL_SAFE_145_DEG,
)
from grippers_arm.gripper_calibration import GRIPPER_CLOSED_MM, position_from_width

SERVO_IDS = range(1, 6)
LOAD_MAX_RAW = 1023.0
MIN_HOLD_LOAD_RATIO = 0.04
SAFE_START_TOLERANCE_RAW = 120
RETRY_TIGHTEN_MM = 5.0
MAX_START_SERVO2_TEMP_C = 40


def confirm(message):
    """Wait for Enter before a transition; q aborts without moving."""
    answer = input(f"\n{message}\nEnter=계속, q=중단 > ").strip().lower()
    if answer == "q":
        raise KeyboardInterrupt("operator aborted before transition")
    if answer:
        raise RuntimeError("Enter 또는 q만 입력하세요")


def read_arm(driver):
    return {servo_id: driver.get_position(servo_id) for servo_id in SERVO_IDS}


def raw_goals(driver, angles_deg):
    return {
        servo_id: driver.degrees_to_position(angles_deg[servo_id - 1]) for servo_id in SERVO_IDS
    }


def near_pose(actual, expected, tolerance=SAFE_START_TOLERANCE_RAW):
    return all(abs(actual[i] - expected[i]) <= tolerance for i in SERVO_IDS)


def glide(driver, label, angles_deg, steps=30, delay=0.10):
    start = read_arm(driver)
    goal = raw_goals(driver, angles_deg)
    print(f"\n[{label}] start={start}")
    print(f"[{label}] goal={goal}")
    for step_index in range(1, steps + 1):
        ratio = step_index / steps
        waypoint = {
            servo_id: round(start[servo_id] + ratio * (goal[servo_id] - start[servo_id]))
            for servo_id in SERVO_IDS
        }
        for servo_id, position in waypoint.items():
            if not driver.set_position(servo_id, position):
                raise RuntimeError(f"servo {servo_id} write failed at step {step_index}")
        time.sleep(delay)
        if step_index % 5 == 0:
            print(f"[{label}] step={step_index}/{steps} present={read_arm(driver)}")
    time.sleep(1.0)


def set_width(driver, width_mm):
    goal = position_from_width(width_mm)
    if not driver.set_position(6, goal):
        raise RuntimeError("servo 6 position write failed")
    time.sleep(1.5)
    load_raw = driver.get_load(6)
    ratio = abs(load_raw) / LOAD_MAX_RAW
    print(
        f"gripper width_command={width_mm:.1f}mm goal={goal} "
        f"position={driver.get_position(6)} load_raw={load_raw} load_ratio={ratio:.4f}"
    )
    return ratio


def report(driver, label):
    loads = {servo_id: driver.get_load(servo_id) for servo_id in range(1, 7)}
    temperatures = {servo_id: driver.get_temperature(servo_id) for servo_id in range(1, 7)}
    print(f"\n[{label}] arm={read_arm(driver)} gripper={driver.get_position(6)}")
    print(f"[{label}] load={loads}")
    print(f"[{label}] temp={temperatures}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("profile", choices=sorted(HORIZONTAL_GRASP_POSES_DEG))
    parser.add_argument("--port", default="/dev/soarm")
    args = parser.parse_args()

    profile = FLOOR_GRASP_PROFILES[args.profile]
    grasp_pose = HORIZONTAL_GRASP_POSES_DEG[args.profile]
    safe_pose = HORIZONTAL_SAFE_145_DEG

    driver = STS3215Driver(args.port)
    driver.connect()

    servo2_temp = driver.get_temperature(2)
    if servo2_temp > MAX_START_SERVO2_TEMP_C:
        raise RuntimeError(
            f"servo 2 온도 {servo2_temp}°C가 시작 상한 "
            f"{MAX_START_SERVO2_TEMP_C}°C를 초과했습니다. 냉각 후 재시도하세요"
        )

    actual = read_arm(driver)
    safe_raw = raw_goals(driver, safe_pose)
    grasp_raw = raw_goals(driver, grasp_pose)
    if not (near_pose(actual, safe_raw) or near_pose(actual, grasp_raw)):
        raise RuntimeError(
            "시작 자세가 등록된 safe/grasp 자세와 다릅니다. 자동 이동하지 않습니다: "
            f"actual={actual} safe={safe_raw} grasp={grasp_raw}"
        )

    print(f"profile={args.profile} geometry={profile}")
    report(driver, "start")

    confirm("작업 공간과 베이스가 안전한지 확인했습니다. 145mm 안전 자세로 이동")
    glide(driver, "safe", safe_pose)

    confirm(f"그리퍼를 {profile.preopen_width_mm:.1f}mm로 열기")
    set_width(driver, profile.preopen_width_mm)

    confirm(f"물체를 치운 상태입니다. 파지 중심 {profile.grasp_center_height_mm:.1f}mm로 이동")
    glide(driver, "grasp", grasp_pose)

    confirm("물체를 두 손가락 중앙에 놓고 손을 완전히 뺐습니다. 그리퍼 닫기")
    ratio = set_width(driver, profile.close_width_mm)
    if ratio < MIN_HOLD_LOAD_RATIO:
        retry_width_mm = max(GRIPPER_CLOSED_MM, profile.close_width_mm - RETRY_TIGHTEN_MM)
        confirm(
            f"파지 부하 {ratio:.4f}가 임계값 {MIN_HOLD_LOAD_RATIO:.2f} 미만입니다. "
            f"물체가 중앙에 있고 손을 뺀 상태라면 {retry_width_mm:.1f}mm로 한 번 더 조이기"
        )
        ratio = set_width(driver, retry_width_mm)
        if ratio < MIN_HOLD_LOAD_RATIO:
            raise RuntimeError(
                f"재조임 후에도 파지 부하 {ratio:.4f}가 임계값 "
                f"{MIN_HOLD_LOAD_RATIO:.2f} 미만입니다. 상승하지 않습니다"
            )

    midpoint = tuple(
        (grasp + safe) / 2.0 for grasp, safe in zip(grasp_pose, safe_pose, strict=True)
    )
    confirm("파지가 안정적입니다. 중간 높이까지 시험 상승")
    glide(driver, "mid-lift", midpoint, steps=20, delay=0.12)
    report(driver, "mid-lift")
    mid_load_raw = driver.get_load(6)
    mid_load_ratio = abs(mid_load_raw) / LOAD_MAX_RAW
    if mid_load_ratio < MIN_HOLD_LOAD_RATIO:
        raise RuntimeError(
            f"중간 상승 후 파지 부하 {mid_load_ratio:.4f}가 임계값 "
            f"{MIN_HOLD_LOAD_RATIO:.2f} 미만입니다. 145mm 상승하지 않습니다"
        )

    confirm("미끄러짐이 없습니다. 145mm 운반 전 안전 자세까지 상승")
    glide(driver, "safe-lift", safe_pose, steps=20, delay=0.12)
    print("145mm에서 5초 유지")
    for second in range(1, 6):
        time.sleep(1.0)
        load_raw = driver.get_load(6)
        print(
            f"hold={second}/5 gripper={driver.get_position(6)} "
            f"load_raw={load_raw} load_ratio={abs(load_raw) / LOAD_MAX_RAW:.4f}"
        )

    confirm("5초 유지 성공을 확인했습니다. 물체를 파지 높이로 내리기")
    glide(driver, "lower", grasp_pose)

    confirm("물체가 바닥에 안정적으로 닿았습니다. 그리퍼 열기")
    set_width(driver, profile.preopen_width_mm)

    confirm("물체와 손을 이동 경로에서 치웠습니다. 145mm 안전 자세로 복귀")
    glide(driver, "finish-safe", safe_pose)
    report(driver, "complete")
    print("\n수평 파지 시험 완료")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n운영자 요청으로 다음 동작 전에 중단했습니다. 현재 자세를 유지합니다.")
