#!/usr/bin/env python3
"""Operator-gated empty-hand validation for the direct basket drop path.

This does not release an object.  It validates IDLE -> DROP_195 -> IDLE
without the SAFE_145 waypoints.  Keep the base stationary.
"""

import time

from driver_sdk import STS3215Driver
from grippers_arm.floor_grasp_profiles import (
    BASKET_DROP_195_RAW,
    IDLE_CRADLE_RAW,
)
from grippers_arm.gripper_calibration import position_from_width

SERVO_IDS = range(1, 6)
START_TOLERANCE_RAW = 120
MAX_START_SERVO2_TEMP_C = 40


def confirm(message):
    answer = input(f"\n{message}\nEnter=계속, q=중단 > ").strip().lower()
    if answer == "q":
        raise KeyboardInterrupt("operator aborted before transition")
    if answer:
        raise RuntimeError("Enter 또는 q만 입력하세요")


def read_arm(driver):
    return {servo_id: driver.get_position(servo_id) for servo_id in SERVO_IDS}


def near_pose(actual, expected):
    return all(
        abs(actual[servo_id] - expected[servo_id - 1]) <= START_TOLERANCE_RAW
        for servo_id in SERVO_IDS
    )


def glide_raw(driver, label, goal_raw, steps=30, delay=0.10):
    start = read_arm(driver)
    goal = {servo_id: goal_raw[servo_id - 1] for servo_id in SERVO_IDS}
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


def report(driver, label):
    print(f"\n[{label}] arm={read_arm(driver)} gripper={driver.get_position(6)}")
    print(f"[{label}] load={ {i: driver.get_load(i) for i in range(1, 7)} }")
    print(f"[{label}] temp={ {i: driver.get_temperature(i) for i in range(1, 7)} }")
    print(f"[{label}] voltage={ {i: driver.get_voltage(i) for i in range(1, 7)} }")


def main():
    driver = STS3215Driver("/dev/soarm")
    driver.connect()

    actual = read_arm(driver)
    if not near_pose(actual, IDLE_CRADLE_RAW):
        raise RuntimeError(
            "시작 자세가 등록된 IDLE과 다릅니다. 자동 이동하지 않습니다: " f"actual={actual}"
        )
    servo2_temp = driver.get_temperature(2)
    if servo2_temp > MAX_START_SERVO2_TEMP_C:
        raise RuntimeError(f"servo 2 온도 {servo2_temp}°C — 냉각 후 재시도하세요")
    if not all(driver.get_torque(i) is True for i in range(1, 7)):
        raise RuntimeError("torque OFF인 servo가 있습니다")

    report(driver, "start")
    confirm("빈손이고 바구니·차체·케이블 주변이 안전합니다. 그리퍼를 80mm로 열기")
    if not driver.set_position(6, position_from_width(80.0)):
        raise RuntimeError("servo 6 position write failed")
    time.sleep(1.5)

    confirm("차체·바구니·케이블 간섭을 지켜보며 IDLE에서 실측 DROP_195로 직접 이동")
    glide_raw(driver, "drop-195", BASKET_DROP_195_RAW)
    report(driver, "drop-195")

    confirm("직접 전개 경로의 무간섭을 확인했습니다. DROP_195에서 IDLE로 직접 복귀")
    glide_raw(driver, "return-idle", IDLE_CRADLE_RAW)
    report(driver, "complete")
    print("\n빈손 IDLE ↔ DROP_195 직접 왕복 완료")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n운영자 요청으로 다음 동작 전에 중단했습니다. 현재 자세를 유지합니다.")
