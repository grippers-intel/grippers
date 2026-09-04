#!/usr/bin/env python3
"""Operator-gated empty-hand validation for the production (300mm) basket
drop pose — BASKET_DROP_300_RAW in floor_grasp_profiles.py.

This does not release an object.  It validates IDLE -> DROP_300 -> IDLE
without the SAFE_145 waypoints.  Keep the base stationary.

BASKET_DROP_300_RAW replaced the originally-taught BASKET_DROP_195_RAW on
2026-09-04 ("그리퍼 사이의 물체가 바구니에 안 닿을 것 같다"는 사용자 우려).
It is not a measured/taught pose — it is the FK-computed solution from
tools/drop_pose_height_test.py (--height-mm 300 --lock-servo1-to-idle),
which keeps the same forward reach (차체 전면 기준 약 200mm) and the same
horizontal finger orientation as the old 195mm pose, re-solving only
servo2/3/4 for the new height. servo1 is also intentionally different from
the old taught value: the old value (2029) was -37raw(-3.25°) off from
IDLE(2066), which made the base rotate slightly on every DROP entry/return
(실기 관찰, 2026-09-04) — 사용자 지시("웬만하면 그냥 고정해줘")로 IDLE의
servo1을 그대로 물려써 그 회전을 없앴다. This script — and the earlier
230/250/270mm intermediate steps that led to it — confirmed the computed
pose interference-free before it was promoted into floor_grasp_profiles.py
and rebuilt into the grippers_arm ROS package (2026-09-04).
"""

import time

# soarm_lab을 먼저 import해야 한다 — soarm_lab/__init__.py가 자기 디렉터리를
# sys.path에 얹어 둬서 driver_sdk를 flat import할 수 있게 만든다
# (arm_driver_node.py / tools/align_to_idle.py와 동일한 규칙).
import soarm_lab  # noqa: F401
from driver_sdk import STS3215Driver
from grippers_arm.floor_grasp_profiles import (
    BASKET_DROP_300_RAW,
    IDLE_CRADLE_RAW,
    TAUGHT_POSITION_LIMITS,
)
from grippers_arm.gripper_calibration import GRIPPER_CLOSED_MM, position_from_width

DROP_300_RAW = BASKET_DROP_300_RAW

SERVO_IDS = range(1, 6)
START_TOLERANCE_RAW = 120
MAX_START_SERVO2_TEMP_C = 50

GRIPPER_CLOSED_RAW = position_from_width(GRIPPER_CLOSED_MM)

SETTLE_TOLERANCE_RAW = 120
SETTLE_TIMEOUT_SEC = 15.0
SETTLE_POLL_SEC = 0.3


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


def wait_until_converged(
    driver,
    label,
    targets,
    tolerance=SETTLE_TOLERANCE_RAW,
    timeout=SETTLE_TIMEOUT_SEC,
    poll=SETTLE_POLL_SEC,
):
    deadline = time.monotonic() + timeout
    present = {sid: driver.get_position(sid) for sid in targets}
    while True:
        offsets = {sid: present[sid] - targets[sid] for sid in targets}
        if all(abs(offset) <= tolerance for offset in offsets.values()):
            print(f"[{label}] 수렴 확인 offsets={offsets}")
            return present
        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"[{label}] {timeout}s 안에 허용치 {tolerance}로 수렴하지 않았습니다: "
                f"present={present} targets={targets} offsets={offsets}"
            )
        time.sleep(poll)
        present = {sid: driver.get_position(sid) for sid in targets}


def report(driver, label):
    print(f"\n[{label}] arm={read_arm(driver)} gripper={driver.get_position(6)}")
    print(f"[{label}] load={ {i: driver.get_load(i) for i in range(1, 7)} }")
    print(f"[{label}] temp={ {i: driver.get_temperature(i) for i in range(1, 7)} }")
    print(f"[{label}] voltage={ {i: driver.get_voltage(i) for i in range(1, 7)} }")


def main():
    for servo_id, raw in zip(range(1, 6), DROP_300_RAW):
        lo, hi = TAUGHT_POSITION_LIMITS[servo_id]
        if not (lo <= raw <= hi):
            raise RuntimeError(
                f"DROP_300_RAW servo{servo_id}={raw}가 TAUGHT_POSITION_LIMITS "
                f"[{lo},{hi}] 밖입니다 — 계산이 이 팔의 현재 캘리브레이션과 "
                "안 맞습니다. 실기로 옮기지 마세요."
            )

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

    print(f"\n목표 DROP_300_RAW(production)={DROP_300_RAW}")
    confirm("차체·바구니·케이블 간섭을 지켜보며 IDLE에서 DROP_300(계산값)으로 직접 이동")
    glide_raw(driver, "drop-300", DROP_300_RAW)
    report(driver, "drop-300")

    confirm("직접 전개 경로의 무간섭을 확인했습니다. DROP_300에서 IDLE로 직접 복귀")
    glide_raw(driver, "return-idle", IDLE_CRADLE_RAW)
    idle_raw = {servo_id: IDLE_CRADLE_RAW[servo_id - 1] for servo_id in SERVO_IDS}
    wait_until_converged(driver, "return-idle", idle_raw)

    confirm("그리퍼 주변이 비어 있습니다. 정식 IDLE로 그리퍼 닫기")
    if not driver.set_position(6, position_from_width(GRIPPER_CLOSED_MM)):
        raise RuntimeError("servo 6 position write failed")
    time.sleep(1.5)
    wait_until_converged(driver, "gripper-idle-close", {6: GRIPPER_CLOSED_RAW})

    report(driver, "complete")
    print("\n빈손 IDLE ↔ DROP_300 직접 왕복 완료")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n운영자 요청으로 다음 동작 전에 중단했습니다. 현재 자세를 유지합니다.")
