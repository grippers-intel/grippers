#!/usr/bin/env python3
"""Operator-gated empty-hand validation for the direct basket drop path.

This does not release an object.  It validates IDLE -> DROP_195 -> IDLE
without the SAFE_145 waypoints.  Keep the base stationary.
"""

import time

# soarm_lab을 먼저 import해야 한다 — soarm_lab/__init__.py가 자기 디렉터리를
# sys.path에 얹어 둬서 driver_sdk를 flat import할 수 있게 만든다
# (arm_driver_node.py / tools/align_to_idle.py와 동일한 규칙). 실기
# (2026-08-21)에서 이 줄 없이 바로 driver_sdk를 import해
# ModuleNotFoundError로 확인됨.
import soarm_lab  # noqa: F401
from driver_sdk import STS3215Driver
from grippers_arm.floor_grasp_profiles import (
    BASKET_DROP_195_RAW,
    IDLE_CRADLE_RAW,
)
from grippers_arm.gripper_calibration import GRIPPER_CLOSED_MM, position_from_width

SERVO_IDS = range(1, 6)
START_TOLERANCE_RAW = 120
MAX_START_SERVO2_TEMP_C = 50

# CLOSED는 하드코딩하지 않는다 — align_to_idle.py와 동일하게 gripper_calibration의
# 실측 보정표에서 그대로 끌어온다.
GRIPPER_CLOSED_RAW = position_from_width(GRIPPER_CLOSED_MM)

# glide_raw는 고정 스텝 수(30)×delay(0.1s)로만 보간을 커밋하고 present가 실제로
# goal에 닿았는지는 보지 않는다. 큰 폭 이동(IDLE 접기)은 그 창 안에 안 끝날 수
# 있다 — horizontal_grasp_hardware_test.py에서 실기(2026-08-21)로 확인된 문제와
# 동일. 마지막 IDLE 복귀에는 반드시 이 확인을 거친다.
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
    """glide_raw가 끝난 뒤에도 present가 goal에 닿지 않았을 수 있다 (모듈 상단
    SETTLE_TOLERANCE_RAW 주석 참고). targets에 있는 서보(팔 1~5뿐 아니라
    그리퍼 6도 가능)가 전부 tolerance 안에 들어올 때까지 poll 간격으로 최대
    timeout초 present를 다시 읽는다. 끝까지 못 들어오면 무엇이 얼마나
    남았는지 담아 RuntimeError를 낸다.

    ⚠️ 물체를 잡느라 목표에 못 미치는 게 정상인 호출에는 쓰지 않는다 — 여기는
    "주변이 비었다고 확신하는" 자유 이동에만 쓴다."""
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
    idle_raw = {servo_id: IDLE_CRADLE_RAW[servo_id - 1] for servo_id in SERVO_IDS}
    wait_until_converged(driver, "return-idle", idle_raw)

    # IDLE 관례는 그리퍼 CLOSED다 (align_to_idle.py의 idle_targets() 참고).
    # 시험 시작에 80mm로 열어뒀으니 여기서 닫아 정식 IDLE로 맞춘다.
    confirm("그리퍼 주변이 비어 있습니다. 정식 IDLE로 그리퍼 닫기")
    if not driver.set_position(6, position_from_width(GRIPPER_CLOSED_MM)):
        raise RuntimeError("servo 6 position write failed")
    time.sleep(1.5)
    wait_until_converged(driver, "gripper-idle-close", {6: GRIPPER_CLOSED_RAW})

    report(driver, "complete")
    print("\n빈손 IDLE ↔ DROP_195 직접 왕복 완료")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n운영자 요청으로 다음 동작 전에 중단했습니다. 현재 자세를 유지합니다.")
