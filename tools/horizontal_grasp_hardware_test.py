#!/usr/bin/env python3
"""Interactive, operator-gated SO-ARM101 horizontal grasp hardware test.

Every physical transition requires Enter.  Entering ``q`` stops before the
next transition; Ctrl-C also leaves the arm at its latest commanded waypoint.
Run only with the robot attended and the base stationary.
"""

import argparse
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
    FLOOR_GRASP_PROFILES,
    HORIZONTAL_GRASP_POSES_DEG,
    HORIZONTAL_SAFE_145_DEG,
    HORIZONTAL_SAFE_145_RAW,
    IDLE_CRADLE_RAW,
)
from grippers_arm.gripper_calibration import (
    GRIPPER_CLOSED_MM,
    GRIPPER_GRASP_MIN_MM,
    position_from_width,
)

SERVO_IDS = range(1, 6)
LOAD_MAX_RAW = 1023.0
MIN_HOLD_LOAD_RATIO = 0.04
SAFE_START_TOLERANCE_RAW = 120
RETRY_TIGHTEN_MM = 5.0
MAX_START_SERVO2_TEMP_C = 50

# CLOSED는 하드코딩하지 않는다 — align_to_idle.py와 동일하게 gripper_calibration의
# 실측 보정표에서 그대로 끌어온다.
GRIPPER_CLOSED_RAW = position_from_width(GRIPPER_CLOSED_MM)

# glide_raw/glide는 고정 스텝 수(30)×delay(0.1s)로만 보간을 커밋하고 present가
# 실제로 goal에 닿았는지는 보지 않는다. 큰 폭 이동(예: IDLE 접기)은 그 창 안에
# 안 끝날 수 있다 — 실기(2026-08-21)에서 step=30/30에 servo 2가 920 raw,
# servo 4가 462 raw 남은 채 "완료"가 찍혔다. STS3215 자체 컨트롤러는 이미
# 써놓은 goal을 향해 계속 움직이므로 위험하진 않았지만(나중에 확인하니 도착),
# 이후 사람이 없는 자동화 경로에서는 이 창을 놓치면 안 도착한 걸 도착했다고
# 보고하게 된다. 마지막 전환에는 반드시 이 확인을 거친다.
SETTLE_TOLERANCE_RAW = 120
SETTLE_TIMEOUT_SEC = 15.0
SETTLE_POLL_SEC = 0.3


# --yes 로 켠다. 자동 사이클에서는 사람이 매 전환마다 Enter 를 칠 수 없다.
# 확인만 건너뛸 뿐, 파지 부하·서보 온도·시작 자세 검사는 그대로 살아 있다 —
# 물체를 놓쳤으면 들어올리기 전에 여전히 멈춘다.
AUTO = False


def confirm(message):
    """Wait for Enter before a transition; q aborts without moving."""
    if AUTO:
        print(f"\n{message}\n  → 자동 진행")
        return
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


# 2026-08-23: 스텝당 이동이 커서 서보가 "최대 속도로 튕기고 100ms 대기"를 반복해
# 눈에 띄게 떨었다(servo2 기준 스텝당 약 55카운트 ≈ 5도). 총 소요 시간은 그대로 두고
# 스텝을 3배 잘게 쪼개 스텝당 이동을 1/3로 줄였다. 도달 자세는 바뀌지 않는다.
# 여전히 거칠면 --accel 로 서보 자체 가속도를 걸어볼 것.
def glide_raw(driver, label, goal_raw, steps=90, delay=0.034):
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
        # read_arm 은 서보 5회 읽기다. 30Hz 루프에서 자주 부르면 타이밍이 흔들리므로
        # 스텝 수와 무관하게 6번쯤만 찍는다.
        if step_index % max(1, steps // 6) == 0:
            print(f"[{label}] step={step_index}/{steps} present={read_arm(driver)}")
    time.sleep(1.0)


def glide(driver, label, angles_deg, steps=90, delay=0.034):
    glide_raw(
        driver,
        label,
        tuple(raw_goals(driver, angles_deg).values()),
        steps=steps,
        delay=delay,
    )


def wait_until_converged(
    driver,
    label,
    targets,
    tolerance=SETTLE_TOLERANCE_RAW,
    timeout=SETTLE_TIMEOUT_SEC,
    poll=SETTLE_POLL_SEC,
):
    """glide_raw/glide/set_width가 끝난 뒤에도 present가 goal에 닿지 않았을
    수 있다 (모듈 상단 SETTLE_TOLERANCE_RAW 주석 참고). targets에 있는
    서보(팔 1~5뿐 아니라 그리퍼 6도 가능)가 전부 tolerance 안에 들어올 때까지
    poll 간격으로 최대 timeout초 present를 다시 읽는다. 끝까지 못 들어오면
    무엇이 얼마나 남았는지 담아 RuntimeError를 낸다 — "완료"를 실제로 확인
    없이 찍지 않는다.

    ⚠️ 물체를 잡느라 목표에 못 미치는 게 정상인 호출(그리퍼로 물체를 쥘 때)에는
    쓰지 않는다 — 그건 require_hold_load처럼 load로 판정해야 한다. 여기는
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


def require_hold_load(driver, stage):
    load_raw = driver.get_load(6)
    ratio = abs(load_raw) / LOAD_MAX_RAW
    print(
        f"[{stage}] gripper={driver.get_position(6)} " f"load_raw={load_raw} load_ratio={ratio:.4f}"
    )
    if ratio < MIN_HOLD_LOAD_RATIO:
        raise RuntimeError(
            f"{stage} 후 파지 부하 {ratio:.4f}가 임계값 "
            f"{MIN_HOLD_LOAD_RATIO:.2f} 미만입니다. 현재 자세를 유지합니다"
        )
    return ratio


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
    parser.add_argument(
        "--accel",
        type=int,
        default=None,
        help="서보 1~5 가속도(0~254)를 이 값으로 설정한다. 생략하면 건드리지 않는다. "
             "낮을수록 완만하지만 너무 낮으면 웨이포인트를 못 따라가 뒤처진다. 30~60 권장",
    )
    parser.add_argument(
        "--drop-to-basket",
        action="store_true",
        help="CARRY_IDLE 검증 후 DROP_195에서 투하하고 IDLE로 복귀",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="운영자 확인(Enter)을 건너뛰고 끝까지 진행한다. 자동 사이클용. "
             "안전 검사는 그대로 동작한다",
    )
    args = parser.parse_args()

    global AUTO
    AUTO = args.yes

    profile = FLOOR_GRASP_PROFILES[args.profile]
    grasp_pose = HORIZONTAL_GRASP_POSES_DEG[args.profile]
    safe_pose = HORIZONTAL_SAFE_145_DEG

    driver = STS3215Driver(args.port)
    driver.connect()

    if args.accel is not None:
        # 서보 자체 가속도 제한. 스텝마다 최대 속도로 튕기는 대신 완만하게
        # 가감속한다. 값이 너무 낮으면 33ms 안에 웨이포인트에 못 닿아 뒤처지므로,
        # 부드러워졌는지와 함께 수렴 offsets 도 같이 봐야 한다.
        for servo_id in SERVO_IDS:
            driver.set_acceleration(servo_id, args.accel)
        print(f"[setup] servo 1~5 가속도={args.accel}")

    servo2_temp = driver.get_temperature(2)
    if servo2_temp > MAX_START_SERVO2_TEMP_C:
        raise RuntimeError(
            f"servo 2 온도 {servo2_temp}°C가 시작 상한 "
            f"{MAX_START_SERVO2_TEMP_C}°C를 초과했습니다. 냉각 후 재시도하세요"
        )

    actual = read_arm(driver)
    safe_raw = raw_goals(driver, safe_pose)
    grasp_raw = raw_goals(driver, grasp_pose)
    idle_raw = {servo_id: IDLE_CRADLE_RAW[servo_id - 1] for servo_id in SERVO_IDS}
    if not (
        near_pose(actual, idle_raw) or near_pose(actual, safe_raw) or near_pose(actual, grasp_raw)
    ):
        raise RuntimeError(
            "시작 자세가 등록된 idle/safe/grasp 자세와 다릅니다. 자동 이동하지 않습니다: "
            f"actual={actual} idle={idle_raw} safe={safe_raw} grasp={grasp_raw}"
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
        # 빈 닫힘 하한이 아니라 파지 하한으로 clamp한다 — 얇은 체스말은
        # close_width_mm가 이미 그 하한이라, 9.0으로 clamp하면 재시도가
        # 조이는 게 아니라 벌리는 명령이 된다.
        retry_width_mm = max(GRIPPER_GRASP_MIN_MM, profile.close_width_mm - RETRY_TIGHTEN_MM)
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
    glide(driver, "mid-lift", midpoint, steps=60, delay=0.040)
    report(driver, "mid-lift")
    mid_load_raw = driver.get_load(6)
    mid_load_ratio = abs(mid_load_raw) / LOAD_MAX_RAW
    if mid_load_ratio < MIN_HOLD_LOAD_RATIO:
        raise RuntimeError(
            f"중간 상승 후 파지 부하 {mid_load_ratio:.4f}가 임계값 "
            f"{MIN_HOLD_LOAD_RATIO:.2f} 미만입니다. 145mm 상승하지 않습니다"
        )

    confirm("미끄러짐이 없습니다. 145mm 운반 전 안전 자세까지 상승")
    glide(driver, "safe-lift", safe_pose, steps=60, delay=0.040)
    require_hold_load(driver, "safe-145")

    confirm("파지가 유지됐습니다. 그리퍼는 닫은 채 CARRY_IDLE로 접기")
    glide_raw(driver, "carry-idle", IDLE_CRADLE_RAW)
    report(driver, "carry-idle")
    require_hold_load(driver, "carry-idle")

    if args.drop_to_basket:
        confirm(
            "바구니 중심을 그리퍼 중심에 ±5mm 이내로 맞추고 이동 경로에서 "
            "손을 뺐습니다. CARRY_IDLE에서 DROP_195로 직접 전개"
        )
        glide_raw(driver, "basket-drop-195", BASKET_DROP_195_RAW)
        report(driver, "basket-drop-195")
        require_hold_load(driver, "basket-drop-195")

        confirm("물체가 바구니 입구 중앙 위에 있습니다. 그리퍼를 80mm로 열어 투하")
        set_width(driver, profile.preopen_width_mm)

        confirm("투하를 확인했습니다. 빈손 DROP_195에서 IDLE로 직접 복귀")
        glide_raw(driver, "basket-return-idle", IDLE_CRADLE_RAW)
        wait_until_converged(driver, "basket-return-idle", idle_raw)

        # IDLE 관례는 그리퍼 CLOSED다 (align_to_idle.py의 idle_targets() 참고).
        # 투하 직후엔 그리퍼가 열린 채라 여기서 닫아 정식 IDLE로 맞춘다.
        confirm("그리퍼 주변이 비어 있습니다. 정식 IDLE로 그리퍼 닫기")
        set_width(driver, GRIPPER_CLOSED_MM)
        wait_until_converged(driver, "gripper-idle-close", {6: GRIPPER_CLOSED_RAW})

        report(driver, "basket-complete")
        print("\n수평 파지 및 바구니 투하 시험 완료")
        return

    confirm("CARRY_IDLE 파지가 유지됐습니다. 145mm 안전 자세로 다시 전개")
    glide_raw(driver, "carry-return-safe", HORIZONTAL_SAFE_145_RAW)
    report(driver, "carry-return-safe")
    require_hold_load(driver, "carry-return-safe")

    confirm("운반 자세 왕복 성공을 확인했습니다. 물체를 파지 높이로 내리기")
    glide(driver, "lower", grasp_pose)

    confirm("물체가 바닥에 안정적으로 닿았습니다. 그리퍼 열기")
    set_width(driver, profile.preopen_width_mm)

    confirm("물체와 손을 이동 경로에서 치웠습니다. 145mm 안전 자세로 복귀")
    glide(driver, "finish-safe", safe_pose)
    wait_until_converged(driver, "finish-safe", safe_raw)
    report(driver, "complete")
    print("\n수평 파지 시험 완료")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n운영자 요청으로 다음 동작 전에 중단했습니다. 현재 자세를 유지합니다.")
