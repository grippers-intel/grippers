#!/usr/bin/env python3
"""SO-ARM101 IDLE 자세 정렬 도구.

전원 투입 후 servo torque가 꺼진 상태에서는 팔이 중력으로 처진다. 이 도구는
현재 자세를 읽어 registered IDLE 자세(IDLE_CRADLE_RAW, servo 6은 CLOSED)로
천천히 정렬한다. arm_driver_node는 기동 시 IDLE 편차를 로그로만 남기고 절대
자동으로 움직이지 않으므로(안전 계약 — arm_driver_node.py의
``_log_idle_offset`` 참고), 사람이 이 도구를 확인하며 실행하는 것이 유일한
정렬 경로다.

⚠️ 반드시 알아야 할 하드웨어 거동 (2026-08-21 Pi 실기에서 확인, driver_sdk
소스에는 이 동작이 없다 — 펌웨어 레벨이라 코드만 읽어서는 알 수 없다):

    STS3215는 goal_position 레지스터에 write하면 torque가 자동으로
    활성화된다. 즉 torque가 꺼진 채 늘어져 있는 관절에 목표 자세를 바로
    write하면, write가 도달하는 순간 torque가 켜지면서 그 목표를 향해
    급하게 움직이기 시작할 수 있다.

따라서 안전한 순서는 다음과 같고, 이 파일은 그 순서를 그대로 구현한다:

    1) 전 서보 present position을 읽는다.
    2) 각 서보의 goal에 자기 present 값을 그대로 write한다.
       → 이 시점에 torque가 켜지지만 goal == present이므로 움직임은 0이다.
    3) 그다음에야 목표(IDLE/CLOSED)로 선형 보간 이동을 시작한다.

이 순서를 건너뛰고 곧장 목표를 write하면, torque가 켜지는 순간 무엇을 향해
움직일지 예측할 수 없다. 다음에 이 파일을 만지는 사람은 반드시 이 순서를
유지할 것.
"""

import argparse
import sys
import time

from grippers_arm.floor_grasp_profiles import IDLE_CRADLE_RAW
from grippers_arm.gripper_calibration import GRIPPER_CLOSED_MM, position_from_width

SERVO_IDS = range(1, 6)
GRIPPER_SERVO_ID = 6

# CLOSED는 하드코딩하지 않는다 — gripper_calibration의 실측 보정표에서 그대로
# 끌어온다 (GRIPPER_CALIBRATION_POINTS[0] == (9.0, 1150), 9.0mm==GRIPPER_CLOSED_MM).
GRIPPER_CLOSED_RAW = position_from_width(GRIPPER_CLOSED_MM)

DEFAULT_PORT = "/dev/soarm"
DEFAULT_STEPS = 12
DEFAULT_SETTLE_SEC = 0.6
DEFAULT_TOLERANCE_RAW = 120

# 이보다 큰 편차의 자동 이동은 위험하다고 본다 — 사람이 손으로 대략 맞춘 뒤
# 재실행하도록 안내하고 아무것도 쓰지 않는다.
REJECT_TOLERANCE_RAW = 800
MAX_START_SERVO2_TEMP_C = 50

# 끼임 감지: 이만큼 스텝 동안 진전(prior_error - current_error)이 잡음 여유
# STALL_PROGRESS_RAW를 넘지 못하면 끼임으로 본다.
#
# 이 값은 "스텝당 최소 유의미 진전"이지 "정렬 성공 판정 기준"이 아니다 —
# 그건 --tolerance(기본 120, DEFAULT_TOLERANCE_RAW)가 담당한다. 실기
# (2026-08-21)에서 offset=6인 서보가 12스텝 보간의 반올림 격자상 스텝마다
# 동일한 waypoint를 받아 "진전 없음"으로 오판, 이미 최종 허용치 안에
# 들어와 있는데도 JamDetected가 나는 걸 확인했다. glide_to_targets가
# current_error를 STALL_PROGRESS_RAW가 아니라 tolerance와 비교해 "이미
# 충분히 가깝다"를 먼저 걸러내는 이유가 이것.
STALL_STEPS = 2
STALL_PROGRESS_RAW = 2

# 보간 이동용 저속/저가속 값. 실측 튜닝값이 아니라 "느리고 안전한 쪽"으로
# 잡은 보수적 기본값이다 — 2026-08-21에 6스텝 0.5초로도 부드러웠다.
SPEED_RAW = 150
ACCELERATION_RAW = 20


class JamDetected(RuntimeError):
    """보간 이동 중 끼임(진전 없음)을 감지해 중단했을 때 발생한다."""


def idle_targets():
    """servo 1..5는 IDLE_CRADLE_RAW, servo 6(gripper)은 CLOSED로 정렬한다."""
    targets = {servo_id: IDLE_CRADLE_RAW[servo_id - 1] for servo_id in SERVO_IDS}
    targets[GRIPPER_SERVO_ID] = GRIPPER_CLOSED_RAW
    return targets


def read_positions(driver, servo_ids):
    return {servo_id: driver.get_position(servo_id) for servo_id in servo_ids}


def check_safe_to_align(
    status,
    targets,
    reject_tolerance=REJECT_TOLERANCE_RAW,
    max_servo2_temp=MAX_START_SERVO2_TEMP_C,
):
    """하나라도 위반하면 사유 문자열 리스트를 돌려준다. 빈 리스트면 안전 — 이
    함수는 절대 driver에 쓰지 않는다."""
    problems = []

    offline = sorted(servo_id for servo_id, s in status.items() if not s.online)
    if offline:
        problems.append(f"통신 불가 servo: {offline}")

    for servo_id, target in targets.items():
        s = status.get(servo_id)
        if s is None or not s.online or s.position is None:
            continue
        offset = s.position - target
        if abs(offset) > reject_tolerance:
            problems.append(
                f"servo {servo_id} 편차 {offset:+d}가 허용치 {reject_tolerance}를 "
                "초과합니다. 손으로 대략 맞춘 뒤 재실행하세요"
            )

    servo2 = status.get(2)
    if servo2 is not None and servo2.online and servo2.temperature is not None:
        if servo2.temperature > max_servo2_temp:
            problems.append(
                f"servo 2 온도 {servo2.temperature}°C가 상한 {max_servo2_temp}°C를 초과했습니다. "
                "냉각 후 재시도하세요"
            )

    return problems


def report_offsets(status, targets):
    lines = []
    for servo_id in sorted(targets):
        s = status.get(servo_id)
        target = targets[servo_id]
        if s is None or not s.online or s.position is None:
            lines.append(f"servo {servo_id}: offline target={target}")
            continue
        offset = s.position - target
        lines.append(f"servo {servo_id}: present={s.position} target={target} offset={offset:+d}")
    return "\n".join(lines)


def latch_torque_at_present(driver, targets):
    """goal <- present write. STS3215는 이 write에 torque를 자동으로 켜지만
    goal == present라 움직임은 0이다 (모듈 docstring 참고). 실패하면
    아무것도 더 진행하지 않고 예외를 낸다."""
    present = read_positions(driver, targets)
    for servo_id, position in present.items():
        if position is None:
            raise RuntimeError(f"servo {servo_id} present position 읽기 실패 — latch 중단")
        if not driver.set_position(servo_id, position):
            raise RuntimeError(f"servo {servo_id} goal<-present write 실패 — latch 중단")
    return present


def glide_to_targets(
    driver,
    start,
    targets,
    steps=DEFAULT_STEPS,
    settle=DEFAULT_SETTLE_SEC,
    stall_steps=STALL_STEPS,
    stall_progress=STALL_PROGRESS_RAW,
    tolerance=DEFAULT_TOLERANCE_RAW,
):
    """선형 보간 이동. 스텝마다 present를 읽어 로그로 출력한다.

    목표에 유의미하게 못 미친 채(오차가 stall_progress보다 큰 채) 진전이
    stall_steps 스텝 연속 없으면 즉시 중단한다 — 전 서보를 현재 위치로
    goal 고정하고 어느 서보가 걸렸는지 담아 JamDetected를 낸다.

    단, current_error가 이미 tolerance(최종 허용치) 이내면 그 서보는 스텝
    진전 여부와 무관하게 "이미 다 왔다"로 보고 stall 판정에서 뺀다. 총
    오프셋이 작으면(예: 6 raw를 12스텝으로 보간) 반올림 격자상 연속 스텝의
    waypoint가 같은 값이 되어 stall_progress(2)보다 미세하게 낮은 진전만
    보일 수 있는데, 이건 끼임이 아니라 애초에 옮길 게 거의 없었던 것이다."""
    servo_ids = list(targets)
    prior_error = {servo_id: abs(start[servo_id] - targets[servo_id]) for servo_id in servo_ids}
    stall_counts = dict.fromkeys(servo_ids, 0)

    for step_index in range(1, steps + 1):
        ratio = step_index / steps
        waypoint = {
            servo_id: round(start[servo_id] + ratio * (targets[servo_id] - start[servo_id]))
            for servo_id in servo_ids
        }
        for servo_id, position in waypoint.items():
            if not driver.set_position(servo_id, position):
                raise RuntimeError(f"servo {servo_id} write 실패 — step {step_index}/{steps}")
        time.sleep(settle)

        present = read_positions(driver, servo_ids)
        print(f"[align] step={step_index}/{steps} present={present}")

        for servo_id in servo_ids:
            position = present[servo_id]
            if position is None:
                continue
            current_error = abs(position - targets[servo_id])
            if current_error <= tolerance:
                stall_counts[servo_id] = 0
                prior_error[servo_id] = current_error
                continue

            progressed = (prior_error[servo_id] - current_error) > stall_progress
            stall_counts[servo_id] = 0 if progressed else stall_counts[servo_id] + 1
            prior_error[servo_id] = current_error

            if stall_counts[servo_id] >= stall_steps:
                for stuck_id in servo_ids:
                    stuck_position = present.get(stuck_id)
                    if stuck_position is not None:
                        driver.set_position(stuck_id, stuck_position)
                raise JamDetected(
                    f"servo {servo_id}가 목표까지 {current_error} 남은 채 {stall_steps}스텝 "
                    "연속 진전이 없습니다. 현재 위치를 goal로 고정했습니다"
                )

    return read_positions(driver, servo_ids)


def _connect(port):
    # driver_sdk(pyserial 의존)는 여기서만 import한다 — 그래야 위의 검사/보간
    # 로직은 하드웨어 없이도 fake driver로 단위 테스트할 수 있다.
    #
    # soarm_lab을 먼저 import해야 한다 — soarm_lab/__init__.py가 자기
    # 디렉터리를 sys.path에 얹어 둬서 driver_sdk를 flat import할 수 있게
    # 만든다 (arm_driver_node.py와 동일한 규칙). 실기(2026-08-21)에서
    # 이 줄 없이 바로 driver_sdk를 import해 ModuleNotFoundError로 확인됨.
    import soarm_lab  # noqa: F401
    from driver_sdk import STS3215Driver

    driver = STS3215Driver(port)
    return driver if driver.connect() else None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--steps", type=int, default=DEFAULT_STEPS)
    parser.add_argument("--settle", type=float, default=DEFAULT_SETTLE_SEC)
    parser.add_argument("--dry-run", action="store_true", help="검사와 리포트만 하고 종료")
    parser.add_argument("--tolerance", type=int, default=DEFAULT_TOLERANCE_RAW)
    args = parser.parse_args(argv)

    targets = idle_targets()

    driver = _connect(args.port)
    if driver is None:
        print(f"[align] 연결 실패: {args.port}", file=sys.stderr)
        return 1

    status = driver.get_all_status()
    problems = check_safe_to_align(status, targets)
    if problems:
        print("[align] 안전 검사 실패 — 아무것도 쓰지 않았습니다:", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        return 1

    print(report_offsets(status, targets))

    if args.dry_run:
        print("[align] --dry-run — 검사와 리포트만 수행했습니다")
        return 0

    print("[align] goal<-present write로 torque를 latch합니다 (이동 없음)")
    start = latch_torque_at_present(driver, targets)

    for servo_id in targets:
        driver.set_speed(servo_id, SPEED_RAW)
        driver.set_acceleration(servo_id, ACCELERATION_RAW)

    try:
        final = glide_to_targets(
            driver, start, targets, steps=args.steps, settle=args.settle, tolerance=args.tolerance
        )
    except JamDetected as e:
        print(f"[align] 끼임 감지 — 중단: {e}", file=sys.stderr)
        return 2

    print(f"[align] final={final}")
    final_offsets = {
        servo_id: final[servo_id] - targets[servo_id]
        for servo_id in targets
        if final.get(servo_id) is not None
    }
    worst = max(final_offsets.values(), key=abs) if final_offsets else None
    if worst is None or abs(worst) > args.tolerance:
        print(
            f"[align] 최종 오차가 허용치 {args.tolerance}를 초과했습니다: {final_offsets}",
            file=sys.stderr,
        )
        return 3

    print(f"[align] IDLE 정렬 완료 — 최종 오차 {final_offsets}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[align] 운영자 중단 — 현재 자세를 유지합니다", file=sys.stderr)
        sys.exit(2)
