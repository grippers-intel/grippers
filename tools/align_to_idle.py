#!/usr/bin/env python3
"""SO-ARM101 IDLE 자세 정렬 도구.

전원 투입 후 servo torque가 꺼진 상태에서는 팔이 중력으로 처진다. 이 도구는
현재 자세를 읽어 registered IDLE 자세(IDLE_CRADLE_RAW, servo 6은 CLOSED)로
천천히 정렬한다.

2026-08-25부터 arm_driver_node가 **첫 바닥 자세 이동 요청 때 같은 정렬을
자동으로** 수행한다(``_auto_align_to_idle``). 그래도 이 도구는 남는다 —
노드를 띄우지 않은 상태(포트가 비어 있을 때)에서 쓸 수 있는 유일한 경로이고,
``auto_align_on_first_move:=false``로 자동 정렬을 껐을 때의 수동 경로이기도
하다. 두 구현은 같은 안전 순서(goal<-present latch 후 이동)를 지키지만
코드를 공유하지 않는다: 이 도구는 driver_sdk에 직접 붙고, 노드 쪽은
자기가 이미 쥔 백엔드를 쓴다.

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

# ⚠️ 2026-08-24 사용자 지시로 **편차 상한 거부를 껐다**. 예전에는 이 값을
# 넘으면 아무것도 쓰지 않고 "손으로 대략 맞춘 뒤 재실행하라"고 안내했는데,
# 실기에서 IDLE 복귀가 필요한 상황은 대개 편차가 클 때라 그 가드가 오히려
# 매번 걸림돌이었다. 사용자가 현장에서 팔을 보며 "위험 요소 없다"고 판단해
# 껐다.
#
# 이제 이 값은 **거부 기준이 아니라 경고 기준**이다 — 넘으면 알리기만 하고
# 정렬은 그대로 진행한다. 남아 있는 보호장치는 그대로다:
#   - 통신 불가 서보가 있으면 여전히 거부한다(위치를 못 읽으면 보간 자체가
#     불가능하다)
#   - servo 2 과열(MAX_START_SERVO2_TEMP_C)은 여전히 거부한다
#   - 이동 중 끼임 감지(JamDetected)는 그대로 살아 있다 — 실제로 뭔가에
#     걸리면 그 자리에서 멈추고 현재 위치로 goal을 고정한다
LARGE_OFFSET_WARN_RAW = 800
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

# 보간이 끝난 뒤 실제 도달을 기다리는 값 (converge_at_targets 참고).
#
# 타임아웃은 SPEED_RAW에서 거꾸로 잡는다: 실측 단위가 대략 raw/s라(2026-08-24,
# 레지스터 150에서 153 raw/s) 이 도구가 감당해야 하는 최대 편차 ~1700 raw는
# 11s가 넘게 걸린다. 8s로는 모자라서 큰 편차를 정렬할 때 도달 직전에 포기했다.
#
# ⚠️ 여기서 SPEED_RAW를 느리게 쓰는 건 의도한 것이다(사람이 지켜보며 도는
# 정렬이라 느린 편이 안전하다). 다만 이 값은 서보 레지스터에 남아 이후 다른
# 코드의 이동 속도까지 바꾼다 — arm_driver_node는 그래서 이동마다 자기 속도를
# 다시 쓴다(_glide_to_raw_positions 주석 참고). 새 도구를 만들 때도 속도를
# 상속하지 말고 직접 쓸 것.
CONVERGE_POLL_SEC = 0.2
CONVERGE_TIMEOUT_SEC = 20.0


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
    max_servo2_temp=MAX_START_SERVO2_TEMP_C,
):
    """하나라도 위반하면 사유 문자열 리스트를 돌려준다. 빈 리스트면 안전 — 이
    함수는 절대 driver에 쓰지 않는다.

    편차 크기는 더 이상 거부 사유가 아니다(위 LARGE_OFFSET_WARN_RAW 주석
    참고) — 큰 편차는 large_offsets()로 따로 뽑아 경고만 한다."""
    problems = []

    offline = sorted(servo_id for servo_id, s in status.items() if not s.online)
    if offline:
        problems.append(f"통신 불가 servo: {offline}")

    servo2 = status.get(2)
    if servo2 is not None and servo2.online and servo2.temperature is not None:
        if servo2.temperature > max_servo2_temp:
            problems.append(
                f"servo 2 온도 {servo2.temperature}°C가 상한 {max_servo2_temp}°C를 초과했습니다. "
                "냉각 후 재시도하세요"
            )

    return problems


def large_offsets(status, targets, warn_tolerance=LARGE_OFFSET_WARN_RAW):
    """경고만 할 큰 편차 목록(거부하지 않는다). 순수 함수."""
    found = []
    for servo_id, target in targets.items():
        s = status.get(servo_id)
        if s is None or not s.online or s.position is None:
            continue
        offset = s.position - target
        if abs(offset) > warn_tolerance:
            found.append(f"servo {servo_id} 편차 {offset:+d} (경고 기준 {warn_tolerance} 초과)")
    return found


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

    return converge_at_targets(driver, targets, tolerance=tolerance)


def converge_at_targets(
    driver,
    targets,
    tolerance=DEFAULT_TOLERANCE_RAW,
    timeout=CONVERGE_TIMEOUT_SEC,
    poll=CONVERGE_POLL_SEC,
):
    """보간이 끝난 뒤 서보가 실제로 목표에 도달할 때까지 기다린다.

    ⚠️ 2026-08-24 실기로 확인한 문제: 보간 루프는 마지막 스텝에서 목표값을
    write한 직후 바로 present를 읽고 끝냈다. write는 "명령을 보냈다"일 뿐
    "도달했다"가 아니라서, 편차가 클수록(그날은 servo 2가 +1668) 서보가
    보간 속도를 못 따라가 마지막 스텝에서 수백 raw가 남은 채로 종료됐다
    (실측 최종 잔차 servo 2 = 593, servo 4 = -534 → 허용치 120 초과로
    실패 반환). 스텝 수를 늘리는 것보다 **끝에서 도달을 기다리는 쪽**이
    맞다 — 남은 거리가 얼마든 goal은 이미 목표에 박혀 있으므로 서보는
    계속 그쪽으로 가고, 우리는 그게 멎기를 기다리기만 하면 된다.

    잔차가 tolerance 안에 들어오면 즉시 반환한다. 시간 안에 못 들어오면
    **예외를 내지 않고** 마지막 present를 그대로 돌려준다 — 최종 판정과
    종료 코드는 호출부(main)가 이미 하고 있고, 여기서 예외를 내면 그
    리포트가 사라지기 때문이다.
    """
    servo_ids = list(targets)
    for servo_id, target in targets.items():
        driver.set_position(servo_id, target)

    deadline = time.monotonic() + timeout
    while True:
        present = read_positions(driver, servo_ids)
        errors = {
            servo_id: present[servo_id] - targets[servo_id]
            for servo_id in servo_ids
            if present[servo_id] is not None
        }
        if errors and all(abs(error) <= tolerance for error in errors.values()):
            return present
        if time.monotonic() >= deadline:
            print(f"[align] 도달 대기 {timeout}s 초과 — 잔차 {errors}", file=sys.stderr)
            return present
        time.sleep(poll)


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

    for warning in large_offsets(status, targets):
        print(f"[align] ⚠️ {warning} — 거부하지 않고 그대로 정렬합니다", file=sys.stderr)

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
