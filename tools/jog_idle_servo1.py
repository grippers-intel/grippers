#!/usr/bin/env python3
"""servo 1(Base) IDLE 자세 미세 조정 콘솔 — 그리퍼가 정면을 보도록 손으로 맞출 때 쓴다.

**servo 1만 움직인다.** 다른 관절(2-6)은 이 스크립트가 전혀 건드리지 않는다.

align_to_idle.py 모듈 docstring이 설명하는 것과 같은 안전 절차를 servo 1
하나에만 적용한다 — STS3215는 goal_position write 시 torque가 자동으로
켜지므로, 시작 전 present 값을 그대로 goal에 한 번 write해 torque를
latch한다(움직임 없음). 그 이후부터 사용자가 입력하는 상대 이동만 반영한다.

조작:
  숫자(도, +/-) 입력 후 Enter  → 시작 위치 기준 누적 상대 이동. 부호로 방향
                                  전환 가능(과감히 틀었다가 다시 반대로도 됨).
  빈 입력 + Enter               → 확정. 최종 raw 값을 출력하고 종료(더 이상
                                  움직이지 않음 — 이미 그 자리에 있다).
  q + Enter                     → 확정하지 않고 종료 — 시작 위치로 되돌린다.

시작점 기준 누적 ±{MAX_TOTAL_OFFSET_DEG}도를 넘는 이동은 거부한다 — 오타나
큰 숫자 입력으로 갑자기 크게 튀는 걸 막는 가드레일이다. 더 필요하면
--max-offset-deg로 늘릴 것.

확정 시 이 스크립트는 소스 파일을 직접 고치지 않는다 — 최종 raw 값만
출력하니, 사람이 확인하고 다음 한 곳에 반영할 것:

    ros2_ws/src/grippers_arm/grippers_arm/floor_grasp_profiles.py 의
    IDLE_CRADLE_RAW 튜플 첫 번째 원소(servo 1 자리)를 그 값으로 교체.
    (이 튜플이 유일한 소스라 여기만 바꾸면 align_to_idle.py/데몬/
    grasp_test_console.py 등 IDLE_CRADLE_RAW를 쓰는 곳 전부에 반영된다.)
"""
import argparse
import sys

DEFAULT_PORT = "/dev/soarm"
SERVO_ID = 1
MAX_TOTAL_OFFSET_DEG = 30.0
SPEED_RAW = 150
ACCELERATION_RAW = 20

# STS3215 프로토콜 상수: 4095 raw counts == 360도. driver_sdk.STS3215Driver의
# degrees_to_position/position_to_degrees와 같은 비율이다 — 절대 기준점
# (POS_CENTER)은 필요 없다. 상대 이동량만 다루므로
# degrees_to_position(position_to_degrees(raw)+d) == raw + d*RAW_PER_DEG가
# 대수적으로 항상 성립한다(POS_CENTER가 양변에서 상쇄된다).
RAW_PER_DEG = 4095 / 360.0


def raw_after_delta(start_raw, cumulative_delta_deg):
    """순수 함수 — 하드웨어 없이 단위 테스트한다."""
    return start_raw + round(cumulative_delta_deg * RAW_PER_DEG)


def clamp_check(cumulative_delta_deg, max_offset_deg):
    """가드레일 확인. 문제 있으면 사유 문자열, 없으면 None. 순수 함수."""
    if abs(cumulative_delta_deg) > max_offset_deg:
        return (f"누적 이동량 {cumulative_delta_deg:+.1f}°가 허용치 ±{max_offset_deg}°를 "
                "초과합니다 — 적용하지 않았습니다")
    return None


def parse_command(raw_input):
    """반환: ("confirm", None) | ("quit", None) | ("move", float) | ("invalid", 원문). 순수 함수."""
    text = raw_input.strip()
    if text == "":
        return ("confirm", None)
    if text.lower() in ("q", "quit", "exit"):
        return ("quit", None)
    try:
        return ("move", float(text))
    except ValueError:
        return ("invalid", text)


def _connect(port):
    # driver_sdk(pyserial 의존)는 여기서만 import한다 — align_to_idle.py의
    # _connect()와 같은 이유로, 위의 순수 함수들은 하드웨어/pyserial 없이도
    # 단위 테스트할 수 있게 유지한다.
    import soarm_lab  # noqa: F401  (flat import를 위해 먼저 import)
    from driver_sdk import STS3215Driver

    driver = STS3215Driver(port)
    return driver if driver.connect() else None


def run(port, max_offset_deg):
    driver = _connect(port)
    if driver is None:
        print(f"[jog] 연결 실패: {port}", file=sys.stderr)
        return 1

    if not driver.ping(SERVO_ID):
        print(f"[jog] servo {SERVO_ID} 응답 없음", file=sys.stderr)
        return 1
    start_raw = driver.get_position(SERVO_ID)
    if start_raw is None:
        print(f"[jog] servo {SERVO_ID} present position 읽기 실패", file=sys.stderr)
        return 1

    # torque가 꺼져 있을 수 있으니 goal<-present로 먼저 latch(움직임 없음).
    if not driver.set_position(SERVO_ID, start_raw):
        print(f"[jog] servo {SERVO_ID} torque latch 실패", file=sys.stderr)
        return 1
    driver.set_speed(SERVO_ID, SPEED_RAW)
    driver.set_acceleration(SERVO_ID, ACCELERATION_RAW)

    cumulative_deg = 0.0
    print(f"[jog] servo {SERVO_ID} 시작 raw={start_raw}")
    print("[jog] 숫자(도) 입력 후 Enter로 이동 · 빈 입력+Enter로 확정 · q로 취소")

    while True:
        try:
            text = input(f"[누적 {cumulative_deg:+.1f}°] jog> ")
        except EOFError:
            text = "q"

        kind, value = parse_command(text)

        if kind == "invalid":
            print(f"[jog] 숫자로 해석할 수 없습니다: {value!r}")
            continue

        if kind == "quit":
            driver.set_position(SERVO_ID, start_raw)
            print(f"[jog] 취소 — 시작 위치로 복귀: raw={start_raw}")
            driver.disconnect()
            return 2

        if kind == "confirm":
            final_raw = driver.get_position(SERVO_ID)
            print(f"[jog] 확정 — 최종 raw={final_raw} (시작 대비 {cumulative_deg:+.1f}°)")
            print(f"[jog] floor_grasp_profiles.py의 IDLE_CRADLE_RAW 첫 번째 값을 "
                  f"{final_raw}로 바꾸세요.")
            driver.disconnect()
            return 0

        # kind == "move"
        trial_cumulative = cumulative_deg + value
        problem = clamp_check(trial_cumulative, max_offset_deg)
        if problem:
            print(f"[jog] {problem}")
            continue

        new_raw = raw_after_delta(start_raw, trial_cumulative)
        if not driver.set_position(SERVO_ID, new_raw):
            print("[jog] write 실패 — 이동 취소")
            continue
        cumulative_deg = trial_cumulative
        print(f"[jog] raw={new_raw}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--max-offset-deg", type=float, default=MAX_TOTAL_OFFSET_DEG)
    args = parser.parse_args(argv)
    try:
        return run(args.port, args.max_offset_deg)
    except KeyboardInterrupt:
        print("\n[jog] 중단 — 마지막 위치를 유지합니다(확정되지 않음)", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
