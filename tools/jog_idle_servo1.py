#!/usr/bin/env python3
"""servo 1(Base) IDLE 자세 미세 조정 콘솔 — 그리퍼가 정면을 보도록 손으로 돌려 맞출 때 쓴다.

**servo 1만 건드린다.** 다른 관절(2-6)은 이 스크립트가 전혀 손대지 않는다.

손으로 직접 돌리는 방식이다(자동 구동 아님) — servo 1은 베이스 회전(yaw)
축이라 중력 부하가 없어 torque를 꺼도 팔이 처지지 않는다.

조작:
  Enter (첫 번째)  → servo 1 torque 해제. 이제 손으로 자유롭게 돌아간다.
  (손으로 원하는 자세로 돌린다)
  Enter (두 번째)  → 그 순간의 위치를 goal로 write해 torque를 다시 켠다
                      (STS3215는 goal write 시 torque가 자동으로 켜진다 —
                      align_to_idle.py 모듈 docstring 참고). goal==현재
                      위치이므로 확정 순간 튀는 움직임은 없다.
  q                → 첫 Enter 이후 아무 때나 입력하면 원래 시작 위치로
                      되돌리고 그 자리에서 torque를 다시 켠다(자동 구동으로
                      복귀 — 취소하고 싶을 때 손을 뗀 채로 입력할 것).

확정 시 이 스크립트는 소스 파일을 직접 고치지 않는다 — 최종 raw 값만
출력하니, 사람이 확인하고 다음 한 곳에 반영할 것:

    ros2_ws/src/grippers_arm/grippers_arm/floor_grasp_profiles.py 의
    IDLE_CRADLE_RAW 튜플 첫 번째 원소(servo 1 자리)를 그 값으로 교체.
    (이 튜플이 유일한 소스라 여기만 바꾸면 align_to_idle.py/데몬/
    grasp_test_console.py 등 IDLE_CRADLE_RAW를 쓰는 곳 전부에 반영된다.)
"""
import sys

DEFAULT_PORT = "/dev/soarm"
SERVO_ID = 1


def _connect(port):
    # driver_sdk(pyserial 의존)는 여기서만 import한다 — align_to_idle.py의
    # _connect()와 같은 이유.
    import soarm_lab  # noqa: F401  (flat import를 위해 먼저 import)
    from driver_sdk import STS3215Driver

    driver = STS3215Driver(port)
    return driver if driver.connect() else None


def run(port):
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

    print(f"[jog] servo {SERVO_ID} 현재 raw={start_raw}")
    try:
        text = input("torque 해제하려면 Enter (취소하려면 q) > ")
    except EOFError:
        text = "q"
    if text.strip().lower() in ("q", "quit", "exit"):
        print("[jog] 아무것도 하지 않고 종료")
        driver.disconnect()
        return 2

    if not driver.set_torque(SERVO_ID, False):
        print(f"[jog] servo {SERVO_ID} torque 해제 실패", file=sys.stderr)
        driver.disconnect()
        return 1
    print(f"[jog] servo {SERVO_ID} torque 해제됨 — 손으로 원하는 자세로 돌리세요")

    try:
        text = input("확정하려면 Enter (원위치로 되돌리려면 q) > ")
    except EOFError:
        text = "q"

    if text.strip().lower() in ("q", "quit", "exit"):
        driver.set_position(SERVO_ID, start_raw)
        print(f"[jog] 취소 — 시작 위치로 복귀: raw={start_raw}")
        driver.disconnect()
        return 2

    current_raw = driver.get_position(SERVO_ID)
    if current_raw is None:
        print(f"[jog] servo {SERVO_ID} present position 읽기 실패 — 확정 중단", file=sys.stderr)
        driver.disconnect()
        return 1

    if not driver.set_position(SERVO_ID, current_raw):
        print(f"[jog] servo {SERVO_ID} torque 재latch 실패", file=sys.stderr)
        driver.disconnect()
        return 1

    print(f"[jog] 확정 — 최종 raw={current_raw} (시작 대비 {current_raw - start_raw:+d})")
    print(f"[jog] floor_grasp_profiles.py의 IDLE_CRADLE_RAW 첫 번째 값을 "
          f"{current_raw}로 바꾸세요.")
    driver.disconnect()
    return 0


def main(argv=None):
    port = argv[0] if argv else DEFAULT_PORT
    try:
        return run(port)
    except KeyboardInterrupt:
        print("\n[jog] 중단됨", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
