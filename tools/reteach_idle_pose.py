#!/usr/bin/env python3
"""IDLE 자세 전체 재설정 콘솔 — servo 1-5 전부 torque 해제 후 손으로 재포즈.

⚠️ servo1(Base, yaw)과 달리 servo2-5(Shoulder/Elbow/Wrist Pitch/Wrist Roll)는
중력 부하가 있는 관절이다. 다섯 개를 동시에 torque 해제하면 팔이 자체 무게로
쓰러질 수 있다 — **Enter를 누르기 전에 반드시 팔을 손으로 받치고 있을 것.**

servo 6(그리퍼)은 다루지 않는다. IDLE_CRADLE_RAW는 servo 1-5 raw 5개
튜플이고, 그리퍼의 IDLE 목표(CLOSED)는 실측 mm 보정표
(gripper_calibration.GRIPPER_CLOSED_MM)에서 따로 나온다 — 손으로 대충 쥔
위치를 "닫힘"으로 재정의하면 그 보정표와 어긋나게 된다.

조작:
  Enter (첫 번째) → servo 1-5 전부 torque 해제(경고 출력 후). 손으로 팔
                     전체를 원하는 IDLE 자세로 옮긴다(계속 받치고 있을 것).
  Enter (두 번째) → 그 순간 각 관절의 위치를 goal로 write해 torque를 다시
                     켠다(STS3215는 goal write 시 torque가 자동으로
                     켜진다 — align_to_idle.py 모듈 docstring 참고).
                     goal==present이므로 확정 순간 튀는 움직임은 없다.
  q                → 첫 Enter 이후 아무 때나 입력하면 5관절 전부 원래 시작
                     위치로 되돌린다 — **이건 자동 구동이라 팔이 스스로
                     움직인다**, 취소할 땐 손을 뗀 채로 입력할 것.

중간에 일부 관절만 torque 해제/재latch에 실패하면, 이미 처리된 관절은
안전하게 시작 위치로 다시 잠그거나 그 사실을 명확히 출력한다 — 일부만
풀린 채 조용히 끝나는 상황을 피하기 위함이다.

확정 시 이 스크립트는 소스 파일을 직접 고치지 않는다 — 최종 raw 5개를
출력하니 사람이 확인하고 다음 한 곳에 반영할 것:

    ros2_ws/src/grippers_arm/grippers_arm/floor_grasp_profiles.py 의
    IDLE_CRADLE_RAW 튜플 전체를 그 값으로 교체.
"""
import sys

DEFAULT_PORT = "/dev/soarm"
SERVO_IDS = list(range(1, 6))


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
        print(f"[reteach] 연결 실패: {port}", file=sys.stderr)
        return 1

    start = {}
    for sid in SERVO_IDS:
        if not driver.ping(sid):
            print(f"[reteach] servo {sid} 응답 없음 — 중단", file=sys.stderr)
            return 1
        pos = driver.get_position(sid)
        if pos is None:
            print(f"[reteach] servo {sid} present position 읽기 실패 — 중단", file=sys.stderr)
            return 1
        start[sid] = pos

    print(f"[reteach] 시작 위치: {start}")
    print("[reteach] ⚠️  Enter를 누르면 servo 1-5 전부 torque가 풀립니다.")
    print("[reteach] ⚠️  servo 2-5는 중력 부하가 있습니다 — 팔을 손으로 받친 뒤 Enter 하세요.")
    try:
        text = input("모두 받쳤으면 Enter (취소하려면 q) > ")
    except EOFError:
        text = "q"
    if text.strip().lower() in ("q", "quit", "exit"):
        print("[reteach] 아무것도 하지 않고 종료")
        driver.disconnect()
        return 2

    released = []
    for sid in SERVO_IDS:
        if not driver.set_torque(sid, False):
            print(
                f"[reteach] servo {sid} torque 해제 실패 — 이미 푼 관절 {released}은 "
                "안전을 위해 시작 위치로 다시 잠급니다",
                file=sys.stderr,
            )
            for done_sid in released:
                driver.set_position(done_sid, start[done_sid])
            driver.disconnect()
            return 1
        released.append(sid)
    print("[reteach] servo 1-5 torque 해제됨 — 손으로 원하는 IDLE 자세로 옮기세요")

    try:
        text = input("확정하려면 Enter (원위치로 되돌리려면 q) > ")
    except EOFError:
        text = "q"

    if text.strip().lower() in ("q", "quit", "exit"):
        for sid in SERVO_IDS:
            driver.set_position(sid, start[sid])
        print(f"[reteach] 취소 — 시작 위치로 복귀: {start}")
        driver.disconnect()
        return 2

    final = {}
    for sid in SERVO_IDS:
        pos = driver.get_position(sid)
        if pos is None:
            print(
                f"[reteach] servo {sid} present position 읽기 실패 — 확정 중단, "
                "나머지 관절은 여전히 torque가 풀린 상태입니다. 손으로 받치세요",
                file=sys.stderr,
            )
            driver.disconnect()
            return 1
        final[sid] = pos

    for i, sid in enumerate(SERVO_IDS):
        if not driver.set_position(sid, final[sid]):
            remaining = SERVO_IDS[i:]
            print(
                f"[reteach] servo {sid} torque 재latch 실패 — {remaining} 관절은 "
                "여전히 torque가 풀린 상태입니다. 손으로 받치세요",
                file=sys.stderr,
            )
            driver.disconnect()
            return 1

    ordered = tuple(final[sid] for sid in SERVO_IDS)
    print(f"[reteach] 확정 — 최종 raw: {final}")
    print("[reteach] floor_grasp_profiles.py의 IDLE_CRADLE_RAW를 다음으로 바꾸세요:")
    print(f"    IDLE_CRADLE_RAW = {ordered}")
    driver.disconnect()
    return 0


def main(argv=None):
    port = argv[0] if argv else DEFAULT_PORT
    try:
        return run(port)
    except KeyboardInterrupt:
        print(
            "\n[reteach] 중단됨 — 중력 부하 관절이 torque 풀린 채일 수 있습니다. 손으로 받치세요.",
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
