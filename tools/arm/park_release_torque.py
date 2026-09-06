#!/usr/bin/env python3
"""인수인계·보관용 — 팔이 IDLE 크래들에 있는지 확인하고 torque 를 푼다.

## 왜 필요한가

`fold_to_cradle` 로 IDLE 자세까지는 서비스로 갈 수 있지만, 그 뒤 **torque 를
끄는 경로가 arm_driver 에 없다.** 그래서 로봇을 넘겨줄 때마다 즉석
스크립트를 짜게 되는데, 그건 재부팅하면 사라지고 어디에도 안 남는다
(2026-09-07 인수인계에서 실제로 그럴 뻔했다).

torque 를 끄는 이유는 두 가지다.

  - IDLE 은 관절 부하가 0 인 **크래들 안착** 자세라(floor_grasp_profiles.
    IDLE_CRADLE_RAW 주석), 토크를 풀면 팔이 지령 위치에서 크래들 바닥으로
    마지막 몇 raw 를 스스로 내려앉는다. 지령으로는 못 가는 자리다.
  - 서보가 계속 전류를 쓰지 않는다. 2026-09-07 실기 뒤 servo 6 이 39°C 였다.

## 안전

⚠️ **IDLE 이 아닌 자세에서 실행하면 팔이 중력으로 쓰러진다.** servo 2-5 는
중력 부하가 있는 관절이다(reteach_idle_pose.py 상단 경고와 같은 이유).
그래서 이 도구는 먼저 IDLE 과의 차이를 재고, 허용치를 넘으면 **아무것도 하지
않고** 종료한다. 정말 그 자세에서 풀어야 하면 --force 를 준다.

⚠️ arm_driver 가 떠 있으면 /dev/soarm 을 배타 잠금하고 있어 연결이 실패한다.
먼저 arm_driver 를 내리고 실행할 것. 순서:

    1) ros2 service call /arm_driver/set_gripper ... {width_mm: 0.0}
    2) ros2 service call /arm_driver/fold_to_cradle std_srvs/srv/Trigger
    3) arm_driver 종료
    4) python3 tools/arm/park_release_torque.py

그리퍼(servo 6)도 같이 푼다 — 닫힌 채로 두면 무는 힘을 계속 유지한다.
닫아 두고 싶으면 1) 을 먼저 하라는 것이 위 순서의 뜻이고, 토크를 풀어도
턱은 그 자리에 남는다(백래시만큼만 벌어진다).
"""
import argparse
import sys

DEFAULT_PORT = "/dev/soarm"
SERVO_IDS = list(range(1, 7))

#: floor_grasp_profiles.IDLE_CRADLE_RAW 의 사본 (servo 1-5).
#: ⚠️ 저 파일이 기준이다 — 재교시하면 여기도 같이 고칠 것. 계층이 달라
#: (ros2 패키지 vs 순수 도구) import 하지 않고 복제한다.
IDLE_CRADLE_RAW = (2066, 829, 3092, 2751, 3071)

#: 이 이상 벌어져 있으면 IDLE 이 아니라고 본다. arm_driver 의 자동 정렬이
#: "이미 IDLE" 로 통과시키는 폭(±120)과 같은 자릿수로 잡되 조금 좁혔다 —
#: 여기서 틀리면 팔이 쓰러지므로 통과 조건이 더 엄해야 한다.
IDLE_TOLERANCE_RAW = 60


def _connect(port):
    # driver_sdk(pyserial 의존)는 여기서만 import 한다 — reteach_idle_pose.py
    # 의 _connect() 와 같은 이유.
    import soarm_lab  # noqa: F401  (flat import 를 위해 먼저 import)
    from driver_sdk import STS3215Driver

    driver = STS3215Driver(port)
    return driver if driver.connect() else None


def idle_offsets(positions):
    """servo 1-5 의 IDLE 대비 차이. 못 읽은 관절은 None 으로 남는다."""
    return {sid: (None if positions.get(sid) is None
                  else positions[sid] - IDLE_CRADLE_RAW[sid - 1])
            for sid in range(1, 6)}


def too_far_from_idle(offsets, tolerance=IDLE_TOLERANCE_RAW):
    """IDLE 이라고 보기 어려운 관절들. 못 읽은 것도 여기 넣는다 — 모르는
    관절을 '괜찮다'로 치면 안 된다."""
    return {sid: off for sid, off in offsets.items()
            if off is None or abs(off) > tolerance}


def run(port, force=False):
    driver = _connect(port)
    if driver is None:
        print(f"[park] 연결 실패: {port} "
              f"(arm_driver 가 떠 있으면 배타 잠금 때문이다 — 먼저 내릴 것)",
              file=sys.stderr)
        return 1

    positions = {}
    for sid in SERVO_IDS:
        positions[sid] = driver.get_position(sid) if driver.ping(sid) else None

    offsets = idle_offsets(positions)
    print(f"[park] 현재 위치: {positions}")
    print(f"[park] IDLE 대비: "
          + " ".join(f"s{sid}={'?' if off is None else format(off, '+d')}"
                     for sid, off in offsets.items()))

    bad = too_far_from_idle(offsets)
    if bad and not force:
        print(f"[park] IDLE 이 아니다 — {bad} (허용 ±{IDLE_TOLERANCE_RAW}). "
              f"토크를 풀면 팔이 중력으로 쓰러진다. 먼저 fold_to_cradle 을 "
              f"부르거나, 정말 여기서 풀려면 --force 를 줄 것.", file=sys.stderr)
        driver.disconnect()
        return 2
    if bad:
        print(f"[park] ⚠️ --force — IDLE 이 아닌데 푼다: {bad}. 팔을 받칠 것.")

    failed = []
    for sid in SERVO_IDS:
        if not driver.set_torque(sid, False):
            failed.append(sid)

    if failed:
        print(f"[park] torque 해제 실패: servo {failed} — 나머지는 풀렸다. "
              f"팔을 받친 채 다시 실행할 것.", file=sys.stderr)
        driver.disconnect()
        return 1

    print(f"[park] servo {SERVO_IDS} torque 해제 완료 — 크래들에 얹혔다.")
    driver.disconnect()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--force", action="store_true",
                        help="IDLE 이 아니어도 푼다 — 팔을 손으로 받친 경우에만")
    args = parser.parse_args(argv)
    return run(args.port, force=args.force)


if __name__ == "__main__":
    raise SystemExit(main())
