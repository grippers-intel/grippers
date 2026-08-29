"""재시도를 켜고 캘리브레이션한다.

## 왜 필요한가

LeRobot 의 모터 통신은 `num_retry` 기본값이 **0** 이다. 즉 **한 번 실패하면
그 자리에서 예외를 던지고 죽는다.**

    ConnectionError: Failed to write 'Torque_Enable' on id_=6 after 1 tries.
    ConnectionError: Failed to sync read 'Present_Position' ...

그런데 실측해 보니 통신 자체는 멀쩡하다 — 정지 상태 1,800회, 움직이는 중
70,260회 모두 실패 0, 전압도 11.8~11.9V 로 안정적이었다. 즉 **아주 드물게
나는 실패에 LeRobot 이 그대로 죽는 것**이 문제였다.

여기서는 `write` 와 `sync_read` 를 감싸 재시도를 넣고, 표준 캘리브레이션
절차를 그대로 실행한다. LeRobot 파일은 건드리지 않는다.

    python calibrate_retry.py follower COM8 grippers_arm
    python calibrate_retry.py leader   COM7 leader

⚠️ 대화형이다. **진짜 터미널 창**에서 실행할 것 (Enter 입력이 필요하다).
"""
import sys
import time

from lerobot.motors.feetech import FeetechMotorsBus

RETRY = 5
BACKOFF_S = 0.02


def _patch_bus() -> None:
    """write / sync_read / sync_write 에 재시도를 씌운다."""
    for name in ("write", "sync_read", "sync_write"):
        original = getattr(FeetechMotorsBus, name, None)
        if original is None:
            continue

        def make(orig, label):
            def wrapper(self, *a, **kw):
                last = None
                for attempt in range(RETRY):
                    try:
                        return orig(self, *a, **kw)
                    except ConnectionError as e:
                        last = e
                        time.sleep(BACKOFF_S)
                        if attempt == 0:
                            print(f"    [재시도] {label} 실패 — 다시 시도합니다")
                raise last
            return wrapper

        setattr(FeetechMotorsBus, name, make(original, name))


def main() -> int:
    if len(sys.argv) < 4:
        print(__doc__)
        return 2
    kind, port, dev_id = sys.argv[1], sys.argv[2], sys.argv[3]

    _patch_bus()
    print(f"재시도 {RETRY}회로 감싸고 시작합니다.\n")

    if kind == "follower":
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        dev = SO101Follower(SO101FollowerConfig(port=port, id=dev_id))
    elif kind == "leader":
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
        dev = SO101Leader(SO101LeaderConfig(port=port, id=dev_id))
    else:
        print("첫 인자는 follower 또는 leader")
        return 2

    dev.connect(calibrate=False)
    try:
        dev.calibrate()
        print("\n✅ 캘리브레이션 완료 — 파일이 저장됐습니다.")
    finally:
        dev.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
