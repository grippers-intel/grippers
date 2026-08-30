"""재시도를 켜고 시연을 녹화한다 — `lerobot-record` 를 감싼 것.

## 왜 필요한가

`calibrate_retry.py` 와 **같은 이유**다. LeRobot 의 모터 통신은 `num_retry`
기본값이 0 이라 **한 번 실패하면 그 자리에서 죽는다.** 캘리브레이션에서
두 번 그렇게 죽었고, 실측해 보니 통신 자체는 멀쩡했다 — 팔을 움직이는 중
70,260회 읽기에 실패 0, 전압 11.8~11.9V 안정.

캘리브레이션은 죽어도 다시 하면 그만이지만, **녹화는 다르다.** 25분짜리
30 에피소드를 찍는 도중에 죽으면 그때까지가 날아간다(`--resume=true` 로
이어붙일 수는 있지만 그 에피소드는 버려진다). 그래서 처음부터 감싼다.

## 쓰는 법

`lerobot-record` 의 인자를 **그대로** 준다.

    python tools/arm/record_retry.py \
      --robot.type=so101_follower --robot.port=COM8 --robot.id=grippers_arm \
      --dataset.repo_id=lsy0284/gripper_pick_v1 \
      --dataset.single_task="pick up the queen" ...

⚠️ 대화형이다. **진짜 터미널 창**에서 실행할 것.
"""
import sys
import time

from lerobot.motors.feetech import FeetechMotorsBus

RETRY = 5
BACKOFF_S = 0.02


def _patch_bus() -> None:
    """write / sync_read / sync_write 에 재시도를 씌운다.

    calibrate_retry.py 와 같은 패치다. 한쪽만 고치는 일이 없도록 둘을 같이
    볼 것.
    """
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
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    _patch_bus()
    print(f"모터 통신을 재시도 {RETRY}회로 감쌌습니다.\n")

    # draccus 가 sys.argv 를 그대로 읽으므로 우리 인자를 그 자리에 둔다.
    from lerobot.scripts.lerobot_record import main as record_main
    sys.argv = [sys.argv[0]] + sys.argv[1:]
    record_main()
    return 0


if __name__ == "__main__":
    sys.exit(main())
