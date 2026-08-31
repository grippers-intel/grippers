"""Feetech 버스 읽기·쓰기에 재시도를 씌운다.

## 왜 필요한가

LeRobot 의 모터 통신은 `num_retry` 기본값이 **0** 이다. 즉 **한 번 실패하면
그 자리에서 예외를 던지고 죽는다.**

    ConnectionError: Failed to write 'Torque_Enable' on id_=5 after 1 tries.
                     [TxRxResult] Incorrect status packet!

그런데 실측해 보니 통신 자체는 멀쩡하다 — 정지 상태 1,800회, 움직이는 중
70,260회 모두 실패 0, 전압도 11.8~11.9V 로 안정적이었다. 즉 **아주 드물게
나는 실패에 LeRobot 이 그대로 죽는 것**이 문제다.

## 왜 한 파일에 모았나

`calibrate_retry.py` 와 `record_retry.py` 가 같은 패치를 각각 들고 있었고, 두
파일 모두 "한쪽만 고치는 일이 없도록 둘을 같이 볼 것"이라고 적어 두었다.
`rollout_policy.py` 가 세 번째 사본을 만들 차례가 되어 여기로 모은다.
고칠 곳이 하나면 어긋날 일이 없다.

## 쓰는 법

    from feetech_retry import patch_bus
    patch_bus()          # LeRobot 을 import 하기 전이든 후든 상관없다

LeRobot 파일은 건드리지 않는다. 클래스 메서드만 감싼다.
"""

import time

RETRY = 5
BACKOFF_S = 0.02


def patch_bus(retry: int = RETRY, backoff_s: float = BACKOFF_S, verbose: bool = True) -> int:
    """`write` / `sync_read` / `sync_write` 에 재시도를 씌운다.

    Returns:
        감싼 메서드 개수. 두 번 불러도 중복해서 감싸지 않는다.
    """
    from lerobot.motors.feetech import FeetechMotorsBus

    wrapped = 0
    for name in ("write", "sync_read", "sync_write"):
        original = getattr(FeetechMotorsBus, name, None)
        if original is None or getattr(original, "_retry_wrapped", False):
            continue

        def make(orig, label):
            def wrapper(self, *a, **kw):
                last = None
                for attempt in range(retry):
                    try:
                        return orig(self, *a, **kw)
                    except ConnectionError as e:
                        last = e
                        time.sleep(backoff_s)
                        if attempt == 0 and verbose:
                            print(f"    [재시도] {label} 실패 — 다시 시도합니다")
                raise last

            wrapper._retry_wrapped = True
            return wrapper

        setattr(FeetechMotorsBus, name, make(original, name))
        wrapped += 1

    return wrapped
