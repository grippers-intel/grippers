#!/usr/bin/env python3
"""무는 힘을 조절한다 — servo 6 의 힘 관련 레지스터를 읽고 P 게인을 바꾼다.

## 왜 이 도구인가

2026-09-07 사용자: "지금 파지가 너무 약하게 되는데 토크값을 올릴 수 있을까?"

레지스터를 읽어 보니 **토크는 이미 최대였다.**

    Max_Torque(16)    1000    (=100%)
    Torque_Limit(48)  1000    (=100%)

둘 다 상한이므로 "토크값을 올린다"로는 더 낼 것이 없다. 그런데 무는 힘이
약하다면 원인은 상한이 아니라 **상한까지 안 올라가는 것**이다.

STS3215 는 위치 제어 서보다. 무는 힘은 이렇게 만들어진다:

    출력 ~= P x (목표 위치 - 현재 위치)          , 단 Torque_Limit 로 잘림

즉 힘의 근원은 **도달하지 못하는 거리**(위치 오차)이고, 그것을 힘으로
바꾸는 배율이 P 다. 물체를 물면 턱이 물체 두께에서 멈추므로 오차가 생기고,
서보는 그 오차만큼 계속 밀어붙인다. 그것이 무는 힘이다.

지금 값:

    목표      1106 raw   (set_gripper(0.0mm) -> gripper_calibration
                          .position_from_width 가 첫 구간을 외삽한 값)
    퀸 물었을 때 1190 raw
    오차       84 raw
    P(21)      16        <- Feetech STS3215 기본값은 보통 32 다

P 가 기본값의 절반이다. 오차 84 는 그대로인데 배율만 절반이니 힘도 절반이다.
출력이 Torque_Limit(1000)에 붙어 있었다면 "약하다"는 증상이 안 나온다 —
즉 지금은 포화 전이고, P 를 올리면 그만큼 힘이 는다.

## 다른 두 방법을 왜 안 쓰는가

  오차를 키운다     더 깊이 닫으라고 명령하면 오차가 커진다. 그런데 목표는
                    Min_Angle_Limit(9번, 지금 1090)에서 서보가 잘라낸다.
                    지금 목표 1106 이니 여유가 16 raw 뿐이라 의미가 없고,
                    1090 자체를 내리는 것은 **빈 턱으로 닫을 때** 턱이 제
                    기계 스토퍼를 계속 밀게 된다는 뜻이라 발열이 는다.
  Punch 를 올린다   최소 기동력(24번)은 오차가 작을 때도 바닥 출력을 준다.
                    정지 마찰을 이기는 값이라, 무는 힘(오차가 큰 구간)에는
                    거의 기여하지 않으면서 미세 위치에서 떨림만 는다.

## 쓰는 법

    python3 tools/arm/gripper_grip_gain.py                # 읽기만 (기본)
    python3 tools/arm/gripper_grip_gain.py --set-p 32     # P 를 32 로
    python3 tools/arm/gripper_grip_gain.py --set-p 16     # 되돌리기
    python3 tools/arm/gripper_grip_gain.py --probe        # 실제로 물려 본다

⚠️ arm_driver 가 떠 있으면 /dev/soarm 을 배타 잠금하고 있어 실패한다.
   먼저 `tools/stop_bringup.sh` 로 내릴 것.

⚠️ P 는 **EEPROM** 이다. 전원을 꺼도 남고, 이 팔을 쓰는 팀원 전부에게
   적용된다. 되돌리려면 위의 `--set-p 16`.

## --probe 로 확인하는 법

턱 사이에 물체를 넣고 실행하면 닫고 나서 위치·부하·전류·온도를 읽는다.
P 를 바꾸기 **전과 후**를 같은 물체로 재면 효과가 숫자로 남는다.

    (전)  P=16  도달 1190  전류 ...
    (후)  P=32  도달 ....  전류 ...

도달 위치가 더 작아지면(더 깊이 물면) 실제로 더 세게 조인 것이다.
온도가 55°C 를 넘으면 즉시 되돌릴 것 — 서보가 과열 래치로 죽는다.
"""
import argparse
import sys
import time

DEFAULT_PORT = "/dev/soarm"
BAUD = 1_000_000
GRIPPER_ID = 6

#: set_gripper(0.0mm) 이 만드는 목표. gripper_calibration.position_from_width
#: 가 첫 보정점(9.0mm, 1150)의 기울기를 0mm 까지 외삽한 값이다.
FULL_CLOSE_RAW = 1106

#: (주소, 이름, 바이트수, 설명)
REGISTERS = (
    (9,  "Min_Angle_Limit", 2, "목표 위치의 하한 — 이보다 깊이는 명령해도 잘린다"),
    (11, "Max_Angle_Limit", 2, "목표 위치의 상한"),
    (16, "Max_Torque",      2, "토크 상한 (1000 = 100%)"),
    (21, "Position_P",      1, "* 무는 힘의 배율. 오차 1 raw 당 출력"),
    (22, "Position_D",      1, "감쇠 — 크면 도달이 느려지고 떨림이 준다"),
    (23, "Position_I",      1, "적분 — 0 이면 정상 오차를 안 없앤다(파지에서는 그게 맞다)"),
    (24, "Punch",           2, "최소 기동력 — 오차가 작을 때의 바닥 출력"),
    (28, "보호 전류",       2, "이 전류를 넘으면 보호가 걸린다"),
    (48, "Torque_Limit",    2, "실시간 토크 상한 (1000 = 100%)"),
)

ADDR_POSITION_P = 21
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED = 46
ADDR_LOCK = 55
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_PRESENT_CURRENT = 69


class Link:
    def __init__(self, port: str):
        import scservo_sdk as scs

        self._scs = scs
        self._ph = scs.PortHandler(port)
        if not self._ph.openPort():
            raise SystemExit(
                f"{port} 열기 실패 — arm_driver 가 떠 있으면 먼저 내릴 것 "
                "(tools/stop_bringup.sh)")
        self._ph.setBaudRate(BAUD)
        self._pk = scs.PacketHandler(0)

    def read(self, addr: int, size: int):
        fn = self._pk.read1ByteTxRx if size == 1 else self._pk.read2ByteTxRx
        value, comm, err = fn(self._ph, GRIPPER_ID, addr)
        if comm != self._scs.COMM_SUCCESS or err != 0:
            return None
        return value

    def write(self, addr: int, size: int, value: int) -> bool:
        fn = self._pk.write1ByteTxRx if size == 1 else self._pk.write2ByteTxRx
        comm, err = fn(self._ph, GRIPPER_ID, addr, value)
        return comm == self._scs.COMM_SUCCESS and err == 0

    def close(self) -> None:
        self._ph.closePort()


def signed_load(raw) -> str:
    """Present_Load 는 10비트 크기 + 방향비트(10번)다."""
    if raw is None:
        return "읽기 실패"
    magnitude = raw & 0x3FF
    sign = "-" if raw & 0x400 else "+"
    return f"{sign}{magnitude} ({magnitude / 1000 * 100:.1f}%)"


def show(link: Link) -> None:
    print(f"servo {GRIPPER_ID} (그리퍼) — 힘 관련 레지스터\n")
    for addr, name, size, why in REGISTERS:
        value = link.read(addr, size)
        shown = "읽기 실패" if value is None else str(value)
        print(f"  {addr:>3}  {name:<16} {shown:>9}   {why}")

    print("\n현재 상태")
    position = link.read(ADDR_PRESENT_POSITION, 2)
    print(f"  위치      {position}")
    print(f"  부하      {signed_load(link.read(ADDR_PRESENT_LOAD, 2))}")
    current = link.read(ADDR_PRESENT_CURRENT, 2)
    if current is not None:
        print(f"  전류      {current} ({current * 6.5:.0f}mA)")
    voltage = link.read(ADDR_PRESENT_VOLTAGE, 1)
    if voltage is not None:
        print(f"  전압      {voltage / 10:.1f}V")
    temperature = link.read(ADDR_PRESENT_TEMPERATURE, 1)
    if temperature is not None:
        print(f"  온도      {temperature}°C")

    gain = link.read(ADDR_POSITION_P, 1)
    if position is not None and gain is not None:
        error = FULL_CLOSE_RAW - position
        print(f"\n  0mm 로 닫으라고 하면 목표는 {FULL_CLOSE_RAW} raw 다.")
        if error < 0:
            print(f"  지금 위치({position})는 그보다 {-error} raw 벌어져 있다 — "
                  f"물고 있다면 그 오차가 곧 무는 힘이고, 배율은 P={gain} 이다.")
        else:
            print("  지금은 이미 그 안쪽이라 오차가 없다 — 빈 턱이거나 열려 있다.")


def set_gain(link: Link, value: int) -> int:
    if not 0 <= value <= 254:
        print(f"P 는 0~254 여야 한다: {value}")
        return 1

    before = link.read(ADDR_POSITION_P, 1)
    print(f"Position_P: {before} -> {value}")
    if before == value:
        print("이미 그 값이다 — 아무것도 안 한다.")
        return 0

    # EEPROM 은 Lock 을 풀어야 쓰인다. 무슨 일이 있어도 다시 잠근다 —
    # 풀린 채로 두면 이후의 어떤 쓰기도 조용히 EEPROM 을 갉는다.
    if not link.write(ADDR_LOCK, 1, 0):
        print("EEPROM 잠금 해제 실패")
        return 1
    try:
        ok = link.write(ADDR_POSITION_P, 1, value)
    finally:
        if not link.write(ADDR_LOCK, 1, 1):
            print("⚠️ EEPROM 을 다시 잠그지 못했다 — 전원을 껐다 켤 것")

    if not ok:
        print("쓰기 실패")
        return 1

    after = link.read(ADDR_POSITION_P, 1)
    if after != value:
        print(f"⚠️ 확인 실패 — 다시 읽으니 {after} 다")
        return 1
    print(f"확인: Position_P = {after}")
    print(f"되돌리려면: --set-p {before}")
    return 0


def probe(link: Link, settle_s: float) -> int:
    print("⚠️ 턱을 완전히 닫습니다. 물체가 물려 있지 않으면 빈 턱이 "
          "기계 스토퍼에 닿습니다.\n")
    if not link.write(ADDR_TORQUE_ENABLE, 1, 1):
        print("토크를 켜지 못했다")
        return 1
    link.write(ADDR_GOAL_SPEED, 2, 600)
    if not link.write(ADDR_GOAL_POSITION, 2, FULL_CLOSE_RAW):
        print("목표 위치를 쓰지 못했다")
        return 1

    time.sleep(settle_s)
    gain = link.read(ADDR_POSITION_P, 1)
    position = link.read(ADDR_PRESENT_POSITION, 2)
    current = link.read(ADDR_PRESENT_CURRENT, 2)
    temperature = link.read(ADDR_PRESENT_TEMPERATURE, 1)

    print(f"  P          {gain}")
    print(f"  목표       {FULL_CLOSE_RAW}")
    print(f"  도달       {position}")
    if position is not None:
        print(f"  위치 오차  {position - FULL_CLOSE_RAW} raw   <- 이것이 힘의 근원")
    print(f"  부하       {signed_load(link.read(ADDR_PRESENT_LOAD, 2))}")
    if current is not None:
        print(f"  전류       {current} ({current * 6.5:.0f}mA)")
    if temperature is not None:
        print(f"  온도       {temperature}°C"
              + ("   ⚠️ 55°C 를 넘었다 — P 를 되돌릴 것" if temperature > 55 else ""))
    print("\n토크는 켠 채로 둡니다 — 물고 있는 것을 떨어뜨리지 않기 위해서다.")
    print("풀려면: python3 tools/arm/park_release_torque.py")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="그리퍼(servo 6)의 무는 힘 관련 레지스터를 읽고 P 를 바꾼다")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--set-p", type=int, metavar="N",
                        help="Position_P 를 N 으로 쓴다 (EEPROM — 영구)")
    parser.add_argument("--probe", action="store_true",
                        help="실제로 닫아 보고 위치·부하·전류·온도를 읽는다")
    parser.add_argument("--settle", type=float, default=1.5,
                        help="--probe 에서 닫고 기다리는 시간(초)")
    args = parser.parse_args()

    link = Link(args.port)
    try:
        if args.set_p is not None:
            return set_gain(link, args.set_p)
        if args.probe:
            return probe(link, args.settle)
        show(link)
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
