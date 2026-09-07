"""무는 힘을 되찾는다 — servo 6 의 힘 관련 레지스터를 읽고 고친다.

## 무엇을 재고 있었나

2026-09-07 사용자: "지금 파지가 너무 약하게 되는데 토크값을 올릴 수 있을까?"
이어서: "텔레옵을 했을 때 팔로암도 리더암 같은 파지력이 나왔었는데 지금은
그만큼 안 나온다."

두 번째 문장이 답을 정했다. **같은 서보로 예전에는 셌다면 원인은 서보의
능력이 아니라 명령이다.** 그래서 lerobot 이 텔레옵 때 써 넣던 값과 지금
서보에 올라 있는 값을 전부 맞대어 봤다(lerobot so_follower.configure 와
MotorsBus.write_calibration 이 쓰는 자리들).

    레지스터                      텔레옵      지금     같은가
    21 P / 23 I / 22 D           16/0/32   16/0/32    같다
    16 Max_Torque_Limit             500      1000    지금이 2배 높다
    48 Torque_Limit               (500)      1000    지금이 2배 높다
    28 Protection_Current            250       250    같다
    36 Overload_Torque                25        25    같다
    31 Homing_Offset                 390      1343    프레임이 953 다르다
     9 Min_Position_Limit           1960      1090    ★ 여기
    11 Max_Position_Limit           2427      2090

**토크 상한은 텔레옵이 오히려 절반(500)이었는데 더 셌다.** 상한이 걸리고
있었다면 반으로 줄였을 때 약해졌어야 한다. 즉 출력은 상한 근처에도 못 갔고,
Max_Torque 를 올리는 것으로는 아무 일도 일어나지 않는다. PID 게인도 텔레옵과
글자 그대로 같다(P=16 은 Feetech 기본 32 가 아니라 **lerobot 이 일부러 쓰는
값**이다 — lekiwi.py 주석 "lower value to avoid shakiness").

## 진짜 원인 — 닫으라는 명령이 잘리고 있다

STS3215 는 위치 제어 서보다. 무는 힘은 이렇게 만들어진다:

    출력 ~= P x (목표 위치 - 현재 위치)          , 단 Torque_Limit 로 잘림

물체를 물면 턱이 두께에서 멈추므로 오차가 남고, 서보가 그 오차만큼 계속
밀어붙이는 것이 무는 힘이다. 힘의 근원은 **도달하지 못하는 거리**다.

그런데 목표는 `Min_Position_Limit`(9번) 에서 펌웨어가 잘라낸다. 두 프레임을
같은 자리로 옮겨 놓고 보면(Present_Position = Actual - Homing_Offset,
프레임 차이 390 - 1343 = -953):

    텔레옵의 하한 1960  ->  지금 프레임으로 1007
    지금의   하한                        1090     <- 83 raw 얕다

퀸을 물었을 때 실측 위치가 1190 이므로:

    텔레옵          목표 1007   위치 오차 183 raw
    지금            목표 1090   위치 오차 100 raw      (서보가 잘라낸 값)
    set_gripper(0)  목표 1106   위치 오차  84 raw      (보정표 외삽값)

**1.83 배**. 텔레옵이 세게 물던 이유가 이것이고, 토크와는 무관하다.

## 고치는 법

    python3 tools/arm/gripper_grip_gain.py --restore-teleop-limit

`Min_Position_Limit` 을 1007 로 되돌린다 — 지어낸 값이 아니라 **텔레옵이
실제로 쓰던 그 자리**(lerobot 캘리브레이션 range_min=1960)를 지금 프레임으로
옮긴 값이다. 더 깊이는 안 준다.

⚠️ 빈 턱으로 닫으면 기계 스토퍼(약 1112)를 계속 밀게 된다 — 텔레옵도 리더를
꽉 쥐면 같은 상태였으니 새로운 위험은 아니지만, 문 것 없이 오래 두지 말 것.
`park_release_torque.py` 가 토크를 푼다.

## 쓰는 법

    python3 tools/arm/gripper_grip_gain.py                    # 읽기만 (기본)
    python3 tools/arm/gripper_grip_gain.py --restore-teleop-limit
    python3 tools/arm/gripper_grip_gain.py --set-min-limit 1090   # 되돌리기
    python3 tools/arm/gripper_grip_gain.py --probe            # 실제로 물려 본다
    python3 tools/arm/gripper_grip_gain.py --set-p 32         # 2차 수단(아래)

⚠️ arm_driver 가 떠 있으면 /dev/soarm 을 배타 잠금하고 있어 실패한다.
   먼저 `tools/stop_bringup.sh` 로 내릴 것.

⚠️ 9번과 21번은 **EEPROM** 이다. 전원을 꺼도 남고, 이 팔을 쓰는 팀원 전부에게
   적용된다.

## P 게인은 왜 2차 수단인가

P 를 32(Feetech 기본값)로 올리면 같은 오차에서 힘이 2배가 된다. 다만 그것은
**텔레옵보다 세게** 만드는 것이지 텔레옵을 되찾는 것이 아니고, lerobot 이
16 을 쓰는 이유(떨림)를 도로 불러들인다. 하한을 되돌려도 모자랄 때만 쓸 것.

## --probe 로 확인하는 법

턱 사이에 물체를 넣고 실행하면 닫고 나서 위치·부하·전류·온도를 읽는다.
바꾸기 **전과 후**를 같은 물체로 재면 효과가 숫자로 남는다. 도달 위치가 더
작아지면 실제로 더 세게 조인 것이다. 온도가 55°C 를 넘으면 되돌릴 것.
"""
import argparse
import sys
import time

DEFAULT_PORT = "/dev/soarm"
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

#: lerobot 캘리브레이션(host/vla/calibration/grippers_arm.json)의 그리퍼
#: range_min 과 그때의 Homing_Offset. 텔레옵이 실제로 쓰던 자리다.
TELEOP_RANGE_MIN_RAW = 1960
TELEOP_HOMING_OFFSET = 390

#: 되돌릴 하한. 지금 프레임으로 옮긴 값은 실행 시점의 Homing_Offset 으로
#: 계산한다 — 상수로 박아 두면 오프셋이 바뀐 팔에서 조용히 틀린다.
#: (2026-09-07 실측 오프셋 1343 에서는 1007 이 나온다.)

#: 안전 울타리. 텔레옵이 쓰던 자리보다 **더 깊은** 값은 받지 않는다.
#: 그 아래는 검증된 적이 없고, 빈 턱이 스토퍼를 미는 힘만 커진다.
MIN_LIMIT_FLOOR_MARGIN_RAW = 0

ADDR_POSITION_P = 21
ADDR_MIN_POSITION_LIMIT = 9
ADDR_HOMING_OFFSET = 31
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
    """servo 6 의 레지스터 하나를 읽고 쓰는 얇은 껍데기.

    ⚠️ scservo_sdk 가 아니라 `driver_sdk.STS3215Driver` 를 쓴다 — 컨테이너에
    scservo_sdk 가 없다(2026-09-07 확인). 저장소의 다른 서보 도구
    (park_release_torque.py, reteach_idle_pose.py)와 같은 경로다.

    ⚠️ 이름 앞에 `_` 가 붙은 메서드를 부른다. 이 드라이버는 임의 주소 접근을
    공개 API 로 안 내놓는데, 여기서 필요한 것이 정확히 그것이다(P 게인은
    get/set 이 없다).
    """

    def __init__(self, port: str):
        # driver_sdk(pyserial 의존)는 여기서만 import 한다 —
        # park_release_torque.py 의 _connect() 와 같은 이유.
        import soarm_lab  # noqa: F401  (flat import 를 위해 먼저 import)
        from driver_sdk import STS3215Driver

        self._drv = STS3215Driver(port)
        if not self._drv.connect():
            raise SystemExit(
                f"{port} 연결 실패 — arm_driver 가 떠 있으면 배타 잠금 "
                "때문이다. 먼저 tools/stop_bringup.sh 로 내릴 것")

    def read(self, addr: int, size: int):
        if size == 2:
            return self._drv._read_u16(GRIPPER_ID, addr)
        data = self._drv._read(GRIPPER_ID, addr, 1)
        return None if not data else data[0]

    def write(self, addr: int, size: int, value: int) -> bool:
        if size == 2:
            return self._drv._write_u16(GRIPPER_ID, addr, value)
        return self._drv._write_u8(GRIPPER_ID, addr, value)

    def close(self) -> None:
        self._drv.disconnect()


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

    floor, homing = teleop_min_limit(link)
    low = link.read(ADDR_MIN_POSITION_LIMIT, 2)
    if floor is not None and low is not None:
        print("")
        print(f"  텔레옵이 쓰던 하한은 지금 프레임(Homing_Offset={homing})으로 "
              f"{floor} 다.")
        if low > floor:
            print(f"  지금 하한은 {low} — 닫으라는 명령이 {low - floor} raw "
                  f"얕은 데서 잘린다. 그만큼 무는 힘이 준다.")
            print("  되돌리려면: --restore-teleop-limit")
        else:
            print(f"  지금 하한은 {low} — 텔레옵과 같거나 더 깊다.")


def teleop_min_limit(link: Link):
    """텔레옵이 쓰던 하한을 **지금 서보의 프레임으로** 옮긴 값.

    Present_Position = Actual_Position - Homing_Offset 이므로, 오프셋이
    바뀌면 같은 물리 자리의 raw 가 통째로 이동한다. 그래서 상수로 박지 않고
    매번 살아 있는 오프셋을 읽어서 계산한다.
    """
    live = link.read(ADDR_HOMING_OFFSET, 2)
    if live is None:
        return None, None
    # Feetech 는 최상위 비트를 부호로 쓴다.
    signed = -(live & 0x7FFF) if live & 0x8000 else live
    return TELEOP_RANGE_MIN_RAW + (TELEOP_HOMING_OFFSET - signed), signed


def set_min_limit(link: Link, value: int) -> int:
    """Min_Position_Limit(9) 을 쓴다 — 무는 힘의 진짜 손잡이."""
    floor, homing = teleop_min_limit(link)
    if floor is None:
        print("Homing_Offset 을 못 읽었다 — 안전 하한을 계산할 수 없다")
        return 1

    before = link.read(ADDR_MIN_POSITION_LIMIT, 2)
    high = link.read(11, 2)
    print(f"Homing_Offset = {homing}  ->  텔레옵의 하한은 지금 프레임으로 {floor}")
    print(f"Min_Position_Limit: {before} -> {value}")

    if value < floor - MIN_LIMIT_FLOOR_MARGIN_RAW:
        print(f"거부 — {floor} 보다 깊다. 그 아래는 검증된 적이 없고, 빈 턱이 "
              f"기계 스토퍼를 미는 힘만 커진다.")
        return 1
    if high is not None and value >= high:
        print(f"거부 — 상한({high}) 보다 크거나 같다")
        return 1
    if before == value:
        print("이미 그 값이다 — 아무것도 안 한다.")
        return 0

    if not link.write(ADDR_LOCK, 1, 0):
        print("EEPROM 잠금 해제 실패")
        return 1
    try:
        ok = link.write(ADDR_MIN_POSITION_LIMIT, 2, value)
    finally:
        if not link.write(ADDR_LOCK, 1, 1):
            print("⚠️ EEPROM 을 다시 잠그지 못했다 — 전원을 껐다 켤 것")

    if not ok:
        print("쓰기 실패")
        return 1
    after = link.read(ADDR_MIN_POSITION_LIMIT, 2)
    if after != value:
        print(f"⚠️ 확인 실패 — 다시 읽으니 {after} 다")
        return 1
    print(f"확인: Min_Position_Limit = {after}")
    print(f"되돌리려면: --set-min-limit {before}")
    return 0


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
    parser.add_argument("--restore-teleop-limit", action="store_true",
                        help="Min_Position_Limit 을 텔레옵이 쓰던 자리로 되돌린다 "
                             "(무는 힘의 진짜 손잡이 — EEPROM)")
    parser.add_argument("--set-min-limit", type=int, metavar="RAW",
                        help="Min_Position_Limit 을 직접 쓴다 (되돌릴 때 1090)")
    parser.add_argument("--set-p", type=int, metavar="N",
                        help="2차 수단. Position_P 를 N 으로 쓴다 (EEPROM — 영구)")
    parser.add_argument("--probe", action="store_true",
                        help="실제로 닫아 보고 위치·부하·전류·온도를 읽는다")
    parser.add_argument("--settle", type=float, default=1.5,
                        help="--probe 에서 닫고 기다리는 시간(초)")
    args = parser.parse_args()

    link = Link(args.port)
    try:
        if args.restore_teleop_limit:
            floor, _homing = teleop_min_limit(link)
            if floor is None:
                print("Homing_Offset 을 못 읽었다")
                return 1
            return set_min_limit(link, floor)
        if args.set_min_limit is not None:
            return set_min_limit(link, args.set_min_limit)
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
