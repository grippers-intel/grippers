"""IDLE 자세 재교시 — 윈도우판. `tools/reteach_idle_pose.py` 와 같은 절차다.

## 왜 따로 있나

원본은 Pi 의 `driver_sdk`(`/dev/soarm`)를 쓴다. 그런데 시연 수집을 노트북에서
하기로 하면서 **팔로워암이 노트북(COM8)에 붙어 있다.** Pi 에는 차체 보드
(`1a86:55d4` → `/dev/rrc`)와 라이다만 남아 있어 원본을 돌릴 수가 없다.

raw 카운트는 서보 안에서 나오는 값이라 **어느 호스트에서 읽든 같다.**
여기서 잡은 값을 그대로 Pi 의 `floor_grasp_profiles.py` 에 넣으면 된다.

## 절차 (원본과 동일)

    Enter(1회)  servo 1-5 torque 해제       ← 누르기 전에 팔을 받칠 것
                손으로 원하는 IDLE 자세로 옮긴다 (계속 받친 채)
    Enter(2회)  그 위치를 goal 로 써서 torque 재점등
    q           첫 Enter 뒤 아무 때나 → 시작 위치로 자동 복귀
                (팔이 스스로 움직인다. 손 떼고 입력할 것)

⚠️ **servo 2-5 는 중력 부하 관절이다.** 다섯을 동시에 풀면 팔이 자기 무게로
쓰러진다. 혼자 하지 말 것 — 한 손으로 팔을 받치고 다른 손으로 Enter 를 친다.

⚠️ servo 6(그리퍼)은 건드리지 않는다. 그리퍼의 IDLE 목표는 실측 mm 보정표
(`gripper_calibration.GRIPPER_CLOSED_MM`)에서 따로 나온다 — 손으로 대충 쥔
위치를 "닫힘"으로 재정의하면 그 표와 어긋난다.

    python tools/arm/reteach_idle_win.py COM8

대화형이라 **진짜 터미널 창**에서 실행할 것.
"""
import sys
import time

import scservo_sdk as scs

SERVO_IDS = [1, 2, 3, 4, 5]
NAMES = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
         4: "wrist_flex", 5: "wrist_roll"}

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56
ADDR_MODEL_NUMBER = 3

RETRY = 5          # calibrate_retry.py 와 같은 이유 — 기본 재시도가 0이다
BACKOFF_S = 0.02


class Bus:
    def __init__(self, port):
        self.ph = scs.PortHandler(port)
        self.pk = scs.PacketHandler(0)
        if not self.ph.openPort():
            raise SystemExit(f"{port} 열기 실패 — 다른 프로그램이 잡고 있나요?")
        self.ph.setBaudRate(1000000)

    def _retry(self, fn):
        for attempt in range(RETRY):
            v, comm, err = fn()
            if comm == scs.COMM_SUCCESS and err == 0:
                return v
            time.sleep(BACKOFF_S)
        return None

    def ping(self, sid):
        return self._retry(lambda: self.pk.read2ByteTxRx(
            self.ph, sid, ADDR_MODEL_NUMBER)) is not None

    def get_position(self, sid):
        return self._retry(lambda: self.pk.read2ByteTxRx(
            self.ph, sid, ADDR_PRESENT_POSITION))

    def set_torque(self, sid, on):
        for _ in range(RETRY):
            comm, err = self.pk.write1ByteTxRx(
                self.ph, sid, ADDR_TORQUE_ENABLE, 1 if on else 0)
            if comm == scs.COMM_SUCCESS and err == 0:
                return True
            time.sleep(BACKOFF_S)
        return False

    def set_position(self, sid, pos):
        """goal 을 쓰면 STS3215 는 torque 를 자동으로 켠다.

        goal == present 로 쓰므로 확정 순간 튀는 움직임은 없다."""
        for _ in range(RETRY):
            comm, err = self.pk.write2ByteTxRx(
                self.ph, sid, ADDR_GOAL_POSITION, int(pos))
            if comm == scs.COMM_SUCCESS and err == 0:
                return True
            time.sleep(BACKOFF_S)
        return False

    def close(self):
        self.ph.closePort()


def _ask(prompt):
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return "q"


def run(port):
    bus = Bus(port)

    start = {}
    for sid in SERVO_IDS:
        if not bus.ping(sid):
            print(f"[reteach] servo {sid} 응답 없음 — 중단", file=sys.stderr)
            bus.close()
            return 1
        pos = bus.get_position(sid)
        if pos is None:
            print(f"[reteach] servo {sid} 위치 읽기 실패 — 중단", file=sys.stderr)
            bus.close()
            return 1
        start[sid] = pos

    print("[reteach] 시작 위치")
    for sid in SERVO_IDS:
        print(f"    servo {sid} {NAMES[sid]:15s} {start[sid]}")
    print()
    print("[reteach] ⚠️  Enter 를 누르면 servo 1-5 의 torque 가 전부 풀립니다.")
    print("[reteach] ⚠️  servo 2-5 는 중력 부하 관절입니다 — 팔을 손으로 받친 뒤 Enter.")

    if _ask("모두 받쳤으면 Enter (취소하려면 q) > ") in ("q", "quit", "exit"):
        print("[reteach] 아무것도 하지 않고 종료")
        bus.close()
        return 2

    released = []
    for sid in SERVO_IDS:
        if not bus.set_torque(sid, False):
            print(f"[reteach] servo {sid} torque 해제 실패 — 이미 푼 {released} 는 "
                  "안전을 위해 시작 위치로 다시 잠급니다", file=sys.stderr)
            for done in released:
                bus.set_position(done, start[done])
            bus.close()
            return 1
        released.append(sid)

    print("[reteach] torque 해제됨 — 손으로 원하는 IDLE 자세로 옮기세요")

    if _ask("확정하려면 Enter (원위치로 되돌리려면 q) > ") in ("q", "quit", "exit"):
        for sid in SERVO_IDS:
            bus.set_position(sid, start[sid])
        print(f"[reteach] 취소 — 시작 위치로 복귀: {start}")
        bus.close()
        return 2

    final = {}
    for sid in SERVO_IDS:
        pos = bus.get_position(sid)
        if pos is None:
            print(f"[reteach] servo {sid} 위치 읽기 실패 — 확정 중단. 나머지 관절은 "
                  "여전히 torque 가 풀려 있습니다. 손으로 받치세요", file=sys.stderr)
            bus.close()
            return 1
        final[sid] = pos

    for i, sid in enumerate(SERVO_IDS):
        if not bus.set_position(sid, final[sid]):
            print(f"[reteach] servo {sid} torque 재latch 실패 — {SERVO_IDS[i:]} 관절은 "
                  "여전히 풀려 있습니다. 손으로 받치세요", file=sys.stderr)
            bus.close()
            return 1

    ordered = tuple(final[sid] for sid in SERVO_IDS)
    print("\n[reteach] 확정")
    for sid in SERVO_IDS:
        d = final[sid] - start[sid]
        print(f"    servo {sid} {NAMES[sid]:15s} {final[sid]:5d}   (시작 대비 {d:+d})")
    print("\n[reteach] floor_grasp_profiles.py 의 IDLE_CRADLE_RAW 를 다음으로 바꾸세요:")
    print(f"    IDLE_CRADLE_RAW = {ordered}")
    bus.close()
    return 0


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    return run(argv[0])


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
