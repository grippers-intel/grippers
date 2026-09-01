"""SO-101 서보 레지스터를 6축 비교표로 읽는다. 읽기 전용 — 아무것도 쓰지 않는다.

    lerobot-venv/Scripts/python.exe servo_regs.py

왜 이 스크립트인가
------------------
"리더는 잘 움직이는데 팔로워의 wrist_flex 만 굼뜨게 따라온다" 를 진단한다.
lerobot 이 connect 할 때마다 덮어쓰는 값은 6축이 반드시 같으므로 원인이 될 수 없다.

  덮어씀(원인 불가)  P/I/D_Coefficient, Acceleration, Maximum_Acceleration,
                     Return_Delay_Time, Operating_Mode, Min/Max_Position_Limit
  안 덮어씀(후보)    Maximum_Velocity_Limit, Torque_Limit, Max_Torque_Limit

그래서 아래 표에서 **wrist_flex(ID 4)만 다른 값**이 나오면 그게 원인이다.
"""
import sys
import serial.tools.list_ports as lp
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode

NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
REGS = [
    ("Maximum_Velocity_Limit", "안 덮어씀 ★"),
    ("Torque_Limit",           "안 덮어씀 ★"),
    ("Max_Torque_Limit",       "그리퍼만 ★"),
    ("P_Coefficient",          "6축 동일"),
    ("I_Coefficient",          "6축 동일"),
    ("D_Coefficient",          "6축 동일"),
    ("Acceleration",           "6축 동일"),
    ("Maximum_Acceleration",   "6축 동일"),
    ("Min_Position_Limit",     "calib 에서"),
    ("Max_Position_Limit",     "calib 에서"),
    ("Homing_Offset",          "calib 에서"),
    ("Present_Voltage",        "상태"),
    ("Present_Temperature",    "상태"),
    ("Present_Load",           "상태"),
    ("Torque_Enable",          "상태"),
]

def find_arms():
    out = []
    for p in sorted([p for p in lp.comports() if "CH343" in (p.description or "")],
                    key=lambda x: x.device):
        try:
            found = FeetechMotorsBus.scan_port(p.device)
        except Exception:
            continue
        if found:
            out.append((p.device, max(found)))
    return out

def dump(port, baud):
    motors = {n: Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)
              for i, n in enumerate(NAMES, start=1)}
    bus = FeetechMotorsBus(port, motors)
    bus._connect(handshake=False)
    bus.set_baudrate(baud)
    table = {}
    for reg, _ in REGS:
        row = {}
        for n in NAMES:
            try:
                row[n] = bus.read(reg, n, normalize=False)
            except Exception:
                row[n] = None
        table[reg] = row
    bus.port_handler.closePort()
    return table

def main():
    arms = find_arms()
    if not arms:
        print("모터가 응답하는 포트가 없습니다. 팔을 연결하고 전원을 켜세요.")
        return 1
    for port, baud in arms:
        t = dump(port, baud)
        tq = t["Torque_Enable"]
        v = [x for x in t["Present_Voltage"].values() if x is not None]
        on = sum(1 for x in tq.values() if x)
        # 역할은 **전압**으로 가른다. 팔로워는 12V 대, 리더는 5V 대다.
        # 토크는 전원을 껐다 켜거나 연결이 끊기면 0 이 되므로 기준이 못 된다.
        avg_v = (sum(v) / len(v) / 10) if v else 0.0
        role = ("팔로워" if avg_v >= 8.0 else "리더") if v else "?"
        vv = f"{sum(v)/len(v)/10:.1f}V" if v else "?"
        print("=" * 96)
        print(f"{port}  baud {baud}  ({role} 추정 · 평균 {vv} · 토크 {on}/6)")
        print("=" * 96)
        print(f"{'레지스터':>24} {'비고':>11} | " + " ".join(f"{n[:9]:>9}" for n in NAMES))
        print("-" * 96)
        for reg, note in REGS:
            row = t[reg]
            vals = [row[n] for n in NAMES]
            # wrist_flex 가 다른 관절(그리퍼 제외)과 다르면 표시
            others = [x for n, x in zip(NAMES, vals) if n not in ("wrist_flex", "gripper") and x is not None]
            wf = row["wrist_flex"]
            odd = wf is not None and others and all(wf != o for o in others)
            mark = "  <<< wrist_flex 만 다름" if odd else ""
            cells = " ".join(("{:>9}".format("-" if x is None else x)) for x in vals)
            print(f"{reg:>24} {note:>11} | {cells}{mark}")
        print()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
