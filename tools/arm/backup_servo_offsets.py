"""캘리브레이션 전에 서보 EEPROM 값을 백업한다.

## 왜 필요한가

LeRobot 캘리브레이션은 파일만 쓰는 게 아니라 **서보 안의 `Homing_Offset` 을
직접 덮어쓴다**(`lerobot/motors/feetech/feetech.py:275`).

    Present_Position = Actual_Position - Homing_Offset

그런데 이 팔에는 그리퍼 프로젝트의 교시 자세가 **RAW 서보값**으로 박혀 있다
(`floor_grasp_profiles.py` 의 HORIZONTAL_SAFE_145_RAW · IDLE_CRADLE_RAW ·
CARRY_RAW 등). Homing_Offset 이 바뀌면 **같은 RAW 값이 다른 물리 자세**가 되어
실기로 얻은 파지 자세가 전부 어긋난다.

이 스크립트는 그 값들을 JSON 으로 남겨, 나중에 그리퍼 미션을 되살릴 수 있게 한다.

    python backup_servo_offsets.py COM8            # 백업
    python backup_servo_offsets.py COM8 --restore <파일>   # 복구
"""
import json
import sys
from datetime import datetime
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

# SO-101 의 관절 구성. so_follower.py 와 같은 순서·모델.
MOTORS = {
    "shoulder_pan": (1, "sts3215"),
    "shoulder_lift": (2, "sts3215"),
    "elbow_flex": (3, "sts3215"),
    "wrist_flex": (4, "sts3215"),
    "wrist_roll": (5, "sts3215"),
    "gripper": (6, "sts3215"),
}

# 백업할 레지스터. Homing_Offset 이 핵심이고 나머지는 참고용이다.
FIELDS = ["Homing_Offset", "Min_Position_Limit", "Max_Position_Limit",
          "Present_Position"]

OUT_DIR = Path(__file__).parent / "servo_backup"


def make_bus(port: str) -> FeetechMotorsBus:
    return FeetechMotorsBus(
        port=port,
        motors={n: Motor(i, m, MotorNormMode.RANGE_M100_100)
                for n, (i, m) in MOTORS.items()},
    )


def backup(port: str) -> int:
    bus = make_bus(port)
    bus.connect(handshake=False)
    data = {"port": port, "when": datetime.now().isoformat(timespec="seconds"),
            "motors": {}}
    for name in MOTORS:
        row = {}
        for f in FIELDS:
            try:
                row[f] = int(bus.read(f, name, normalize=False))
            except Exception as e:
                row[f] = f"읽기 실패: {type(e).__name__}"
        data["motors"][name] = row
        print(f"  {name:14s} {row}")
    bus.disconnect()

    OUT_DIR.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = OUT_DIR / f"servo_{port}_{stamp}.json"
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n저장: {path}")
    return 0


def restore(port: str, src: str) -> int:
    data = json.loads(Path(src).read_text(encoding="utf-8"))
    bus = make_bus(port)
    bus.connect(handshake=False)
    for name, row in data["motors"].items():
        v = row.get("Homing_Offset")
        if isinstance(v, int):
            bus.write("Homing_Offset", name, v, normalize=False)
            print(f"  {name:14s} Homing_Offset <- {v}")
    bus.disconnect()
    print("\n복구 완료. 그리퍼 미션의 교시 자세를 실제로 확인할 것.")
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    p = sys.argv[1]
    if "--restore" in sys.argv:
        sys.exit(restore(p, sys.argv[sys.argv.index("--restore") + 1]))
    sys.exit(backup(p))
