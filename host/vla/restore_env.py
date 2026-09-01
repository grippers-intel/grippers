"""새 환경에서 v4 녹화 조건을 복원한다.

    python host/vla/restore_env.py              # 검사만 (아무것도 안 쓴다)
    python host/vla/restore_env.py --apply      # 캘리브레이션을 HF 캐시로 복사
    python host/vla/restore_env.py --apply --servo COM8   # 서보에도 쓴다

## 왜 이 스크립트가 필요한가

녹화 조건 중 **저장소가 추적하지 못하는 것이 둘** 있다.

1. **캘리브레이션 파일** — `~/.cache/huggingface/lerobot/calibration/` 에 있다.
   이게 서보 EEPROM 과 어긋나면 `is_calibrated` 가 False 가 되고, `connect()` 가
   `calibrate()` 를 걸어 **조용히 영점을 덮어쓴다**. 오류도 프롬프트도 없다.
2. **서보 EEPROM 의 속도 상한** — `Maximum_Velocity_Limit`. lerobot 이 절대
   안 건드리므로 팔을 바꾸거나 초기화하면 그 값이 그대로 살아 있다.

여기 있는 `calibration/*.json` 이 v4 를 찍은 그 값이다.
"""
import argparse
import json
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE / "calibration"
MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper"]
VELOCITY_LIMIT = 250          # 2026-09-01 실측 — 65 면 팔로워가 굼뜨다


def hf_paths():
    from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS, TELEOPERATORS
    return {
        "grippers_arm.json": Path(HF_LEROBOT_CALIBRATION) / ROBOTS / "so_follower" / "grippers_arm.json",
        "leader.json": Path(HF_LEROBOT_CALIBRATION) / TELEOPERATORS / "so_leader" / "leader.json",
    }


def check_files(apply: bool) -> int:
    bad = 0
    for name, dst in hf_paths().items():
        src = SRC / name
        if not src.exists():
            print(f"  ✗ 저장소에 {name} 이 없다"); bad += 1; continue
        same = dst.exists() and json.loads(dst.read_text(encoding="utf-8")) == \
            json.loads(src.read_text(encoding="utf-8"))
        print(f"  {'○' if same else '✗'} {name}  ->  {dst}")
        if same:
            continue
        bad += 1
        if apply:
            dst.parent.mkdir(parents=True, exist_ok=True)
            if dst.exists():
                keep = dst.with_suffix(".json.before_restore")
                shutil.copy2(dst, keep)
                print(f"      기존 파일 백업: {keep.name}")
            shutil.copy2(src, dst)
            print("      복사함")
    return bad


def check_servo(port: str, apply: bool) -> int:
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    f = json.loads((SRC / "grippers_arm.json").read_text(encoding="utf-8"))
    motors = {n: Motor(f[n]["id"], "sts3215", MotorNormMode.RANGE_M100_100) for n in MOTORS}
    calib = {n: MotorCalibration(**{k: f[n][k] for k in
                                    ("id", "drive_mode", "homing_offset", "range_min", "range_max")})
             for n in MOTORS}
    bus = FeetechMotorsBus(port, motors, calibration=calib)
    bus._connect(handshake=False)
    bus.set_baudrate(1000000)

    on = sum(bus.read("Torque_Enable", n, normalize=False) for n in MOTORS)
    print(f"  토크 {on}/6" + ("  ← 쓰기 전에 꺼야 안전하다" if on and apply else ""))

    print(f"  is_calibrated = {bus.is_calibrated}")
    vel = {n: bus.read("Maximum_Velocity_Limit", n, normalize=False) for n in MOTORS}
    odd = [n for n, v in vel.items() if v != VELOCITY_LIMIT]
    print(f"  Maximum_Velocity_Limit = {sorted(set(vel.values()))}"
          f"{'  ← 기대값 %d 와 다름' % VELOCITY_LIMIT if odd else ''}")

    bad = (0 if bus.is_calibrated else 1) + (1 if odd else 0)
    if apply and bad:
        if on:
            print("  ✗ 토크가 켜져 있어 쓰지 않았다. 팔을 받치고 전원을 껐다 켠 뒤 다시 실행할 것.")
        else:
            if not bus.is_calibrated:
                bus.write_calibration(calib); print("  캘리브레이션을 서보에 씀")
            for n in odd:
                bus.write("Maximum_Velocity_Limit", n, VELOCITY_LIMIT, normalize=False)
            if odd:
                print(f"  Maximum_Velocity_Limit 을 {VELOCITY_LIMIT} 로 씀 ({len(odd)}축)")
            print(f"  다시 확인: is_calibrated = {bus.is_calibrated}")
            bad = 0
    bus.port_handler.closePort()
    return bad


def main() -> int:
    ap = argparse.ArgumentParser(description="v4 녹화 환경 복원")
    ap.add_argument("--apply", action="store_true", help="실제로 쓴다 (기본은 검사만)")
    ap.add_argument("--servo", metavar="PORT", help="서보까지 확인/복원 (예: COM8)")
    a = ap.parse_args()

    print("캘리브레이션 파일" + ("" if a.apply else "  (검사만)"))
    bad = check_files(a.apply)
    if a.servo:
        print()
        print(f"서보 {a.servo}" + ("" if a.apply else "  (검사만)"))
        bad += check_servo(a.servo, a.apply)
    print()
    if bad:
        print("어긋난 항목이 있다." + ("" if a.apply else "  --apply 로 맞출 수 있다."))
    else:
        print("v4 녹화 조건과 일치한다.")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
