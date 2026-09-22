"""팔 점검 — 읽기만 한다. 아무것도 쓰지 않는다.

    python3 tools/arm_check.py [--config robot.yaml] [--port /dev/soarm]

서보별: 응답, Homing_Offset(캘리브레이션과 비교), 위치 한계, 현재 위치(raw/정책 단위),
부하, 온도, 토크. 문제가 있으면 종료 코드 1.
"""
from __future__ import annotations

import argparse
import sys

from _common import add_common_args, open_from_args

from vla_common.arm_units import JOINT_NAMES, SERVO_IDS


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    args = ap.parse_args()
    cfg, calib, bus = open_from_args(args)
    problems = []
    try:
        print(f"포트 {bus.port} @ {bus.baudrate}")
        print(f"{'id':>2} {'관절':<14} {'응답':<4} {'homing(EEPROM/파일)':<20} {'한계':<12} "
              f"{'raw':>5} {'부하':>5} {'온도':>4} {'토크':<4}")
        raw_all = []
        for sid, name, joint in zip(SERVO_IDS, JOINT_NAMES, calib.joints):
            online = bus.ping(sid)
            if not online:
                problems.append(f"servo {sid} 응답 없음")
                print(f"{sid:>2} {name:<14} {'X':<4}")
                raw_all.append(None)
                continue
            homing = bus.read_homing_offset(sid)
            limits = bus.read_position_limits(sid)
            pos = bus.read_position(sid)
            load = bus.read_load_ratio(sid)
            temp = bus.read_temperature(sid)
            torque = bus.read_torque(sid)
            raw_all.append(pos)
            mark = "" if homing == joint.homing_offset else "  <-- 불일치"
            if homing != joint.homing_offset:
                problems.append(f"servo {sid} Homing_Offset {homing} != {joint.homing_offset}")
            if pos is None:
                problems.append(f"servo {sid} 위치 읽기 실패")
            lim = f"{limits[0]}..{limits[1]}" if limits else "?"
            print(f"{sid:>2} {name:<14} {'O':<4} {f'{homing}/{joint.homing_offset}':<20} {lim:<12} "
                  f"{pos if pos is not None else '?':>5} {load if load is not None else 0:>5.2f} "
                  f"{temp if temp is not None else '?':>4} {str(torque):<4}{mark}")
        volt = bus.read_voltage(SERVO_IDS[0])
        print(f"\n전압 {volt if volt is not None else '?'} V (하한 {cfg.arm.min_voltage_v} V)")
        if volt is not None and volt < cfg.arm.min_voltage_v:
            problems.append(f"전압 낮음 {volt:.1f}V")
        if all(r is not None for r in raw_all):
            pol = calib.raw_to_policy(raw_all)
            print("정책 단위: " + ", ".join(f"{n}={v:+.1f}" for n, v in zip(JOINT_NAMES, pol)))
        if bus.stats.read_failures:
            print(f"읽기 재시도 {bus.stats.read_failures}회 / 읽기 {bus.stats.reads}회")
    finally:
        bus.close()
    if problems:
        print("\n문제:\n  " + "\n  ".join(problems))
        if any("Homing_Offset" in p for p in problems):
            print("  -> tools/write_calibration.py 로 캘리브레이션 파일 값을 서보에 쓴다")
        return 1
    print("\n이상 없음")
    return 0


if __name__ == "__main__":
    sys.exit(main())
