#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""파이에서 교시 당시 Homing_Offset 으로 되돌린다 — LeRobot 없이.

## 왜 따로 있는가

`backup_servo_offsets.py` 는 LeRobot 의 FeetechMotorsBus 를 쓰고 포트 이름이
COM7/COM8 이다 — 캘리브레이션을 돌린 윈도우 PC 기준이다. 그런데 베이스라인과
VLA 수집을 오가려면 **파이에서** 오프셋을 바꿔야 한다. LeRobot 이 망가뜨린
것을 되돌리자고 LeRobot 을 설치해야 한다면 그건 되돌리는 길이 아니다.

여기서는 이미 저장소에 있는 `driver_sdk` 로 같은 레지스터를 읽고 쓴다
(`get_homing_offset` / `set_homing_offset` — EEPROM 잠금은 SDK 가 알아서
풀고 잠근다).

## 기준값은 어디서 오는가

`floor_grasp_profiles.TAUGHT_HOMING_OFFSETS` 하나다. 교시 자세와 그 자세를
잰 오프셋은 한 쌍이라 같은 파일에 있다. 여기서 숫자를 다시 적지 않는다 —
사본이 늘면 갈라진다.

## 쓰는 법

    python3 tools/arm/restore_taught_offsets.py                지금 상태만 본다
    python3 tools/arm/restore_taught_offsets.py --apply --yes  되돌린다

`--apply` 는 **토크를 끕니다 — 팔이 중력으로 내려옵니다.** EEPROM 쓰기는
토크가 꺼져 있어야 하고, 이건 우회할 수 없습니다. 팔 아래를 비우고,
물건을 들고 있지 않은 상태에서 하세요.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT / "third_party" / "soarm_provided_d" / "soarm_lab"))
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "grippers_arm"))

from driver_sdk import STS3215Driver  # noqa: E402
from grippers_arm import calib_identity  # noqa: E402
from grippers_arm.floor_grasp_profiles import TAUGHT_HOMING_OFFSETS  # noqa: E402

SETTLE_S = 0.05


def _report(current) -> int:
    result = calib_identity.verdict(current, TAUGHT_HOMING_OFFSETS)
    print(result.message())
    return 0 if result.ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(
        description="교시 당시 Homing_Offset 확인·복구 (LeRobot 불필요)")
    ap.add_argument("--port", default="/dev/soarm")
    ap.add_argument("--apply", action="store_true", help="실제로 되돌린다")
    ap.add_argument("--yes", action="store_true",
                    help="토크를 끄고 팔이 내려오는 것을 승인한다")
    args = ap.parse_args()

    drv = STS3215Driver(port=args.port)
    if not drv.connect():
        print(f"{args.port} 를 열지 못했습니다.", file=sys.stderr)
        print("  arm_driver_node 나 텔레옵이 떠 있으면 포트를 잡고 있습니다 — "
              "먼저 내리세요.", file=sys.stderr)
        return 2

    try:
        servo_ids = sorted(TAUGHT_HOMING_OFFSETS)
        current = calib_identity.read_offsets(drv, servo_ids)
        print("현재 Homing_Offset")
        for sid in servo_ids:
            want = TAUGHT_HOMING_OFFSETS[sid]
            got = current[sid]
            mark = "" if got == want else "   <- 교시값 %d" % want
            print(f"  servo {sid}: {got}{mark}")
        print()

        if not args.apply:
            return _report(current)

        if calib_identity.verdict(current, TAUGHT_HOMING_OFFSETS).ok:
            print("이미 교시 당시 값입니다 — 아무것도 쓰지 않았습니다.")
            return 0

        if not args.yes:
            print("--apply 는 토크를 끕니다. **팔이 중력으로 내려옵니다.**")
            print("팔 아래를 비우고 물건을 놓은 뒤 --yes 를 같이 주세요.")
            return 2

        print("토크 해제 — 팔이 내려옵니다")
        drv.set_all_torque(False)
        time.sleep(SETTLE_S)

        failed = []
        for sid in servo_ids:
            want = TAUGHT_HOMING_OFFSETS[sid]
            if not drv.set_homing_offset(sid, want):
                failed.append(sid)
            time.sleep(SETTLE_S)

        # 쓴 것을 다시 읽어 확인한다. EEPROM 쓰기는 조용히 실패할 수 있고,
        # 그 상태로 미션을 돌리는 것이 이 도구가 막으려는 바로 그 일이다.
        after = calib_identity.read_offsets(drv, servo_ids)
        print()
        if failed:
            print(f"쓰기 실패한 서보: {failed}", file=sys.stderr)
        rc = _report(after)
        if rc == 0:
            print()
            print("토크는 꺼진 채입니다. arm_driver_node 를 다시 띄우면 "
                  "IDLE 로 정렬합니다.")
        return rc
    finally:
        drv.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
