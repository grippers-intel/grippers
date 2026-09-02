#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""파이에서 교시 당시 Homing_Offset·Min/Max_Angle_Limit 으로 되돌린다 — LeRobot 없이.

## 왜 따로 있는가

`backup_servo_offsets.py` 는 LeRobot 의 FeetechMotorsBus 를 쓰고 포트 이름이
COM7/COM8 이다 — 캘리브레이션을 돌린 윈도우 PC 기준이다. 그런데 베이스라인과
VLA 수집을 오가려면 **파이에서** 오프셋을 바꿔야 한다. LeRobot 이 망가뜨린
것을 되돌리자고 LeRobot 을 설치해야 한다면 그건 되돌리는 길이 아니다.

여기서는 이미 저장소에 있는 `driver_sdk` 로 같은 레지스터를 읽고 쓴다
(`get_homing_offset` / `set_homing_offset` — EEPROM 잠금은 SDK 가 알아서
풀고 잠근다. Min/Max_Angle_Limit 은 SDK 에 공개 API가 없어
`position_limit_registers.py` 가 저수준 프리미티브로 대신한다).

⚠️ 2026-09-01 실기: 이 도구가 Homing_Offset만 복구하던 시절, 그리퍼가 전혀
안 닫히는 사고가 났다. Homing_Offset 은 이미 복구돼 있었는데도 그랬다 —
원인은 Min/Max_Angle_Limit 이었다. LeRobot/VLA 캘리브레이션은 이 레지스터도
Homing_Offset 과 같이 덮어쓰는데, 이 도구는 그걸 보지도 쓰지도 않았다. 그날은
스크래치패드 즉석 스크립트로 응급 복구했고 저장소에는 반영되지 않았다 — 이제
이 도구가 둘 다 같이 확인·복구한다.

## 기준값은 어디서 오는가

`floor_grasp_profiles.TAUGHT_HOMING_OFFSETS`·`TAUGHT_POSITION_LIMITS` 둘이다.
교시 자세와 그 자세를 잰 값들은 한 쌍이라 같은 파일에 있다. 여기서 숫자를
다시 적지 않는다 — 사본이 늘면 갈라진다.

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
from grippers_arm import position_limit_registers as poslim  # noqa: E402
from grippers_arm.floor_grasp_profiles import (  # noqa: E402
    TAUGHT_HOMING_OFFSETS,
    TAUGHT_POSITION_LIMITS,
)

SETTLE_S = 0.05
TAUGHT_MIN, TAUGHT_MAX = poslim.split_taught(TAUGHT_POSITION_LIMITS)


def _offsets_ok(current) -> bool:
    return calib_identity.verdict(current, TAUGHT_HOMING_OFFSETS).ok


def _limits_ok(current_limits) -> bool:
    current_min, current_max = poslim.split_limits(current_limits)
    return (calib_identity.verdict(current_min, TAUGHT_MIN).ok
            and calib_identity.verdict(current_max, TAUGHT_MAX).ok)


def _report(current, current_limits) -> int:
    offsets = calib_identity.verdict(current, TAUGHT_HOMING_OFFSETS)
    print("Homing_Offset:", offsets.message())
    current_min, current_max = poslim.split_limits(current_limits)
    min_result = calib_identity.verdict(current_min, TAUGHT_MIN)
    max_result = calib_identity.verdict(current_max, TAUGHT_MAX)
    print("Min_Angle_Limit:", min_result.message())
    print("Max_Angle_Limit:", max_result.message())
    ok = offsets.ok and min_result.ok and max_result.ok
    return 0 if ok else 1


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
        limit_ids = sorted(TAUGHT_POSITION_LIMITS)
        current = calib_identity.read_offsets(drv, servo_ids)
        current_limits = poslim.read_all(drv, limit_ids)
        print("현재 Homing_Offset")
        for sid in servo_ids:
            want = TAUGHT_HOMING_OFFSETS[sid]
            got = current[sid]
            mark = "" if got == want else "   <- 교시값 %d" % want
            print(f"  servo {sid}: {got}{mark}")
        print("현재 Min/Max_Angle_Limit")
        for sid in limit_ids:
            want = TAUGHT_POSITION_LIMITS[sid]
            got = current_limits[sid]
            mark = "" if got == want else f"   <- 교시값 {want}"
            print(f"  servo {sid}: {got}{mark}")
        print()

        if not args.apply:
            return _report(current, current_limits)

        if _offsets_ok(current) and _limits_ok(current_limits):
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
                failed.append(("Homing_Offset", sid))
            time.sleep(SETTLE_S)
        for sid in limit_ids:
            lo, hi = TAUGHT_POSITION_LIMITS[sid]
            if not poslim.set_position_limits(drv, sid, lo, hi):
                failed.append(("Angle_Limit", sid))
            time.sleep(SETTLE_S)

        # 쓴 것을 다시 읽어 확인한다. EEPROM 쓰기는 조용히 실패할 수 있고,
        # 그 상태로 미션을 돌리는 것이 이 도구가 막으려는 바로 그 일이다
        # (2026-09-01 실기가 정확히 이 실패 모드였다 — Angle_Limit 쓰기
        # 자체를 시도조차 안 했으니 당연히 확인도 안 됐다).
        after = calib_identity.read_offsets(drv, servo_ids)
        after_limits = poslim.read_all(drv, limit_ids)
        print()
        if failed:
            print(f"쓰기 실패한 항목: {failed}", file=sys.stderr)
        rc = _report(after, after_limits)
        if rc == 0:
            print()
            print("토크는 꺼진 채입니다. arm_driver_node 를 다시 띄우면 "
                  "IDLE 로 정렬합니다.")
        return rc
    finally:
        drv.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
