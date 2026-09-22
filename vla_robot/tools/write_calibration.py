"""arm_calibration.json 의 Homing_Offset(선택: 위치 한계)을 서보 EEPROM 에 쓴다.

    python3 tools/write_calibration.py              # 차이만 보여 준다 (기본, 아무것도 안 쓴다)
    python3 tools/write_calibration.py --apply      # 토크를 끄고 쓴다
    python3 tools/write_calibration.py --apply --limits   # Min/Max_Position_Limit 도 쓴다

## 왜 필요한가

정책은 LeRobot 캘리브레이션 좌표계로 학습됐다. 서보 EEPROM 의 오프셋이 다르면 같은
명령이 다른 자세가 된다(기존 코드: 교시 오프셋과 섞여 wrist_roll 85.8도 어긋남).
arm_driver_node 는 불일치를 발견하면 기동을 거부하고, 이 도구로 맞춘다.

## ⚠️ 주의

- 토크를 끈다. **팔이 처지므로 손으로 받치고** 실행할 것.
- 위치 한계(--limits)는 lerobot write_calibration 과 같게 range_min/max 를 쓴다.
  wrist_roll 은 range 가 14틱이라 이 값을 쓰면 서보가 사실상 그 관절을 잠근다
  (기존 팀의 roll lock 과 같은 상태). 원치 않으면 --limits 를 빼라.
- 쓰기 전에 현재 EEPROM 값을 화면에 남긴다. 되돌리려면 그 값을 기록해 둘 것.
"""
from __future__ import annotations

import argparse
import sys

from _common import add_common_args, confirm, open_from_args

from vla_common.arm_units import SERVO_IDS


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--apply", action="store_true", help="실제로 쓴다")
    ap.add_argument("--limits", action="store_true", help="Min/Max_Position_Limit 도 쓴다")
    ap.add_argument("--yes", action="store_true", help="확인 질문 생략")
    args = ap.parse_args()
    _cfg, calib, bus = open_from_args(args)
    try:
        todo = []
        print(f"{'id':>2} {'관절':<14} {'homing EEPROM -> 파일':<24} {'한계 EEPROM -> 파일':<26}")
        for sid, joint in zip(SERVO_IDS, calib.joints):
            if not bus.ping(sid):
                print(f"servo {sid} 응답 없음 — 중단")
                return 2
            live = bus.read_homing_offset(sid)
            limits = bus.read_position_limits(sid)
            want_limits = (joint.range_min, joint.range_max)
            need_h = live != joint.homing_offset
            need_l = args.limits and limits != want_limits
            print(f"{sid:>2} {joint.name:<14} {f'{live} -> {joint.homing_offset}':<24} "
                  f"{f'{limits} -> {want_limits}':<26}{'  *' if need_h or need_l else ''}")
            if need_h or need_l:
                todo.append((sid, joint, need_h, need_l))
        if not todo:
            print("\n이미 일치한다. 쓸 것 없음.")
            return 0
        if not args.apply:
            print(f"\n{len(todo)}개 서보가 다르다. 쓰려면 --apply")
            return 3
        if not confirm("토크를 끄고 EEPROM 을 씁니다. 팔을 손으로 받치세요.", args.yes):
            print("취소")
            return 1
        bus.set_torque(SERVO_IDS, False)
        failed = []
        for sid, joint, need_h, need_l in todo:
            if need_h and not bus.write_homing_offset(sid, joint.homing_offset):
                failed.append(f"servo {sid} homing")
            if need_l and not bus.write_position_limits(sid, joint.range_min, joint.range_max):
                failed.append(f"servo {sid} limits")
        if failed:
            print(f"실패: {failed}")
            return 1
        print("완료. 토크는 꺼진 상태다. arm_driver_node 를 띄우면 현재 자세에서 토크가 켜진다.")
        return 0
    finally:
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
