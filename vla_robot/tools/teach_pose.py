"""이름 포즈 실측 — 토크를 끄고 손으로 자세를 잡은 뒤 arm_poses.yaml 에 저장한다.

    python3 tools/teach_pose.py --name drop --note "chess 상자 위"
    python3 tools/teach_pose.py --name drop --keep-gripper     # 그리퍼는 토크 유지(물체를 쥔 채)

값은 정책 단위(servo1-5 도, gripper 0..100)로 저장되고 measured: true 가 된다.
캘리브레이션이 서보와 다르면 거부한다 — 틀린 좌표계로 잰 포즈는 쓸모가 없다.

⚠️ 토크를 끄면 팔이 처진다. 손으로 받치고 시작할 것.
⚠️ ros2 가 --symlink-install 이 아니면 저장 후 colcon build 를 다시 해야 노드가 새 값을 본다.
"""
from __future__ import annotations

import argparse
import sys
import threading
import time

from _common import add_common_args, confirm, open_from_args

from vla_common.arm_units import GRIPPER_INDEX, JOINT_NAMES, SERVO_IDS
from vla_common.config import load_poses, save_pose


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--name", required=True, help="포즈 이름 (예: drop, carry)")
    ap.add_argument("--note", default="", help="설명")
    ap.add_argument("--keep-gripper", action="store_true", help="그리퍼 토크는 유지한다")
    ap.add_argument("--yes", action="store_true")
    args = ap.parse_args()
    cfg, calib, bus = open_from_args(args)
    try:
        wrong = [sid for sid in SERVO_IDS if bus.read_homing_offset(sid) != calib.homing_offsets[sid]]
        if wrong:
            print(f"Homing_Offset 이 캘리브레이션과 다르다: servo {wrong} — write_calibration.py 먼저")
            return 2
        free = [sid for sid in SERVO_IDS if not (args.keep_gripper and sid == SERVO_IDS[GRIPPER_INDEX])]
        if not confirm(f"servo {free} 토크를 끕니다. 팔을 손으로 받치세요.", args.yes):
            return 1
        raw = bus.read_positions(SERVO_IDS)
        if raw is None:
            print("위치 읽기 실패")
            return 2
        bus.write_goal_positions(dict(zip(SERVO_IDS, raw)))
        bus.set_torque(free, False)

        stop = threading.Event()
        latest: dict = {}

        def show() -> None:
            while not stop.is_set():
                r = bus.read_positions(SERVO_IDS)
                if r is not None:
                    latest["raw"] = r
                    pol = calib.raw_to_policy(r)
                    line = " ".join(f"{n[:6]}={v:+6.1f}" for n, v in zip(JOINT_NAMES, pol))
                    print("\r" + line + "   ", end="", flush=True)
                time.sleep(0.2)

        t = threading.Thread(target=show, daemon=True)
        t.start()
        try:
            answer = input("\n자세를 잡고 Enter = 저장, q + Enter = 취소\n")
        finally:
            stop.set()
            t.join(timeout=1.0)
        raw = bus.read_positions(SERVO_IDS) or latest.get("raw")
        # 저장 여부와 무관하게 현재 자세에서 토크를 다시 켠다.
        if raw is not None:
            bus.write_goal_positions(dict(zip(SERVO_IDS, raw)))
        bus.set_torque(SERVO_IDS, True)
        if answer.strip().lower() == "q" or raw is None:
            print("저장하지 않았다. 토크는 다시 켰다.")
            return 1
        values = calib.raw_to_policy(raw)
        save_pose(cfg.arm.poses_file, args.name, values, args.note)
        saved = load_poses(cfg.arm.poses_file)[args.name]
        print(f"저장: {args.name} = {list(saved.values)} -> {cfg.arm.poses_file}")
        return 0
    finally:
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
