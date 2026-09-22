"""이름 포즈로 팔을 옮긴다 — ROS 없이. 벤치 작업과 복구용.

    python3 tools/goto_pose.py --pose idle
    python3 tools/goto_pose.py --pose carry --keep-gripper
    python3 tools/goto_pose.py --target -5.5 -103.3 95.8 73.5 -0.2 7 --duration 4

## 왜 필요한가

토크가 꺼지면 팔이 중력으로 처진다(2026-09-22 실측: wrist_flex 가 89도 내려앉음).
그 상태에서는 `pi_mission_node` 가 "팔이 알려진 자세가 아니다"로 작업을 거부한다 —
관절 공간 직선 보간이 바닥을 쓸 수 있어서 자동으로 움직이지 않는 것이 이 프로젝트의 규칙이다.
그 규칙을 지키면서 사람이 안전을 판단하고 되돌리는 경로가 이 도구다.

## ⚠️ 이 도구는 팔을 실제로 움직인다

- 관절 공간 **직선 보간**이다. 출발 자세에 따라 그리퍼가 바닥이나 차체를 스칠 수 있다.
  움직이기 전에 관절별 이동량을 표로 보여주고 확인을 받는다(`--yes` 로 생략).
- `arm_driver_node` 가 떠 있으면 포트를 못 열어 실패한다(의도된 동작).
- 도착 후 **토크를 켠 채로 둔다.** 끄면 다시 처진다. 일부러 끄려면 `--release`.
"""
from __future__ import annotations

import argparse
import sys
import time

from _common import add_common_args, confirm, open_from_args

from vla_common.arm_units import GRIPPER_INDEX, JOINT_NAMES, NUM_JOINTS, SERVO_IDS
from vla_common.config import load_poses

#: 도착으로 보는 오차(도, 그리퍼는 %). arm_driver_node 의 POSE_ARRIVE_TOL 과 같은 값.
ARRIVE_TOL = 3.0
SETTLE_MAX_S = 3.0
#: 이보다 큰 이동은 한 번 더 경고한다. 큰 이동일수록 경로가 예측에서 벗어난다.
BIG_MOVE_DEG = 60.0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--pose", help="arm_poses.yaml 의 이름 (idle / carry / drop ...)")
    g.add_argument("--target", nargs=NUM_JOINTS, type=float, metavar=("J1", "J2", "J3", "J4", "J5", "G"),
                   help="정책 단위 목표 6개 (servo1-5 도, gripper 0..100)")
    ap.add_argument("--keep-gripper", action="store_true", help="그리퍼는 지금 값을 유지한다(물체를 문 채 이동)")
    ap.add_argument("--duration", type=float, default=0.0, help="이동 시간(초). 0 이면 설정 속도로 계산")
    ap.add_argument("--speed", type=int, default=0, help="Goal_Velocity(raw/s). 0 이면 무제한(보간이 속도를 정한다)")
    ap.add_argument("--release", action="store_true", help="도착 후 토크를 끈다 (팔이 처진다)")
    ap.add_argument("--yes", action="store_true")
    args = ap.parse_args()
    cfg, calib, bus = open_from_args(args)
    try:
        # 캘리브레이션이 서보와 다르면 정책 단위가 다른 자세를 가리킨다 — 움직이지 않는다.
        wrong = [sid for sid in SERVO_IDS if bus.read_homing_offset(sid) != calib.homing_offsets[sid]]
        if wrong:
            print(f"Homing_Offset 이 캘리브레이션과 다르다: servo {wrong}\n"
                  "  tools/write_calibration.py 로 먼저 맞출 것 — 지금 움직이면 엉뚱한 자세로 간다")
            return 2
        volt = bus.read_voltage(SERVO_IDS[0])
        if volt is not None and volt < cfg.arm.min_voltage_v:
            print(f"서보 전압 {volt:.1f}V < {cfg.arm.min_voltage_v:.1f}V — 충전 후 다시 할 것")
            return 2

        if args.pose:
            poses = load_poses(cfg.arm.poses_file)
            pose = poses.get(args.pose)
            if pose is None:
                print(f"모르는 포즈: {args.pose} (있는 것: {sorted(poses)})")
                return 2
            if not pose.measured:
                print(f"'{args.pose}' 는 아직 실측되지 않았다 — tools/teach_pose.py --name {args.pose}")
                return 2
            target = list(pose.values)
            label = args.pose
        else:
            target, label = [float(v) for v in args.target], "target"

        raw = bus.read_positions(SERVO_IDS)
        if raw is None:
            print("관절 위치를 못 읽었다")
            return 2
        start = calib.raw_to_policy(raw)
        if args.keep_gripper:
            target[GRIPPER_INDEX] = start[GRIPPER_INDEX]

        print(f"{'관절':<14} {'현재':>8} {'목표':>8} {'이동':>8}")
        for name, a, b in zip(JOINT_NAMES, start, target):
            print(f"{name:<14} {a:8.1f} {b:8.1f} {b - a:+8.1f}")
        span = max(abs(b - a) for a, b in zip(start[:GRIPPER_INDEX], target[:GRIPPER_INDEX]))
        duration = args.duration if args.duration > 0 else max(
            1.0, span / max(cfg.arm.pose_speed_deg_s, 1.0))
        print(f"\n가장 큰 관절 이동 {span:.1f}도 · 이동 시간 {duration:.1f}초 · 전압 {volt}V")
        if span > BIG_MOVE_DEG:
            print(f"⚠️ 이동이 큽니다({span:.0f}도). 관절 공간 직선이라 그리퍼가 바닥·차체를 스칠 수 있습니다 — 경로를 눈으로 확인할 것")
        if not confirm(f"[{label}] 로 팔을 움직입니다. 주변을 비우세요.", args.yes):
            print("취소")
            return 1

        # 토크를 켜기 전에 목표를 현재 위치로 맞춘다(RAM 에 남은 옛 목표로 튀지 않게).
        bus.write_goal_positions(dict(zip(SERVO_IDS, raw)))
        bus.set_goal_velocity(SERVO_IDS, args.speed)
        bus.set_torque(SERVO_IDS, True)
        time.sleep(0.1)

        rate = max(cfg.arm.pose_rate_hz, 5.0)
        steps = max(1, int(round(duration * rate)))
        started = time.monotonic()
        for k in range(1, steps + 1):
            s = k / steps
            s = s * s * (3.0 - 2.0 * s)          # smoothstep — 시작·끝을 부드럽게
            point = [a + (b - a) * s for a, b in zip(start, target)]
            bus.write_goal_positions(dict(zip(SERVO_IDS, calib.policy_to_raw(point))))
            remaining = started + k / rate - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)

        deadline = time.monotonic() + SETTLE_MAX_S
        while True:
            now_raw = bus.read_positions(SERVO_IDS)
            if now_raw is None:
                print("도착 확인용 읽기 실패")
                return 2
            final = calib.raw_to_policy(now_raw)
            errs = [abs(a - b) for a, b in zip(final, target)]
            if args.keep_gripper:
                errs[GRIPPER_INDEX] = 0.0
            if max(errs) <= ARRIVE_TOL or time.monotonic() > deadline:
                break
            time.sleep(0.1)

        worst = max(errs)
        print(f"\n도착: " + "  ".join(f"{n[:6]}={v:+.1f}" for n, v in zip(JOINT_NAMES, final)))
        print(f"최대 오차 {worst:.1f} ({JOINT_NAMES[errs.index(worst)]})"
              + ("" if worst <= ARRIVE_TOL else "  ⚠️ 목표에 못 미쳤다 — 걸렸거나 전압 부족"))
        if args.release:
            bus.set_torque(SERVO_IDS, False)
            print("토크 끔 — 팔이 처집니다")
        else:
            print("토크 유지 — 자세를 잡고 있습니다")
        return 0 if worst <= ARRIVE_TOL else 1
    finally:
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
