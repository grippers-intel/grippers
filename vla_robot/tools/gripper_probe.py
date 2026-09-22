"""그리퍼 개구율 실측 — 손으로 움직이는 동안 값을 기록한다.

    python3 tools/gripper_probe.py --seconds 60 --label empty      # 손으로 움직이며 관찰
    python3 tools/gripper_probe.py --set 0 --label empty-closed    # 모터로 닫고 정착값 측정
    python3 tools/gripper_probe.py --set 0 --label queen           # 물체를 물린 채 닫기

정책 단위(0..100)는 **열린 정도**다. 0 = 캘리브레이션 range_min(가장 닫힘),
100 = range_max(가장 열림). 각도도, 턱 폭(mm)도 아니다 — lerobot 의 RANGE_0_100 이고
정책이 이 단위로 학습됐다.

이 도구로 재는 것은 두 가지다.

1. **0%/100% 가 실제 물리 끝단과 맞는가.** 캘리브레이션 창이 실제 가동범위보다 좁거나
   넓으면 정책 출력이 잘리거나 남는다.
2. **grasp_check 기준값.** 빈손으로 닫았을 때와 물체를 물었을 때의 값 사이에 문턱을 둔다.
   두 값이 안 벌어지면 파지 성공 판정을 개구율로는 못 한다는 뜻이다.

⚠️ 토크가 켜져 있으면 손으로 못 움직인다. 켜져 있으면 알려주고 그대로 진행한다
   (정책 재생 중 값을 보고 싶을 때도 쓰므로 끄지는 않는다).
"""
from __future__ import annotations

import argparse
import sys
import time

from _common import add_common_args, open_from_args

from vla_common.arm_units import GRIPPER_INDEX, SERVO_IDS


def drive(bus, calib, sid, joint, args) -> int:
    """그리퍼를 모터로 목표까지 움직이고, 움직임이 멎을 때까지 기다린 뒤 값을 읽는다.

    물체를 물면 목표에 도달하지 못하고 멎는다 — 그 **멎은 위치**가 물체 폭이고,
    그때의 부하가 쥐는 힘이다. 둘 다 grasp_check 의 기준값이 된다.
    """
    def pct_of(raw):
        return calib.raw_to_policy([0, 0, 0, 0, 0, raw])[GRIPPER_INDEX]

    start_raw = bus.read_position(sid)
    if start_raw is None:
        print("그리퍼 위치를 못 읽었다")
        return 2
    target_raw = calib.gripper_percent_to_raw(args.set)
    print(f"시작 raw {start_raw} ({pct_of(start_raw):.1f}%) -> 목표 raw {target_raw} ({args.set:.1f}%)")
    print("⚠️ 턱에서 손을 떼 주세요 — 모터가 움직입니다")
    # 토크를 켜기 전에 목표를 현재 위치로 맞춘다(RAM 에 남은 옛 목표로 튀지 않게).
    bus.write_goal_positions({sid: start_raw})
    bus.set_goal_velocity([sid], args.speed)
    bus.set_torque([sid], True)
    time.sleep(0.1)
    steps = 20
    for k in range(1, steps + 1):
        bus.write_goal_positions({sid: int(start_raw + (target_raw - start_raw) * k / steps)})
        time.sleep(0.05)
    # 정착 대기: 위치 변화가 멎으면 끝. 물체를 물면 목표 전에 멎는다.
    # 어디서 멎었는지가 물체 폭이므로 궤적을 그대로 찍는다 — 값 하나만 보면
    # "물체를 물고 멎은 것"과 "그냥 다 닫힌 것"이 구분되지 않는다.
    last, still = None, 0
    trace = []
    for _ in range(40):
        time.sleep(0.1)
        raw = bus.read_position(sid)
        if raw is None:
            continue
        trace.append((raw, bus.read_load_ratio(sid) or 0.0))
        if last is not None and abs(raw - last) <= 2:
            still += 1
            if still >= 5:
                break
        else:
            still = 0
        last = raw
    final = bus.read_position(sid)
    load = bus.read_load_ratio(sid) or 0.0
    temp = bus.read_temperature(sid)
    reached = abs(final - target_raw) <= 5
    print()
    if trace:
        print("  궤적 (0.1s 간격, raw/부하):")
        cells = [f"{r}({l:.2f})" for r, l in trace[:20]]
        for i in range(0, len(cells), 6):
            print("    " + "  ".join(cells[i:i + 6]))
    print(f"[{args.label or '-'}] 정착 raw {final} = {pct_of(final):6.2f}%  부하 {load:.2f}  {temp}°C")
    print("  목표 도달" if reached else f"  목표({target_raw})에 못 미치고 멎음 — 물체를 물었거나 기계적 한계")
    if not args.keep_torque:
        bus.set_torque([sid], False)
        print("  토크 끔")
    else:
        print("  토크 유지 — 문 상태로 남긴다")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--label", default="", help="기록용 이름 (empty / queen / rook ...)")
    ap.add_argument("--set", type=float, default=None, metavar="PCT",
                    help="그리퍼를 이 개구율(0..100)로 **모터로** 움직이고 정착값을 잰다. "
                         "손힘으로는 학습 때의 닫힘값까지 안 닫힌다 — 그 기준선은 이 경로로만 나온다")
    ap.add_argument("--speed", type=int, default=300, help="Goal_Velocity (raw/s). 작을수록 부드럽다")
    ap.add_argument("--keep-torque", action="store_true",
                    help="측정 뒤에도 토크를 켜 둔다(물체를 문 채 유지). 기본은 끈다")
    args = ap.parse_args()
    cfg, calib, bus = open_from_args(args)
    sid = SERVO_IDS[GRIPPER_INDEX]
    joint = calib.joints[GRIPPER_INDEX]
    try:
        if args.set is not None:
            return drive(bus, calib, sid, joint, args)
        if bus.read_torque(sid):
            print("⚠️ 그리퍼 토크가 켜져 있다 — 손으로는 안 움직인다")
        print(f"캘리브레이션 창: raw {joint.range_min}..{joint.range_max} = 0..100%")
        print(f"{args.seconds:.0f}초 기록 시작 (label={args.label or '-'})")
        period = 1.0 / max(args.hz, 1.0)
        end = time.monotonic() + args.seconds
        samples = []          # (t, raw, pct, load)
        started = time.monotonic()
        while time.monotonic() < end:
            raw = bus.read_position(sid)
            if raw is not None:
                pct = calib.raw_to_policy([0, 0, 0, 0, 0, raw])[GRIPPER_INDEX]
                load = bus.read_load_ratio(sid) or 0.0
                samples.append((time.monotonic() - started, raw, pct, load))
            time.sleep(period)
        if not samples:
            print("샘플을 하나도 못 읽었다")
            return 2
        raws = [s[1] for s in samples]
        pcts = [s[2] for s in samples]
        lo_i, hi_i = raws.index(min(raws)), raws.index(max(raws))
        print(f"\n샘플 {len(samples)}개")
        print(f"  최소  raw {raws[lo_i]:4d}  {pcts[lo_i]:6.2f}%  (t={samples[lo_i][0]:.1f}s)")
        print(f"  최대  raw {raws[hi_i]:4d}  {pcts[hi_i]:6.2f}%  (t={samples[hi_i][0]:.1f}s)")
        print(f"  마지막 raw {raws[-1]:4d}  {pcts[-1]:6.2f}%  부하 {samples[-1][3]:.2f}")
        # 창을 벗어난 관측은 캘리브레이션 창이 실제 가동범위보다 좁다는 뜻이다.
        below = sum(1 for r in raws if r < joint.range_min)
        above = sum(1 for r in raws if r > joint.range_max)
        if below or above:
            print(f"  ⚠️ 창 밖 관측: 아래 {below}개 · 위 {above}개 — 실제 가동범위가 더 넓다")
        print("\n타임라인 (2초마다 % / 창 밖은 * 표시)")
        bucket = 2.0
        line = []
        for t0 in range(0, int(args.seconds), int(bucket)):
            window = [s for s in samples if t0 <= s[0] < t0 + bucket]
            if not window:
                continue
            mid = window[len(window) // 2]
            mark = "*" if not (joint.range_min <= mid[1] <= joint.range_max) else " "
            line.append(f"{t0:3d}s {mid[2]:6.1f}%{mark}")
        for i in range(0, len(line), 5):
            print("   " + "  ".join(line[i:i + 5]))
        return 0
    finally:
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
