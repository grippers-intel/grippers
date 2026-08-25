#!/usr/bin/env python3
"""그리퍼가 얼마나 더 세게 쥘 수 있는지 재는 도구.

2026-08-25 사용자 지시 "최대한 세게 잡자"에 답하려면 먼저 알아야 하는 두
숫자가 있고, 둘 다 추측이 아니라 실측이어야 한다:

    (1) **빈 턱이 실제로 멈추는 지점.** GRIPPER_CLOSED_MM(9.0mm, raw 1150)은
        보정표의 첫 점일 뿐 물리적 하드스톱이 아니다. 실제로 빈 채 1150을
        명령하면 서보는 1155에서 멈춘다 — 즉 1150은 이미 도달 불가능한
        목표다. 하드스톱이 어디인지 알아야 "파지 전용 하한"을 어디까지
        내려도 되는지 정할 수 있다.

    (2) **과주행 1mm당 붙는 부하.** 힘은 도달할 폭이 아니라 도달하지 못하는
        거리가 만든다. 2026-08-25 회차의 거친 추정은 mm당 약 0.02였지만
        (rook 3.6mm→0.0821, knight 4.1mm→0.0899, box 7.0mm→0.1447) 세 점의
        물체가 전부 달라 같은 곡선 위의 점이 아니다. 한 물체로 훑어야 한다.

⚠️ 이 도구는 arm_driver의 그리퍼 하한을 **런타임 파라미터로 잠깐 낮춘 뒤
반드시 되돌린다**. 어떤 경로로 끝나든(예외, Ctrl+C 포함) 하한을 복구하고
그리퍼를 안전 폭으로 되돌리는 것이 이 파일의 가장 중요한 계약이다.

⚠️ 빈 스윕은 턱을 서로 밀어붙이는 것이라 **서보가 뜨거워진다**. 그래서
매 스텝 온도를 읽고 상한을 넘으면 즉시 중단한다. 실측에서 servo 6이 이미
52°C까지 올라간 적이 있다.

사용:
  python3 gripper_force_probe.py --empty              # 빈 턱 하드스톱
  python3 gripper_force_probe.py --holding knight     # 물체를 문 채 힘 곡선
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from grasp_test_console import CLASS_TO_PROFILE, GraspTestNode, RunLog
from grippers_arm.floor_grasp_profiles import FLOOR_GRASP_PROFILES
from grippers_arm.gripper_calibration import GRIPPER_CLOSED_MM, width_from_position
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

# 스윕 하한. 보정표 첫 구간의 기울기가 약 4.9 raw/mm라 2.0mm는 raw로
# 1116쯤이다 — 1150에서 34 raw 아래다.
SWEEP_MIN_MM = 2.0
SWEEP_STEP_MM = 1.0
# 매 스텝 명령 후 정착을 기다리는 시간(GRASP_SETTLE_SEC과 같은 근거).
STEP_SETTLE_S = 1.2
# 이 온도를 넘으면 즉시 중단하고 되돌린다.
MAX_SERVO6_TEMP_C = 50
# 위치가 이 폭 안에서만 변하면 더 이상 안 움직이는 것으로 본다
# (arm_driver_node.GRIPPER_MOTION_SETTLED_RAW과 같은 값).
STOPPED_RAW = 3


def set_min_width(node, width_mm) -> bool:
    """arm_driver의 그리퍼 하한 파라미터를 바꾼다."""
    from rcl_interfaces.srv import SetParameters

    client = node.create_client(SetParameters, "/arm_driver_node/set_parameters")
    if not client.wait_for_service(timeout_sec=5.0):
        print("  [경고] arm_driver_node/set_parameters 서비스 없음")
        return False
    request = SetParameters.Request()
    request.parameters = [
        Parameter("min_gripper_width_mm", Parameter.Type.DOUBLE, float(width_mm))
        .to_parameter_msg()
    ]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done():
        print("  [경고] 파라미터 설정 응답 없음")
        return False
    results = future.result().results
    if not results:
        print("  [경고] 파라미터 설정 결과가 비어 있습니다")
        return False
    failed = [r for r in results if not r.successful]
    if failed:
        print(f"  [경고] 파라미터 설정 거부됨: {failed[0].reason or '(사유 없음)'}")
        return False
    return True


def sample(node, log, label, commanded_mm, attempts=3):
    """한 스텝의 실측. (present_raw, 폭mm, load, temp) 또는 None.

    ⚠️ 한 번 실패했다고 스윕 전체를 접지 않는다(2026-08-25). 이 버스는
    패킷을 이따금 흘려서 정지 상태에서도 묶음 읽기가 10번에 1번쯤 깨지는데,
    그때마다 측정을 처음부터 다시 하게 되면 아무것도 잴 수 없다."""
    state = None
    for attempt in range(attempts):
        state = node.arm_state()
        if state is not None and state.ok:
            break
        why = "응답 없음" if state is None else (state.message or "ok=False")
        print(f"  [{commanded_mm:5.1f}mm] 서보 읽기 실패({why}) — "
              f"재시도 {attempt + 1}/{attempts}")
        time.sleep(0.3)
    if state is None or not state.ok:
        why = "응답 없음" if state is None else (state.message or "ok=False")
        print(f"  [{commanded_mm:5.1f}mm] 서보 읽기 실패 — {why}")
        log.log("force_sample_failed", label=label, commanded_mm=commanded_mm, why=why)
        return None
    raw = int(state.position_raw[5])
    width = width_from_position(raw)
    load = float(state.load_ratio[5])
    temp = int(state.temperature_c[5])
    over = width - commanded_mm
    print(f"  명령 {commanded_mm:5.1f}mm → 실제 {width:5.1f}mm (raw {raw})  "
          f"과주행 {over:+5.1f}mm  load {load:.4f}  {temp}°C")
    log.log("force_sample", label=label, commanded_mm=commanded_mm,
            present_raw=raw, width_mm=width, overtravel_mm=over,
            load_ratio=round(load, 4), temperature_c=temp)
    return raw, width, load, temp


def sweep(node, log, label, start_mm, min_mm, step_mm):
    """start_mm에서 min_mm까지 좁혀 가며 매 스텝 실측한다."""
    rows = []
    commanded = start_mm
    previous_raw = None
    while commanded >= min_mm - 1e-9:
        resp = node.set_gripper(commanded)
        if resp is None or not resp.ok:
            print(f"  [{commanded:.1f}mm] set_gripper 실패 — 중단")
            break
        time.sleep(STEP_SETTLE_S)
        row = sample(node, log, label, commanded)
        if row is None:
            break
        raw, _, _, temp = row
        rows.append((commanded,) + row)

        if temp > MAX_SERVO6_TEMP_C:
            print(f"  ⚠️ servo 6 온도 {temp}°C — 상한 {MAX_SERVO6_TEMP_C}°C 초과, 중단합니다")
            break
        if previous_raw is not None and abs(raw - previous_raw) <= STOPPED_RAW:
            print(f"     (위치가 {STOPPED_RAW} raw 안에서만 변합니다 — 하드스톱으로 보입니다)")
        previous_raw = raw
        commanded -= step_mm
    return rows


def report(rows, label):
    if not rows:
        print(f"\n=== {label}: 표본 없음 ===")
        return
    print(f"\n=== {label} 정리 ===")
    stalled = rows[-1]
    print(f"  가장 좁게 명령한 값 {stalled[0]:.1f}mm에서 실제 {stalled[2]:.1f}mm "
          f"(raw {stalled[1]}), load {stalled[3]:.4f}, {stalled[4]}°C")
    first = rows[0]
    d_over = stalled[2] - stalled[0] - (first[2] - first[0])
    d_load = stalled[3] - first[3]
    if abs(d_over) > 0.1:
        print(f"  과주행 {d_over:+.1f}mm에 load {d_load:+.4f} → "
              f"mm당 약 {d_load / d_over:+.4f}")
    raws = [r[1] for r in rows]
    moved = max(raws) - min(raws)
    verdict = "움직임 없음 — 이미 하드스톱" if moved <= STOPPED_RAW else "아직 여유가 있습니다"
    print(f"  present raw 범위 {min(raws)}~{max(raws)} ({verdict})")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--empty", action="store_true",
                      help="빈 턱으로 하드스톱을 찾는다 (팔 앞에 아무것도 없어야 함)")
    mode.add_argument("--holding", metavar="CLS", choices=sorted(CLASS_TO_PROFILE),
                      help="이 클래스를 문 채 힘 곡선을 훑는다 (손으로 물려 주세요)")
    ap.add_argument("--min-mm", type=float, default=SWEEP_MIN_MM)
    ap.add_argument("--step-mm", type=float, default=SWEEP_STEP_MM)
    args = ap.parse_args()

    label = "empty" if args.empty else args.holding
    start_mm = GRIPPER_CLOSED_MM
    if args.holding:
        profile = CLASS_TO_PROFILE[args.holding]
        start_mm = FLOOR_GRASP_PROFILES[profile].close_width_mm

    log = RunLog(label, "gripper_force_probe")
    print("=== 그리퍼 파지력 실측 ===")
    print(f"모드: {'빈 턱 하드스톱' if args.empty else f'{args.holding}을 문 채 힘 곡선'}")
    print(f"스윕: {start_mm:.1f}mm → {args.min_mm:.1f}mm, {args.step_mm:.1f}mm 간격")
    print(f"상세 로그: {log.path}")
    if args.empty:
        print("⚠️ 턱 사이에 아무것도 없어야 합니다 — 턱이 서로를 밀게 됩니다.")
    else:
        print(f"⚠️ {args.holding}을 턱 사이에 손으로 물려 두고 시작하세요.")
    print(f"⚠️ servo 6이 {MAX_SERVO6_TEMP_C}°C를 넘으면 즉시 중단합니다.")
    if input("준비되면 Enter (q=중단): ").strip().lower() == "q":
        return 2

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GraspTestNode()
    lowered = False
    try:
        # 사전 점검 — 이미 뜨겁거나 버스가 안 읽히면 아무것도 하지 않는다.
        pre = sample(node, log, f"{label}:사전점검", start_mm)
        if pre is None:
            print("[실패] 시작 전 서보 상태를 읽지 못했습니다 — arm_driver를 확인하세요")
            return 1
        if pre[3] > MAX_SERVO6_TEMP_C:
            print(f"[실패] servo 6이 이미 {pre[3]}°C입니다 "
                  f"(상한 {MAX_SERVO6_TEMP_C}°C) — 식힌 뒤 다시 하세요")
            return 1

        lowered = set_min_width(node, args.min_mm)
        if not lowered:
            print("[실패] 그리퍼 하한을 낮추지 못했습니다 — arm_driver가 최신인지 확인하세요")
            return 1
        print(f"arm_driver 그리퍼 하한을 {args.min_mm:.1f}mm로 잠시 낮췄습니다\n")
        rows = sweep(node, log, label, start_mm, args.min_mm, args.step_mm)
        report(rows, label)
        return 0
    except KeyboardInterrupt:
        print("\n[중단] 운영자 중단")
        return 130
    finally:
        # 어떤 경로로 끝나든 되돌린다 — 이 블록이 이 파일의 핵심 계약이다.
        try:
            node.set_gripper(GRIPPER_CLOSED_MM)
        finally:
            if lowered and not set_min_width(node, GRIPPER_CLOSED_MM):
                print("⚠️⚠️ 그리퍼 하한 복구에 실패했습니다 — arm_driver를 재시작하세요",
                      file=sys.stderr)
            else:
                print(f"그리퍼 하한을 {GRIPPER_CLOSED_MM}mm로 복구했습니다")
        log.log("run_end")
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        print(f"상세 로그: {log.path}")


if __name__ == "__main__":
    sys.exit(main())
