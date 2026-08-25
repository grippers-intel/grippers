#!/usr/bin/env python3
"""물체별 팔 자세·파지 판정 검증 — 주행 없음.

2026-08-25 사용자 지시: "비어있을 때를 baseline으로 보고 각 물체별로
IDLE -> SAFE_145 -> 파지 -> CARRY_IDLE -> BASKET_DROP -> IDLE을 한번씩
돌면서 그리퍼 값, depth camera 값, 서보 값을 검증하는 코드".

이 도구가 하는 일과 하지 않는 일:

    한다   — 등록 자세마다 servo 1..6의 실제 위치·부하·온도·torque를 읽어
             기대 자세와의 잔차를 표로 남긴다.
           — 같은 체크포인트에서 depth 카메라 관측(found/h/w/x)을 남긴다.
           — 빈 회차를 먼저 돌아 **그 세션의 기준선**을 만들고, 이후 물체
             회차를 전부 그 기준선과 비교한다.
    안 한다 — 주행. cmd_vel을 단 한 번도 발행하지 않는다. 물체는 사람이
             팔 앞에 놓고, 필요하면 손으로 턱 사이에 밀어 넣는다.

왜 빈 회차가 먼저인가: load 기준선(demo_rook_run의 EMPTY_CARRY_LOAD=0.0352)은
2026-08-24에 한 번 잰 상수인데, 빈 그리퍼의 부하는 배터리 전압과 서보 온도에
따라 움직인다. 같은 세션에서 방금 잰 값과 비교하면 그 변동이 상쇄된다.

servo 6 잔차는 **실패가 아니라 측정값**이다. servo 6에는 토크 제한 레지스터가
없어 파지력이 곧 위치 오차이므로, 물체를 문 상태에서 명령 폭에 도달하지 못하는
것이 정상이고 오히려 그 오차 크기가 "얼마나 세게 쥐고 있는가"다.

사전 준비:
  - depth_camera · depth_cam_rotate_node · perception_node · arm_driver
  - 팔 아래에 바구니나 완충재를 둘 것 — 투하 단계에서 물체가 195mm 높이에서
    떨어진다.
  - odom_publisher는 필요 없다(주행하지 않는다).

사용:
  python3 tools/pose_verify_cycle.py                      # 빈 회차 + 6종 전부
  python3 tools/pose_verify_cycle.py --classes rook,queen # 빈 회차 + 2종
  python3 tools/pose_verify_cycle.py --baseline-only      # 빈 회차만
  python3 tools/pose_verify_cycle.py --skip-baseline --classes rook
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from grasp_test_console import (
    CLASS_TO_PROFILE,
    GRIPPER_CLOSED_MM,
    GraspTestNode,
    RunLog,
    estimate_position,
)
from grippers_arm.floor_grasp_profiles import (
    FLOOR_GRASP_PROFILES,
    GRASP_OBJECT_CENTER_FORWARD_MM,
)
from grippers_arm.gripper_calibration import width_from_position
from pose_verify_expectations import (
    CYCLE_CHECKPOINTS,
    EMPTY_CLOSE_WIDTH_ERROR_MM,
    POSE_TOLERANCE_RAW,
    SLIP_CLOSURE_MM,
    empty_cycle_is_contaminated,
    expected_gripper_mm,
    expected_poses,
    load_verdict,
    pose_ok,
    pose_residuals,
    slip_verdict,
    vision_verdict_no_drive,
)
from rclpy.signals import SignalHandlerOptions

# load 판정 여유. 단위는 양자 하나 = 4/1023 = 0.00391.
#
# ⚠️ 2026-08-25에 2양자(0.0078)에서 4양자로 올렸다. knight를 CARRY_IDLE로
# 접다가 놓쳤는데 load가 빈 기준선보다 정확히 **2양자** 높아 성공으로
# 판정됐다 — 즉 예전 값은 놓친 상태와 문 상태의 경계 바로 위에 있었다.
#
# 4양자가 안전한 이유는 같은 회차 실측이 보여준다. 실제로 물고 있던 다섯
# 물체의 여유는 7 / 14 / 19 / 20 / 29양자였고, 놓친 knight만 2양자였다.
# 4는 그 사이에서 아래쪽(queen의 7)에 3양자를 남긴다.
#
# 애초에 2양자가 부족했던 근본 이유: **빈 그리퍼의 load 자체가 자세마다
# 6~11양자로 흔들린다**(같은 빈 회차 안에서 관측된 폭이다). 한 지점에서
# 잰 기준선과 2양자 차이를 다투는 것은 그 흔들림보다 작은 값을 다투는
# 것이었다.
LOAD_MARGIN = 0.0156
# 그리퍼 개폐와 관절 이동 뒤 부하가 정착할 때까지의 여유(GRASP_SETTLE_SEC과
# 같은 실측 근거 — 1.0s 안정 + 여유).
SETTLE_S = 1.2

# 빈 회차에서 쓸 profile. 팔 자세는 회차마다 물체별로 달라지므로 빈 회차는
# 자세 기준선이 아니라 **부하·관측 기준선**을 만드는 것이 목적인데, 그래도
# 어떤 자세로든 한 바퀴 돌아야 하므로 하나를 골라야 한다. rook은 지금까지
# 실기 회차가 가장 많아 비교할 과거 데이터가 제일 많다.
BASELINE_PROFILE = "chess_rook"
BASELINE_RAW_CLS = "rook"


# --classes all의 순서. **알파벳 순이 아니다** — 그러면 box가 맨 앞에 오는데,
# box(큐브)는 검출이 가장 불안정한 클래스라(사용자 확인, 2026-08-25: "이전
# 세션에서도 cube(box) 검출이 잘 안되었으니까") 첫 회차가 실패하면 도구가
# 고장난 것처럼 보인다. 실측 이력이 가장 많고 거리 보정값 K가 있는 rook을
# 먼저 돌려 파이프라인이 멀쩡하다는 것을 확인한 뒤 어려운 것으로 넘어간다.
#
# 거리 보정값이 없는 둘(box, star — K_CLASS가 None)을 뒤로 보낸 것도 같은
# 이유다. 그쪽은 전방 거리 표시 자체가 안 나온다.
DEFAULT_CLASS_ORDER = ("rook", "knight", "queen", "soccer", "box", "star")


def default_class_order():
    """DEFAULT_CLASS_ORDER 중 실제로 존재하는 클래스만. 새 클래스가 생기면
    빠뜨리지 않고 뒤에 붙인다 — 순서를 손으로 관리하되 누락은 막는다."""
    known = list(CLASS_TO_PROFILE)
    ordered = [c for c in DEFAULT_CLASS_ORDER if c in known]
    return ordered + [c for c in sorted(known) if c not in ordered]


def _prompt(text: str) -> None:
    """Enter를 기다린다. q면 중단."""
    if input(text).strip().lower() == "q":
        raise KeyboardInterrupt


class ArmSnapshot:
    """GetArmState 응답을 파이썬 기본형으로 옮긴 것.

    ⚠️ ROS 메시지의 고정 길이 배열 필드는 numpy 배열이고, list()로 감싸도
    원소는 numpy 스칼라로 남는다. 그대로 두면 json.dumps가 죽고(2026-08-25
    첫 실행이 여기서 끊겼다) 산술 결과도 numpy 타입으로 전파된다. 읽자마자
    한 번 변환해 두면 아래로 흐르는 코드가 전부 평범한 int/float만 다룬다.
    """

    __slots__ = ("position_raw", "load_ratio", "temperature_c", "torque_on")

    def __init__(self, response):
        self.position_raw = [int(v) for v in response.position_raw]
        self.load_ratio = [float(v) for v in response.load_ratio]
        self.temperature_c = [int(v) for v in response.temperature_c]
        self.torque_on = [bool(v) for v in response.torque_on]


def read_state(node, log, checkpoint):
    """servo 1..6 실측. 실패하면 None."""
    response = node.arm_state()
    if response is None or not response.ok:
        message = "응답 없음" if response is None else response.message
        print(f"    [서보] 읽기 실패 — {message}")
        log.log("arm_state", checkpoint=checkpoint, ok=False, message=message)
        return None
    state = ArmSnapshot(response)
    log.log(
        "arm_state",
        checkpoint=checkpoint,
        ok=True,
        position_raw=state.position_raw,
        load_ratio=[round(v, 4) for v in state.load_ratio],
        temperature_c=state.temperature_c,
        torque_on=state.torque_on,
    )
    return state


def observe(node, log, checkpoint, raw_cls):
    """depth 카메라 관측 한 번. (found, h, w, x) 또는 None."""
    obs = node.observe(raw_cls, timeout_sec=1.5)
    if obs is None:
        print("    [관측] 응답 없음")
        log.log("observe", checkpoint=checkpoint, ok=False)
        return None
    if not obs.found:
        print(f"    [관측] {raw_cls} 안 보임")
        log.log("observe", checkpoint=checkpoint, ok=True, found=False)
        return obs
    forward_m, lateral_m = estimate_position(obs, raw_cls)
    # estimate_position은 obs.h/obs.w(numpy float32)에서 계산하므로 결과도
    # numpy다 — ArmSnapshot과 같은 이유로 여기서 기본형으로 내린다.
    forward_m = None if forward_m is None else float(forward_m)
    lateral_m = None if lateral_m is None else float(lateral_m)
    where = ""
    if forward_m is not None:
        where = f" · 전방 {forward_m * 100:.1f}cm 좌우 {lateral_m * 100:+.1f}cm"
    print(f"    [관측] {raw_cls} h={obs.h:.1f}px w={obs.w:.1f}px x={obs.x:.1f}px{where}")
    log.log(
        "observe",
        checkpoint=checkpoint,
        ok=True,
        found=True,
        h=float(obs.h),
        w=float(obs.w),
        x=float(obs.x),
        forward_m=forward_m,
        lateral_m=lateral_m,
    )
    return obs


def report_checkpoint(state, expected_pose, expected_mm, baseline):
    """한 체크포인트의 표 한 줄들을 출력하고, 나중에 비교할 요약을 돌려준다."""
    if state is None:
        return None

    actual = list(state.position_raw)
    residuals = pose_residuals(expected_pose, actual[:5])
    ok = pose_ok(residuals)
    joints = " ".join(
        f"s{i + 1}={actual[i]}({residuals[i]:+d})" for i in range(5)
    )
    print(f"    [서보] {joints}  → {'OK' if ok else '⚠️ 허용치 초과'}")

    gripper_raw = actual[5]
    gripper_mm = width_from_position(gripper_raw)
    if expected_mm is None:
        print(f"    [그리퍼] {gripper_mm:.1f}mm (raw {gripper_raw}) — 명령 없음")
        width_error = None
    else:
        width_error = gripper_mm - expected_mm
        # 이 오차는 실패가 아니라 파지력의 대리 측정값이다(모듈 docstring 참고).
        print(
            f"    [그리퍼] 명령 {expected_mm:.1f}mm → 실제 {gripper_mm:.1f}mm "
            f"({width_error:+.1f}mm, raw {gripper_raw}) = 파지력 대리값"
        )

    load = state.load_ratio[5]
    delta = "" if baseline is None else f"  (빈 기준선 대비 {load - baseline:+.4f})"
    print(f"    [부하] servo6 {load:.4f}{delta}")

    hot = [
        f"s{i + 1}={state.temperature_c[i]}°C"
        for i in range(6)
        if state.temperature_c[i] >= 45
    ]
    if hot:
        print(f"    [온도] ⚠️ {' '.join(hot)}")
    torque_off = [i + 1 for i in range(6) if not state.torque_on[i]]
    if torque_off:
        print(f"    [torque] ⚠️ OFF: {torque_off}")

    return {
        "pose_ok": ok,
        "residuals": residuals,
        "gripper_mm": gripper_mm,
        "width_error": width_error,
        "load": load,
        "worst_joint": max(range(5), key=lambda i: abs(residuals[i])) + 1,
        "worst_raw": max(residuals, key=abs),
    }


def run_cycle(node, log, *, raw_cls, profile, empty, baseline, interactive):
    """한 회차를 끝까지 돈다. 체크포인트별 요약 dict를 돌려준다.

    실패하면 (요약, 실패사유)에서 사유가 채워진다 — 팔은 그 자리에 남으므로
    호출부가 recover를 결정한다."""
    label = "빈 회차(기준선)" if empty else raw_cls
    spec = FLOOR_GRASP_PROFILES[profile]
    print(f"\n{'=' * 68}\n=== {label} — profile={profile}")
    print(f"    preopen={spec.preopen_width_mm}mm  close={spec.close_width_mm}mm  "
          f"release={spec.release_width_mm}mm  파지중심={spec.grasp_center_height_mm}mm")
    log.log("cycle_start", raw_cls=raw_cls, profile=profile, empty=empty)

    results = {}

    # servo1은 safe/grasp/midpoint 동안 얼어 있다 — arm_driver가 이동을 시작할
    # 때 읽은 값이 그대로 기대값이 되므로, 우리도 같은 시점에 읽어 둔다.
    start_state = read_state(node, log, "idle_start")
    if start_state is None:
        return results, "시작 시 서보 상태를 읽지 못했습니다"
    frozen_servo1 = start_state.position_raw[0]
    poses = expected_poses(profile, frozen_servo1)
    print(f"    servo1 동결값 = {frozen_servo1} (safe/grasp/midpoint 기대값에 사용)")

    def checkpoint(name, state=None):
        pose_key, width_key = next(
            (p, w) for n, p, w in CYCLE_CHECKPOINTS if n == name
        )
        print(f"  [{name}]")
        state = state if state is not None else read_state(node, log, name)
        summary = report_checkpoint(
            state,
            poses[pose_key],
            expected_gripper_mm(profile, width_key),
            None if baseline is None else baseline.get(name, {}).get("load"),
        )
        if summary is not None:
            results[name] = summary
            log.log("checkpoint", name=name, **{
                k: v for k, v in summary.items() if k != "residuals"
            }, residuals=summary["residuals"])
        return summary

    checkpoint("idle_start", start_state)

    if interactive:
        if empty:
            # ⚠️ 2026-08-25: 이 확인이 없어 빈 회차가 앞에 놓인 물체를 집어
            # 올렸고, 오염된 기준선으로 이후 회차를 전부 비교했다.
            _prompt("\n  빈 회차입니다 — 팔 앞에 아무것도 없어야 합니다. "
                    "확인하고 Enter (q=중단): ")
        else:
            _prompt(f"\n  물체 **중심**을 차체 전면에서 "
                    f"{GRASP_OBJECT_CENTER_FORWARD_MM / 10:.0f}cm 앞, 정면에 놓고 Enter "
                    "(q=중단): ")

    # 내려가면 팔이 depth 카메라를 가린다 — 정면을 볼 수 있는 마지막 순간이다.
    before = observe(node, log, "before_descend", raw_cls)
    h_before = float(before.h) if before is not None and before.found else None

    print("\n  safe로 이동")
    if not node.move_floor_pose(profile, "safe"):
        return results, "safe 이동 실패"
    time.sleep(SETTLE_S)
    checkpoint("safe_down")

    # 내려가기 전에 연다 — 닫힌 손가락이 물체 자리를 통과하며 밀어내지 않게.
    print(f"\n  그리퍼 열기 {spec.preopen_width_mm}mm (내려가기 전)")
    node.set_gripper(spec.preopen_width_mm)
    time.sleep(SETTLE_S)
    checkpoint("preopen")

    print("\n  grasp로 이동")
    if not node.move_floor_pose(profile, "grasp"):
        return results, "grasp 이동 실패"
    time.sleep(SETTLE_S)
    checkpoint("grasp")

    if not empty and interactive:
        _prompt("\n  물체가 열린 턱 사이에 있는지 확인하고 Enter — "
                "필요하면 손으로 밀어 넣으세요 (q=중단): ")

    print(f"\n  그리퍼 닫기 {spec.close_width_mm}mm")
    resp = node.set_gripper(spec.close_width_mm)
    if resp is None or not resp.ok:
        return results, "그리퍼 닫기 실패"
    time.sleep(SETTLE_S)
    closed_summary = checkpoint("closed")

    # 빈 회차가 정말 비었는지는 여기서만 알 수 있다. 여기서 멈추지는 않는다 —
    # 물고 있는 채로 중단하면 팔이 바닥 높이에 물체를 든 채 남는다. 회차는
    # 끝까지 돌려 바구니에 놓고 IDLE로 돌아온 뒤, main이 기준선을 버린다.
    if empty and closed_summary is not None:
        contaminated = empty_cycle_is_contaminated(closed_summary["width_error"])
        results["_contaminated"] = bool(contaminated)
        if contaminated:
            print(f"\n  ⚠️⚠️ 빈 회차인데 그리퍼가 무언가를 물었습니다 "
                  f"(폭 오차 {closed_summary['width_error']:+.1f}mm, "
                  f"상한 {EMPTY_CLOSE_WIDTH_ERROR_MM}mm).")
            print("     이 기준선은 쓸 수 없습니다 — 회차는 끝까지 돌려 "
                  "물체를 내려놓고 IDLE로 복귀합니다.")

    # 바닥에서 IDLE로 곧장 가면 그리퍼가 바닥을 쓸어간다 — 검증된 상승 체인.
    for stage, name in (("midpoint", "midpoint_up"), ("safe", "safe_up"), ("idle", "carry_idle")):
        print(f"\n  {stage}로 이동")
        if not node.move_floor_pose(profile, stage):
            return results, f"{stage} 이동 실패"
        time.sleep(SETTLE_S)
        checkpoint(name)

    after = observe(node, log, "carry_idle", raw_cls)
    h_after = float(after.h) if after is not None and after.found else None
    found_after = None if after is None else bool(after.found)

    carry = results.get("carry_idle", {}).get("load")
    baseline_carry = None if baseline is None else baseline.get("carry_idle", {}).get("load")
    width_closed = results.get("closed", {}).get("gripper_mm")
    width_carry = results.get("carry_idle", {}).get("gripper_mm")
    verdicts = {
        # 순서가 곧 신뢰도 순이다 — slip이 가장 직접적인 기계적 증거다.
        "slip": slip_verdict(width_closed, width_carry),
        "load": load_verdict(carry, baseline_carry, LOAD_MARGIN),
        "vision": vision_verdict_no_drive(h_before, found_after),
    }
    print_verdicts(label, carry, baseline_carry, h_before, h_after,
                   width_closed, width_carry, verdicts, empty)
    log.log("verdict", raw_cls=raw_cls, empty=empty, carry_load=carry,
            baseline_carry=baseline_carry, h_before=h_before, h_after=h_after,
            width_closed_mm=width_closed, width_carry_mm=width_carry, **verdicts)
    results["_verdicts"] = verdicts

    print("\n  drop으로 이동")
    if not node.move_floor_pose(profile, "drop"):
        return results, "drop 이동 실패"
    time.sleep(SETTLE_S)
    checkpoint("drop")

    # 활짝 열지 않는다 — 물체가 턱 사이에서 빠져나올 만큼만.
    print(f"\n  그리퍼 열기 {spec.release_width_mm}mm (투하)")
    node.set_gripper(spec.release_width_mm)
    time.sleep(SETTLE_S)
    checkpoint("released")

    # 접기 **전에** 닫는다 — 다음 동작이 요구하는 형상을 그 동작 전에 만든다.
    print(f"\n  그리퍼 닫기 {GRIPPER_CLOSED_MM}mm (IDLE 복귀 전)")
    node.set_gripper(GRIPPER_CLOSED_MM)
    time.sleep(SETTLE_S)
    checkpoint("closed_to_fold")

    print("\n  idle로 복귀")
    if not node.move_floor_pose(profile, "idle"):
        return results, "idle 복귀 실패"
    time.sleep(SETTLE_S)
    checkpoint("idle_end")

    log.log("cycle_ok", raw_cls=raw_cls, profile=profile, empty=empty)
    return results, None


def print_verdicts(label, carry, baseline_carry, h_before, h_after,
                   width_closed, width_carry, verdicts, empty):
    print(f"\n  --- {label} 파지 판정 ---")
    if empty:
        print("    빈 회차 — 판정하지 않고 기준선으로만 씁니다"
              f" (CARRY_IDLE load = {carry if carry is None else round(carry, 4)})")
        return

    # [1] 미끄러짐 — 가장 직접적인 기계적 증거다. 물체가 빠지면 그것을 막던
    # 것이 없어지므로 턱이 그만큼 더 닫힌다.
    slipped = verdicts["slip"]
    if slipped is None:
        print("    [미끄러짐] 판정 불가(그리퍼 폭을 못 읽었습니다)")
    else:
        closure = width_closed - width_carry
        print(f"    [미끄러짐] 파지 직후 {width_closed:.1f}mm → CARRY {width_carry:.1f}mm "
              f"= 턱이 {closure:+.1f}mm 더 닫힘 (상한 {SLIP_CLOSURE_MM}mm) → "
              f"{'⚠️ 운반 중 놓침' if slipped else '유지'}")

    load_ok = verdicts["load"]
    if load_ok is None:
        print("    [load] 판정 불가")
    else:
        margin = carry - baseline_carry
        print(f"    [load] {carry:.4f} vs 빈 {baseline_carry:.4f} = {margin:+.4f} "
              f"({margin / 0.00391:+.1f}양자, 요구 {LOAD_MARGIN / 0.00391:.0f}양자) → "
              f"{'성공' if load_ok else '실패'}")

    vision = verdicts["vision"]
    if vision is None:
        print("    [시각] 판정 불가(기준 관측 또는 응답 없음)")
    else:
        after = "안 보임" if h_after is None else f"h={h_after:.1f}px로 보임"
        print(f"    [시각] 기준 h={h_before:.1f}px → 지금 {after} → "
              f"{'성공(사라짐)' if vision else '⚠️ 실패(아직 바닥에 있습니다)'}")

    # ⚠️ slip은 극성이 반대다 — True가 "놓쳤다"(나쁨)이고, load/vision은
    # True가 "성공"이다. 실패 신호가 하나라도 있으면 실패로 본다: 이쪽
    # 방향의 오판은 사람이 눈으로 확인하게 만들 뿐이지만, 반대 방향은
    # 빈 그리퍼로 미션을 계속하게 한다.
    failures = []
    if verdicts["slip"] is True:
        failures.append("미끄러짐")
    if verdicts["load"] is False:
        failures.append("load")
    if verdicts["vision"] is False:
        failures.append("시각")
    undecided = [key for key, value in verdicts.items() if value is None]

    if len(undecided) == len(verdicts):
        print("    판정 불가 — 어떤 신호도 읽지 못했습니다")
    elif failures:
        print(f"    ⚠️⚠️ 파지 실패 — 근거: {', '.join(failures)}")
    elif not undecided:
        print("    ★★ 세 신호 모두 성공")
    else:
        print(f"    ★ 성공 — 읽은 신호 모두 성공 (판정 불가: {', '.join(undecided)})")


def print_summary(all_results):
    print(f"\n{'=' * 68}\n=== 전체 요약")
    header = f"{'회차':<10}{'자세 최악':<22}{'닫힘 폭오차':<14}{'CARRY load':<12}{'판정'}"
    print(header)
    print("-" * 68)
    for label, results in all_results.items():
        checkpoints = [
            (n, r) for n, r in (results or {}).items() if not n.startswith("_")
        ]
        if not checkpoints:
            print(f"{label:<10}(기록 없음 — 회차가 첫 체크포인트 전에 끊겼습니다)")
            continue
        worst_name, worst = max(checkpoints, key=lambda item: abs(item[1]["worst_raw"]))
        pose_cell = f"{worst_name} s{worst['worst_joint']} {worst['worst_raw']:+d}"
        closed = results.get("closed", {})
        width_cell = (
            "-" if closed.get("width_error") is None else f"{closed['width_error']:+.1f}mm"
        )
        carry = results.get("carry_idle", {}).get("load")
        load_cell = "-" if carry is None else f"{carry:.4f}"
        verdicts = results.get("_verdicts", {})
        # slip만 극성이 반대다(True=놓침). 표에서는 전부 "성공/실패"로
        # 통일해 읽는 사람이 신호마다 방향을 되새기지 않게 한다.
        labels = []
        for key, value in verdicts.items():
            good = (value is False) if key == "slip" else (value is True)
            bad = (value is True) if key == "slip" else (value is False)
            labels.append(f"{key}={'성공' if good else ('실패' if bad else '불가')}")
        verdict_cell = " / ".join(labels) or "-"
        if results.get("_contaminated"):
            verdict_cell = "⚠️ 오염 — 빈 회차가 물체를 물었습니다"
        print(f"{label:<10}{pose_cell:<22}{width_cell:<14}{load_cell:<12}{verdict_cell}")
    print(f"\n자세 허용치 ±{POSE_TOLERANCE_RAW} raw. 그리퍼 폭오차는 실패가 아니라 "
          "파지력 대리값입니다.")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--classes", default="all",
                    help=f"쉼표 구분. all = {','.join(sorted(CLASS_TO_PROFILE))}")
    ap.add_argument("--baseline-only", action="store_true", help="빈 회차만 돌고 끝낸다")
    ap.add_argument("--skip-baseline", action="store_true",
                    help="빈 회차를 건너뛴다(기준선 비교가 없어진다)")
    ap.add_argument("--no-prompt", action="store_true",
                    help="물체 배치 확인 프롬프트를 띄우지 않는다(빈 회차 자동화용)")
    args = ap.parse_args()

    if args.classes == "all":
        classes = default_class_order()
    else:
        classes = [c.strip() for c in args.classes.split(",") if c.strip()]
    unknown = [c for c in classes if c not in CLASS_TO_PROFILE]
    if unknown:
        print(f"알 수 없는 클래스: {unknown} — 가능: {sorted(CLASS_TO_PROFILE)}", file=sys.stderr)
        return 2
    if args.baseline_only:
        classes = []

    log = RunLog(",".join(classes) or "empty", "pose_verify_cycle")
    print("=== 자세·파지 검증 회차 (주행 없음) ===")
    print(f"상세 로그: {log.path}")
    print(f"⚠️ 물체 배치 전제: 중심이 차체 전면에서 "
          f"{GRASP_OBJECT_CENTER_FORWARD_MM / 10:.0f}cm 앞, 정면입니다.")
    print("⚠️ 팔 아래에 바구니나 완충재를 두세요 — 투하 단계에서 물체가 떨어집니다.")
    print("⚠️ 팔은 IDLE에서 시작합니다. 벗어나 있으면 arm_driver가 첫 이동 때 자동 정렬합니다.")

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GraspTestNode()
    all_results = {}
    exit_code = 0
    try:
        baseline = None
        if not args.skip_baseline:
            baseline, why = run_cycle(
                node, log, raw_cls=BASELINE_RAW_CLS, profile=BASELINE_PROFILE,
                empty=True, baseline=None, interactive=not args.no_prompt)
            all_results["빈 회차"] = baseline
            if why:
                print(f"\n[중단] 빈 회차 실패: {why}")
                log.log("cycle_failed", raw_cls="empty", why=why)
                return 3
            if baseline.get("_contaminated"):
                print("\n[중단] 빈 기준선이 오염됐습니다 — 팔 앞의 물체를 모두 치우고 "
                      "다시 실행하세요.")
                print("  (기준선 없이 자세만 보려면 --skip-baseline)")
                log.log("cycle_failed", raw_cls="empty", why="contaminated_baseline")
                print_summary(all_results)
                return 3

        for raw_cls in classes:
            results, why = run_cycle(
                node, log, raw_cls=raw_cls, profile=CLASS_TO_PROFILE[raw_cls],
                empty=False, baseline=baseline, interactive=not args.no_prompt)
            all_results[raw_cls] = results
            if why:
                print(f"\n[중단] {raw_cls} 실패: {why} — 팔은 그 자리에 있습니다.")
                print("  arm_driver의 recover_idle로 복귀시킨 뒤 다시 실행하세요.")
                log.log("cycle_failed", raw_cls=raw_cls, why=why)
                exit_code = 4
                break

        print_summary(all_results)
        return exit_code

    except KeyboardInterrupt:
        print("\n[중단] 운영자 중단 — 팔 상태는 직접 확인하세요(자동 복구 없음).")
        log.log("aborted")
        print_summary(all_results)
        return 130
    finally:
        log.log("run_end")
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        print(f"상세 로그: {log.path}")


if __name__ == "__main__":
    sys.exit(main())
