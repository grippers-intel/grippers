#!/usr/bin/env python3
"""팔만 쓰는 GRASP 한 사이클 + 측정값 수집 — 주행 없음.

2026-08-24 사용자 지시: "6개 물체를 바로 파지하고 바구니에 넣는 매커니즘.
먼저 빈채로 테스트하고, 각 물체를 팔이 내려와서 바로 잡을 수 있는 위치에
놓고 depth 카메라 면적·중심과 파지 시 load·그리퍼캠 면적을 빈 상태와
비교해 기록할 것."

이 도구는 **차를 전혀 움직이지 않는다.** 물체를 사람이 파지 가능한 자리에
직접 놓고, 팔만 내려가 잡는다. auto_grasp_sequence.py(정렬·주행 포함)와
용도가 다르다 — 이쪽은 **자세와 측정값을 확정하기 위한 데이터 수집**이다.

    python3 grasp_cycle.py --empty              빈 상태 기준선 (먼저 이것부터)
    python3 grasp_cycle.py --raw-cls rook       물체를 놓고 한 사이클
    python3 grasp_cycle.py --raw-cls rook --no-drop   바구니 투하 생략

기록하는 것 (전부 DATASET_PATH에 한 줄로 누적된다):

  depth 카메라(파지 직전, 팔이 내려가기 전)
      x, y 중심 픽셀 · h, w · 면적(h*w) · 추정 전방거리 · 추정 좌우거리
      -> 물체마다 "파지 가능한 자리"가 화면에서 어떻게 보이는지의 정답표가 된다

  그리퍼 캠 면적
      열고 내려온 직후(파지 전) · 닫은 직후
      -> 빈 상태와의 차이가 곧 "물체가 손가락 사이에 있다"는 신호다

  load_ratio
      닫은 직후 · midpoint(들어올린 뒤) · safe · drop 직전 · 놓은 뒤
      -> 빈 상태와의 차이가 곧 "물체를 물고 있다"는 신호다

⚠️ 빈 상태 기준선을 먼저 받아야 나머지가 의미를 갖는다. 그리퍼캠 면적은
밝기 임계(>150) 최대 컨투어라 바닥 재질·조명·손가락 자체가 이미 어느 정도
면적을 만든다 — 그 값을 모르면 물체 면적을 해석할 수 없다. load도 마찬가지로
빈 채로 닫아도 0이 아니다.

사전 준비: odom_publisher는 필요 없다(주행 안 함). depth_camera ·
depth_cam_rotate_node · perception_node · arm_driver 가 떠 있어야 한다.
--empty 는 perception 없이도 돌지만, 그러면 depth 관측 항목이 비어 기록된다.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions

from grasp_test_console import (
    CLASS_TO_PROFILE,
    GRIPPER_CLOSED_MM,
    GraspTestNode,
    GripperCam,
    RunLog,
    estimate_position,
    restart_perception_node,
    save_yolo_annotated,
)
from grippers_arm.floor_grasp_profiles import FLOOR_GRASP_PROFILES

# 실행마다 한 줄씩 누적되는 데이터셋. 호스트
# ~/docker/shared/grippers/recordings 와 같은 자리라 맥북에서 scp로 바로 꺼낼 수
# 있다. RunLog(/tmp/...)는 한 실행의 상세 기록이고, 이쪽은 실행 간 비교용
# 요약이라 목적이 다르다.
DATASET_PATH = "/grippers/recordings/grasp_dataset.jsonl"

# 닫힘/이동 뒤 load가 정착할 때까지의 여유. arm_driver가 이미 그리퍼 정지까지
# 기다리지만(_wait_gripper_motion_settled), load 값 자체는 조금 더 늦게 안정된다
# (GRASP_SETTLE_SEC 주석의 실측: 0.77s 거의 안정, 1.03s 이후 불변).
LOAD_SETTLE_S = 1.2

LOAD_THRESHOLD = 0.04  # domain/task/states.py GraspState.LOAD_THRESHOLD과 동일


def measure_area(cam, label, log, samples=5):
    """그리퍼캠 면적을 여러 번 재서 중앙값을 쓴다.

    단발 측정은 프레임마다 크게 튄다 — 2026-08-24 실기 4단계 로그에서 1초
    간격 연속 표본이 22564 -> 28430 -> 12794 -> 4383 -> 46480처럼 흔들렸다.
    데이터로 남길 값은 그 잡음을 걷어내야 한다."""
    values = []
    for _ in range(samples):
        area = cam.measure_area_px2()
        if area is not None:
            values.append(area)
        time.sleep(0.15)
    if not values:
        print(f"  [{label}] 그리퍼캠 면적: 검출 안 됨")
        log.log("area", where=label, area_px2=None, samples=0)
        return None
    values.sort()
    median = values[len(values) // 2]
    print(f"  [{label}] 그리퍼캠 면적: {median:.0f}px²  (표본 {len(values)}, "
          f"최소 {values[0]:.0f} 최대 {values[-1]:.0f})")
    log.log("area", where=label, area_px2=median, samples=len(values),
            min_px2=values[0], max_px2=values[-1])
    return median


def measure_load(node, label, log):
    time.sleep(LOAD_SETTLE_S)
    load = node.get_load()
    if load is None:
        print(f"  [{label}] load: 읽기 실패")
    else:
        print(f"  [{label}] load_ratio: {load:.4f}")
    log.log("load", where=label, load_ratio=load)
    return load


def observe_depth(node, raw_cls, log):
    """파지 직전 자리에서 depth 카메라가 보는 값. 팔이 내려가기 전에 부른다
    (내려간 팔이 화면을 가린다)."""
    obs = node.observe(raw_cls)
    if obs is None or not obs.found:
        print("  [depth] 물체를 못 찾음")
        log.log("depth_observe", found=False)
        return None
    forward_m, lateral_m = estimate_position(obs, raw_cls)
    record = {
        "found": True,
        "x": obs.x,
        "h": obs.h,
        "w": obs.w,
        "area_px2": obs.h * obs.w,
        "forward_m": forward_m,
        "lateral_m": lateral_m,
    }
    print(f"  [depth] x={obs.x:.1f} h={obs.h:.1f} w={obs.w:.1f} "
          f"면적={obs.h * obs.w:.0f}px²")
    if forward_m is not None:
        print(f"          추정 전방 {forward_m * 100:.1f}cm · 좌우 {lateral_m * 100:+.1f}cm")
    log.log("depth_observe", **record)
    return record


def load_baseline():
    """DATASET_PATH에서 가장 최근의 빈 상태 기준선을 읽는다. 없으면 None."""
    try:
        with open(DATASET_PATH, encoding="utf-8") as f:
            rows = [json.loads(line) for line in f if line.strip()]
    except (OSError, json.JSONDecodeError):
        return None
    empties = [r for r in rows if r.get("empty")]
    return empties[-1] if empties else None


def _delta(label, value, base, unit=""):
    if value is None or base is None:
        return f"  {label:<22} {value if value is not None else '—'}  (기준선 없음)"
    return (f"  {label:<22} {value:>10.4f}{unit}  기준선 {base:>10.4f}{unit}  "
            f"차이 {value - base:+10.4f}{unit}")


def print_comparison(record, baseline):
    print("\n=== 빈 상태 기준선과 비교 ===")
    if baseline is None:
        print("  기준선이 없습니다 — `--empty`로 먼저 한 번 돌리세요.")
        return
    print(f"  기준선 기록 시각: {baseline.get('t_iso', '?')}")
    for key, label, unit in (
        ("area_open", "그리퍼캠(열림)", "px²"),
        ("area_closed", "그리퍼캠(닫힘)", "px²"),
        ("load_closed", "load(닫힘)", ""),
        ("load_midpoint", "load(midpoint)", ""),
        ("load_safe", "load(safe)", ""),
    ):
        print(_delta(label, record.get(key), baseline.get(key), unit))
    held = record.get("load_midpoint")
    if held is not None and baseline.get("load_midpoint") is not None:
        margin = held - baseline["load_midpoint"]
        verdict = "물체를 들고 있다고 볼 수 있음" if margin > 0.005 else "⚠️ 빈 상태와 구분이 안 됨"
        print(f"\n  판정: midpoint load가 기준선보다 {margin:+.4f} — {verdict}")


def append_dataset(record):
    try:
        with open(DATASET_PATH, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")
        print(f"데이터셋에 추가: {DATASET_PATH}")
    except OSError as e:
        print(f"[경고] 데이터셋 기록 실패: {e}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raw-cls", default="rook", choices=sorted(CLASS_TO_PROFILE))
    ap.add_argument("--empty", action="store_true",
                    help="물체 없이 돌려 기준선을 만든다 (가장 먼저 할 것)")
    ap.add_argument("--profile", default=None)
    ap.add_argument("--no-drop", action="store_true", help="바구니 투하 단계를 건너뛴다")
    args = ap.parse_args()

    profile = args.profile or CLASS_TO_PROFILE[args.raw_cls]
    close_width_mm = FLOOR_GRASP_PROFILES[profile].close_width_mm
    preopen_mm = FLOOR_GRASP_PROFILES[profile].preopen_width_mm

    log = RunLog(args.raw_cls if not args.empty else "empty", profile)
    mode = "빈 상태 기준선" if args.empty else f"물체 {args.raw_cls}"
    print(f"=== GRASP 사이클 — {mode} ===")
    print(f"profile={profile}  preopen={preopen_mm}mm  close={close_width_mm}mm")
    print(f"상세 로그: {log.path}")
    print("⚠️ 이 도구는 차를 움직이지 않습니다 — 물체를 팔이 바로 잡을 수 있는 자리에 두세요.")

    record = {
        "t": time.time(),
        "t_iso": time.strftime("%Y-%m-%d %H:%M:%S"),
        "empty": bool(args.empty),
        "raw_cls": None if args.empty else args.raw_cls,
        "profile": profile,
        "close_width_mm": close_width_mm,
        "preopen_width_mm": preopen_mm,
    }
    baseline = None if args.empty else load_baseline()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GraspTestNode()
    cam = None
    perception_was_killed = False
    try:
        input("\n배치 완료 후 Enter로 시작 (Ctrl+C로 중단): ")

        # --- depth 관측: 팔이 내려가기 전에 해야 한다 -------------------
        if not args.empty:
            print("\n[1] depth 카메라 관측 (팔 내려가기 전)")
            depth = observe_depth(node, args.raw_cls, log)
            record["depth"] = depth
            capture = save_yolo_annotated(node, args.raw_cls)
            if capture is not None:
                record["yolo_capture"] = capture.get("path")
                log.log("yolo_capture", **capture)

        # --- 팔 내리기: 반드시 열고 내려간다 ----------------------------
        print("\n[2] safe → 그리퍼 열기 → grasp")
        if not node.move_floor_pose(profile, "safe"):
            print("  safe 실패 — arm.log 확인")
            node.move_floor_pose(profile, "recover_idle")
            return 2
        # 내려가기 전에 연다(사용자 지시 2026-08-24) — 닫힌 손가락이 물체가
        # 있는 공간을 통과해 내려가면 물체를 밀어낸다.
        node.set_gripper(preopen_mm)
        print(f"  그리퍼 열림({preopen_mm}mm) — 내려가기 전")
        if not node.move_floor_pose(profile, "grasp"):
            print("  grasp 실패 — arm.log 확인")
            node.move_floor_pose(profile, "recover_idle")
            return 2

        # perception_node가 /dev/gripper_cam을 쥐고 있어 넘겨받는다.
        subprocess.run(["pkill", "-f", "grippers_perception/perception_node"],
                       stdin=subprocess.DEVNULL)
        perception_was_killed = True
        time.sleep(1.0)
        cam = GripperCam()

        print("\n[3] 파지 전 측정 (열린 채 내려온 상태)")
        record["area_open"] = measure_area(cam, "열림", log)
        record["load_open"] = measure_load(node, "열림", log)

        # --- 파지 ------------------------------------------------------
        print("\n[4] 그리퍼 닫기")
        resp = node.set_gripper(close_width_mm)
        if resp is None or not resp.ok:
            print("  닫기 실패 — arm.log 확인")
            node.move_floor_pose(profile, "recover_idle")
            return 3
        record["area_closed"] = measure_area(cam, "닫힘", log)
        record["load_closed"] = measure_load(node, "닫힘", log)

        # --- 들어올리기 ------------------------------------------------
        print("\n[5] midpoint → safe (들어올리기)")
        if not node.move_floor_pose(profile, "midpoint"):
            print("  midpoint 실패 — arm.log 확인")
            node.move_floor_pose(profile, "recover_idle")
            return 4
        record["load_midpoint"] = measure_load(node, "midpoint", log)
        if not node.move_floor_pose(profile, "safe"):
            print("  safe 실패 — arm.log 확인")
            node.move_floor_pose(profile, "recover_idle")
            return 4
        record["load_safe"] = measure_load(node, "safe", log)

        # --- 바구니 투하 ------------------------------------------------
        if args.no_drop:
            print("\n[6] --no-drop — 투하 생략, idle로 복귀합니다")
            node.move_floor_pose(profile, "idle")
        else:
            input("\n[6] 바구니가 팔 아래 오도록 맞춘 뒤 Enter로 투하: ")
            if not node.move_floor_pose(profile, "drop"):
                print("  drop 실패 — arm.log 확인")
                node.move_floor_pose(profile, "recover_idle")
                return 5
            record["load_before_release"] = measure_load(node, "투하 직전", log)
            node.set_gripper(preopen_mm)
            record["load_after_release"] = measure_load(node, "놓은 뒤", log)
            record["area_after_release"] = measure_area(cam, "놓은 뒤", log)
            node.move_floor_pose(profile, "idle")
            node.set_gripper(GRIPPER_CLOSED_MM)

        record["ok"] = True
        print("\n완료 — IDLE 복귀")
        return 0

    except KeyboardInterrupt:
        print("\n[중단] 팔 상태는 직접 확인하세요(자동 복구 없음).")
        record["ok"] = False
        record["aborted"] = True
        log.log("aborted")
        return 130
    finally:
        log.log("run_end")
        log.close()
        if cam is not None:
            cam.close()
        if perception_was_killed:
            restart_perception_node()
        node.destroy_node()
        rclpy.shutdown()

        append_dataset(record)
        if not args.empty:
            print_comparison(record, baseline)
        else:
            print("\n기준선을 기록했습니다 — 이제 물체별로 돌리면 자동으로 비교됩니다.")
        print(f"\n상세 로그: {log.path}")


if __name__ == "__main__":
    sys.exit(main())
