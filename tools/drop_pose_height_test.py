#!/usr/bin/env python3
"""drop 자세(BASKET_DROP_300)를 수평 유지한 채 높이만 바꿀 수 있는지 미리
확인하는 순수 기구학(FK) 테스트.

하드웨어를 전혀 건드리지 않는다 — third_party/soarm_provided_d의 FK 모델만
써서 "이 목표(전방 거리 동일, 높이만 변경, 손가락은 계속 수평)를 만족하는
관절각이 존재하는가, 서보 허용 범위(TAUGHT_POSITION_LIMITS) 안인가"만
계산한다. 실기로 옮기기 전 1차 필터다 — 여기서 PASS가 나와도 실기 확인은
따로 필요하고, FAIL이 나오면 애초에 실기를 시도할 이유가 없다.

풀이 방식: servo1(선회)·servo5(손목 롤)는 좌우/파지 방향과만 관련 있으므로
기준 자세(기본값 BASKET_DROP_300_RAW, production 현재값) 값에 고정하고,
servo2(어깨)·servo3(팔꿈치)·servo4(손목 피치) 세 개만 뉴턴법으로 풀어
(전방 x, 높이 z, pitch) 세 목표를 맞춘다 — 미지수 3개·조건 3개라 이 팔
자세들이 원래 그렇듯 완전 결정계다.

⚠️ 2026-09-04: 195mm였던 기준을 이 도구로 230→250→270→300mm까지 차례로
올려 실기 확인 후 300mm를 채택했다(BASKET_DROP_300_RAW로 floor_grasp_
profiles.py에 반영됨). 기준 자세를 이 기본값 자체로 갱신했으므로, 앞으로
더 조정할 일이 있으면 지금 production 값에서부터 다시 이어서 계산한다.

사용 예:
    .venv/bin/python3 tools/drop_pose_height_test.py --height-mm 320
    .venv/bin/python3 tools/drop_pose_height_test.py --height-mm 320 --lock-servo1-to-idle
"""

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "grippers_arm"))


def _find_soarm_lab() -> Path:
    """third_party/soarm_provided_d는 git 서브모듈이라 워크트리마다 체크아웃
    여부가 다르다 — 없으면 형제 체크아웃(grippers/)도 찾아본다."""
    candidates = [
        ROOT / "third_party" / "soarm_provided_d" / "soarm_lab",
        Path.home() / "Desktop" / "intel" / "grippers" / "third_party" / "soarm_provided_d" / "soarm_lab",
    ]
    for c in candidates:
        if (c / "fk_core.py").exists():
            return c
    raise SystemExit(
        "fk_core.py를 못 찾았습니다 — third_party/soarm_provided_d 서브모듈이 "
        f"필요합니다. 확인한 경로: {[str(c) for c in candidates]}\n"
        "  git submodule update --init third_party/soarm_provided_d"
    )


SOARM_LAB = _find_soarm_lab()
sys.path.insert(0, str(SOARM_LAB))

import numpy as np  # noqa: E402
from fk_core import FKSo101  # noqa: E402

from grippers_arm.floor_grasp_profiles import (  # noqa: E402
    BASKET_DROP_300_RAW,
    IDLE_CRADLE_RAW,
    TAUGHT_POSITION_LIMITS,
)

# tools/pose_verify_expectations.py의 deg_to_raw와 완전히 같은 식이어야 한다
# (driver_sdk.STS3215Driver.degrees_to_position 기준).
POS_CENTER = 2048
COUNTS_PER_TURN = 4095

# base_link 원점의 바닥 위 높이 — tests/test_floor_grasp_profiles.py와 같은 계약.
BASE_ABOVE_FLOOR_MM = 98.0

SERVO_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def raw_to_deg(raw: float) -> float:
    return (raw - POS_CENTER) / COUNTS_PER_TURN * 360.0


def deg_to_raw(deg: float) -> int:
    return int(round(POS_CENTER + (deg / 360.0) * COUNTS_PER_TURN))


def tip_pose(fk, pose_deg):
    """(전방 x mm, 높이 mm, 수평 기준 pitch deg)."""
    position, rotation = fk.fk_deg(list(pose_deg))
    approach = rotation @ np.array([0.0, 0.0, 1.0])
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, float(approach[2])))))
    return position[0] * 1000.0, position[2] * 1000.0 + BASE_ABOVE_FLOOR_MM, pitch


def solve_height_keep_pitch(fk, base_pose_deg, target_height_mm, target_x_mm=None,
                             servo1_deg=None, iters=200, tol=1e-4):
    """servo1·5는 고정하고 servo2·3·4만 풀어 (x, z, pitch)가 기준 자세와
    같아지게 한다 — pitch를 기준값에 고정하므로 '수평 유지'가 목표식에
    그대로 들어간다.

    servo1_deg를 주면 기준 자세의 servo1 대신 그 값을 고정값으로 쓴다 —
    2026-09-04 실기에서 DROP 자세로 들어갈 때만 servo1이 IDLE(2066raw)에서
    기존 DROP 값(2029raw)으로 살짝(-37raw, -3.25°) 돌아가는 게 눈에 띄어,
    "웬만하면 고정해" 지시에 따라 IDLE의 servo1을 그대로 물려 쓸 수 있게 한
    파라미터다."""
    theta1, theta2, theta3, theta4, theta5 = base_pose_deg
    if servo1_deg is not None:
        theta1 = servo1_deg
    base_x, base_height, base_pitch = tip_pose(fk, base_pose_deg)
    if target_x_mm is None:
        target_x_mm = base_x
    target = np.array([target_x_mm, target_height_mm, base_pitch])

    q = np.radians([theta2, theta3, theta4])

    def residual(q_vec):
        pose = [theta1, math.degrees(q_vec[0]), math.degrees(q_vec[1]),
                math.degrees(q_vec[2]), theta5]
        x, z, pitch = tip_pose(fk, pose)
        return np.array([x, z, pitch]) - target

    d = 1e-5
    for _ in range(iters):
        r = residual(q)
        if np.linalg.norm(r) < tol:
            break
        J = np.zeros((3, 3))
        for i in range(3):
            qp = q.copy()
            qp[i] += d
            J[:, i] = (residual(qp) - r) / d
        try:
            dq = np.linalg.solve(J, -r)
        except np.linalg.LinAlgError:
            dq = np.linalg.lstsq(J, -r, rcond=None)[0]
        dq = np.clip(dq, -0.3, 0.3)
        q = q + dq

    pose_deg = [theta1, math.degrees(q[0]), math.degrees(q[1]), math.degrees(q[2]), theta5]
    achieved_x, achieved_z, achieved_pitch = tip_pose(fk, pose_deg)
    residual_final = np.array([achieved_x, achieved_z, achieved_pitch]) - target
    return pose_deg, residual_final, target, base_x, base_height, base_pitch


def check_joint_limits(pose_deg):
    """TAUGHT_POSITION_LIMITS(raw)와 대조 — 계산이 수렴해도 서보 물리
    한계 밖이면 실기에서 그대로 못 움직인다."""
    report = []
    for servo_id, deg in zip(range(1, 6), pose_deg):
        raw = deg_to_raw(deg)
        lo, hi = TAUGHT_POSITION_LIMITS[servo_id]
        ok = lo <= raw <= hi
        margin = min(raw - lo, hi - raw)
        report.append((servo_id, deg, raw, lo, hi, ok, margin))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--height-mm", type=float, required=True,
                         help="목표 파지/투하 중심 높이(mm). 예: 320. "
                              "'~cm'로 말씀하신 값이라면 물리적으로 대부분 "
                              "불가능합니다(이 팔의 최대 도달은 대략 40cm 안팎) — "
                              "mm로 해석해 계산합니다.")
    parser.add_argument("--x-mm", type=float, default=None,
                         help="목표 전방 거리(mm). 기본값은 기준 자세(DROP_300, "
                              "production 현재값)의 전방 거리를 그대로 유지합니다.")
    parser.add_argument("--base-pose", choices=["drop300"], default="drop300",
                         help="기준 자세 — 지금은 BASKET_DROP_300_RAW(production "
                              "현재값)만 지원합니다.")
    parser.add_argument("--lock-servo1-to-idle", action="store_true",
                         help="servo1을 기준 DROP 자세 값 대신 IDLE_CRADLE_RAW의 "
                              "servo1로 고정합니다 — 2026-09-04 실기 지시: DROP "
                              "진입 시 servo1이 살짝 돌아가는 게 눈에 띄어 아예 안 "
                              "돌게 함. BASKET_DROP_300_RAW 자체가 이미 이 방식으로 "
                              "만들어졌으므로, 그 값을 기준으로 계속 조정한다면 켜 두는 "
                              "쪽이 일관적이다.")
    args = parser.parse_args()

    fk = FKSo101()
    base_pose_raw = BASKET_DROP_300_RAW
    base_pose_deg = tuple(raw_to_deg(r) for r in base_pose_raw)
    servo1_deg = raw_to_deg(IDLE_CRADLE_RAW[0]) if args.lock_servo1_to_idle else None

    pose_deg, residual, target, base_x, base_height, base_pitch = solve_height_keep_pitch(
        fk, base_pose_deg, args.height_mm, args.x_mm, servo1_deg=servo1_deg,
    )
    limits = check_joint_limits(pose_deg)

    pos_ok = abs(residual[0]) < 1.0 and abs(residual[1]) < 1.0
    pitch_ok = abs(residual[2]) < 1.0
    limits_ok = all(row[5] for row in limits)
    verdict = pos_ok and pitch_ok and limits_ok

    print("=== drop 자세 높이 변경 가능성 테스트 (순수 계산 — 하드웨어 미사용) ===\n")
    print(f"기준 자세: BASKET_DROP_300_RAW = {base_pose_raw}")
    print(f"  기준 전방거리 {base_x:.1f}mm, 기준 높이 {base_height:.1f}mm(FK), pitch {base_pitch:.2f}°(수평)\n")
    print(f"목표: 전방거리 {target[0]:.1f}mm(기준 유지) 그대로, "
          f"높이 {args.height_mm:.1f}mm, pitch {target[2]:.2f}°(기준과 동일 = 수평 유지)\n")

    print("[관절별 결과]")
    print(f"{'servo':<14}{'목표(deg)':>12}{'raw':>8}{'허용범위(raw)':>18}{'여유(raw)':>12}{'판정':>8}")
    for (servo_id, deg, raw, lo, hi, ok, margin), name in zip(limits, SERVO_NAMES):
        base_raw = base_pose_raw[servo_id - 1]
        print(f"{servo_id}:{name:<11}{deg:>12.2f}{raw:>8d}{f'[{lo},{hi}]':>18}"
              f"{margin:>12d}{'OK' if ok else 'X':>8}   (기준 대비 Δ{raw - base_raw:+d} raw)")

    print("\n[도달 정확도]")
    print(f"  전방거리 오차 {residual[0]:+.3f}mm, 높이 오차 {residual[1]:+.3f}mm, "
          f"pitch 오차 {residual[2]:+.3f}°")

    print("\n[결론]")
    if verdict:
        print(f"  ✅ 계산상 가능 — 손가락을 계속 수평으로 유지한 채 높이를 "
              f"{base_height:.0f} -> {args.height_mm:.0f}mm로 바꾸는 관절각이 "
              "존재하고, 서보 허용 범위 안입니다.")
    else:
        reasons = []
        if not pos_ok:
            reasons.append("목표 위치(전방거리/높이)에 수렴하지 못함")
        if not pitch_ok:
            reasons.append("그 위치에서는 손가락을 수평으로 유지할 수 없음(pitch 어긋남)")
        if not limits_ok:
            bad = [f"servo{r[0]}" for r in limits if not r[5]]
            reasons.append(f"서보 허용 범위(TAUGHT_POSITION_LIMITS) 초과: {', '.join(bad)}")
        print("  ❌ 계산상 불가능 —", "; ".join(reasons))
        print("     (허용 범위 초과라면 pitch 조건을 약간 풀거나 전방거리를 "
              "줄이면 가능해질 수 있습니다 — --x-mm으로 시도해 보세요.)")

    print("\n⚠️ 이 결과는 관절 도달성만 확인합니다. 실기에서는 케이블 간섭, "
          "차체/바구니와의 충돌, 서보 부하·온도까지 별도로 확인해야 합니다 — "
          "tools/basket_drop_pose_hardware_test.py 같은 실기 테스트가 그 역할입니다.")

    return 0 if verdict else 1


if __name__ == "__main__":
    sys.exit(main())
