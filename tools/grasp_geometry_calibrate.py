#!/usr/bin/env python3
"""GRASP 정렬 판정을 켜는 데 필요한 두 값을 실측한다 (2026-08-26).

`domain/task/grasp_alignment.py`의 판정이 이 둘 없이는 아무것도 못 한다.
지금은 둘 다 None이라 치우친 물체를 전부 Host로 넘기고 있다.

    JAW_LINE_DEPTH_FORWARD_M   턱 선을 **뎁스 카메라 판독값으로** 적은 것
    SERVO1_AXIS_TO_JAW_MM      servo 1 회전축에서 턱 중심까지의 수평 거리

## 왜 턱 선을 뎁스 판독값으로 재는가

이미 아는 값(차체 전면 기준 166mm)을 환산해 쓰면 안 된다. 뎁스 카메라의
전방 거리는 클래스별 K를 **base_link 기준** 줄자로 잡아 만든 값이라, 차체
전면 기준 값과 차체 절반 길이만큼 어긋난다 — 잡으려는 영역의 깊이보다 큰
오프셋이다. 중간 변수를 하나 없애는 쪽이 항상 낫다(바구니 정지 거리를
라이다 판독값으로 직접 잡은 것과 같은 이유).

## 모드 A — 턱 선 (자기검증형)

물체를 "전진 없이 그대로 닫아도 물리는" 자리에 놓고,

    1. 팔이 올라간 상태에서 뎁스 카메라로 전방 거리를 읽는다  <- 후보값
    2. 미세 전진 **없이** 내려가 닫는다
    3. 부하가 올라가면 그 자리가 곧 턱 선이었다는 뜻이다      <- 검증

읽고 나서 실제로 물어 보므로 값이 맞는지 그 자리에서 확인된다.

## 모드 B — servo 1 팔 길이

바닥 파지 자세에서 servo 1을 +각도, -각도로 돌리고 **턱 중심이 바닥에
그리는 두 점 사이 거리**를 사람이 잰다.

    팔 길이 = 두 점 사이 거리 / (2 * tan(각도))

각도를 크게 잡을수록 재는 오차의 영향이 줄지만, 서비스가 15도에서 막는다.

## 실행 전

    ros2 run grippers_arm arm_driver --ros-args -p arm_port:=/dev/soarm
    ros2 run grippers_perception perception_node        (모드 A만)
    ros2 launch peripherals depth_camera.launch.py      (모드 A만)

    python3 grasp_geometry_calibrate.py --mode jaw --label queen
    python3 grasp_geometry_calibrate.py --mode servo1 --profile chess_queen
"""

import argparse
import math
import statistics
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from grippers_interfaces.action import MoveToFloorPose
from grippers_interfaces.srv import GetArmState, ObserveTarget, OffsetBaseYaw, SetGripper
from grippers_arm.floor_grasp_profiles import FLOOR_GRASP_PROFILES

BANNER = "=" * 68
SAMPLES = 7                # 턱 선 관측 표본 수 — 중앙값을 쓴다
SERVO1_PROBE_DEG = 12.0    # 서비스 한계(15도) 안쪽에서 최대한 크게

# 라벨 -> 교시 프로필. baseline_mission._OBJECT_WIDTH_MM와 같은 대응이다.
PROFILE_BY_LABEL = {
    "queen": "chess_queen", "knight": "chess_knight", "rook": "chess_rook",
    "box": "cube", "star": "star_column", "soccer": "soccer_polyhedron",
}


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("grasp_geometry_calibrate")
        self._observe = self.create_client(ObserveTarget, "/perception/observe_target")
        self._gripper = self.create_client(SetGripper, "/arm_driver/set_gripper")
        self._state = self.create_client(GetArmState, "/arm_driver/get_arm_state")
        self._hold = self.create_client(Trigger, "/arm_driver/hold_position")
        self._yaw = self.create_client(OffsetBaseYaw, "/arm_driver/offset_base_yaw")
        self._floor = ActionClient(self, MoveToFloorPose, "/arm_driver/move_to_floor_pose")

    def _call(self, client, request, label, timeout=15.0):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"{label} 서비스 없음")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        result = future.result()
        if result is None:
            raise RuntimeError(f"{label} 응답 없음")
        return result

    def observe(self, label):
        return self._call(self._observe, ObserveTarget.Request(raw_cls=label), "observe_target")

    def hold(self):
        return self._call(self._hold, Trigger.Request(), "hold_position")

    def set_gripper(self, width_mm):
        request = SetGripper.Request()
        request.width_mm = float(width_mm)
        return self._call(self._gripper, request, "set_gripper", timeout=20.0)

    def arm_state(self):
        return self._call(self._state, GetArmState.Request(), "get_arm_state")

    def offset_yaw(self, offset_rad):
        request = OffsetBaseYaw.Request()
        request.offset_rad = float(offset_rad)
        return self._call(self._yaw, request, "offset_base_yaw", timeout=30.0)

    def stage(self, profile, stage, timeout=60.0):
        if not self._floor.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("move_to_floor_pose 액션 서버 없음")
        goal = MoveToFloorPose.Goal()
        goal.profile, goal.stage = profile, stage
        send = self._floor.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=15.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"'{stage}' 거부됨")
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(self, done, timeout_sec=timeout)
        outcome = done.result()
        if outcome is None or not outcome.result.reached:
            raise RuntimeError(f"'{stage}' 도달 실패")


def mode_jaw_line(node, label):
    profile = PROFILE_BY_LABEL[label]
    geometry = FLOOR_GRASP_PROFILES[profile]

    print(BANNER)
    print("모드 A · 턱 선 실측  (JAW_LINE_DEPTH_FORWARD_M)")
    print(BANNER)
    print(f"  대상: {label} -> {profile} (폭 {geometry.object_width_mm}mm)")
    print()
    print("  1) 팔은 올라간 자세(IDLE 또는 CARRY)여야 합니다 — 카메라 시야 확보용")
    print("  2) 물체를 **미세 전진 없이 그대로 닫아도 물릴** 자리에 놓으세요.")
    print("     지난 실기 기준: 차체 전면에서 약 166mm, 정면 중앙")
    input("  준비되면 Enter > ")

    readings = []
    for i in range(SAMPLES):
        response = node.observe(label)
        if response.found and response.metric_ok:
            readings.append((response.forward_m, response.lateral_m))
            print(f"    {i + 1}/{SAMPLES}  전방 {response.forward_m:.4f} m  "
                  f"좌우 {response.lateral_m * 1000:+.1f} mm")
        else:
            print(f"    {i + 1}/{SAMPLES}  검출 실패 "
                  f"(found={response.found} metric_ok={response.metric_ok})")

    if len(readings) < 3:
        print("\n  ⛔ 유효 관측이 3개 미만입니다 — 조명/거리/클래스를 확인하세요.")
        return None

    forward = statistics.median(r[0] for r in readings)
    lateral = statistics.median(r[1] for r in readings)
    spread = max(r[0] for r in readings) - min(r[0] for r in readings)
    print()
    print(f"  전방 중앙값 = {forward:.4f} m   (표본 폭 {spread * 1000:.1f} mm)")
    print(f"  좌우 중앙값 = {lateral * 1000:+.1f} mm")
    print()
    print("  이제 **전진 없이** 내려가 닫아서 이 값이 맞는지 확인합니다.")
    print("  ⚠️ 물체를 건드리지 마세요.")
    input("  진행하려면 Enter (Ctrl-C로 중단) > ")

    node.hold()
    node.set_gripper(geometry.preopen_width_mm)
    node.stage(profile, "safe")
    node.stage(profile, "grasp")
    node.set_gripper(geometry.close_width_mm)
    state = node.arm_state()
    load = float(state.load_ratio[5])
    print(f"    파지 부하 = {load:.4f}")

    node.stage(profile, "safe")
    node.stage(profile, "carry")

    verdict = "물렸습니다 — 이 값이 턱 선입니다" if load >= 0.05 else \
        "⚠️ 안 물렸습니다 — 물체가 턱 선이 아닌 자리에 있었습니다"
    print()
    print(BANNER)
    print(f"  {verdict}")
    print(f"  JAW_LINE_DEPTH_FORWARD_M = {forward:.4f}")
    if abs(lateral) > 0.010:
        print(f"  ⚠️ 좌우 {lateral * 1000:+.1f}mm 치우쳐 있었습니다 — "
              "DEPTH_LATERAL_TO_JAW_CENTER_M 후보이거나 배치가 어긋난 것입니다")
    print(BANNER)
    return forward


def mode_servo1(node, profile):
    print(BANNER)
    print("모드 B · servo 1 팔 길이 실측  (SERVO1_AXIS_TO_JAW_MM)")
    print(BANNER)
    print(f"  자세: {profile} 바닥 파지 자세로 내려갑니다.")
    print("  ⚠️ 그리퍼 앞을 비워 두세요. 물체가 있으면 치입니다.")
    input("  준비되면 Enter > ")

    node.hold()
    node.stage(profile, "safe")
    node.stage(profile, "grasp")

    probe = math.radians(SERVO1_PROBE_DEG)
    marks = []
    for direction, name in ((+1, "왼쪽"), (-1, "오른쪽")):
        response = node.offset_yaw(direction * probe)
        if not response.ok:
            print(f"  ⛔ servo 1 거부: {response.message}")
            node.offset_yaw(0.0)
            return None
        print(f"    {name} {SERVO1_PROBE_DEG:.0f}도 -> servo 1 raw {response.position_raw}")
        input(f"    턱 중심이 바닥에 오는 지점을 표시하고 Enter ({name} 표시) > ")
        marks.append(name)
        # 반대쪽으로 가려면 두 배를 돌려야 하므로 먼저 중앙으로 되돌린다.
        node.offset_yaw(-direction * probe)

    print()
    raw = input(f"  두 표시 사이 거리(mm)를 입력하세요 > ").strip()
    node.stage(profile, "safe")
    node.stage(profile, "carry")
    try:
        span_mm = float(raw)
    except ValueError:
        print("  ⛔ 수치가 아닙니다 — 다시 실행하세요.")
        return None

    reach = span_mm / (2.0 * math.tan(probe))
    print()
    print(BANNER)
    print(f"  두 점 사이 {span_mm:.1f} mm, 각도 ±{SERVO1_PROBE_DEG:.0f}도")
    print(f"  SERVO1_AXIS_TO_JAW_MM = {reach:.1f}")
    print(f"  (1도당 좌우 {reach * math.tan(math.radians(1.0)):.1f} mm)")
    print(BANNER)
    return reach


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--mode", choices=("jaw", "servo1"), required=True)
    parser.add_argument("--label", default="queen", choices=sorted(PROFILE_BY_LABEL))
    parser.add_argument("--profile", default="chess_queen")
    args = parser.parse_args()

    rclpy.init()
    node = CalibrationNode()
    try:
        if args.mode == "jaw":
            mode_jaw_line(node, args.label)
        else:
            mode_servo1(node, args.profile)
        return 0
    except KeyboardInterrupt:
        print("\n중단합니다.")
        return 130
    except Exception as exc:  # noqa: BLE001 -- 실기 도구
        print(f"\n오류: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
