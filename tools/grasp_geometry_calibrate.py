#!/usr/bin/env python3
"""GRASP 정렬 판정을 켜는 데 필요한 두 값을 실측한다 (2026-08-26).

`domain/task/grasp_alignment.py`의 판정이 이 둘 없이는 아무것도 못 한다.
지금은 둘 다 None이라 치우친 물체를 전부 Host로 넘기고 있다.

    JAW_LINE_DEPTH_FORWARD_M   턱 선을 **클래스마다** 뎁스 판독값으로 적은 것
    SERVO1_AXIS_TO_JAW_MM      servo 1 회전축에서 턱 중심까지의 수평 거리

## 왜 클래스마다 따로 재는가

클래스별 거리 보정 K의 정확도가 제각각이다 — rook만 3점 최소제곱이고
나머지는 먼 거리 1점이라, 파지 거리대에서 배율 오차가 크다. 2026-08-25에
여섯 물체를 **같은 물리 18cm**에 놓았더니 queen 14.4 / rook 18.3 /
knight 18.7 / soccer 25.6cm로 읽혔다.

턱 선을 하나로 공용하면 그 오차가 전진 거리에 그대로 실린다 — 실제 24mm를
가야 하는 상황에서 queen은 -17mm, soccer는 101mm가 나온다. 같은 클래스로 잰
턱 선을 빼면 오차가 대부분 상쇄돼 한 자릿수 mm로 줄어든다.

**쓸 클래스마다 한 번씩 돌려야 한다.**

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

## 모드 C — 클래스별 거리 보정 K

    K = 거리 * (sqrt(bbox 면적) - 2.5)

알려진 거리 여러 곳에 놓고 읽어 최소제곱으로 K를 낸다. 거리의 기준점은
**아무 데나 잡아도 되지만 클래스 안에서 일관돼야 한다** — GRASP는 언제나
(관측 - 그 클래스의 턱 선)만 쓰므로 기준점이 상쇄된다. 재기 쉬운 차체 전면을
권한다.

## 실행 전 (모드 A·C는 넷 다 필요)

    ros2 run grippers_arm arm_driver --ros-args -p arm_port:=/dev/soarm
    ros2 launch peripherals depth_camera.launch.py
    ros2 run grippers_perception depth_cam_rotate_node
    ros2 run grippers_perception perception_node

⚠️ **depth_cam_rotate_node를 빼먹기 쉽다.** perception_node는 회전 보정된
스트림만 구독하므로, 이게 없으면 카메라가 돌고 있어도 YOLO에 프레임이 한
장도 안 간다 — 증상은 "그냥 검출 실패"라 원인이 안 드러난다(2026-08-26
실기에서 실제로 겪었다). 그래서 이 도구는 시작할 때 프레임이 실제로
흐르는지 먼저 확인한다.

    python3 grasp_geometry_calibrate.py --mode jaw --label queen
    python3 grasp_geometry_calibrate.py --mode servo1 --profile chess_queen
    python3 grasp_geometry_calibrate.py --mode seat --label rook
    python3 grasp_geometry_calibrate.py --mode k --label rook
    python3 grasp_geometry_calibrate.py --mode confirm --label rook
    python3 grasp_geometry_calibrate.py --mode load --label rook
    python3 grasp_geometry_calibrate.py --mode scale --label rook
"""

import argparse
import math
import statistics
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from grippers_interfaces.action import MoveToFloorPose
from grippers_interfaces.srv import GetArmState, ObserveTarget, OffsetBaseYaw, SetGripper
from grippers_arm.floor_grasp_profiles import FLOOR_GRASP_PROFILES

BANNER = "=" * 68
SAMPLES = 7                # (구) 관측 표본 수 — 캐시를 안 비우는 곳에서만 쓴다
# 턱 선 표본 수. 캐시를 비워 가며 뜨므로 하나에 약 3.3초씩 든다 — 표본을
# 늘리는 값보다 사람이 기다리는 값이 더 빨리 커져서 4로 잡았다.
JAW_SAMPLES = 4

# 들어올린 **뒤에도** 부하가 이만큼 더 떨어지면 미끄러지는 중으로 본다.
# 닫는 순간과 비교하지 않는다 — 위 판정부 주석 참고.
LOAD_SLIP_DROP = 0.010
# 턱 목의 깊이(실측 2026-08-26). 턱 끝에 걸렸을 때 얼마나 더 가까이
# 놓아야 하는지 안내하는 데 쓴다.
JAW_THROAT_DEPTH_M = 0.023
# Ros2Perception.STILL_THERE_H_RATIO와 같은 값이어야 한다 — 이 도구가
# 실제 판정을 재현하는 것이 목적이므로 다르면 거짓 안심/거짓 경고가 된다.
STILL_THERE_H_RATIO = 0.8
# perception_node.OBSERVE_CACHE_SEC와 같은 값이어야 한다 — 게이트 시험이
# 서로 독립적인 관측을 쓰려면 표본 사이에 이만큼은 기다려야 한다.
OBSERVE_CACHE_SEC = 3.0
# 게이트 시험 표본 수. 하나에 캐시 대기 + 수집이 붙어 약 5초씩 든다.
GATE_SAMPLES = 4

# 척도 확인 표본 수. 캐시를 비워 가며 뜨므로 하나에 약 5초씩 든다.
SCALE_SAMPLES = 3
# 이 안이면 척도가 유지된 것으로 본다. 줄자 200mm에 1mm 오차가 0.5%이므로
# 사람이 재는 정밀도를 넘지 않는 선에서 잡는다.
SCALE_TOLERANCE = 0.05
# 밀 수 있는 최대 거리. 거리 게이트가 약 0.3m 너머를 거부하므로, 파지
# 거리대(0.18m)에서 출발하면 이 정도가 한계다 — 2026-08-26 실측에서
# 0.2889m는 잡히고 약 0.39m는 통째로 미검출이었다.
#
# ⚠️ 이 상한이 이 검사의 정밀도를 묶는다. 판독 잡음이 차이에 약 2mm 실리므로
# 100mm를 밀면 척도 불확실도가 약 2%다. 즉 이 검사는 **5% 넘는 어긋남만
# 잡아낼 수 있고**, 그보다 작은 차이는 있는지 없는지 못 가린다.
SCALE_MAX_PUSH_MM = 120.0
# perception_node.OBSERVE_MIN_BOTTOM_Y_PX와 같은 값이어야 한다.
OBSERVE_MIN_BOTTOM_Y_PX = 290.0
# perception_node.CLASS_DISTANCE_CALIBRATION_SQRT_PX_M의 현재 값.
# 척도가 어긋났을 때 보정값을 바로 낼 수 있게 여기 적어 둔다 —
# 도메인 계층이 ROS 패키지를 import하지 않는 것과 같은 이유로 복사한다.
# baseline_constants.JAW_LINE_DEPTH_FORWARD_M의 사본. 척도 어긋남이 전진량에
# 얼마나 실리는지 그 자리에서 보여주는 데만 쓴다.
JAW_LINE_FOR_HINT = {"rook": 0.1757, "knight": 0.1881, "queen": 0.1421}
CURRENT_K = {
    "knight": 35.9307, "queen": 28.3382, "rook": 34.8340,
    "soccer": 18.9592, "box": None, "star": None,
}
SERVO1_PROBE_DEG = 12.0    # 서비스 한계(15도) 안쪽에서 최대한 크게

# 빈 부하를 재는 자세 순회. 순서는 arm_driver_node의 전이 제약을 따른다 —
# grasp는 safe에서만, midpoint는 grasp에서만, drop은 idle/safe/carry에서만
# 시작할 수 있다.
LOAD_STAGE_TOUR = ("safe", "grasp", "midpoint", "safe", "carry", "drop", "carry")
LOAD_SAMPLES = 5           # 한 버스트의 표본 수
LOAD_SAMPLE_GAP_S = 0.3

# 여섯 프로필이 실제로 쓰는 닫힘 목표 폭 전부
# (물체 폭 - GRIPPER_SQUEEZE_MM, 하한 GRIPPER_GRASP_MIN_MM=7.0으로 clamp).
LOAD_WIDTH_SWEEP_MM = (7.0, 9.5, 25.0, 30.0, 31.0)

# 같은 조건을 몇 번 되풀이해 떠돌이 폭을 잴 것인가.
LOAD_REPEATS = 8

# 지금까지 관측된 빈손 부하의 최대. 한 회차가 최악을 재현하지 못할 수 있으므로
# 권장값은 이 값과 이번 회차 중 큰 쪽으로 낸다.
HISTORICAL_EMPTY_MAX = 0.0430  # 2026-08-25

# 지금 코드에 박혀 있는 값. 실측이 이걸 넘는지 보는 것이 이 모드의 목적이다.
LOAD_THRESHOLD_NOW = 0.04
# 2026-08-26 실측된 파지 부하의 최소 — queen/knight CARRY 0.0626.
GRIPPED_LOAD_MIN = 0.0626
# 판독 양자. 관측값이 전부 이 배수다(0.0235=6, 0.0391=10, 0.0430=11).
# 여유를 양자 단위로 말해야 "한 칸 차이"인지 "여러 칸 차이"인지 드러난다.
LOAD_QUANTUM = 1.0 / 256.0

# perception_node가 YOLO를 돌리는 스트림. depth_cam_rotate_node가 낸다.
ROTATED_RGB_TOPIC = "/depth_cam/rgb/image_rotated"

# perception_node의 BBOX_PADDING_PX와 같은 값이어야 한다 — 검출기 성질에서
# 온 여유분이라 클래스와 무관하다.
BBOX_PADDING_PX = 2.5

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
        self._frames = 0
        self.create_subscription(Image, ROTATED_RGB_TOPIC, self._on_frame,
                                 qos_profile_sensor_data)

    def _on_frame(self, _msg):
        self._frames += 1

    def require_camera(self):
        """YOLO에 프레임이 실제로 가고 있는지 먼저 확인한다.

        이걸 안 보면 depth_cam_rotate_node가 빠졌을 때 증상이 "검출 실패"로만
        나타나 원인이 안 드러난다 — 2026-08-26 실기에서 실제로 겪었다."""
        for _ in range(60):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._frames:
                return
        raise RuntimeError(
            f"{ROTATED_RGB_TOPIC}에 프레임이 없습니다 — "
            "depth_cam_rotate_node가 떠 있는지 확인하세요\n"
            "    ros2 run grippers_perception depth_cam_rotate_node")

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


def mode_gate(node, label):
    """새 오검출 게이트가 **진짜 물체를 죽이지 않는지** 확인한다.

    2026-08-26에 observe_target에 세 겹의 게이트를 걸었다 — 신뢰도 0.70,
    화면 아래쪽(bbox 아래끝 >= 290px), 5프레임 중 3회 합의. 배경 오검출
    (노트북을 rook 0.60으로 잡던 것)을 막기 위해서다.

    막는 쪽은 실기로 확인됐지만 **통과시켜야 할 쪽은 아직 아니다.** 게이트가
    너무 빡빡하면 파지 거리의 진짜 물체가 통째로 사라지고, 그러면 GRASP가
    영영 시작되지 않는다. 여기서는 물체를 실제 파지 자리에 놓고 세 게이트를
    각각 얼마나 여유 있게 통과하는지 본다."""
    print(BANNER)
    print(f"모드 F · '{label}' 오검출 게이트 통과 여부")
    print(BANNER)
    print("  팔은 올라간 자세로 두고, 물체를 **실제 파지 자리**에 놓으세요.")
    print("  (차체 전면에서 약 166mm, 정면 중앙)")
    input("  준비되면 Enter > ")

    # ⚠️ 표본 사이에 캐시가 만료되도록 기다린다. 안 그러면 노드가 같은
    # 표본을 계속 돌려줘서 "7번 다 통과"가 사실은 한 번의 관측이 된다 —
    # 2026-08-26에 실제로 그렇게 나왔다.
    results = []
    for i in range(GATE_SAMPLES):
        if i:
            time.sleep(OBSERVE_CACHE_SEC + 0.3)
        response = node.observe(label)
        results.append(response)
        if response.found:
            print(f"    {i + 1}/{GATE_SAMPLES}  통과  h={response.h:.1f}px "
                  f"w={response.w:.1f}px  전방 {response.forward_m:.3f}m")
        else:
            print(f"    {i + 1}/{GATE_SAMPLES}  **게이트 탈락 또는 미검출**")

    hits = sum(1 for r in results if r.found)
    print()
    print(BANNER)
    if hits == GATE_SAMPLES:
        print(f"  ✅ {hits}/{GATE_SAMPLES} 전부 통과 — 게이트가 진짜 물체를 막지 않습니다.")
        print("     (표본마다 캐시를 비웠으므로 서로 독립적인 관측입니다)")
    elif hits == 0:
        print(f"  ⛔ {GATE_SAMPLES}회 모두 탈락 — **게이트가 너무 빡빡합니다.**")
        print("     perception_node 로그의 '[observe] 게이트 탈락' 줄을 보세요.")
        print("     신뢰도 때문인지 화면 위치 때문인지 거기 찍힙니다.")
        print("     ros2 run ... 로그: docker exec ... tail /tmp/p.log")
    else:
        print(f"  ⚠️ {hits}/{GATE_SAMPLES}만 통과 — 경계에 걸쳐 있습니다.")
        print("     합의(5중 3)는 넘겼더라도 여유가 없습니다. 로그를 보고")
        print("     OBSERVE_CONF_THRESHOLD 또는 OBSERVE_MIN_BOTTOM_Y_PX를 조정하세요.")
    print(BANNER)
    return hits


def mode_confirm(node, label):
    """"파지 후 CARRY에서 물체가 보이면 실패" 규칙이 실제로 성립하는지 본다.

    사용자 지시(2026-08-26): 턱 끝에 꽉 물려도 안 떨어질 수 있으므로 부하만
    믿을 수 없고, CARRY 자세에서 뎁스 카메라에 목표가 보이면 파지 실패로
    처리해야 한다. 그 규칙은 이미 구현돼 있다(Perception.confirm_grasp).

    **여기서 확인하는 것은 그 규칙의 반대 방향이다.** 물체를 제대로 들고
    CARRY로 갔을 때, 그리퍼에 물린 그 물체가 카메라에 잡히면 안 된다 —
    잡히면 성공한 파지가 매번 실패로 뒤집힌다. 팔이 접힌 자세에서 물체가
    시야 밖에 있는지는 계산으로 알 수 없고 실기로만 확인된다."""
    profile = PROFILE_BY_LABEL[label]
    geometry = FLOOR_GRASP_PROFILES[profile]

    print(BANNER)
    print(f"모드 E · '{label}' CARRY에서 파지물이 보이는지 확인")
    print(BANNER)
    print("  팔이 내려가 턱을 벌립니다. 물체를 넣어 주세요.")
    input("  준비되면 Enter > ")

    node.hold()
    node.set_gripper(geometry.preopen_width_mm)
    node.stage(profile, "safe")
    node.stage(profile, "grasp")
    input("  턱 사이에 물체를 넣고 Enter > ")

    before = node.observe(label)
    print(f"    [기준] 바닥의 물체: found={before.found} h={before.h:.1f}px")

    node.set_gripper(geometry.close_width_mm)
    node.stage(profile, "midpoint")
    node.stage(profile, "safe")
    node.stage(profile, "carry")
    load = float(node.arm_state().load_ratio[5])
    print(f"    CARRY 부하 = {load:.4f}")
    if load < 0.05:
        print("    ⛔ 들고 있지 않습니다 — 이 시험은 물체를 든 상태여야 합니다.")
        return None

    # confirm_grasp의 **실제 판정**을 그대로 재현한다. found만 세면 배경
    # 오검출까지 경고로 잡혀 과하게 겁을 준다 — 2026-08-26에 실제로 그랬다.
    threshold = before.h * STILL_THERE_H_RATIO
    print(f"    (기준 h={before.h:.1f}px -> '그대로 있다' 임계 {threshold:.1f}px)")
    seen, would_fail, heights = 0, 0, []
    for i in range(SAMPLES):
        response = node.observe(label)
        verdict = "검출 없음 -> 성공"
        if response.found:
            seen += 1
            heights.append(response.h)
            still = response.h >= threshold
            would_fail += int(still)
            verdict = ("그대로 있다 -> 파지 실패" if still
                       else "더 작다 = 다른 개체 -> 성공")
        print(f"    {i + 1}/{SAMPLES}  found={response.found}  "
              f"h={response.h:.1f}px  {verdict}")

    print()
    print(BANNER)
    if would_fail == 0:
        print(f"  ✅ confirm_grasp는 {SAMPLES}프레임 모두 **파지 성공**으로 판정합니다.")
        print("     규칙이 성립합니다.")
        if seen:
            print(f"  ⚠️ 다만 {seen}/{SAMPLES}회 무언가 검출되긴 했습니다 "
                  f"(h 최대 {max(heights):.0f}px, 임계 {threshold:.0f}px).")
            print("     물고 있는 물체가 아니라 **배경 오검출**입니다. 지금은 크기가")
            print("     작아 걸러지지만, 더 큰 오검출이 나오면 뒤집힙니다.")
    else:
        print(f"  ⛔ {would_fail}/{SAMPLES}회가 **파지 실패**로 판정됩니다.")
        print("     성공한 파지가 뒤집힙니다 — CARRY 자세를 더 접거나")
        print("     오검출을 걸러내야 합니다.")
    print(BANNER)
    return would_fail


def mode_seat(node, label):
    """턱으로 직접 물려 **좌우 영점**을 잰다.

    ⚠️ 2026-08-26 실기로 이 모드의 전제 절반이 틀렸다는 것이 드러났다.
    평행 턱은 곧게 닫히므로 **깊이 방향으로는 물체를 끌어당기지 않는다** —
    손가락 끝에 있으면 끝에서 그대로 물린다(rook에서 실제로 그랬다).
    그래서 여기서 나오는 전방 거리는 턱 선이 **아니라** 조작자가 밀어 넣은
    자리일 뿐이다. 턱 선은 --mode jaw(들어올림 검사 포함)로 재야 한다.

    좌우는 다르다. 두 손가락이 대칭으로 닫히므로 물체는 좌우로는 반드시
    턱 중심에 앉는다. 그래서 이 모드는 **좌우 영점에만** 쓴다.

    모드 A는 조작자가 눈으로 놓은 자리를 읽는다. 그게 정말 턱 중앙인지는
    알 수 없다 — 턱이 168mm까지 벌어져 있어 15mm쯤 치우쳐도 평행 턱이
    알아서 끌어당겨 물기 때문에, "물렸다"는 사실이 "중앙이었다"의 증거가
    못 된다(2026-08-26 실기에서 두 번 다 +14~16mm가 나왔다).

    여기서는 **기계가 직접 앉힌다.** 턱을 닫으면 물체가 턱 중심선과 목
    안쪽으로 끌려 들어가고, 그 상태로 다시 벌린 뒤 팔을 들어 읽으면
    조작자의 눈이 판단에서 빠진다."""
    profile = PROFILE_BY_LABEL[label]
    geometry = FLOOR_GRASP_PROFILES[profile]

    print(BANNER)
    print(f"모드 D · '{label}' 좌우 영점  (턱이 좌우로는 중앙에 앉힌다)")
    print(BANNER)
    print("  팔이 바닥 파지 자세로 내려가 턱을 벌립니다.")
    input("  준비되면 Enter > ")

    node.hold()
    node.set_gripper(geometry.preopen_width_mm)
    node.stage(profile, "safe")
    node.stage(profile, "grasp")

    print()
    print("  벌어진 턱 사이로 물체를 밀어 넣으세요 — 대충 넣으셔도 됩니다.")
    print("  닫으면 턱이 알아서 중앙과 목 안쪽으로 끌어당깁니다.")
    input("  넣었으면 Enter > ")

    node.set_gripper(geometry.close_width_mm)
    state = node.arm_state()
    load = float(state.load_ratio[5])
    print(f"    물린 부하 = {load:.4f}")
    if load < 0.05:
        print("    ⛔ 안 물렸습니다 — 다시 넣고 실행하세요.")
        node.set_gripper(geometry.preopen_width_mm)
        node.stage(profile, "safe")
        node.stage(profile, "carry")
        return None

    print("  이제 놓고 팔을 들어 그 자리를 읽습니다. 물체를 건드리지 마세요.")
    node.set_gripper(geometry.preopen_width_mm)
    node.stage(profile, "safe")
    node.stage(profile, "carry")

    readings = []
    for i in range(SAMPLES):
        response = node.observe(label)
        if response.found and response.metric_ok:
            readings.append((response.forward_m, response.lateral_m))
            print(f"    {i + 1}/{SAMPLES}  전방 {response.forward_m:.4f} m  "
                  f"좌우 {response.lateral_m * 1000:+.1f} mm")

    if len(readings) < 3:
        print("\n  ⛔ 유효 관측이 3개 미만입니다.")
        return None

    forward = statistics.median(r[0] for r in readings)
    lateral = statistics.median(r[1] for r in readings)
    print()
    print(BANNER)
    print(f'  DEPTH_LATERAL_TO_JAW_CENTER_M["{label}"] = {lateral:.4f}')
    print()
    print("  물체를 턱이 좌우 중앙에 앉혔으므로, 이 값이 곧 카메라 영점입니다.")
    print(f"  ⚠️ 전방 {forward:.4f}는 **턱 선이 아닙니다** — 평행 턱은 깊이")
    print("     방향으로 끌어당기지 않아, 밀어 넣은 자리가 그대로 나옵니다.")
    print("     턱 선은 --mode jaw로 재세요(들어올림 검사가 붙어 있습니다).")
    print(BANNER)
    return forward, lateral


def mode_k(node, label):
    """클래스별 거리 보정 K를 여러 거리에서 최소제곱으로 낸다."""
    print(BANNER)
    print(f"모드 C · '{label}' 거리 보정 K 실측")
    print(BANNER)
    print("  알려진 거리 여러 곳에 물체를 놓고 읽습니다.")
    print("  거리 기준점은 재기 쉬운 곳(차체 전면 권장)으로 잡되,")
    print("  **한 클래스 안에서는 끝까지 같은 기준**을 쓰세요.")
    print("  파지 거리대를 포함해 2~4점을 권합니다 (예: 0.15 / 0.25 / 0.40 m).")
    print("  빈 줄을 입력하면 계산으로 넘어갑니다.")
    print()

    samples = []
    while True:
        raw = input(f"  거리(m) [{len(samples)}점 수집됨] > ").strip()
        if not raw:
            break
        try:
            distance_m = float(raw)
        except ValueError:
            print("    수치가 아닙니다.")
            continue

        areas = []
        for _ in range(SAMPLES):
            response = node.observe(label)
            if response.found:
                areas.append(response.w * response.h)
        if len(areas) < 3:
            print(f"    ⛔ 유효 검출 {len(areas)}회 — 다시 놓고 시도하세요.")
            continue
        area = statistics.median(areas)
        effective = math.sqrt(area) - BBOX_PADDING_PX
        print(f"    면적 중앙값 {area:.0f} px²  ->  sqrt-pad = {effective:.2f} px"
              f"  ->  K = {distance_m * effective:.4f}")
        samples.append((distance_m, effective))

    if not samples:
        print("\n  수집된 점이 없습니다.")
        return None

    # d ~= K / e 를 K에 대해 최소제곱: K = sum(d/e) / sum(1/e^2)
    numerator = sum(d / e for d, e in samples if e > 0)
    denominator = sum(1.0 / (e * e) for _d, e in samples if e > 0)
    k = numerator / denominator

    print()
    print(BANNER)
    print(f'  "{label}": {k:.4f},')
    print()
    print("  적합도 확인 — 각 점에서 이 K가 되돌려주는 거리:")
    for d, e in samples:
        print(f"    실제 {d:.3f} m  ->  추정 {k / e:.3f} m  "
              f"(오차 {(k / e - d) * 1000:+.0f} mm)")
    print()
    print("  perception_node.py의 CLASS_DISTANCE_CALIBRATION_SQRT_PX_M에 넣으세요.")
    print("  ⚠️ K를 바꾸면 그 클래스의 턱 선도 다시 재야 합니다 — 둘은 같은")
    print("     척도 위에 있어야 (관측 - 턱 선)이 의미를 갖습니다.")
    print(BANNER)
    return k


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

    # ⚠️ 표본 사이에 캐시가 만료되도록 기다린다. 안 그러면 노드가 같은 표본을
    # 계속 돌려줘 "7개 중앙값"이 사실은 관측 하나가 되고, 산포는 언제나
    # 0.0000으로 찍혀 **정밀해 보이는 허상**이 된다. 2026-08-26에 --mode gate
    # 에서 같은 결함을 잡았는데 여기는 놓쳤다.
    readings = []
    for i in range(JAW_SAMPLES):
        if i:
            time.sleep(OBSERVE_CACHE_SEC + 0.3)
        response = node.observe(label)
        if response.found and response.metric_ok:
            readings.append((response.forward_m, response.lateral_m))
            print(f"    {i + 1}/{JAW_SAMPLES}  전방 {response.forward_m:.4f} m  "
                  f"좌우 {response.lateral_m * 1000:+.1f} mm")
        else:
            print(f"    {i + 1}/{JAW_SAMPLES}  검출 실패 "
                  f"(found={response.found} metric_ok={response.metric_ok})")

    if len(readings) < 3:
        print("\n  ⛔ 유효 관측이 3개 미만입니다 — 조명/거리/클래스를 확인하세요.")
        return None

    forward = statistics.median(r[0] for r in readings)
    lateral = statistics.median(r[1] for r in readings)
    spread = max(r[0] for r in readings) - min(r[0] for r in readings)
    print()
    print(f"  전방 중앙값 = {forward:.4f} m   (표본 폭 {spread * 1000:.1f} mm"
          f", 서로 독립적인 관측 {len(readings)}개)")
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

    # 닫힌 직후의 부하만으로는 **턱 끝 파지를 못 가른다.** 2026-08-26 실측:
    # 턱 끝(들다가 미끄러짐) 0.0821 > 제대로 물림 0.0782로 실패한 쪽이 오히려
    # 높았다. 그래서 실제로 들어올려 부하가 유지되는지 본다 — 조작자의
    # 눈대중을 판정에서 빼는 것이 이 검사의 목적이다.
    closed = float(node.arm_state().load_ratio[5])
    node.stage(profile, "midpoint")
    lifted = float(node.arm_state().load_ratio[5])
    node.stage(profile, "safe")
    node.stage(profile, "carry")
    carried = float(node.arm_state().load_ratio[5])
    print(f"    부하  닫음 {closed:.4f}  ->  들어올림 {lifted:.4f}  ->  CARRY {carried:.4f}")

    # 순서가 중요하다. "애초에 못 물었다"와 "물었다가 미끄러졌다"는 원인도
    # 대처도 다르다 — 2026-08-26에 queen이 미끄러졌는데 도구가 "턱 밖"이라고
    # 잘못 말했다.
    # 판정 기준은 "닫음 대비 낙폭"이 아니라 **들어올린 뒤에도 계속
    # 흘러내리는가**다. 2026-08-26 실측 세 건이 그걸 말한다:
    #
    #   knight 성공  0.0782 -> 0.0782 -> 0.0782   들기->CARRY  0.0000
    #   queen  성공  0.0821 -> 0.0626 -> 0.0626   들기->CARRY  0.0000
    #   queen  실패  0.0821 -> 0.0391 -> 0.0274   들기->CARRY -0.0117
    #
    # 닫음 대비 낙폭은 성공(-0.0195)과 실패(-0.0547) 둘 다 크다 — 닫는
    # 순간의 부하에는 눌러 들어가는 동적 성분이 섞여 있어 들어올리면
    # 어차피 내려앉는다. 그걸 실패로 보면 멀쩡한 파지를 버린다.
    gripped = closed >= 0.05
    held = carried >= 0.05
    weakened = held and carried < lifted - LOAD_SLIP_DROP

    print()
    print(BANNER)
    if not gripped:
        print("  ⛔ 애초에 물지 못했습니다 — 물체가 턱 사이에 없었습니다.")
        print("     좌우 정렬과 배치를 먼저 확인하세요.")
        return None
    if not held:
        print("  ⚠️ 물었다가 **들어올리며 놓쳤습니다** — 턱 끝 파지입니다.")
        print(f"     이 값({forward:.4f})은 턱 선이 아닙니다.")
    elif weakened:
        print("  ⚠️ 들어올린 뒤에도 부하가 계속 흘러내립니다 — 미끄러지는 중입니다.")
        print(f"     이 값({forward:.4f})은 턱 선으로 쓰기에 위험합니다.")
    if not held or weakened:
        print(f"     물체를 **자로 재서 약 {JAW_THROAT_DEPTH_M * 1000:.0f}mm 앞으로**"
              " 옮기고 다시 하세요.")
        print("     ⚠️ 다음 판독값이 그만큼 줄지는 않습니다 — 클래스마다 거리")
        print("        배율이 달라서, 옮긴 물리 거리와 판독 변화량이 다릅니다.")
        return None

    print("  제대로 물렸고 들어올려도 유지됐습니다 — 이 값이 턱 선입니다.")
    print(f'  JAW_LINE_DEPTH_FORWARD_M["{label}"] = {forward:.4f}')
    print(f'  (참고) 이때 좌우 판독 = {lateral:.4f} — 좌우 영점은 --mode seat로 재세요')
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


def mode_load(node, label):
    """빈 그리퍼 부하의 **띠**를 재서 LOAD_THRESHOLD를 정한다.

    ## 원인을 찾으려 하지 말 것 — 이미 두 번 배제됐다

    이 값이 0.0235~0.0430으로 흔들린다는 것은 2026-08-25부터 알려져 있었고,
    2026-08-26에 원인을 두 번 찾으려다 두 번 다 빗나갔다.

        자세 가설 — 7개 자세 x 2개 폭을 돌았더니 편차 0.0000. 아니다.
                    그리퍼는 손목 끝단이라 팔의 중력 성분을 안 탄다.
        폭   가설 — 여섯 프로필의 닫힘 폭을 훑었으나 단조성이 없었고,
                    **같은 9.5mm가 한 회차엔 0.0391, 다음 회차엔 0.0313**
                    이었다. 아니다.

    실제 거동은 이렇다. **한 버스트 안에서는 완벽히 고정, 버스트 사이에서는
    떠돈다.** 1.5초 동안 5표본을 떠도 편차가 0.0000인데, 자세를 옮기거나
    그리퍼를 다시 명령하면 한두 양자씩 옮겨 앉는다. servo 6이 마지막 정지
    토크를 래치해 두고 다음 명령에서 다시 잡는 것으로 보인다.

    ⚠️ 그래서 **한 변수만 바꿔 가며 재면 떠돌이가 그 변수 탓으로 보인다.**
    2026-08-26에 자세만 바꿔 재고 "자세도 영향이 있다"는 틀린 경고를 냈다.
    이 도구가 지금 하는 일은 원인 규명이 아니라 **띠의 폭을 재는 것**이다.

    ## 원인을 몰라도 임계는 정할 수 있다

        빈손  0.0235 ~ 0.0430   (관측 전체)
        파지  0.0626 ~ 0.0821   (rook/knight/queen)

    두 띠 사이가 5양자다. 떠돌이의 폭보다 **사이 간격**이 넓으면 임계 하나로
    가를 수 있다. 여기서 확인할 것은 딱 그것뿐이다 — 이번 회차의 빈손 띠가
    파지 하한 아래에 통째로 들어오는가.
    """
    profile = PROFILE_BY_LABEL[label]
    geometry = FLOOR_GRASP_PROFILES[profile]

    print(BANNER)
    print("모드 L · 빈 그리퍼 부하 띠 실측  (LOAD_THRESHOLD)")
    print(BANNER)
    print("  ⛔ **그리퍼를 반드시 비우고, 앞도 치워 주세요.**")
    print("     물체가 물려 있으면 이 측정 전체가 무의미해집니다.")
    input("  준비되면 Enter > ")

    node.hold()
    observed = []

    def burst():
        """한 버스트. 이 안에서는 값이 안 움직이므로 대푯값 하나만 남긴다."""
        values = []
        for _ in range(LOAD_SAMPLES):
            time.sleep(LOAD_SAMPLE_GAP_S)
            values.append(float(node.arm_state().load_ratio[5]))
        observed.extend(values)
        return max(values), max(values) - min(values)

    # ── 1부 · 닫힘 폭 훑기 ───────────────────────────────────────────────
    # 폭이 원인은 아니지만, 실제로 쓰이는 폭을 전부 지나면서 표본을 모으는
    # 데는 의미가 있다 — 띠의 폭을 재려면 조건을 다양하게 밟아야 한다.
    print()
    print("  ── 1부 · 닫힘 목표 폭별 " + "─" * 34)
    node.set_gripper(geometry.preopen_width_mm)
    node.stage(profile, "safe")
    node.stage(profile, "grasp")

    worst, _ = burst()
    print(f"    열림 {geometry.preopen_width_mm:5.1f} mm   {worst:.4f}")
    for width_mm in LOAD_WIDTH_SWEEP_MM:
        node.set_gripper(width_mm)
        worst, _ = burst()
        users = sorted(name for name, geom in FLOOR_GRASP_PROFILES.items()
                       if abs(geom.close_width_mm - width_mm) < 0.05)
        print(f"    닫힘 {width_mm:5.1f} mm   {worst:.4f}   "
              f"[{', '.join(users) or '미사용'}]")

    # ── 2부 · 같은 조건 되풀이 ───────────────────────────────────────────
    # 자세도 폭도 고정하고 그리퍼만 다시 명령한다. 여기서 나오는 폭이 곧
    # **떠돌이 그 자체**다 — 다른 변수가 하나도 안 바뀌었으므로 다른 것에
    # 돌릴 여지가 없다.
    print()
    print("  ── 2부 · 조건 고정, " + f"{LOAD_REPEATS}회 되풀이 " + "─" * 27)
    trace = []
    for i in range(LOAD_REPEATS):
        node.set_gripper(geometry.preopen_width_mm)
        node.set_gripper(LOAD_WIDTH_SWEEP_MM[0])
        worst, spread = burst()
        trace.append(worst)
        print(f"    {i + 1}/{LOAD_REPEATS}   {worst:.4f}   버스트 내 편차 {spread:.4f}")

    node.set_gripper(geometry.preopen_width_mm)
    node.stage(profile, "safe")
    node.stage(profile, "idle")

    # ── 결론 ─────────────────────────────────────────────────────────────
    print()
    print(BANNER)
    wander = max(trace) - min(trace)
    print(f"  떠돌이 폭(조건 고정, {LOAD_REPEATS}회) = {wander:.4f} "
          f"= {wander / LOAD_QUANTUM:.1f} 양자")

    session_max = max(observed)
    ceiling = max(session_max, HISTORICAL_EMPTY_MAX)
    print(f"  이번 회차 빈손 최대 = {session_max:.4f}")
    if ceiling > session_max:
        print(f"  과거 관측 최대     = {HISTORICAL_EMPTY_MAX:.4f} "
              "<- 이번 회차가 최악을 재현하지 못했으므로 이쪽을 씁니다")
    print(f"  실측 파지 부하 최소 = {GRIPPED_LOAD_MIN:.4f} "
          "(2026-08-26 rook/knight/queen)")

    if ceiling >= GRIPPED_LOAD_MIN:
        print()
        print("  ⛔ **빈손 띠가 파지 띠와 겹칩니다.** 부하 하나로는 못 가릅니다 —")
        print("     뎁스 카메라 확인(confirm_grasp)이 유일한 판정이 됩니다.")
        return observed

    recommended = (ceiling + GRIPPED_LOAD_MIN) / 2.0
    margin = (GRIPPED_LOAD_MIN - ceiling) / 2.0
    print()
    print(f"  ✅ 권장  LOAD_THRESHOLD = {recommended:.4f}")
    print(f"          EMPTY_LOAD_CEILING = {recommended:.4f}")
    print(f"     양쪽 여유 각각 {margin:.4f} = {margin / LOAD_QUANTUM:.1f} 양자")
    if margin <= wander:
        print(f"     ⛔ 여유가 떠돌이 폭({wander:.4f})보다 좁습니다 — "
              "임계 하나로는 못 가릅니다.")
    elif margin < 2 * LOAD_QUANTUM:
        print("     ⚠️ 두 양자 미만입니다 — 판독 한 칸만 틀려도 뒤집힙니다.")
    print(BANNER)
    return observed


def mode_scale(node, label):
    """판독 **척도**가 맞는지 본다 — 기준점을 몰라도 되는 방식으로.

    ## 왜 --mode k로는 안 되는가

    기존 K는 base_link 기준 줄자로 잡았다. 차체 전면 기준으로 다시 재면 차체
    절반 길이만큼 다른 K가 나오는데, 그러면 **숫자가 달라도 모델이 바뀐 건지
    기준점이 다른 건지 구분이 안 된다.** 모델을 갈아 끼운 뒤 "척도가
    유지됐는가"를 묻는 데는 절대 K가 오히려 방해가 된다.

    ## 차이를 보면 기준점이 사라진다

        판독 = K / e,   실제거리 = d + c   (c = 모르는 기준점 오프셋)

    한 자리에서 읽고, 줄자로 잰 만큼 **곧게 뒤로** 밀고, 다시 읽는다. 두
    판독의 차이에서 c가 상쇄된다. 200mm 밀었는데 판독이 200mm 변하면 척도가
    맞는 것이고, 180mm만 변했으면 K가 그 비율만큼 틀린 것이다.

        척도 f = 판독변화 / 줄자거리
        보정된 K = 지금 K / f

    ## 왜 이것이 GRASP가 필요로 하는 바로 그 검사인가

    GRASP는 절대 거리를 안 쓴다 — `(관측 - 그 클래스의 턱 선)`이라는 **차이**만
    쓴다. 그래서 K의 절대값이 틀려도 차이만 맞으면 전진량은 정확하다. 반대로
    차이가 틀리면 절대값이 아무리 그럴듯해도 전진량이 어긋난다.

    f ~= 1.0이면 그 클래스의 턱 선을 다시 잴 필요가 없다.
    """
    print(BANNER)
    print(f"모드 S · '{label}' 판독 척도 확인")
    print(BANNER)
    print("  물체를 정면에 놓고 한 번, **줄자로 잰 만큼 곧게 뒤로 밀고**")
    print("  한 번 더 읽습니다. 기준점은 몰라도 됩니다 — 차이만 씁니다.")
    print()
    print("  ⚠️ 좌우로 흔들리면 안 됩니다. 앞뒤로만 미세요.")
    print(f"     파지 거리대(약 0.18m)에서 시작해 {SCALE_MAX_PUSH_MM:.0f}mm 정도 미세요.")
    print("     ⚠️ 더 멀리 밀면 거리 게이트(bbox 아래끝 >= "
          f"{OBSERVE_MIN_BOTTOM_Y_PX:.0f}px)에 걸려")
    print("        원거리 자리가 통째로 미검출이 됩니다 — 2026-08-26에 200mm에서 겪었습니다.")
    print()

    def read():
        """캐시를 비워 가며 독립 표본을 뜨고 중앙값을 낸다."""
        values = []
        for i in range(SCALE_SAMPLES):
            if i:
                time.sleep(OBSERVE_CACHE_SEC + 0.3)
            response = node.observe(label)
            if not response.found:
                print("      (미검출 — 건너뜁니다)")
                continue
            values.append(response.forward_m)
            print(f"      {i + 1}/{SCALE_SAMPLES}  전방 {response.forward_m:.4f} m")
        if len(values) < 2:
            raise RuntimeError(
                "유효 검출이 부족합니다.\n"
                "    원거리 자리에서 이렇게 되면 대개 **거리 게이트**입니다 —\n"
                f"    observe_target은 bbox 아래끝이 {OBSERVE_MIN_BOTTOM_Y_PX:.0f}px\n"
                "    아래에 있어야 통과시키는데, 멀어질수록 물체가 화면 위로\n"
                "    올라가 약 0.3m 너머는 통째로 거부됩니다(배경 오검출을\n"
                "    막는 그 게이트라 풀 수 없습니다).\n"
                f"    -> 미는 거리를 {SCALE_MAX_PUSH_MM:.0f}mm 이하로 줄이세요.")
        return statistics.median(values)

    input("  가까운 자리에 놓고 Enter > ")
    near = read()
    print(f"    가까운 자리 판독 = {near:.4f} m")

    print()
    raw = input("  뒤로 민 거리(mm)를 줄자로 재서 입력하세요 > ").strip()
    try:
        moved_mm = float(raw)
    except ValueError:
        print("  ⛔ 수치가 아닙니다.")
        return None
    if moved_mm <= 0:
        print("  ⛔ 0보다 커야 합니다.")
        return None

    input("  민 뒤 Enter > ")
    far = read()
    print(f"    먼 자리 판독 = {far:.4f} m")

    change_mm = (far - near) * 1000.0
    scale = change_mm / moved_mm

    print()
    print(BANNER)
    print(f"  줄자 이동   = {moved_mm:7.1f} mm")
    print(f"  판독 변화   = {change_mm:7.1f} mm")
    print(f"  척도 f      = {scale:7.3f}")
    print()

    current_k = CURRENT_K.get(label)
    if abs(scale - 1.0) <= SCALE_TOLERANCE:
        print(f"  ✅ 척도가 유지됐습니다 (오차 {abs(scale - 1.0) * 100:.1f}%,"
              f" 허용 {SCALE_TOLERANCE * 100:.0f}%).")
        print(f"     '{label}'의 K와 턱 선을 **다시 잴 필요가 없습니다.**")
    else:
        print(f"  ⛔ 척도가 {(scale - 1.0) * 100:+.1f}% 어긋났습니다.")
        if current_k:
            print(f"     보정된 K = {current_k:.4f} / {scale:.3f} "
                  f"= {current_k / scale:.4f}")
            print("     perception_node.py의 CLASS_DISTANCE_CALIBRATION_SQRT_PX_M에")
            print("     넣고, **그 클래스의 턱 선과 좌우 영점을 다시 재세요**")
            print("     (--mode jaw, --mode seat). 셋은 같은 척도 위에 있어야 합니다.")
        else:
            print(f"     '{label}'은 K가 아직 없습니다 — --mode k로 먼저 잡으세요.")
    print()
    print("  ⚠️ 이 검사의 분해능:")
    print(f"     줄자 1mm 오차 -> 척도 {100.0 / moved_mm:.2f}%")
    print(f"     판독 잡음(약 2mm) -> 척도 {200.0 / moved_mm:.1f}%")
    print("     즉 이 검사가 '유지됐다'고 해도 그 폭 안의 어긋남은 못 가립니다.")
    if current_k:
        jaw = JAW_LINE_FOR_HINT.get(label)
        if jaw:
            offset_mm = jaw * (scale - 1.0) * 1000.0
            print()
            print(f"     참고: 척도가 {(scale - 1.0) * 100:+.1f}%면 턱 선 {jaw:.4f}m 위치가")
            print(f"     {offset_mm:+.1f}mm 어긋나 읽힙니다 — 전진량에 그만큼 상시 오프셋이")
            print("     얹힙니다. 전진량 자체가 20~30mm라 무시할 크기가 아닙니다.")
    print(BANNER)
    return scale


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--mode",
        choices=("jaw", "seat", "confirm", "gate", "servo1", "k", "load", "scale"),
        required=True)
    parser.add_argument("--label", default="queen", choices=sorted(PROFILE_BY_LABEL))
    parser.add_argument("--profile", default="chess_queen")
    args = parser.parse_args()

    rclpy.init()
    node = CalibrationNode()
    try:
        if args.mode == "jaw":
            node.require_camera()
            mode_jaw_line(node, args.label)
        elif args.mode == "gate":
            node.require_camera()
            mode_gate(node, args.label)
        elif args.mode == "confirm":
            node.require_camera()
            mode_confirm(node, args.label)
        elif args.mode == "seat":
            node.require_camera()
            mode_seat(node, args.label)
        elif args.mode == "k":
            node.require_camera()
            mode_k(node, args.label)
        elif args.mode == "load":
            # 카메라를 안 쓴다 — 팔과 부하만 본다.
            mode_load(node, args.label)
        elif args.mode == "scale":
            node.require_camera()
            mode_scale(node, args.label)
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
