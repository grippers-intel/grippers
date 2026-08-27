#!/usr/bin/env python3
"""퀸 파지 → 저속 접근 → 라이다 자동 정지 → INSERT 통합 실기 테스트.

2026-08-26 사용자 지시로 만든 도구다. 그날 앞선 테스트에서 바구니를 차체
앞 12cm에 **놓고** carry→drop→투하까지 성공했는데, 그건 "이미 도착해 있는"
상태에서의 검증이었다. 이 도구는 그 앞단, 즉 **주행하다가 어디서 멈춰야
하는가**를 붙여서 처음부터 끝까지 한 번에 돌린다.

    바구니 50cm 앞에서 퀸 파지 → CARRY → 저속 전진하며 1초마다 라이다·뎁스로
    바구니까지 거리 측정 → 앞 테스트에서 얻은 라이다 값에 도달하면 정지 +
    특별 신호 → 사람이 Enter → INSERT

## 왜 "연속 주행"이 아니라 "짧은 버스트 + 정지"인가

두 가지 이유가 겹친다.

1. **데드밴드.** 이 베이스는 0.05m/s 명령에 바퀴가 아예 안 돈다(2026-08-24
   실기, grasp_test_console.APPROACH_SPEED_MPS 주석). 실제로 움직이는 최저
   속도가 0.06m/s라, "더 느리게"를 속도로는 만들 수 없다. 같은 주석의 결론
   그대로 **짧은 버스트와 정지를 반복**해 평균 속도를 낮춘다.
2. **정지 상태에서 재는 게 정확하다.** 라이다 한 바퀴는 10Hz라 주행 중
   스캔은 한 프레임 안에서도 차량이 움직인 만큼 왜곡된다. 버스트가 끝나고
   멈춘 뒤 재면 그 왜곡이 없다.

덤으로 안전 성질이 하나 더 생긴다 — 한 사이클에 앞으로 갈 수 있는 거리가
버스트 하나(약 21mm)로 **상한이 걸린다**. 아무리 판단이 늦어도 그 이상은
바구니 쪽으로 못 간다.

## 정지 판단을 왜 라이다로만 하는가

뎁스 카메라도 같이 읽어서 출력하지만 **정지 판단에는 안 쓴다**. 두 가지
이유다: (a) 목표값(TARGET_LIDAR_M)이 라이다로 잰 값이라 같은 센서로 비교해야
의미가 있고, (b) 이 구조광 뎁스 카메라는 근거리에서 값이 통째로 비는 일이
잦다(2026-08-23에 파지용으로는 폐기한 이유와 같다). 뎁스는 "라이다가 헛것을
보고 있지 않은지" 교차 확인하는 참고값이다.

## 접근하는 동안 빔이 훑는 높이가 달라진다

라이다가 11.3도 아래로 기울어져 있어, 빔이 바구니를 때리는 **높이가 거리에
따라 변한다**. 50cm에서는 바닥 위 40mm(바구니 아랫부분), 13.9cm에서는
112mm(테두리 바로 아래)다. 사진으로 확인된 이 바구니는 앞턱이 앞으로
튀어나온 형상이라, 접근하는 동안 같은 "정면"이라도 서로 다른 면을 본다.

그래도 정지 판단은 성립한다 — 기울기가 고정이라 **거리와 빔 높이가 1:1로
묶여 있기** 때문이다. 판독값이 0.1386m이 되는 순간의 빔 높이는 언제나
112mm이고, 이는 2026-08-26에 성공한 그 투하 때와 같은 면을 같은 높이에서
보고 있다는 뜻이다.

주의할 것은 수렴이 매끄럽지 않을 수 있다는 점이다. 앞턱이 위쪽 벽보다 더
튀어나와 있으면 중간 거리에서 판독값이 한 번 가까워졌다가 다시 멀어질 수
있다. `select_face_points`가 항상 **가장 가까운 덩어리**를 취하므로 그럴 때
멈추는 쪽은 "더 멀리"다 — 안전한 방향이다. 출력의 잔차·폭·점 개수 열을
보면서 피팅이 튀는 구간이 있는지 확인할 것.

## 실행 전 필요한 것 (컨테이너 안, ROS_DOMAIN_ID=21)

    ros2 launch controller odom_publisher.launch.py      # cmd_vel/odom_raw
    ros2 launch peripherals lidar.launch.py              # /scan_raw
    ros2 launch peripherals depth_camera.launch.py       # 뎁스(참고값)
    ros2 run grippers_arm arm_driver --ros-args -p arm_port:=/dev/soarm

    (launch 두 개는 need_compile=True, DEPTH_CAMERA_TYPE=ascamera 환경변수가
     있어야 뜬다)

perception_node는 이 도구가 쓰지 않지만, 껐다면 테스트 후 반드시 다시 올릴 것.
"""

import argparse
import math
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from std_srvs.srv import Trigger

from grippers_interfaces.action import MoveToFloorPose
from grippers_interfaces.srv import GetArmState, GetLoad, SetGripper
from grippers_arm.floor_grasp_profiles import FLOOR_GRASP_PROFILES

# basket_lidar_align은 아직 Pi의 ros2_ws에 빌드돼 있지 않다(브랜치 작업물).
# 복사본을 만들면 상수가 갈라지므로 저장소 원본을 경로로 직접 집어넣는다.
_ALIGN_DIR = Path(__file__).resolve().parent.parent / "ros2_ws/src/grippers_base/grippers_base"
sys.path.insert(0, str(_ALIGN_DIR))
import basket_lidar_align as align  # noqa: E402


# --- 실측에서 온 상수 ------------------------------------------------------

# 2026-08-26엔 퀸 전용으로 하드코딩돼 있었다. 2026-08-27에 --profile로
# 바꿀 수 있게 열었다 — main()이 argparse 뒤 이 전역을 다시 쓴다.
PROFILE = "chess_queen"
PROFILE_LABEL = {
    "chess_queen": "퀸", "chess_knight": "나이트", "chess_rook": "룩",
    "cube": "박스", "star_column": "스타", "soccer_polyhedron": "축구공",
}


def _eul_reul(word: str) -> str:
    """word 뒤에 붙일 목적격 조사('을'/'를'). 받침 유무로 정한다."""
    code = ord(word[-1]) - 0xAC00
    if 0 <= code < 11172 and code % 28 != 0:
        return "을"
    return "를"


def _i_ga(word: str) -> str:
    """word 뒤에 붙일 주격 조사('이'/'가'). 받침 유무로 정한다."""
    code = ord(word[-1]) - 0xAC00
    if 0 <= code < 11172 and code % 28 != 0:
        return "이"
    return "가"

# 2026-08-26 실기: 이 라이다 판독 거리에서 carry→drop→투하가 성공했다
# (정면 피팅 거리 0.1386m, 잔차 2.8mm, yaw -0.87도. 나이트로 검증).
#
# ⚠️ 이 값은 **차체 전면에서 바구니까지의 거리가 아니다** — 라이다가 읽은
# 값 그 자체다. 굳이 차체 기준으로 환산하지 않는 이유: 정지 판단을 어차피
# 라이다로 하니, 중간에 오프셋을 끼우면 오차만 하나 더 생긴다. 2026-08-26에
# 판으로 잰 오프셋(4.54cm)과 바구니로 잰 오프셋(1.9cm)이 2.6cm 어긋나 있어
# 실제로 믿을 수 없는 값이기도 하다(바구니 앞턱이 튀어나온 형상 탓으로 추정).
# 2026-08-26 2차 실기에서 0.1386을 목표로 잡았더니 0.1301에서 멈췄다 —
# 아래 ARRIVE_TOLERANCE_M 주석의 구조적 오버슈트 때문이다. 둘 다 투하에
# 성공했으므로 **검증된 창은 [0.130, 0.139]**이고, 이제 오버슈트가
# 목표 아래로 내려가지 않게 고쳤으므로 목표를 창의 위쪽 끝에 맞춘다.
#
# 아래로 못 내리는 이유가 따로 있다. 라이다 판독 약 0.125m에서 빔이
# 바구니 테두리를 넘어가 **바구니를 통째로 놓친다**(basket_lidar_align의
# beam_height_m 참고). 0.1301에서 잰 빔 높이가 114.0mm로 테두리까지
# 1.0mm밖에 안 남았었다. 그래서 이 목표값은 "더 붙이면 좋다"가 아니라
# **절벽에서 떨어져 있으라**는 값이다.
TARGET_LIDAR_M = 0.140

# 목표에 이만큼 들어오면 도착으로 본다.
#
# 2026-08-26: 5mm였는데 **최소 버스트 이동량(15mm)보다 작아서** 구조적으로
# 지나쳤다. 남은 거리가 6mm였을 때 "도착 아님"으로 판정하고 최소 버스트
# 15mm를 나갈 수밖에 없어 목표를 8.5mm 지나쳤다. 허용치가 최소 이동량보다
# 작으면 반드시 이렇게 된다.
#
# 그래서 10mm로 올리고, 그와 별개로 "남은 거리가 최소 버스트 이동량보다
# 작으면 더 나눌 수 없으니 도착으로 친다"는 규칙을 넣었다. 데드밴드 때문에
# 버스트를 15mm보다 잘게 못 쪼개므로, **실효 도착 창은 결국 15mm가**
# 정한다 — 이 상수를 더 올리기 전에는 15mm 아래로 못 내려간다.
ARRIVE_TOLERANCE_M = 0.010

# 이보다 가까워지면 판단을 기다리지 않고 즉시 멈춘다.
#
# 2026-08-26: 0.105였는데 **닿을 수 없는 값이었다**. 정지 창이 [0.140,
# 0.155]이고 그 아래는 절벽(0.125)이라, 0.105를 읽는 상황은 이미 바구니를
# 놓친 뒤다. 절벽 바로 아래로 올려 둔다 — 정상 주행이면 절대 안 걸리고,
# 걸렸다면 "보고 있는 게 우리가 교정한 그 면이 아니다"라는 뜻이다.
EMERGENCY_MIN_M = 0.120

# 근거리 안전 구멍(2026-08-26 실기분석 §5/§9): 라이다 빔이 테두리를 넘어가면
# 판독값이 "멀어지는" 방향으로 거짓말한다 — 바구니 뒤 벽까지의 거리이거나
# math.inf다. 둘 다 값이 커지므로, EMERGENCY_MIN_M처럼 "판독값이 작으면
# 멈춘다"는 규칙은 원리적으로 이 실패 모드를 못 잡는다.
#
# 대신 이 거리(CLIFF_WATCH_M) 안쪽부터는 "전진했는데 판독값이 커졌다"와
# "가까이서 정면 피팅을 놓쳤다" 두 신호로 직접 잡는다 — 절벽을 넘는 순간
# (전환 즉시) 잡는 가장 이른 방어선이다(같은 분석 §9의 후보 3번·2번을
# 함께 적용). CLIFF_JUMP_M은 정상 잔차(1~5mm대)보다 넉넉히 큰 여유다.
CLIFF_WATCH_M = 0.20
CLIFF_JUMP_M = 0.015

# 실제로 바퀴가 도는 최저 속도. 낮추지 말 것 — 아래로는 아무리 오래 줘도
# 안 움직이는데 /odom_raw는 움직였다고 보고한다(2026-08-24 실기).
BURST_SPEED_MPS = 0.06
BURST_S = 0.35          # 약 21mm
MIN_BURST_S = 0.25      # 이보다 짧으면 정지마찰을 못 이길 수 있다
# 한 사이클에 앞으로 갈 수 있는 최소 거리. 이보다 잘게 못 쪼개므로 도착
# 판정의 실효 분해능이 곧 이 값이다.
MIN_BURST_TRAVEL_M = MIN_BURST_S * BURST_SPEED_MPS
NUDGE_S = 0.20          # 파지 전 수동 미세 전진 한 번

SETTLE_S = 0.35         # 정지 후 스캔이 안정될 때까지
CYCLE_S = 1.0           # 사용자 지시: 1초마다 모니터링

# 이보다 멀면 라이다가 바구니를 볼 수 없다. 빔이 기울어져 있어 라이다에서
# 70cm 앞에서 바닥에 닿기 때문이다(basket_lidar_align의 z(x) = 140 - 0.1998x).
# 그 너머의 "정면 반사"는 전부 바닥이지 바구니가 아니다 — 2026-08-26에
# 바구니를 치우자 곧바로 0.78~0.81m 바닥 반사가 잡혔고, 피팅은 폭·잔차
# 검사로 정확히 걸러 냈다. 시작 거리 50cm는 이 한계 안쪽이라 성립한다.
LIDAR_FLOOR_REACH_M = 0.70

# 폭주 방지 상한. 50cm에서 시작해 13.9cm까지면 36cm면 충분하다.
MAX_TRAVEL_M = 0.60
MAX_CYCLES = 90

# 뎁스 이미지에서 볼 관심영역 — 640x480의 **가운데 세로 띠 전체**다.
#
# 왜 중앙 사각형이 아닌가: 2026-08-26에 실제 프레임을 떠 보니 상단 2/3가
# 통째로 0(무효)이고 하단에만 값이 있었다. 이 스트림은 회전 보정 전이라
# (depth_cam_rotate_node를 안 거친다) 위아래가 뒤집혀 있는 데다, 팔이
# IDLE로 접혀 있으면 그리퍼가 근접 시야를 가려 그 영역이 통째로 0이 된다.
# 세로로 어디가 살아 있을지 고정할 수 없으므로 **세로는 전부 보고 가로만
# 가운데로 좁힌다**.
DEPTH_COLUMN_HALF_W = 80

# 정면에서 가장 가까운 값을 대표값으로 쓰되, 최솟값은 잡음 한 점에 끌려
# 다니므로 하위 10퍼센타일을 쓴다.
DEPTH_NEAR_PERCENTILE = 10

# 파지 성공 참고선. 2026-08-25 벤치에서 나이트가 0.0626이었다. 퀸은 더
# 얇아(17.0mm) 같은 7.0mm 명령에서 더 눌리므로 최소 이 정도는 나와야 한다.
# **자동 판정에 쓰지 않는다** — 사람이 보고 Enter로 넘긴다.
GRASP_LOAD_HINT = 0.05

BANNER = "=" * 68


class KeyReader:
    """Enter 없이 한 글자씩 논블로킹으로 읽는다(grasp_test_console과 같은 방식)."""

    def __enter__(self):
        # 진짜 터미널이 아니면(파이프·비대화형 ssh) cbreak를 걸 수 없다.
        # --monitor-only처럼 키가 필요 없는 용도로 그대로 돌 수 있게 죽지
        # 않고 비활성 상태로 넘어간다. 키가 실제로 필요한 단계는
        # wait_enter가 막아 준다.
        self.enabled = sys.stdin.isatty()
        if not self.enabled:
            print("⚠️ 터미널이 아니라 키 입력을 못 받습니다 — --monitor-only만 쓸 수 있습니다.",
                  file=sys.stderr)
            return self
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc):
        if self.enabled:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def ensure_cbreak(self):
        if not self.enabled:
            return
        tty.setcbreak(self._fd)
        termios.tcflush(self._fd, termios.TCIFLUSH)

    def getch(self):
        if not self.enabled:
            return None
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if ready else None

    def wait_enter(self, prompt):
        if not self.enabled:
            raise RuntimeError("키 입력을 받을 수 없는 환경입니다 — 실제 터미널에서 실행하세요")
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
        try:
            line = input(prompt)
        finally:
            tty.setcbreak(self._fd)
        if line.strip().lower() == "q":
            raise KeyboardInterrupt


class ApproachNode(Node):
    def __init__(self):
        super().__init__("basket_approach_insert_test")
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan_raw", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Image, "/ascamera/camera_publisher/depth0/image_raw",
            self._on_depth, qos_profile_sensor_data)
        self._scan = None
        self._depth = None

        self._gripper = self.create_client(SetGripper, "/arm_driver/set_gripper")
        self._load = self.create_client(GetLoad, "/arm_driver/get_load")
        self._state = self.create_client(GetArmState, "/arm_driver/get_arm_state")
        self._hold = self.create_client(Trigger, "/arm_driver/hold_position")
        self._floor = ActionClient(self, MoveToFloorPose, "/arm_driver/move_to_floor_pose")

        self._buzzer = None
        try:
            from ros_robot_controller_msgs.msg import BuzzerState
            self._buzzer_msg = BuzzerState
            self._buzzer = self.create_publisher(BuzzerState, "/ros_robot_controller/set_buzzer", 1)
        except ImportError:
            pass

    # --- 센서 -------------------------------------------------------------

    def _on_scan(self, msg):
        self._scan = msg

    def _on_depth(self, msg):
        self._depth = msg

    def pump(self, seconds=0.0):
        deadline = time.time() + seconds
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            if time.time() >= deadline:
                return

    def lidar_face(self):
        """정면 바구니까지의 거리·정렬을 잰다. `(FaceFit, 최근접거리)`."""
        self._scan = None
        for _ in range(60):                      # 최대 3초, 새 스캔 한 장 기다린다
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._scan is not None:
                break
        if self._scan is None:
            return None, None
        msg = self._scan
        points = align.scan_to_front_points(
            msg.ranges, msg.angle_min, msg.angle_increment,
            range_min=max(msg.range_min, 0.02), range_max=min(msg.range_max, 3.0))
        fit = align.fit_basket_face(points, expected_bearing_rad=0.0)
        ahead = [math.hypot(x, y) for x, y in points
                 if x > 0.0 and abs(math.atan2(y, x)) <= math.radians(15.0)]
        return fit, (min(ahead) if ahead else None)

    def depth_center_m(self):
        """정면 세로 띠에서 **가장 가까운 쪽** 거리(m)와 유효 픽셀 비율.

        참고값 전용이다 — 정지 판단에는 안 쓴다(모듈 docstring 참고).
        0은 "측정 실패"라 유효 픽셀에서 뺀다. 구조광 뎁스는 최소 거리
        아래를 전부 0으로 내보내므로, 유효 비율 자체가 "너무 가까워서
        안 보인다"는 신호로도 읽힌다."""
        msg = self._depth
        if msg is None or msg.encoding != "16UC1":
            return None, 0.0
        import numpy as np
        dtype = ">u2" if msg.is_bigendian else "<u2"
        frame = np.frombuffer(msg.data, dtype=dtype)
        frame = frame.reshape(msg.height, msg.step // 2)[:, :msg.width]
        cx = msg.width // 2
        strip = frame[:, cx - DEPTH_COLUMN_HALF_W:cx + DEPTH_COLUMN_HALF_W]
        valid = strip[strip > 0]
        if valid.size == 0:
            return None, 0.0
        near = float(np.percentile(valid, DEPTH_NEAR_PERCENTILE))
        return near / 1000.0, float(valid.size) / float(strip.size)

    # --- 구동 -------------------------------------------------------------

    def drive(self, seconds, speed=BURST_SPEED_MPS):
        twist = Twist()
        twist.linear.x = speed
        deadline = time.time() + seconds
        while time.time() < deadline:
            self.cmd_pub.publish(twist)
            self.pump(0.05)
        self.stop()

    def stop(self):
        zero = Twist()
        for _ in range(6):                       # 한 번은 유실될 수 있다
            self.cmd_pub.publish(zero)
            self.pump(0.02)

    def beep(self, repeat=3):
        if self._buzzer is None:
            return
        msg = self._buzzer_msg()
        msg.freq = 1900
        msg.on_time = 0.12
        msg.off_time = 0.12
        msg.repeat = repeat
        self._buzzer.publish(msg)
        self.pump(0.1)

    # --- 팔 ---------------------------------------------------------------

    def _call(self, client, request, timeout=15.0, label=""):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"{label} 서비스 없음 — arm_driver가 떠 있는지 확인하세요")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        result = future.result()
        if result is None:
            raise RuntimeError(f"{label} 응답 없음(timeout {timeout}s)")
        return result

    def hold_position(self):
        """토크를 켠다. 토크가 꺼져 있으면 set_gripper가 ok=False로 거부된다."""
        return self._call(self._hold, Trigger.Request(), label="hold_position")

    def set_gripper(self, width_mm):
        request = SetGripper.Request()
        request.width_mm = float(width_mm)
        return self._call(self._gripper, request, timeout=20.0, label="set_gripper")

    def get_load(self):
        return self._call(self._load, GetLoad.Request(), label="get_load").load_ratio

    def arm_state(self):
        return self._call(self._state, GetArmState.Request(), label="get_arm_state")

    def move_stage(self, stage, profile=None, timeout=60.0):
        # ⚠️ 기본 인자를 profile=PROFILE로 두면 클래스 정의 시점(=import
        # 시점)의 값(chess_queen)에 영구히 고정된다 — main()이 나중에
        # 전역 PROFILE을 바꿔도 이 함수의 기본값은 안 따라온다(파이썬
        # 기본 인자는 def 시점에 한 번만 평가된다, servo1_offset_for의
        # 같은 함정 참고). 그래서 여기서 매 호출마다 전역을 다시 읽는다.
        if profile is None:
            profile = PROFILE
        if not self._floor.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("move_to_floor_pose 액션 서버 없음")
        goal = MoveToFloorPose.Goal()
        goal.profile = profile
        goal.stage = stage
        send = self._floor.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=15.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"'{stage}' 목표가 거부됐습니다")
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(self, done, timeout_sec=timeout)
        outcome = done.result()
        if outcome is None:
            raise RuntimeError(f"'{stage}' 결과 없음(timeout {timeout}s)")
        if not outcome.result.reached:
            raise RuntimeError(f"'{stage}' 도달 실패")
        return outcome.result


# --- 단계 -------------------------------------------------------------------


def preflight(node):
    print(BANNER)
    print("0단계 · 사전 점검")
    print(BANNER)
    state = node.arm_state()
    pose = [int(v) for v in state.position_raw]
    print(f"  팔 현재 자세 raw = {pose[:5]}   servo6(그리퍼) = {pose[5]}")
    print(f"  온도 = {[int(v) for v in state.temperature_c][:5]} C")

    fit, nearest = node.lidar_face()
    if fit is None:
        raise RuntimeError("/scan_raw에서 스캔이 안 옵니다 — lidar.launch.py 확인")
    print(f"  라이다 정면 최근접 = {_m(nearest)}   피팅 = {_fit_str(fit)}")
    if not fit.ok:
        print(f"  ⚠️ 지금은 바구니 정면이 안 잡힙니다. {LIDAR_FLOOR_REACH_M:.2f} m 너머는 빔이")
        print("     바닥에 먼저 닿아 바구니를 못 봅니다 — 바구니를 50cm 부근에 두세요.")
        print("     (2단계는 정면이 잡히기 전까지 최근접 거리로만 움직입니다)")

    node.pump(1.5)
    depth_m, coverage = node.depth_center_m()
    if depth_m is None:
        print("  뎁스 중앙 = 값 없음 (참고값이라 계속 진행합니다)")
    else:
        print(f"  뎁스 중앙 = {depth_m:.3f} m (유효 {coverage * 100:.0f}%)")

    profile = FLOOR_GRASP_PROFILES[PROFILE]
    print(f"  프로파일 '{PROFILE}': 폭 {profile.object_width_mm}mm · "
          f"열기 {profile.preopen_width_mm}mm · 잡기 {profile.close_width_mm}mm · "
          f"놓기 {profile.release_width_mm}mm")
    print(f"  정지 목표(라이다) = {TARGET_LIDAR_M:.4f} m  (허용 +{ARRIVE_TOLERANCE_M * 1000:.0f}mm, "
          f"비상정지 {EMERGENCY_MIN_M:.3f} m)")
    beam = align.beam_height_m(TARGET_LIDAR_M) * 1000.0
    print(f"  그 거리에서 빔 높이 = {beam:.1f}mm  vs 테두리 {align.BASKET_RIM_HEIGHT_M * 1000:.0f}mm "
          f"(여유 {align.BASKET_RIM_HEIGHT_M * 1000 - beam:+.1f}mm)")


def phase_grasp(node, keys):
    label = PROFILE_LABEL.get(PROFILE, PROFILE)
    print()
    print(BANNER)
    print(f"1단계 · {label} 파지  (바구니에서 약 50cm 떨어진 자리)")
    print(BANNER)
    profile = FLOOR_GRASP_PROFILES[PROFILE]

    keys.wait_enter(f"  {label}{_eul_reul(label)} 차체 전면 19cm 정면에 놓고 Enter (q로 종료) > ")

    print("  토크 켜는 중...")
    node.hold_position()

    print(f"  그리퍼 여는 중 ({profile.preopen_width_mm}mm) — 내려가기 전에 먼저 연다")
    node.set_gripper(profile.preopen_width_mm)

    print("  idle → safe ...")
    node.move_stage("safe")
    print("  safe → grasp (바닥 자세) ...")
    node.move_stage("grasp")

    print()
    print("  이제 열린 턱 사이로 퀸이 들어오도록 차체를 조금씩 전진시킵니다.")
    print(f"    f = 전진 한 번(약 {BURST_SPEED_MPS * NUDGE_S * 1000:.0f}mm) · "
          f"b = 후진 한 번 · Enter = 다 됐음")
    keys.ensure_cbreak()
    nudges = 0
    while True:
        key = keys.getch()
        if key in ("\n", "\r"):
            break
        if key == "f":
            node.drive(NUDGE_S)
            nudges += 1
            print(f"    전진 {nudges}회 (누적 약 {nudges * BURST_SPEED_MPS * NUDGE_S * 1000:.0f}mm)")
        elif key == "b":
            node.drive(NUDGE_S, speed=-BURST_SPEED_MPS)
            nudges -= 1
            print(f"    후진 (누적 약 {nudges * BURST_SPEED_MPS * NUDGE_S * 1000:.0f}mm)")
        node.pump(0.05)
    node.stop()

    print(f"  그리퍼 닫는 중 ({profile.close_width_mm}mm) ...")
    response = node.set_gripper(profile.close_width_mm)
    node.pump(1.2)
    settled = node.get_load()
    verdict = "충분해 보입니다" if settled >= GRASP_LOAD_HINT else "⚠️ 낮습니다 — 헐거울 수 있습니다"
    print(f"  응답 ok={response.ok}  응답 부하={response.load_ratio:.4f}  "
          f"정착 부하={settled:.4f}  ({verdict}, 참고선 {GRASP_LOAD_HINT})")

    keys.wait_enter(f"  {label}{_i_ga(label)} 제대로 물렸는지 눈으로 확인하고 Enter (q로 종료) > ")

    print("  grasp → safe (midpoint 경유) ...")
    node.move_stage("safe")
    print("  safe → carry ...")
    node.move_stage("carry")
    state = node.arm_state()
    print(f"  CARRY 도달. servo4 = {int(state.position_raw[3])}  "
          f"그리퍼 부하 = {float(state.load_ratio[5]):.4f}")
    return settled


def phase_approach(node, keys):
    print()
    print(BANNER)
    print("2단계 · 저속 접근  (1초마다 라이다·뎁스 측정, 목표 도달 시 자동 정지)")
    print(BANNER)
    print("  주행 중 아무 키나 누르면 즉시 정지하고 중단합니다.")
    keys.wait_enter("  시작하려면 Enter (q로 종료) > ")
    keys.ensure_cbreak()

    print()
    # ⚠️ 좌우 오프셋 열은 2026-08-26 통주행 뒤에 붙였다. 그날 INSERT 조건
    # 네 개 중 셋(판독 안정성·점 개수·부하)은 로그에서 읽어낼 수 있었는데
    # **좌우 오프셋만 어디에도 안 찍혀서** 유일하게 검증을 못 했다.
    # 하필 그게 실측이 아니라 계산으로 잡은 값(BASKET_LATERAL_TOLERANCE_M)이다.
    #
    # 사용자 지시(2026-08-26): 실제로는 바구니에 **사선으로 진입할 수도**
    # 있다. 사선이면 거리와 yaw가 멀쩡해도 팔이 바구니 끝을 벗어난 곳을
    # 겨눌 수 있으므로, 좌우 오프셋이 네 조건 중 가장 중요해진다.
    print("   #   경과    라이다     남음   yaw     잔차   폭    점   좌우    뎁스        상태")
    travelled = 0.0
    started = time.time()
    prev_distance = None   # 절벽 감시용 — 직전 사이클 판독 거리

    for cycle in range(1, MAX_CYCLES + 1):
        node.pump(SETTLE_S)
        fit, nearest = node.lidar_face()
        depth_m, coverage = node.depth_center_m()

        if fit is None:
            print(f"  {cycle:3d}  {time.time() - started:5.1f}s   스캔 없음 — 정지")
            node.stop()
            return None

        distance = fit.distance_m if fit.ok else (nearest if nearest is not None else math.inf)
        source = "피팅" if fit.ok else "최근접"
        remaining = distance - TARGET_LIDAR_M
        depth_str = f"{depth_m:.3f}m/{coverage * 100:3.0f}%" if depth_m is not None else "  값없음  "

        print(f"  {cycle:3d}  {time.time() - started:5.1f}s  "
              f"{distance:6.3f}m  {remaining * 1000:+6.0f}mm  "
              f"{math.degrees(fit.yaw_error_rad):+5.1f}deg  "
              f"{fit.residual_m * 1000:4.1f}mm  {fit.face_width_m * 1000:3.0f}  "
              f"{fit.point_count:3d}  {_lateral_str(fit)}  {depth_str}  {source}"
              + ("" if fit.ok else f"  [{fit.reason}]"))

        # 근거리 절벽 방어 — EMERGENCY_MIN_M 검사보다 먼저 본다. "작아지면
        # 멈춘다"는 그 규칙이 못 잡는 실패 모드라서, 여기서 직접 잡는다.
        if prev_distance is not None and prev_distance <= CLIFF_WATCH_M:
            if not fit.ok:
                node.stop()
                print()
                print(f"  ⛔ 근거리 피팅 실패 — 직전 {prev_distance:.3f}m에서 정면을 "
                      f"잡았다가 놓쳤습니다. 절벽(빔이 테두리를 넘어감)일 수 있어 "
                      f"멈춥니다. [{fit.reason}]")
                return None
            if distance > prev_distance + CLIFF_JUMP_M:
                node.stop()
                print()
                print(f"  ⛔ 근거리에서 판독값이 커졌습니다 ({prev_distance:.3f}m → "
                      f"{distance:.3f}m). 절벽을 넘었을 가능성이 있어 멈춥니다 — "
                      f"더 가까이 가면 상황이 나빠지기만 합니다.")
                return None
        prev_distance = distance

        if distance <= EMERGENCY_MIN_M:
            node.stop()
            print()
            print(f"  ⛔ 비상정지 — {distance:.3f} m 는 하한 {EMERGENCY_MIN_M:.3f} m 안쪽입니다.")
            return None

        # 남은 거리를 최소 버스트보다 잘게 못 쪼갠다 — 여기서 한 번 더
        # 나가면 반드시 목표 아래로 내려간다. 그래서 도착으로 친다.
        if (distance <= TARGET_LIDAR_M + ARRIVE_TOLERANCE_M
                or remaining < MIN_BURST_TRAVEL_M):
            node.stop()
            node.beep()
            print()
            print(BANNER)
            print("  ★★★  도 착  ★★★   목표 라이다 거리에 들어왔습니다.")
            print(f"    라이다 거리 = {distance:.4f} m   (목표 {TARGET_LIDAR_M:.4f} m, "
                  f"오차 {(distance - TARGET_LIDAR_M) * 1000:+.1f} mm)")
            print(f"    정렬 yaw   = {math.degrees(fit.yaw_error_rad):+.2f} deg   "
                  f"잔차 {fit.residual_m * 1000:.1f} mm   점 {fit.point_count}개")
            print(f"    좌우 오프셋 = {_lateral_str(fit).strip()}   "
                  f"(허용 ±{align_tolerance_mm():.0f} mm)")
            print(f"    뎁스 참고  = {depth_str}")
            print(f"    이동 거리  = 약 {travelled * 1000:.0f} mm ({cycle} 사이클)")
            print(f"    빔 높이    = {align.beam_height_m(distance) * 1000:.1f} mm "
                  f"(테두리 {align.BASKET_RIM_HEIGHT_M * 1000:.0f}mm까지 "
                  f"{(align.BASKET_RIM_HEIGHT_M - align.beam_height_m(distance)) * 1000:+.1f}mm)")
            print(BANNER)
            return fit

        if travelled >= MAX_TRAVEL_M:
            node.stop()
            print()
            print(f"  ⛔ 이동 상한 {MAX_TRAVEL_M:.2f} m 도달 — 바구니를 못 찾은 것으로 보고 멈춥니다.")
            return None

        key = keys.getch()
        if key is not None:
            node.stop()
            print()
            print(f"  ⛔ 사용자 중단(키 '{key!r}')")
            return None

        # 목표를 지나치지 않도록 남은 거리에 맞춰 버스트를 줄인다.
        burst = min(BURST_S, max(MIN_BURST_S, remaining * 0.8 / BURST_SPEED_MPS))
        node.drive(burst)
        travelled += burst * BURST_SPEED_MPS

        elapsed = SETTLE_S + burst
        if elapsed < CYCLE_S:
            node.pump(CYCLE_S - elapsed)

    node.stop()
    print(f"  ⛔ 사이클 상한 {MAX_CYCLES} 도달 — 멈춥니다.")
    return None


def phase_insert(node, keys):
    print()
    print(BANNER)
    print("3단계 · INSERT")
    print(BANNER)
    print("  ⚠️ 팔을 크게 전개합니다. 정렬이 어긋났다고 보이면 지금 q로 빠져나오세요.")
    keys.wait_enter("  INSERT를 실행하려면 Enter (q로 종료) > ")
    profile = FLOOR_GRASP_PROFILES[PROFILE]

    print("  carry → drop ...")
    node.move_stage("drop")
    state = node.arm_state()
    print(f"    도달 자세 raw = {[int(v) for v in state.position_raw][:5]}")
    before = node.get_load()

    print(f"  그리퍼 여는 중 ({profile.release_width_mm}mm) — 투하 ...")
    node.set_gripper(profile.release_width_mm)
    node.pump(1.2)
    after = node.get_load()
    print(f"    부하 {before:.4f} → {after:.4f}   "
          f"({'놓임' if after < before - 0.01 else '⚠️ 변화가 작습니다 — 아직 물고 있을 수 있습니다'})")

    print("  접기 전에 그리퍼 닫는 중 (9.0mm) ...")
    node.set_gripper(9.0)

    print("  drop → idle ...")
    node.move_stage("idle")
    state = node.arm_state()
    print(f"    IDLE 도달 raw = {[int(v) for v in state.position_raw][:5]}")
    return before, after


def align_tolerance_mm():
    """INSERT가 허용하는 좌우 오프셋(mm). 도메인 상수와 한 곳에서 읽는다."""
    from domain.task import baseline_constants as bc
    return bc.BASKET_LATERAL_TOLERANCE_M * 1000.0


def _lateral_str(fit):
    """좌우 오프셋 표시.

    ⚠️ `lateral_known`이 False면 **모르는 것**이지 0이 아니다. 0으로 찍으면
    "가운데 있다"로 읽혀 정반대 결론이 나오므로 물음표로 구분한다 —
    바구니 면이 시야 가장자리에 걸려 한쪽 끝만 보일 때 실제로 그렇게 된다.
    """
    if not getattr(fit, "lateral_known", False):
        return "  ?  "
    return f"{fit.lateral_offset_m * 1000:+5.0f}"


def _m(value):
    return "측정 불가" if value is None else f"{value:.3f} m"


def _fit_str(fit):
    if fit is None:
        return "없음"
    if not fit.ok:
        return f"실패 ({fit.reason})"
    return (f"{fit.distance_m:.3f} m · yaw {math.degrees(fit.yaw_error_rad):+.2f}deg · "
            f"잔차 {fit.residual_m * 1000:.1f}mm · {fit.point_count}점")


def main():
    global TARGET_LIDAR_M, PROFILE

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--target", type=float, default=TARGET_LIDAR_M,
                        help=f"정지할 라이다 거리 m (기본 {TARGET_LIDAR_M})")
    parser.add_argument("--profile", default=PROFILE,
                        choices=sorted(FLOOR_GRASP_PROFILES.keys()),
                        help=f"파지 프로파일 (기본 {PROFILE}) — 2026-08-27까지 퀸만 "
                             "실기 검증됨, 나머지는 오늘 처음 이 도구로 돎")
    parser.add_argument("--skip-grasp", action="store_true",
                        help="이미 물체를 물고 CARRY에 있을 때 2단계부터 시작")
    parser.add_argument("--monitor-only", action="store_true",
                        help="주행·팔 없이 라이다/뎁스 측정만 1초마다 출력")
    args = parser.parse_args()
    TARGET_LIDAR_M = args.target
    PROFILE = args.profile

    if os.environ.get("ROS_DOMAIN_ID") != "21":
        print("⚠️ ROS_DOMAIN_ID가 21이 아닙니다 — 노드가 서로 안 보입니다.", file=sys.stderr)

    rclpy.init()
    node = ApproachNode()
    try:
        with KeyReader() as keys:
            if args.monitor_only:
                print("측정만 출력합니다. Ctrl-C로 종료.")
                while True:
                    fit, nearest = node.lidar_face()
                    depth_m, coverage = node.depth_center_m()
                    depth_str = (f"{depth_m:.3f}m/{coverage * 100:.0f}%"
                                 if depth_m is not None else "값없음")
                    print(f"  최근접 {_m(nearest)} · 피팅 {_fit_str(fit)} · 뎁스 {depth_str}")
                    node.pump(CYCLE_S)

            preflight(node)
            if not args.skip_grasp:
                phase_grasp(node, keys)
            fit = phase_approach(node, keys)
            if fit is None:
                print("\n접근이 목표에 도달하지 못했습니다 — INSERT는 건너뜁니다.")
                return 1
            phase_insert(node, keys)
            print()
            print(BANNER)
            _label = PROFILE_LABEL.get(PROFILE, PROFILE)
            print(f"완료. {_label}{_i_ga(_label)} 바구니 안에 "
                  "들어갔는지 눈으로 확인해 주세요.")
            print(BANNER)
            return 0
    except KeyboardInterrupt:
        print("\n중단합니다.")
        return 130
    except Exception as exc:                     # noqa: BLE001 -- 실기 도구
        print(f"\n오류: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            node.stop()
        except Exception:                        # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
