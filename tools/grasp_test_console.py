#!/usr/bin/env python3
"""GRASP 실기 테스트 콘솔 — 하드웨어 연결 시 대화식으로 접근→파지→운반→투하
전 과정을 단계별 키 입력으로 검증한다.

⚠️ 이 파일은 하드웨어 미연결 상태에서 작성됐다(2026-08-23). 실기 연결 후
아래 값·가정을 반드시 재확인할 것:
  - FX/CX(카메라 내참수, yolov5_ros2/cv_tool.py에서 그대로 가져옴) — 실제
    RGB 카메라가 그 파일 기준과 같은 카메라인지 확인

그리퍼캠(/dev/gripper_cam) 스트림·면적 판정은 2026-09-01에 뺐다 — 3단계에서
그걸 넘겨받으려고 perception_node를 죽이던 이유(confirm_grasp이 그 장치를
독점) 자체가 죽은 코드였다(observe_target 기반으로 대체된 뒤 안 지워짐,
grippers_perception/perception_node.py 참고). 이제 3~7단계 내내
perception_node가 그대로 떠 있고, 파지 판정은 load_ratio(그리퍼 부하)만
본다 — 그리퍼캠 면적은 애초에 demo_rook_run.py에서도 "빈 그리퍼가 문 것보다
면적이 크게" 나와 신뢰 못 한다고 확인된 신호였다.

실행 (2026-08-23 실기로 검증된 절차 — 컨테이너는 exec_shell.sh로 들어간
zsh, `-it`인 대화형 tty가 있어야 한다):

`/home/pi/docker/shared/grippers`가 컨테이너의 `/grippers`·`/ros2_ws`와
바인드 마운트라 파일은 맥북에서 scp로 바로 그 경로에 놓으면 컨테이너 안에
즉시 나타난다 — docker cp 불필요:

    scp tools/grasp_test_console.py pi@10.82.133.189:/home/pi/docker/shared/grippers/tools/grasp_test_console.py

컨테이너 안(zsh)에서 순서대로 — 이 셸은 bash가 아니라 zsh이므로 반드시
setup.zsh를 쓸 것(setup.bash를 zsh에서 소싱하면 BASH_SOURCE 문법 때문에
경로 계산이 깨진다):

    export need_compile=False
    export DEPTH_CAMERA_TYPE=ascamera
    export ROS_DOMAIN_ID=21
    source /opt/ros/humble/setup.zsh
    source /ros2_ws/install/setup.zsh
    source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.zsh

    ros2 launch controller odom_publisher.launch.py > /tmp/odom.log 2>&1 &
    ros2 launch peripherals depth_camera.launch.py > /tmp/depth_cam.log 2>&1 &
    sleep 8
    ros2 run grippers_perception depth_cam_rotate_node > /tmp/rotate.log 2>&1 &
    ros2 run grippers_perception perception_node > /tmp/perception.log 2>&1 &
    ros2 run grippers_arm arm_driver --ros-args -p enable_torque_on_start:=true > /tmp/arm.log 2>&1 &
    sleep 3

    python3 /grippers/tools/grasp_test_console.py --raw-cls rook

`depth_camera.launch.py`가 내부적으로 띄우는 `ascamera_node`는 `/ros2_ws`가
아니라 별도 워크스페이스(`third_party_ros2/third_party_ws`)에 설치돼 있다
— 위 `source .../third_party_ws/install/setup.zsh` 줄을 빠뜨리면
`package 'ascamera' not found`로 조용히 실패하고 카메라가 안 뜬다(2026-08-23
재확인). `ros2 launch`는 첫 시도에서 `uvc_open:Busy`로 한 번 실패했다가
자동 재시도해 뜨는 경우가 있어 `sleep 8` 정도 여유를 둔다.

다른 물체로 테스트할 때는 --raw-cls만 바꾼다(profile은 자동 유도됨):
knight, queen, box, soccer, star.

`grippers_base/base_driver`와 LiDAR는 일부러 안 띄운다 — 이 스크립트는
cmd_vel/odom_raw를 직접 쓰고 base_driver 서비스는 호출하지 않는다.

⚠️ 재시작할 땐 새로 띄우기 전에 이전 프로세스를 먼저 죽일 것
(`pkill -f odom_publisher`, `pkill -f depth_camera.launch`,
`pkill -f depth_cam_rotate_node`, `pkill -f grippers_perception/perception_node`)
— 안 그러면 같은 노드가 중복으로 떠서 시리얼/카메라 장치를 서로 충돌시킨다
(arm_driver 중복 실행 때 겪은 것과 같은 문제, 2026-08-23 실기 확인).

로그: 실행하면 컨테이너 안 `/tmp/grasp_test_log_<epoch>.jsonl`에 사람이 볼
필요 없는 구조화 기록(JSON Lines, 매 단계 관측값·거리·면적·load)이 남는다 —
시작할 때와 끝날 때 이 경로가 화면에 그대로 찍힌다. Claude에게 분석을
맡기려면 컨테이너 -> 호스트 -> 맥북 순으로 꺼내온다(경로의 <epoch>는 실행 시
찍힌 실제 값으로 바꿀 것):

    ssh pi@10.82.133.189 "docker cp IntelPi:/tmp/grasp_test_log_<epoch>.jsonl /tmp/"
    scp pi@10.82.133.189:/tmp/grasp_test_log_<epoch>.jsonl .

키 배치:
  1단계 — Enter                  : 룩 위치(전방 cm, 좌/우 cm) 관측 + YOLO 캡처 저장
  (2단계 정렬 주행은 2026-08-24 제거 — 물체를 사람이 직접 놓고 시작한다)
  3단계 — g                      : GRASP 진입(파지 직전 자세)
  4단계 — Space/a/d, c(정지)     : 미세 전진
  5단계 — g                      : 파지(닫기)→들어올리기(midpoint), 부하 확인
  6단계 — w/a/s/d, c(정지)       : CARRY_IDLE 도달 후 자유 주행(바구니로 이동)
  7단계 — Enter(확인 후 자동)     : 바구니 투하(drop→그리퍼 열기→idle)
  아무 단계에서나 Ctrl+C          : 즉시 정지 명령 발행 후 종료(그 이후 상태는
                                    수동으로 확인할 것 — 자동 복구 없음)

6개 물체 클래스 전부 대응한다 — `--raw-cls`로 고른다(기본값 rook, 오늘 세션
테스트 대상). `--profile`은 생략하면 CLASS_TO_PROFILE로 자동 유도되므로 보통
안 줘도 된다. box/star는 K_CLASS가 아직 미실측(perception_node.py 참고)이라
전방/좌우 cm 추정은 건너뛰고 경고만 찍힌다 — 그리퍼 동작 자체는 문제없다.

    --raw-cls: knight | queen | rook | box | soccer | star
"""
from __future__ import annotations

import argparse
import json
import os
import math
import select
import subprocess
import sys
import termios
import threading
import time
import tty
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from grippers_interfaces.action import MoveToFloorPose
from grippers_interfaces.srv import GetArmState, GetLoad, ObserveTarget, SetGripper

# arm_driver_node와 같은 소스에서 직접 가져온다 — close_width_mm 등을 이
# 파일에 따로 베껴 적으면 profile 값이 바뀔 때 조용히 어긋난다.
from grippers_arm.floor_grasp_profiles import FLOOR_GRASP_PROFILES
from grippers_arm.gripper_calibration import GRIPPER_CLOSED_MM

# --- 오늘(2026-08-23) 실기로 확정한 값들 ---------------------------------

# perception_node.py CLASS_DISTANCE_CALIBRATION_SQRT_PX_M과 동일(2026-08-23
# 핫스팟 실기 실측). distance_m = K_CLASS / sqrt(bbox_area_px), bbox_area_px
# = h*w. None은 미실측 클래스(box, star) — 이 스크립트는 해당 클래스에서
# 전방/좌우 cm 계산을 건너뛴다.
#
# ⚠️ 2026-08-27: queen이 여기서 두 번(2026-08-26, 2026-08-27) 안 갈아 끼워진
# 채로 남아 있었다 — perception_node.py를 고칠 때마다 이 사본도 같이 고칠 것.
K_CLASS = {
    "knight": 39.5578,
    "queen": 38.3357,
    "rook": 37.7658,
    "box": 23.2733,
    "soccer": 25.8794,
    "star": 24.1690,
}
# 2026-08-24: 거리 모델을 z = K/sqrt(hw)에서 z = K/(sqrt(hw) - BBOX_PADDING_PX)로
# 바꿨다. 룩을 0.40·0.70·1.04m 세 거리에서 재니 역산 K가 35.49 -> 36.78 -> 37.40으로
# 거리에 따라 단조 증가했는데, 순수 핀홀이라면 상수여야 한다 — 검출 bbox가 물체
# 실루엣보다 항상 일정 픽셀 크게 잡히기 때문이다. 그 몫을 빼면 세 점이 전부
# ±0.4cm 안에 들어온다(자세한 근거는 perception_node.py의 같은 날짜 주석).
BBOX_PADDING_PX = 2.5

# perception/observe_target의 raw_cls 이름 -> arm_driver/move_to_floor_pose의
# profile 이름. 인식기와 팔 쪽 명명이 서로 다르다(MoveToFloorPose.action 주석
# 참고: "cube | star_column | soccer_polyhedron | chess_rook | chess_queen |
# chess_knight").
CLASS_TO_PROFILE = {
    "knight": "chess_knight",
    "queen": "chess_queen",
    "rook": "chess_rook",
    "box": "cube",
    "soccer": "soccer_polyhedron",
    "star": "star_column",
}

# ⚠️ 2026-08-24 정정: 이전 값(fx=602.7175, cx=351.3056)은 벤더 파일
# yolov5_ros2/cv_tool.py에서 가져온 것이라 **이 카메라 값이 아니었다**.
# 실제 /ascamera/camera_publisher/rgb0/camera_info 실측값으로 교체한다.
#
# depth_cam_rotate_node가 이미지를 180도 회전시키므로 주점 cx도 뒤집어야
# 한다(cx' = width - cx) — perception_node._on_rgb_camera_info와 같은 보정.
# fx는 회전과 무관해 그대로 쓴다.
#
# 좌우 오프셋은 ObserveTarget이 y(세로) 픽셀을 안 줘서 완전한 왜곡보정은
# 못 하고 표준 핀홀 근사(X = (u−cx)·Z/fx, 왜곡 무시)만 쓴다.
CAMERA_WIDTH_PX = 640
FX_PX = 588.9754638671875
CX_PX = CAMERA_WIDTH_PX - 325.3050842285156  # = 314.695 (180도 회전 보정 후)

# 실측 내참수를 써도 남는 좌우 잔차. 2026-08-24 실기: rook을 차체 중심선
# (좌우 0cm)·전방 40cm에 놓았는데 좌측 2.91cm로 관측됐다 — 뎁스카메라가
# 차체 중심에서 물리적으로 어긋나게 장착된 것으로 보인다(사용자 확인).
#
# 2026-08-24 후속: 70cm 정중앙에서 한 번 더 재서 두 모델을 갈랐다.
#   (a) 장착 위치가 옆으로 밀림  → 오차가 거리와 무관한 상수(m)
#   (b) 장착이 yaw로 틀어짐/주점 오차 → 오차가 거리에 비례(= 상수 픽셀)
# 70cm에서 (a)는 bbox 중심 x=290.6px, (b)는 271.8px을 예측했고 실측은
# 284.0px이었다 — 잔차가 (a) 0.80cm, (b) 1.47cm로 **(a)가 더 잘 맞는다**.
# 그래서 픽셀이 아니라 미터로 더하는 지금 형태를 유지한다.
#
# ⚠️⚠️ 2026-08-24 미해결 — **회전 정렬 후에는 이 보정이 안 맞는다**.
#
# auto_grasp_sequence.py --turn-only로 물체를 화면 중앙에 맞춘 직후, 같은
# 물체의 좌우 위치를 세 가지로 재보면 전부 다르게 나온다:
#
#     도구 보고(이 보정 적용 후)  +0.2cm   (즉 "정렬 완료"로 판정)
#     같은 프레임의 보정 전 원시값 -2.71cm
#     사용자가 depth 카메라로 확인 -4.5cm
#     줄자 실측(진짜 값)           -6.5cm   <- 이대로 직진하면 못 잡는다
#
# 즉 정렬이 끝났다고 보고한 시점에 물체는 실제로 6.5cm 왼쪽에 있었다.
# 이 보정값(+2.91cm)은 **차체를 안 돌린 상태**에서 40cm·70cm 두 점으로
# 잡은 것이라, 회전이 개입한 뒤의 기하는 담고 있지 않다.
#
# 같은 실행에 원인을 가리키는 단서가 하나 더 있다: 제자리 회전만 했는데
# 보고된 전방 거리가 48.0 -> 51.8cm로 3.8cm(7.9%) **늘었다**(총 회전 17.0도).
# 카메라가 회전축 위에 있다면 제자리 회전은 카메라-물체 거리를 거의 안
# 바꿔야 한다. 거리가 변했다는 건 **카메라가 회전 중심에서 옆으로 떨어져
# 장착돼 있다**는 뜻이고, 그 기하에서는 "화면 중앙에 오도록 회전"이 곧
# "그리퍼 진행선 위에 놓기"가 아니다 — 두 선이 나란히 어긋난 채로 남는다.
#
# 후보 원인(아직 어느 것도 확정 안 됨):
#   1. 카메라가 회전 중심에서 옆으로 떨어져 있어 회전 후 오차가 남는다
#      (위 거리 변화가 이쪽을 지지한다)
#   2. 파지 중심 자체가 중심선에서 좌측 20mm다(floor_grasp_profiles.py의
#      HORIZONTAL_SAFE_145_RAW 실측 주석) — 부호가 반대라 이것만으로는
#      설명이 안 되지만 일부는 여기서 온다
#   3. LATERAL_BIAS_M 자체가 회전 없는 조건에서만 맞는 값이다
#
# 가르는 방법: 물체를 **처음부터 정중앙**에 두고(회전 불필요) 같은 세 값을
# 재보면 된다. 그때도 어긋나면 3번, 맞으면 1번/2번이다.
LATERAL_BIAS_M = 0.0291

LOAD_THRESHOLD = 0.04  # domain/task/states.py GraspState.LOAD_THRESHOLD과 동일

# 이 콘솔 자신은 더 이상 그리퍼캠을 안 쓰지만(2026-09-01, 아래 GripperCam
# docstring 참고), auto_grasp_sequence.py·grasp_cycle.py·
# straight_approach_calibrate.py가 여기서 GripperCam/restart_perception_node/
# start_stream_server를 그대로 import해 쓴다 — 지우지 않는다.
GRIPPER_STREAM_PORT = 8090

# perception_node 재기동을 기다리는 최대 시간 — ultralytics 모델 로드가
# 오래 걸린다(실측 15s 안팎).
PERCEPTION_RESTART_TIMEOUT_S = 40.0

# 1단계에서 YOLO 적용 이미지를 남길 곳·모델. perception_node.py의
# CPU_YOLO_MODEL_PATH_DEFAULT와 같은 파일을 쓴다(같은 추론을 눈으로 보려는
# 것이므로 다른 모델을 쓰면 의미가 없다). 호스트
# ~/docker/shared/grippers/recordings = 컨테이너 /grippers/recordings 바인드
# 마운트라 맥북에서 scp로 바로 꺼낼 수 있다.
YOLO_CAPTURE_DIR = "/grippers/recordings/yolo_captures"
# perception_node.CPU_YOLO_MODEL_PATH_DEFAULT와 같은 경로를 봐야 한다 — 콘솔이
# 캡처에 쓰는 모델과 perception_node가 판단에 쓰는 모델이 다르면 이 콘솔의
# 진단값을 믿을 수 없다. /tmp가 아니라 바인드 마운트된 /grippers/models인
# 이유는 perception_node.py의 같은 날짜 주석 참고.
YOLO_MODEL_PATH = "/grippers/models/best.pt"
YOLO_CAPTURE_CONF = 0.25  # perception_node의 CONF_THRESHOLD(0.45)보다 낮게 —
                          # "왜 못 잡았나"를 보려면 탈락한 약한 검출도 보여야 한다
RGB_WAIT_TIMEOUT_S = 5.0  # 1단계 캡처 전 프레임 대기 상한

APPROACH_SPEED_MPS = 0.06  # 오늘 forward_manual.py에서 검증된 속도
# ⚠️ 2026-08-24 실기: 원래 0.05였는데(apply_axis_floor의 min_speed와 동일 —
# **데드밴드 경계**) 4단계에서 바퀴가 한 번도 안 돌았다. 키 입력은 정상이었고
# (c=정지가 먹혀 단계가 끝났다) cmd_vel도 나갔지만 0.05m/s가 정지마찰을 못
# 이겼다. 그런데 /odom_raw는 명령을 적분할 뿐이라 0.957m 이동했다고 보고했다
# — 회전 쪽 정지마찰 문제(HANDOFF §3-4)와 같은 함정이 직진에도 있다.
# 2단계에서 실제로 움직인 APPROACH_SPEED_MPS(0.06)로 올린다. 더 느리게 가야
# 하면 속도를 낮추지 말고 **짧은 버스트 + 정지**를 반복할 것 —
# 데드밴드 아래 속도는 아무리 오래 줘도 안 움직인다.
FINE_SPEED_MPS = APPROACH_SPEED_MPS
TURN_BIAS_RAD_S = 0.15  # a/d 보조 회전 — 전진과 결합된 회전이라 순수 회전
                          # 정지마찰 문제(오늘 실기 확인)와 무관하게 작동해야 함
TICK_S = 0.05  # cmd_vel 발행 주기 (20Hz)


# --- 구조화 로그(사람이 읽는 용도가 아니라, 나중에 분석하기 위한 기록) ----


def _json_default(value):
    """numpy 스칼라·배열을 파이썬 기본형으로 바꾼다.

    ROS 메시지의 고정 길이 배열 필드는 numpy dtype(int32/float32)으로
    들어온다. list()로 감싸도 **원소는 그대로 numpy 스칼라**라서
    json.dumps가 "Object of type int32 is not JSON serializable"로 죽는다 —
    2026-08-25 pose_verify_cycle 첫 실행이 정확히 여기서 끊겼다.

    호출부마다 int()/float()를 뿌리는 대신 여기서 한 번 막는다. RunLog를
    쓰는 도구가 여럿이고, 새 ROS 필드를 로그에 넣을 때마다 같은 함정을
    다시 밟게 되기 때문이다."""
    item = getattr(value, "item", None)
    if item is not None:
        try:
            return item()  # numpy 스칼라, 그리고 크기 1인 배열
        except (TypeError, ValueError):
            pass
    tolist = getattr(value, "tolist", None)
    if tolist is not None:
        return tolist()  # numpy 배열
    raise TypeError(f"JSON으로 바꿀 수 없는 값: {type(value).__name__}")


class RunLog:
    """터미널 출력과 별개로, 측정값을 JSON Lines 한 줄씩 파일에 남긴다.
    사용자가 직접 읽으라는 로그가 아니라 — 실기 세션 뒤에 이 파일 하나만
    넘기면 각 단계의 관측값·거리·면적·load를 그대로 다시 분석할 수 있게
    하려는 목적이다. 한 이벤트당 한 줄이라 실행 중간에 죽어도(Ctrl+C,
    크래시) 그 앞까지는 그대로 유효하다."""

    def __init__(self, raw_cls: str, profile: str):
        self.path = f"/tmp/grasp_test_log_{int(time.time())}.jsonl"
        self._f = open(self.path, "a", encoding="utf-8")
        self.log("run_start", raw_cls=raw_cls, profile=profile)

    def log(self, event: str, **fields):
        record = {"t": round(time.time(), 3), "event": event, **fields}
        self._f.write(json.dumps(record, ensure_ascii=False, default=_json_default) + "\n")
        self._f.flush()  # 매 줄 flush — 비정상 종료에도 그때까지는 남는다

    def close(self):
        self._f.close()


# --- 터미널 raw 모드 키 입력 -------------------------------------------


class KeyReader:
    """터미널을 cbreak 모드로 바꿔 Enter 없이 한 글자씩 논블로킹으로 읽는다.
    `with KeyReader() as kr:` 로 쓰고, 빠져나오면 원래 터미널 설정을 복원한다."""

    def __enter__(self):
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def ensure_cbreak(self):
        """cbreak 모드를 다시 걸고, 쌓여 있던 입력을 버린다.

        ⚠️ 2026-08-24 실기: 1단계에 YOLO 캡처(ultralytics import + 추론)를
        넣은 뒤부터 2·4단계에서 space가 전혀 안 먹혔다(odom 0.000m — 명령
        자체가 안 나갔다). ultralytics는 import·초기화 과정에서 여러
        서브프로세스(git/pip 확인 등)를 띄우는데, 그 자식이 stdin을 물려받아
        터미널 속성을 되돌려 놓으면 이 콘솔의 cbreak가 풀린다. 원인을
        하나로 특정하는 대신, **주행 루프에 들어가기 직전 항상 다시 건다** —
        비용이 사실상 0이고 어떤 경로로 풀렸든 복구된다.

        tcflush로 버퍼도 비운다 — 모드가 풀린 동안 눌린 키가 쌓여 있으면
        주행 시작하자마자 엉뚱한 명령(예: 즉시 정지)으로 소비된다."""
        tty.setcbreak(self._fd)
        termios.tcflush(self._fd, termios.TCIFLUSH)

    def cbreak_ok(self) -> bool:
        """cbreak가 지금도 걸려 있는지만 확인한다 — tcflush 등 부수효과가
        없어 주행 루프 안에서 매 틱 불러도 안전하다(ensure_cbreak과 달리
        키 버퍼를 비우지 않으므로, 눌러 둔 키를 지우지 않는다).

        2026-09-01 재발: ultralytics 지연 import를 없앤 뒤에도(load_yolo_model
        참고) 4단계(drive_phase) 도중 키가 다시 안 먹힌 사례가 나왔다 —
        이번엔 진입 시점이 아니라 **루프가 도는 중간**에 풀린 것으로 보인다.
        어느 경로로 풀렸는지 이 파일 안에서는 아직 특정하지 못했다(그 시점에
        이 스크립트가 서브프로세스를 새로 띄우지 않는다 — GripperCam도
        2026-09-01부터 이 콘솔은 안 씀). ensure_cbreak()의 방침(진입 시
        무조건 다시 건다)을 루프 내부로 넓혀, 원인을 특정하지 못해도 매 틱
        스스로 고치게 한다."""
        current = termios.tcgetattr(self._fd)
        return not (current[tty.LFLAG] & termios.ICANON)

    def getch_nonblocking(self) -> str | None:
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if not r:
            return None
        return sys.stdin.read(1)

    def wait_enter(self, prompt: str):
        """Enter 키 대기 — 이 호출 동안만 cooked 모드로 잠깐 되돌린다(입력
        에코·줄 단위 편집이 필요해서다). q 입력 시 종료 신호로 KeyboardInterrupt.

        복귀도 ensure_cbreak()과 같은 방식(cbreak + tcflush)이다 — 예전에는
        `tty.setcbreak`만 하고 안 비웠는데, 그 사이 쌓인 입력이 있으면 다음
        단계에서 엉뚱한 키로 소비된다(ensure_cbreak 쪽 docstring과 같은 문제)."""
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
        try:
            line = input(prompt)
        finally:
            self.ensure_cbreak()
        if line.strip().lower() == "q":
            raise KeyboardInterrupt


# --- 그리퍼 캠: 공유 프레임 버퍼 + MJPEG 스트림 ---------------------------
#
# ⚠️ 이 콘솔(main()) 자신은 2026-09-01부터 아래 GripperCam을 더 이상 안
# 쓴다 — 3단계에서 이걸 넘겨받으려고 perception_node를 죽이던 이유(구
# confirm_grasp이 이 장치를 독점)가 그 자체로 죽은 코드였다(observe_target
# 기반으로 대체된 뒤 안 지워짐, perception_node.py 참고). 그래도 이 클래스와
# start_stream_server/restart_perception_node는 지우지 않는다 —
# auto_grasp_sequence.py·grasp_cycle.py·straight_approach_calibrate.py가
# 각자의 흐름에서 그대로 import해 쓴다.


class GripperCam:
    """`/dev/gripper_cam`을 한 번만 열고 백그라운드 스레드로 계속 최신
    프레임을 갱신한다. 면적 측정과 MJPEG 스트림이 이 하나의 캡처를
    공유한다 — 장치를 두 번 열면 경합·프레임 손상 위험이 있다.

    ⚠️ perception_node는 lazy하게 여는 게 아니라 `__init__` 시점에 confirm_grasp용
    기준(빈 그리퍼) 프레임을 찍으려고 이 장치를 무조건 열어서 계속 쥐고 있다
    (2026-08-23 실기 재확인 — `lsof /dev/video0`으로 perception_node가 fd를 들고
    있는 걸 직접 확인했다. 예전 세션 노트의 "confirm_grasp를 호출해야 lazy하게
    연다"는 가정은 틀렸다). 그래서 이 클래스를 만들기 전에 `main()`이 먼저
    perception_node를 죽인다 — observe_target은 1~2단계에서만 쓰고 그 뒤로는
    안 쓰므로 안전하다.

    ⚠️ 2026-09-01: 위 문단은 이제 이 콘솔 자신에는 안 맞는다 — perception_node.py
    가 confirm_grasp의 그리퍼캠 경로를 통째로 제거해서(observe_target으로
    대체된 뒤 죽은 코드였다) 더 이상 이 장치를 열지 않는다. 이 클래스를 아직
    쓰는 다른 도구(auto_grasp_sequence.py 등)는 각자 자기 흐름에서 죽이고
    되살리는 책임을 진다 — 이 클래스 자체는 손대지 않는다."""

    def __init__(self, device="/dev/gripper_cam", width=640, height=480):
        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        for _ in range(8):  # 오늘 세션에서 확인한 웜업 관례
            self._cap.grab()
        self._lock = threading.Lock()
        self._frame = None
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            ok, frame = self._cap.read()
            if ok:
                with self._lock:
                    self._frame = frame
            time.sleep(0.03)

    def latest(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self._cap.release()

    def measure_area_px2(self) -> float | None:
        """오늘 세션에서 확정한 절차: 그레이스케일 임계 → 모폴로지 open/close
        → 최대 컨투어 면적. 흰 룩 대 원목 바닥/어두운 손가락 대비를 이용한다."""
        frame = self.latest()
        if frame is None:
            return None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        return float(cv2.contourArea(max(contours, key=cv2.contourArea)))


def _make_stream_handler(cam: GripperCam):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    frame = cam.latest()
                    if frame is not None:
                        ok, jpg = cv2.imencode(".jpg", frame)
                        if ok:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                            self.wfile.write(jpg.tobytes())
                            self.wfile.write(b"\r\n")
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass  # 접속 로그로 터미널이 지저분해지는 것을 막는다

    return Handler


def start_stream_server(cam: GripperCam) -> str:
    """MJPEG 스트림 서버를 백그라운드 스레드로 띄우고 접속 URL을 돌려준다.
    도커 컨테이너가 host 네트워크 모드가 아니면 포트를 게시해야 맥북에서
    닿는다 — 파일 상단 경고 참고."""
    import socket

    server = HTTPServer(("0.0.0.0", GRIPPER_STREAM_PORT), _make_stream_handler(cam))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        pi_ip = socket.gethostbyname(socket.gethostname())
    except OSError:
        pi_ip = "<파이 IP>"
    return f"http://{pi_ip}:{GRIPPER_STREAM_PORT}/"


# --- ROS2 노드 ------------------------------------------------------------


class GraspTestNode(Node):
    def __init__(self):
        super().__init__("grasp_test_console")
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom_raw", self._on_odom, 10)
        self._pose = None  # (x, y) m, odom_raw 원본 좌표계

        # 1단계 YOLO 캡처용 — depth_cam_rotate_node가 180도 보정해 내보내는
        # 스트림이다(원본 rgb0/image는 카메라가 거꾸로 달려 뒤집혀 나온다).
        # perception_node가 observe_target에 쓰는 것과 같은 토픽이라, 캡처
        # 이미지가 곧 그 판정이 본 화면이다.
        self._latest_rgb = None
        self.create_subscription(Image, "depth_cam/rgb/image_rotated", self._on_rgb, 10)

        self._observe_client = self.create_client(ObserveTarget, "perception/observe_target")
        self._gripper_client = self.create_client(SetGripper, "arm_driver/set_gripper")
        self._load_client = self.create_client(GetLoad, "arm_driver/get_load")
        self._arm_state_client = self.create_client(GetArmState, "arm_driver/get_arm_state")
        self._floor_pose_client = ActionClient(self, MoveToFloorPose, "arm_driver/move_to_floor_pose")

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pose = (p.x, p.y)

    def _on_rgb(self, msg: Image):
        self._latest_rgb = msg

    def pump(self):
        """구독 콜백(오도메트리)만 처리한다 — 서비스/액션 대기는 각 호출부에서
        spin_until_future_complete로 직접 한다. 이 스크립트는 유일한
        executor이므로(mission_orchestrator처럼 이미 도는 executor가 없다)
        _ros_call.py의 "중첩 executor 금지" 경고가 적용되지 않는다."""
        rclpy.spin_once(self, timeout_sec=0.0)

    def observe(self, raw_cls: str, timeout_sec=3.0):
        if not self._observe_client.wait_for_service(timeout_sec=timeout_sec):
            print("  [경고] perception/observe_target 서비스 없음")
            return None
        # force_fresh=True — 사용자가 키를 눌러 매번 독립적으로 부르는
        # 관측이라, 3초 캐시에 걸려 직전(다른 자세·다른 위치) 표본을
        # 돌려받으면 지금 화면과 다른 답이 나온다(2026-09-01, ObserveTarget.
        # srv force_fresh 필드 추가와 같은 이유).
        future = self._observe_client.call_async(
            ObserveTarget.Request(raw_cls=raw_cls, force_fresh=True))
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result() if future.done() else None

    def set_gripper(self, width_mm: float, timeout_sec=5.0):
        if not self._gripper_client.wait_for_service(timeout_sec=timeout_sec):
            print("  [경고] arm_driver/set_gripper 서비스 없음")
            return None
        future = self._gripper_client.call_async(SetGripper.Request(width_mm=width_mm))
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result() if future.done() else None

    def get_load(self, timeout_sec=3.0):
        if not self._load_client.wait_for_service(timeout_sec=timeout_sec):
            print("  [경고] arm_driver/get_load 서비스 없음")
            return None
        future = self._load_client.call_async(GetLoad.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result().load_ratio if future.done() else None

    def arm_state(self, timeout_sec=3.0):
        """servo 1..6의 위치·부하·온도·torque를 한 번에 읽는다.

        arm_driver_node가 /dev/soarm을 독점하므로 도구가 서보를 실측할 수
        있는 유일한 경로다 — driver_sdk로 직접 붙으면 팔 이동이 깨진다."""
        if not self._arm_state_client.wait_for_service(timeout_sec=timeout_sec):
            print("  [경고] arm_driver/get_arm_state 서비스 없음")
            return None
        future = self._arm_state_client.call_async(GetArmState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result() if future.done() else None

    def move_floor_pose(self, profile: str, stage: str, timeout_sec=30.0) -> bool:
        if not self._floor_pose_client.wait_for_server(timeout_sec=5.0):
            print("  [경고] arm_driver/move_to_floor_pose 액션 서버 없음")
            return False
        goal = MoveToFloorPose.Goal(profile=profile, stage=stage)
        goal_future = self._floor_pose_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=5.0)
        if not goal_future.done() or goal_future.result() is None or not goal_future.result().accepted:
            print(f"  [실패] {stage} 단계 거부됨(서보 과열/자세 조건 등 — arm.log 확인)")
            return False
        result_future = goal_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            print(f"  [실패] {stage} 단계 결과 없음(타임아웃)")
            return False
        reached = result_future.result().result.reached
        print(f"  move_to_floor_pose({profile}, {stage}) -> reached={reached}")
        return reached

    def stop(self):
        for _ in range(10):  # 0.5초 — Ctrl+C 시 잔여 발행을 확실히 밀어넣는다
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)


def estimate_position(obs: ObserveTarget.Response, raw_cls: str):
    """(전방 m, 좌/우 m, +면 우측) — 없으면 (None, None)."""
    if obs is None or not obs.found:
        return None, None
    k = K_CLASS.get(raw_cls)
    if k is None:
        print(f"  [경고] '{raw_cls}'는 거리 보정값(K_CLASS) 미실측 — 전방 거리 계산 불가")
        return None, None
    area_px2 = obs.h * obs.w
    if area_px2 <= 0:
        return None, None
    effective_px = math.sqrt(area_px2) - BBOX_PADDING_PX
    if effective_px <= 0:
        return None, None
    z_m = k / effective_px
    lateral_m = (obs.x - CX_PX) * z_m / FX_PX + LATERAL_BIAS_M
    return z_m, lateral_m


def _bgr_from_image_msg(msg: Image) -> np.ndarray:
    """Image -> BGR. cv_bridge를 쓰지 않는다 — 이 환경의 cv_bridge 확장이
    numpy 1.x ABI로 빌드돼 numpy 2.x와 충돌해 세그폴트를 낸다(2026-08-23
    실기 확인, perception_node.py의 같은 이름 함수와 같은 이유)."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("bgr8", "rgb8"):
        img = buf.reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1] if enc == "rgb8" else img
    if enc == "mono8":
        return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    raise ValueError(f"지원하지 않는 인코딩: {msg.encoding}")


def load_yolo_model():
    """YOLO 모델을 미리 로드한다 — `main()`이 `KeyReader`로 들어가기 **전에**
    불러야 한다(2026-09-01, 재발 방지).

    ⚠️ 2026-08-24 실기: 예전에는 이 로딩을 1단계 안(save_yolo_annotated)에서
    지연 import했는데, ultralytics는 import·초기화 과정에서 서브프로세스
    (git/pip 확인 등)를 띄우고 그 자식이 stdin을 물려받아 터미널 속성을
    되돌려 놓는다 — 그게 cbreak 모드였다면 풀려버린다. 처음엔 "주행 루프에
    들어가기 직전 항상 cbreak를 다시 건다"로 우회했지만(ensure_cbreak),
    2026-09-01 실기에서 같은 증상(키 입력 무반응)이 재발했다 — 우회가 모든
    경로를 못 덮는다는 뜻이다. 근본 원인(cbreak가 걸린 채로 ultralytics를
    부르는 것) 자체를 없앤다: cbreak가 걸리기 전, 즉 이 함수가 `with
    KeyReader()` 진입 전에 불리면 애초에 망가뜨릴 raw 모드가 없다.

    실패(모델 없음)는 예외를 올리지 않고 None을 돌려준다 — 1단계 YOLO 캡처는
    진단용 부가 기능이라 본 테스트를 막으면 안 된다."""
    try:
        from ultralytics import YOLO
    except ImportError:
        print("[준비] ultralytics 미설치 — 1단계 YOLO 캡처는 건너뜁니다")
        return None
    print("[준비] YOLO 모델 로딩 중...")
    model = YOLO(YOLO_MODEL_PATH)
    print("[준비] YOLO 모델 로딩 완료.")
    return model


_MODEL_NOT_PASSED = object()  # save_yolo_annotated의 model 기본값 — 아래 참고


def save_yolo_annotated(node: GraspTestNode, raw_cls: str, model=_MODEL_NOT_PASSED,
                         out_dir: str = YOLO_CAPTURE_DIR):
    """1단계 관측 시점의 RGB 프레임에 YOLO 검출을 그려 저장한다.

    관측(observe_target)이 왜 그 결과를 냈는지 눈으로 확인하기 위한 것이다 —
    특히 found=False일 때 "물체가 화면에 없었나 / 잡혔는데 신뢰도가 낮았나 /
    다른 클래스로 잡혔나"를 가른다. 그래서 임계값을 perception_node보다 낮게
    (YOLO_CAPTURE_CONF) 잡아 **탈락한 검출까지** 그린다. 목표 클래스는 초록,
    나머지는 회색으로 구분한다.

    `model`은 `load_yolo_model()`이 **`KeyReader` 진입 전에** 미리 로드해
    넘긴 것이어야 한다 — 왜 여기서 지연 import하면 안 되는지는
    load_yolo_model()의 docstring 참고(cbreak 중 ultralytics 로딩이 키
    입력을 죽였던 문제, 2026-09-01). `None`이면(ultralytics 미설치) 조용히
    건너뛴다.

    `model`을 아예 안 넘긴 옛 호출부(auto_grasp_sequence.py·grasp_cycle.py,
    아직 이 함수 안에서 지연 import하던 시절 그대로 쓴다)를 위해 그 경우만
    예전처럼 여기서 지연 import한다 — **이 경로는 여전히 같은 위험에
    노출돼 있다.** 그 두 도구에서도 같은 증상(키 입력 무반응)이 나오면
    load_yolo_model()을 각자의 KeyReader 진입 전으로 옮기고 여기도 필수
    인자로 좁힐 것."""
    if model is _MODEL_NOT_PASSED:
        try:
            from ultralytics import YOLO
        except ImportError:
            print("  [캡처] ultralytics 미설치 — YOLO 캡처 건너뜀")
            return None
        model = YOLO(YOLO_MODEL_PATH)
    elif model is None:
        return None
    # ⚠️ 이 콘솔은 상시 스핀하지 않는다 — 구독 콜백은 node.pump()나
    # spin_until_future_complete()가 도는 동안에만 처리된다. 1단계에서
    # observe_target 서비스가 없으면 그 경로마저 거의 안 돌아서 _latest_rgb가
    # None인 채로 여기 온다(2026-08-24 실기 확인). 프레임이 올 때까지 잠깐
    # 직접 펌프한다.
    deadline = time.time() + RGB_WAIT_TIMEOUT_S
    while node._latest_rgb is None and time.time() < deadline:
        node.pump()
        time.sleep(0.05)
    if node._latest_rgb is None:
        print(f"  [캡처] {RGB_WAIT_TIMEOUT_S}s 안에 RGB 프레임이 안 옴 — "
              "depth_cam_rotate_node / ascamera_node가 떠 있는지 확인할 것")
        return None
    try:
        frame = _bgr_from_image_msg(node._latest_rgb).copy()
        result = model.predict(frame, verbose=False, conf=YOLO_CAPTURE_CONF)[0]
    except Exception as exc:  # noqa: BLE001 -- 진단 기능이 테스트를 막지 않는다
        print(f"  [캡처] YOLO 실행 실패({exc}) — 건너뜀")
        return None

    lines = []
    for box in result.boxes:
        conf = float(box.conf[0])
        name = result.names[int(box.cls[0])]
        x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
        is_target = name == raw_cls
        color = (0, 255, 0) if is_target else (160, 160, 160)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, f"{name} {conf:.2f}", (x1, max(14, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        lines.append(f"{name} conf={conf:.2f} bbox=({x1},{y1},{x2},{y2}) h={y2-y1} w={x2-x1}")

    if not lines:
        cv2.putText(frame, "NO DETECTION", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    import os
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"step1_{raw_cls}_{time.strftime('%Y%m%d_%H%M%S')}.jpg")
    cv2.imwrite(path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    print(f"  [캡처] YOLO 적용 이미지 저장: {path}")
    for line in lines:
        print(f"    - {line}")
    if not lines:
        print(f"    - 검출 0건 (conf {YOLO_CAPTURE_CONF} 기준)")
    return {"path": path, "detections": lines}


def print_position(node: GraspTestNode, raw_cls: str, label: str) -> dict:
    """관측·출력하고, RunLog에 그대로 넣을 수 있는 dict로 돌려준다."""
    obs = node.observe(raw_cls)
    if obs is None or not obs.found:
        # 2026-09-01: observe_target이 왜 못 찾았는지(obs.reason)를 이제
        # 돌려준다 — 신뢰도 미달인지, 화면 위치(파지 거리 아님) 게이트
        # 탈락인지, 다중 프레임 합의 미달인지, 애초에 검출 자체가 없는지를
        # 구분해서 보여준다. 예전엔 전부 "물체를 못 찾음"으로만 찍혀서,
        # YOLO는 conf 0.97로 정확히 잡았는데 위치 게이트에 걸린 걸 bbox를
        # 직접 대조해서야 알아낸 적이 있다(같은 날 실기).
        reason = getattr(obs, "reason", "") if obs is not None else "관측 서비스 응답 없음"
        suffix = f" — {reason}" if reason else ""
        print(f"  [{label}] 물체를 못 찾음{suffix}")
        return {"found": False, "reason": reason}
    z_m, lateral_m = estimate_position(obs, raw_cls)
    info = {"found": True, "x": obs.x, "h": obs.h, "w": obs.w}
    if z_m is None:
        print(f"  [{label}] x={obs.x:.1f} h={obs.h:.1f} w={obs.w:.1f} (거리 계산 불가)")
        return info
    side = "우측" if lateral_m >= 0 else "좌측"
    print(f"  [{label}] 전방 {z_m*100:.1f}cm, {side} {abs(lateral_m)*100:.1f}cm "
          f"(x={obs.x:.1f} h={obs.h:.1f} w={obs.w:.1f})")
    info.update(forward_m=z_m, lateral_m=lateral_m)
    return info


# --- 주행 루프(공용) -------------------------------------------------------


def drive_phase(
    node: GraspTestNode,
    kr: KeyReader,
    *,
    keymap: dict,
    speed: float,
    report=None,
    legend=None,
):
    """`c`가 눌릴 때까지 `keymap`에 정의된 키로 cmd_vel을 발행한다.
    반환값: (odom 시작좌표, odom 종료좌표) — 둘 다 None이면 오도메트리 미수신.

    `report`는 1초에 한 번 불리는 인자 없는 콜백이다 — 운전하는 사람에게
    "언제 c를 누를지"를 알려주는 용도로, 무엇을 보여줄지는 호출부가 정한다
    (depth 거리는 demo_rook_run.py 참고)."""
    kr.ensure_cbreak()  # 위 ensure_cbreak docstring 참고 — 키가 안 먹는 사고 방지
    if legend is None:
        legend = ("  [space]/[a]/[d] 전진, [c] 정지" if "w" not in keymap else
                  "  [w]전진 [s]후진 [a]전진+좌회전 [d]전진+우회전, [c] 정지")
    print(legend)
    node.pump()
    start_pose = node._pose
    linear_x, angular_z = 0.0, 0.0
    last_report_t = 0.0
    while True:
        node.pump()
        if not kr.cbreak_ok():  # 매 틱 검사 — cbreak_ok() 문서 참고(2026-09-01 재발)
            kr.ensure_cbreak()
        key = kr.getch_nonblocking()
        if key is not None:
            key = key.lower()
            if key in keymap:
                linear_x, angular_z = keymap[key](speed)
            if key == "c":
                break
        t = Twist()
        t.linear.x = linear_x
        t.angular.z = angular_z
        node.cmd_pub.publish(t)

        if report is not None and time.time() - last_report_t >= 1.0:
            report()
            # report()가 서비스 호출로 수백 ms를 쓸 수 있다 — 그동안 cmd_vel이
            # 끊기면 base 드라이버 워치독이 차를 세워 주행이 툭툭 끊긴다.
            # 돌아오자마자 같은 명령을 다시 밀어 공백을 최소화한다.
            node.cmd_pub.publish(t)
            last_report_t = time.time()

        time.sleep(TICK_S)

    node.stop()
    end_pose = node._pose
    return start_pose, end_pose


def odom_distance_m(start_pose, end_pose):
    if start_pose is None or end_pose is None:
        return None
    dx = end_pose[0] - start_pose[0]
    dy = end_pose[1] - start_pose[1]
    return math.hypot(dx, dy)


SPACE_KEYMAP = {
    " ": lambda v: (v, 0.0),
    "a": lambda v: (v, TURN_BIAS_RAD_S),
    "d": lambda v: (v, -TURN_BIAS_RAD_S),
}

WASD_KEYMAP = {
    "w": lambda v: (v, 0.0),
    "s": lambda v: (-v, 0.0),
    "a": lambda v: (v, TURN_BIAS_RAD_S),
    "d": lambda v: (v, -TURN_BIAS_RAD_S),
}


# --- 메인 시퀀스 -----------------------------------------------------------


def restart_perception_node() -> bool:
    """`GripperCam`을 쓰려고 죽였던 perception_node를 다시 띄운다.

    ⚠️ 이 콘솔(main()) 자신은 2026-09-01부터 이 함수를 안 부른다 — perception_node를
    죽일 이유(그리퍼캠 확보) 자체가 없어졌다. auto_grasp_sequence.py·
    grasp_cycle.py는 여전히 이 함수를 쓴다 — 죽인 쪽이 되살릴 책임을 진다는
    원칙은 그대로다.

    카메라를 놓아준 뒤(GripperCam.close) 불러야 한다 — 안 그러면
    perception_node가 기동 시 장치를 못 열고 바로 죽는다."""
    ros_setup = "/ros2_ws/install/setup.bash"
    if not os.path.exists(ros_setup):
        print(f"  [경고] {ros_setup}이 없어 perception_node를 되살리지 못했습니다")
        return False
    print("  perception_node 재기동 중...")
    subprocess.Popen(
        ["setsid", "bash", "-lc",
         f"source {ros_setup} && exec ros2 run grippers_perception perception_node "
         "> /tmp/perception.log 2>&1"],
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    # 모델 로드(ultralytics)까지 시간이 걸린다 — 떴는지 확인될 때까지 기다린다.
    for _ in range(int(PERCEPTION_RESTART_TIMEOUT_S / 0.5)):
        time.sleep(0.5)
        if subprocess.run(["pgrep", "-f", "grippers_perception/perception_node"],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
            print("  perception_node 재기동 완료.")
            return True
    print("  [경고] perception_node가 다시 뜨지 않았습니다 — /tmp/perception.log 확인")
    return False


def recover_to_idle(node: "GraspTestNode", profile: str, log: "RunLog", why: str) -> bool:
    """실패로 중단할 때 팔을 IDLE로 되돌린다(사용자 요청, 2026-08-24).

    "idle"이 아니라 "recover_idle" 단계를 쓴다 — 이동이 실패하면 팔은 정의상
    등록된 자세들 **사이**에 멈춰 서는데, 바로 그 상태가 "idle"의 시작 자세
    게이트에 걸려 거부되기 때문이다(arm_driver_node._move_floor_stage의
    recover_idle 주석 참고). 즉 정작 복구가 필요한 순간에만 복구가 막힌다.

    복구 자체가 실패해도 예외를 올리지 않는다 — 이건 이미 실패한 경로를
    수습하는 중이라, 여기서 또 터지면 원래 실패 원인이 로그에서 묻힌다.
    대신 사람이 손으로 처리하도록 분명히 알린다."""
    print(f"  [복구] {why} — 팔을 IDLE로 되돌립니다...")
    try:
        # profile은 recover_idle에서 안 쓰이지만(IDLE로만 간다) 액션이
        # 유효한 이름을 요구한다.
        ok = node.move_floor_pose(profile, "recover_idle")
    except Exception as e:  # 복구 경로는 절대 원래 실패를 덮지 않는다
        print(f"  [복구 실패] {e}")
        ok = False
    log.log("recover_to_idle", ok=ok, why=why)
    if ok:
        print("  [복구] IDLE 복귀 완료.")
    else:
        print("  [복구 실패] 팔이 중간 자세에 멈춰 있습니다 — "
              "arm_driver를 끄고 tools/align_to_idle.py로 직접 정렬하세요.")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument(
        "--raw-cls", default="rook", choices=sorted(CLASS_TO_PROFILE),
        help="perception/observe_target에 보낼 클래스명 (기본값 rook — 오늘 테스트 대상)",
    )
    ap.add_argument(
        "--profile", default=None,
        help="arm_driver/move_to_floor_pose 프로파일명 (생략 시 --raw-cls로 자동 유도)",
    )
    args = ap.parse_args()
    profile = args.profile or CLASS_TO_PROFILE[args.raw_cls]
    close_width_mm = FLOOR_GRASP_PROFILES[profile].close_width_mm
    print(f"대상 클래스: raw_cls={args.raw_cls}  profile={profile}  close_width={close_width_mm}mm")

    log = RunLog(args.raw_cls, profile)
    print(f"분석용 로그 파일: {log.path}  (끝나면 이 파일을 Claude에게 넘길 것)")

    # 2026-09-01부터 이 콘솔은 3단계에서 perception_node를 죽이지 않는다
    # (구 confirm_grasp의 그리퍼캠 독점이 죽은 코드였다 — perception_node.py
    # 2026-09-01 정리 참고). 그래도 다른 이유로 안 떠 있을 수 있으니 확인은
    # 남겨둔다.
    if subprocess.run(
        ["pgrep", "-f", "grippers_perception/perception_node"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    ).returncode != 0:
        print(
            "\n⚠️  perception_node가 떠 있지 않습니다 — 1·2단계 관측과 YOLO 캡처가\n"
            "    전부 실패합니다. 다른 터미널에서 먼저 띄우고 다시 실행하세요:\n"
            "        ros2 run grippers_perception perception_node > /tmp/perception.log 2>&1 &\n"
        )

    # KeyReader(cbreak 모드) 진입 전에 미리 로드한다 — load_yolo_model()
    # docstring 참고(cbreak 중 ultralytics 로딩이 키 입력을 죽였던 문제,
    # 2026-09-01 재발 후 근본 수정).
    yolo_model = load_yolo_model()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GraspTestNode()
    try:
        with KeyReader() as kr:
            # 1단계 -----------------------------------------------------
            kr.wait_enter("\n[1단계] 차량·룩 배치 완료 후 Enter (종료하려면 q+Enter): ")
            log.log("step1_observe", **print_position(node, args.raw_cls, "1단계 관측"))
            # 관측 결과와 같은 프레임에 YOLO를 그려 남긴다 — found=False일 때
            # 원인을 눈으로 가르기 위한 진단용(save_yolo_annotated 참고).
            capture = save_yolo_annotated(node, args.raw_cls, yolo_model)
            if capture is not None:
                log.log("step1_yolo_capture", **capture)

            # 2단계(정렬 주행)는 2026-08-24 사용자 지시로 제거했다 — 물체를
            # 사람이 직접 원하는 위치에 놓고 시작하므로 여기서 다시 정렬할
            # 이유가 없고, 실기에서 이 구간이 매번 문제만 됐다(데드밴드 아래
            # 속도로 안 움직이거나, odom이 안 움직인 이동을 보고하거나).
            # 단계 번호는 로그 호환을 위해 그대로 둔다(step3~7).

            # 3단계 -----------------------------------------------------
            kr.wait_enter("\n[3단계] g + Enter로 GRASP 진입 (파지 전 자세로 이동): ")
            # _move_floor_stage는 팔 관절(servo 1-5)만 움직인다 — 그리퍼(servo 6)는
            # 별개라 여기서 직접 열어야 한다.
            #
            # ⚠️ 여는 시점이 중요하다(사용자 지시, 2026-08-24). 예전에는
            # safe -> grasp로 다 내려간 **뒤에** 열었는데, 그러면 닫힌 손가락이
            # 물체가 있는 공간을 그대로 통과해 내려가면서 물체를 밀어낸다.
            # 열어 놓고 내려가면 손가락이 물체 양옆으로 비켜 지나간다.
            #
            # IDLE이 아니라 safe에서 여는 이유: IDLE은 팔이 차체 위에 접힌
            # 자세라 거기서 168mm까지 벌리면 손가락이 차체에 닿을 수 있는데
            # 아직 실측으로 확인된 적이 없다. safe는 차체 전면 185mm 앞·145mm
            # 높이라 사방이 비어 있고, "내려가기 전에 연다"는 목적은 여기서
            # 이미 달성된다.
            preopen_mm = FLOOR_GRASP_PROFILES[profile].preopen_width_mm
            ok = node.move_floor_pose(profile, "safe")
            if ok:
                gripper_resp = node.set_gripper(preopen_mm)
                log.log("step3_gripper_open", ok=bool(gripper_resp and gripper_resp.ok),
                        width_mm=preopen_mm)
                print(f"  그리퍼 열림({preopen_mm}mm) — 내려가기 전에 연다")
                ok = node.move_floor_pose(profile, "grasp")
            log.log("step3_grasp_entry", ok=ok)
            if not ok:
                print("  GRASP 진입 실패 — 서보 온도/자세 조건을 arm.log에서 확인 후 재시도할 것")
                recover_to_idle(node, profile, log, "GRASP 진입 실패")
                return
            # 그리퍼캠(/dev/gripper_cam)을 넘겨받으려고 perception_node를 죽이던
            # 자리였다 — 그 목적이던 confirm_grasp의 그리퍼캠 경로 자체가
            # 죽은 코드였다(2026-08-26에 observe_target 기반으로 대체된 뒤
            # 안 지워짐, grippers_perception/perception_node.py 2026-09-01
            # 정리 참고). perception_node는 더 이상 이 장치를 열지 않으므로
            # 죽일 이유가 없다 — 그대로 두고 observe_target도 계속 쓸 수 있다.

            # 4단계 -----------------------------------------------------
            print("\n[4단계] Space로 미세 전진")
            start4, end4 = drive_phase(node, kr, keymap=SPACE_KEYMAP, speed=FINE_SPEED_MPS)
            d4 = odom_distance_m(start4, end4)
            print(f"  정지. 이번 구간 이동거리(odom_raw)={'%.3fm' % d4 if d4 is not None else '측정 불가'}"
                  f"  ⚠️ odom은 명령 적분값이라 실제 이동의 증거가 아니다 — 눈으로 확인할 것")
            log.log("step4_stop", distance_m=d4)

            # 5단계 -----------------------------------------------------
            kr.wait_enter("\n[5단계] g + Enter로 파지(닫기 후 들어올리기): ")
            resp = node.set_gripper(close_width_mm)  # floor_grasp_profiles[profile].close_width_mm
            if resp is None or not resp.ok:
                print("  그리퍼 닫기 실패")
                log.log("step5_close", ok=False)
                recover_to_idle(node, profile, log, "그리퍼 닫기 실패")
                return
            print(f"  닫힘(폭 {close_width_mm}mm). load_ratio={resp.load_ratio:.4f} (기준 {LOAD_THRESHOLD})")
            log.log("step5_close", ok=True, close_width_mm=close_width_mm,
                    load_ratio=resp.load_ratio)
            if resp.load_ratio < LOAD_THRESHOLD:
                print(f"  [경고] 닫힘 부하가 기준({LOAD_THRESHOLD}) 미만 — 빈 채로 닫혔을 수 있다")
            # ⚠️ 2026-08-24 실기: 여기서 곧바로 들어올려 닫힘과 상승이 겹쳤다.
            # arm_driver의 set_gripper가 위치 정지까지 기다리도록 고쳤지만
            # (_wait_gripper_motion_settled), 실제로 물렸는지는 사람이 보는 게
            # 가장 확실하다 — 들어올리기 전에 한 번 끊는다.
            kr.wait_enter("  그리퍼가 완전히 닫혔는지 확인 후 Enter로 들어올리기 (q로 중단): ")
            if not node.move_floor_pose(profile, "midpoint"):
                print("  들어올리기(midpoint) 실패")
                log.log("step5_midpoint", ok=False)
                recover_to_idle(node, profile, log, "들어올리기 실패")
                return
            mid_load = node.get_load()
            print(f"  midpoint load_ratio={mid_load:.4f}" if mid_load is not None else "  load 확인 실패")
            if mid_load is not None and mid_load < LOAD_THRESHOLD:
                print("  [경고] 부하가 기준 미만 — 파지 실패(미끄러짐) 가능성")
            log.log("step5_midpoint", ok=True, load_ratio=mid_load)

            # 6단계 -----------------------------------------------------
            ok = node.move_floor_pose(profile, "safe") and node.move_floor_pose(profile, "idle")
            log.log("step6_carry_idle", ok=ok)
            if not ok:
                print("  CARRY_IDLE 복귀 실패 — 수동으로 상태 확인할 것")
                recover_to_idle(node, profile, log, "CARRY_IDLE 복귀 실패")
                return
            print("\n[6단계] CARRY_IDLE 도달. w/a/s/d로 바구니까지 주행, c로 정지")
            print("  (참고: 오늘 실기에서 순수 제자리 회전은 작동하지 않았다 — a/d는 전진과 결합된 완만한 회전이다)")
            start6, end6 = drive_phase(node, kr, keymap=WASD_KEYMAP, speed=APPROACH_SPEED_MPS)
            log.log("step6_stop", distance_m=odom_distance_m(start6, end6))

            # 7단계 -----------------------------------------------------
            kr.wait_enter("\n[7단계] 바구니 위치 확인했으면 Enter로 투하 실행: ")
            # domain/task/states.py InsertState와 동일한 순서(drop -> 그리퍼 열기 -> idle).
            if not node.move_floor_pose(profile, "drop"):
                print("  drop 자세 실패 — 수동으로 상태 확인할 것")
                log.log("step7_drop", ok=False)
                recover_to_idle(node, profile, log, "drop 자세 실패")
                return
            node.set_gripper(168.0)  # domain/task/states.py OPEN_MM
            node.move_floor_pose(profile, "idle")
            log.log("step7_drop", ok=True)
            # 물체를 놓았으니 벌어진 채로 두지 않는다 — align_to_idle.py가 IDLE의
            # 정상 상태로 규정하는 완전 닫힘(GRIPPER_CLOSED_MM)으로 되돌린다.
            close_resp = node.set_gripper(GRIPPER_CLOSED_MM)
            log.log("step7_gripper_close", ok=bool(close_resp and close_resp.ok))
            print(f"\n완료 — IDLE 복귀, 그리퍼 닫음({GRIPPER_CLOSED_MM}mm).")

    except KeyboardInterrupt:
        print("\n[중단] 정지 명령 발행 중...")
        node.stop()
        print("정지 완료. 이후 로봇 상태는 직접 확인할 것(자동 복구 없음).")
        log.log("aborted")
    finally:
        log.log("run_end")
        log.close()
        print(f"\n분석용 로그: {log.path}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
