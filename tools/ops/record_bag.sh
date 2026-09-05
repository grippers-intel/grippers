#!/bin/bash
# grippers 실기 녹화 — RGB + 소용량 토픽만(포트폴리오/기록용, 2026-09-03).
#
# 여러 줄을 새로 띈 대화형 셸에 한 번에 붙여넣다가 앞부분(mkdir/cd)이
# 씹혀서 그 어디에도 bag이 안 만들어진 사고가 있었다(2026-09-03) — 그래서
# 스크립트 하나로 묶는다. 또한 /mission/state 는 grippers_interfaces 전용
# 메시지 타입이라 /ros2_ws/install/setup.bash 도 같이 소싱해야 녹화된다
# (안 하면 경고만 찍고 그 토픽만 조용히 빠진다 — 처음 시도 때 이걸 놓쳤다).
#
# depth/pointcloud 는 뺀다(사용자 규칙, 58GB 디스크를 순식간에 채운다).

set -eo pipefail

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

mkdir -p ~/bags
cd ~/bags

OUT="mission_$(date +%Y%m%d_%H%M%S)"
echo "녹화 시작 -> ~/bags/$OUT (Ctrl+C로 멈추세요)"

ros2 bag record -o "$OUT" \
  /ascamera/camera_publisher/rgb0/image \
  /depth_cam/rgb/image_rotated \
  /mission/state \
  /mission/emergency_stop \
  /odom_raw \
  /scan_raw \
  /ros_robot_controller/imu_raw \
  /ros_robot_controller/battery \
  /joint_states \
  /controller_manager/joint_states \
  /cmd_vel \
  /controller/cmd_vel \
  /tf \
  /tf_static
