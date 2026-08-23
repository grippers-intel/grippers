#!/usr/bin/env bash
# 좌표 주행 — 스택이 꺼져 있으면 알아서 띄운다.
#
#   ./nav.sh --reset            지금 자리를 원점으로 (바닥 홈 표시에 놓고)
#   ./nav.sh --teach basket     지금 자리를 'basket' 으로 저장
#   ./nav.sh basket             그 자리로 주행
#   ./nav.sh --list             저장된 목적지 보기
#   ./nav.sh --probe            오도메트리 검사
set -uo pipefail

ENV_SETUP='source /opt/ros/humble/setup.bash
           source /ros2_ws/install/setup.bash
           export ROS_DOMAIN_ID=21 need_compile=False DEPTH_CAMERA_TYPE=ascamera'

ensure_base() {
  if docker exec IntelPi pgrep -f "lib/controller/odom_publisher" >/dev/null 2>&1; then
    echo "▶ 모터 드라이버 이미 동작 중"; return 0
  fi
  # 노드만 죽고 런치 껍데기가 남는 일이 있다(벤더 set_odom 버그). 둘 다 정리한다.
  docker exec IntelPi pkill -f ros_robot_controller >/dev/null 2>&1 || true
  docker exec IntelPi pkill -f "odom_publisher.launch" >/dev/null 2>&1 || true
  docker exec IntelPi pkill -f "lib/controller/odom_publisher" >/dev/null 2>&1 || true
  sleep 2
  echo "▶ 모터 드라이버 기동"
  docker exec -d IntelPi bash -lc "$ENV_SETUP
      ros2 launch controller odom_publisher.launch.py > /tmp/base.log 2>&1"
  for i in $(seq 1 30); do
    docker exec IntelPi pgrep -f "lib/controller/odom_publisher" >/dev/null 2>&1 && { echo "  ✓ 준비됨"; sleep 3; return 0; }
    sleep 1
  done
  echo "✗ 모터 드라이버가 안 뜹니다:"; docker exec IntelPi tail -15 /tmp/base.log; exit 1
}

ensure_base

docker exec IntelPi bash -lc "$ENV_SETUP
    python3 -u /grippers/tools/nav/goto.py $*"
