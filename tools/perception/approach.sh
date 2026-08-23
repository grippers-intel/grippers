#!/usr/bin/env bash
# 파지 위치로 자동 접근 — 필요한 스택이 꺼져 있으면 알아서 띄운다.
#
#   ./approach.sh --dry-run              움직이지 않고 오차만 본다
#   ./approach.sh                        실제 접근
#   ./approach.sh --teach --note "..."   지금 위치를 기준값으로 저장
#
# 스택을 빼먹어 "왜 안 움직이지" 로 시간을 버리는 일이 반복됐다. 그래서 실행 전에
# 카메라와 모터 드라이버를 여기서 확인한다. 배터리 전압도 같이 찍는다 — 전압이
# 낮으면 명령은 정상인데 바퀴만 안 도는, 가장 헷갈리는 증상이 나온다.
set -uo pipefail

ENV_SETUP='source /opt/ros/humble/setup.bash
           source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash
           source /ros2_ws/install/setup.bash
           export ROS_DOMAIN_ID=21 need_compile=False DEPTH_CAMERA_TYPE=ascamera'

ensure() {                       # ensure <프로세스패턴> <설명> <런치명령> <로그>
  local pat="$1" label="$2" cmd="$3" log="$4"
  if docker exec IntelPi pgrep -f "$pat" >/dev/null 2>&1; then
    echo "▶ $label 이미 동작 중"; return 0
  fi
  echo "▶ $label 기동"
  docker exec -d IntelPi bash -lc "$ENV_SETUP
      $cmd > $log 2>&1"
  for i in $(seq 1 25); do
    docker exec IntelPi pgrep -f "$pat" >/dev/null 2>&1 && { echo "  ✓ 준비됨"; return 0; }
    sleep 1
  done
  echo "✗ $label 이 안 뜹니다:"; docker exec IntelPi tail -15 "$log"; exit 1
}

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

ensure ascamera_node        "뎁스카메라"   "ros2 launch peripherals depth_camera.launch.py" /tmp/depthcam.log
ensure_base
sleep 3

# 배터리 — 낮으면 바퀴가 안 돈다. 실패해도 진행은 막지 않는다.
V=$(docker exec IntelPi bash -lc "$ENV_SETUP
    timeout 6 ros2 topic echo /ros_robot_controller/battery --once 2>/dev/null" \
    | grep -oE '[0-9]+' | head -1)
if [ -n "${V:-}" ]; then
  echo "▶ 배터리 ${V} mV"
  # 실측: 6944 mV 에서 베이스 정지, 7181 mV 에서는 주행. 7.4V 급 팩으로 보인다.
  [ "$V" -lt 7400 ] && echo "  ⚠ 낮습니다 — 바퀴가 안 돌거나 중간에 멈출 수 있습니다."
else
  echo "▶ 배터리 전압을 못 읽었습니다 (진행합니다)"
fi
echo

docker exec IntelPi bash -lc "$ENV_SETUP
    python3 -u /grippers/tools/perception/approach.py $*"
