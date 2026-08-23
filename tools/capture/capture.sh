#!/usr/bin/env bash
# 뎁스카메라 RGB 프레임 촬영 — 드라이버가 꺼져 있으면 알아서 띄운다.
#
#   ./capture.sh empty1          12초 촬영
#   ./capture.sh a_near 20       20초 촬영
#
# 이 카메라는 OpenCV로 직접 열면 안 된다. /dev/depth_cam 을 그냥 열면 RGB와 뎁스가
# 한 프레임에 쌓인 YUYV 1280x1040 원시 스트림이 잡혀 초록/보라 띠만 찍힌다.
# 제대로 된 RGB는 ascamera 드라이버의 토픽으로만 나온다.
set -euo pipefail

LABEL="${1:-}"; DUR="${2:-12}"
[ -n "$LABEL" ] || { echo "사용법: $0 <라벨> [촬영초]" >&2; exit 2; }

# ascamera 는 /ros2_ws 가 아니라 third_party_ws 에 있다. 이걸 빼먹으면
# "package 'ascamera' not found" 로 죽는다.
ENV_SETUP='source /opt/ros/humble/setup.bash
           source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash
           source /ros2_ws/install/setup.bash
           export ROS_DOMAIN_ID=21 need_compile=False DEPTH_CAMERA_TYPE=ascamera'

if ! docker exec IntelPi pgrep -f ascamera_node >/dev/null 2>&1; then
  echo "▶ 뎁스카메라 드라이버 기동"
  docker exec -d IntelPi bash -lc "$ENV_SETUP
      ros2 launch peripherals depth_camera.launch.py > /tmp/depthcam.log 2>&1"
  for i in $(seq 1 20); do
    docker exec IntelPi pgrep -f ascamera_node >/dev/null 2>&1 && { echo "  ✓ 준비됨"; sleep 3; break; }
    [ "$i" -eq 20 ] && { echo "✗ 드라이버가 안 뜹니다:"; docker exec IntelPi tail -5 /tmp/depthcam.log; exit 1; }
    sleep 1
  done
else
  echo "▶ 드라이버 이미 동작 중"
fi

docker exec IntelPi bash -lc "$ENV_SETUP
    python3 -u /grippers/tools/capture/capture_ros.py --duration $DUR --label '$LABEL'" \
  2>&1 | grep --line-buffered -E '^\[캡처\]'
