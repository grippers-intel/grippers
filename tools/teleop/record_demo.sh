#!/usr/bin/env bash
# 시연 rosbag2 녹화. 컨테이너 안에서 실행한다.
#
# 루트 파티션 여유가 크지 않으므로(확인 시점 3.9G) 기본 토픽 집합에서
# 원본 이미지를 뺐다. 카메라를 남기려면 --with-camera 를 주되, 남은 용량을
# 먼저 확인할 것 — 640x480 RGB 30fps 원본은 분당 1.5GB 를 넘는다.
set -euo pipefail

OUT_DIR="${OUT_DIR:-/grippers/recordings}"
WITH_CAMERA=0
MIN_FREE_MB=2048

for a in "$@"; do
  case "$a" in
    --with-camera) WITH_CAMERA=1 ;;
    --help|-h) sed -n '2,10p' "$0"; exit 0 ;;
    *) echo "알 수 없는 인자: $a" >&2; exit 2 ;;
  esac
done

source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"

free_mb=$(df -Pm "$(dirname "$OUT_DIR")" | awk 'NR==2{print $4}')
if [ "$free_mb" -lt "$MIN_FREE_MB" ]; then
  echo "여유 공간 ${free_mb}MB — ${MIN_FREE_MB}MB 미만이라 중단합니다." >&2
  echo "정리 후 다시 시도하세요: docker exec IntelPi apt-get clean" >&2
  exit 1
fi

TOPICS=(
  /cmd_vel /odom /tf /tf_static /scan
  /teleop/leader_counts /teleop/follower_counts
  /teleop/engaged /teleop/arm_joint_states
)
if [ "$WITH_CAMERA" -eq 1 ]; then
  # 존재하는 것만 넣는다 — 없는 토픽을 주면 ros2 bag 이 그냥 조용히 비운다.
  for t in $(ros2 topic list 2>/dev/null | grep -E 'image_raw/compressed|camera_info'); do
    TOPICS+=("$t")
  done
fi

mkdir -p "$OUT_DIR"
BAG="$OUT_DIR/demo_$(date +%Y%m%d_%H%M%S)"
echo "녹화 → $BAG"
echo "토픽: ${TOPICS[*]}"
echo "여유: ${free_mb}MB   (Ctrl-C 로 종료)"
exec ros2 bag record -o "$BAG" "${TOPICS[@]}"
