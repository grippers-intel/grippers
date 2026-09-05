#!/bin/bash
# grippers 실기 bringup 정지 — 개별 PID를 하나씩 찾아 kill하지 않는다.
# bringup_now.sh 가 setsid 로 띄운 launch 프로세스 그룹 전체에 SIGINT를
# 한 번 보내면, ros2 launch 자신의 정상 종료 경로(각 노드에 SIGINT ->
# 응답 없으면 SIGTERM)를 그대로 타서 base/odom/ekf/카메라/팔/인식까지
# 전부 한 번에 정리된다 — 이게 ros2 launch 를 끄는 정석 방법이다.
#
# 이 스크립트는 kill 을 실제로 하므로 **사용자가 직접 실행**해야 한다
# (모터 제어 프로세스에 영향을 줄 수 있는 명령은 Claude Code 가 대필하지
# 않는다는 이 프로젝트의 표준 원칙 — CHANGES_2026-09-02.md 참고).

set -euo pipefail

PGID_FILE=/tmp/bringup.pgid

if [ ! -f "$PGID_FILE" ]; then
  echo "$PGID_FILE 가 없습니다 — bringup_now.sh 로 띄운 게 아니면 이 스크립트로는 못 끕니다."
  echo "그 경우 ps -eo pid,cmd | grep -E 'ros_robot_controller|odom_publisher|ekf_node|joint_state_publisher|ascamera_node|arm_driver_node|perception_node|robot_state_publisher' 로 직접 찾아서 정리하세요."
  exit 1
fi

PGID=$(cat "$PGID_FILE")
echo "프로세스 그룹 $PGID 에 SIGINT 전송..."
kill -INT -- "-$PGID" 2>/dev/null || true

for i in $(seq 1 10); do
  if ! kill -0 -- "-$PGID" 2>/dev/null; then
    echo "정상 종료됨"
    rm -f "$PGID_FILE"
    exit 0
  fi
  sleep 1
done

echo "10초 안에 안 죽어서 SIGKILL 보냅니다"
kill -9 -- "-$PGID" 2>/dev/null || true
rm -f "$PGID_FILE"
