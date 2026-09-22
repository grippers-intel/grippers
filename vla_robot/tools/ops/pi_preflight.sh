#!/usr/bin/env bash
# 기동 전 점검 — Pi 에서 (컨테이너 밖/안 어디서든) 실행한다.
#
#   bash tools/ops/pi_preflight.sh
#
# 무엇을 보는가
#   1. ros_robot_controller 가 **정확히 1개**인가.
#      0 개  -> 부팅 자동 실행분이 죽었다. use_vendor_controller:=true 로 띄우거나 서비스를 다시 시작.
#      2 개+ -> 바퀴가 안 돈다. 그런데 /odom_raw 는 "돌고 있다"고 보고한다(추측항법).
#               2026-09-07 에 팀이 이것으로 몇 시간을 태웠다. 반드시 하나만 남길 것.
#   2. 장치가 제자리에 있는가 — /dev/soarm(팔) · /dev/rrc(차체) · /dev/gripper_cam(정책 입력)
#      그리퍼캠이 없으면 VLA 파지는 시작조차 하지 말 것 — 정책 입력이 영상 하나뿐이라
#      끊기면 학습 데이터의 평균 동작만 반복한다(허공을 짚는다).
set -u

fail=0
n=$(pgrep -fc "ros_robot_controller" || true)
printf 'ros_robot_controller 프로세스: %s개\n' "$n"
if [ "$n" -eq 0 ]; then
  echo "  ⚠️ 없음 — 차체 명령이 바퀴까지 못 간다"; fail=1
elif [ "$n" -gt 2 ]; then
  # `ros2 run` 래퍼 + 실제 노드로 2개까지는 정상이다.
  echo "  ⚠️ 중복 의심 — 아래에서 부팅 자동 실행분만 남기고 정리할 것"; pgrep -fa "ros_robot_controller"; fail=1
else
  echo "  정상"
fi

for dev in /dev/soarm /dev/rrc /dev/gripper_cam; do
  if [ -e "$dev" ]; then
    printf '%-18s 있음 -> %s\n' "$dev" "$(readlink -f "$dev")"
  else
    printf '%-18s 없음\n' "$dev"
    [ "$dev" = "/dev/gripper_cam" ] && echo "  ⚠️ 그리퍼캠 없음 — VLA 파지 불가 (USB 연결·케이블 고정 확인)"
    fail=1
  fi
done

if command -v fuser >/dev/null 2>&1 && [ -e /dev/soarm ]; then
  holder=$(fuser /dev/soarm 2>/dev/null || true)
  [ -n "$holder" ] && { echo "/dev/soarm 를 쓰는 프로세스:$holder — arm_driver_node 가 이미 떠 있으면 도구는 못 연다"; }
fi

[ "$fail" -eq 0 ] && echo "점검 통과" || echo "점검 실패 — 위 항목을 먼저 해결할 것"
exit "$fail"
