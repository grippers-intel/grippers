#!/usr/bin/env bash
# 차체 컨트롤러가 "명령을 안 먹을" 때 되살린다 — **Pi 재부팅 대신**.
#
#   bash tools/ops/rrc_recover.sh          # 진단(부저)만
#   bash tools/ops/rrc_recover.sh --fix    # 노드를 다시 띄운다
#
## 증상 (2026-09-23 실기에서 확인)
#
# 보드 -> Pi 는 멀쩡한데 Pi -> 보드 만 통째로 죽는다.
#
#   살아 있음   /ros_robot_controller/imu_raw 49 Hz · battery 1 Hz · rosout 에 에러 없음
#   죽어 있음   set_motor 를 20 Hz 로 보내도 바퀴가 안 돈다. **부저도 안 울린다**
#
# ⚠️ `/odom_raw` 는 cmd_vel 을 적분하는 **개루프**라 이 고장을 못 잡는다 — 바퀴가 멈춰
#    있어도 "44 cm 갔다"고 보고한다. 그래서 진단은 **부저**로 한다: 소리가 나면 쓰기
#    방향이 살아 있는 것이고, 안 나면 죽은 것이다. 모터를 돌려 보는 것보다 안전하고 빠르다.
#
# 원인은 보드도 배터리도 아니라 **노드의 시리얼 세션**이다. 그래서 노드만 다시 띄우면
# 몇 초 만에 복구된다(예전에는 이 증상마다 Pi 를 재부팅했다).
set -u

DOMAIN=${ROS_DOMAIN_ID:-21}
SETUP='source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash'

beep() {
  echo "부저를 울립니다 — 소리가 나면 Pi -> 보드 쓰기가 살아 있는 것입니다."
  bash -lc "export ROS_DOMAIN_ID=$DOMAIN && $SETUP && \
    timeout 6 ros2 topic pub --once /ros_robot_controller/set_buzzer \
    ros_robot_controller_msgs/msg/BuzzerState \
    '{freq: 1900, on_time: 0.3, off_time: 0.3, repeat: 2}'" >/dev/null 2>&1
  echo "  (소리가 났습니까?)"
}

count_nodes() {
  pgrep -fc "ros_robot_controller/lib/ros_robot_controller" || true
}

if [ "${1:-}" != "--fix" ]; then
  echo "컨트롤러 노드: $(count_nodes) 개"
  beep
  echo
  echo "소리가 안 났다면:  bash tools/ops/rrc_recover.sh --fix"
  exit 0
fi

echo "컨트롤러 노드를 다시 띄웁니다 (전: $(count_nodes) 개)"
# SIGINT 로 내린다 — 시리얼을 닫고 나가게 한다.
pkill -INT -f "ros_robot_controller/lib/ros_robot_controller" 2>/dev/null || true
pkill -INT -f "ros2 run ros_robot_controller" 2>/dev/null || true
sleep 4
pkill -9 -f "ros_robot_controller/lib/ros_robot_controller" 2>/dev/null || true
sleep 1

# 부팅 자동 실행분과 같은 모양으로 띄운다(래퍼를 덧대지 않는다 — preflight 가 개수를 센다).
setsid bash -lc "export ROS_DOMAIN_ID=$DOMAIN && $SETUP && \
  exec ros2 run ros_robot_controller ros_robot_controller" >/tmp/rrc.log 2>&1 &
sleep 8
echo "후: $(count_nodes) 개   (로그 /tmp/rrc.log)"
beep
echo
echo "여전히 안 울리면 전원을 내렸다 올릴 것 — 그때는 보드 쪽 문제다."
