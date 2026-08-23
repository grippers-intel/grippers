#!/usr/bin/env bash
# 시연 텔레옵 — 노트북에서 이거 하나만 실행하면 된다.
#
#   ./teleop.sh            팔 + 베이스 조종 시작
#   ./teleop.sh --record   위에 더해 rosbag2 녹화까지
#   ./teleop.sh --check    실행하지 않고 준비 상태만 점검
#   ./teleop.sh --status   지금 뭐가 돌고 있는지
#   ./teleop.sh --stop     파이에 남은 프로세스 정리
#   ./teleop.sh --arm-only  베이스 없이 팔만 (벤치 테스트)
#   ./teleop.sh --base-only 리더 암 없이 베이스만 (키보드 조종)
#
# 파이 쪽 프로세스는 이 스크립트가 띄우고, 종료할 때 같이 정리한다.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI="${PI_SSH_HOST:-pi}"                  # ~/.ssh/config 의 Host 별칭
PY="$HERE/.venv/bin/python"
LOG_FOLLOWER=/tmp/teleop_follower.log
LOG_BASE=/tmp/teleop_base.log
LOG_BAG=/tmp/teleop_bag.log

# 베이스는 controller.launch.py 가 아니라 한 단계 아래인 odom_publisher.launch.py
# 를 띄운다. controller.launch.py 는 imu_filter → **imu_calib 패키지**를 포함하는데
# 이 컨테이너에 그 패키지가 없어서 런치 전체가 SIGINT 로 죽는다. 텔레옵에는
# IMU 필터·EKF 가 필요 없고, odom_publisher.launch.py 만으로 보드 드라이버
# (ros_robot_controller) + /cmd_vel 구독 + odom 이 모두 올라온다.
#
# need_compile 은 controller/odom_publisher launch 가 os.environ['...'] 으로 직접
# 읽어서 없으면 KeyError 로 죽는다. 컨테이너 어디에도 설정돼 있지 않으므로
# 여기서 넣어준다(False = 소스 트리 경로 사용, grippers_bringup 의 기본값과 동일).
ROS_ENV='source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash &&
         export ROS_DOMAIN_ID=21 && export need_compile=False'

RECORD=0; ARM_ONLY=0; BASE_ONLY=0; ACTION=start
for a in "$@"; do
  case "$a" in
    --record)   RECORD=1 ;;
    --arm-only)  ARM_ONLY=1 ;;
    --base-only) BASE_ONLY=1 ;;
    --check)    ACTION=check ;;
    --status)   ACTION=status ;;
    --stop)     ACTION=stop ;;
    -h|--help)  sed -n '2,11p' "$0"; exit 0 ;;
    *) echo "알 수 없는 인자: $a" >&2; exit 2 ;;
  esac
done

say() { printf '\033[36m▶ %s\033[0m\n' "$*"; }
ok()  { printf '  \033[32m✓\033[0m %s\n' "$*"; }
die() { printf '\033[31m✗ %s\033[0m\n' "$*" >&2; exit 1; }

dexd()    { ssh "$PI" "docker exec -d IntelPi bash -lc '$1'"; }
running() { ssh "$PI" "docker exec IntelPi pgrep -f '$1' >/dev/null 2>&1"; }

stop_pi_side() {
  say "파이 쪽 정리"
  # 베이스를 세우는 게 프로세스를 죽이는 것보다 먼저다.
  ssh "$PI" "docker exec IntelPi bash -lc '
      $ROS_ENV
      timeout 5 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist {} >/dev/null 2>&1 || true
      pkill -f follower_teleop_node.py || true
      pkill -f \"ros2 bag record\" || true
  '" >/dev/null 2>&1 || true
  ok "팔 수신기·녹화 정지, 베이스 정지 명령 전송"
  echo "    (베이스 스택은 켜 둡니다 — 다음 실행이 빨라집니다)"
}

case "$ACTION" in
  status)
    say "파이 상태"
    running follower_teleop_node.py && ok "팔 수신기 동작 중"   || echo "  · 팔 수신기 정지"
    running odom_publisher          && ok "베이스 스택 동작 중" || echo "  · 베이스 스택 정지"
    running "ros2 bag record"       && ok "녹화 중"             || echo "  · 녹화 안 함"
    exit 0 ;;
  stop)
    stop_pi_side; exit 0 ;;
esac

# ── 사전 점검 ───────────────────────────────────────────────────────────────
say "사전 점검"

LEADER_PORT=""
if [ "$BASE_ONLY" -eq 0 ]; then
  LEADER_PORT="$(ls /dev/cu.usbmodem* 2>/dev/null | head -1 || true)"
  [ -n "$LEADER_PORT" ] || die "리더 암을 못 찾았습니다. USB 연결을 확인하세요 (ls /dev/cu.usbmodem*)
       리더 암 없이 베이스만 조종하려면: ./teleop.sh --base-only"
  ok "리더 암   $LEADER_PORT"
  [ -x "$PY" ] || die "파이썬 환경이 없습니다:
       cd $HERE && uv venv --python 3.12 && uv pip install pyserial"
else
  ok "베이스 전용 모드 — 리더 암 불필요"
fi

ssh -o BatchMode=yes -o ConnectTimeout=8 "$PI" true 2>/dev/null || die \
"파이에 SSH가 안 됩니다. 재부팅으로 주소가 바뀌었을 수 있습니다:
       ping6 -c 2 ff02::1%en0 >/dev/null; ndp -an | grep -i '2c:cf:67'
       찾은 주소를 ~/.ssh/config 의 Host pi HostName 에 넣으세요 (%en0 → %%en0)"
ok "파이      SSH 연결됨"

if [ "$BASE_ONLY" -eq 0 ]; then
  ARM_IDS="$(ssh "$PI" "docker exec IntelPi python3 /grippers/tools/teleop/scan_ids.py" 2>/dev/null | tail -1)"
  case "$ARM_IDS" in
    *"[1, 2, 3, 4, 5, 6]"*) ok "팔로워 암 서보 6개 정상" ;;
    *) die "팔로워 암이 응답하지 않습니다 — $ARM_IDS
       암의 서보 전원 라인과 스위치를 확인하세요 (USB는 보드 로직만 먹입니다)" ;;
  esac
fi

if [ "$ACTION" = check ]; then
  running odom_publisher && ok "베이스 스택 동작 중" \
                         || echo "  · 베이스 스택은 실행할 때 자동으로 띄웁니다"
  say "점검 완료 — 실행 준비됨"
  exit 0
fi

# ── 파이 쪽 기동 ────────────────────────────────────────────────────────────
if [ "$ARM_ONLY" -eq 0 ]; then
  if running odom_publisher; then
    say "베이스 스택 이미 동작 중"
  else
    say "베이스 스택 기동 (보드 드라이버 · odom)"
    dexd "$ROS_ENV && ros2 launch controller odom_publisher.launch.py > $LOG_BASE 2>&1"
    for i in $(seq 1 30); do
      running odom_publisher && { ok "준비됨"; break; }
      [ "$i" -eq 30 ] && die "베이스 스택이 안 뜹니다:
       ssh $PI \"docker exec IntelPi cat $LOG_BASE\""
      sleep 1
    done
  fi
fi

if [ "$BASE_ONLY" -eq 1 ]; then
  # 팔 수신기를 띄우지 않는다 — 팔에 토크가 걸리지 않아 현재 자세 그대로 있는다.
  say "조종 시작 — w/s 전후, a/d 좌우평행, q/e 회전, SPACE 정지, Ctrl-C 종료"
  echo
  exec ssh -t "$PI" "docker exec -it IntelPi bash -lc '$ROS_ENV &&
      python3 -u /grippers/tools/teleop/base_teleop_mecanum.py'"
fi

running follower_teleop_node.py && {
  ssh "$PI" "docker exec IntelPi pkill -f follower_teleop_node.py" >/dev/null 2>&1 || true
  sleep 1
}

say "팔 수신기 기동"
EXTRA=""; [ "$ARM_ONLY" -eq 1 ] && EXTRA="--no-ros"
dexd "$ROS_ENV && python3 -u /grippers/tools/teleop/follower_teleop_node.py $EXTRA > $LOG_FOLLOWER 2>&1"

for i in $(seq 1 20); do
  if ssh "$PI" "docker exec IntelPi grep -q '준비 완료' $LOG_FOLLOWER" 2>/dev/null; then
    ok "준비됨"; break
  fi
  if [ "$i" -eq 20 ]; then
    ssh "$PI" "docker exec IntelPi cat $LOG_FOLLOWER" 2>/dev/null || true
    die "팔 수신기가 안 뜹니다"
  fi
  sleep 1
done

if [ "$RECORD" -eq 1 ]; then
  say "rosbag2 녹화 시작"
  dexd "$ROS_ENV && bash /grippers/tools/teleop/record_demo.sh > $LOG_BAG 2>&1"
  sleep 3
  ssh "$PI" "docker exec IntelPi grep -E '녹화 →|중단' $LOG_BAG" 2>/dev/null || true
fi

# ── 조종 ────────────────────────────────────────────────────────────────────
cleanup() {
  echo
  stop_pi_side
  [ "$RECORD" -eq 1 ] && echo "    녹화본: ssh $PI \"docker exec IntelPi ls -t /grippers/recordings | head -3\""
  return 0
}
trap cleanup EXIT

say "조종 시작 — Ctrl-C 로 끝내면 파이 쪽도 같이 정리됩니다"
"$PY" "$HERE/leader_teleop.py" --leader-port "$LEADER_PORT"
