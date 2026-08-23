#!/usr/bin/env bash
# 자율 파지 한 사이클 — 접근 → 파지 → 운반 자세. 필요한 스택은 알아서 띄운다.
#
#   ./cycle.sh                     접근 후 파지, CARRY_IDLE 에서 정지
#   ./cycle.sh --drop              바구니 투하까지 (바구니가 그리퍼 아래 있을 때만)
#   ./cycle.sh --push 20           마무리 전진 거리를 바꾼다(mm)
#   ./cycle.sh --cls star --profile chess_rook
#   ./cycle.sh -- --min-speed 0.07 -- 뒤 인자는 접근 루프로 넘어간다
#
# 팔은 **IDLE 로 접혀 있어야** 한다. 그래야 카메라가 바닥을 본다. 어긋나 있으면
# 파지 스크립트가 시작 자세 검사에서 스스로 멈춘다.
set -uo pipefail

CLS=rook
PROFILE=chess_rook
ACCEL=40
# 접근은 여기까지만 온다. 파지 지점은 카메라가 못 본다 — 물체가 화면 아래로
# 잘려 검출을 잃는다(실측: offset 20 부터 놓침). 그래서 잘 보이는 데까지 서보로
# 붙이고(OFFSET_H), 남은 구간은 오도메트리로 전진한다(PUSH_MM).
OFFSET_H=15        # 실측: 15 는 안정적으로 수렴, 20 부터 검출을 잃음
PUSH_MM=50         # **검증된 값** (2026-08-23). 부하 0.0665 로 안정 파지, 물체를 밀지 않는다.
                   # 지시 50 에 실제 56.6mm — 증분 전진의 분해능이 ±7mm 쯤이다
MIN_MV=7000        # 실측: 6944 에서 정지, 7181 에서 주행
WARN_MV=7400
DROP=""
EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    --cls)     CLS="$2"; shift 2 ;;
    --profile) PROFILE="$2"; shift 2 ;;
    --accel)   ACCEL="$2"; shift 2 ;;
    --drop)    DROP="--drop-to-basket"; shift ;;
    --min-mv)  MIN_MV="$2"; shift 2 ;;
    --offset-h) OFFSET_H="$2"; shift 2 ;;
    --push)    PUSH_MM="$2"; shift 2 ;;
    --)        shift; EXTRA=("$@"); break ;;
    *) echo "모르는 인자: $1" >&2; exit 2 ;;
  esac
done

ENV_SETUP='source /opt/ros/humble/setup.bash
           source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash
           source /ros2_ws/install/setup.bash
           export ROS_DOMAIN_ID=21 need_compile=False DEPTH_CAMERA_TYPE=ascamera'
ARM_ENV='export PYTHONPATH=/ros2_ws/src/grippers_arm:/third_party/soarm_provided_d'

ensure() {
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

# 전압이 낮으면 명령은 나가는데 바퀴가 안 돈다 — 제일 헷갈리는 증상이라 먼저 본다.
#
# **임계값은 실측에서 나왔다.** 6944 mV 에서 베이스가 죽었고 7181 mV 에서는
# 주행했다. 이 팩은 7.4V 급(2S)으로 보이며 만충이 8.4V 언저리다 — 11.1V 팩
# 기준으로 잡으면 영원히 시작하지 못한다. 만충 전압을 확인하면 다시 조정할 것.
# 쓸 수 있는 창이 좁으니(대략 6.9~8.4V) 토크 여유도 크지 않다.
V=$(docker exec IntelPi bash -lc "$ENV_SETUP
    timeout 6 ros2 topic echo /ros_robot_controller/battery --once 2>/dev/null" \
    | grep -oE '[0-9]+' | head -1)
if [ -n "${V:-}" ]; then
  echo "▶ 배터리 ${V} mV"
  if [ "$V" -lt "$MIN_MV" ]; then
    echo "  ✗ 너무 낮습니다(${MIN_MV} mV 미만). 충전하세요. 중단합니다."
    echo "    팩이 다르면 --min-mv 로 바꾸세요."
    exit 1
  fi
  [ "$V" -lt "$WARN_MV" ] && echo "  ⚠ 여유가 없습니다. 중간에 멈추면 전압부터 의심하세요."
fi

cat <<EOF

  대상: $CLS   프로파일: $PROFILE
  접근 보정 +${OFFSET_H}px  ·  마무리 전진 ${PUSH_MM}mm
  아레나에 사람 손과 케이블이 없고, 팔이 IDLE 로 접혀 있는지 확인하세요.
EOF
read -r -p "Enter=시작, 그 외=중단 > " a
[ -z "$a" ] || { echo "중단"; exit 130; }

echo
echo "══ 1/2  접근 ══"
docker exec IntelPi bash -lc "$ENV_SETUP
    python3 -u /grippers/tools/perception/approach.py --cls '$CLS' \
        --offset-h $OFFSET_H --final-push $PUSH_MM ${EXTRA[*]:-}"
rc=$?
case $rc in
  0) ;;
  3) echo "✗ 물체를 못 찾았습니다. 시야 안에 두고 다시 실행하세요."; exit 3 ;;
  130) echo "✗ 중단됨"; exit 130 ;;
  *) echo "✗ 접근이 수렴하지 않았습니다(코드 $rc). 게인이나 허용 오차를 조정하세요."; exit $rc ;;
esac

echo
echo "══ 2/2  파지 ══"
docker exec IntelPi bash -lc "$ENV_SETUP
    $ARM_ENV
    cd /grippers && python3 -u tools/horizontal_grasp_hardware_test.py '$PROFILE' \
        --yes --accel $ACCEL $DROP"
rc=$?
[ $rc -eq 0 ] && echo && echo "✔ 사이클 완료" || echo "✗ 파지 실패(코드 $rc) — 팔 자세를 확인하세요"
exit $rc
