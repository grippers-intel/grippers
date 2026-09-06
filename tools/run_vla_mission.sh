#!/usr/bin/env bash
# VLA 파지 미션 — 실기로 검증된 인자 묶음 하나. 컨테이너 안에서 실행한다.
#
#   ./run_vla_mission.sh              ACT 120k, Pi 로컬 추론 (노트북 불필요)
#   ./run_vla_mission.sh --dp         DP, 노트북 policy_server 원격 추론
#   ./run_vla_mission.sh --host-ip 192.168.0.5
#   ./run_vla_mission.sh --force      이미 떠 있는 노드를 정리하고 띄운다
#
# ⚠️ 이 스크립트가 있는 이유는 **인자를 손으로 치면 빠뜨리기 때문**이다. 이
# 저장소는 같은 사고를 두 번 겪었다:
#
#   2026-09-01  host_ip 를 안 넘겨 노드 기본값 192.168.0.10 으로 보고가 나갔다
#   2026-09-05  auto_align_on_first_move:=false 가 런치에 배선이 안 돼 조용히 무시됐다
#
# ros2 launch 는 모르는 인자를 오류로 알리지 않는다 — 오타는 조용히 사라진다.
#
# ── 왜 이 값들인가 ────────────────────────────────────────────────────────
#
# use_depth_gate:=false   뎁스캠은 이 팀 구성에 없다. 파지 성공은 그리퍼 위치로
#                         판정한다(baseline_constants.GRIPPER_HELD_POSITION_RAW).
# use_depth_camera:=false 위와 짝이다. 2026-09-07 실측: 켜 두면 perception_node 가
#                         CPU 100% 를 먹고 그리퍼캠이 10Hz 설정에서 0.4Hz 로
#                         굶는다 — 정책이 2.5초 낡은 프레임을 본다.
# policy_source=local     ACT 는 Pi CPU 에서 397~465ms(듀티 14%)로 돈다. 노트북이
#                         필요 없고, 네트워크가 실패 지점에서 빠진다.
# host_ip                 **노트북 주소.** 첫 명령이 오면 자동으로 갱신되지만,
#                         그 전에 나가는 보고는 이 값으로 간다.
set -uo pipefail

BACKEND=act
HOST_IP=192.168.0.2
POLICY_URL=http://192.168.0.2:8770
#: /shared 에 둔다 — 저장소 안에 두면 git stash -u 에 휩쓸린다(2026-09-05).
CHECKPOINT=/shared/act_v5_all_180_120k_120000
FORCE=""
LOG=/tmp/bringup.log

while [ $# -gt 0 ]; do
  case "$1" in
    --act)        BACKEND=act; shift ;;
    --dp)         BACKEND=dp; shift ;;
    --host-ip)    HOST_IP="$2"; shift 2 ;;
    --policy-url) POLICY_URL="$2"; shift 2 ;;
    --checkpoint) CHECKPOINT="$2"; shift 2 ;;
    --force)      FORCE=1; shift ;;
    --log)        LOG="$2"; shift 2 ;;
    -h|--help)    sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "모르는 인자: $1" >&2; exit 2 ;;
  esac
done

# ── 이미 떠 있으면 멈춘다 ────────────────────────────────────────────────
#
# 두 벌이 겹쳐 뜨면 같은 시리얼 포트를 두 번 열고, 죽일 때 서로의 자식을
# 좀비로 남긴다(2026-09-07 에 실제로 그렇게 됐다). 조용히 겹치는 것보다
# 서서 알려 주는 편이 낫다.
RUNNING=$(pgrep -f "grippers_|ros_robot_controller|bringup.launch" | grep -v $$ || true)
if [ -n "$RUNNING" ]; then
  if [ -z "$FORCE" ]; then
    echo "이미 떠 있는 노드가 있다 — 먼저 내리거나 --force 를 줄 것:" >&2
    pgrep -af "grippers_|ros_robot_controller|bringup.launch" | grep -v $$ >&2
    exit 1
  fi
  echo "[run] --force — 기존 노드를 정리한다"
  pkill -INT -f "bringup.launch"; sleep 5
  pkill -9 -f "grippers_|ros_robot_controller|ldlidar|ascamera|bringup.launch" 2>/dev/null
  sleep 3
fi

# ── 세 워크스페이스를 전부 source 한다 ───────────────────────────────────
#
# 하나라도 빠지면 인터페이스나 실행파일을 못 찾는다. need_compile=False 는
# 팀원 스택이 기동 때 재빌드를 시도하지 않게 하는 스위치다.
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=21 need_compile=False DEPTH_CAMERA_TYPE=ascamera

if [ "$BACKEND" = "act" ]; then
  POLICY_ARGS=(policy_source:=local "checkpoint:=$CHECKPOINT" device:=cpu)
  echo "[run] ACT — Pi 로컬 추론, 체크포인트 $CHECKPOINT"
else
  POLICY_ARGS=(policy_source:=remote "policy_url:=$POLICY_URL")
  echo "[run] DP — 원격 추론 $POLICY_URL (노트북 policy_server 가 떠 있어야 한다)"
fi

set -x
exec ros2 launch grippers_bringup bringup.launch.py \
  use_fake_base:=false use_fake_arm:=false use_fake_perception:=false \
  use_vla:=true grasp_backend:=vla \
  "${POLICY_ARGS[@]}" \
  use_depth_gate:=false use_depth_camera:=false \
  default_grasp_label:=queen \
  vla_record_dir:=/grippers/runs/vla \
  "host_ip:=$HOST_IP" \
  > "$LOG" 2>&1
