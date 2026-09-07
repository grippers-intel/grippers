#!/usr/bin/env bash
# VLA 파지 미션 — 실기로 검증된 인자 묶음 하나. 컨테이너 안에서 실행한다.
#
#   ./run_vla_mission.sh              DP, 노트북 policy_server 원격 추론 (기본)
#   ./run_vla_mission.sh --act        ACT 120k, Pi 로컬 추론 (노트북 불필요)
#   ./run_vla_mission.sh --host-ip 192.168.0.5
#   ./run_vla_mission.sh --force      이미 떠 있는 노드를 정리하고 띄운다
#   ./run_vla_mission.sh --vla-only   정책이 안 시킨 것을 전부 뺀다
#                                     (진단용 — 운반·투하는 동작하지 않는다)
#
# ⚠️ 기본이 DP 인 것은 **지금 무엇을 재고 있느냐**에 달린 선택이다(2026-09-07
# 사용자 지시). 지금까지 실기에서 파지→운반→투하를 끝까지 완주한 유일한
# 기록이 DP 쪽이고(2026-09-07 06:00 판, 5회 중 1회), ACT 는 아직 파지 성공
# 기록이 없다. 나중에 RTC(추론과 재생을 겹쳐 팔이 안 서게 하는 기법)를
# 붙일 수 있는 것도 DP 뿐이다 — 반복 샘플러가 있어야 하는데 ACT 는 순전파
# 한 번이다.
#
# ⚠️ 대신 DP 는 **노트북이 떠 있어야 Pi 가 기동한다.** vla_inference_node 가
# 시작할 때 health 로 부딪혀 보고 없으면 일부러 죽는다(파지 도중에 아는
# 것보다 낫다). 노트북 없이 굴려야 하면 --act 를 쓸 것.
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

BACKEND=dp
HOST_IP=192.168.0.2
POLICY_URL=http://192.168.0.2:8770
#: /shared 에 둔다 — 저장소 안에 두면 git stash -u 에 휩쓸린다(2026-09-05).
CHECKPOINT=/shared/act_v5_all_180_120k_120000
#: DP 서버가 이 값으로 떠 있어야 한다. 체크포인트 config 기본값은 32 인데
#: 그러면 재생이 1.07초라 추론 542ms 대비 여유가 절반이다(2026-09-07 실측).
EXPECT_N_ACTION_STEPS=63
FORCE=""
VLA_ONLY=false
LOG=/tmp/bringup.log

while [ $# -gt 0 ]; do
  case "$1" in
    --act)        BACKEND=act; shift ;;
    --dp)         BACKEND=dp; shift ;;
    --host-ip)    HOST_IP="$2"; shift 2 ;;
    --policy-url) POLICY_URL="$2"; shift 2 ;;
    --checkpoint) CHECKPOINT="$2"; shift 2 ;;
    --force)      FORCE=1; shift ;;
    --vla-only)   VLA_ONLY=true; shift ;;
    --log)        LOG="$2"; shift 2 ;;
    -h|--help)    sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "모르는 인자: $1" >&2; exit 2 ;;
  esac
done

# ── 부팅 때 자동 실행된 컨트롤러부터 치운다 ──────────────────────────────
#
# Pi 는 부팅하면 ros_robot_controller 를 자동으로 띄운다. bringup 도 자체
# 컨트롤러를 띄우므로(controller/odom_publisher.launch.py 가 포함한다),
# 그대로 두면 **두 프로세스가 같은 시리얼 포트를 문다.**
#
# ⚠️ 그 상태의 증상이 고약하다 — 소프트웨어는 끝까지 정상으로 보인다.
# 노드도 다 뜨고, cmd_vel 도 나가고, set_motor 도 정상값(0.3445 rps)이
# 찍히는데 **바퀴만 안 돈다.** /odom_raw 는 명령을 되읽는 추측항법이라
# 오히려 '돌고 있다'고 말한다(drive_stall.py 주석). 2026-09-07 에 이걸로
# 몇 시간을 태웠고, 재부팅 뒤 중복을 없애자마자 바퀴가 돌았다.
#
# bringup 이 안 떠 있는데 컨트롤러만 있으면 그건 자동 실행분이다.
if ! pgrep -f "bringup.launch" >/dev/null 2>&1; then
  STRAY=$(pgrep -f "ros_robot_controller" || true)
  if [ -n "$STRAY" ]; then
    echo "[run] 부팅 자동 실행 ros_robot_controller 정리 — 시리얼 포트 중복 방지"
    pkill -9 -f "ros_robot_controller" 2>/dev/null
    sleep 3
  fi
fi

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
# ⚠️ source 동안만 set -u 를 끈다. ROS 의 setup.bash 는 AMENT_TRACE_SETUP_FILES
# 같은 미정의 변수를 참조하는데, set -u 아래서는 그게 즉시 오류다:
#
#     /opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
#
# 2026-09-07 첫 실행이 여기서 죽었다. 우리 스크립트의 오타는 계속 잡고 싶으니
# 끄는 것은 이 네 줄 동안뿐이다.
set +u
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash
source /ros2_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=21 need_compile=False DEPTH_CAMERA_TYPE=ascamera

if [ "$BACKEND" = "act" ]; then
  POLICY_ARGS=(policy_source:=local "checkpoint:=$CHECKPOINT" device:=cpu)
  echo "[run] ACT — Pi 로컬 추론, 체크포인트 $CHECKPOINT"
else
  # ── 서버부터 두드려 본다 ────────────────────────────────────────────
  #
  # 없으면 vla_inference_node 가 기동에서 죽는데, 그 실패는 ROS 로그 깊숙이
  # 묻혀서 "왜 안 뜨지"로 몇 분을 태운다. 여기서 먼저 물어보면 한 줄로 끝난다.
  #
  # 겸사겸사 n_action_steps 를 확인한다. **이 값은 서버가 정한다** — 체크포인트
  # config 기본값은 32(재생 1.07초)인데, 그러면 추론 542ms 대비 여유가 2배로
  # 줄어 팔 정지 비율이 커진다. 63(재생 2.10초)으로 띄우기로 한 이유다.
  echo "[run] DP — 원격 추론 $POLICY_URL 확인 중..."
  HEALTH=$(python3 - "$POLICY_URL" <<'PYEOF' 2>/dev/null
import json, sys, urllib.request
try:
    with urllib.request.urlopen(sys.argv[1] + "/health", timeout=5) as r:
        info = json.load(r)
except Exception:
    sys.exit(1)
print(f"{info.get('ckpt')}|{info.get('n_action_steps')}|{info.get('policy_hw')}")
PYEOF
  )
  if [ -z "$HEALTH" ]; then
    echo "" >&2
    echo "  노트북 policy_server 에 못 붙었다: $POLICY_URL" >&2
    echo "  노트북에서 먼저 띄울 것 (.venv-dp 여야 한다 — ACT 용 .venv 로는" >&2
    echo "  이 체크포인트가 안 읽힌다):" >&2
    echo "" >&2
    echo "    .venv-dp\Scripts\python.exe grippers\tools\arm\policy_server.py \\" >&2
    echo "      --ckpt ckpt_dp_v5_all\dp_v5_all_180_60k_060000 \\" >&2
    echo "      --device cuda --scheduler DDPM --denoise 10 --n-action-steps 63 --host 0.0.0.0" >&2
    echo "" >&2
    echo "  노트북 없이 굴리려면 --act 를 쓸 것." >&2
    exit 1
  fi
  SRV_CKPT=${HEALTH%%|*}; REST=${HEALTH#*|}; SRV_STEPS=${REST%%|*}; SRV_HW=${REST#*|}
  echo "[run] 서버 응답 — $SRV_CKPT / n_action_steps $SRV_STEPS / 입력 $SRV_HW"
  if [ "$SRV_STEPS" != "$EXPECT_N_ACTION_STEPS" ]; then
    echo "[run] ⚠️ n_action_steps 가 $SRV_STEPS 다 (기대 $EXPECT_N_ACTION_STEPS)." >&2
    echo "        재생 길이가 달라져 팔 정지 비율이 바뀐다 — 서버를" >&2
    echo "        --n-action-steps $EXPECT_N_ACTION_STEPS 로 다시 띄우는 것이 맞다." >&2
  fi
  POLICY_ARGS=(policy_source:=remote "policy_url:=$POLICY_URL")
fi

# ⚠️ setsid + PGID 파일. 팀 도구인 tools/ops/stop_bringup.sh 가
# /tmp/bringup.pgid 를 읽어 **프로세스 그룹**에 SIGINT 를 보낸다
# (kill -INT -- -$PGID). 그러려면 새 세션의 리더여야 하고, 그 PID 를
# 남겨 둬야 한다.
#
# 2026-09-07: 이걸 안 해서 stop_bringup.sh 가 우리 판을 못 껐고
# ("bringup_now.sh 로 띄운 게 아니면 못 끕니다"), run_mission 의 자동
# 정리도 같은 이유로 실패했다. 우리 스크립트만 팀 도구 밖에 있을 이유가 없다.
#
# exec 를 안 쓰므로 이 스크립트는 바로 프롬프트를 돌려준다 — 로그는
# "$LOG" 로 간다(bringup_now.sh 와 같은 동작).

set -x
setsid ros2 launch grippers_bringup bringup.launch.py \
  use_fake_base:=false use_fake_arm:=false use_fake_perception:=false \
  use_vla:=true grasp_backend:=vla \
  "${POLICY_ARGS[@]}" \
  use_depth_gate:=false use_depth_camera:=false \
  "vla_only:=$VLA_ONLY" \
  default_grasp_label:=queen \
  vla_record_dir:=/grippers/runs/vla \
  "host_ip:=$HOST_IP" \
  > "$LOG" 2>&1 &

PGID_FILE=/tmp/bringup.pgid
echo "$!" > "$PGID_FILE"
echo "[run] launch PID/PGID = $(cat $PGID_FILE) — 정지: stop_bringup.sh, 로그: $LOG"
