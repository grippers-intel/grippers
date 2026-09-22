# vla_robot

VLA 정책으로 물체를 집는 로봇팔(SO-ARM101)과 탑뷰 ArUco 기반 자율주행 차체(MentorPi 메카넘)를 처음부터 다시 작성한 프로젝트입니다.
`hardware/`의 구조(역할 분담, 인터페이스 모양, 실측값)는 참고만 했습니다. 코드는 한 벌만 새로 작성했습니다.

```
 노트북 (Windows)                                 Raspberry Pi 5 (ROS2 Humble)
 ┌───────────────────────────┐   UDP 5005        ┌──────────────────────────────────────────┐
 │ host/run_host.py          │ ─ HostCommand ──▶ │ pi_mission_node ──Twist──▶ 벤더 odom_publisher │
 │  탑뷰캠×2 + ArUco 위치추정 │                   │   │ (속도 제한 · 워치독 · 작업 순서)          │
 │  Geti 물체 검출 · 경로계획 │ ◀── PiStatus ──── │   ├─ vla/run_grasp ─▶ vla_grasp_node       │
 │  Host FSM                 │   UDP 5006        │   └─ arm/*          ─▶ arm_driver_node ─▶ 서보 │
 └───────────────────────────┘                   │ gripper_cam_node ─▶ gripper_cam/image_raw  │
 policy_server/ (선택, GPU)  ◀── HTTP 8770 ────── │   (vla_grasp_node 가 구독)                  │
                                                  └──────────────────────────────────────────┘
```

## 기존 `hardware/`에서 무엇이 달라졌나

| 기존 문제 | 새 구조 |
|---|---|
| 호환되지 않는 UDP 규격 2개가 같은 포트를 씀 | `vla_common/protocol.py` 하나를 Host와 Pi가 함께 import |
| 완료 보고(GRASP_DONE)를 한 번만 보내서, 패킷 하나만 잃어도 영원히 대기 | 결과를 매 상태 패킷에 반복하고, `JobTracker`가 job_id로 판정 |
| 캘리브레이션 2개(교시/LeRobot)가 EEPROM에 번갈아 쓰임 (wrist_roll 85.8° 어긋남) | LeRobot 캘리브레이션 **하나만** 사용. 다르면 기동 거부 |
| 서보 버스 접근 방식 4가지, 여러 프로세스가 동시에 열 수 있음 | `feetech_bus.py` 하나. POSIX에서 포트 독점 |
| 두 노드가 그리퍼캠을 서로 다른 해상도로 열어 `Device busy` | `gripper_cam_node`만 장치를 열고 나머지는 토픽으로 받음 |
| 런치 인자 오타가 조용히 무시됨 | 설정은 `robot.yaml` 한 파일. 모르는 키가 있으면 기동 거부 |
| udev가 모든 ttyACM을 `/dev/rrc`로 묶음 | 시리얼 번호 기반 규칙(`udev/`) 또는 `/dev/serial/by-id` 사용 |
| Host 코드 사본 4개 | `host/` 하나 |
| 정책 입력 색 순서가 녹화(RGB)와 추론(BGR)에서 다름 | `policy.image_color`로 명시(기본 rgb). 변환 위치는 한 곳뿐 |

## 디렉터리

```
vla_robot/
├── ros2_ws/src/
│   ├── vla_common/            순수 파이썬 계약 (Host·Pi·서버가 함께 사용)
│   │   protocol.py            UDP 규격, JobTracker
│   │   motion_limits.py       속도 제한
│   │   arm_units.py           raw <-> 정책 단위 (lerobot 공식과 동일)
│   │   config.py              엄격한 YAML 로더 (robot.yaml, arm_poses.yaml)
│   │   policy_runner.py       로컬 lerobot 추론 (lerobot을 필요할 때만 import)
│   │   policy_client.py       원격 추론 클라이언트
│   │   policy_wire.py         추론 서버 HTTP 규격
│   ├── vla_robot_interfaces/  msg/srv/action
│   ├── vla_robot_arm/         arm_driver_node + feetech_bus
│   ├── vla_robot_vision/      gripper_cam_node
│   ├── vla_robot_policy/      vla_grasp_node
│   ├── vla_robot_mission/     pi_mission_node (+ 순수 로직 pi_mission_core, udp_link)
│   └── vla_robot_bringup/     launch/robot.launch.py, config/{robot.yaml, arm_calibration.json, arm_poses.yaml}
├── host/                      노트북 미션 (host/README.md 참고)
├── policy_server/             GPU 추론 서버
├── tools/                     arm_check / write_calibration / teach_pose (시리얼 직접 사용)
└── udev/99-vla-robot.rules
```

## Pi 설치

실제 로봇의 컨테이너 구성·배포·기동 절차는 **[docs/pi_deploy.md](docs/pi_deploy.md)** 에 있습니다
(worktree 배포, colcon 빌드, source 순서, `ros_robot_controller` 중복 문제).

아래는 일반적인 요약입니다.

## Pi 설치 (ROS2 Humble 컨테이너 안)

```bash
# 1) 의존성
sudo apt install python3-serial python3-yaml python3-opencv
pip install lerobot==<체크포인트와 같은 버전>   # policy.source: local 일 때만

# 2) 빌드 — 설정 파일을 소스에서 바로 읽도록 symlink-install 사용
cd vla_robot/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 3) udev: 기존 99-ttyACM0.rules 는 삭제하고 udev/99-vla-robot.rules 의 CHANGE_ME 를 채워 설치
```

`robot.yaml`의 `policy.checkpoint`에는 `act_v5_all_180_120k_120000` 경로를 적습니다.
체크포인트를 컨테이너에 복사할 때 `config.json`의 `pretrained_revision` 같은 필드는 지우지 않아도 됩니다. `policy_runner`가 적재에 실패하면 모르는 필드를 뺀 사본으로 다시 시도합니다.

## 캘리브레이션 출처

`vla_robot_bringup/config/arm_calibration.json`은 v5 녹화 때 LeRobot이 실제로 읽은 파일과 같습니다.
원본은 `F:\ml-cache\huggingface\lerobot\calibration\robots\so_follower\grippers_arm.json`(2026-09-02 16:38)입니다.
아래 파일들도 6개 관절의 homing offset과 위치 한계가 모두 같은 것을 확인했습니다(2026-09-14).

- `hardware/grippers/host/vla/calibration/grippers_arm.json`
- `hardware/grippers/tools/arm/servo_backup/servo_COM8_recording_frame.json`
- 마지막 서보 EEPROM 백업 `servo_COM8_20260903_155913.json`

다시 캘리브레이션하면 이 파일을 새 값으로 바꾸고, 새 값으로 정책을 다시 학습해야 합니다.

## 첫 기동 순서 (순서대로 할 것)

```bash
# 1. 팔 점검 (읽기만). Homing_Offset 불일치가 나오면 2번으로
python3 tools/arm_check.py

# 2. 캘리브레이션을 서보에 쓴다. 기본은 차이만 출력하고, --apply 를 줘야 실제로 쓴다. 팔을 손으로 받칠 것
python3 tools/write_calibration.py --apply

# 3. 투하 자세 실측 (drop 은 measured: false 상태라 PLACE 가 거부된다)
python3 tools/teach_pose.py --name drop --keep-gripper --note "상자 위"

# 4. 정책 없이 기동해 차체와 팔만 확인
ros2 launch vla_robot_bringup robot.launch.py use_policy:=false
#    노트북에서: python host/tools/udp_teleop.py --pi-ip <PI_IP>   (WASD/QE, space=정지)
#    팔 포즈:   ros2 action send_goal /arm/move_to_pose vla_robot_interfaces/action/MoveToPose "{pose_name: idle}"

# 5. 파지 단독 시험
ros2 launch vla_robot_bringup robot.launch.py
ros2 action send_goal --feedback /vla/run_grasp vla_robot_interfaces/action/RunVlaGrasp "{label: queen}"

# 6. 노트북에서 전체 미션
python host/run_host.py --pi-ip <PI_IP>
```

비상 정지: `ros2 topic pub --once /vla_robot/estop std_msgs/msg/Empty` (해제는 `/vla_robot/estop_reset`). Host 화면에서는 space 키.

## 원격 추론 (선택)

Pi CPU에서 ACT는 청크당 0.4~0.47초라 로컬로 충분합니다. Diffusion Policy나 SmolVLA는 원격으로 돌립니다.

```bash
python policy_server/policy_server.py --ckpt D:/ckpt/dp_v5_all_180_60k_060000 --host 0.0.0.0 --device cuda --n-action-steps 63
```

그런 다음 `robot.yaml`에서 `policy.source: remote`, `policy.url: http://<노트북IP>:8770`으로 바꿉니다.

## ⚠️ 실기에서 반드시 확인·측정할 값

1. **정책 색 순서 (`policy.image_color`)**
   - LeRobot 녹화 기본값이 RGB이고, 실기로 물체를 집은 기준 롤아웃(`hardware/grippers/tools/arm/rollout_policy.py`)도 RGB였기 때문에 기본값을 `rgb`로 두었습니다.
   - 기존 ROS 경로(`grippers_vla/policy_runner.py`)는 BGR을 넣었습니다.
   - 파지 성공률이 이상하면 가장 먼저 확인하십시오.
2. **`grasp_check.min_gripper_percent` / `min_load_ratio`**: 빈손으로 닫았을 때와 물체를 쥐었을 때의 값을 `ros2 service call /arm/get_state ...`로 재서 정합니다.
3. **`arm_poses.yaml`의 `drop`**: 반드시 `teach_pose.py`로 실측합니다.
4. **udev 시리얼 번호**: `udev/99-vla-robot.rules`의 `CHANGE_ME` 값을 채웁니다.
5. **회전 속도**
   - 기존 팀이 잰 "0.6 rad/s"는 벤더 `cmd_vel` 클램프(±0.5)가 적용된 값입니다.
   - 이 프로젝트는 클램프가 없는 `controller/cmd_vel`에 속도를 보내므로 기본값을 0.5로 두었습니다.
6. **배터리 전압에 따른 데드밴드**: 8.0V 이하에서는 0.10 m/s 명령에 차가 움직이지 않았습니다. 기본 직진 속도는 0.15 m/s입니다.

## 테스트

```bash
# ROS 없이 순수 로직 테스트 (Windows/Linux)
pip install numpy pyyaml pytest pyserial
cd vla_robot/ros2_ws/src
PYTHONPATH=vla_common:vla_robot_arm:vla_robot_mission python -m pytest vla_common/test vla_robot_arm/test vla_robot_mission/test
```

ROS 노드(`*_node.py`), 런치 파일, 실제 서보·카메라·정책 추론은 Pi 실기에서만 확인할 수 있습니다.
