# 개발 환경 · 실행 · 테스트


## Prerequisites — 로봇 (Pi 5)

- **Linux** (개발 환경 기준)
- **ROS 2 Humble** — `IntelPi` Docker 이미지(`ros:humble-export`)로 통일. 호스트 Pi 5의 Jazzy는 사용하지 않음
  - ✅ 기준 배포판 확정 (2026-08-17, #96) — Humble 컨테이너 유지. 근거는 [`hld.md`](../design/hld.md) §9 #13, [`rejected_designs.md`](rejected_designs.md#9-ros2-배포판-통일)
- **Python 3.10** (`IntelPi` 컨테이너 내장). CI는 Ubuntu 24.04 + Python 3.12로 도므로 버전 특이 문법은 피할 것
- Git (submodule 지원 — `third_party/soarm_provided_d`)
- `MACHINE_TYPE=MentorPi_Mecanum` 환경변수 — `IntelPi` 이미지에 설정되어 있음
- **Hailo 런타임** — 하드웨어는 8/11 장착 완료, 소프트웨어 경로는 미검증. 공식 `hailo-all` 은 Raspberry Pi OS 기준인데
  우리는 **Ubuntu 24.04** 이므로 PCIe 드라이버 + HailoRT 별도 설치 가능성이 높고, 컨테이너에 **`/dev/hailo0` 패스스루**가 필요합니다
- **디스크 여유 확인 필수** — `df -h /`. PCIe를 HAT이 점유하므로 NVMe 증설로 해결 불가

## Prerequisites — 노트북 (관제 콘솔)

- **ROS 2 통신 가능 환경** — Ubuntu 네이티브 권장. Windows면 WSL2에서 DDS 디스커버리 이슈 확인 필요
- **`ROS_DOMAIN_ID` 를 로봇과 동일하게 설정**
- **오디오 입출력** — 내장 마이크/스피커 또는 USB 핀마이크
- STT 엔진 — `whisper.cpp` (1순위) 또는 `vosk` — 지연 실측 후 확정
- ⚠️ **네트워크가 크리티컬 패스입니다.** 시연장 WiFi가 AP 격리를 켜두면 DDS 디스커버리가 되지 않습니다.
  전용 라우터나 로봇 핫스팟을 준비하고, M2에서 반드시 실측하세요

## Installation (로봇)

**1. [Pi 5 호스트] 저장소 clone (서브모듈 포함)**

```bash
mkdir -p ~/docker/shared
git clone --recurse-submodules https://github.com/grippers-intel/grippers.git ~/docker/shared/grippers
```

`--recurse-submodules` 를 빠뜨리면 `third_party/soarm_provided_d` 가 빈 폴더로 받아집니다. 이미 clone했다면:

```bash
cd ~/docker/shared/grippers
git submodule update --init --recursive
```

**2. [Pi 5 호스트] `ros_start.sh` 마운트 경로 확인**

```bash
-v ${HOME}/docker/shared/grippers/ros2_ws:/ros2_ws \
-v ${HOME}/docker/shared/grippers:/grippers \
-v ${HOME}/docker/shared/grippers/third_party:/third_party \
```

**3. [Pi 5 호스트] 컨테이너 기동 → 진입**

```bash
./ros_start.sh
./exec_shell.sh
```

**4. [IntelPi 컨테이너] Python 의존성 설치**

```bash
pip3 install --no-cache-dir pyserial mujoco numpy pytest
```

**5. [IntelPi 컨테이너] PYTHONPATH 등록**

```bash
echo 'export PYTHONPATH="/grippers:/third_party/soarm_provided_d:${PYTHONPATH}"' >> ~/.zshrc
source ~/.zshrc
```

**6. [Pi 5 호스트] 시리얼 장치 udev 규칙 등록**

MentorPi 베이스 보드와 SO-ARM101은 `/dev/ttyACM*` 번호가 USB 연결 순서에
따라 바뀔 수 있으므로 고정 심볼릭 링크를 사용합니다.

```bash
sudo tee /etc/udev/rules.d/99-grippers-serial.rules >/dev/null <<EOF
KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", SYMLINK+="soarm", MODE="0666", GROUP="dialout"
KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", SYMLINK+="rrc", MODE="0666", GROUP="dialout"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

장치를 다시 연결한 뒤 확인합니다.

```bash
ls -l /dev/soarm /dev/rrc
```

기준:

- SO-ARM101 (`1a86:55d3`) → `/dev/soarm`
- MentorPi 베이스 보드 (`1a86:55d4`) → `/dev/rrc`

`arm_driver_node`와 `bringup.launch.py`의 기본 `arm_port`는 `/dev/soarm`입니다.
`/dev/rrc`와 같은 실제 장치를 가리키면 arm driver는 기동을 거부합니다.

**7. [IntelPi 컨테이너] 빌드**

```bash
cd /ros2_ws
sudo rosdep init 2>/dev/null
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.zsh
```

`Summary: N packages finished` 가 뜨고 실패 패키지가 없으면 완료입니다.

## Run — 시뮬레이션 (하드웨어 불필요)

가장 빠른 검증은 `domain/task` 의 FSM을 Fake 어댑터로 끝까지 돌리는 것입니다 (ROS2도 필요 없음):

```bash
cd /grippers
python3 -m pytest tests/ -v
```

`IDLE → SCAN → ... → DONE` 까지 루프가 **유한 스텝 안에 종료되면** 도메인 로직은 정상입니다.

## Run — 실기

```bash
# 터미널 1 — MentorPi 저수준 드라이버 (odom, ekf, 모터)
cd /ros2_ws && source install/setup.zsh
export need_compile=False
ros2 launch controller controller.launch.py

# 터미널 2 — base_driver
ros2 run grippers_base base_driver

# 터미널 3 — arm_driver
ros2 run grippers_arm arm_driver

# 터미널 4 — perception
ros2 run grippers_perception perception

# 터미널 5 — mission_orchestrator
ros2 run grippers_mission mission_orchestrator
```

**노트북 측 (관제 콘솔)**

```bash
export ROS_DOMAIN_ID=<로봇과 동일한 값>

# 통신 확인 — 로봇 토픽이 보여야 함
ros2 topic list

# 명령 발행 (CLI — GUI 완성 전 폴백)
ros2 topic pub --once /command std_msgs/String "data: '장난감 정리해줘'"

# 상태 확인
ros2 topic echo /mission/state

# 음성 콘솔 (M3 이후)
ros2 run grippers_console voice_io
```

> **주의**: `mission_orchestrator` 가 `APPROACH` 에 들어가는 순간 실제로 `/cmd_vel` 이 발행되어
> 베이스가 움직입니다. 처음 실행 시 바퀴를 들어두거나 충분한 공간을 확보하세요.

## Troubleshooting

| 증상 | 확인 사항 |
|---|---|
| **노트북에서 로봇 토픽이 안 보임** | ① `ROS_DOMAIN_ID` 일치 ② **시연장 WiFi AP 격리** — 전용 라우터/핫스팟으로 전환 ③ 방화벽(UDP 7400~7500번대) |
| 음성 명령 지연 | 오디오를 토픽으로 보내고 있지 않은지 확인 — **STT는 노트북에서 끝내고 텍스트만 발행** |
| 주행 중 Pi 리셋 | `vcgencmd get_throttled` — 전원 도메인 분리 여부 |
| 로봇이 제자리에서 정지 | LiDAR 스캔에 팔이 장애물로 검출 — 각도 마스킹 확인 |
| **같은 물체를 계속 다시 집으려 함** | 처리 완료 목록 등록 여부, 상자 영역 마스킹 확인 — **루프 FSM 최대 리스크** |
| **먼 쪽 물체만 검출 안 됨** | C270가 1280×720으로 열렸는지 확인 — `toy` 40 mm면 최악점 21.4 px라 정상이면 잡힌다. **여유가 7%뿐이라** 조명·모션블러부터 의심할 것. **최대 폭 37.4 mm 미만이면 규격 문제**. YOLO resize(`imgsz`)도 확인 |
| **가까운 벽 앞 물체만 검출 안 됨** | 정상이다 — 실효 커버는 벽에서 **0.26 m부터** 시작한다(울타리 35 × 45 cm 확정 기준). 그 띠는 **마주보는 반대편 카메라**가 덮으므로 **두 대가 다 열렸는지** 확인 |
| **파지가 일정한 방향으로 빗나감** | 호모그래피에 **박스 중심**을 넣고 있는지 확인 → **아래쪽 모서리**로 교체 |
| **검정 상자를 못 찾거나 그림자를 상자로 오인** | 밝은 테두리·ArUco 부착 여부 확인. `L*` 임계값만으로는 그림자와 구분 불가 |
| 상자 색 인식 실패 | 화이트밸런스 고정 여부, LAB 범위 재튜닝 |
| **체스말을 `toy` 상자에 넣음** | 오각별기둥 ↔ 룩 혼동이 유일하게 남은 쌍이다(둘 다 세운 기둥). 두 형상의 실루엣 차이를 학습 분포에 넣었는지 확인 |
| 치수 추정값이 실제와 다름 | 바닥면 호모그래피 재캘리브레이션. 세운 물체는 높이 부분 역산에 의존 |
| 화면에서 물체가 사라짐 | 로봇 차체 또는 다른 물체가 가림 — `SCAN` 은 정지·후퇴 후 수행. 시선 방향 일렬 배치 회피 |
| 주행 중 로봇이 떨림 | `/cmd_vel` 발행 주체가 2개 이상인지 확인 |
| 물체가 그리퍼 안에서 자전 | 마찰 패드 또는 V홈 핑거 적용 여부 |
| `ParameterAlreadyDeclaredException` | launch와 노드 양쪽 파라미터 중복 선언 |
| `ros_robot_controller` 가 `/dev/rrc` 못 찾음 | `ls /dev/ttyUSB* /dev/ttyACM*` 로 실제 연결 확인 |
| `git push` 시 `node: Permission denied` | `sudo git config --system --unset-all credential.helper` 후 `git config --global credential.helper store`, 비밀번호 자리에 PAT 입력 |

---


도메인 로직은 **하드웨어 없이 전량 검증**하는 것을 목표로 합니다.
CI는 매 push마다 lint + unittest + Fake 어댑터 기반 전체 미션 파이프라인을 실행합니다.

```bash
cd /grippers
export PYTHONPATH="/grippers:${PYTHONPATH}"
python3 -m pytest tests/ -v
```

**루프 FSM에서 반드시 있어야 하는 테스트**

| 테스트 | 검증 내용 |
|---|---|
| `test_full_mission_completes` | `IDLE` 시작 → 전체 물체 처리 → `DONE` 종료 |
| `test_terminates_on_repeated_detections` | **`ScriptedPerception` 이 같은 목록을 계속 반환해도 유한 스텝에 종료** |
| `test_grasp_failure_continues_mission` | 파지 실패 물체가 보류되고 다음 물체로 진행 |
| `test_reject_when_no_phi_solution` | 투입 불가 시 `REJECT` 후 미션 계속 |
| `test_fetch_selects_target_class` | FETCH 모드에서 지정 클래스만 선택 |
| `test_placement_rule_changed_by_command` | 명령으로 배치 규칙이 실제로 변경됨 |
| `test_estop_interrupts_immediately` | 어느 상태에서든 즉시 `ESTOP` 전이 |

> **`test_terminates_on_repeated_detections` 가 가장 중요합니다.** 루프 FSM의 최대 리스크인
> 무한 루프를 **하드웨어 없이 CI에서** 잡습니다. 실기로 잡으면 반나절이 날아갑니다.

---


---

[← README](../../README.md) · [Documentation](../README.md#-documentation)
