# Pi 실기 체크리스트

MentorPi + SO-ARM101 실기 시 위에서부터 진행합니다. 실패한 단계가 있으면 다음 모션 단계로 넘어가지 않습니다.

명령 앞의 실행 위치를 구분합니다.

- **Pi 호스트**: `/dev/soarm`, `/dev/rrc`, udev 확인
- **ROS2 컨테이너**: `ros2 run`, `ros2 launch`, action/service/topic 확인
- 저장소 경로는 환경마다 다를 수 있으므로 시작 시 `pwd`와 workspace 위치를 먼저 확인합니다.

## 0. 코드 / 안전 preflight

### 0-1. 코드 상태

저장소 루트에서 실행합니다.

```bash
git status --short
git branch --show-current
git log --oneline --decorate -5
```

서브모듈을 확인합니다. **`third_party/soarm_provided_d` 가 비어 있으면 2단계가 통째로 실패합니다.**

```bash
git submodule status
```

앞에 `-` 가 붙어 있으면 미초기화 상태이므로 받아옵니다.

```bash
git submodule update --init --recursive
```

ROS 2 환경을 확인합니다.

```bash
source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
ros2 pkg prefix grippers_interfaces
```

#148 2단계 베이스 제어가 현재 checkout에 포함됐는지 확인합니다.

```bash
grep -nE 'YAW_ALIGN_TOL_RAD|YAW_REALIGN_TRIG_RAD|REALIGN_MIN_DIST_M|DRIVE_TO_TIMEOUT_SEC'   ros2_ws/src/grippers_base/grippers_base/base_driver_node.py
```

네 상수가 나오지 않으면 #148 수정 전 코드이므로 1 m / 90° 실측을 진행하지 않습니다.

안전 기준:

- 첫 주행은 바퀴를 띄운 상태로 확인
- 팔/베이스 이동 경로 비우기
- 즉시 전원 차단 가능 상태 유지
- `enable_torque_on_start` 기본값 `false` 유지

## 1. USB / udev 확인

```bash
ls -l /dev/soarm /dev/rrc
readlink -f /dev/soarm
readlink -f /dev/rrc
```

두 경로는 서로 다른 실제 장치를 가리켜야 합니다.

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true
udevadm info -q property -n /dev/soarm | sort
udevadm info -q property -n /dev/rrc | sort
```

## 2. SO-ARM101 서보 기준선

저장소 루트에서 실행합니다.

```bash
PYTHONPATH=third_party/soarm_provided_d/soarm_lab python3 -c "from driver_sdk import STS3215Driver; d=STS3215Driver('/dev/soarm'); print('connect=', d.connect()); print('connected=', d.is_connected()); [print(i, 'torque=', d.get_torque(i), 'position=', d.get_position(i), 'voltage=', d.get_voltage(i)) for i in range(1,7)]"
```

ID 1~6의 `torque / position / voltage`를 기록합니다. `None`이 하나라도 나오면 모션 테스트를 중단합니다. 특히 ID 5를 별도로 확인합니다.

## 3. arm_driver / #157 확인

자동 torque enable 없이 기동:

```bash
ros2 run grippers_arm arm_driver   --ros-args   -p arm_port:=/dev/soarm   -p enable_torque_on_start:=false
```

기대 결과:

- 장치 연결 실패 → 기동 실패
- torque 상태 읽기 실패 → 기동 실패
- torque OFF → 경고하되 자동 enable 안 함
- 모두 ON → 정상 기동

명시적 enable은 팔을 지지하고 작업 공간을 비운 상태에서만 실행합니다.

```bash
ros2 run grippers_arm arm_driver   --ros-args   -p arm_port:=/dev/soarm   -p enable_torque_on_start:=true
```

> 3~4단계의 단독 `arm_driver` 시험을 마치면 **Ctrl-C로 해당 노드를 종료한 뒤** 5단계 전체 bringup으로 넘어갑니다. 단독 노드와 bringup의 arm_driver를 동시에 띄우지 않습니다.

## 4. 팔 단위 기능

```bash
ros2 service call /arm_driver/hold_position std_srvs/srv/Trigger "{}"
ros2 service call /arm_driver/get_load grippers_interfaces/srv/GetLoad "{}"
ros2 service call /arm_driver/set_gripper grippers_interfaces/srv/SetGripper "{width_mm: 90.0}"
ros2 service call /arm_driver/fold_to_cradle std_srvs/srv/Trigger "{}"
```

`GRIPPER_OPEN_MM`과 `CRADLE_XYZ_M`은 실측 전 placeholder이므로 충돌 여부를 확인한 뒤 실행합니다.

## 5. 전체 bringup

```bash
ros2 launch grippers_bringup bringup.launch.py   use_fake_arm:=false   use_fake_perception:=true   use_fake_interpreter:=true   arm_port:=/dev/soarm
```

다른 터미널:

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
ros2 topic info /cmd_vel --verbose
```

## 6. 베이스 정지 / odom 확인

```bash
ros2 service call /base_driver/stop std_srvs/srv/Trigger "{}"
ros2 topic echo /odom
```

현재 odom은 `cmd_vel` 기반 오픈루프 dead reckoning이므로 실제 이동 거리는 별도로 줄자로 측정합니다.

## 7. #148 2단계 주행 실측

이 단계는 #148 코드가 현재 checkout에 포함된 것을 0단계에서 확인한 경우에만 진행합니다.

아래 예시는 시험 시작 시 `/odom`이 대략 `x=0, y=0, yaw=0`인 경우입니다. 앞선 주행으로 odom이 누적됐다면 재기동하거나 현재 pose 기준으로 목표 좌표를 다시 계산합니다.

약 90° 정렬 후 이동 전에 다른 터미널에서 실제 명령을 관찰합니다.

```bash
ros2 topic echo /cmd_vel
```

주행 명령:

```bash
ros2 action send_goal   /base_driver/drive_to   grippers_interfaces/action/DriveTo   "{target: {x: 0.0, y: 1.0, theta: 0.0}}"   --feedback
```

확인:

- ALIGN에서 `linear.x == 0`
- DRIVE에서 `angular.z == 0`
- 큰 yaw drift 시 DRIVE → ALIGN 복귀
- 도착 후 `/cmd_vel` zero
- 실제 최종 위치와 시간 기록

직선 1 m:

```bash
ros2 action send_goal   /base_driver/drive_to   grippers_interfaces/action/DriveTo   "{target: {x: 1.0, y: 0.0, theta: 0.0}}"   --feedback
```

튜닝 대상:

- `YAW_ALIGN_TOL_RAD`
- `YAW_REALIGN_TRIG_RAD`
- `REALIGN_MIN_DIST_M`

한 번에 여러 상수를 바꾸지 말고 한 변수씩 변경 후 같은 조건에서 재시험합니다.

## 8. 카메라 / perception

확정 기준입니다. 정본은 [`workspace_layout.html`](../design/workspace_layout.html) Rev.II 이고,
재현은 `python tools/a2/coverage_analysis.py` 입니다.

- Logitech C270 ×2 · 1280×720 · 실측 `f = 1410 px`
- 마주보는 두 변 중앙 (각자 담당 절반 1800 × 900 mm)
- 높이 **1650 mm** (삼각대 최대치 — 고정값)
- 후퇴 **950 mm**
- 하향 **44.1°**
- 작업 공간 1.8 × 1.8 m
- 물체 폭 **40 mm 확정** (최악점 21.4 px)

### 설치 오차가 좁습니다

```bash
# 바닥에 변 중앙선(x = 900 mm)을 먼저 긋고, 그 선에 맞춰 삼각대를 세웁니다.
```

**좌우 허용 오차는 ±14 mm 뿐입니다.** 1 mm 어긋날 때마다 `max|u|` 가 0.70 px 씩 늘고,
현재 여유가 10 px(629.99 / 640)입니다. 25 mm 어긋나면 프레임을 넘습니다.
높이보다 좌우가 훨씬 예민하니 **줄자로 재서 세웁니다.**

### 🔴 가까운 벽 앞 0.26 m 는 프레임에 안 들어옵니다 — 정상입니다

후퇴 950 mm 는 권고치 1100 mm 보다 짧고, 그 대가로 **자기 앞쪽 띠를 포기**했습니다.
그 구간은 **마주보는 반대편 카메라가 덮습니다.**

- 울타리(35 × 45 cm) 가림 **0.26 m**
- 화각 전폭 시작 0.21 m · 프레임 하단 컷 0.06 m
- → **실효 커버는 벽에서 0.26 m 부터**

벽에 붙은 물체가 한쪽 카메라에서 안 보이는 것은 **고장이 아닙니다.**
다만 **카메라 한 대가 죽으면 그 띠는 아무도 못 봅니다** — 한 대만 켜고 시험한 결과를
전역 관측으로 읽지 마십시오.

### 물체 크기와 카메라 문제를 혼동하지 않습니다

- 30 mm 물체는 최악점 **16.0 px** 로 검출 하한 20 px 미달입니다. **배치로는 못 고칩니다**
  (높이·후퇴 전수 탐색 최대치가 17.7 px).
- 20 px 을 보장하는 최소 폭은 **37.4 mm**, 확정 규격은 **40 mm**(21.4 px)입니다.
- 여유가 **7% 뿐**이라 조명·모션블러·`imgsz` resize 로 유효 픽셀이 줄면 곧바로 하한입니다.
- **현재 실물 30~37 mm 는 재출력 대상입니다.**

## 9. fake perception E2E

#146 시연 요건상 fake perception이 허용된 경우에만 사용합니다.

```bash
ros2 launch grippers_bringup bringup.launch.py   use_fake_arm:=false   use_fake_perception:=true   use_fake_interpreter:=true   arm_port:=/dev/soarm
```

이 모드는 실제 카메라 검출이 아니라 FakePerception 좌표를 이용한 FSM + 실제 하드웨어 통합 시험입니다.

## 10. 종료 후 기록

```bash
git rev-parse HEAD
git status --short
```

기록 항목:

- commit SHA
- `/dev/soarm`, `/dev/rrc` 실제 경로
- servo 1~6 torque / position / voltage
- ID 5 상태
- arm fault 발생 여부
- 1 m 실제 이동량
- 90° 정렬 후 최종 위치/각도 오차
- #148 튜닝값
- 카메라 실제 capture 해상도
- **웹캠 실측 설치값 — 높이 · 후퇴 · 하향각 · 변 중앙선에서의 좌우 어긋남(mm)**
- **벽에서 몇 m 부터 실제로 검출됐는지** (계산값 0.26 m 와 대조)
- 시험에 쓴 물체 실측 폭
- fake/real perception 여부
- 실패 로그와 재현 명령
