# Pi 배포

실측으로 확인한 이 로봇(2026-09-22, `raspberrypi` / 192.168.0.7)의 실제 구성에 맞춘 절차입니다.

## 1. 이 Pi 가 어떻게 생겼나

```
호스트 (Raspberry Pi OS, Python 3.13, ROS 없음)
└── docker 컨테이너 IntelPi
      /grippers   <- 호스트 /home/pi/docker/shared/grippers   (팀 저장소 체크아웃)
      /ros2_ws    <- 호스트 /home/pi/docker/shared/grippers/ros2_ws
      ROS2 Humble · Python 3.10.12 · colcon
      pyserial 3.5 · numpy 1.26.4 · cv2 5.0.0 · torch 2.13.0+cpu · lerobot 0.4.4 · PyYAML 5.4.1
```

- **ROS 는 컨테이너 안에만 있습니다.** 호스트에서 `ros2` 를 치면 없습니다.
- `pi` 사용자가 `docker` 그룹이라 `sudo` 없이 `docker exec` 가 됩니다.
- **lerobot 0.4.4 가 이미 있습니다** — ACT v5 체크포인트를 만든 버전과 같으므로 Pi 로컬 추론이 됩니다
  (`policy.source: local`). Diffusion Policy 는 0.6.x 형식이라 이 컨테이너에서는 안 읽힙니다 → 원격으로.
- 장치: `/dev/soarm`(팔, 1a86:55d3) · `/dev/rrc`(차체 보드, 1a86:55d4) · `/dev/gripper_cam` · `/dev/ttyUSB0`(라이다)

> [!warning] 팀원이 같이 씁니다
> `/home/pi/docker/shared/grippers` 체크아웃은 다른 사람 브랜치에 가 있을 수 있습니다
> (2026-09-22 확인 시 `kica927/baseline_mission`, 서브모듈에 로컬 수정 있음).
> **브랜치를 바꾸지 마세요.** 아래처럼 worktree 를 하나 더 붙입니다.

## 2. 배포 (git worktree)

호스트에서:

```bash
cd /home/pi/docker/shared/grippers
git fetch origin sysy009/vla_robot_rewrite
git worktree add vla_deploy origin/sysy009/vla_robot_rewrite     # -> /grippers/vla_deploy
echo "/vla_deploy/" >> .git/info/exclude                          # 남의 git status 를 더럽히지 않는다
```

`.git/info/exclude` 는 이 기계에만 적용되고 커밋되지 않습니다. 걷어낼 때는:

```bash
git worktree remove vla_deploy
```

## 3. 빌드 (컨테이너 안)

```bash
docker exec -it IntelPi bash -lc '
  source /opt/ros/humble/setup.bash &&
  cd /grippers/vla_deploy/vla_robot/ros2_ws &&
  colcon build --symlink-install'
```

> [!warning] `--symlink-install` 이어도 **설정 파일은 복사됩니다**
> 파이썬 소스는 심링크되지만 `data_files`(config/launch)는 복사본입니다(2026-09-22 실측).
> `robot.yaml`·`arm_poses.yaml` 을 고치면 **설치본은 옛 값 그대로**입니다.
> `teach_pose.py` 가 소스의 yaml 을 고치므로 이 함정에 매번 걸립니다.
>
> 그래서 기동할 때 **소스의 config 를 직접 지정하는 것을 권합니다**(아래 4번).
> 그렇게 하면 `poses_file` 도 그 파일 기준으로 풀려 소스 쪽 `arm_poses.yaml` 을 읽습니다.
> 설치본을 쓰려면 설정을 고칠 때마다 `colcon build` 를 다시 하십시오.

## 4. 기동

```bash
docker exec -it IntelPi bash -lc '
  export ROS_DOMAIN_ID=21 &&
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&                      # 벤더: controller · ros_robot_controller
  source /grippers/vla_deploy/vla_robot/ros2_ws/install/setup.bash &&
  bash /grippers/vla_deploy/vla_robot/tools/ops/pi_preflight.sh &&
  ros2 launch vla_robot_bringup robot.launch.py     config:=/grippers/vla_deploy/vla_robot/ros2_ws/src/vla_robot_bringup/config/robot.yaml'
```

`config:=` 로 **소스 경로**를 주는 것이 기본입니다 — 설치본은 빌드 시점의 복사본이라 설정 변경이 반영되지 않습니다(3번 경고).

source 순서가 중요합니다. 벤더 워크스페이스를 먼저 얹어야 `controller` 패키지를 찾습니다.

단계별로 켜기:

```bash
ros2 launch vla_robot_bringup robot.launch.py use_policy:=false           # 팔·차체만
ros2 launch vla_robot_bringup robot.launch.py use_base:=false use_camera:=false  # 팔만
```

## 5. ⚠️ ros_robot_controller 중복 — 바퀴가 안 도는 원인

**부팅 때 `ros_robot_controller` 가 자동으로 뜹니다.** 여기에 하나를 더 띄우면 바퀴가 멈춥니다.
그런데 `/odom_raw` 는 보낸 명령을 되읽는 추측항법이라 **"정상적으로 돌고 있다"고 보고합니다.**
노드 수·`cmd_vel`·`set_motor` 가 전부 정상으로 보이고, 탑뷰로 봐야 안 도는 것이 드러납니다.
팀이 2026-09-07 에 이것으로 몇 시간을 태웠습니다.

그래서 `robot.launch.py` 는 **컨트롤러를 띄우지 않습니다**(`use_vendor_controller:=false` 기본).
벤더 `odom_publisher.launch.py` 를 통째로 포함하지 않는 이유도 이것입니다 — 그 런치가 컨트롤러를
같이 띄웁니다. 우리는 `odom_publisher` 노드만 직접 띄웁니다.

```bash
pgrep -fa ros_robot_controller     # ros2 run 래퍼 + 노드 = 2줄이 정상
```

- 0 줄이면 자동 실행분이 죽은 것 → `use_vendor_controller:=true` 로 켜거나 서비스를 재시작
- 3 줄 이상이면 중복 → 부팅 자동 실행분만 남기고 정리

`tools/ops/pi_preflight.sh` 가 이 검사와 장치 확인을 한 번에 합니다.

## 6. 체크포인트

`robot.yaml` 의 기본값은 `/shared/act_v5_all_180_120k_120000` 입니다(컨테이너 안 경로).
저장소 밖에 두는 이유는 `git stash -u` 에 휩쓸려 사라진 사고가 있었기 때문입니다.

```bash
docker exec IntelPi ls /shared/act_v5_all_180_120k_120000
```

없으면 노트북에서 복사하거나 `policy.source: remote` 로 노트북 GPU 를 씁니다.

## 7. 도구를 쓸 때 (arm_check / write_calibration / teach_pose / goto_pose / gripper_probe)

도구는 시리얼을 **직접** 엽니다. `arm_driver_node` 가 떠 있으면 포트 독점 때문에 실패합니다(의도된 동작).
런치를 내리고 쓰거나, 노드를 뺀 채 기동하십시오.

호스트에서도 돌아갑니다(호스트 Python 3.13 에 pyserial·pyyaml 이 있습니다).
⚠️ **경로가 다릅니다** — `/grippers` 는 컨테이너 안 경로입니다.

```bash
# 호스트에서
cd /home/pi/docker/shared/grippers/vla_deploy/vla_robot && python3 tools/arm_check.py
# 컨테이너 안에서
docker exec -it IntelPi bash -lc "cd /grippers/vla_deploy/vla_robot && python3 tools/arm_check.py"
```

## 8. 되돌리기

```bash
git worktree remove vla_deploy          # 빌드 산출물까지 같이 사라진다
```

컨테이너·벤더 워크스페이스·udev 는 건드리지 않으므로 다른 팀원 작업에 영향이 없습니다.

## 차체가 명령을 안 먹을 때 (2026-09-23 실기)

**증상**: 바퀴가 전혀 안 돈다. 소리도 없다. 그런데 `/ros_robot_controller/imu_raw` 는
49 Hz, `battery` 는 1 Hz 로 계속 들어오고 rosout 에 에러도 없다.
`set_motor` 도 20 Hz 로 정상 발행된다.

즉 **보드 → Pi 는 멀쩡한데 Pi → 보드 만 죽은** 반이중 고장이다.

> [!warning] `/odom_raw` 로는 이 고장을 못 잡는다
> `odom_publisher` 는 cmd_vel 을 적분하는 **개루프**다. 바퀴가 멈춰 있어도 "44 cm 갔다"고
> 보고한다. 2026-09-23 에 실제로 그 값을 믿고 배터리 탓으로 잘못 짚었다.
> 주행 검증은 **탑뷰 마커나 줄자**로 실제 이동을 봐야 한다.

**진단은 부저로 한다** — 모터를 돌려 보는 것보다 안전하고 빠르다.

```bash
bash tools/ops/pi_preflight.sh --beep      # 소리가 나면 쓰기 방향 정상
```

**복구는 컨트롤러 노드만 다시 띄운다. Pi 재부팅은 필요 없다.**

```bash
bash tools/ops/rrc_recover.sh --fix        # 몇 초면 끝난다
```

원인은 보드도 배터리도 아니라 **노드의 시리얼 세션**이다. 다시 띄우면 부저·모터가 함께
살아난다(2026-09-23 확인, 그날 두 번 발생·두 번 복구). 그래도 안 울리면 그때는 보드
쪽이라 전원을 내렸다 올린다.

**왜 노드 재기동이 듣나** — 컨트롤러 로그에 `buf_write` 실패가 하나도 안 남는다. 즉 Pi 의
`port.write()` 는 정상 반환했고 보드가 받고도 무시한 것이다(SDK 의 쓰기-실패 재연결
경로는 예외가 안 나서 타지지 않는다). ttyACM 을 다시 열면 **DTR 토글로 보드 MCU 가
리셋**되므로, 전원을 내리는 것과 같은 효과를 몇 초에 낸다.

> [!danger] `/ros_robot_controller/pwm_servo/get_state` 를 호출하지 말 것
> 벤더 노드가 **즉사한다** — `TypeError: get_pwm_servo_state() takes 2 positional
> arguments but 3 were given` (콜백 시그니처 버그). 2026-09-23 에 자동 진단 수단을
> 찾다가 밟았다. 서보가 달려 있지 않아 요청·응답으로 쓰기 방향을 확인할 방법도 없으니,
> **진단은 부저로 한다.**
