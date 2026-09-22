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

`--symlink-install` 이어야 `arm_poses.yaml`·`robot.yaml` 을 고쳤을 때 재빌드 없이 반영됩니다
(`teach_pose.py` 가 소스의 yaml 을 고칩니다).

## 4. 기동

```bash
docker exec -it IntelPi bash -lc '
  export ROS_DOMAIN_ID=21 &&
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&                      # 벤더: controller · ros_robot_controller
  source /grippers/vla_deploy/vla_robot/ros2_ws/install/setup.bash &&
  bash /grippers/vla_deploy/vla_robot/tools/ops/pi_preflight.sh &&
  ros2 launch vla_robot_bringup robot.launch.py'
```

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
