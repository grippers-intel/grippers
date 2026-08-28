# Pi 재연결 직후 절차 — 2026-08-28 기준

LAN이 끊긴 채 세션이 두 번 끝나 Pi가 `c3a2bb1`에 멈춰 있다. 다시 붙는 순간
헤매지 않도록, **위에서부터 순서대로** 붙여넣기만 하면 되게 적었다.

명령 블록에 `#` 주석이 없는 것은 의도다 — 로컬 zsh에 그대로 붙여넣을 때
깨지기 때문이다. 설명은 전부 블록 바깥에 둔다.

---

## 0. 접속

맥에서:

```
ssh pi@raspberrypi.local
```

mDNS 이름이 안 잡히면(지난 세션에 실제로 그랬다) IP를 직접 준다. 과거 기록은
`10.82.133.189`인데 DHCP라 바뀌었을 수 있다. 같은 랜에서 찾으려면:

```
ping -c 2 raspberrypi.local
arp -a | grep -i b8:27:eb
```

## 1. 컨테이너 진입

사람이 직접 (대화형, 진짜 TTY여야 한다):

```
cd ~/docker && ./exec_shell.sh
```

자동화·비대화형이면 `exec_shell.sh`가 `cannot attach stdin to a TTY-enabled
container`로 실패한다. 그때는 이쪽:

```
docker exec IntelPi bash -lc '명령'
```

⚠️ 두 경로의 셸이 다르다. `exec_shell.sh`는 **zsh**(→ `setup.zsh`),
`docker exec ... bash -lc`는 **bash**(→ `setup.bash`)다.

## 2. 코드 받기 — 다섯 커밋

```
cd /grippers
git fetch --all
git status -sb
git pull
git log --oneline -6
```

`aca9d75`가 아니라 **`80448c9`**까지 와야 한다(2026-08-28 기준 최신).
받아야 할 것:

| 커밋 | 내용 | 리빌드 |
|---|---|---|
| `28d4626` | INSERT 좌우 오프셋 게이팅 | 불필요 |
| `dfac702` | 낡은 함수명 참조 수정 | 불필요 |
| `4db7ec3` | 죽은 bbox 휴리스틱 삭제 | 불필요 |
| `aca9d75` | MAX_GRASP_RETRY 정리, 시도 횟수 보고 | 불필요 |
| `deeecf0` | 문서 갱신 | 불필요 |
| `aa43e13` | 통주행 테스트 추가 | 불필요 |
| `80448c9` | `elapsed_s` 미션별로 | **필요** (`grippers_mission`) |

`80448c9`만 `ros2_ws/src/` 안이라 리빌드가 필요하다:

```
cd /grippers/ros2_ws
export ROS_DOMAIN_ID=21
source /opt/ros/humble/setup.bash
colcon build --packages-select grippers_mission
```

로컬 수정이 있어 `git pull`이 막히면 먼저 확인한다. 지우기 전에 무엇인지
반드시 본다:

```
git status
git diff
```

## 3. 도메인 테스트부터 (하드웨어 안 켜고)

```
cd /grippers
python3 -m pytest -q tests
```

**433개 통과**가 기준이다. 여기서 깨지면 하드웨어를 켜기 전에 먼저 본다 —
실기에서 원인을 찾는 것보다 여기가 훨씬 싸다.

## 4. 노드 기동

이전 프로세스를 먼저 정리한다. ⚠️ `pkill -f "ros2 run grippers"`처럼
뭉뚱그리지 말 것 — 자기가 띄운 다른 노드까지 죽는다. PID로 골라 죽인다:

```
ps aux | grep -E "perception_node|arm_driver|odom_publisher|depth_cam_rotate|mission_orchestrator" | grep -v grep
```

환경 (bash 경로 기준):

```
export ROS_DOMAIN_ID=21
export need_compile=False
export DEPTH_CAMERA_TYPE=ascamera
export MACHINE_TYPE=MentorPi_Mecanum
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash
```

기동:

```
ros2 launch controller odom_publisher.launch.py > /tmp/odom.log 2>&1 &
ros2 launch peripherals depth_camera.launch.py > /tmp/depth_cam.log 2>&1 &
sleep 8
ros2 run grippers_perception depth_cam_rotate_node > /tmp/rotate.log 2>&1 &
ros2 run grippers_perception perception_node > /tmp/perception.log 2>&1 &
ros2 run grippers_arm arm_driver --ros-args -p enable_torque_on_start:=true > /tmp/arm.log 2>&1 &
sleep 3
ros2 node list
```

확인:

```
grep -i "best.pt\|model" /tmp/perception.log | tail -5
ros2 topic hz /scan_raw --window 10
```

`perception_node`가 `/grippers/models/best.pt`(train-9)를 물었는지 본다.
배포·재시작 뒤에는 `perception_node`를 **반드시** 다시 띄운다 —
`depth_cam_rotate_node`도 같이 떠 있어야 한다.

## 5. 미배포 코드 실기 확인 — INSERT 좌우 오프셋 게이팅

`28d4626`이 실기로 **한 번도 안 돌아가 봤다.** 좌우 오프셋이 허용치를 넘는
상황을 실제로 만들어, ⛔ 분기가 팔을 안 펼치고 중단하는지 봐야 한다.

정상 케이스부터:

```
cd /grippers
python3 tools/basket_approach_insert_test.py --profile queen
```

그다음 바구니를 옆으로 밀어 오프셋을 만들고 같은 명령을 다시 돌린다.
기대: 팔이 안 펴지고 ⛔ 메시지와 함께 중단.

⚠️ 오프셋이 23mm보다 작으면 라이다가 **구조적으로 못 잰다**(방위각 창 안에
바구니 양쪽 가장자리가 안 걸린다). 게이팅을 보려면 충분히 크게 밀어야 한다 —
허용치가 70mm이므로 100mm 이상 밀 것.

## 6. 실제 주행 준비

`mission_orchestrator`가 지난 세션에 `use_fake_base:=true`로 떠 있었다.
실제로 바퀴를 돌리려면 다시 띄운다:

```
ps aux | grep mission_orchestrator | grep -v grep
```

기존 PID를 죽인 뒤:

```
ros2 run grippers_mission mission_orchestrator --ros-args \
  -p use_fake_base:=false -p use_fake_arm:=false \
  -p use_fake_perception:=false -p use_fake_host:=false \
  -p host_ip:=<Host PC의 IP> > /tmp/mission.log 2>&1 &
sleep 3
ros2 topic echo /mission/state --once
```

`IDLE`이 나오면 정상이다.

## 7. Host 연동 (Host 쪽 반영이 끝난 뒤에만)

Host 저장소가 2026-08-26 확정 규격을 아직 안 따르고 있으면 **차량이 한 번도
안 움직인다.** 붙이기 전에 맥에서 먼저 확인한다:

```
cd ~/Desktop/intel/grippers
python3 tools/host_link_conformance.py --as-is
```

**5/5**가 나와야 붙일 수 있다. 2/4면 Host 쪽이 아직 안 고쳐진 것이다 —
`grippers_docs/grippers_host_requests_20260827.md`를 Host 팀에 전달한다.

5/5가 나오면 실기 루프백에서 볼 것 네 가지:

1. Pi가 5005에서 Host JSON을 파싱하는가 (`/tmp/mission.log`에 파싱 경고가 없어야)
2. Host가 5006에서 Pi 보고를 받는가
3. 명령을 끊으면 Pi가 3사이클(0.3초) 안에 멈추는가
4. 회전+병진 혼합에 `REJECTED`가 오는가

## 8. 실기에서 처음 확인할 것 — 제자리 회전

⚠️ **`AGREED_ROTATION_RAD_S = 0.25`는 실측으로 검증된 적이 없는 값이다.**

2026-08-24 실측(`tools/inplace_rotation_test.py`, 사람이 눈으로 판정)은
1.2 / 0.8 / 0.6 / 0.5 / 0.4 / 0.355 / 0.3을 시험해 전부 돌았다 — 문턱은
0.3보다 낮지만 **어디까지 낮은지는 안 재봤다.** 0.25는 그 아래다.

Host의 `DriveSequencer`는 모든 방위 정렬을 제자리 회전으로 한다. 안 돌면
Host가 `yaw+`를 영원히 보내며 수렴하지 않는다. 그리고 `/odom_raw`는 명령을
적분할 뿐이라 **회전 실패를 못 잡는다** — 눈으로 봐야 한다.

붙이기 전에 0.25에서 실제로 도는지부터 확인한다:

```
cd /grippers
python3 tools/inplace_rotation_test.py --topic controller/cmd_vel
```

0.25를 입력해 보고, 안 돌면 도는 최저값을 찾아 팀에 알린다 — 이 값은 Host와
Pi가 같이 쓰는 합의 상수라 한쪽만 바꾸면 안 된다.

## 9. 마무리

Pi 홈에 정리 전 백업이 남아 있다(164MB). 지울지는 사용자 판단:

```
ls -lh ~/grippers_worktree_backup_20260826_1240.tgz
```

세션을 끝낼 때 로그를 맥으로 내려 둔다:

```
scp pi@raspberrypi.local:/tmp/grasp_test_log_*.jsonl ~/Downloads/
```
