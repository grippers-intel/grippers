# Pi 접속·기동 절차 — 2026-08-28 갱신

이 문서는 원래 "LAN이 끊긴 Pi에 다시 붙는 절차"였다. 2026-08-28에 재연결이
끝나고 망까지 옮겨서, **현재 상태 기준의 기동 절차**로 다시 썼다.

명령 블록에 `#` 주석이 없는 것은 의도다 — 로컬 zsh에 그대로 붙여넣을 때
깨지기 때문이다. 설명은 전부 블록 바깥에 둔다.

---

## 0. 접속 — 주소가 바뀌었다

    이전  10.82.133.189   SSID Lemma      더 이상 유효하지 않음
    현재  192.168.0.7     SSID iptime     고정 IP

```
ssh pi@192.168.0.7
```

mDNS도 된다:

```
ssh pi@raspberrypi.local
```

못 찾으면 같은 랜에서 MAC으로 찾는다. Pi의 MAC은 `2c:cf:67:6e:e6:07`이다
(`b8:27:eb`가 아니다 — 그건 옛 라즈베리파이 재단 OUI다):

```
arp -a | grep -i 2c:cf:67
```

⚠️ ipTIME이 꺼져 있으면 Pi가 `Lemma`로 자동 복귀한다(우선순위 iptime 10 /
Lemma 5). 그때는 `Lemma` 망에서 찾아야 한다.

## 1. 컨테이너 진입

사람이 직접 (대화형, 진짜 TTY여야 한다):

```
cd ~/docker && ./exec_shell.sh
```

자동화·비대화형이면 이쪽:

```
docker exec IntelPi bash -lc '명령'
```

⚠️ 두 경로의 셸이 다르다. `exec_shell.sh`는 **zsh**(→ `setup.zsh`),
`docker exec ... bash -lc`는 **bash**(→ `setup.bash`)다.

## 2. ⚠️ 망이 바뀌었으면 여기부터 두 가지를 먼저 한다

2026-08-28 망 이전에서 실제로 둘 다 당했다. 증상이 "고장"처럼 보여서 원인을
엉뚱한 데서 찾게 된다.

### 2-1. ROS 노드를 재기동한다

이미 떠 있던 노드의 DDS 엔드포인트는 **옛 IP에 묶인 채** 남는다. 프로세스는
멀쩡히 살아 있는데 서로 못 본다.

**`ps`로 확인하면 안 된다.** 이렇게 확인한다:

```
export ROS_DOMAIN_ID=21
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 topic list
```

`/parameter_events`와 `/rosout`만 나오면 끊긴 것이다. 4절 순서로 다시 띄운다.

### 2-2. 컨테이너 DNS를 고친다

컨테이너의 `/etc/resolv.conf`는 시작 시점 사본이라 옛 망의 DNS가 박혀 있다.
`git fetch`가 `Could not resolve host: github.com`으로 죽는다.

```
docker exec IntelPi cat /etc/resolv.conf
```

호스트 값과 다르면 맞춰 준다:

```
docker exec -u root IntelPi bash -c 'printf "nameserver 203.248.252.2\nnameserver 164.124.101.2\n" > /etc/resolv.conf'
```

컨테이너를 재시작하면 Docker가 호스트 값으로 다시 만든다.

## 3. 코드 상태 확인

2026-08-28 기준 Pi는 **`10e4734`로 원격과 동기**되어 있다.

```
cd /grippers
git fetch --all
git status -sb
git log --oneline -3
```

받을 것이 있으면:

```
git pull --ff-only
```

`ros2_ws/src/` 안이 바뀌었으면 리빌드한다:

```
cd /grippers/ros2_ws
export ROS_DOMAIN_ID=21
source /opt/ros/humble/setup.bash
colcon build --packages-select grippers_mission
```

로컬 수정이 있어 `git pull`이 막히면, 지우기 전에 무엇인지 반드시 본다:

```
git status
git diff
```

## 4. 도메인 테스트부터 (하드웨어 안 켜고)

```
cd /grippers
python3 -m pytest -q tests
```

**443개 통과**가 기준이다(2026-08-28). 여기서 깨지면 하드웨어를 켜기 전에
먼저 본다 — 실기에서 원인을 찾는 것보다 여기가 훨씬 싸다.

## 5. 노드 기동

이전 프로세스를 먼저 정리한다. ⚠️ `pkill -f "ros2 run grippers"`처럼
뭉뚱그리지 말 것 — 자기가 띄운 다른 노드까지 죽는다. PID로 골라 죽인다:

```
ps aux | grep -E "perception_node|arm_driver|odom_publisher|depth_cam_rotate|mission_orchestrator|battery_warn" | grep -v grep
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
ros2 topic list | grep -E "battery|imu"
```

`perception_node`가 `/grippers/models/best.pt`(train-9)를 물었는지 본다.
배포·재시작 뒤에는 `perception_node`를 **반드시** 다시 띄운다 —
`depth_cam_rotate_node`도 같이 떠 있어야 한다.

## 6. 전원 확인

```
ros2 topic echo /ros_robot_controller/battery --once
```

| | 전압 | 뜻 |
|---|---|---|
| 경고 | **7150 mV 무부하** | 이 아래로는 미션을 시작하지 않는다 |
| 실측 정지 | 6944 mV 부하 | 모터가 안 돈다 |
| 리튬 무릎 | 7000 mV (셀당 3.50 V) | 아래로 급락 |

⚠️ **부하 중에 재면 안 된다.** 멀쩡한 팩도 회전 중에는 7122 mV를 찍는다.

⚠️ **팔은 별도 전원이다.** 베이스 7.9 V / 팔 12.1 V (2026-08-28 실측). 위
문턱은 **베이스에만** 해당한다.

저전압 경고 노드(LED 5초 점멸)는 부팅 시 자동으로 뜬다. 수동으로 띄우려면:

```
docker exec -d IntelPi bash -lc "/shared/battery_warn/run_battery_warn.sh"
```

## 7. 미실기 코드 확인 — INSERT 좌우 오프셋 게이팅

`28d4626`이 실기로 **한 번도 안 돌아가 봤다.** 좌우 오프셋이 허용치를 넘는
상황을 실제로 만들어, ⛔ 분기가 팔을 안 펼치고 중단하는지 봐야 한다.

정상 케이스부터:

```
cd /grippers
python3 tools/basket_approach_insert_test.py --profile queen
```

그다음 바구니를 옆으로 밀어 오프셋을 만들고 같은 명령을 다시 돌린다.
기대: 팔이 안 펴지고 ⛔ 메시지와 함께 중단.

⚠️ 오프셋이 23mm보다 작으면 라이다가 **구조적으로 못 잰다**. 허용치가
70mm이므로 100mm 이상 밀 것.

## 8. Host 연동

Host PC는 **`192.168.0.2`**다. Pi와 같은 `iptime` 망에 있다.

먼저 맥에서 규격 적합성을 본다:

```
cd ~/Desktop/intel/grippers
python3 tools/host_link_conformance.py --as-is
```

2026-08-28 기준 Host 코드(`grippers/host/`)는 **5/6**이다. 유일한 실패는
`BASKET_APPROACH_MPS`(0.06)를 Host가 안 갖고 있는 것 — 차는 움직이지만 Host의
도착 예측이 어긋난다.

미션 노드 기동:

```
ros2 run grippers_mission mission_orchestrator --ros-args \
  -p use_fake_base:=false -p use_fake_arm:=false \
  -p use_fake_perception:=false -p use_fake_host:=false \
  -p host_ip:=192.168.0.2 > /tmp/mission.log 2>&1 &
sleep 3
ros2 topic echo /mission/state --once
```

`IDLE`이 나오면 정상이다.

### ⚠️ Host PC 방화벽이 인바운드를 드롭한다

2026-08-28 확인. Pi에서 본 상태:

    ping                 100% 손실
    ARP                  REACHABLE (d8:bb:c1:a9:cf:96) — 랜에는 있다
    UDP 5006             무응답
    UDP 5999 (대조군)     무응답

아무도 안 듣는 5999조차 ICMP port unreachable을 안 돌려준다 = 방화벽이 조용히
버리고 있다. 즉 **Pi → Host 보고(5006)가 막힐 수 있다.**

Host → Pi(5005)는 문제없다. Pi에는 방화벽이 없고(`iptables INPUT ACCEPT`),
컨테이너가 `NetworkMode=host`라 포트 게시도 필요 없다.

연동 당일 **가장 먼저** 확인할 것:

1. Host 프로그램을 띄운다(Windows가 방화벽 허용을 물으면 허용)
2. Pi에서 보고를 한 발 보낸다
3. Host가 실제로 받는지 Host 화면에서 확인한다

받지 못하면 Host 쪽에서 UDP 5006 인바운드 허용 규칙을 추가해야 한다.

### 실경로 왕복은 이미 확인됐다

맥을 Host 자리에 놓고 실제 `UdpHostLink`로 왕복시킨 결과(2026-08-28):

    Host->Pi 5005 : {"state":"APPROACH_BOX","linear_x":0.06,...}
    Pi 파싱       : HostCommand(state='APPROACH_BOX', linear_x=0.06, ...)
    Pi->Host 5006 : {"report":"INSERT_READY","state":"APPROACH_BOX",...}

새 망 + 컨테이너 host 모드 + 진짜 파싱 코드는 전부 통과한다. **남은 변수는
Host PC 방화벽뿐이다.**

## 9. 제자리 회전 — 검증 완료

이전 판에서 "가장 먼저 확인할 것"으로 적어 둔 항목이다. **2026-08-28에
실측으로 확인됐다.**

| 명령 rad/s | 자이로 실측 | 실측/명령 | 회전 |
|---|---|---|---|
| 0.25 | 0.2114 | 0.845 | ✅ |
| 0.20 | 0.1424 | 0.712 | ✅ |
| 0.15 | 0.0906 | 0.604 | ✅ |

`AGREED_ROTATION_RAD_S = 0.25`는 실제로 돈다. 정지마찰은 문턱이 아니라
기울기였다 — 명령이 낮을수록 덜 돈다. 판정은 `/odom_raw`가 아니라 **IMU
자이로**로 했다(전자는 명령을 적분할 뿐이다).

횡이동(`linear_y`)도 같은 날 확인했다 — **0.03 m/s까지 동작**한다.

## 10. 마무리

Pi 홈에 정리 전 백업이 남아 있다(164MB). 지울지는 사용자 판단:

```
ls -lh ~/grippers_worktree_backup_20260826_1240.tgz
```

세션을 끝낼 때 로그를 맥으로 내려 둔다:

```
scp pi@192.168.0.7:/tmp/grasp_test_log_*.jsonl ~/Downloads/
```
