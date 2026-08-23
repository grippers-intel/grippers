# tools/teleop — 시연 영상용 수동 조종

자율주행 시나리오를 **사람이 그대로 재현**하기 위한 텔레옵 묶음이다.
한 손은 SO-101 리더 암, 다른 손은 키보드. 전 과정은 rosbag2로 남는다.

## 실행

노트북에서 이것 하나면 된다. 파이 쪽 프로세스는 스크립트가 띄우고 종료할 때 같이 정리한다.

```bash
cd ~/grippers-teleop && ./teleop.sh
```

| 명령 | 하는 일 |
|---|---|
| `./teleop.sh` | 베이스 스택 + 팔 수신기 기동 후 조종 시작 |
| `./teleop.sh --record` | 위에 더해 rosbag2 녹화 |
| `./teleop.sh --check` | 실행하지 않고 준비 상태만 점검 |
| `./teleop.sh --status` | 지금 뭐가 돌고 있는지 |
| `./teleop.sh --stop` | 파이에 남은 프로세스 정리 |
| `./teleop.sh --arm-only` | 베이스 없이 팔만 (벤치 테스트) |

## 키

| 키 | 동작 | 키 | 동작 |
|---|---|---|---|
| **리더 암** | 손으로 움직이면 팔로워가 따라옴 | `f` | 팔 추종 켜기/끄기 (켜는 순간이 기준점) |
| `w`/`s` | 전진/후진 | `a`/`d` | 좌/우 평행이동 (메카넘) |
| `q`/`e` | 좌/우 제자리회전 | `SPACE` | 베이스 즉시 정지 |
| `z`/`x` | 속도 -/+ | `Ctrl-C` | 종료 (파이 쪽도 정리) |

## 구조

```
노트북(맥)                          라즈베리파이 5 (IntelPi 컨테이너, network=host)
┌────────────────────┐             ┌──────────────────────────────────────┐
│ leader_teleop.py   │  UDP 47800  │ follower_teleop_node.py              │
│  리더 암 50Hz 읽기 ├────────────>│  팔    → /dev/soarm 서보             │
│  + 키보드(베이스)  │  IPv6       │  베이스 → /cmd_vel → 컨트롤러        │
└────────────────────┘             │  상태  → /teleop/* → rosbag2         │
                                   └──────────────────────────────────────┘
```

리더를 파이에 USB로 직결하지 않는 이유는 성능이 아니라 **시나리오**다. 파이는 움직이는
베이스 위에 있어서, 케이블이 로봇을 따라 아레나 바닥을 쓸며 흩뿌린 물체를 밀어낸다.

실측: **50.0Hz, 10초간 패킷 유실 0%, 간격 중앙값 19.9ms / 최대 36.2ms.**

## 최초 1회 준비

### 노트북

```bash
cd ~/grippers-teleop
uv venv --python 3.12 && uv pip install pyserial
```

`driver_sdk.py`, `teleop_protocol.py`, `leader_teleop.py`, `teleop.sh` 를 이 디렉터리에 둔다.
리더 암은 macOS가 드라이버 없이 인식한다(`ls /dev/cu.usbmodem*`).
`ssh pi` 가 비밀번호 없이 되어야 한다(키 등록 완료 상태).

### 팔로워 암 서보 전원 ⚠️

USB는 서보 보드의 **로직만** 먹인다. 암의 서보 전원 라인이 따로 들어가야 서보가 응답한다.
`./teleop.sh --check` 가 이걸 잡아준다. 직접 보려면:

```bash
docker exec IntelPi python3 /grippers/tools/teleop/scan_ids.py   # [1,2,3,4,5,6] 이어야 정상
```

## 왜 델타(상대) 추종인가

두 팔 모두 `calibration.json` 이 없어 같은 서보 카운트가 서로 다른 물리 각도를 뜻한다.
그래서 `f` 를 누르는 순간의 리더/팔로워 자세를 각각 기준점으로 잡아 그 뒤의 변화량만 전달한다.

부수 효과가 본래 목적보다 중요하다 — 절대 추종이었다면 리더와 팔로워 자세가 어긋난 상태에서
`f` 를 누르는 즉시 팔이 최대 속도로 날아간다. 델타 방식은 정의상 그 순간의 오차가 0이다.

## 안전 장치

| 장치 | 동작 |
|---|---|
| 데드맨 | 0.4초간 패킷이 없으면 **베이스는 즉시 정지**, **팔은 토크를 켠 채 자세 유지**. 팔에서 토크를 끄면 잡고 있던 물건과 함께 떨어진다 |
| 슬루 제한 | 패킷당 관절 최대 80카운트(50Hz 기준 ≈350°/s). 패킷 유실 뒤 큰 점프를 흡수 |
| 델타 상한 | 기준점에서 1400카운트를 넘는 목표는 통신 오류로 보고 버린다 |
| 관절 한계 | `JOINT_LIMITS` 창으로 클램프. 미보정 관절 1~5는 사실상 통과이며, 이때 실제 한계는 **리더 암의 기구적 스토퍼**가 대신 잡는다 |
| 종료 시 | 베이스 정지를 먼저 내고, 팔은 토크를 유지해 자세를 붙든다. 내리려면 `--relax-on-exit` |

## 시연 중 주의

- `arm_driver_node` 와 텔레옵 수신기는 **같은 시리얼 버스를 동시에 열 수 없다.**
  bringup 을 함께 쓸 경우 `use_fake_arm:=true` 가 필수다.
- `grippers_mission` 도 `/cmd_vel` 에 publish 하므로 텔레옵과 같이 띄우면 경쟁한다.
- 베이스 스택은 `controller.launch.py` 가 아니라 `odom_publisher.launch.py` 를 띄운다.
  전자는 `imu_filter` → **`imu_calib` 패키지**를 포함하는데 이 컨테이너에 없어서 런치 전체가
  SIGINT 로 죽는다. 텔레옵에는 IMU 필터·EKF 가 필요 없다.
- 파이 재부팅으로 IPv6 링크로컬이 바뀌면:
  `ping6 -c 2 ff02::1%en0 >/dev/null; ndp -an | grep -i '2c:cf:67'`
  찾은 주소를 `~/.ssh/config` 에 넣을 때 `%en0` → `%%en0` 로 이스케이프한다.

## 알려진 제약

- **미보정** — 관절 1~5에 소프트웨어 한계가 없다. 보정을 넣으면 `driver_sdk` 의
  `window_fraction()` / `position_from_fraction()` 경로로 절대 추종 전환도 가능하다.
- `/teleop/arm_joint_states` 의 각도는 "서보 2048 = 0도" 가정의 **근사값**이다.
  정확한 값이 필요하면 같이 녹화되는 `*_counts` 원시 카운트를 써라.
- 그리퍼(id6)는 리더와 팔로워의 가동 카운트 폭이 다를 수 있다. 개폐 범위가 안 맞으면 `--gain`.
- 루트 파티션 여유가 작다(94% 사용). `record_demo.sh` 는 2GB 미만이면 중단하고,
  기본 토픽 집합에서 원본 이미지를 뺀다.

## 진단 도구

```bash
.venv/bin/python probe_leader.py                                    # 리더 (노트북)
docker exec IntelPi python3 /grippers/tools/teleop/probe_follower.py # 팔로워 (파이)
docker exec IntelPi python3 /grippers/tools/teleop/scan_ids.py       # ID 스캔
docker exec IntelPi python3 /grippers/tools/teleop/udp_sink.py       # 전송로만 측정
docker exec IntelPi python3 /grippers/tools/teleop/test_follower_logic.py  # 로직 검증
```
