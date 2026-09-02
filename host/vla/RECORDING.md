# VLA 녹화 환경

이 폴더는 **저장소가 추적하지 못하는 녹화 조건**을 담는다. 코드만 clone 해서는
같은 데이터가 안 나오는 부분이다.

```
calibration/grippers_arm.json   팔로워 캘리브레이션 (VLA 영점)
calibration/leader.json         리더 캘리브레이션
restore_env.py                  새 환경에서 위 둘을 복원하고 서보를 검증
```

## 새 환경에서 시작하기

```bash
python host/vla/restore_env.py                    # 검사만
python host/vla/restore_env.py --apply --servo COM8   # 복원
```

`--apply` 없이 돌리면 아무것도 안 쓴다. 서보에 쓸 때는 **토크가 꺼져 있어야**
하며(전원을 껐다 켜면 꺼진다), 켜져 있으면 쓰지 않고 알려준다.

## 녹화 명령

```powershell
lerobot-teleoperate `
  --robot.type=so101_follower --robot.port=COM8 --robot.id=grippers_arm `
  --robot.disable_torque_on_disconnect=false `
  --teleop.type=so101_leader --teleop.port=COM7 --teleop.id=leader `
  --robot.cameras="{ 'gripper': {'type':'opencv','index_or_path':0,'width':1280,'height':720,'fps':30,'fourcc':'MJPG','rotation':180} }" `
  --fps=30 --display_data=false
```

`lerobot-record` 도 같은 `--robot` / `--teleop` / `--robot.cameras` 를 쓴다.

## 함정 여섯 개

### ① 카메라 번호는 고정이 아니다

탑뷰 C920 을 꽂고 빼는 것만으로 그리퍼캠 번호가 바뀐다.

```
탑뷰 2대 연결  ->  그리퍼캠 2번
탑뷰 없음      ->  그리퍼캠 0번
```

**매번 확인할 것.**

```bash
python host/aruco/camera_devices.py
```

### ② 카메라 백엔드 — MSMF 가 통째로 사라질 수 있다

**2026-09-02 정정.** 예전에 여기 "CAP_DSHOW 는 720p 가 10fps 가 된다"고 적혀
있었는데, 그건 백엔드 탓이 아니라 **속성을 거는 순서** 탓이었다. 같은 카메라
같은 DSHOW 로 실측:

| 순서 | 포맷 | fps |
|---|---|---|
| 해상도 -> FOURCC | MJPG | **29.9** |
| FOURCC -> 해상도 | YUY2 | 10.0 |
| 해상도만 (fourcc 미지정) | YUY2 | 10.0 |

DSHOW 는 **해상도를 먼저 걸어야** MJPG 로 협상된다. LeRobot 은 반대로 한다
(`camera_opencv.py:202`, "FOURCC first as it can affect available FPS/resolution").
그래서 lerobot + DSHOW 조합이 10fps 가 된다.

그리고 MSMF 는 **드라이버 상태에 따라 아예 안 열릴 수 있다.** 2026-09-02 에
index 0~7 이 전부 `isOpened()=False` 였고, 같은 카메라가 DSHOW 로는 열렸다.
`lerobot-record` 는 기본이 `CAP_ANY`(윈도우에서 MSMF)라 이 상태면 녹화가 아예
시작되지 않는다.

`tools/arm/rollout_policy.py` 에는 MSMF 실패 시 DSHOW 로 떨어지고 순서를 고쳐
끼우는 코드가 들어 있다(`patch_dshow_property_order`). 녹화도 같은 증상이면
그 함수를 참고할 것.

**죽은 장치도 거를 것.** Phone Link 가상 카메라가 index 1 에 잡히는데 열리고
읽히지만 "휴대폰 연결 안 됨" 단색 그림을 준다. 프레임 `std` 가 8 미만이면 버린다.

### ③ 해상도를 낮춰야 한다면 4:3 이 아니라 16:9 로

640×480(4:3)은 **축소가 아니라 크롭**이다. 1280 폭에서 960 폭만 남기고 좌우를
버려 **가로 화각의 25%** 가 사라진다. 광각 카메라로 바꾼 의미가 없어진다.

| 요청 | 실제 | fps | 화각 |
|---|---|---|---|
| 1280×720 | 1280×720 | 30.1 | 유지 |
| **640×360** | 640×360 | 29.9 | **유지** |
| 640×480 | 640×480 | 29.9 | **잘라냄** |
| 848×480 · 1024×576 | 다른 값으로 바뀜 | 30 | 잘라냄 |

문제는 해상도가 아니라 **종횡비**다. 낮춰야 하면 `640×360`.

### ④ `--display_data=true` 는 RAM 을 먹는다

rerun 이 1280×720 RGB 를 그대로 받아 **초당 165MB** 씩 쌓는다. 실측으로 RAM
15.7GB 를 채웠고 프로세스를 강제 종료해야 했다. 화면이 필요하면
`--display_compressed_images=true` 를 같이 줄 것.

### ⑤ 캘리브레이션이 어긋나면 조용히 재캘리브레이션된다

`connect(calibrate=True)` 가 기본값이고, `is_calibrated` 는 **6축 × 3항목 = 18개
중 하나라도** 다르면 False 다. 그러면 영점이 통째로 덮어써진다. 오류도 프롬프트도
없다. `restore_env.py` 로 먼저 확인할 것.

### ⑥ 속도 상한은 lerobot 이 안 건드린다

`Maximum_Velocity_Limit` 은 서보 EEPROM 에만 있고 lerobot 이 절대 안 쓴다.
**65 로 묶여 있으면 리더를 크게 움직여도 팔로워가 굼뜨게 따라온다**(2026-09-01
실측. 리더는 250 이었다). 전원을 꺼도 안 사라지므로 증상이 계속 재현된다.

`P_Coefficient` · `Acceleration` 은 매 `connect()` 마다 6축에 같은 값이 써지므로
관절별 차이가 날 수 없다 — 원인이 아니다.

## 룩 촬영 (v6) — 2026-09-02 계획

### 왜 이렇게 찍는가

v5(퀸 15개)를 실기에 올려 본 결과가 근거다. 정책은 전 구간을 완주하지만
**파지를 못 한다**(0/5). 원인을 오프라인으로 재 보면:

```
상태를 고정하고 이미지만 에피소드별로 바꿨을 때 elbow 출력 변동   14.36도
그 변동과 정답 파지 깊이의 상관                                  r = -0.235 (설명력 6%)
```

**정책은 이미지에 강하게 반응하는데, 그 반응이 물체 위치를 뜻하지 않는다.**
조명·그림자·주변물 배치에 붙었다. 15개로는 배경을 외우는 쪽이 물체를 찾는
쪽보다 쉬운 길이었다.

그리고 v5 는 필요 없는 분포에 표본을 썼다. 자율주행차가 탑뷰로 최단거리를
계산해 **물체 앞 20cm ± 2cm** 까지 데려다 주므로 팔은 그 좁은 띠만 알면 되는데,
파지 elbow 를 -52 ~ +0.4도(폭 52.4도)로 흩뿌려 중앙 ±4도 안에 3개뿐이었다.

### 그래서

| | |
|---|---|
| 총 에피소드 | **25~30** |
| 거리 | 차체 전면에서 물체 중심까지 **20cm ± 2cm**, 격자 없이 매번 무작위 |
| 주변물 | 매 에피소드마다 별·큐브·축구공·바구니·퀸을 **전부 다른 자리로** |
| 조명 | 낮/밤·조명 on/off 를 섞는다 |
| 시작 자세 | ±5도씩 흔든다 (v5 는 첫 프레임 elbow 산포가 0.08도였다) |
| 녹화 시작 | 뻗기 직전부터 (v5 는 대기 구간이 18%였다) |

**거리를 촘촘히 나눌 필요는 없다.** ±2cm 는 좁아서 연속으로 흩뿌리면 된다.
에피소드 수가 필요한 이유는 거리 커버가 아니라 **배경을 매번 다르게 뽑아
신경망이 배경으로 답을 맞히지 못하게** 하는 것이다.

⚠️ 거리는 **자로 잰다.** 깊이 카메라 숫자로 확인하면 안 된다 — 물리적으로 같은
180mm 에 놓은 물체가 카메라 기준 queen 14.4 / rook 18.3 / soccer 25.6cm 로
읽힌 실측이 있다(`floor_grasp_profiles.GRASP_OBJECT_CENTER_FORWARD_MM` 주석).

### 명령

```powershell
lerobot-record `
  --robot.type=so101_follower --robot.port=COM8 --robot.id=grippers_arm `
  --robot.disable_torque_on_disconnect=false `
  --teleop.type=so101_leader --teleop.port=COM7 --teleop.id=leader `
  --robot.cameras="{ 'gripper': {'type':'opencv','index_or_path':0,'width':1280,'height':720,'fps':30,'fourcc':'MJPG','rotation':180} }" `
  --dataset.repo_id=lsy0284/gripper_pick_v6_rook `
  --dataset.single_task="pick up the rook" `
  --dataset.num_episodes=30 --dataset.fps=30 `
  --display_data=false
```

`single_task` 문자열이 퀸과 **달라야** 언어 조건 비교가 성립한다. ACT 는 이
문자열을 아예 안 읽고 SmolVLA 는 읽으므로, 문장이 둘이 되어야 비로소
"ACT 는 앞의 것을 집고 SmolVLA 는 구분한다"를 대조할 수 있다.

### 촬영 전 점검

```bash
python host/vla/restore_env.py                       # 캘리브레이션 확인(쓰지 않음)
python host/aruco/camera_devices.py                  # 그리퍼캠 번호 확인
python tools/align_to_idle.py --vla --dry-run        # 프레임·시작자세 확인
```

`align_to_idle` 은 이제 서보의 `Homing_Offset` 을 읽어 **기대 프레임과 다르면
거부한다**. 교시 상태에서 `--vla` 를 돌리면 wrist_roll 이 994틱(87도) 도는데,
그걸 막는다.

### 촬영 뒤 검증 — 실기에 나가기 전에

학습이 끝나면 **실기보다 먼저** 이걸 돌린다. 상태를 고정하고 이미지만 바꿔
출력 변동을 재는 것이다.

```
v5 단독   14.36도 (상관 6%)
v6        ?
```

이 값이 눈에 띄게 줄고 상관이 올라가야 "배경 의존이 줄었다"고 말할 수 있다.
안 줄었으면 실기에 나가도 같은 실패를 반복한다.

## ⚠️ 영점 충돌 — VLA 와 그리퍼 미션은 공존하지 않는다

`ros2_ws/.../floor_grasp_profiles.py` 의 RAW 상수(`HORIZONTAL_SAFE_145_RAW`,
`IDLE_CRADLE_RAW`, `CARRY_RAW` 등)는 **08-30 영점**을 전제한다. 여기 있는
캘리브레이션은 **08-31 VLA 영점**이라 같은 RAW 가 다른 물리 자세가 된다.

```
VLA 녹화   ->  host/vla/restore_env.py --apply --servo COM8
그리퍼 미션 ->  python tools/arm/backup_servo_offsets.py COM8 --restore \
                  tools/arm/servo_backup/servo_COM8_20260830_191419.json
```

**둘을 동시에 만족시킬 수 없다.** 작업을 바꿀 때마다 전환해야 한다.

## 버전

```
lerobot 0.4.4
opencv-python 4.12.0
torch 2.10.0+cpu
Python 3.11
```

여기 적힌 동작(정규화 배율, `configure()` 가 덮어쓰는 레지스터, `is_calibrated`
판정)은 **lerobot 0.4.4 의 소스를 읽고 확인한 것**이다. 버전이 다르면 다시 확인할 것.

## 진단 도구

```bash
python tools/arm/servo_regs.py                    # 6축 레지스터 비교표
python tools/arm/backup_servo_offsets.py COM8     # 서보 EEPROM 백업
```

`servo_regs.py` 는 어느 레지스터를 lerobot 이 덮어쓰는지 표에 같이 적어 준다.
관절 하나만 이상할 때 **덮어쓰는 값은 원인이 될 수 없다**는 것이 판정의 핵심이다.
