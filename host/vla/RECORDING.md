# v4 녹화 환경

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

### ② `CAP_DSHOW` 로 열면 720p 가 10 fps 가 된다

이 저장소의 탑뷰 코드는 `cv2.CAP_DSHOW` 를 쓰지만 **lerobot 은 `CAP_ANY`**
(윈도우에서 MSMF)를 쓴다. 같은 카메라가 백엔드에 따라 다르게 동작한다.

```
CAP_DSHOW  1280x720  ->  10.0 fps   (MJPG 요청이 무시되고 YUY2 로 잡힘)
CAP_ANY    1280x720  ->  30.1 fps   (MJPG 있으나 없으나)
```

**직접 잰 fps 가 이상하면 백엔드를 먼저 의심할 것.** DSHOW 로 재고 "이 카메라는
720p 30fps 가 안 나온다"고 결론 내리면 틀린다.

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
