# grippers-host-mac

`grippers` 프로젝트의 **Host PC 코드를 macOS(Apple Silicon)에서 돌아가게** 옮긴
것이다. 원본은 Windows 전용이었다.

원본: `kica927/grippers` 의 `host/` (PR #44, `sysy009/host-topview-merge` — 팀 origin에는
병합되지 않았고 sysy009 fork 브랜치에만 있다).

프로젝트 전체 개요·FSM·하드웨어 구성은 메인 저장소(Pi 쪽) 참고 —
[`kica927/grippers`](https://github.com/kica927/grippers) `kica927/baseline_mission` 브랜치, 특히
[`docs/design/state_machine.md`](https://github.com/kica927/grippers/blob/kica927/baseline_mission/docs/design/state_machine.md).
이 저장소는 Host(관제 콘솔)의 macOS 포팅 세부사항만 다루고 중복 설명하지 않는다.

---

## 결론부터

**돌아간다.** 막힐 줄 알았던 지점(OpenVINO/geti-sdk 가 Apple Silicon 을
지원하는가)이 실제로는 문제가 아니었다.

2026-08-28, macOS 15.7.7 / arm64 (Apple M1 Pro) 실측:

```
openvino 2025.4.1        available_devices = ['CPU']
                         FULL_DEVICE_NAME  = Apple M1 Pro
geti-sdk 2.13.1          import OK
opencv 4.14.0            CAP_AVFOUNDATION = 1200
카메라                    system_profiler 열거 OK, 인덱스 0 실제로 열림
vehicle_link             import OK, 합의 상수 정상 (0.1 / 0.25)
geti 추론                 ✅ 실제로 검출됨 (아래)
```

### geti 추론이 Apple Silicon 에서 실제로 돈다

`model.bin` 을 구해 붙여서 확인했다. 배관만 도는 것이 아니라 **검출 결과가
제대로 나온다.**

```
_annotated.jpg   star:0.86
00001.jpg        star:0.87, rook:0.83, soccer:0.79
00002.jpg        star:0.84, rook:0.81, soccer:0.79
```

**그리고 Windows 보다 빠르다.**

| | 추론 1회 |
|---|---|
| Host 팀 Windows CPU | 364 ms |
| Host 팀 Windows iGPU | 250 ms |
| **Apple M1 Pro (CPU)** | **130 ms  (7.7 Hz)** |

모델 로드 3.4초. Host 팀이 검출 불일치 때문에 iGPU 를 버리고 CPU 로 확정했는데,
M1 Pro 의 CPU 가 그 iGPU 보다도 빠르므로 **디바이스 선택 문제 자체가 없다.**

---

## 무엇을 고쳤나

### 1. 카메라 백엔드 — `host/camera_backend.py` (신규)

원본은 여덟 군데에서 `cv2.VideoCapture(i, cv2.CAP_DSHOW)` 를 직접 부른다.
DirectShow 는 Windows 전용이다.

**증상이 조용해서 위험하다.** macOS 에도 `cv2.CAP_DSHOW` 상수는 있고(값 700)
예외도 안 난다 — `isOpened()` 가 False 를 돌려줄 뿐이다. 카메라가 없는 것과
구별이 안 된다.

플랫폼 분기를 한 곳에 모았다.

| 플랫폼 | 백엔드 |
|---|---|
| Windows | `CAP_DSHOW` (원본 그대로) |
| macOS | `CAP_AVFOUNDATION` |
| Linux | `CAP_V4L2` (참고용) |

**Windows 동작은 안 바뀐다.** 같은 백엔드를 고르므로 원본과 동일하다.

### 2. 장치 열거 — `host/aruco/camera_devices.py`

원본은 DirectShow COM 인터페이스를 `ctypes` 로 직접 호출해 카메라 이름을
읽는다(`list_video_devices`). Windows 에서는 그 열거 순서가 곧 `cv2` 인덱스라
이름으로 카메라를 고를 수 있었다.

macOS 는 `system_profiler -json SPCameraDataType` 으로 읽는다. 공개 API
(`find_indices` / `resolve_indices` / `names_of` / `report`)는 그대로 두고
가장 아래 한 층만 갈아 끼웠다.

### 3. `vehicle_link.py` — **한 줄도 안 고쳤다**

그 파일은 규격을 문서에서 베끼지 않고 `domain/ports/baseline_ports.py` 와
`domain/task/motion.py` 를 직접 import 한다. 좋은 설계라 그대로 살렸다.

그래서 이 저장소는 **원본과 같은 레이아웃**(`repo/host/`, `repo/domain/`)을
유지한다. `vehicle_link.py` 의 `Path(__file__).parent.parent` 가 그대로
맞아떨어진다.

---

## ⚠️ macOS 에서 달라지는 것

### 초점을 고정할 수 없다 — 정확도에 영향

원본 `open_camera()` 는 오토포커스를 끄고 초점을 고정한다. C920 은 초점이
움직이면 초점거리가 같이 변해서, 캘리브레이션해 둔 내부 파라미터가 그 순간부터
틀린 값이 되기 때문이다.

**OpenCV 의 AVFoundation 백엔드는 `CAP_PROP_AUTOFOCUS`/`CAP_PROP_FOCUS` 를
지원하지 않는다.** `cap.set()` 이 False 를 돌려주고 아무 일도 안 일어난다.

`camera_backend.lock_focus()` 는 이것을 조용히 넘기지 않고 경고를 돌려주며,
`camera_devices.open_camera()` 가 그 경고를 stderr 에 한 번 찍는다.

**ArUco 위치 정확도가 Windows 만큼 안 나올 수 있다.** 회피책은 카메라 쪽에
있다 — C920 은 웹캠 유틸리티나 UVC 명령으로 초점을 미리 고정할 수 있고, 한 번
고정하면 OpenCV 가 안 건드린다.

### 카메라 권한이 필요하다

TCC(개인정보 보호)가 막으면 OpenCV 는 예외를 안 던지고 이렇게만 찍는다.

```
OpenCV: not authorized to capture video (status 0)
```

**이식 중에 실제로 이걸 만났다.** 모르면 포팅이 깨진 줄 안다.

시스템 설정 > 개인정보 보호 및 보안 > 카메라 에서 실행 주체(터미널/iTerm/
VS Code)를 켤 것. 목록에 없으면 카메라를 한 번 열어 본 뒤 다시 볼 것 —
시도해야 목록에 생긴다.

진단:

```
python3 -c "import sys; sys.path.insert(0,'host'); import camera_backend as c; print(c.diagnose())"
```

### 장치 인덱스 순서를 못 믿는다

Windows 는 DirectShow 열거 순서 = `cv2` 인덱스였다. macOS 에는 그런 보장이
없다. 이름으로 못 찾으면 `camera_backend.probe_indices()` 로 실제로 열어 보고
확인할 것.

---

## `model.bin` 은 이 저장소에 없다

`host/geti_sdk-deployment/` 에는 `model.xml`(1.3MB)만 들어 있다. 가중치
`model.bin`(85MB)은 원본 저장소가 ignore 하고 있고 LFS 로 올릴지 별도 배포로
뺄지 **아직 미정**이라, 그 결정을 앞질러 가지 않으려고 여기서도 ignore 한다.

위 검증은 가중치를 따로 받아 아래 자리에 놓고 했다.

```
host/geti_sdk-deployment/deployment/Detection/model/model.bin
```

같은 모델인지는 해시로 확인할 수 있다 — `model.xml` 과 `config.json` 이
저장소 것과 일치해야 한다.

## 웹캠 두 대 — 실기 확인 완료. 그리고 내 첫 판단이 틀렸다

C920 두 대를 USB 허브에 붙여 확인했다(2026-08-28).

**결론: macOS 에서는 이름으로 카메라를 고를 수 없다. `CAM_INDICES` 가 맞다.**

처음에는 반대로 판단했다. 열거 결과가 이렇게 나와서

```
AVFoundation / system_profiler :  [0] FaceTime  [1] C920  [2] C920
```

"`CAM_INDICES = (0, 1)` 은 내장 카메라를 잡는 틀린 값이고, 이름으로 골라야
한다" 고 적고 그렇게 구현했다. **실제로 열어 보니 정반대였다.**

```
실제 cv2.VideoCapture           :  [0] C920  [1] C920  [2] FaceTime
```

OpenCV 는 외장을 먼저 놓는다. 그래서 원본의 `(0, 1)` 이 이 맥에서도 맞는
값이었고, 내 "이름 기반" 코드가 `[1, 2]`(C920 한 대 + 내장 카메라)를 골라
**오른쪽 화면에 사람 얼굴이 나왔다.**

`AVCaptureDeviceDiscoverySession` 에 외장 타입을 먼저 요구해도 macOS 가 제
순서대로 돌려주므로, **열거 결과로는 cv2 인덱스를 복원할 수 없다.** 그래서
`list_video_devices()` 는 macOS 에서 빈 목록을 돌려주고, 호출부가
`config.CAM_INDICES` 로 떨어지게 했다.

인덱스를 확정하는 유일하게 확실한 방법은 찍어 보는 것이다.

```
python3 -c "import sys; sys.path.insert(0,'host'); import camera_backend as c; print(c.save_shots())"
```

`/tmp/camshot/idx*.png` 를 눈으로 보고 `config.CAM_INDICES` 를 정한다.

### run_mission.py 실기 결과

```
python3 run_mission.py --cams 0 1 --show-cams --display 1 --cam-width 900 --mock-complete
```

```
[display] 화면 1 origin=(1352, 0) 에 카메라 창 2개 (900x506)
[SEARCH_TARGET] x=1346.0mm y=661.8mm yaw=88.3deg cams=2
[APPROACH_PIECE] target=rook -> [GRASP] -> [CARRY_TO_DEST]
[hz] 10.14 Hz (99 ms/사이클)  캡처+ArUco 49  geti 1  FSM 4  화면 21 ms
```

카메라 두 대로 로봇 포즈를 잡고 FSM 이 전이한다. **Windows 보다 빠르다.**

| | 루프 |
|---|---|
| Host 팀 Windows | 7.0 Hz (143 ms) |
| macOS / M1 Pro | **9.6~10.1 Hz (99~104 ms)** |

Host 팀은 143 ms 중 90 ms 가 화면 렌더라고 했는데 여기서는 21 ms 다.

`CARRY_TO_DEST` 에서 멈춰 있는 것은 정상이다 — `--mock-complete` 는 차량이
없는 상태라 로봇이 실제로 안 움직여서 목적지에 영영 도착하지 않는다.

### 창 크기와 위치

`--display 1` 로 확장 화면에, `--cam-width` 로 크기를 정한다. 지도 크기는
`LIVEMAP_SIZE_IN` 환경변수로 조절한다(기본 9인치, 원본은 6인치였다).

## 설치

```
brew install python@3.11
/opt/homebrew/opt/python@3.11/bin/python3.11 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

파이썬 3.11 고정은 원본과 같은 이유다 — 3.13 이상에서는 pip 이 옛 geti-sdk
2.1.0 으로 되돌아가 깨진다. 실제로 깔린 전체 목록은
`requirements.lock.macos.txt` 참고.

## Pi 실기 테스트 준비 — "테스트 준비" 매크로

"테스트 준비"라고 하면 Pi의 `tools/ops/test_ready.sh` 딱 하나만 실행한다(이
저장소가 아니라 Pi 쪽 `grippers` 체크아웃에 있는 스크립트다 — 실기 전체 흐름의
일부라 여기 운영 문서로 같이 적어 둔다). 그 전까지는 코드 배포 확인 -> EEPROM
비교 -> bringup을 매번 손으로 하나씩 ssh 왕복하며 했고 bringup 쪽에서만 env·
워크스페이스·패키지명 문제로 세 번 헤맸다(그 root cause는 아래 3단계에 이미
반영돼 있다). 이 스크립트는 그 세 단계를 이어 붙인 것뿐이고 **kill은 전혀
하지 않는다** — 그래서 사람 확인 없이 바로 실행해도 안전하다.

```
ssh pi@<라즈베리파이 IP 또는 호스트> 'bash ~/docker/shared/grippers/tools/ops/test_ready.sh'
```

git 동기화는 컨테이너 밖 Pi 호스트에서, EEPROM 확인과 bringup은 `docker exec`로
컨테이너(`IntelPi`) 안에서 한다.

**포함되지 않는 것** — 이 둘은 항상 사용자가 직접 한다.
- `run_mission.py` 실행(실제 미션 구동)
- `stop_bringup.sh`(kill을 포함하는 정지)

### 1/3 — 코드 상태 확인

`~/docker/shared/grippers`(Pi 호스트에 바인드 마운트된 그리퍼스 저장소 체크아웃)
에서 `git fetch --all`, `git status -sb`. `origin/kica927/baseline_mission`
보다 몇 커밋 뒤처졌는지 세서, 뒤처졌으면 `git merge --ff-only origin/
kica927/baseline_mission`으로만 받는다 — rebase나 강제 merge는 없다. fast-
forward가 안 되면(로컬에 갈라진 커밋이 있으면) 그대로 실패해서 사람이 보게
한다.

### 2/3 — EEPROM 캘리브레이션 비교 (읽기 전용)

컨테이너 안에서 `tools/arm/restore_taught_offsets.py`를 **`--apply` 없이**
실행한다 — 현재 서보 6개의 `Homing_Offset`을 읽어 `floor_grasp_profiles.
TAUGHT_HOMING_OFFSETS`(교시 당시 값)와 비교만 한다(`calib_identity.py`).
불일치가 있어도 이 스크립트는 절대 고치지 않는다 — 되돌리려면 `--apply
--yes`가 필요한데, 그건 서보 토크를 꺼서 팔이 중력으로 내려오기 때문에
반드시 사람이 직접 실행해야 한다.

### 3/3 — bringup

컨테이너 안에서 `tools/ops/bringup_now.sh 192.168.0.9`:

- 이미 뜬 노드(`ros_robot_controller`/`odom_publisher`/`ekf_node`/
  `joint_state_publisher`/`ascamera_node`/`arm_driver_node`/
  `perception_node`/`robot_state_publisher`, 좀비 제외)가 있으면 **kill하지
  않고** `stop_bringup.sh`를 먼저 돌리라는 안내만 찍고 종료한다(중복 기동
  방지).
- `ROS_DOMAIN_ID=21`(기본값), `need_compile=False`, `DEPTH_CAMERA_TYPE=
  ascamera`, ascamera 라이브러리용 `LD_LIBRARY_PATH`를 지정한다.
- 워크스페이스 4개를 **이 순서로** source한다(하나라도 빠지면 그 안 패키지가
  조용히 "not found"가 된다):
  1. `/opt/ros/humble` — ROS2 자체
  2. `~/ros2_ws` — MentorPi 벤더 스택(`bringup` 패키지가 여기 있음 — LD19/
     제스처/라인추적 등 범용 데모지 그리퍼스 전용이 아니다)
  3. `~/third_party_ros2/third_party_ws` — ascamera 드라이버
  4. `/ros2_ws` — **그리퍼스 프로젝트 전용** 워크스페이스(이 저장소의
     `ros2_ws/`를 컨테이너에 바인드 마운트한 것). `grippers_arm`/
     `grippers_perception`/`grippers_bringup`이 전부 여기 있다.

  ⚠️ `grippers_bringup`(4번)과 `bringup`(2번)은 이름이 겹치지만 완전히
  다른 패키지다 — `bringup`으로 launch하면 벤더 데모 스택만 뜨고
  `arm_driver_node`/`perception_node`는 아예 안 뜬다.
- `setsid ros2 launch grippers_bringup bringup.launch.py use_fake_base:=false
  use_fake_arm:=false use_fake_perception:=false host_ip:=<인자, 기본
  192.168.0.9>`를 백그라운드로 launch한다. 로그는 `/tmp/bringup.log`,
  launch PID/PGID는 `/tmp/bringup.pgid`에 기록해 둔다(나중에
  `stop_bringup.sh`가 이 PGID로 정확히 정지시킨다).

### 마지막 — 노드 확인

컨테이너 안에서 `ros2 node list`로 실제로 떴는지 확인하고 끝난다.

## `domain/` 사본 관리

`domain/ports/baseline_ports.py` 와 `domain/task/motion.py` 는 본 저장소에서
복사한 것이다. 원본이 바뀌면 여기가 조용히 낡는다 — **본 저장소가 사본 문제로
이미 세 번 당했다.**

```
python3 tools/check_domain_sync.py --upstream ~/Desktop/intel/grippers
```

일치하면 0, 다르면 1 을 돌려주고 고칠 명령을 알려준다.
