# grippers-host-mac

`grippers` 프로젝트의 **Host PC 코드를 macOS(Apple Silicon)에서 돌아가게** 옮긴
것이다. 원본은 Windows 전용이었다.

원본: `kica927/grippers` 의 `host/` (PR #44, `sysy009/host-topview-merge`)

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
```

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

## ⚠️ 확인하지 못한 것

**geti 추론을 실제로 돌려 보지 못했다.** `host/geti_sdk-deployment/` 에
`model.xml`(1.3MB)만 있고 **`model.bin`(85MB)이 없다.** 원본 저장소에서
`.gitignore` 로 빠져 있기 때문이며, PR #44 본문도 "지금은 ignore 라
`geti_sdk-deployment` 가 반쪽입니다" 라고 적고 있다.

즉 이건 macOS 문제가 아니라 **Windows 에서도 그대로인 문제**다. 가중치를
받으면 그때 추론까지 확인할 수 있다. 런타임(OpenVINO)이 arm64 에서 CPU 장치를
잡는 것까지는 위에 적은 대로 확인됐다.

Host 팀이 iGPU 대신 **CPU 로 확정**했으므로(디바이스 간 검출 불일치 8건),
Apple Silicon 에 GPU 플러그인이 없는 것은 이 프로젝트에서 손해가 아니다.

---

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

## `domain/` 사본 관리

`domain/ports/baseline_ports.py` 와 `domain/task/motion.py` 는 본 저장소에서
복사한 것이다. 원본이 바뀌면 여기가 조용히 낡는다 — **본 저장소가 사본 문제로
이미 세 번 당했다.**

```
python3 tools/check_domain_sync.py --upstream ~/Desktop/intel/grippers
```

일치하면 0, 다르면 1 을 돌려주고 고칠 명령을 알려준다.
