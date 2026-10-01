# vla_robot Host (노트북)

탑뷰 카메라 2대 + ArUco 로 로봇 위치를, Geti 검출기로 기물 위치를 잡고, 미션 FSM 이 매 사이클
`HostCommand` 를 Pi 로 보낸다. 규격은 `../ros2_ws/src/vla_common/vla_common/protocol.py` 하나뿐이다.

## 설치 (Windows, Python 3.11)

```powershell
cd vla_robot\host
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt          # vla_common 을 -e 로 함께 설치
pip install geti-sdk==2.13.1             # 검출기를 쓸 때만
```

`vla_common` 설치를 건너뛰어도 `host_config.py` 가 경로를 자동으로 얹는다.

## 준비물

- **카메라 캘리브레이션**: `calib/cam0.npz`, `calib/cam1.npz` (기존 `hardware/grippers_topview/calib` 에서 복사해 둠).
  카메라를 바꾸거나 초점이 달라지면 `python tools/calibrate_camera.py --cam 0` 으로 다시 잰다.
  파일이 없으면 근사값으로 돌며 큰 경고를 찍는다(수 cm 오차).
- **Geti 모델**(87 MB, 저장소에 넣지 않음):
  `hardware\grippers_topview\geti_sdk-deployment\deployment` 폴더를 `host\models\geti_deployment` 로 복사.
- **손 검출 모델**(선택, 7.8 MB, 저장소에 넣지 않음): `requirements.txt` 의 mediapipe +
  `models/hand_landmarker.task`
  (https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task).
  없으면 경고만 하고 손 없이 돈다. 손은 지도에 주황 원("hand L2")으로만 표시한다 — 미션은 아직 쓰지 않는다.
  카메라에서 잡히는지는 `python tools/hand_probe.py --live` 로 따로 본다.
- **실측값**: `config/host.yaml` — 마커 높이·바닥 마커 좌표·상자 좌표·yaw_offset.
  모르는 키가 있으면 기동을 거부한다.

## 실행

```powershell
python run_host.py --sim                      # 차량·카메라 없이 전체 흐름 확인
python run_host.py --sim --step               # n 키로 한 단계씩
python run_host.py --detector none --show-cams  # 카메라+ArUco 만 확인(명령은 콘솔 출력)
python run_host.py --pi-ip 192.168.0.7        # 실기
python tools/udp_teleop.py --pi-ip 192.168.0.7  # ArUco 없이 Pi 주행/팔 작업만 시험
```

### 화면

기본 화면은 **팀원 시연 UI**(`ui/grippers-ui.html`, 세로 432×768)다. run_host 가 `127.0.0.1:8765` 에
작은 HTTP 서버를 띄우고 크롬/엣지 `--app` 창(주소창 없음)으로 연다 — 윈도우·맥 모두 추가 설치가 없다
(pywebview 불필요). 둘 다 없으면 기본 브라우저로 열린다. **창을 닫아도 run_host 는 돈다 — 끝내려면 Ctrl+C.**

- 실행 화면: 지도(로봇·기물·경로·바구니·손) + 단계(1/4 접근 → 2/4 집기 → 3/4 운반 → 4/4 놓기)와 진행률
- 트레이: 비상 정지(해제는 카드의 "초기화 후 재개") · AUTO/MANUAL · Reset(두 번) · Prev/Next
- 키: `Esc` 비상 정지 · `→`/`←` 다음/이전(수동) · `d` 디버그(x·y·yaw·명령·대상·그립) · `l` 범례
- **입력창 = 사람 지시**(아래 "지시"). 마이크(음성)는 아직 연결 전 — 알림만
- UI 파일은 팀원 원본을 그대로 두고 `arena.js` 의 `vla_robot:` 주석 자리만 고쳤다(장판 1.98×1.83 · 손 표시).
  상태 문구는 `view/ui_state.py`, 서버·창은 `view/web_view.py`, 브라우저 쪽 연결은 `ui/host_bridge.js`.

### 지시 (Claude API)

입력창에 문장을 쓰면 `mission/instruction.py` 가 Claude(`claude-opus-5-5`, effort low)로
**어떤 기물(라벨 여럿 가능) · 하나/전부 · 정리(바구니)/가져오기**를 해석하고, FSM 이 그 기물만 고른다.
예: "퀸을 바구니에 넣어줘" · "체스 말만 전부 정리해줘" · "자유롭게 움직이는 말 정리해".
라벨은 그 순간 작업 구역에 보이는 것 중에서만 고른다. 해석을 못 하면 카드에 이유와 함께 보이는 기물 목록이 떠서 직접 고를 수 있다.

- **키**: `ANTHROPIC_API_KEY` **사용자 환경변수**(파일·저장소에 두지 않는다). 없으면 입력할 때 카드로 알린다.
  PowerShell: `[Environment]::SetEnvironmentVariable('ANTHROPIC_API_KEY', '<키>', 'User')` → 터미널을 새로 연다.
- **확인**: `python tools/try_instruction.py "체스 말만 전부 정리해줘"` (부를 때마다 요금)
- **모드**(`instruction.mode`): `auto` = 보이는 기물을 모두 정리하되 지시가 오면 그것부터 ·
  `instructed` = 지시가 있을 때만 움직인다. 초기화(Reset)는 지시도 취소한다.
- **가져오기**("가져와") = 사람 손에 건네기. 지시를 받을 때 탑뷰에 손이 보여야 접수한다(없으면 카드 —
  "바구니에 넣기"로 바꿀 수 있다). 집은 뒤 보이는 손에서 가장 가까운 위치(F1~R3)의 **정차점**
  (`handover.spots`: [손 x, 손 y, 정차 x, 정차 y])으로 가서 손 쪽을 보고(남는 각도는 팔 base),
  Pi 에 PLACE(`place_pose=handover`)를 보낸다. Pi 는 `arm_poses.yaml` 의 **handover**(ㄱ자) 자세로 가서
  `place.handover_wait_s`(1 s) 기다린 뒤 연다. 정차점에서 손이 `hand_wait_s`(20 s) 동안 안 보이면 바구니로 간다.
  ⚠️ handover 자세는 **아직 실측 전**(`measured: false`) — 재기 전에는 건네기가 실패로 끝나고 재시도 뒤 멈춘다.
- 시뮬에서 손 두기: `python run_host.py --sim --sim-hand L2` (지시는 입력창으로 — Claude 키 필요)

예전 OpenCV 지도는 `--view cv`(또는 `view.kind: cv`). 키: `q` 종료 · `space` ESTOP 래치(`r` 로만 해제) ·
`r` 리셋 · `n` 다음(수동) · `p` 이전 · `m` 수동/자동 전환

## 상태 흐름

```
SEARCH_TARGET -> APPROACH_PIECE -> GRASP -> CARRY_TO_DEST -> FACE_BOX -> NUDGE_BOX -> PLACE -> SEARCH_TARGET
                                     | 실패: 기물 보류(90 s) 후 SEARCH        | 실패: FACE_BOX 부터 재시도(2회), 넘으면 HALTED
```

전선 상태: SEARCH/HALTED=IDLE(stop) · APPROACH_PIECE=APPROACH · CARRY/FACE=CARRY · NUDGE=APPROACH_BOX ·
GRASP/PLACE=해당 작업(stop) · ESTOP 래치=ESTOP.

- 매 사이클 명령을 반드시 하나 보낸다. pose 를 잃으면 stop 을 보낸다.
- GRASP/PLACE 중에는 pose 와 무관하게 같은 상태를 계속 보낸다(팔이 마커를 가리는 건 정상).
- 작업 완료는 `JobTracker` 로 판정한다(Pi 가 결과를 매 패킷 반복 → UDP 유실에 안전).

## 테스트

```powershell
python -m pytest -q tests
```
