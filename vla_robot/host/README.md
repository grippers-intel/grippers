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

지도 창 키: `q` 종료 · `space` ESTOP 래치(`r` 로만 해제) · `r` 리셋 · `n` 다음(수동) · `p` 이전 · `m` 수동/자동 전환

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
