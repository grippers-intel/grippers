# Grippers — 작업 인수인계 (2026-08-24 갱신)

이 문서 하나로 지금 상태·오늘까지의 실기 이력·코드 목록·다음에 돌릴 테스트
절차를 전부 파악할 수 있도록 다시 정리했다. **0번(지금 당장 확인할 것)부터
읽을 것.**

---

## 0. 지금 당장 확인할 것 (최우선)

**팔(SO-ARM101) 서보 통신이 세션 종료 시점에 완전히 끊겼다.**
`arm_driver_node`를 껐다 다시 띄워도 `FATAL: SO-ARM101 torque 상태를
읽지 못했습니다 — servo IDs: [1, 2, 3, 4, 5, 6]`로 즉시 죽었다 — servo
6개 전부와 통신 자체가 안 되는 상태. 그 직전까지 몇 시간 동안 정상
동작했으므로(수십 차례 파지 성공) 소프트웨어 문제가 아니라 **케이블/커넥터
이완, 서보 보드 이상, 전원 문제 같은 하드웨어 레벨 문제**로 추정된다.
**물리적으로 팔 배선·전원을 먼저 점검할 것.** 토크가 꺼진 채(무통신)
방치된 상태로 Pi 전원을 껐다 — 재기동 시 팔이 중력에 맡겨진 자세일 수
있으니 다치지 않게 주의해서 확인할 것.

Pi는 `sudo poweroff`로 안전 종료된 상태다. 다시 켤 때:
1. 팔 배선 육안 점검 (위 참고)
2. Pi 부팅 → `~/docker/exec_shell.sh`로 컨테이너 진입(반드시 **실제
   대화형 터미널**에서 — tty 없이 실행하면 `cannot attach stdin to a
   TTY-enabled container` 에러가 난다. 스크립트 자체는 문제없음, 2026-08-23
   재확인)
3. `arm_driver` 기동해서 통신 복구됐는지부터 확인 (§4 절차 참고)

---

## 1. 현재 상태 스냅샷 (2026-08-23 밤 기준)

- **Pi**: 안전 종료됨(`sudo poweroff`)
- **팔**: 통신 두절, 토크 꺼짐 (§0 참고)
- **베이스**: 정지
- **그리퍼**: 물체 없음(load 0.0 확인 후 종료)
- **git**: 오늘 작업 전부 `kica927/grippers`(개인 미러)에 푸시+머지 완료
  — 브랜치 `kica927/hw-test-20260824`, PR
  [`#4`](https://github.com/kica927/grippers/pull/4), `main`에 머지됨.
  팀 org 리포(`grippers-intel/grippers`)는 이번에도 건드리지 않음(사용자
  지시 유지 중).
- **Pi 로컬 git 클론**(`/home/pi/docker/shared/grippers`, 컨테이너
  `/grippers`·`/ros2_ws`와 바인드 마운트)은 **맥과 별개의 독립된 클론**이다
  — 브랜치 `kica927/hw-test-20260823`, 어제 기준 여러 파일에 커밋 안 된
  로컬 수정이 있었다(`base_driver_node.py`, `visual_approach_control.py`,
  `perception_node.py` 등). 맥 쪽(personal-mirror에 푸시된 버전)과 100%
  같은 내용인지 아직 미확인 — 다음에 Pi 접속하면 `git diff`로 비교 권장.
  `tools/*.py` 신규 스크립트들은 scp로 바인드 마운트 폴더에 직접 넣은
  것들이라 Pi 쪽 git엔 untracked 상태로 남아있다.

---

## 2. 코드 인벤토리

| 파일 | 역할 | 검증 상태 |
|---|---|---|
| `tools/grasp_test_console.py` | 대화식 GRASP 테스트 콘솔. 6개 클래스 대응(rook/knight/queen/box/soccer/star), 관측→정렬 전진→GRASP 진입(그리퍼 예열림)→그리퍼캠 면적 기준 파지→들어올리기→CARRY_IDLE→바구니 투하까지 키 입력으로 단계별 실행. JSON Lines 구조화 로그 남김 | ✅ **검증됨** — 룩·축구공 각 1회 7단계 전체 성공 |
| `tools/auto_approach_grasp_rook.py` | 위 콘솔의 완전 자동화 버전(룩 전용). SCAN 없이 관측 폐루프로 직진+좌우회전 결합 이동해 목표 지점 수렴 → GRASP 진입 → 그리퍼캠 면적 기준 미세전진 → 파지 → CARRY_IDLE | ⚠️ **3회 튜닝, 미완**(§3-3 참고) — 좌우 목표는 310px(화면 정가운데 좌 10px), 미세전진 목표는 40cm로 마지막 조정했으나 이 조합은 아직 실기 미검증 |
| `ros2_ws/src/grippers_base/grippers_base/scan_track_control.py` | SCAN 대상 물체로 **제자리회전+직진 분리** 추적, 원위치 복귀(직선 벡터 방식), 시각 기반(YOLO, LiDAR 아님) 장애물 회피의 순수 제어 수학. 회전 실패 원인 조사 결과가 모듈 docstring에 전부 기록돼 있음 | 🔴 **하드웨어 완전 미검증** — pytest 30개만 통과 |
| `tests/test_scan_track_control.py` | 위 모듈 단위 테스트 30개 | ✅ 전부 통과 (`PYTHONPATH=. .venv/bin/pytest tests/test_scan_track_control.py`) |
| `tools/scan_track_return.py` | 위 제어 수학을 실행하는 콘솔. `--raw-cls`로 물체 선택 → SCAN+추적(기본 35cm) → 원위치 복귀. 회전 명령은 `controller/cmd_vel`(클램프 없는 토픽)에 발행 | 🔴 **하드웨어 완전 미검증** — 다음 세션 최우선 테스트 대상 |
| `ros2_ws/src/grippers_base/grippers_base/visual_approach_control.py` | (기존) 회전+전진 결합형 APPROACH 제어 수학. 순수 회전 단독 테스트 실패 이력 있음 | ⚠️ 재검증 보류 — scan_track_control.py의 "정렬/전진 완전 분리" 설계로 대체 시도 중 |
| `HORIZONTAL_SAFE_145_RAW`/`IDLE_CRADLE_RAW` 등 `floor_grasp_profiles.py` | 파지 자세 상수 | ✅ 안정적으로 계속 작동 확인 |

---

## 3. 테스트 이력 (연대순 요약)

### 3-1. 2026-08-23 — 그리퍼캠 근접 파지 절차 확정, 7단계 콘솔 완성

- **그리퍼캠 파지 판정 기준 확정**: 컨투어 면적(그레이스케일 임계
  150 → 5×5 모폴로지 open/close → `findContours` 최대 면적) **82,854px²
  (640×480의 27.0%) 이상**이면 닫아도 된다. 그 이상(예: 172,738px²,
  56.2%)도 문제없이 성공 — "기준치 초과"가 조건이지 정확히 맞출 필요는
  없다.
- `grasp_test_console.py` 완성, 룩·축구공 각 1회 **7단계 전체 성공**
  (load 0.0704~0.0782, 기준 0.04 상회, midpoint까지 유지).
- **버그 3개 수정**:
  1. 그리퍼캠 검출 안 됨 — `perception_node.__init__`이 confirm_grasp용
     기준 프레임을 찍으려고 `/dev/gripper_cam`을 기동 즉시 무조건 열어서
     계속 쥐고 있었다(예전 "lazy하게 연다"는 가정이 틀렸음, `lsof`로 직접
     확인). GRASP 진입 후 `observe_target`을 더 안 쓰는 시점에
     `perception_node`를 자동으로 죽이도록 수정.
  2. 그리퍼 안 열림 — `_move_floor_stage`가 팔 관절(servo 1-5)만 움직이고
     그리퍼(servo 6)는 손을 안 댄다는 걸 놓쳐서, GRASP 진입 후 명시적으로
     여는 코드가 빠져 있었다. 추가 완료.
  3. `exec_shell.sh` 컨테이너 진입 실패 — 스크립트 문제가 아니라
     `docker exec -it`를 tty 없는 환경(비대화형 SSH 등)에서 실행해서였다.
     실제 터미널에서는 정상 작동.

### 3-2. 2026-08-23 — 환경 함정 다수 재확인

§6에 통합 정리.

### 3-3. 2026-08-23 — 자동화 스크립트 3회 튜닝 (미완)

`auto_approach_grasp_rook.py`를 세 번 실기 실행하며 좌우 정렬 목표와
미세전진 거리를 조정:

| 회차 | 좌우 목표(px) | 미세전진 목표 | 결과 |
|---|---|---|---|
| 1 | 320(화면 정중앙) | 10cm | 물체가 그리퍼 기준 너무 오른쪽, load 0.0352(파지 부실) |
| 2 | 170.1(옛 교시값) | 20cm | 이번엔 너무 왼쪽, load 0.0352(**1회차와 정확히 같은 값**) |
| 3 | 310(정중앙에서 좌 10px) + align_first 추가 | 40cm(최소 30cm 보장) | 아직 좌우 편향 있음, load 0.0352(**3회 연속 동일값**) |

**3회 연속 load가 정확히 같다는 건 우연이 아니라 매번 같은 방식으로
살짝 헛집었다는 신호일 가능성이 높다** — 다음 실기에서 3회차 조합(310px
+ 40cm)부터 재검증하고, 그래도 load가 똑같이 나오면 좌우 정렬이 아니라
다른 근본 원인(그리퍼 기구 자체의 편향? 카메라-그리퍼 캘리브레이션 자체가
틀렸을 가능성?)을 의심할 것.

### 3-4. 2026-08-24 — 제자리 회전 실패 원인 규명 (코드 조사)

어제 실기에서 제자리 회전(0.3~0.6 rad/s)이 전혀 안 먹혔던 이유를 코드
조사로 확정 — 하드웨어 한계가 아니라 **커맨드 방식 문제 3가지**:

1. **토픽이 틀렸다.** `odom_publisher_node.py`가 구독하는 `cmd_vel`은
   `angular.z`를 **±0.5 rad/s로 강제로 자른다**(`app_cmd_vel_callback`).
   어제 이 프로젝트의 모든 도구(`base_driver_node.py` 포함)가 여기 발행
   했다. 벤더 자체 텔레옵/조이스틱/라이다회피 코드는 전부 클램프 없는
   **`controller/cmd_vel`**에 발행한다(조이스틱 max_angular=3.0, 라이다
   회피 1.2 rad/s 등).
2. **속도가 정지마찰 문턱보다 낮았다.** 실측 선속도 데드밴드(0.05m/s)를
   회전팔 길이(0.1407m, `odom_publisher_node.py`의 실제 wheelbase/
   track_width 런타임값 기준)로 환산하면 **약 0.355 rad/s**. 제자리
   회전은 바퀴가 바닥을 옆으로 긁어야 해서 직진보다 토크가 더 필요하다.
   벤더 코드 실제 사용값: 텔레옵 최저 0.5, 라이다회피 1.2 rad/s.
3. **`/odom_raw`는 회전 여부를 검증 못 한다.** `cal_odom_fun`이 적분하는
   값은 엔코더가 아니라 **명령으로 받은 linear_x/angular_z 그 자체**다 —
   바퀴가 완전히 멈춰 있어도 완벽한 회전을 했다고 보고한다. 앞으로 odom
   기반 위치 추정(원위치 복귀 등)은 그 전까지의 모든 이동 명령이 실제로
   바퀴를 움직였을 때만 유효하다는 전제를 깔아야 한다.

**권장 조치**: 제자리 회전은 `controller/cmd_vel`에 **1.0~1.2 rad/s**로
발행. 0~0.7 rad/s 구간은 아예 명령하지 말 것(`apply_align_turn_floor`
패턴 참고). 회전이 실제로 됐는지는 `/odom_raw`가 아니라 **눈으로 직접**
확인할 것(첫 실기 필수).

### 3-5. 2026-08-24 — 신규 기능 3종 설계·구현 (SCAN/추적/복귀/회피)

사용자 요청 3가지를 `scan_track_control.py` + `tests/` + 실행 콘솔
`scan_track_return.py`로 구현(§2 참고). 핵심 설계 결정:

- **거리 신호**: h(박스 세로 픽셀) 기반이 기본값 — SCAN 시 첫 관측으로
  `target_h`를 그 자리에서 역산(`establish_target_h`, 물체 실측 크기
  불필요, 현재 관측값의 면적 공식 기반 거리로 스케일링만 함). 종횡비가
  기준에서 40% 넘게 벗어나면(오검출/병합 박스 의심) 화면 면적 공식으로
  폴백.
- **원위치 복귀**: 지나온 경로를 그대로 재생하지 않고, `/odom_raw`의
  절대 (x,y)로 시작점까지 **직선 벡터 하나**를 계산해 "제자리 회전 1회 +
  직진 1회"로 돌아간다(사용자가 제시한 두 방식 중 후자 채택 — 재생 단계가
  적어 누적 오차가 훨씬 적음).
- **장애물 회피**: LiDAR는 이 높이의 체스말을 못 본다는 게 이미
  확인됐으므로, 목표가 아닌 나머지 5개 클래스를 매 반복 `observe_target`
  으로 각각 확인해 경로(좌우 ±15cm 편측 폭, 목표까지 남은 거리 안)에
  있으면 반대쪽으로 `linear.y`만 내서 피한다(좌/우 둘 중 하나로만 커밋).

**미검증 — Pi가 꺼져 있어 임포트조차 확인 못 함.** 다음 세션 최우선
테스트 대상(§4).

### 3-6. 2026-08-24 — 기타

- 예전 세션들이 스크래치패드에 남긴 depth 카메라·그리퍼캠 캡처 사진들을
  전수 확인해 `~/Downloads/depth_camera_captures/`(24장, 전부 640×480,
  깨진 YUYV 캡처 1장 포함)와 그리퍼캠 대표 샘플 1장으로 정리해 옮김.
  **참고 발견**: 그리퍼캠은 5~10cm 근접 촬영 시 초점이 잘 안 맞는다(최소
  초점 거리 한계로 추정) — 파지 순간 사진이 대부분 블러인 이유.

---

## 4. 다음 세션 절차 (순서대로)

### 4-0. 팔 하드웨어 점검 (§0)

물리 점검 후에도 안 되면 여기서 멈추고 사용자에게 보고 — 아래 단계는
전부 팔 통신이 정상이라는 전제다(단, §4-2/4-3은 팔을 안 쓰므로 팔이 안
고쳐져도 먼저 시도해볼 수 있다).

### 4-1. 표준 노드 기동 (매번 반복되는 절차, zsh 기준)

```
export need_compile=False
export DEPTH_CAMERA_TYPE=ascamera
export ROS_DOMAIN_ID=21
source /opt/ros/humble/setup.zsh
source /ros2_ws/install/setup.zsh
source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.zsh

ros2 launch controller odom_publisher.launch.py > /tmp/odom.log 2>&1 &
ros2 launch peripherals depth_camera.launch.py > /tmp/depth_cam.log 2>&1 &
sleep 8
ros2 run grippers_perception depth_cam_rotate_node > /tmp/rotate.log 2>&1 &
ros2 run grippers_perception perception_node > /tmp/perception.log 2>&1 &
ros2 run grippers_arm arm_driver --ros-args -p enable_torque_on_start:=true > /tmp/arm.log 2>&1 &
sleep 3
```

재시작 시 반드시 이전 프로세스를 먼저 `pkill -f <패턴>`으로 죽일 것
(§6 참고 — 중복 실행하면 시리얼/카메라 장치 충돌).

### 4-2. `scan_track_return.py` 최초 실기 검증 — 최우선

```
python3 /grippers/tools/scan_track_return.py --raw-cls rook
```

- **회전이 실제로 되는지부터 눈으로 확인.** `controller/cmd_vel`에
  1.2 rad/s로 발행하도록 이미 반영돼 있지만, 실기 확인 전까지는 여전히
  가설이다. 안 돌면 즉시 q+Enter로 중단.
- 회전이 되면: SCAN 정렬 → 35cm 접근 → 원위치 복귀까지 지켜보며 로그
  확인. 장애물 회피(§3-5)는 목표 근처에 다른 물체를 하나 놔두고 유발해볼
  것.
- 구조화 로그(`/tmp/grasp_test_log_<epoch>.jsonl`)를 세션 끝나기 전에
  다운로드해서 분석 요청할 것.

### 4-3. `auto_approach_grasp_rook.py` 3회차 조합 재검증

```
ros2 run grippers_perception perception_node > /tmp/perception.log 2>&1 &  # 꺼져 있으면
sleep 2
python3 /grippers/tools/auto_approach_grasp_rook.py
```

§3-3의 "3회 연속 load 0.0352 동일" 현상이 재현되는지 확인. 재현되면
좌우 정렬 문제가 아니라 다른 근본 원인을 의심할 것.

### 4-4. 나머지 5개 클래스 자동화 확장 여부 결정

3-2/4-3이 안정화되면, `grasp_test_console.py`는 이미 6개 클래스 다
지원하니 `auto_approach_grasp_rook.py`/`scan_track_return.py`를 다른
클래스로도 넓힐지 사용자와 논의.

---

## 5. 확정된 미션 파이프라인 설계 (변경 없음, 계속 유효)

1. 물체 detect(YOLO)
2. 물체 정면 약 30~40cm로 접근(회피 기동 포함) — `scan_track_return.py`가
   이 단계의 후보
3. GRASP 돌입 — 그리퍼 열고 파지 자세로 내려옴
4. 물체 방향으로 직진 접근(수십 cm, 그리퍼캠 면적 기준으로 정지)
5. 그리퍼 캠 컨투어 면적 + 부하(load) 값 둘 다로 파지 검증
6. 들어올리기 → CARRY_IDLE → 바구니 투하

**아직 코드 미반영**: GRASP 단계에서 servo 1을 능동으로 움직여 물체를
그리퍼 정면으로 재정렬하는 보정(사용자 지시, 평행 죠의 수동적 자기정렬을
보강하는 용도) — §3-3의 반복되는 파지 부실 문제와 관련 있을 수 있음.

---

## 6. 환경 함정 전체 목록 (누적)

- **컨테이너 기본 셸은 zsh, bash 아님** — `setup.bash`를 zsh에서 소싱하면
  `${BASH_SOURCE}` 문법 때문에 경로 계산이 깨진다(`/ros2_ws/setup.sh`
  같은 엉뚱한 경로를 찾다 실패). 반드시 `setup.zsh` 사용.
- **`ascamera` 패키지는 `/ros2_ws`가 아니라
  `/home/ubuntu/third_party_ros2/third_party_ws`에 설치돼 있다** — 이
  워크스페이스도 같이 소싱 안 하면 `depth_camera.launch.py`가
  `package 'ascamera' not found`로 조용히 실패한다. `uvc_open:Busy`로
  첫 시도 실패 후 자동 재시도하는 경우가 있어 기동 후 8초 정도 기다릴 것.
- **노드 재시작 전 이전 프로세스를 반드시 먼저 죽일 것** — `arm_driver`,
  `odom_publisher`, `depth_cam_rotate_node`, `perception_node` 전부
  중복 실행하면 시리얼/카메라 장치 충돌이 난다(둘 다 살아서 서로 명령을
  덮어써 통신 에러가 남 — `pkill -f <패턴>`으로 확실히 정리, 좀비
  프로세스(`<defunct>`)는 이미 죽은 것이니 무시해도 됨).
- **`rclpy.init()`의 기본 SIGINT 처리** — Ctrl+C 시 정지 명령이 안 나갈
  수 있다. `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`
  (`from rclpy.signals import SignalHandlerOptions`) 필수.
- **구독자 연결 대기 필수** — `pub.get_subscription_count() > 0` 확인 후
  발행 시작. DDS discovery 전 발행은 조용히 유실.
- **`ROS_DOMAIN_ID=21`을 매 셸/스크립트에서 export** — 컨테이너 기본값은
  0이다(안 하면 로봇이 안 움직이는데 스크립트는 성공 메시지를 찍는다).
- **`docker exec` 안에서 실행해야 함** — 호스트에만 둔 스크립트는
  컨테이너 안에서 못 찾는다. 이번엔 `/home/pi/docker/shared/grippers`
  (호스트) = `/grippers`(컨테이너) 바인드 마운트를 이용해 `docker cp`
  없이 scp만으로 즉시 반영시켰다.
- **`/dev/gripper_cam`은 `perception_node`가 기동 즉시 무조건 쥔다**
  (lazy 아님, §3-1 참고) — 직접 열려면 `pkill -f
  grippers_perception/perception_node` 먼저. `observe_target`을 더
  안 쓰는 시점에 죽이는 게 안전하다.
- **팔 시리얼 포트(`/dev/soarm`) 동시 접속 금지** — `arm_driver_node`가
  이미 열고 있는데 별도 스크립트가 같은 포트를 열면 전 서보 torque가
  꺼진다. 직접 접근 전 `pkill -f grippers_arm/arm_driver` 후, 현재 위치
  래치(`align_to_idle.py`류 기법)한 뒤 노드 재기동.
- **서보2 과열 게이트(40°C)** — 반복 테스트 시 자주 걸림, 연속 테스트
  사이 냉각 시간을 둘 것.
- **`cmd_vel`은 `angular.z`를 ±0.5rad/s로 클램프한다** — 회전 명령은
  `controller/cmd_vel`(클램프 없음)에 발행할 것(§3-4).
- **`/odom_raw`는 엔코더가 아니라 명령 적분값** — 회전이 실패해도
  odom은 성공한 것처럼 보고한다(§3-4).
- **`observe_target`의 "최대 높이 박스 선택" 로직이 혼잡한 장면에서
  불안정** — 낮은 신뢰도 오검출 박스(비정상 종횡비)를 고르는 경우 관찰됨.
  미수정 — 의심스러우면 YOLO 추론 스냅샷을 직접 확인.
- 뎁스카메라는 OpenCV로 직접 열면 안 됨(YUYV 결합 스트림, 초록/보라 띠).
  `ascamera` ROS 드라이버 경유 필수, 카메라 180도 뒤집혀 있음
  (`depth_cam_rotate_node`가 보정).
- `imu_calib` 패키지 없음 → EKF/`/odom` 없음, `/odom_raw`만 사용.
- 인식 동작점(`conf 0.45 · k-of-n 0.6 · 순도 ≥0.80 · y ≥290 · 산포
  ≤40px`)과 6클래스 순도 수치는 유효하나, 현재는
  `grippers_perception/perception_node`가 이 로직을 전담(`tools/
  perception/*.py` 옛 CLI 아님).
- 수평 파지(체스 기물 몸통을 옆에서 감싸 쥠)가 맞다는 결론, 파지 프로파일
  치수는 `floor_grasp_profiles.py`에 반영돼 있음.

---

## 7. 실측 상수 모음

| 상수 | 값 | 출처 |
|---|---|---|
| `K_CLASS`(RGB 면적→거리, `z_m=K/sqrt(h*w)`) | knight 38.0307, queen 31.1632, rook 37.3992, soccer 20.6092, box/star 미실측 | perception_node.py, 2026-08-23 실측 |
| 그리퍼캠 파지 판정 면적(rook) | 82,854px²(640×480의 27.0%) 이상 | 2026-08-23 실기 역산 |
| `LOAD_THRESHOLD` | 0.04 | domain/task/states.py |
| 성공 파지 시 load 재현값(rook) | 0.0704 | 2026-08-23 반복 재현 |
| 카메라 프레임(관측용) | 640×480 | camera_info 실측 |
| 그리퍼캠 프레임 | 640×480(초기 1회 1280×720 테스트 이력 있음) | 2026-08-24 이미지 감사 |
| 회전팔 길이(wheelbase+track_width)/2 | 0.1407m | odom_publisher_node.py 런타임값 |
| 정지마찰 문턱(회전 환산) | 약 0.355 rad/s | 2026-08-24 계산 |
| `cmd_vel` angular 클램프 | ±0.5 rad/s | odom_publisher_node.py |
