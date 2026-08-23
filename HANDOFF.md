# Grippers — 작업 인수인계 (2026-08-23 갱신)

이전 HANDOFF(06:00, CLI 도구 단계 검증)에 이어, 이후 세션에서 ROS2 노드 기반
아키텍처(`grippers_base`/`grippers_arm`/`grippers_perception`, `mission_orchestrator`)로
전환한 뒤 실기로 진행한 작업 결과. **아래 "확정된 파이프라인 설계"와 "미해결
과제"가 지금 가장 중요하다.**

---

## 지금 하드웨어 상태

- 팔: IDLE(cradle) 자세, torque 켜짐, 정상
- 그리퍼: 열림(80mm)
- 룩(rook): 그리퍼에서 내려놓음(정밀 배치 아님 — 중간 높이에서 열어 낙하)
- 베이스: 정지
- 서보2 온도: 상승 이력 있음(41→43°C, 안전 상한 40°C) — 다음 GRASP 자세
  진입 전 반드시 냉각 확인 필요

---

## 확정된 파이프라인 설계 (사용자 지시, 최우선 참고)

1. 물체 detect (YOLO)
2. 물체 정면 **30cm**로 접근 (필요 시 회피 기동)
3. **GRASP 돌입** — 그리퍼 열고 파지 자세로 내려옴
4. 물체 방향으로 **직진 접근** — 정확한 전진 거리는 아직 미확정, 실측/캘리브레이션 필요 (다음 세션 최우선 과제)
5. **그리퍼 캠 컨투어 면적 + 부하(load) 값 둘 다**로 파지 검증
6. 들어올리기

**신규 설계 지시 (미반영, 코드 작업 필요)**: GRASP 단계에서 **servo 1을 능동
적으로 움직여 물체가 그리퍼 정면(좌우 중앙)으로 오도록 보정**한다. 지금까지는
평행 죠(parallel jaw)가 벌어진 채 전진하면 좌우 오차를 수동적으로 흡수하는
효과만 확인됐다 — 이건 그걸 대체하는 게 아니라 보강하는 능동 보정이다.

**캘리브레이션 방법론 (사용자 지시)**: 4번 단계의 전진 거리를 정할 때 Claude가
임의로 타이밍/거리 기반 스크립트를 짜지 않는다. 사용자가 **WASD 키보드
텔레옵으로 직접 베이스를 조작**하고, Claude는 그리퍼 카메라를 실시간
모니터링하면서 정지 조건(예: 컨투어 면적이 파지 기준치 초과)을 실시간으로
안내한다. 정밀 이동거리는 `/odom_raw`(`nav_msgs/Odometry`, 휠 엔코더 기반)로
교차 검증한다 — Ctrl+C 타이밍 추정보다 훨씬 정확하다. `/odom` 은 없다(EKF
미가동, `imu_calib` 패키지 부재) — 반드시 `/odom_raw`를 쓸 것.

---

## 이번 세션에 실기로 검증한 것

### 그리퍼 캠 기반 근접 파지 절차 — 2회 연속 성공

절차: GRASP 자세(그리퍼 열림)로 진입 → 그리퍼 캠을 보며 조금씩 전진 → 물체가
손가락 사이에 확실히 들어온 것을 확인 → 정지 후 닫기.

**판정 기준(신규 확정)**: 그리퍼 캠 프레임(640×480)에서 물체 컨투어 면적이
**82,854px²(27.0%) 이상**이면 닫아도 된다. 그 이상(예: 172,738px², 56.2%)도
문제없이 성공했다 — "기준치 초과"가 조건이지 정확히 맞출 필요는 없다.

| 회차 | 방식 | 닫기 직전 면적 | 닫을 때 load | midpoint load |
|---|---|---|---|---|
| 1 | 2cm+3cm 수동 미세 전진 | (기준치 역산) | 0.0704 | 0.0704 (lift 끝까지 유지) |
| 2 | `forward_manual.py`로 직접 조작, Ctrl+C 정지 | 172,738px² | 0.0899 | 0.0704 |

두 회차 모두 `LOAD_THRESHOLD=0.04`를 크게 상회.

컨투어 측정 방법: 그레이스케일 → `cv2.threshold(gray, 150, 255, THRESH_BINARY)`
→ 5×5 모폴로지 open/close → `cv2.findContours` → 최대 면적 컨투어.

### 순수 전진은 안정적, 회전+전진 APPROACH는 미해결

- **회전+전진으로 재설계한 APPROACH**(`visual_approach_control.py`,
  `base_driver_node.py`, 커밋 `298e884` personal-mirror)는 실기 첫 테스트에서
  좌측 약 90° 회전, 목표 이탈. 이후 제자리 회전만 단독 테스트(0.3, 0.6 rad/s)
  했으나 모터가 소리만 내고 실제 회전 없음 — **사용자 지시로 전면 중단**
  ("그냥 멈춰. 하나도 안 움직이니까"). 원인 미규명, 재검증은 다음 기회로 미룸.
- 반면 **순수 전진(linear_x만)은 안정적으로 작동** — 오늘 성공 사례 전부 순수
  전진 기반. 회전 문제 해결 전까지는 "정렬 이후 순수 전진만으로 파지 직전까지
  접근"이 유일하게 검증된 경로.
- 장애물 회피(옆으로 비키기) 로직도 이 커밋에 같이 들어갔으나, 아래 LiDAR
  문제로 **실전 사용은 보류** 상태.

### LiDAR — 연결 확인, 각도 보정 미해결, 사용 보류

- LD19가 실제로 연결돼 있고(`/dev/ldlidar`), `/scan_raw`(주의: launch 인자
  기본값은 `/scan`이지만 실제 토픽은 `/scan_raw`)로 정상 발행함을 확인.
- LD19 장착 높이(base_link 기준 9.25cm)에서는 체스말·축구공 같은 작은
  파지 대상이 전방 스캔에 아예 안 잡힌다 — 즉 이 게이트가 파지 대상 자체를
  장애물로 오인할 걱정은 없다(장점). 다만 낮은 문턱류 장애물은 여전히 못 봄.
- **각도 기준 불일치 발견**: 카메라상 완만한 좌측 오프셋 물체가 LiDAR
  각도로는 +82~89°로 나옴 — LiDAR 프레임과 base_link/카메라 정면 축 사이에
  설명 안 된 회전 오프셋이 있다. **사용자 지시로 이 보정은 보류**
  ("라이다는 보류"). 회피 로직을 실전 투입하려면 이걸 먼저 풀어야 한다.
- `-60°~-90°` 구간은 로봇 자체 팔/구조물에 의한 자기 차폐로, 항상
  0.026~0.034m가 나온다 — 실제 장애물 아님.

### 서보2 과열 안전 게이트 — 반복 테스트 시 자주 걸림

`MAX_FLOOR_POSE_SERVO2_TEMP_C=40`°C. 오늘 두 차례(45°C, 이후 41→43°C) 걸려
`move_to_floor_pose(..., "grasp"/"idle")`가 즉시 거부됨. **이전에는 이 거부를
전부 "safe→grasp 자세 재확인 실패"(pose tolerance 문제)로 오진단했었는데,
실제로는 서보 과열이 지배적 원인이었다.** 연속 실기 테스트 사이에 의도적으로
냉각 시간을 두는 것을 권장. `idle 복귀는 safe/drop 자세에서만 시작 가능`
이라는 별도 상태 제약도 있음(순서: `... → safe → idle`, `midpoint`에서 바로
`idle`은 거부됨).

---

## 이번 세션에 새로 겪은 환경 함정

- **`rclpy.init()`의 기본 SIGINT 처리** — 기본값은 자체 시그널 핸들러를 깔아,
  Ctrl+C 시 스크립트의 `except KeyboardInterrupt`/`finally`보다 먼저 컨텍스트를
  닫아버릴 수 있다(`RCLError: Failed to publish: publisher's context is
  invalid`, 정지 명령이 실제로는 하나도 안 나감). 수동 `cmd_vel` 스크립트는
  반드시 `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`
  (`from rclpy.signals import SignalHandlerOptions`)로 열 것.
- **구독자 연결 대기 필수** — `pub.get_subscription_count() > 0`을 확인하고
  발행 시작할 것. DDS discovery 전에 보낸 메시지는 조용히 유실된다.
- **`ROS_DOMAIN_ID=21`을 매 셸/스크립트에서 export** — 안 하면 스크립트가
  "전진 시작" 메시지를 찍고도 실제로는 다른 DDS 도메인으로 발행돼 로봇이
  안 움직인다.
- **`depth_camera.launch.py`가 띄우는 `ascamera_node`는 `/ros2_ws`가 아니라
  별도 워크스페이스(`/home/ubuntu/third_party_ros2/third_party_ws`)에 설치돼
  있다** — `source .../third_party_ws/install/setup.zsh`를 빠뜨리면
  `package 'ascamera' not found`로 조용히 실패한다(2026-08-23 재확인, 옛
  HANDOFF 노트가 이미 언급했지만 이번 세션에서 실제로 빠뜨려서 재확인함).
  `ros2 launch`가 `uvc_open:Busy`로 첫 시도에 실패했다가 자동 재시도해
  뜨는 경우가 있어 기동 후 8초 정도 여유를 두고 확인할 것.
- **노드 재시작 전엔 항상 이전 프로세스를 먼저 죽일 것** — `odom_publisher`,
  `depth_cam_rotate_node`, `perception_node` 전부 중복 실행하면(구 프로세스를
  안 죽이고 새로 띄우면) arm_driver 중복 실행 때와 같은 시리얼/장치 충돌이
  난다(2026-08-23 재확인). `pkill -f <패턴>` 후 기동할 것.
- **`docker exec` 안에서 실행해야 함** — 호스트(`/home/pi/...`)에만 둔
  스크립트는 컨테이너 안에서 `No such file or directory`. `docker cp`로
  `/tmp/`에 복사해서 컨테이너 안에서 실행할 것.
- **`ros2 topic pub --rate`를 백그라운드(`&`)로 단일 비대화형 SSH
  `bash -c` 안에서 돌리는 방식은 신뢰 불가** — 전용 Python 스크립트를 쓸 것
  (`forward_manual.py` 패턴 참고, 아래).
- **`/dev/gripper_cam`(`/dev/video0`)은 `perception_node`가 계속 독점
  보유** — **정정(2026-08-23 재실기 확인, `lsof /dev/video0`으로 직접
  확인)**: "confirm_grasp를 호출해야 lazy하게 연다"는 이전 노트는 틀렸다.
  실제로는 `perception_node.__init__`이 confirm_grasp용 기준(빈 그리퍼)
  프레임을 찍으려고 **기동 즉시 무조건** 이 장치를 열어서 그대로 쥐고
  있는다. 직접 `cv2.VideoCapture`로 접근하려면(예: `grasp_test_console.py`)
  먼저 `pkill -f grippers_perception/perception_node` 필요(YOLO/`scan_floor`/
  `observe_target`도 같이 멈춤 — 필요하면 나중에 재기동). `observe_target`을
  더 안 쓰는 시점(예: 접근 완료 후)에 죽이면 안전하다.
- **팔 시리얼 포트(`/dev/soarm`) 동시 접속 금지** — `arm_driver_node`가 이미
  열고 있는 상태에서 별도 진단 스크립트가 같은 포트에 `STS3215Driver`로
  붙으면 전 서보 torque가 예고 없이 꺼진다. 직접 접근 전엔 반드시
  `pkill -f grippers_arm/arm_driver`, 이후 현재 위치를 읽어 그 자리에서
  `set_position(i, present[i])`로 torque를 래치한 뒤 노드 재기동.
- **`observe_target`의 "매칭 클래스 중 최대 높이 박스 선택" 로직이 혼잡한
  장면에서 불안정** — 낮은 신뢰도의 가짜 "rook" 박스(비정상 종횡비)를 대신
  고르는 경우가 여러 번 관찰됨. 코드 미수정 — 의심스러우면 직접 YOLO 추론
  스냅샷으로 눈으로 확인할 것.

---

## `forward_manual.py` — 수동 전진 + 안전 정지 (검증된 최종본)

컨테이너 안(`docker cp`로 `/tmp/forward_manual.py`)에서 실행:

```python
#!/usr/bin/env python3
import sys, time
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist

speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.06

rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = rclpy.create_node('forward_manual')
pub = node.create_publisher(Twist, 'cmd_vel', 10)

start = time.time()
while pub.get_subscription_count() == 0 and time.time() - start < 5:
    time.sleep(0.05)
print(f'구독자 연결됨 ({pub.get_subscription_count()}) — {speed} m/s로 전진 시작. Ctrl+C로 정지.')

t = Twist()
t.linear.x = speed

try:
    while True:
        pub.publish(t)
        time.sleep(0.05)
except KeyboardInterrupt:
    print('정지 명령 발행 중...')
finally:
    stop_until = time.time() + 1.0
    while time.time() < stop_until:
        pub.publish(Twist())
        time.sleep(0.02)
    print('정지 완료.')
    node.destroy_node()
    rclpy.shutdown()
```

실행: `docker exec -it IntelPi bash -lc "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=21 && python3 /tmp/forward_manual.py 0.06"`

---

## 다음 세션 최우선 과제

1. **직진 접근 거리 캘리브레이션** — 위 "확정된 파이프라인 설계" 4번.
   룩을 실측 30cm에 놓고, GRASP 진입 → WASD 텔레옵으로 전진 → 그리퍼 캠
   면적 + `/odom_raw` 이동거리를 같이 기록 → 면적이 82,854px² 이상이 되는
   지점까지의 누적 이동거리를 산출해 고정값으로 만든다.
2. **GRASP 시 servo 1 좌우 능동 보정** 구현 — 아직 코드 미반영.
3. **회전(제자리 회전) 미작동 원인 규명** — 회전+전진 APPROACH 재검증의
   전제조건. 모터가 소리만 내고 회전 안 하는 원인(정지마찰? 배터리 전압?
   부호/게인?) 미규명.
4. **LiDAR 각도 기준 보정** — 회피 기동을 실전 투입하려면 필요. 사용자가
   명시적으로 보류 지시했으니, 재개 시점은 사용자 판단을 따를 것.
5. `observe_target`의 오탐 박스 선택 문제 — 근본 수정(프레임 간 안정적
   `track_id` 또는 클래스+위치 기반 매칭) 필요, 이번 세션엔 손 안 댐.
6. `/grippers/config/approach_target.json`(Pi 로컬, h=247.0)이 아직
   Mac/개인 repo에 커밋 안 됨 — 회전 APPROACH 검증이 끝나기 전까지는 보류
   상태 유지 중.

---

## 참고 — 이전(06:00) HANDOFF에서 여전히 유효한 것

- 뎁스카메라는 OpenCV로 직접 열면 안 됨(YUYV 결합 스트림) — `ascamera` ROS
  드라이버 경유 필수, 카메라 180도 뒤집혀 있음.
- `imu_calib` 패키지 없음 → EKF/`​/odom` 없음, `/odom_raw`만 사용.
- 경로 이원화: 호스트 `/home/pi/docker/shared/grippers/…` = 컨테이너 `/grippers/…`.
- 인식 동작점(`conf 0.45 · k-of-n 0.6 · 순도 ≥0.80 · y ≥290 · 산포 ≤40px`)과
  6클래스 순도 수치는 여전히 유효한 값이나, **현재 아키텍처에서는
  `tools/perception/*.py` CLI가 아니라 `grippers_perception/perception_node`
  가 이 로직을 담당**하므로 옛 도구 경로를 그대로 실행하려 하지 말 것.
- 수평 파지(체스 기물 몸통을 옆에서 감싸 쥠)가 맞다는 결론, 파지 프로파일
  치수(`chess_rook` 폭 24.5mm, 파지높이 45mm, 예열림 80mm, 닫기 15mm)는
  `floor_grasp_profiles.py`에 그대로 반영돼 있음.
