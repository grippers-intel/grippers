# HLD — High Level Design

> **상태: 실제 코드 기준으로 전면 재작성 (2026-09-04).** 이 문서는 원래 **암실 반출**
> 미션(§1의 "작업실/암실/좁은 출구", §6의 `TRANSIT_OUT`/`DOCKING`/`NARROW_EXIT` 등)을
> 기술했다 — 이 프로젝트의 **첫 번째** 설계였고 8/14 freeze를 목표했지만 채택되지
> 않았다. 그 뒤 장난감 정리(TIDY/FETCH, `SCAN`/`SELECT` 루프, [`class_diagram.md`](class_diagram.md)
> 이전 판)로 한 번, 2026-08-26 Host 지시 실행형 FSM(`baseline_mission.py`)으로 다시 한 번
> 아키텍처가 바뀌었다. 이 문서는 이제 **세 번째이자 실제 배포된 설계**만 기술한다 —
> FSM 전이 자체의 단일 소스는 여전히 [`state_machine.md`](state_machine.md)다.

---

## 0. 이 문서의 범위

README는 **왜 이 문제인가**를 설명한다. HLD는 **무엇을 어떻게 만드는가**를 정의한다.

| 다루는 것 | 다루지 않는 것 |
|---|---|
| 컴포넌트 분해와 책임 경계 | 프로젝트 동기·배경 → README |
| 인터페이스 명세 (UDP 링크·ROS 토픽/서비스/액션) | 함수 단위 구현 |
| 상태 전이와 실패 처리 → `state_machine.md`가 단일 소스 | 클래스 구조 → `class_diagram.md` |
| 하드웨어 구성·좌표계 | 오차/문턱값 실측치 → `error_budget.md` |

---

## 1. 시스템 컨텍스트

### 1.1 Host / Pi 역할 분담 (팀 확정, 2026-08-26)

**Host(`grippers-host-mac`, 노트북)가 공간에 관한 모든 것을 소유한다** — 오버헤드 웹캠
2대로 물체·차량 좌표를 추정하고, 목표를 선정하고, 경로를 계산하고, 매 사이클 속도
명령을 만든다. **Pi는 그 명령을 실행하고, 자기 센서로만 알 수 있는 것을 판단해
보고할 뿐이다.** 이것이 이 문서 전체를 관통하는 단일 결정이다 —
`domain/ports/baseline_ports.py` 최상단 docstring이 이 결정의 원문이다.

| 액터 / 외부 요소 | 상호작용 | 방향 |
|---|---|---|
| Host(맥) | UDP 명령(`HostCommand`) 5005 | → Pi |
| Pi | UDP 보고(`Report`+`MissionState`) 5006 | → Host |
| 작업자 | E-STOP(`threading.Event`, Pi 로컬) | → Pi FSM |
| 아레나(정상광) | 파지 대상 6클래스(체스말 3·장난감 3, 3D 프린팅), 바구니 2개(좌/우) | 환경 |
| 하드웨어 종료일 | **2026-09-08** | 제약 |

### 1.2 제약 조건

| 구분 | 제약 |
|---|---|
| 좌표 | Pi 어떤 포트에도 좌표·경로가 없다 — 있으면 Host의 일이 새는 신호 |
| `/cmd_vel` | 발행 주체는 `base_driver`(`Ros2MecanumBase`) 하나뿐 — 실제로 STM32에 쓰는 것은 별도의 `controller`/`odom_publisher_node`(`Controller`) |
| 링크 | UDP는 최신 명령만 본다 — 재전송 대기 없음. `None`(안 옴)과 정지 명령은 다르다(`LinkWatchdog`) |
| 라이다 | 바닥 위 140mm·11.3° 하향틸트 — 바닥 물체 회피엔 못 쓴다, 바구니 정면 판정 전용 |
| 전원 | 팔 서보(3S LiPo)와 로직 전원 도메인 분리 |
| 기간 | 하드웨어 접근 2026-09-08 종료 — 이후 녹화 데이터·순수 소프트웨어만 |

---

## 2. 논리 아키텍처

### 2.1 계층

```
Host(맥, grippers-host-mac)
      │  UDP 5005 HostCommand{state, linear_x, linear_y, angular_z, stop}
      ▼
┌──────────────────────────────────────────────────────┐
│ domain/  (순수 Python · ROS2 타입 모름)                │
│   task/baseline_mission.py   BaselineMission FSM        │
│   ports/baseline_ports.py    HostCommand/MissionState/Report │
└──────────────────┬─────────────────────────────────────┘
                   │ domain/ports/ (ABC) — 5종
      ┌────────────┼─────────────┬────────────┬──────────┐
      ▼            ▼             ▼            ▼          ▼
  HostLink     BaseDriver    ArmDriver   Perception    Lidar
      │            │             │            │          │
 real│fake    real│fake     real│fake    real│fake  real│fake
      ▼            ▼             ▼            ▼          ▼
 UdpHostLink  Ros2MecanumBase Ros2ArmDriver Ros2Perception Ros2Lidar
                                                                  + FakeHostLink/FakeBase/FakeArm/ScriptedPerception/FakeLidar
      │
      ▼ UDP 5006 Report+MissionState
   Host(맥)
```

> 클래스 단위 구조는 [`class_diagram.md`](class_diagram.md)를 참고하라. 포트는 **5종**이다
> (`HostLink`·`BaseDriver`·`ArmDriver`·`Perception`·`Lidar`). 이전 판의 `TransformProvider`·
> `CommandInterpreter`는 baseline 경로에 없다 — 좌표 변환 자체가 필요 없고(좌표가 Pi에
> 안 옴), 자연어 명령 경로가 배포되지 않았다(`class_diagram.md` §5).

- `BaselinePorts` 데이터클래스는 여기에 **`estop`(threading.Event)과 `watchdog`
  (`LinkWatchdog`)**을 포트 ABC가 아닌 인터럽트/카운터로 들고 있다.
- `BaselineMission.run()`은 **제너레이터**로, 매 사이클 상태를 `yield`한다. 노드가 이를
  받아 `/mission/state`로 발행한다(ROS 쪽 관측용 — Host로 가는 실제 보고는 UDP로
  별도로 나간다).

### 2.2 ROS2 패키지 구성

| 패키지 | 산출물 | baseline 연결 |
|---|---|---|
| `grippers_interfaces` | msg 4 · srv 9 · action 3 | 일부만 baseline이 실제로 씀(§4.5) |
| `grippers_mission` | `mission_orchestrator_node`, `battery_buzzer_node` | ✅ FSM 본체 |
| `grippers_base` | `base_driver_node` | ✅ |
| `grippers_arm` | `arm_driver_node` | ✅ 기동 시 교시 캘리브레이션 대조 |
| `grippers_perception` | `perception_node`, `depth_cam_rotate_node`, `gripper_cam_publisher_node` | ✅ (그리퍼캠은 모니터링 전용) |
| `grippers_language` | `language_node` | ⚠️ **빌드되지만 mission_orchestrator가 구독 안 함**(`class_diagram.md` §5) |
| `grippers_vla` | SmolVLA/ACT 실험 | ⚠️ **stretch 브랜치, baseline에 미병합**(`grippers-smolvla-is-a-stretch-branch`) |
| `grippers_bringup` | launch 재조합 | ✅ |
| `driver/controller` | `odom_publisher_node`(`Controller`) | ✅ **`/cmd_vel`을 실제로 STM32에 쓰는 노드** — MentorPi 벤더 스택 |

MentorPi 벤더 스택(`app`, `driver`, `peripherals`, `navigation`, `slam`, `yolov5_ros2` 등)이
`ros2_ws/src`에 함께 들어 있다. `grippers_bringup`은 대회용 `bringup.launch.py`를 통째로
쓰지 않고 `controller`/`depth_camera`/`lidar`만 골라 포함한다 — `/cmd_vel` 경쟁 방지가
이유다.

### 2.3 컴포넌트 책임

| 노드 | 위치 | 책임 | 하지 않는 것 |
|---|---|---|---|
| `mission_orchestrator` | Pi | FSM 실행, UDP 송수신, `/mission/state` 발행 | 직접 하드웨어 접근, 좌표 계산 |
| `perception` | Pi | 뎁스캠 소유, `identify_target`/`monitor_clearance`/`remember_target`/`confirm_grasp` | 목표 선정(그건 Host의 Geti 모델) |
| `arm_driver` | Pi | Feetech SDK, 관절 이동, 부하 조회, **기동 시 EEPROM 오프셋 대조** | 무엇을 잡을지 결정 |
| `base_driver`(`Ros2MecanumBase`) | Pi | `/cmd_vel` 발행만 | STM32에 실제로 쓰기(그건 `controller`) |
| `controller`(`Controller`) | Pi | `/cmd_vel` 구독 → STM32 모터 명령 | 명령 판단 |
| Host(`grippers-host-mac`) | 맥 | 좌표 추정, 목표 선정, 경로 계산, UDP 송수신 | ROS2 패키지가 아니라 순수 Python |

> `/cmd_vel` 발행 주체는 `base_driver` 하나뿐이지만, **바퀴를 실제로 돌리는 것은
> `controller`다** — 이 두 노드가 모두 떠 있어야 한다는 것이 2026-09-08 RUNBOOK
> §3.5의 핵심 경고다(`controller`가 없으면 명령이 구독자 0으로 조용히 버려진다).

---

## 3. 실행 구조

### 3.1 스레딩 모델

`mission_orchestrator_node`는 FSM을 **별도 데몬 스레드**에서 순차 실행하고, rclpy는
`MultiThreadedExecutor`로 스핀한다. FSM이 포트 호출로 블로킹 중이어도 E-STOP이 즉시
들어오게 하기 위함이다.

```
[FSM thread]  BaselineIdleState → ... → execute(ports) → 포트 호출 대기
[executor]    ports.estop.is_set() 를 매 루프 확인
              → 다음 사이클 진입 시 BaselineEstopState로 전이
```

### 3.2 배치

| 실행 위치 | 구동 요소 | 상태 |
|---|---|---|
| Raspberry Pi 5 (`IntelPi` 컨테이너, Ubuntu 24.04) | 전 노드 | ✅ |
| Raspberry Pi AI HAT+ 2 (Hailo-10H, 8GB) | YOLO 검출 추론 | ✅ HailoRT 5.1.1 연동, HEF 09-02 최신 export 배포 |
| Host(맥) | 오버헤드 웹캠 C920 ×2, 좌표 추정, 경로 계산, UDP 송수신 | ✅ ROS2 불필요 — 순수 Python |
| x86_64 Ubuntu 호스트 | Hailo DFC(ONNX→HEF 컴파일) | 이 문서 갱신 시점엔 이미 해소된 과거 이슈 — HEF는 매 학습마다 재컴파일해 배포 중 |

컨테이너 정의는 `docker/Dockerfile`. `mission_orchestrator_node`의
`sys.path.insert(0, '/grippers')`는 PYTHONPATH 미설정 환경 대비 안전장치로 유지된다.

---

## 4. 인터페이스 명세

### 4.1 Host↔Pi 링크 (UDP, ROS 밖)

**`state_machine.md`/`baseline_ports.py`가 단일 소스다.** ROS 토픽이 아니라 JSON-over-UDP다.

| 방향 | 포트 | 페이로드 |
|---|---|---|
| Host → Pi | 5005 | `{state, linear_x, linear_y, angular_z, stop}` (`HostCommand`) |
| Pi → Host | 5006 | `{report, state, detail, fix?}` (`Report`/`MissionState`/`Correction`) |

`fix`는 선택 필드다 — `*_BLOCKED`류 보고에만 Host가 그대로 실행할 수 있는 보정값이
함께 실린다(`domain.task.corrections.Correction`).

### 4.2 ROS2 토픽

| 이름 | 타입 | 발행 | 구독 | 비고 |
|---|---|---|---|---|
| `/mission/state` | `grippers_interfaces/MissionState` | `mission_orchestrator` | (관중 오버레이·디버깅, `hud` 미착수) | `state`·`contact_count`·`elapsed_s` **전부 채워짐**(과거 HLD가 미채움으로 기록했던 것과 다름 — `mission_orchestrator_node.py:118-119` 확인) |
| `/cmd_vel` | `geometry_msgs/Twist` | `base_driver` 단독 | `controller` | 경쟁 시 진동/비재현 버그 |
| `/scan_raw` | `sensor_msgs/LaserScan` | LiDAR 드라이버 | `Ros2Lidar` 어댑터 | 서비스 왕복 없이 직접 구독 |

### 4.3 서비스·액션 — 실제 사용 vs 미사용

`grippers_interfaces`에 정의된 것과 baseline이 실제로 부르는 것이 갈린다.

| 이름 | 대응 포트 메서드 | baseline에서 |
|---|---|---|
| `ObserveTarget.srv` | `Perception.identify_target()` | ✅ |
| `MonitorClearance.srv` | `Perception.monitor_clearance()` | ✅ |
| `ConfirmGrasp.srv` | `Perception.confirm_grasp()` | ✅ |
| `SetGripper.srv` | `ArmDriver.set_gripper()` | ✅ |
| `GetLoad.srv` | `ArmDriver.get_load()` | ✅ |
| `OffsetBaseYaw.srv` | `ArmDriver.offset_base_yaw()` | ✅ |
| `GetArmState.srv` | 캘리브레이션 대조 등 | ✅ (`arm_driver_node._check_taught_calibration`) |
| `MoveToFloorPose.action` | `ArmDriver.move_to_floor_pose()` | ✅ GRASP/INSERT 시퀀스의 주력 |
| `MoveToCartesian.action` | `ArmDriver.move_to_cartesian()` | ⚠️ **정의·서버는 있으나 baseline FSM은 안 부름**(좌표 기반 파지였던 이전 설계의 흔적, `arm_driver.py` 포트 docstring) |
| `ReorientArm.action` | `ArmDriver.reorient()` | ⚠️ **서버가 스텁**(`settled=True`만 반환) — baseline도 안 부름 |
| `Parse.srv` | `CommandInterpreter.parse()` | ❌ baseline 미사용(`class_diagram.md` §5) |
| `ConfirmPhrase.srv` | `CommandInterpreter.confirm_phrase()` | ❌ baseline 미사용 |
| `Detection.msg`/`DetectionArray.msg`/`MissionSpec.msg` | 이전 SCAN/SELECT 설계 | ❌ baseline 미사용 — `identify_target()`은 `TargetObservation` 하나만 반환하고 목록형이 아니다 |

### 4.4 `MissionState.msg`

```
string state          # State.name — UDP Report의 state와 별개 채널
uint32 contact_count  # 채워짐 (mission_orchestrator_node.py:118)
float64 elapsed_s     # 채워짐 — 이번 미션 경과 시간(mission_orchestrator_node.py:119)
```

---

## 5. 좌표계

Pi 쪽 포트에는 좌표가 **없다** — §2.1의 핵심 결정과 같은 이유다. 좌표계가 남아 있는
곳은 Host(`grippers-host-mac`, ArUco 기준 아레나 좌표계)와, Pi 내부의 두 로컬 판정뿐이다.

| 판정 | 기준 | 단위 |
|---|---|---|
| GRASP 정렬(`grasp_alignment.judge`) | Pi 자기 뎁스캠, 차체 정면 기준 전후·좌우 | m |
| INSERT 판정(`Lidar.basket_face`) | 라이다 원점 기준 거리·yaw·좌우 오프셋 | m, rad |

단위 규약은 필드명에 그대로 박혀 있다 — `_m`(미터), `_mm`(밀리미터, 개구 폭 전용),
`_rad`(각도). 이전 판이 지적했던 "`set_gripper`가 deg를 받는다"는 불일치는 이미
해소됐다 — 현재 시그니처는 `set_gripper(width_mm: float)`이다.

---

## 6. FSM 상태 전이

전이 그래프·상태별 계약은 전부 **[`state_machine.md`](state_machine.md)**가 단일 소스다.
이 문서(이전 판)가 그리던 `TRANSIT_OUT`/`DOCKING`/`IDENTIFY`/`NARROW_EXIT`/`RETURN`/
`RELEASE` 선형 체인은 존재한 적이 없는 설계이고, 실제 상태는 `IDLE`/`APPROACH`/
`GRASP`/`CARRY`/`INSERT`/`DONE`(+`ESTOP` 인터럽트) 6+1개다.

---

## 7. 핵심 알고리즘 — 위치만 안내

기하 해석적 자세계획(φ-solve, `H_proj(φ) = L·|sin φ| + w·|cos φ| ≤ H_gap - margin`)은
암실 반출 설계 전용이었고, 뒤이은 상자 투입 설계에서도 결국 보류됐다(`sequences.md`
이전 판 §3 "보류" 표기). **현재 INSERT는 기하 해석이 아니라 라이다 정면 판정의 문턱값
비교**로 대체됐다 — 알고리즘이랄 것이 없고, 실측 상수 비교만 있다.

| 판정 | 문서 |
|---|---|
| GRASP 정렬·파지 시퀀스 | [`sequences.md`](sequences.md) §1·§2 |
| INSERT 판정·투하 시퀀스 | [`sequences.md`](sequences.md) §3·§4 |
| 실측 문턱값 전부 | [`error_budget.md`](error_budget.md) |

---

## 8. 예산 배분

오차/문턱값 실측은 **[`error_budget.md`](error_budget.md)**로 이동했다 — 이전 판의
"도킹 정렬 오차·개구부 추정 오차" RSS 합성 모델은 좌표 기반 설계 전용이라 지금은
성립하지 않는다. 지금 있는 것은 그리퍼 부하·라이다 거리/yaw/좌우·INSERT 안정성처럼
**직접 실측한 개별 문턱값**이다.

성능 예산(추론 지연·미션 전체 소요)은 이번 재작성 시점에도 여전히 별도로 실측된 적이
없다 — §9 미결에 남긴다.

---

## 9. 미결 사항 (2026-09-04 기준)

이전 판의 미결 14건은 전부 암실/장난감-정리 설계 전용(조명 프로파일, `ReorientArm`
채택 여부, MentorPi 벤더 스택 범위 등)이라 이 아키텍처엔 적용되지 않는다. 실제로
남아 있는 것은 다음이다(출처: `grippers-baseline-wt` 핸드오버 09-01/09-03).

| # | 항목 | 상태 |
|---|---|---|
| 1 | RETURN_HOME 오실레이션 탈출 로직 | 커밋됨(`0552ee0`), **실기 미검증** — Pi 재연결 후 최우선 |
| 2 | NUDGE_BOX 하드스톱 안전반경 | 커밋됨, **실기 미검증** |
| 3 | GRASP AND 판정이 실제 무부하에서 오탐 없이 도는지 | 09-03 star/box 대응 후 재확인 필요 |
| 4 | GRASP 사이클 중 27~28초 구간 지연 원인 | 09-01 사용자 리뷰 발견, 로그/rosbag 대조 필요 |
| 5 | INSERT 낙하 위치가 바구니 안쪽으로 치우치는 경향 | 09-01 사용자 리뷰 발견, 반복 투입 데이터 필요 |
| 6 | INSERT 드롭 높이(약 250~270mm 안착 이격) 재설계 | 실행 가능성 조사 이전부터 미해결 |
| 7 | 바구니 충돌 근본원인(좌표 정확도·LiDAR 정렬·투입 높이) | 여력 되면 |
| 8 | `class_diagram.md` 등 나머지 설계 문서 갱신 | ✅ 이번 재작성으로 해소(2026-09-04) |

**하드웨어 접근이 2026-09-08 종료**된다 — #1~#3은 그 전에 실기로 확인해야 하고, 그 뒤로는
녹화 데이터·순수 소프트웨어(RoboSec in-process probe 등)만으로 작업이 이어진다.

---

## 10. 참고

| 문서 | 내용 |
|---|---|
| [`../../README.md`](../../README.md) | 배경, 미션 시나리오, 하드웨어 목록 |
| [`state_machine.md`](state_machine.md) | **FSM 전이 단일 소스** |
| [`sequences.md`](sequences.md) | 시퀀스 다이어그램 |
| [`class_diagram.md`](class_diagram.md) | 클래스 다이어그램 — 포트·State·노드 계층 |
| [`architecture.puml`](architecture.puml) | 위와 같은 구조의 PlantUML 버전 |
| [`error_budget.md`](error_budget.md) | 실측 문턱값 |

---

## 11. 변경 이력

| 날짜 | 변경 |
|---|---|
| 2026-09-04 | **전면 재작성** — 암실 반출 설계(§1의 원안)를 실제 배포된 Host 지시 실행형 FSM 기준으로 교체. §4 인터페이스를 UDP 링크 + 실제 사용 중인 srv/action만으로 재정리, §6·§7·§8을 각각 `state_machine.md`/`sequences.md`/`error_budget.md`로 위임, §9 미결 사항을 현재 유효한 8건으로 교체 |
| 2026-08-19 | (이전 판 이력 — 암실 반출 설계 당시) HailoRT 컨테이너 연동 완료 |
| 2026-08-17 | (이전 판 이력) 기준 ROS 2 배포판 확정 |
| 2026-08-12 | (이전 판 이력) 가속기 확보 확정 |
