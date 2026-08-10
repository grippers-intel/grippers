# HLD — High Level Design

> **문서 상태**: 초안 · **freeze 목표: 2026-08-14 (금)**
> **기준 커밋**: `main` (구현이 문서보다 앞서 있는 부분은 구현을 기준으로 기술)
> **관련 이슈**: #25 (본 문서) · #15 유즈케이스 · #17 노드 구성 · #24 명령 문형 · #27 메시지 스키마 · #29 UML
> **변경 규칙**: freeze 이후 변경은 PR + 섹션 오너 승인. §4는 3인 합의.

---

## 0. 이 문서의 범위

README는 **왜 이 문제인가**를 설명합니다. HLD는 **무엇을 어떻게 만드는가**를 정의합니다.
구현자가 이 문서만 보고 자기 모듈을 짤 수 있어야 합니다.

| 다루는 것 | 다루지 않는 것 |
|---|---|
| 컴포넌트 분해와 책임 경계 | 프로젝트 동기·배경 → README |
| 인터페이스 명세 (서비스·액션·포트) | 함수 단위 구현 |
| 상태 전이와 실패 처리 | 실측 데이터 → `measurements.md` |
| 좌표계·데이터 흐름 | 채택하지 않은 설계 → `rejected_designs.md` |
| 예산 배분 | |

---

## 1. 시스템 컨텍스트

### 1.1 외부 경계

| 액터 / 외부 요소 | 상호작용 | 방향 |
|---|---|---|
| 작업자 (격벽 밖) | 키보드 텍스트 명령 | → 시스템 |
| 작업자 | 결과 보고 (성공 여부·접촉·시간) | 시스템 → |
| 작업자 | E-STOP (`/mission/emergency_stop`) | → 시스템 |
| 작업실 (정상광) | 출발·복귀 지점, 장물 배치 슬롯 | 환경 |
| 암실 겸 멸균실 (무광) | 파지 대상, 접촉 금지 구조물 | 환경 |
| 좁은 출구 (높이 ~300mm) | 자세 재조정 통과 대상 | 환경 |
| AI training server | 모델 학습 (오프라인) | 개발 시점만 |

### 1.2 제약 조건

| 구분 | 제약 |
|---|---|
| 조명 | 암실 구간 가시광 사용 불가 → IR 능동 조명 |
| 오염 | 구조물 접촉 0회. 물체를 내려놓을 수 없음 |
| 정보 | `L`, `w`, `H_gap` 을 **사전에 주지 않음** |
| 자율성 | 암실 진입 후 추가 명령 없음 |
| 전원 | 팔 서보와 로직 전원 도메인 분리 |
| 기간 | 2026-09-08 발표 |

---

## 2. 논리 아키텍처

### 2.1 계층

```
작업자 텍스트 명령
      │
      ▼
┌──────────────────────────────────────────────┐
│ domain/  (순수 Python · ROS2 타입 모름)        │
│   task/mission_task.py   MissionTask (제너레이터)│
│   task/states.py         State 체인             │
│   values.py              Pose2D / Point3        │
└──────────────────┬───────────────────────────┘
                   │ domain/ports/ (ABC)
      ┌────────────┼────────────┐
      ▼            ▼            ▼
  BaseDriver   ArmDriver   Perception        + estop (threading.Event)
      │            │            │
 real │ fake  real │ fake  real │ fake
      ▼            ▼            ▼
 Ros2Mecanum  Ros2ArmDriver  Ros2Perception   ← geometry_msgs ↔ values 변환 경계
   FakeBase      FakeArm     FakePerception
```

- **포트는 3종**입니다 (`BaseDriver`, `ArmDriver`, `Perception`). `TransformProvider` 는 도입하지 않았고, 좌표 변환은 각 Real 어댑터 안에서 수행합니다.
- `Ports` 데이터클래스는 여기에 **`estop` (threading.Event 유사 객체)** 를 하나 더 들고 있습니다. 포트 ABC가 아니라 인터럽트 플래그입니다.
- `MissionTask.run()` 은 **제너레이터**로, 매 전이마다 상태를 `yield` 합니다. 노드가 이를 받아 `/mission/state` 로 발행합니다.

### 2.2 ROS2 패키지 구성

| 패키지 | 산출물 | 상태 |
|---|---|---|
| `grippers_interfaces` | msg 1 · srv 7 · action 3 | ✅ |
| `grippers_mission` | `mission_orchestrator_node` | ✅ |
| `grippers_base` | `base_driver_node` | ✅ |
| `grippers_arm` | `arm_driver_node` | ✅ |
| `grippers_perception` | `perception_node` | ✅ |
| `grippers_vla` | — | ⚠️ **패키지만 있고 노드 없음** |
| `grippers_bringup` | `bringup.launch.py` | ✅ |
| `hud` | — | ⚠️ 미착수 |

MentorPi 벤더 스택(`app`, `bringup`, `driver`, `peripherals`, `navigation`, `slam`, `yolov5_ros2` …)이 `ros2_ws/src` 에 함께 들어 있습니다. `grippers_bringup` 은 대회용 `bringup.launch.py` 를 통째로 쓰지 않고 `controller` / `depth_camera` / `lidar` 만 골라 포함합니다 — **`/cmd_vel` 경쟁 방지**가 이유입니다.

### 2.3 컴포넌트 책임

| 노드 | 책임 | 하지 않는 것 | 오너 |
|---|---|---|---|
| `mission_orchestrator` | FSM 실행, 상태 발행, E-STOP 수신 | 직접 하드웨어 접근 | |
| `perception` | 카메라 소유, 조명 프로파일, 검출, 여유 거리 | 모션 결정 | |
| `arm_driver` | 서보 통신, IK, 그리퍼, 부하 조회 | 무엇을 잡을지 결정 | |
| `base_driver` | 주행, LiDAR, 회피기동, 정렬 | 미션 순서 판단 | |
| `vla_inference` | V/L/A 추론 | 상태 관리 | |
| `hud` | 시각화 (`/mission/state` 만 구독) | 제어 명령 발행 | |

> `/cmd_vel` 발행 주체는 `base_driver` **하나뿐**입니다.

---

## 3. 실행 구조

### 3.1 스레딩 모델

`mission_orchestrator_node` 는 FSM을 **별도 데몬 스레드**에서 순차 실행하고, rclpy는 `MultiThreadedExecutor` 로 스핀합니다. FSM이 액션 완료를 기다리며 블로킹 중이어도 E-STOP 콜백이 즉시 들어오게 하기 위함입니다. E-STOP 구독은 `ReentrantCallbackGroup` 을 씁니다.

```
[FSM thread]  IdleState → ... → execute(ports) → 블로킹 대기
[executor]    /mission/emergency_stop 수신 → threading.Event.set()
              → 다음 루프 진입 시 EstopState로 전이
```

### 3.2 배치

| 실행 위치 | 구동 요소 | 상태 |
|---|---|---|
| Raspberry Pi 5 (온보드) | 전 노드 | |
| AI 가속기 | 추론 | M1 벤치마크 후 확정 |
| 호스트 PC (격벽 밖) | 명령 입력 · HUD | |
| AI training server | 모델 학습 | 오프라인 |

**미결** — 컨테이너 배포 전제인지 확인 필요. `mission_orchestrator_node` 가 `sys.path.insert(0, '/grippers')` 를 하드코딩하고 있고, CI에 `docker/Dockerfile` 을 참조하는 잡이 있으나 **해당 파일이 없습니다**. → §9

| 항목 | 값 |
|---|---|
| 로봇 ↔ 호스트 연결 | |
| `ROS_DOMAIN_ID` | |
| 시연 중 네트워크 단절 시 동작 | |

---

## 4. 인터페이스 명세 ★ freeze 대상

### 4.1 토픽

| 이름 | 타입 | 발행 | 구독 | QoS |
|---|---|---|---|---|
| `/mission/state` | `grippers_interfaces/MissionState` | mission_orchestrator | hud | **RELIABLE + TRANSIENT_LOCAL, depth 1** |
| `/mission/emergency_stop` | `std_msgs/Empty` | 호스트 | mission_orchestrator | depth 10 |
| `/cmd_vel` | `geometry_msgs/Twist` | **base_driver 단독** | — | |

### 4.2 서비스

| 이름 | 타입 | 요청 | 응답 |
|---|---|---|---|
| `perception/detect_target` | `DetectTarget` | — | `bool found`, `Pose pose`, `Vector3 dims` |
| `perception/measure_gap` | `MeasureGap` | — | `float64 h_gap`, `Pose2D centerline` |
| `perception/set_light_profile` | `SetLightProfile` | `string profile` | `bool ready` |
| `perception/monitor_clearance` | `MonitorClearance` | — | `front`, `left`, `right`, `top`, `bool contact_risk` |
| `arm_driver/set_gripper` | `SetGripper` | `bool closed` | `bool ok`, `float32 load_ratio` |
| `arm_driver/get_load` | `GetLoad` | — | `float32 load_ratio` |
| `base_driver/align` | `AlignToCenterline` | — | `bool aligned`, `float64 yaw_error` |
| `base_driver/stop` | `std_srvs/Trigger` | — | 표준 |

### 4.3 액션

| 이름 | 타입 | Goal | Result | Feedback |
|---|---|---|---|---|
| `base_driver/drive_to` | `DriveTo` | `Pose2D target` | `bool arrived` | `distance_remaining` |
| `arm_driver/move_to_cartesian` | `MoveToCartesian` | `Point target`, `float32 grip`, `bool down` | `bool reached` | `distance_remaining` |
| *(미사용)* | `ReorientArm` | `float64 phi` | `bool settled` | `current_phi`, `wrist_load` |

> `ReorientArm` 은 정의만 되어 있고 호출부가 없습니다. `NARROW_EXIT` 이 이를 쓰도록 바꿀지 결정 필요 → §6.4

### 4.4 메시지

```
# MissionState.msg
string state          # State.name
uint32 contact_count  # 현재 미채움
float64 elapsed_s     # 현재 미채움
```

> `contact_count` 와 `elapsed_s` 는 **성공 기준(접촉 0회 / 소요 시간)의 측정 채널**입니다. 지금은 `state` 만 채워집니다. → §6.4

### 4.5 포트 시그니처

| Port | 메서드 | 인자 | 반환 |
|---|---|---|---|
| `BaseDriver` | `drive_to(target)` | `Pose2D` | `bool` 도착 여부 |
| | `align_to_centerline()` | — | `float` yaw 오차(rad) |
| | `stop()` | — | `None` |
| `ArmDriver` | `move_to_cartesian(xyz, grip=None, down=False)` | `list[float]`, `float?`, `bool` | `bool` 도달 여부 |
| | `set_gripper(deg)` | `float` (도) | `None` |
| | `get_load()` | — | `float` 부하 비율 |
| `Perception` | `detect_target()` | — | `(found, Point3 pose, Point3 dims)` |
| | `measure_gap()` | — | `.h_gap`, `.centerline` |
| | `set_light_profile(profile)` | `str` | `bool` |
| | `monitor_clearance()` | — | `.front/.left/.right/.top/.contact_risk` |

**결정 필요**

| 항목 | 현재 | 검토 |
|---|---|---|
| `set_gripper` 단위 | 포트는 `deg`, 어댑터는 `closed=(deg<50)` 로 이진화 | 각도 제어를 살릴지, `bool` 로 단순화할지 |
| `detect_target` 반환 | 튜플 3개, `dims` 가 `Point3` | 값 객체 하나로 묶을지 |
| `measure_gap`/`monitor_clearance` 반환 | `SimpleNamespace` | `values.py` 에 dataclass 정의 |
| 파지 자세(orientation) | srv는 `Pose` 지만 어댑터가 `Point3` 로 축소 → **회전 정보 손실** | Top-down 고정이면 명시, 아니면 타입 확장 |

### 4.6 VLA 텐서 인터페이스

→ [`vla_interface.md`](vla_interface.md). 본 문서는 링크만 유지합니다.

---

## 5. 좌표계

### 5.1 규약

| 항목 | 현재 | 비고 |
|---|---|---|
| 길이 단위 | m | `move_to_cartesian(xyz)` 주석 기준 |
| 각도 단위 | rad (`yaw_error`), deg (`set_gripper`) | **혼재 — 통일 필요** |
| 회전 표현 | 미정 | `Pose2D.theta` 만 사용 중 |
| `φ` 정의 | 장축과 **수평면** 사이 각도. 수직 파지 시 90° | |
| 포즈 전달 | `Pose2D` / `Point3` — **프레임 ID 없음** | 어느 프레임 기준인지 타입에 없음 |

### 5.2 프레임 트리 — *작성 필요*

```
map → odom → base_link → { laser, camera_link → camera_optical, arm_base → gripper_tcp }
```

`move_to_cartesian` 의 `xyz` 가 `base_link` 기준인지 `arm_base` 기준인지 문서에 없습니다. **M2 전에 확정해야 합니다.**

---

## 6. FSM 상태 전이

> **출처: `domain/task/states.py`, `domain/task/mission_task.py`**

### 6.1 정상 경로

```
IDLE → TRANSIT_OUT → LIGHT_ADAPT → DOCKING → IDENTIFY
     → GRASP → POSE_PLAN → NARROW_EXIT → RETURN → RELEASE → (None)
```

각 `State.execute(ports)` 가 다음 State 인스턴스를 반환하고, 미션 종료 시 `None` 을 반환합니다. 상태는 불변이며 재시도는 새 인스턴스로 표현합니다.

### 6.2 상태 표

| 상태 | 호출 포트 | 정상 종료 | 실패 전이 | 재시도 상한 |
|---|---|---|---|---|
| `IDLE` | — | 즉시 | — | — |
| `TRANSIT_OUT` | `base.drive_to()` | 도착 | `TRANSIT_OUT_FAILED` | **5** |
| `LIGHT_ADAPT` | `perception.set_light_profile("default")` | 즉시 | — | — |
| `DOCKING` | `base.align_to_centerline()` | 즉시 | — | — |
| `IDENTIFY` | `perception.detect_target()` | 검출 성공 | `IDENTIFY_FAILED` | **3** |
| `GRASP` | `arm.move_to_cartesian()`, `arm.set_gripper(0.0)` | 이동 성공 | `GRASP_FAILED` | **없음** |
| `POSE_PLAN` | `perception.measure_gap()` | 항상 | — | — |
| `NARROW_EXIT` | `arm.move_to_cartesian()`, `perception.monitor_clearance()`, `base.drive_to()` | 복귀 완료 | `NARROW_EXIT_FAILED` | — |
| `RETURN` | — | 즉시 | — | — |
| `RELEASE` | `arm.set_gripper(100.0)` | 종료 | — | — |

### 6.3 종료 · 인터럽트 상태

| 상태 | 동작 | 진입 경로 |
|---|---|---|
| `TRANSIT_OUT_FAILED` | `base.stop()` | 주행 5회 실패 |
| `IDENTIFY_FAILED` | 즉시 종료 | 검출 3회 실패 |
| `GRASP_FAILED` | 즉시 종료 | `move_to_cartesian` 실패 |
| `NARROW_EXIT_FAILED` | `base.stop()` | `contact_risk` 감지 |
| `ESTOP` | `base.stop()` | **`MissionTask.run()` 이 매 루프 `estop.is_set()` 확인 → 전역 인터럽트로 동작** |

> E-STOP은 `arm.hold_position()` 을 호출하지 않습니다. 장물 파지 중 정지 시 **낙하 가능성**이 있습니다. → §6.4

### 6.4 구현과 설계의 차이 — 해소 필요

| # | 항목 | 현재 구현 | 설계 요구 | 우선도 |
|---|---|---|---|---|
| 1 | **부하 기반 파지 검증** | `get_load()` 가 포트에 있으나 `GraspState` 가 호출하지 않음 | README 핵심 기능 | **P0** |
| 2 | **파지 실패 재시도** | `GRASP_FAILED` 즉시 종료 | 재인식 → 보정 → 재시도 | **P0** |
| 3 | **통과 불가 판정 (유즈케이스 2)** | `_solve_phi()` 가 `0.0` 고정, 항상 `NARROW_EXIT` | 해 없으면 거부·복귀 → `REJECT` 상태 | **P0** |
| 4 | **`ReorientArm` 액션 미사용** | `NARROW_EXIT` 이 `_phi_to_xyz()` 스텁으로 `[0.2, 0, 0.15]` 고정 | φ 기반 자세 전환 | **P0** |
| 5 | **`contact_count` / `elapsed_s` 미채움** | `state` 만 발행 | 성공 기준 측정 채널 | **P0** |
| 6 | **E-STOP 시 팔 자세 유지 없음** | `base.stop()` 만 | 낙하 방지 | P1 |
| 7 | **`NARROW_EXIT` 무한 루프** | `while True` 에 타임아웃 없음 | 전 상태 타임아웃 | P1 |
| 8 | **조명 프로파일 문자열** | `"default"` 하드코딩 | `NORMAL` / `DARKROOM` 열거 | P1 |
| 9 | **`RETURN` 이 빈 상태** | 복귀 주행이 `NARROW_EXIT` 내부 | 상태 분리 | P1 |
| 10 | **`align_to_centerline()` 반환 무시** | `yaw_error` 를 안 씀 | 정렬 오차 임계 판정 | P1 |
| 11 | **하드코딩 좌표** | `Pose2D(1.0, 0, 0)`, `[0.2, 0, 0.15]` | launch 파라미터화 | P1 |
| 12 | **`sys.path.insert('/grippers')`** | 노드에 하드코딩 | 패키징 또는 환경변수 | P2 |
| 13 | **`grippers_vla` 빈 패키지** | 노드 없음 | VLA 추론 노드 | P1 |
| 14 | **프레임 ID 없는 좌표 전달** | `Point3` / `Pose2D` | 프레임 명시 | P2 |

---

## 7. 핵심 알고리즘

### 7.1 자세 재조정 계획

```
H_proj(φ) = L·|sin φ| + w·|cos φ| ≤ H_gap − margin
```

전제: 요(yaw)는 `align_to_centerline()` 에서 진행축과 정렬됨 → 높이 1자유도 문제로 축소.
설계 기준값(`L=0.5, H_gap=0.3, margin=0.03, w≈0.02`)에서 **`φ ≲ 30°`** — 거의 눕혀야 합니다.

> **현재 구현**: `PosePlanState._solve_phi()` 가 `0.0` 고정 반환, `_phi_to_xyz()` 도 스텁입니다. 위 수식은 아직 코드에 없습니다. (#47)

| 항목 | 결정 |
|---|---|
| `margin` 결정 방식 | 고정값 / 추정 오차 연동 / 학습 — **택 1 필요** |
| 해 구간 중 선택 | 손목 서보 부하 최소 |
| 해 없음 | `REJECT` 로 전이 |
| 실행 경로 | 손목 단독 불가 → **IK 전체**. `ReorientArm` 액션 사용 검토 |

### 7.2 조명 도메인 전환

| 단계 | 동작 | 정착 대기 |
|---|---|---|
| 진입 감지 | | |
| 파라미터 적용 | 노출·WB 고정 | |
| IR 조명 점등 | | |
| 인식 재개 | | |

프로파일 문자열 집합을 확정해야 합니다 (현재 `"default"` 만 사용).

### 7.3 파지 검증

`get_load()` 임계값 — *M2 실측 후 기입*. `SetGripper` 응답에도 `load_ratio` 가 있어 별도 호출 없이 검증 가능합니다.

---

## 8. 예산 배분

### 8.1 오차 예산 → [`error_budget.md`](error_budget.md)

| 항목 | 목표 (1σ) |
|---|---|
| 도킹 정렬 (`yaw_error`) | |
| 치수 추정 (`dims`) | |
| 개구부 추정 (`h_gap`) | |
| **합성 3σ vs 통과 마진** | |

### 8.2 성능 예산

| 구간 | 목표 지연 |
|---|---|
| 검출 1프레임 | |
| VLA 추론 | |
| `monitor_clearance` 폴링 주기 | |
| 미션 전체 | |

---

## 9. 미결 사항 (Open Questions)

| # | 항목 | 선택지 | 기한 | 담당 |
|---|---|---|---|---|
| 1 | 컨테이너 배포 전제 여부 | `docker/Dockerfile` 신설 / CI `docker-build` 잡 제거 | 8/14 | |
| 2 | `move_to_cartesian` 기준 프레임 | `base_link` / `arm_base` | 8/14 | |
| 3 | 각도 단위 통일 | rad 통일 / 경계에서만 deg | 8/14 | |
| 4 | `ReorientArm` 액션 채택 여부 | 채택 / `move_to_cartesian` 유지 | 8/14 | |
| 5 | 자연어 명령 입력 경로 | `grippers_vla` 노드 / 별도 포트 | 8/14 | |
| 6 | 암실 구조 통과형(입구≠출구) | 통과형 / 왕복형 | | |
| 7 | 접촉 감지 수단 | 센서 / 도전성 테이프 / 영상 판독 | | |
| 8 | AI 가속기 | Hailo-8L / CPU | M1 벤치 후 | |
| 9 | `margin` 결정 방식 | 고정 / 오차연동 / 학습 | | |
| 10 | MentorPi 벤더 스택 유지 범위 | 전체 유지 / 필요 패키지만 | | |

---

## 10. 참고

| 문서 | 내용 |
|---|---|
| [`../README.md`](../README.md) | 배경, 미션 시나리오, 성공 기준 |
| [`sequences.md`](sequences.md) | 시퀀스 다이어그램 |
| [`architecture.puml`](architecture.puml) | 클래스 다이어그램 |
| [`vla_interface.md`](vla_interface.md) | V/L/A 텐서·좌표계 |
| [`error_budget.md`](error_budget.md) | 오차 전파 |
| [`rejected_designs.md`](rejected_designs.md) | 채택하지 않은 설계 |
| [`measurements.md`](measurements.md) | 실측 데이터 |
| [`purchase_ledger.md`](purchase_ledger.md) | 구매 장부 |

---

## 11. 변경 이력

| 날짜 | 버전 | 변경 | PR | 승인 |
|---|---|---|---|---|
| 2026-08-10 | 0.2 | 구현(`main`) 기준으로 §2·§4·§6 재작성 | | |
| | 0.1 | 초안 골격 | | |
