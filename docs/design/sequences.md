# Sequence Diagrams

> **상태: to-be 설계 (8/14 freeze 대상).** 현재 `main` 코드는 아직 이전 주제 기준입니다.

모든 상호작용은 도메인(`MissionTask`)과 **포트** 사이에서만 일어납니다.
ROS2 노드, Feetech 서보 SDK, OpenCV, Hailo 런타임은 다이어그램에 등장하지 않습니다 —
어댑터 뒤에 숨어 있기 때문입니다. 다이어그램에 `rclpy` 가 보인다면 그건 설계가 샌 것입니다.

- [1. TIDY 전체 루프](#1-tidy-전체-루프)
- [2. 파지 검증 및 자동 재시도](#2-파지-검증-및-자동-재시도)
- [3. 상자 투입 자세 재조정 (⏸ 보류)](#3-상자-투입-자세-재조정--보류)
- [4. FETCH 분기](#4-fetch-분기)
- [5. 음성 명령 — 복창과 되묻기](#5-음성-명령--복창과-되묻기)

---

## 1. TIDY 전체 루프

바닥에 흩어진 N개를 종류별 색 상자에 넣는 기본 미션입니다.
**루프가 핵심**이므로 개별 사이클보다 사이클이 반복되는 구조에 주목하세요.

```mermaid
sequenceDiagram
    autonumber
    actor OP as 사용자
    participant I as CommandInterpreter
    participant T as MissionTask<br/>(FSM · Domain)
    participant P as Perception
    participant B as BaseDriver
    participant A as ArmDriver

    Note over OP,A: IDLE
    OP->>I: "장난감 정리해줘"
    I-->>T: MissionSpec(mode=TIDY, placement_rule)
    Note right of I: 규칙 변경 문형도 여기서 흡수<br/>"체스말은 검은 상자에" → rule[CHESS_PIECE]=BLACK

    loop 미처리 대상이 없어질 때까지
        Note over T,A: ① SCAN — 정지 상태에서만 관측
        T->>P: scan_floor()
        P-->>T: list[Detection] (3범주 · pose · dims · yaw)
        Note right of P: 모서리 고정 웹캠 1프레임 (1080p)<br/>호모그래피 입력은 박스 아래 모서리<br/>상자 영역 마스킹 · 로봇 차폐 시 이월

        alt 미처리 대상 0개
            T-->>OP: DONE — 결과 보고
        else 미처리 대상 ≥ 1
            Note over T: ② SELECT — 포트 호출 없음<br/>최근접 미처리 대상 1개 선정

            Note over T,A: ③ APPROACH
            T->>B: drive_to(파지 접근 지점)
            B-->>T: arrived

            Note over T,A: ④ GRASP — 부하 검증 (상세: §2)
            T->>A: 파지 + load 확인
            A-->>T: 파지 성공 (시도 n회)

            Note over T,A: ⑤ TRANSPORT
            T->>P: find_box(rule[cls])
            P-->>T: BoxObservation (색 랜드마크 · LAB)
        Note right of P: ⚫ BLACK 상자는 밝은 테두리/ArUco 로 탐색
            T->>B: drive_to(상자 앞) → align_to_box()
            B-->>T: yaw 오차 이내
            Note right of A: 이송 중 물체는 수평 유지<br/>Transport Pose — 무게중심 안쪽

            Note over T,A: ⑥ POSE_PLAN → ⑦ INSERT (상세: §3)
            T->>P: measure_opening(box)
            P-->>T: opening_mm
            alt φ 해 없음
                T-->>OP: REJECT — "이 물체는 넣을 수 없습니다"
                Note right of T: held_ids 등록 후 SCAN 복귀
            else φ 해 존재
                T->>A: reorient(φ) → 투입 → set_gripper(OPEN)
                A-->>T: 투입 완료
                Note right of T: done_ids 등록 후 SCAN 복귀
            end
        end
    end

    opt E-STOP (임의 시점)
        OP->>T: estop.set()
        T->>B: stop()
        T->>A: hold_position()
        Note right of A: 현재 자세 래치 · 낙하 방지
    end
```

> **관측은 정지 상태에서만** 합니다. 주행 중 모션 블러와 진동으로 검출 신뢰도가 떨어지고,
> 이는 음성을 정지 상태에서만 녹음하는 것과 같은 원칙입니다 — **자기 소음·자기 진동 배제.**

---

## 2. 파지 검증 및 자동 재시도

그리퍼를 닫은 뒤 **서보 부하값으로 물체 유무를 판정**합니다.
별도 힘/토크 센서 없이 폐루프를 구성하는 것이 핵심입니다.

```mermaid
sequenceDiagram
    autonumber
    participant T as MissionTask<br/>(GraspState)
    participant P as Perception
    participant A as ArmDriver

    Note over T,A: 부하 기반 파지 검증 — 힘 센서 없음

    loop 대상 1개 기준 · attempt ≤ MAX_GRASP_RETRY (3)
        T->>A: move_to_cartesian(접근 지점)
        T->>A: move_to_cartesian(파지 지점, down=True)
        T->>A: set_gripper(CLOSED_MM)
        T->>A: get_load()
        A-->>T: load_ratio (0.0~1.0)

        alt load_ratio ≥ LOAD_THRESHOLD
            Note right of A: 파지 성공 → TRANSPORT / DELIVER
        else load_ratio < LOAD_THRESHOLD
            Note right of A: 빈손 — 그리퍼가 끝까지 닫힘
            T->>A: set_gripper(OPEN_MM)
            T->>P: scan_floor()
            P-->>T: 갱신된 목록 — 대상 pose 보정
            Note right of P: 실패한 파지가 물체를 밀었을 수 있음<br/>이전 pose 재사용은 같은 실패를 반복
        end
    end

    Note over T,A: 재시도 소진 → held_ids 등록 후 SCAN 복귀<br/>미션은 끝나지 않는다
```

| 항목 | 내용 |
|---|---|
| 감지 방식 | 그리퍼 서보(id6) 부하 비율 `load_ratio` |
| 값의 범위 | **0.0~1.0 정규화 비율.** 서보 원시값(STS3215 `PRESENT_LOAD` = 0~1023)을 `abs(raw)/1023` 으로 바꾸는 것은 `arm_driver_node` 의 몫입니다 — 서보 각도 변환과 같은 이유입니다(`class_diagram.md` §2). 부호(0x400 비트)는 방향인데 실측에서 일관되지 않아(같은 '빈 채'가 −88 로도 +124 로도) 버립니다 |
| 임계값 | **`LOAD_THRESHOLD = 0.04`** — 실측(2026-08-18, n=25, 정착 후) 두 분포 사이. 아래 표 참조 |
| 정착 대기 | **`GRASP_SETTLE_SEC = 1.5`** (노드). 정착 전에는 빈 채와 물체가 모두 포화값(±500)이라 구분이 불가능합니다 |
| 재시도 상한 | `MAX_GRASP_RETRY = 3` — **대상 1개당**. `SELECT` 가 새 대상을 고를 때 예산을 되돌립니다 (`state_machine.md` §3) |
| 실패 시 동작 | 개방 → **재스캔** → 보정된 pose로 재시도 |
| 소진 시 | `held_ids` 등록 → `SCAN` 복귀 (**미션 계속**) |

> **선형 FSM과의 결정적 차이가 마지막 줄입니다.** 이전에는 `GraspFailedState` 가 `None` 을 반환해
> 미션이 종료됐습니다. 지금은 실패한 물체를 보류하고 다음 물체로 갑니다 — **유즈케이스 3.**

### 부하 임계값 실측 (2026-08-18, n=25)

정착 2초 후 · 절대값 기준. 정규화는 `raw / 1023`.

| 상황 | raw | 정규화 |
|---|---|---|
| 빈 채 / 파지 실패(놓침) | 28, 32 | 0.027 ~ 0.031 |
| 체스말(나이트·룩) | 48 ~ 124 | 0.047 ~ 0.121 |
| 가베(정육면체) | 140 (5/5 일관) | 0.137 |
| 체스말(퀸, 유선형이라 불안정) | 32(놓침) 또는 160 ~ 168 | 0.031 / 0.156 ~ 0.164 |

**빈 채 최대 0.031 < `LOAD_THRESHOLD` = 0.04 < 파지 성공 최소 0.047.**
⚠️ 여유가 크지 않습니다 — raw 기준 32 대 48로 **16틱** 차이입니다. 파지 대상 클래스가
추가되면 재측정해야 합니다.

> **이전 값 `0.15` 는 어떤 물체도 넘지 못했습니다** — 가베조차 0.137 로 미달이라 실기에서
> 파지 판정이 항상 실패했습니다. 여기에 real 어댑터가 정규화하지 않은 원시값(−88, 124)을
> 돌려주고 Fake 는 0~1 값을 돌려주던 계약 분기가 겹쳐, **CI는 통과하는데 실기만 실패하는**
> 상태였습니다.

### 정착 시간 실측

닫힘 명령 후 시간별 부하:

| 경과 | 부하 | 판정 가능? |
|---|---|---|
| 0.26 ~ 0.51s | 500 (포화) | ❌ 이동 중 — 빈 채/물체 동일 |
| 0.77s | 거의 안정 | △ |
| 1.03s 이후 | 완전 고정 (10초까지 한 틱도 변화 없음) | ✅ |

`set_gripper` 가 위치를 명령한 뒤 `GRASP_SETTLE_SEC` 만큼 기다린 다음 응답합니다.
**타이밍 지식은 노드에 둡니다** — `GraspState` 에 `sleep` 을 넣으면 도메인이 서보 물리를
알게 됩니다. `GraspState` 는 `set_gripper()` 응답을 쓰지 않고 `get_load()` 를 따로 부르지만,
`set_gripper` 가 정착까지 붙들고 있으므로 그 다음 호출이 정착 후 값을 읽습니다.

---

## 3. 상자 투입 자세 재조정 (⏸ 보류)

긴 막대(L=0.50 m)를 좁은 상자 입구(W_open=0.40 m)에 넣으려면 **세워야** 합니다.
프로젝트에서 유일하게 닫힌 형태 해가 있는 기하 계획이자, 시연의 하이라이트입니다.

```mermaid
sequenceDiagram
    autonumber
    participant T as MissionTask<br/>(PosePlan → Insert)
    participant P as Perception
    participant B as BaseDriver
    participant A as ArmDriver

    Note over T,A: 전제 — 파지 직후 물체는 수평(φ=0)<br/>요(yaw)는 align_to_box()에서 이미 정렬 → 1자유도 문제

    T->>P: measure_opening(box)
    P-->>T: opening_mm (상자 입구 짧은 변)
    Note right of P: dims_m 은 SCAN 시점 확보<br/>단안이므로 눕힌 물체의 바닥 평면 치수<br/>세운 물체는 클래스 사전값 폴백

    Note over T: solve φ<br/>L·|cos φ| + w·|sin φ| ≤ W_open − margin<br/>φ = 장축과 수평면 사이 각도 (rad)

    alt 해 구간 없음
        T-->>T: REJECT — 투입 불가 판정
        T->>A: move_to_cartesian(바닥 내려놓기)
        T->>A: set_gripper(OPEN_MM)
        Note right of T: 물체를 든 채 미션을 끝내지 않는다<br/>held_ids 등록 후 SCAN 복귀
    else 해 구간 존재
        Note over T: 해 구간 중 손목 서보 부하 최소 φ 선택 (발열 억제)
        T->>B: stop()
        Note right of B: ⚠️ 자세 전환은 반드시 정지 상태에서<br/>주행 중 전환 시 무게중심 이탈 → 전복
        T->>A: reorient(φ)
        A-->>T: is_settled = true
        Note right of A: 수평→수직은 피치 회전<br/>손목 단독 불가 — 어깨·팔꿈치 포함 IK 전체 관여

        loop 투입 중
            T->>P: monitor_clearance()
            P-->>T: front/left/right + contact_risk
            Note right of P: 상자·가구 접촉 = 성공 기준 위반<br/>접촉 예상 시 즉시 정지
        end

        T->>A: move_to_cartesian(입구 상단 → 하강)
        T->>A: set_gripper(OPEN_MM)
        T->>A: fold_to_cradle()
        A-->>T: 투입 완료 (접촉 0회)
    end
```

### 자세 계획

```
H_proj(φ) = L·|cos φ| + w·|sin φ|   ≤   W_open − margin

  L      = 0.50 m   막대 길이 (추정값, SCAN에서 측정)
  w      = 0.045 m  지름 (추정값)
  W_open = 0.40 m   상자 입구 짧은 변 (미결 #8 — 발주 후 실측)
  margin = 0.03 m   안전 여유 (미결 #7 — 고정 / 학습)
  φ      = 장축과 수평면 사이 각도 (rad), 파지 직후 0°

→ φ ≥ 0.83 rad (48°)
```

| W_open | 최소 φ |
|---|---|
| 0.35 m | 55.5° |
| 0.40 m | 47.7° |
| 0.45 m | 41.8° |

> **⚠️ 이전 주제와 부등호가 반대입니다.** 암실 시나리오는 낮은 개구부 **밑을 지나느라 눕혔고**
> (`sin`/`cos` 위치가 반대, `φ ≲ 27°`), 지금은 좁은 입구에 **넣느라 세웁니다.**
> 수식 계열은 같지만 코드를 복사하면 부호가 틀립니다.

> [!NOTE]
> **이 시퀀스는 현재 보류 상태입니다.** 긴 물체가 정리 대상에서 제외되어 자세 재조정을
> 실행할 클래스가 없습니다. 가베·체스말은 모두 `φ=0` 으로 통과합니다.
> 미배정 상자에 긴 물체를 배정하면 즉시 되살아나므로 시퀀스는 그대로 보존합니다.

---

## 4. FETCH 분기

TIDY와 **같은 하드웨어·같은 포트·같은 파지 로직**을 씁니다. 파지 이후 목적지 결정에서만 갈라집니다.

```mermaid
sequenceDiagram
    autonumber
    actor OP as 사용자
    participant I as CommandInterpreter
    participant T as MissionTask
    participant P as Perception
    participant B as BaseDriver
    participant A as ArmDriver

    OP->>I: "체스말 가져와"
    I-->>T: MissionSpec(mode=FETCH, target_cls=CHESS_PIECE)

    T->>P: scan_floor()
    P-->>T: list[Detection]
    Note over T: SELECT — target_cls 일치 필터 추가<br/>TIDY와 다른 유일한 판단 지점

    Note over T,A: APPROACH → GRASP — TIDY와 완전 동일 (§2)

    Note over T,A: DELIVER
    T->>B: drive_to(사용자 위치)
    B-->>T: arrived
    Note right of B: 상자가 아니라 사람이 목적지<br/>find_box() 호출 없음

    Note over T,A: HANDOVER
    T->>A: move_to_cartesian(인계 높이)
    T->>A: set_gripper(OPEN_MM)
    T->>A: get_load()
    A-->>T: load_ratio ≈ 0 (사람이 받아감)
    T-->>OP: "여기 있습니다"
    Note right of T: done_ids 등록 후 SCAN 복귀
```

**분기점은 `GraspState.execute()` 의 마지막 한 줄입니다.**

```python
return TransportState(...) if self.ctx.spec.mode is MissionMode.TIDY else DeliverState(...)
```

> 두 모드가 공유하는 코드량이 이렇게 큰 것은 설계 의도입니다.
> 모드가 늘어도 파지 로직은 한 곳에만 있습니다.

---

## 5. 음성 명령 — 복창과 되묻기

**도메인 코드는 0줄 바뀝니다.** `voice_io` 노드가 기존 명령 토픽에 텍스트를 발행할 뿐입니다.

```mermaid
sequenceDiagram
    autonumber
    actor OP as 사용자
    participant V as voice_io 노드<br/>(STT · TTS)
    participant I as CommandInterpreter
    participant T as MissionTask

    Note over OP,T: 케이스 A — 명확한 명령
    OP->>V: [푸시투토크 누름] "긴 막대 가져와"
    Note right of V: 정지 상태에서만 녹음<br/>모터·서보 소음이 마이크를 덮음
    V->>I: /command "체스말 가져와" (std_msgs/String)
    I-->>V: confirm_phrase → "체스말을 가져올게요"
    V-->>OP: [TTS] "체스말을 가져올게요"
    OP->>V: "응"
    V->>T: MissionSpec 확정 → 실행

    Note over OP,T: 케이스 B — 모호한 명령
    OP->>V: [PTT] "그거 가져와"
    V->>I: /command "그거 가져와"
    I-->>V: CLARIFY — 대상 미상
    V-->>OP: [TTS] "가베인가요, 체스말인가요?"
    OP->>V: "체스말"
    V->>I: /command "체스말" (CORRECT)
    I-->>V: confirm_phrase → "체스말을 가져올게요"
    V-->>OP: [TTS] 복창
    OP->>V: "응"
    V->>T: MissionSpec 확정 → 실행

    Note over OP,T: 케이스 C — STT 오인식
    OP->>V: [PTT] "체스말은 검은 상자에"
    V->>I: /command "체스팔은 검은 상자에" ← 오인식
    I-->>V: CLARIFY — 클래스 미상
    V-->>OP: [TTS] "다시 말씀해 주세요"
    Note right of V: 오실행률 0% 가 성공 기준<br/>확인 없는 실행 경로는 존재하지 않는다
```

| 항목 | 결정 | 이유 |
|---|---|---|
| 푸시투토크 | ✅ 채택 | 로봇 소음이 마이크를 덮음, 시연장 오인식 방지 |
| 웨이크워드 상시 대기 | ❌ 미채택 | 위와 동일 + 시연 통제 |
| 녹음 시점 | **정지 상태에서만** | 자기 소음 배제 (관측과 동일 원칙) |
| 실행 전 복창 | ✅ 필수 | STT는 반드시 틀림 |
| 텍스트 폴백 | ✅ 유지 | 음성만 되는 상태를 만들지 않음 |

> **비전공자가 가장 크게 반응하는 장면은 로봇이 되묻는 순간입니다.**
> 기존 `CLARIFY` → `CORRECT` 문형이 그대로 음성 대화가 되므로,
> 명령 해석 로직을 새로 만드는 게 아니라 입출력 채널만 붙이면 됩니다.

---

## 참고

| 문서 | 내용 |
|---|---|
| [`state_machine.md`](state_machine.md) | **FSM 전이 단일 소스** |
| [`class_diagram.md`](class_diagram.md) | 포트 시그니처, 값 객체 |
