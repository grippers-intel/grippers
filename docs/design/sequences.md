# Sequence Diagrams

> **상태: 실제 코드 기준으로 전면 재작성 (2026-09-04).** 이 문서가 이전에 그리던 TIDY/FETCH
> 루프(자연어 명령 → `CommandInterpreter` → `scan_floor`/`SELECT` → `TRANSPORT`/`POSE_PLAN` →
> `DELIVER`/`HANDOVER`, 8/14 freeze 목표)는 **채택되지 않았다.** 실제로 팀이 확정해 구현한
> 것은 `domain/task/baseline_mission.py`의 Host 지시 실행형 FSM이다. 전이 그래프의 단일
> 소스는 [`state_machine.md`](state_machine.md)이고, 이 문서는 그 상태들 **내부에서** 포트를
> 어떤 순서로 부르는지만 다룬다.

모든 상호작용은 도메인(`BaselineMission`)과 **포트**(`BaselinePorts`: `host`·`base`·`arm`·
`perception`·`lidar`) 사이에서만 일어난다. ROS2 노드, Feetech 서보 SDK, UDP 소켓은
다이어그램에 등장하지 않는다 — 어댑터 뒤에 숨어 있기 때문이다.

- [1. APPROACH — GRASP 조건 판정](#1-approach--grasp-조건-판정)
- [2. GRASP — 파지 시퀀스 (execute 1회로 끝까지)](#2-grasp--파지-시퀀스-execute-1회로-끝까지)
- [3. CARRY — INSERT 조건 판정](#3-carry--insert-조건-판정)
- [4. INSERT — 투하 시퀀스](#4-insert--투하-시퀀스)
- [5. 공통 인터럽트 — E-STOP · LinkWatchdog](#5-공통-인터럽트--e-stop--linkwatchdog)

---

## 1. APPROACH — GRASP 조건 판정

Host가 `state=GRASP`(또는 `GRASP_FORCE`)를 보내는 순간 시작된다. 판정은 두 겹이고,
어느 하나라도 막히면 **그 자리에 머물며** Host에 보정값을 실어 보낸다 — Pi가 스스로
자세를 고치지 않는다.

```mermaid
sequenceDiagram
    autonumber
    participant H as Host<br/>(UDP)
    participant M as BaselineApproachState
    participant P as Perception
    participant B as BaseDriver

    H->>M: HostCommand(state=GRASP)
    M->>B: stop()
    M->>P: identify_target()
    P-->>M: TargetObservation|None

    Note over M: 1겹 — 기본 전제(check_grasp)<br/>정지 + 라벨 식별
    alt 기본 전제 미충족
        M->>H: report(GRASP_BLOCKED, detail, fix)
        Note right of M: force 여도 이 겹은 건너뛰지 않는다
    else 기본 전제 충족
        Note over M: 2겹 — 정렬 판정(grasp_alignment.judge)<br/>물체가 턱이 쓸고 갈 영역(READY) 안인가
        alt READY (또는 force && HOST_CORRECTION)
            M->>H: report(GRASP_READY, detail)
            Note right of M: BaselineGraspState로 전이 (§2)
        else HOST_CORRECTION / UNKNOWN
            M->>H: report(GRASP_BLOCKED, detail, fix)
            Note right of M: UNKNOWN(관측 자체 실패)은<br/>force여도 건너뛰지 않는다
        end
    end
```

**이 판정 한 번에 약 1.7초가 든다**(오검출을 거르는 5프레임 합의 × 프레임당 CPU 추론
0.3초). 그동안 Host 명령을 읽지도 보고하지도 않는다 — 워치독은 "명령이 안 온 것"과
"안 읽은 것"을 구분하므로 여기서는 안 걸리지만, **Host 쪽 타임아웃은 이보다 넉넉해야
한다**(`baseline_mission.py` 주석).

---

## 2. GRASP — 파지 시퀀스 (execute 1회로 끝까지)

`BaselineGraspState.execute()` 한 번 안에서 순서대로 진행한다. Host 명령을 중간에 읽지
않는다 — 이 상태에 있는 동안 Host는 다음 지시를 낼 수 없다.

```mermaid
sequenceDiagram
    autonumber
    participant M as BaselineGraspState
    participant P as Perception
    participant A as ArmDriver
    participant B as BaseDriver
    participant H as Host

    M->>B: stop()
    M->>P: remember_target(label)
    Note right of P: grasp 자세로 내려가면<br/>팔이 뎁스캠을 가린다 — 마지막 기회

    M->>A: move_to_floor_pose(profile, "safe")
    M->>A: set_gripper(preopen_width_mm)
    Note right of A: 내려가기 전에 연다 —<br/>닫힌 손가락은 물체를 밀어낸다
    M->>A: move_to_floor_pose(profile, "grasp")

    M->>B: creep_forward_timed(0.1 m/s, 1.5s)
    Note right of B: 팔이 내려가 그리퍼가 열린 **뒤**에만.<br/>회전 절대 금지 — 열린 그리퍼가<br/>바닥·물체를 옆으로 쓴다

    M->>A: set_gripper(close_width_mm)
    Note right of A: 여기서 부하를 미리 재지 않는다(09-03) —<br/>정착된 자세에서 부하 판독은 낮게<br/>오탐할 수 있다(box 실측)

    M->>A: move_to_floor_pose(profile, "midpoint")
    M->>A: move_to_floor_pose(profile, "safe")
    M->>A: move_to_floor_pose(profile, "carry")

    M->>A: get_load()
    A-->>M: carried
    M->>P: confirm_grasp()
    P-->>M: vanished

    alt carried >= LOAD_THRESHOLD AND vanished
        M->>H: report(GRASP_DONE, state=CARRY, detail)
        Note right of M: BaselineCarryState(grasp_confirmed=True)로 전이
    else 둘 중 하나라도 미충족
        M->>A: move_to_floor_pose(profile, "recover_idle")
        Note right of A: 실패해도 hold_position() —<br/>팔을 바닥 높이에 남기지 않는다
        M->>H: report(GRASP_FAILED, state=APPROACH, detail)
        Note right of M: 재시도 상한 없음 — Host가 다음을 정한다
    end
```

**AND 판정의 이력이 중요하다** — 2026-08-26~09-01엔 AND, 09-01 rook 뎁스 오탐으로 OR로
완화, 09-03 star/box가 반대 방향(부하만 낮게 오탐)을 내면서 다시 AND로 돌아왔다. 두 신호
모두 근접 상황에서 개별적으로 흔들릴 수 있다는 뜻이라, 어느 한쪽만 믿는 설계는 이미 두
번 실패했다(`baseline_mission.py` GRASP 판정부 주석 전문 참고).

---

## 3. CARRY — INSERT 조건 판정

CARRY는 매 사이클 라이다와 부하를 떠 둔다 — INSERT 판정이 "직전 사이클 대비 안정적인가"를
보려면 비교할 표본이 이미 있어야 왕복이 한 번 줄기 때문이다.

```mermaid
sequenceDiagram
    autonumber
    participant H as Host
    participant M as BaselineCarryState
    participant L as Lidar
    participant A as ArmDriver
    participant B as BaseDriver

    H->>M: HostCommand(state=CARRY|APPROACH_BOX|INSERT)
    M->>L: basket_face()
    L-->>M: BasketFace
    M->>A: get_load()
    A-->>M: load
    Note right of M: (face, load)를 이번 사이클 표본으로 보관

    alt state == APPROACH_BOX
        Note over M: 접근 중에도 매 사이클 확인(09-02 사고 이후)
        alt 라이다 거리 < BASKET_MIN_LIDAR_M
            M->>B: stop()
            M->>H: report(INSERT_BLOCKED, "너무 가깝다", fix)
        else 목표창 안(BASKET_STOP_TOLERANCE_M 이내)
            M->>B: stop()
            M->>H: report(APPROACH_BOX_READY, "그만 밀어도 된다")
        else 아직 창 밖
            M->>B: apply_velocity(Host 속도)
        end
    else state == INSERT
        M->>B: stop()
        Note over M: check_insert — 아래 조건 전부 AND
        Note right of M: E-STOP 안 걸림 · 정지 상태 ·<br/>grasp_confirmed(=GRASP에서 이미 확정, 재판정 안 함) ·<br/>라이다 정면 확보(점수·거리·yaw·좌우 허용치) ·<br/>직전 사이클 대비 거리·부하 변화가 허용치 안
        alt 전부 충족
            M->>H: report(INSERT_READY, detail)
            Note right of M: BaselineInsertState로 전이 (§4)
        else 하나라도 미충족
            M->>H: report(INSERT_BLOCKED, detail, fix)
            Note right of M: 표본 없으면 판정 보류,<br/>한 사이클 더 본다
        end
    else state == CARRY
        M->>B: apply_velocity(Host 속도)
    end
```

**`grasp_confirmed`를 여기서 raw 부하로 다시 재지 않는다.** GRASP가 CARRY 진입 시점에
이미 내린 판정(§2)을 그대로 믿는다 — box처럼 부하가 계속 낮게 읽히는 물체에서 재판정이
영원히 막히는 사고가 2026-09-03에 있었다(`baseline_mission.py` 주석).

---

## 4. INSERT — 투하 시퀀스

바닥 파지 높이로 내려가지 않는다. 실측 DROP 자세로 직접 전개한 뒤 그리퍼를 열고,
**부하 변화량**으로 놓임을 판정한다 — 별도 힘 센서가 없다.

```mermaid
sequenceDiagram
    autonumber
    participant M as BaselineInsertState
    participant A as ArmDriver
    participant H as Host

    M->>H: report(STATE, INSERT)
    M->>A: move_to_floor_pose(profile, "drop")

    alt drop 자세 실패
        M->>A: hold_position()
        M->>H: report(INSERT_FAILED, "투하 자세 실패")
        Note right of M: grasp_confirmed는 유지한 채 CARRY로 복귀 —<br/>그리퍼가 놓친 게 아니라 팔 자세만 실패
    else drop 자세 성공
        M->>A: get_load()
        A-->>M: before
        M->>A: set_gripper(release_width_mm)
        M->>A: get_load()
        A-->>M: after

        alt after <= before - RELEASE_LOAD_DROP(0.008)
            M->>H: report(INSERT_DONE, "부하 {before}→{after}")
        else 부하가 충분히 안 줄었다
            M->>H: report(INSERT_FAILED, "부하가 안 줄었다")
            Note right of M: 실패해도 접기는 한다 —<br/>전개한 채 두는 편이 더 위험
        end

        M->>A: set_gripper(CLOSED_MM)
        M->>A: move_to_floor_pose(profile, "idle")
        M->>H: report(IDLE_DONE, state=IDLE)
        Note right of M: 성공/실패 무관하게 무조건 IDLE로 복귀
    end
```

**`RELEASE_LOAD_DROP = 0.008`은 실측 2건(둘 다 실제 성공, 감소폭 0.0313·0.0117)에서
역산한 임시치다** — 실패(안 떨어짐) 사례가 아직 실측된 적이 없어 정확한 경계는 여전히
미실측이다. 옛 문턱 0.015는 2026-09-03 queen의 실제 성공(감소폭 0.0117)을 실패로
오판했다(`baseline_mission.py` `BaselineInsertState` docstring).

---

## 5. 공통 인터럽트 — E-STOP · LinkWatchdog

전이 그래프에는 등장하지 않는(`state_machine.md` §2 참고) 두 인터럽트다. 어느 State의
`execute()` 안에서도 같은 방식으로 작동한다.

```mermaid
sequenceDiagram
    autonumber
    participant Run as BaselineMission.run()
    participant S as 현재 State
    participant W as LinkWatchdog
    participant B as BaseDriver
    participant A as ArmDriver
    participant H as Host

    loop 매 사이클
        Run->>Run: ports.estop.is_set() ?
        alt E-STOP 걸림
            Run->>B: stop()
            Run->>A: hold_position()
            Note right of Run: BaselineEstopState로 강제 전환 —<br/>정상 전이가 아니라 인터럽트
        else E-STOP 안 걸림
            Run->>S: execute(ports)
            S->>H: latest_command()
            H-->>S: HostCommand|None
            S->>W: observe(command)
            alt 결측이 HOST_COMMAND_TIMEOUT_CYCLES(3) 미만
                W-->>S: True — 정상 진행
            else 3사이클 연속 결측
                W-->>S: False
                S->>B: stop()
                S->>H: report(REJECTED, "Host 명령이 N사이클 연속 없음 — 정지")
                Note right of S: None(안 옴)과 정지 명령은<br/>엄격히 다르게 취급한다
            end
        end
    end
```

---

## 참고

| 문서 | 내용 |
|---|---|
| [`state_machine.md`](state_machine.md) | **FSM 전이 단일 소스** |
| [`class_diagram.md`](class_diagram.md) | 포트 시그니처, 값 객체, ROS2 노드 계층 |
| [`architecture.puml`](architecture.puml) | 같은 구조의 PlantUML 버전 |
