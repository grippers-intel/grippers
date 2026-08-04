# Sequence Diagrams

Gripper의 미션 흐름과 핵심 기능별 상세 시퀀스입니다.

모든 상호작용은 도메인(`MissionTask`)과 **포트** 사이에서만 일어납니다. ROS2 노드, Feetech 서보 SDK, OpenCV는 다이어그램에 등장하지 않습니다 — 어댑터 뒤에 숨어 있기 때문입니다.

- [전체 미션 흐름](#전체-미션-흐름)
- [파지 검증 및 자동 재시도](#파지-검증-및-자동-재시도)
- [장축 물체 협로 통과](#장축-물체-협로-통과)

---

## 전체 미션 흐름

일반 환경(Station B) ↔ 제한 조명 구역(Station A) 왕복 회수 미션의 단계별 흐름입니다. 상세가 필요한 구간은 아래 개별 다이어그램으로 분리했습니다.

```mermaid
sequenceDiagram
    autonumber
    actor OP as 작업자<br/>(격벽 밖)
    participant L as VLA-L<br/>명령 해석
    participant T as mission_orchestrator<br/>(FSM · Domain)
    participant V as VLA-V / YOLO<br/>Perception
    participant B as BaseDriver
    participant A as VLA-A<br/>ArmDriver

    Note over OP,A: ① IDLE — 작업실 · 정상광
    OP->>L: 키보드 텍스트 명령 입력
    L-->>T: MissionSpec(대상, 제약)
    Note right of L: 진입 후 추가 명령 없음<br/>로봇 단독 수행

    Note over OP,A: ② TRANSIT_OUT — 회피기동 구간
    T->>A: fold_to_cradle() → torque_off()
    T->>B: 작업실 퇴출 → 좌회전 (빈손, 일반 도어)
    B-->>T: 정적 장애물 회피 후 arrived

    Note over OP,A: ③ LIGHT_ADAPT — 암실 경계
    T->>V: set_light_profile(DARKROOM)
    Note right of V: 노출·WB 고정 · IR 능동 조명 점등<br/>이 구간 인식 결과 미채택<br/>주행은 LiDAR로 계속
    V-->>T: profile_ready

    Note over OP,A: ④ DOCKING — 폐루프 정렬
    T->>V: detect_marker() (IR 반사 마커)
    V-->>T: 정렬 오차
    T->>B: 보정 명령 (홀로노믹 strafe)
    Note right of B: 오도메트리 누적오차 리셋

    Note over OP,A: ⑤ IDENTIFY — 정지 상태에서만
    T->>V: detect_target() × N 프레임 합의
    V-->>T: 장물 pose · 길이 L · 폭 w
    Note right of V: IR 스테레오 raw 기반<br/>치수는 사전에 주어지지 않음

    Note over OP,A: ⑥ GRASP — 수직 파지
    T->>A: 파지 + 서보 부하 검증 (상세: grasp-retry)
    A-->>T: 파지 성공 (시도 n회)
    Note right of A: 장물은 암이 든 채 이동<br/>트레이 적재 없음 — 저속 프로파일

    Note over OP,A: ⑦ POSE_PLAN — 개구부 추정
    T->>V: measure_gap()
    V-->>T: 개구부 높이 H_gap
    T->>T: solve φ (상세: narrow-exit)
    alt 해 존재
        Note right of T: 통과 가능 자세 확정
    else 해 없음
        T-->>OP: "통과 불가" 판정 후 거부 · 원위치 복귀
    end

    Note over OP,A: ⑧ NARROW_EXIT — 핵심 동작
    T->>A: 자세 재조정 후 저속 통과 (상세: narrow-exit)
    A-->>T: 무접촉 통과 완료

    Note over OP,A: ⑨ RETURN — 회피기동 구간
    T->>V: set_light_profile(NORMAL)
    T->>B: 좌회전 → 작업실 재입장
    B-->>T: arrived

    Note over OP,A: ⑩ RELEASE
    T->>A: 지정 슬롯 배치 → fold_to_cradle()
    T-->>OP: 결과 보고 (성공 여부 · 접촉 횟수 · 소요 시간)

    opt E-STOP (임의 시점)
        OP->>T: emergency_stop()
        T->>B: stop()
        T->>A: hold_position()
        Note right of A: 현재 자세 래치 · 낙하 방지
    end
```

---

## 파지 검증 및 자동 재시도

그리퍼를 닫은 뒤 **서보 부하값으로 물체 유무를 판정**합니다. 별도의 힘/토크 센서 없이 폐루프를 구성하는 것이 핵심입니다.

```mermaid
sequenceDiagram
    autonumber
    participant T as MissionTask<br/>(Domain)
    participant P as Perception
    participant TF as TransformProvider
    participant A as ArmDriver

    Note over T,A: 부하 기반 파지 검증 및 자동 재시도

    loop attempt ≤ MAX_RETRY
        T->>TF: transform(target → arm_base)
        TF-->>T: PoseInFrame(target, arm_base)
        T->>A: move_to_cartesian(approach)
        T->>A: move_to_cartesian(grasp, top-down)
        T->>A: set_gripper(CLOSED)
        T->>A: load()
        A-->>T: load_ratio

        alt load_ratio ≥ 임계값
            Note right of A: 파지 성공 → 루프 탈출
        else load_ratio < 임계값
            Note right of A: 파지 실패
            T->>A: set_gripper(OPEN)
            T->>P: detect_targets()
            P-->>T: 보정된 target
        end
    end

    Note over T,A: 별도 힘 센서 없이 서보 부하만으로 폐루프 구성
```

**설계 의도**

| 항목 | 내용 |
|---|---|
| 감지 방식 | 그리퍼 서보 부하 비율 (`load_ratio`) |
| 임계값 | *1주차 실측 후 확정* |
| 재시도 상한 | `MAX_RETRY` — 초과 시 상위 상태로 실패 보고 |
| 실패 시 동작 | 그리퍼 개방 → 재인식 → 목표 자세 보정 |

---

## 장축 물체 협로 통과

긴 물체를 든 채 좁은 통로를 지날 때, 그리퍼 요(yaw) 회전으로 진행 방향 투영 폭을 줄입니다.

```mermaid
sequenceDiagram
    autonumber
    participant T as mission_orchestrator<br/>(FSM · Domain)
    participant V as VLA-V<br/>Perception
    participant B as BaseDriver
    participant A as VLA-A<br/>ArmDriver

    Note over T,A: 장물 자세 재조정 후 좁은 출구 통과 — 프로젝트 핵심 동작

    Note over T,A: 전제 · 개구부는 높이 제한(30cm)이고 장물은 0.5m<br/>수직으로 파지되어 있으므로 눕히지 않으면 통과 불가

    T->>V: measure_object_dims()
    V-->>T: L (길이), w (폭)
    T->>V: measure_gap()
    V-->>T: H_gap (개구부 높이), 중심선
    Note right of V: 치수는 사전에 주어지지 않음<br/>추정 오차 ↔ 안전 마진 트레이드오프

    Note over T: solve φ<br/>L·|sin φ| + w·|cos φ| ≤ H_gap − margin<br/>φ = 장축과 수평면 사이 각도

    alt 해 구간 없음
        T-->>T: 통과 불가 판정 → 거부 · 복귀
    else 해 구간 존재
        Note over T: 해 구간 중 손목 서보 부하 최소 φ 선택 (발열 억제)
        T->>B: align_to_centerline()
        B-->>T: 정렬 완료
        T->>A: reorient_wrist(φ)
        A-->>T: is_settled = true
        Note right of A: 중심잡기 — 무게중심이 베이스 밖으로<br/>가감속이 외란으로 작용
        T->>B: drive_straight(저속 프로파일)
        loop 통과 중
            T->>V: monitor_clearance()
            V-->>T: 상하좌우 여유
            Note right of V: 접촉 예상 시 즉시 정지<br/>벽 접촉 = 오염 = 즉시 실패
        end
        B-->>T: 통과 완료 (접촉 0회)
    end
```

**자세 계획**

```
W_proj(θ) = L·|sin θ| + w·|cos θ|   ≤   D_gap − margin

  L      : 물체 길이
  w      : 물체 폭
  θ      : 진행축 대비 물체 요 각도
  D_gap  : 통로 유효 폭
  margin : 안전 여유
```

해 구간이 여러 개일 경우, **손목 서보 부하가 최소가 되는 θ** 를 선택합니다. 장시간 유지 시 발열을 억제하기 위함입니다.

> [!NOTE]
> 이 기능은 Target 등급입니다. 접촉 0회가 성공 기준이며, 통과 중 여유 거리를 상시 감시합니다.
