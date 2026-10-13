# 시퀀스

> 기준: `vla_robot/` 코드 (2026-10-13). 상태 이름은 [state_machine.md](state_machine.md), 메시지 필드는 [interfaces.md](interfaces.md).

등장하는 쪽:

| 이름 | 실체 |
|---|---|
| 사람 · UI | 시연 UI(입력창 · 마이크) |
| Host | `run_host.py` — 위치 추정 · 검출 · 미션 FSM (노트북) |
| 미션 | `pi_mission_node` (Pi) |
| 파지 | `vla_grasp_node` — ACT 정책 루프 (Pi) |
| 팔 | `arm_driver_node` — 서보 버스 (Pi) |
| 차체 | `odom_publisher` → 제조사 컨트롤러 → 바퀴 (Pi) |

## 1. 정리 — 기물 하나

```mermaid
sequenceDiagram
  autonumber
  actor U as 사람 · UI
  participant H as Host
  participant M as 미션 (Pi)
  participant G as 파지 (Pi)
  participant A as 팔 (Pi)
  participant B as 차체

  U->>H: "모두 정리해줘"
  H->>H: Claude 로 해석 → 보이는 라벨 중 대상 · 개수(all)
  loop 대상이 남아 있는 동안
    H->>H: SEARCH_TARGET — 다음 기물 고르기
    loop APPROACH_PIECE (10 Hz)
      H->>M: HostCommand(APPROACH, 속도)
      M->>B: controller/cmd_vel
    end
    Note over H: 정면 · 거리 0.26–0.31 m
    H->>M: HostCommand(GRASP, label) — 결과 나올 때까지 반복
    M->>B: 정지 (+ 차체 컨트롤러 소음 정리 재기동)
    M->>A: move_to_pose(start)
    M->>M: 기준 사진(그리퍼캠)
    M->>G: vla/run_grasp(label)
    loop 청크마다 (약 3.7 s)
      G->>G: 그리퍼 영상 + 관절 → ACT → 관절 청크
      G->>A: arm/execute_chunk
    end
    G-->>M: 사이클 끝(복귀)
    M->>A: 그리퍼 정렬 · move_to_pose(start, 그리퍼 유지)
    M->>M: 파지 확인 — 기준 사진과 비교(턱 앞 변화 ≥ 18 %)
    M->>A: move_to_pose(carry)
    M-->>H: PiStatus(result: GRASP ok, job_id 증가)
    alt 빈손
      H->>H: APPROACH_PIECE 로 — 위치 다시 읽고 재시도(1번), 그래도 빈손이면 보류
    else 쥠
      loop CARRY_TO_DEST → NUDGE_BOX
        H->>M: HostCommand(CARRY / APPROACH_BOX, 속도)
        M->>B: cmd_vel
      end
      Note over H: 정차 구역 도착 → 바구니 쪽 회전, 남는 각 = arm_yaw_deg
      H->>M: HostCommand(PLACE, arm_yaw_deg)
      M->>A: carry → drop(+ base 회전) → 그리퍼 열기 → return
      M-->>H: PiStatus(result: PLACE ok)
    end
  end
  H-->>U: 완료 화면(넣은 개수)
```

- 한 기물 파지는 실기에서 15~21 s(4~5 청크), 투입은 약 9 s 입니다.
- 바구니로 가는 움직임은 기물 · 자리와 관계없이 **회전 → 직진 → 회전 → 투입** 한 모양을 씁니다([decisions.md](decisions.md)).

## 2. 가져와 — 사람 손에 건네기

```mermaid
sequenceDiagram
  autonumber
  actor U as 사람 · UI
  participant H as Host
  participant M as 미션 (Pi)
  participant A as 팔 (Pi)

  U->>H: "룩 가져와" (손을 장판 위에)
  H->>H: Claude 해석 → intent = fetch · 손 검출 켬
  Note over H: 지시 받을 때 손이 보여야 접수(없으면 바구니로 바꿀지 묻는 카드)
  H->>M: (정리와 같은) 접근 · 파지
  M-->>H: GRASP ok
  H->>H: 지금 보이는 손을 목적지로 — 손 앞 정차점
  loop CARRY_TO_DEST → FACE_HAND
    H->>M: HostCommand(CARRY / APPROACH_BOX, 속도)
  end
  H->>M: HostCommand(PLACE, place_pose = handover)
  M->>A: carry → handover 자세(ㄱ자)
  M->>M: 1 s 기다림(손 맞출 시간)
  M->>A: 그리퍼 열기 → return
  M-->>H: PLACE ok — "건네기 완료"
  H->>H: 건넨 기물은 60 s 동안 손 자리 근처에서 지도에서 뺀다
```

- 손은 MediaPipe 로 찾고, 두 카메라가 같은 손을 보면 광선 교차로 높이까지 풉니다. 손이 옮겨 가면 파지 뒤 **보이는 손**으로 갑니다.
- 손 검출은 CPU 를 아끼려고 "가져와" 해석 · 대기 · 건네기 때만 켭니다.

## 3. 차체 무응답 복구

차체 보드가 쓰기를 오류 없이 무시하는 고장(2026-09-23 · 09-30)에 대한 대응입니다.

```mermaid
sequenceDiagram
  autonumber
  participant H as Host
  participant M as 미션 (Pi)
  participant C as 차체 컨트롤러

  H->>M: HostCommand(APPROACH, 속도)
  M->>C: cmd_vel
  Note over H: 탑뷰상 1.5 s 동안 1 cm · 2° 도 안 움직임 → 무응답
  H->>M: HostCommand(stop, recover_base = true)
  M->>C: 컨트롤러 노드 다시 띄움(포트 재오픈 → 보드 리셋)
  M-->>H: PiStatus(base_recovering = true)
  M-->>H: PiStatus(base_recoveries + 1)
  Note over H: 횟수가 늘면 복구 끝 → 같은 상태로 이어서
  alt 연속 2번 실패 · 시간 초과
    H->>H: HALTED — 사람이 확인
  end
```

미리 막는 재기동도 있습니다 — 스택 기동 직후, 팔 작업 시작 때(바퀴 울음 정리), **60 s 넘게 서 있다 첫 출발 전**(약 6 s).
그래도 드물게(재기동 5 s 뒤 27번 중 1번) 무응답이 남아 위 복구가 처리합니다.
