# 작업 지시서 — Pi 쪽 UDP 브릿지 (Host PC ↔ 차량)

이 문서는 **다른 컴퓨터(Pi, 또는 Pi와 통신하는 개발 머신)에서 작업할 때
그대로 붙여넣어 쓰는 용도**다. Host PC 쪽 코드/맥락은 몰라도 되게 필요한
정보를 여기 다 담았다.

## 배경

탑뷰 카메라 2대 + ArUco 마커 + 객체인식(Geti)으로 로봇 위치와 기물 위치를
계산하는 Host PC(Windows)가 따로 있다. Host PC는 매 사이클(초당 대략
8~10회) "지금 바퀴가 뭘 해야 하는지" 계산해서 같은 와이파이로 UDP를 통해
차량(Pi)에 보낸다. **차량 쪽은 각도나 좌표 계산을 전혀 할 필요가 없다** —
Host가 이미 다 계산해서 "go"/"stop"/"yaw+"/"yaw-" 4가지 동작 중 하나로
정해서 보내준다.

## 할 일

1. Pi에서 **UDP 포트 5005**로 Host가 보내는 명령(JSON)을 계속 받는 리스너를
   만든다.
2. 받은 `cmd` 값에 따라 실제 바퀴를 움직인다 (아래 "동작 매핑" 참고).
3. `status` 가 `"GRASP"` 또는 `"PLACE"` 로 오면, 바퀴는 멈춘 채로 SmolVLA
   파지/배치 루틴을 실행한다 (그리퍼캠 + 차량 RGB캠 사용, 기존에 있는 걸
   그대로 씀).
4. 파지/배치가 끝나면 **UDP 포트 5006**으로 Host에 완료 보고(JSON)를 보낸다.
5. **워치독**: 일정 시간(예: 1~2초, Host와 실측해서 맞출 것) 안에 새 명령이
   안 오면 무조건 정지한다 — Host PC가 꺼지거나 와이파이가 끊겨도 차량이
   마지막 명령대로 계속 움직이면 안 되기 때문.

## Host → Pi (포트 5005) 로 오는 JSON

```json
{
  "cmd": "go",
  "status": "APPROACH_PIECE",
  "robot_x": 0.912,
  "robot_y": 0.543,
  "robot_yaw_deg": 87.3,
  "target_label": null,
  "fresh": true,
  "t": 1735142400.123
}
```

| 필드 | 타입 | 의미 |
|---|---|---|
| `cmd` | string | `"go"` \| `"stop"` \| `"yaw+"` \| `"yaw-"` |
| `status` | string | `"SEARCH_TARGET"` \| `"APPROACH_PIECE"` \| `"GRASP"` \| `"CARRY_TO_DEST"` \| `"FACE_BOX"` \| `"PLACE"` |
| `robot_x`, `robot_y` | number (m) | 참고용 — 실제 판단엔 안 써도 됨(Host가 이미 다 계산함) |
| `robot_yaw_deg` | number (도) | 참고용 |
| `target_label` | string \| null | `status=="GRASP"` 일 때만 — 집을 기물 이름(`queen`/`knight`/`rook`/`star`/`soccer`/`box`) |
| `fresh` | bool | 항상 `true` |
| `t` | number | Host 쪽 시각(Pi 시각과 안 맞을 수 있음 — 워치독은 **Pi가 패킷을 받은 시각** 기준으로 잴 것, 이 값을 그대로 신뢰하지 말 것) |

### 동작 매핑

| `cmd` | 차량이 할 일 |
|---|---|
| `"go"` | 지금 보고 있는 방향 그대로 전진 |
| `"stop"` | 정지 (GRASP/PLACE 중에도 항상 이 값으로 옴) |
| `"yaw+"` | 반시계 방향으로 제자리 회전 시작 — `"stop"` 올 때까지 계속 |
| `"yaw-"` | 시계 방향으로 제자리 회전 시작 — `"stop"` 올 때까지 계속 |

`status` 가 `"GRASP"` 이면 `cmd` 는 항상 `"stop"` 이고, 그 상태에서
SmolVLA 파지를 실행하면 된다. `"PLACE"` 도 마찬가지(내려놓기).

## Pi → Host (포트 5006) 로 보낼 JSON

```json
{"status": "GRASP_DONE", "t": 1735142401.5}
```

| 필드 | 타입 | 의미 |
|---|---|---|
| `status` | string | `"IDLE"` \| `"BUSY"` \| `"GRASP_DONE"` \| `"PLACE_DONE"` \| `"FAILED"` |
| `t` | number | Pi 쪽 시각 |

- 파지 시작하면 `"BUSY"`, 끝나면 `"GRASP_DONE"` 한 번 보내면 된다(Host는
  이걸 받는 즉시 다음 단계로 넘어감).
- 내려놓기도 동일하게 `"PLACE_DONE"`.
- 실패하면 `"FAILED"` — 지금 Host 쪽은 이 값을 아직 특별 처리하진 않지만
  (그냥 그 상태에서 계속 대기) 로그로는 남으니 일단 보내두면 좋다.

**Host의 IP/포트**: Host가 UDP를 보낼 때 발신 IP가 곧 Host의 IP이니,
`recvfrom()` 으로 받은 주소를 그대로 상태 응답 대상으로 써도 되고, 고정
IP를 안다면 그걸로 고정해도 된다.

## 참고용 스켈레톤

같이 첨부한 `pi_udp_bridge.py` 가 최소 동작하는 뼈대다 — `_handle_cmd()`
/ `_do_grasp()` / `_do_place()` 안에 실제 모터 제어·SmolVLA 호출 코드만
채우면 된다. ROS2 노드로 통합하려면, `_handle_cmd()` 안에서 UDP 대신
ROS2 토픽으로 재발행하는 방식으로 바꿔도 되고, 이 스크립트 자체를 ROS2
노드로 감싸도 된다 — 어느 쪽이든 Host 쪽은 이 UDP 인터페이스만 지키면
되니 자유롭게 골라도 된다.

## 실측해서 Host 쪽과 맞출 값

- **워치독 타임아웃**: 지금 Host 쪽 전송 주기는 카메라 1대 기준 실측
  8~10Hz(0.1~0.12초 간격)였다. 카메라 2대 붙이면 더 느려질 수 있어서,
  실제로 몇 Hz 나오는지 같이 재보고 워치독 값을 정할 것 — 너무 짧게 잡으면
  정상 상황에서도 자꾸 오탐으로 멈춘다.
