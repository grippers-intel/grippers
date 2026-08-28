# Host PC ↔ 차량(Pi) 통신 규격

Host PC(탑뷰 카메라 2대 + ArUco + geti, `run_mission.py`)가 계산한 주행/파지
명령을 같은 와이파이로 연결된 차량(Raspberry Pi + Hailo, ROS2)에 실시간으로
넘기기 위한 규격이다. Host PC 코드의 `vehicle_link.VehicleLink` 인터페이스가
이 규격을 그대로 구현할 자리로 이미 준비되어 있다.

## 설계 원칙 — 차량은 각도 계산을 전혀 안 해도 된다

명령은 딱 4가지 동작(`go`/`stop`/`yaw+`/`yaw-`)뿐이다. Host가 ArUco로 매
사이클 로봇의 정확한 위치·방향을 알고 있으므로, "지금 어느 쪽으로 얼마나
돌아야 하는지"까지 Host가 다 계산해서 방향만 정해서 보낸다. 차량은:

- `go` 오면 지금 보고 있는 방향으로 그냥 전진
- `stop` 오면 정지
- `yaw+` 오면 그 방향(반시계)으로 제자리 회전 시작, `stop` 이 올 때까지 계속
- `yaw-` 오면 반대 방향(시계)으로 제자리 회전 시작, `stop` 이 올 때까지 계속

이 4개만 구현하면 되고, 목표 좌표나 목표 각도를 차량이 직접 계산할 필요가
없다.

## 왜 UDP + JSON인가

- **UDP**: 이 프로젝트는 "매 사이클 지금 아는 최선의 명령만 보내고, 오래된
  건 버려도 된다"는 철학으로 설계되어 있다. TCP는 유실된 패킷을 재전송하려
  드는데, 그러면 오래된 명령을 억지로 다시 보내는 꼴이 되어 이 철학과 맞지
  않는다. UDP는 패킷 하나가 빠져도 다음 사이클 새 패킷이 바로 오므로
  문제없다.
- **JSON**: 사람이 읽기 쉽고, 수신 측이 Python이든 C++이든 상관없이 파싱하기
  쉽다.
- Host(Windows)에 ROS2를 설치할 필요가 없다 — Pi 쪽에서 UDP로 받아 필요하면
  ROS2 토픽으로 재발행하는 작은 브릿지 노드 하나만 있으면 된다.

## 통신 구조

```
Host PC (Windows)                          Pi (차량, ROS2)
─────────────────                          ──────────────
run_mission.py                             udp_bridge_node
  │ 매 사이클 명령 계산                       │
  │──── UDP :5005 (명령, JSON) ──────────▶ │ go/stop/yaw+/yaw- 그대로 실행
  │                                         │ (또는 ROS2 토픽으로 재발행)
  │◀─── UDP :5006 (상태, JSON) ──────────── │ GRASP_DONE/PLACE_DONE 보고
  poll_status() 가 이걸 읽음
```

- 전송 주기: 대략 8~10Hz (메인 루프 한 바퀴에 하나씩, 실측 기준)
- 두 방향 모두 같은 서브넷 안이면 사설 IP로 바로 통신 가능

## 패킷 형식

### Host → Pi (포트 5005) — 명령

매 사이클 하나씩 보낸다.

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
| `cmd` | string | `"go"` \| `"stop"` \| `"yaw+"` \| `"yaw-"` — 이번 순간 바퀴가 할 일 |
| `status` | string | 지금 미션 단계. `"SEARCH_TARGET"` \| `"APPROACH_PIECE"` \| `"GRASP"` \| `"CARRY_TO_DEST"` \| `"FACE_BOX"` \| `"PLACE"` |
| `robot_x`, `robot_y` | number (m) | 지금 로봇의 map 좌표 (ArUco로 측정) |
| `robot_yaw_deg` | number (도) | 지금 로봇 방위각. +x축 기준 반시계, 범위 -180~180 |
| `target_label` | string \| null | **status가 "GRASP"일 때만** — 집을 기물 라벨 (`queen`/`knight`/`rook`/`star`/`soccer`/`box`) |
| `fresh` | bool | 항상 `true` — 매 전송마다 새로 보낸다는 워치독용 신호 |
| `t` | number | 전송 시각(unix epoch, 초 단위) — 워치독 판단용 |

**status 별로 실제로 해야 하는 일**
- `SEARCH_TARGET` / `APPROACH_PIECE` / `CARRY_TO_DEST` / `FACE_BOX`: `cmd`
  대로 움직이기만 하면 됨(go/stop/yaw+/yaw-)
- `GRASP`: `cmd` 는 항상 `"stop"`. 이 상태가 되면 차량이 자기 그리퍼캠 +
  RGB캠으로 SmolVLA 파지를 수행하고, 끝나면 아래 상태 채널로
  `"GRASP_DONE"` 보고
- `PLACE`: `cmd` 는 항상 `"stop"`. SmolVLA로 내려놓기 수행 후
  `"PLACE_DONE"` 보고

### Pi → Host (포트 5006) — 상태

상태가 바뀔 때마다(또는 일정 주기로) 보낸다.

```json
{"status": "GRASP_DONE", "t": 1735142401.5}
```

| 필드 | 타입 | 의미 |
|---|---|---|
| `status` | string | `"IDLE"` \| `"BUSY"` \| `"GRASP_DONE"` \| `"PLACE_DONE"` \| `"FAILED"` |
| `t` | number | 전송 시각(unix epoch, 초 단위) |

> ⚠️ 이 필드 이름도 `"status"`이지만 Host→Pi 쪽 `status`(미션 단계)와는
> 다른 값 집합이니 헷갈리지 않게 주의할 것 — 이쪽은 "지금 그 동작이 끝났는지
> 보고"용이다.

## 워치독 (필수)

`t` 기준으로 새 명령이 일정 시간 안에 안 오면 Pi가 자체적으로 정지하도록
구현할 것을 권장한다. Host PC가 꺼지거나 네트워크가 끊겨도 차량이 마지막
명령(`go` 등)으로 계속 움직이는 일이 없도록 하기 위함이다. 정확한 타임아웃
값은 실제 전송 주기를 같이 재본 뒤 정하는 게 안전하다(너무 짧으면 정상
상황에서도 오탐으로 계속 멈춘다).

> ⚠️ 이건 차량의 라이다 기반 반사 회피 레이어와는 완전히 별개다. 라이다
> 회피는 미확인 장애물이 나타났을 때 항상 최우선으로 작동해야 하며, Host
> PC 경유 시 지연이 생겨 안전 기능으로 못 쓴다 — 이 워치독은 어디까지나
> "Host 링크가 끊겼을 때 대비"용이다.

## 팀원에게 전달할 것

1. **IP/포트**: Pi의 고정 IP(또는 호스트네임), 명령 수신 포트(5005), 상태
   송신 포트(5006) — 같은 와이파이 서브넷이면 그대로 통신 가능
2. **위 JSON 스키마** (명령/상태 양쪽)
3. **워치독 구현 필수** — 타임아웃 값은 실측 후 같이 정하기

## 참고 — Host 쪽 소스

- `vehicle_link.py`: `MissionCommand` 데이터클래스, `VehicleLink` 추상
  인터페이스, `ConsoleVehicleLink`(지금 쓰는 콘솔 출력용 자리표시자)
- 실제 전송 구현체(예: `UdpVehicleLink`)를 만들어서 `VehicleLink`를
  상속하면, `run_mission.py`의 `ConsoleVehicleLink(...)` 자리만 바꾸면 되고
  `mission.py` 쪽 로직은 손댈 필요가 없다.
