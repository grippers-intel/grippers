# baseline 미션 — 실측 TODO 표

`domain/task/baseline_mission.py`(사용자가 2026-08-25에 정리한 7단계 흐름)를
실기에 올리기 전에 채워야 할 수치들이다. 코드에서는 `baseline_constants.py`에
`None`으로 두었고, 그 값이 필요한 자리는 판정을 포기하고 Host에 보고만 한다 —
지어낸 숫자로 도는 것처럼 보이게 하지 않는다.

`baseline_constants.unresolved()`가 미해결 목록을 돌려주고,
`tests/test_baseline_mission.py`가 그 목록이 비지 않았다는 사실 자체를 못 박는다.
전부 채우면 그 테스트를 지운다.

---

## 0. 반복 실측이 필요한 예민한 값 (2026-08-26 정정)

### 19cm 정렬과 미세 전진 — 충돌이 아니라 설계다

앞서 이 항목을 "교시 자세와 어긋난다"고 적었는데 **오독이었다.** 사용자 설명:
팔은 바닥 교시 자세로 **열린 채** 내려와 있고, 그 상태에서 차체가 전진해
**물체를 벌어진 턱 사이로 밀어 넣는다.** 평행 턱의 벌어진 목이 좌우
자기정렬 효과까지 낸다.

그러니 19cm는 **정렬(핸드오프) 거리**이고, 턱이 닫히는 지점은 거기서 전진
거리만큼 앞이다. `GRASP_OBJECT_CENTER_FORWARD_MM`(190mm)은 **정적 자세
검증용 배치 거리**로, 물체를 놓고 팔 자세만 확인할 때 쓰는 값이다. 둘이
같을 이유가 없다.

| 상수 | 현재 | 성격 |
|---|---|---|
| `APPROACH_HANDOFF_FORWARD_MM` | 190 mm | 정렬 거리 — Host가 판정 |
| `GRASP_CREEP_FORWARD_MM` | 100 mm | **50 mm로 바뀔 수 있음** |
| `jaw_close_forward_mm()` | 90 mm | 위 둘의 차 — 실측 대상 |

⚠️ **두 값 모두 매우 예민해 여러 번 실측해야 한다**(사용자 명시). 전진이
짧으면 물체가 턱 안쪽까지 안 들어오고, 길면 손가락 판이 물체를 밀어
넘어뜨린다. 확정 전까지 이 값들을 근거로 다른 수치를 유도하지 말 것.

## 1. 줄자로 잴 것

| 상수 | 현재 | 무엇을 재나 | 왜 필요한가 |
|---|---|---|---|
| `MARKER_TO_CHASSIS_FRONT_M` | `None` | ArUco 마커 중심 → 차체 전면 | Host는 마커 위치를 보고, 파지는 차체 전면 기준이다. 이 값 없이는 19cm 판정을 Host가 못 한다. **Host `mission_config.py`는 "실측 0.15"라 적고 Pi 인수인계서는 "미실측"이라 적는다 — 상충한다.** |
| `LIDAR_TO_CHASSIS_FRONT_M` | `None` | 라이다 원점 → 차체 전면 | 라이다가 재는 거리는 라이다 기준인데 정지 거리는 차체 전면 기준 5cm다. 이 값이 없으면 `basket_stop_distance_m()`이 `None`을 내고 INSERT로 안 넘어간다. |
| `BASKET_RIM_HEIGHT_M` | `None` | 바구니 테두리 높이 | 라이다 평면(91mm)이 테두리보다 낮아야 정면이 잡힌다. **Pi는 0.115, Host `config.py`는 `BOX_H`=0.220 — 상충한다.** 0.115가 맞으면 여유가 24mm뿐이라 바닥 요철에 스쳐 지나갈 수 있다. |

세 항목 모두 이미 문서 간 불일치가 있는 값이다. 재고 나면 Pi와 Host 양쪽
문서를 같이 고쳐야 한다.

## 2. 데이터시트 + 실기로 확인할 것

| 상수 | 현재 | 확인 방법 | 왜 필요한가 |
|---|---|---|---|
| `LIDAR_MIN_RANGE_M` | `None` | 데이터시트 후 `ros2 topic echo /scan` | 최소 측정 거리보다 가까운 표면은 아예 안 잡힌다. RPLidar A1급은 대략 0.15m다. **다행히 라이다가 차체 중심 쪽에 있어, 차체 전면이 바구니에서 5cm일 때 라이다는 약 20cm를 읽는다 — 하한 위다.** 다만 `LIDAR_TO_CHASSIS_FRONT_M`을 재기 전엔 확정할 수 없다. |
| `AVOID_LATERAL_STEP_M` | `None` | 실기 조정 | 미세 회피에서 옆으로 비키는 거리. 메카넘휠 옆걸음이다. 너무 작으면 같은 물체에 계속 걸리고, 너무 크면 다른 물체로 들어간다. |

## 3. 재실측이 필요한 것 (값은 있으나 낡음)

| 상수 | 현재 | 문제 |
|---|---|---|
| `LOAD_THRESHOLD` | `0.04` | 근거 실측이 2026-08-18(n=25, 빈 최대 0.031)인데, 08-25 재실측에서 자세별 빈 부하가 0.0235~0.0430으로 흔들렸다. CARRY_IDLE 검사에서 queen이 0.0508로 임계 위 4.6양자뿐이다. |
| `PROXIMITY_STOP_DISTANCE_M` | `0.25` | 잠정값. CPU YOLO 추론 지연을 재서 (지연 × 주행속도)만큼 여유를 더해야 한다. 0.06 m/s에서 1초 지연이면 6cm다. Hailo가 복구되면 다시 재야 한다. |

## 4. 배선이 필요한 것 (수치 아님)

| 항목 | 어디 | 내용 |
|---|---|---|
| `base.creep_forward(m)` | `Ros2MecanumBase` | 미세 전진. 지금은 `FakeBaselineBase`에만 있다. |
| `base.creep_lateral(m)` | `Ros2MecanumBase` | 미세 회피 옆걸음. 위와 같다. |
| `HostLink` 실구현 | 신규 | UDP 송수신. `VEHICLE_LINK_PROTOCOL.md`의 규격 오류 3건을 먼저 고쳐야 한다. |
| `Lidar.basket_face()` 실구현 | 신규 | `/scan` 구독 → `grippers_base/basket_lidar_align.py` 호출 → `BasketFace` 반환. 수학은 이미 있고 테스트도 됐다. |
| `monitor_clearance` 실구현 | `perception_node` | 지금은 항상 `contact_risk=True`(정지)를 반환하는 스텁이라 **baseline이 APPROACH에서 한 발도 못 나간다.** `proximity_gate.py`를 여기에 물리면 된다. |
| Host 쪽 제외 목록 | Host | 파지 재시도 예산을 다 쓴 물체를 Host가 "가장 가까운 체스말"로 또 고르지 않게 해야 한다. Pi의 `hold()`는 Pi 안에서만 유효하다. |

---

## 확정된 값 (참고)

재지 않아도 되는 것들이다.

| 상수 | 값 | 출처 |
|---|---|---|
| `LIDAR_HEIGHT_M` | 0.091 m | 사용자 실측 2026-08-25 |
| `GRASP_OBJECT_CENTER_FORWARD_MM` | 190 mm | 실측 2026-08-20 |
| `BASKET_DROP_REACH_FORWARD_MM` | 200 mm | 실측 2026-08-20 |
| `BASKET_APPROACH_STANDOFF_M` | 0.05 m | 사용자 지시 |
| `MAX_AVOID_STEPS` | 3 | 사용자 지시 |
| `MAX_GRASP_RETRY` | 3 | 사용자 지시 |
