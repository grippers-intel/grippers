# Pi 미션 — 실측 TODO 표

`domain/task/baseline_mission.py`를 실기에 올리기 전에 채워야 할 수치들이다.
코드에서는 `baseline_constants.py`에 모아 두었고, `unresolved()`가 미해결
목록을 돌려준다.

> **2026-08-26 팀 확정으로 이 표가 크게 줄었다.** 좌표계 상수가 전부 사라졌기
> 때문이다 — 물체 좌표, 차량 좌표와 방향, 경로 계산, 차량 제어는 Host가
> 소유한다. `MARKER_TO_CHASSIS_FRONT_M`, `LIDAR_TO_CHASSIS_FRONT_M`,
> `AVOID_LATERAL_STEP_M`은 더 이상 Pi의 값이 아니다.

---

## 남은 TODO — 없음 (2026-08-27 기준 `unresolved()`가 빈 dict를 돌려준다)

한때 TODO였던 두 값은 아래처럼 확정됐다. 갱신 이력은 `baseline_constants.py`
주석과 `grippers_docs/grippers_handover_20260827.md`에 있다.

| 상수 | 확정값 | 근거 |
|---|---|---|
| `GRASP_CREEP_FORWARD_MM` | **70.0 mm** (상한) | 2026-08-27 여섯 클래스를 200mm에 놓고 실측한 필요 전진량이 7~34mm — 그 여유를 넉넉히 잡았다. 실제 전진량은 매번 `관측 - 클래스별 턱선`으로 계산하고 이 값은 관측이 튀었을 때의 안전장치다 |
| `LOAD_THRESHOLD` = `EMPTY_LOAD_CEILING` | **12/256 (0.046875)** | 빈손 최대 11/256, 파지 최소 13/256(INSERT 시점 퀸 0.0508)의 중점. 여유가 양쪽 1양자뿐이라 부하 단독 판정은 하지 않는다 |
| `JAW_LINE_DEPTH_FORWARD_M` | 여섯 클래스 전부 실측 | rook 0.1911 / knight 0.2023 / queen 0.1969 / soccer 0.1934 / box 0.1820 / star 0.1912 (2026-08-27) |

남은 미확정은 상수가 아니라 **Host 쪽 구현**이다: Host가 GRASP 진입 시 물체
중심을 차체 전면 200mm에 조준하기로 했으므로, 그 구현이 끝나면
`GRASP_OBJECT_CENTER_FORWARD_MM`을 190 → 200으로 맞춘다.

### 미세 전진이 무엇인지 (오해하기 쉬운 자리)

팔은 바닥 교시 자세로 **열린 채** 내려와 있고, 그 상태에서 차체가 전진해
**물체를 벌어진 턱 사이로 밀어 넣는다.** 평행 턱의 벌어진 목이 좌우
자기정렬 효과까지 낸다. 그러니 이 전진은 "가까이 가는 것"이 아니라 "집어
넣는 것"이다.

정렬은 Host가 오버헤드로 판정하고 GRASP 명령을 보낼 때 이미 맞춰 놓는다
(2026-08-27 확정: 물체 중심을 차체 전면 200mm에 조준). Pi는 아레나 수준의
정렬을 다시 판정하지 않고, 자기 뎁스캠 실측으로 남은 오차만큼만 전진한다.

### 부하 임계값이 두 개인 이유

`LOAD_THRESHOLD`(쥐었다)와 `EMPTY_LOAD_CEILING`(비었다)는 지금 같은 값이지만
**묻는 질문이 다르다.** 재실측하면 갈라질 수 있으므로 이름을 나눠 두었다.

2026-08-26 실기 참고값 — 나이트·퀸 모두 파지 시 **0.0626**, 놓은 뒤
**0.0313~0.0352**.

---

## 실측으로 채워진 값들 (2026-08-26)

| 상수 | 값 | 근거 |
|---|---|---|
| `LIDAR_HEIGHT_M` | 0.140 | 실측. 이전 0.091은 오측이었다 |
| `LIDAR_TILT_DEG` | 11.3 | 실측. 정면 아래로 기울어져 있다 |
| `LIDAR_MIN_RANGE_M` | 0.020 | LDRobot LD19 (RPLidar A1이 아니다) |
| `BASKET_STOP_LIDAR_M` | 0.140 | 검증 창 [0.130, 0.139]의 위쪽 끝 |
| `BASKET_MIN_LIDAR_M` | 0.128 | 빔이 테두리를 넘는 절벽(0.125)에서 3mm 띄움 |
| `BASKET_STOP_TOLERANCE_M` | 0.015 | 데드밴드 때문에 한 버스트를 15mm보다 잘게 못 쪼갠다 |
| `BASKET_YAW_TOLERANCE_RAD` | 0.087 | 실측 +2.82도 성공에 여유를 둔 5.0도 |
| `BASKET_RIM_HEIGHT_M` | 0.115 | 실측 2026-08-20 |
| `AGREED_LINEAR_MPS` | 0.1 | 팀 합의 |
| `AGREED_ROTATION_RAD_S` | 0.25 | 팀 합의 |

상세 근거는 `grippers_docs/2026-08-26_바구니접근_INSERT_실기분석.md` 참고.

---

## 아직 실기로 안 돌려 본 것 (2026-08-28 갱신)

| 항목 | 상태 |
|---|---|
| `UdpHostLink` ↔ Host 실제 코드 | 🟡 맥 로컬 적합성 시험(`tools/host_link_conformance.py --translated`)만 통과. Host 저장소가 아직 2026-08-26 확정 이전 규격(`cmd`/`status`)이라 **그대로 붙이면 안 움직인다** — `grippers_docs/grippers_host_requests_20260827.md` 참고 |
| `Ros2Lidar` | ✅ 2026-08-27 `basket_approach_insert_test.py`로 여섯 클래스 INSERT 통합 검증에 사용됨 |
| `Ros2MecanumBase.creep_forward` | ✅ 같은 통합 검증에서 저속 접근·자동정지에 사용됨 |
| INSERT 좌우 오프셋 게이팅 (`28d4626`) | 🟡 코드만. 오프셋이 허용치를 넘는 상황을 실기로 만들어 ⛔ 분기가 실제로 도는지 확인 필요 |
| `Ros2Perception.identify_target` | 🔴 클래스를 하나씩 물어보는 방식 — 왕복 6회의 실제 지연 미측정 |
| 사선 진입 INSERT | 🔴 `BASKET_LATERAL_TOLERANCE_M`(0.070)이 유일한 계산값 — 사선 진입 1회로 실측화 필요 |
| FSM 전체 통주행 | 🔴 Host 없이는 못 돌린다. `use_fake_host:=true`로 FakeHostLink 경로만 확인 가능 |
