# 그리퍼스 작업장 도면 (텍스트판)

> 원본: claude.ai 아티팩트 "그리퍼스 작업장 도면 (Copy)" (2026-09-05 생성분)을 Claude Code가 읽을 수 있게 옮긴 파일.
> 값 출처: `grippers-host-mac/host/aruco/config.py`, `host/mission_config.py`, `host/basket_target.py`, `host/mission.py`
> 코드와 다르면 **코드가 정답**이다. 이 문서는 참고용 스냅샷.
>
> **이 저장소(vla_robot)와의 차이** (2026-09-22 대조):
> - 호·투입 목표·부채꼴 값은 전부 일치한다 → `host/mission/basket_target.py`, `host/config/host.yaml`
> - `FACE_BOX 정렬 각도 90°` 는 **더 이상 쓰지 않는다**. 호 위 진입 지점마다 목표 중심을
>   향한 방위각으로 선다(이 문서 "NUDGE 판정 경계호" 절이 실제 구현이다).
> - GRASP 진입 거리: 여기 0.40 m / vla_robot 0.35 m (마커 기준으로 다시 잡았다)
> - 기물 회피 반경: 여기 0.08 m / vla_robot 0.14 m
> - HOME / DELIVER_HERE (0.900, 0.320) 개념이 vla_robot 에는 없다 — 작업이 끝나면 SEARCH 로 돌아간다
> - 카메라 위치·높이는 vla_robot 설정에 없다. 바닥 마커 4점으로 외부 파라미터를 매번 추정한다
> - ⚠️ 상자 치수 `0.210 × 0.350` 은 팀 배치도 Rev.II 의 "실물 0.29 × 0.35" 와 어긋난다 — 실측 필요
>
> 그림: `grippers_workspace_plan.png` (평면도) · `grippers_workspace_elevation.png` (단면도)

## 좌표 규약
- 원점: 가벽 **앞쪽 왼쪽 바닥 모서리** `(0, 0)`, 단위 m
- +x = 오른쪽, +y = 뒤쪽(상자·CAM B 쪽)
- 각도: `atan2(dy, dx)` 규약 — 0° = +x, 90° = +y(12시, 상자 방향), CCW 양수

## 평면도 (위에서 본 모습, 대략적 축척)

```
 y
1.8 ┌─────────────────────[CAM B x=0.9]─────────────────┐  ← 가벽 뒤
    │        ┌──────┐                  ┌──────┐          │
1.625│       │ TOY  │                  │CHESS │          │  상자 중심 y=1.625
    │        │0.45  │                  │1.35  │          │  (0.21×0.35m, 입구는 -y쪽)
1.45│        └─[▭]──┘                  └─[▭]──┘          │  ▭ = INSERT 목표영역
1.4 │M3 ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄ M4│  WORKSPACE_Y 상단
    │   ╎        ◇ dest(0.45,1.30)        ◇ dest(1.35,1.30)╎ 
1.3 │   ╎╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╎  DRIVE_AREA_Y 상단
    │   ╎  DRIVE_AREA  x 0.20–1.60, y 0.30–1.30          ╎
    │   ╎                                                ╎
0.4 │M1 ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄ M2│  WORKSPACE_Y 하단
0.32│   ╎                 ● HOME (0.90, 0.32)            ╎
0.3 │   ╎╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╶╎  DRIVE_AREA_Y 하단
0.0 └─────────────────────[CAM A x=0.9]─────────────────┘  ← 가벽 앞
    0  0.1 0.2           0.9                    1.6 1.7 1.8   x
```

- WORKSPACE: 로봇 마커(높이 0.27m)가 두 카메라 모두에 보이는 구역 = x 0–1.8, y 0.4–1.4
- DRIVE_AREA: 벽 여유(로봇 반경 0.20m)를 뺀, 미션 플래너가 주행 허용하는 구역 = x 0.2–1.6, y 0.3–1.3 (WORKSPACE와 다른 사각형)
- 카메라 수평화각 70.4°는 캘리브레이션 파일(`host/calib/*.npz`)이 없을 때의 근사치

## 실측값 전체 목록

### 작업장 전체
| 항목 | 값 | 소스 |
|---|---|---|
| 가벽 크기 (WALL / WORKSPACE_X) | 0.000 – 1.800 m (폭 1800mm), 1.8×1.8m 정사각 | aruco/config.py WORKSPACE_X |
| 작업구역 세로 (WORKSPACE_Y) | 0.400 – 1.400 m (1000mm) | aruco/config.py WORKSPACE_Y |
| 주행구역 가로 (DRIVE_AREA_X) | 0.200 – 1.600 m (1400mm) | mission_config.py DRIVE_AREA_X |
| 주행구역 세로 (DRIVE_AREA_Y) | 0.300 – 1.300 m (1000mm) | mission_config.py DRIVE_AREA_Y |
| 벽 여유(로봇 반경, 팔 포함) | 0.20 m | mission_config.py ROBOT_RADIUS_WALL_M |
| 기물 회피 반경 | 0.08 m | mission_config.py ROBOT_RADIUS_PIECE_M |

### 바닥 기준마커 (ArUco DICT_4X4_50)
| 항목 | 값 | 소스 |
|---|---|---|
| M1 — 앞쪽 왼편 | (0.100, 0.400) | aruco/config.py FLOOR_MARKER_WORLD[1] |
| M2 — 앞쪽 오른편 | (1.700, 0.400) | FLOOR_MARKER_WORLD[2] |
| M3 — 뒤쪽 왼편 | (0.100, 1.400) | FLOOR_MARKER_WORLD[3] |
| M4 — 뒤쪽 오른편 | (1.700, 1.400) | FLOOR_MARKER_WORLD[4] |
| 바닥마커 크기 | 0.120 m (검은 사각형 바깥변) | FLOOR_MARKER_SIZE |
| 로봇 상판마커 크기 / 고도 | 0.080 m / 0.270 m | ROBOT_MARKER_SIZE / ROBOT_MARKER_HEIGHT |

### 목적지 상자
| 항목 | 값 | 소스 |
|---|---|---|
| 상자 크기 (폭×길이×높이) | 0.210 × 0.350 × 0.220 m | aruco/config.py BOX_W, BOX_L, BOX_H |
| toy 상자 중심 / yaw | (0.450, 1.625) / 180° | BOXES["toy"] |
| chess 상자 중심 / yaw | (1.350, 1.625) / 180° (2026-09-05 정정: 1.45→1.35) | BOXES["chess"] |
| 상자 입구 방향 | −y 쪽(작업구역 방향)으로 개방 | yaw=180 주석 |
| 상자 앞면(입구) y | 1.450 (= 1.625 − 0.175) | 계산값 |

### 기준점 / 카메라
| 항목 | 값 | 소스 |
|---|---|---|
| HOME / DELIVER_HERE | (0.900, 0.320) | mission_config.py DEFAULT_HOME_XY, DELIVER_HERE_XY |
| 카메라 좌우 위치 (양쪽 공통) | x = 0.900 m | aruco/config.py §7 주석 |
| CAM A / CAM B 위치 | 앞벽 y=0 (+y를 봄) / 뒷벽 y=1.8 (−y를 봄) | 도면 |
| 카메라 높이 | 1.300 m, 후퇴 0m(가벽 바로 앞) | §7 주석 |
| 카메라 하향 각도 안전범위 | 35° – 59° (실측 각도 아님, selftest.py 시뮬레이션 기준) | §7 주석 |
| 수평 화각(HFOV, 캘리브레이션 없을 때 근사) | 70.4° | HFOV_DEG |

### 미션 판정 거리 (알고리즘 상수, 참고)
| 항목 | 값 | 소스 |
|---|---|---|
| GRASP 진입 거리 | ≤ 0.40 m | GRASP_TRIGGER_DIST_M |
| PLACE 진입 거리 | ≤ 0.35 m | PLACE_TRIGGER_DIST_M |
| HOME 도착 판정 | ≤ 0.10 m | HOME_ARRIVE_TOL_M |
| FACE_BOX 정렬 각도 | 90° (map 기준 +y = 12시) | BOX_FACE_YAW_DEG |

### INSERT 목표영역 · NUDGE 목적지 (2026-09-04~05 신설)
| 항목 | 값 | 소스 |
|---|---|---|
| 목표영역 가로 절반폭 | ± 0.03 m (입구 중심 기준) | basket_target.py TARGET_HALF_WIDTH_M |
| 목표영역 안쪽 깊이 | 0.03 m | basket_target.py TARGET_INSET_DEPTH_M |
| Host 1차 승인 반경 | 0.15 m (목표영역에서 이 거리 이내) | basket_target.py MAX_APPROACH_DIST_M |
| 승인 조건(방향) | 지향 오차 ≤ ±50° (로봇 헤딩 의존) | basket_target.py MAX_FACING_ERROR_DEG |
| toy 목표영역 | x:[0.420, 0.480] y:[1.450, 1.480], 중심 (0.450, 1.465) | target_rect("toy") |
| chess 목표영역 | x:[1.320, 1.380] y:[1.450, 1.480], 중심 (1.350, 1.465) | target_rect("chess") |
| 상자 접근 여유(중심→목적지 오프셋) | 0.325 m (BOX_L/2 0.175 + 0.15) | mission.py _box_front_xy(), BOX_APPROACH_MARGIN_M |
| toy 목적지 좌측 보정 | 0.00 m (2026-09-03 실기로 7cm 도입 → 2026-09-05 사용자 지시로 되돌림) | TOY_DEST_X_SHIFT_LEFT_M |
| chess 목적지 안쪽 보정 | +0.00 m (현재 미적용) | CHESS_APPROACH_EXTRA_DEPTH_M |
| toy CARRY_TO_DEST 목적지(dest_xy) | (0.450, 1.300) | _box_front_xy("toy") |
| chess CARRY_TO_DEST 목적지(dest_xy) | (1.350, 1.300) | _box_front_xy("chess") |

## NUDGE 판정 경계호 (코드 반영 2026-09-05, 실기 미검증)
- 함수: `basket_target.check_approach_sector()`
- 점(dest_xy) 판정 대신, INSERT 목표영역 **중심**을 기준으로 반경 0.15m 원을 120°씩 3등분
- 접근 방향(남쪽, 중심각 −90°) 부채꼴 = −150° ~ −30°. 그 **바깥 호**가 NUDGE 판정 경계선
- 호 위 어디로 들어오든 "예비 INSERT 후보" → 그 자리에서 **목표중심을 향한 방위각**으로 제자리 정렬
  - 정렬각은 진입점마다 다름: 호 왼쪽 끝 30°, 중앙 90°, 오른쪽 끝 150° (중앙 진입일 때만 기존 고정 90°와 같음)
- 호의 주요 점 (계산값):

| 상자 | 왼쪽 끝 (−150°) | 중앙 (−90°) | 오른쪽 끝 (−30°) |
|---|---|---|---|
| toy (중심 0.450, 1.465) | (0.320, 1.390) | (0.450, 1.315) | (0.580, 1.390) |
| chess (중심 1.350, 1.465) | (1.220, 1.390) | (1.350, 1.315) | (1.480, 1.390) |

- 알려진 위험: ArUco 마커와 로봇 앞부분(그리퍼) 사이 물리 오프셋이 저장소에서 실측되지 않아, 사선 접근 시 과도하게 밀고 들어갈 가능성이 시뮬레이션에서 확인됨 → 실기에서 확인 필요

## 단면도 (y축 방향 높이 관계)
```
 z(m)
1.3  ◆CAM A (y=0, 하향 35–59°)                     CAM B (y=1.8)◆
 ...
0.27 ─ ─ ─ ─ ─ ─ ─ 로봇 마커 중심고도 0.270 ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
0.22                                          ┌────────┐ 상자 0.22
0.0  ════════════════════════════════════════╧════════╧═══════
     y=0 (앞)                               1.45    1.8 (뒤)
```
- 가벽 높이는 코드 상수에 없음(실측 안 됨)
- 앞뒤 0.4m 여유 = 상자 자리(길이 0.35 + 여유 0.05) 기준. 로봇 마커 고도 때문에 생기는 카메라 사각지대(약 0.15m, 근사)도 이 여유 안에 들어감

## 기계 판독용 상수 (YAML)
```yaml
units: m
origin: front-left floor corner
room: {x: [0.0, 1.8], y: [0.0, 1.8]}
workspace: {x: [0.0, 1.8], y: [0.4, 1.4]}
drive_area: {x: [0.2, 1.6], y: [0.3, 1.3]}
robot_radius_wall: 0.20
robot_radius_piece: 0.08
floor_markers:            # ArUco DICT_4X4_50, size 0.120
  1: [0.100, 0.400]
  2: [1.700, 0.400]
  3: [0.100, 1.400]
  4: [1.700, 1.400]
floor_marker_size: 0.120
robot_marker: {size: 0.080, height: 0.270}
box: {w: 0.210, l: 0.350, h: 0.220}
boxes:                    # [x, y, yaw_deg]; yaw 180 = opening faces -y
  toy:   [0.450, 1.625, 180]
  chess: [1.350, 1.625, 180]
home_xy: [0.900, 0.320]
deliver_here_xy: [0.900, 0.320]
cameras:
  A: {x: 0.900, y: 0.0, h: 1.300, looks: +y}
  B: {x: 0.900, y: 1.8, h: 1.300, looks: -y}
  hfov_deg_approx: 70.4
  tilt_safe_range_deg: [35, 59]
mission:
  grasp_trigger_dist: 0.40
  place_trigger_dist: 0.35
  home_arrive_tol: 0.10
  box_face_yaw_deg: 90
insert_target:
  half_width: 0.03
  inset_depth: 0.03
  max_approach_dist: 0.15
  max_facing_error_deg: 50
  toy:   {x: [0.420, 0.480], y: [1.450, 1.480], center: [0.450, 1.465]}
  chess: {x: [1.320, 1.380], y: [1.450, 1.480], center: [1.350, 1.465]}
box_approach_margin: 0.15
toy_dest_x_shift_left: 0.0
chess_approach_extra_depth: 0.0
dest_xy:
  toy:   [0.450, 1.300]
  chess: [1.350, 1.300]
nudge_sector:             # verified in code 2026-09-05, NOT hardware-verified
  radius: 0.15
  center_deg: -90
  half_width_deg: 60      # sector -150 .. -30
```
