# Port Signature Freeze — Tier-1

> **상태: 초안 (draft).** 이슈 [#97](https://github.com/grippers-intel/grippers/issues/97) · 마일스톤 M1 · 목표 완료일 **8/18**
> **freeze 성립 조건: 3인 합의 서명(§7) + 이 문서와 `domain/ports/*.py` 가 동일**
> **단일 소스** — 포트 시그니처는 이 문서와 코드가 짝입니다. `hld.md` §4.5 · `class_diagram.md` §2 는
> 이 문서를 참조하고 중복 정의하지 않습니다.

포트 시그니처가 정해져야 5명이 Fake 를 띄워놓고 병렬로 개발할 수 있습니다 ([#98](https://github.com/grippers-intel/grippers/issues/98) · [#99](https://github.com/grippers-intel/grippers/issues/99)).
**이 문서가 M1 Gate B 의 첫 번째 도미노입니다.**

- [1. freeze 범위 — 무엇을 얼리고 무엇을 얼리지 않는가](#1-freeze-범위--무엇을-얼리고-무엇을-얼리지-않는가)
- [2. 확정 시그니처 (Tier-1)](#2-확정-시그니처-tier-1)
- [3. 이 freeze 가 해소하는 결정](#3-이-freeze-가-해소하는-결정)
- [4. 합의가 필요한 미결 — 서명 전 처리](#4-합의가-필요한-미결--서명-전-처리)
- [5. 머지 순서와 CI 빨간불 구간](#5-머지-순서와-ci-빨간불-구간)
- [6. 변경 절차](#6-변경-절차)
- [7. 승인](#7-승인)

---

## 1. freeze 범위 — 무엇을 얼리고 무엇을 얼리지 않는가

한 번도 실행해본 적 없는 인터페이스를 전부 묶으면 잘못된 설계에 갇히거나, freeze 를 깨서
제도 자체가 무력해집니다. **종이에서 정확히 정할 수 있는 것만 확정합니다** (`milestones.md` §8/14 freeze).

| Tier | 대상 | 상태 | 변경 조건 |
|---|---|---|---|
| **Tier-1** | 포트 **메서드 이름 · 인자 개수 · 인자 이름 · 단위 접미사** · 반환 타입의 **종류**(값 객체 / bool / None 허용 여부) · enum **값** | **freeze** | PR + 3인 합의 |
| **Tier-2** | 임계값 · 재시도 상한 · 값 객체 **내부 필드의 수치 의미** · 동작 타이밍 | **provisional (M2 실측까지)** | 실측 결과로 자유 변경 |

Tier-2 로 남기는 것 (지금 정할 근거가 없는 값):

`LOAD_THRESHOLD` · `MAX_GRASP_RETRY` · `MAX_RESCAN` · `OPEN_MM` / `CLOSE_MM` ·
`confidence` 채택 임계 · `align_to_box()` yaw 허용 오차 · `margin` · φ 해 구간

---

## 2. 확정 시그니처 (Tier-1)

코드가 정본입니다 — [`domain/ports/`](../../domain/ports/) · [`domain/values.py`](../../domain/values.py).
아래는 리뷰용 요약입니다.

### 2.1 값 객체 · enum

| 타입 | 필드 | 비고 |
|---|---|---|
| `ObjectClass` (enum) | `TOY` · `CHESS` | 배정 상자와 1:1. 클래스 내부 형상은 구분하지 않음 |
| `BoxColor` (enum) | `BLACK` · `RED` · `BLUE` · `GREEN` | 4개 전부 투입 가능한 후보 (무작위 기준선 25%) |
| `MissionMode` (enum) | `TIDY` · `FETCH` | |
| `Pose2D` | `x_m` `y_m` `theta_rad` | |
| `Point3` | `x_m` `y_m` `z_m` | |
| `Detection` | `track_id` `cls` `pose_m` `dims_m` `yaw_rad` `confidence` | `scan_floor()` 원소 |
| `BoxObservation` | `color` `pose_m` `opening_mm` `long_axis_rad` | |
| `Clearance` | `front_m` `left_m` `right_m` `contact_risk` | |
| `MissionSpec` | `mode` `placement_rule` `raw_text` `target_cls=None` | |
| `MissionContext` | `spec` `done_ids` `held_ids` `grasp_attempts` + `complete()` `hold()` `retry()` `is_settled()` | **불변** — 전부 새 인스턴스 반환 |

전부 `frozen=True` dataclass 입니다. 단위는 **필드명에 박습니다** (`_m` · `_rad` · `_mm`).
`_mm` 은 길이에만 쓰고 각도에는 절대 쓰지 않습니다.

### 2.2 포트 4종

```python
class BaseDriver(ABC):
    def drive_to(self, target: Pose2D) -> bool: ...
    def align_to_box(self, box: BoxObservation) -> float: ...      # 남은 yaw 오차(rad)
    def stop(self) -> None: ...

class ArmDriver(ABC):
    def move_to_cartesian(self, xyz_m: Point3, down: bool = False) -> bool: ...
    def set_gripper(self, width_mm: float) -> None: ...            # 개구 폭 mm — 각도 아님
    def get_load(self) -> float: ...                               # 0.0~1.0
    def reorient(self, phi_rad: float) -> bool: ...
    def fold_to_cradle(self) -> bool: ...
    def hold_position(self) -> None: ...

class Perception(ABC):
    def scan_floor(self) -> list[Detection]: ...
    def find_box(self, color: BoxColor) -> BoxObservation | None: ...
    def measure_opening(self, box: BoxObservation) -> float: ...   # mm
    def monitor_clearance(self) -> Clearance: ...

class CommandInterpreter(ABC):
    def parse(self, text: str) -> MissionSpec | None: ...
    def confirm_phrase(self, spec: MissionSpec) -> str: ...
```

`Ports` 는 `base` · `arm` · `perception` · `interpreter` · `estop` 5개를 듭니다
(`interpreter` 추가는 마이그레이션 PR #7).

### 2.3 상태 → 포트 호출 대조

`state_machine.md` §3 의 계약이 위 시그니처로 전부 표현되는지 확인한 결과입니다.

| 상태 | 호출 | 확정 시그니처로 표현 가능 |
|---|---|---|
| `IDLE` | `interpreter.parse(text)` | ✅ |
| `SCAN` | `perception.scan_floor()` | ✅ |
| `SELECT` | — (순수 판단) | ✅ 포트 불필요 |
| `APPROACH` | `base.drive_to()` | ✅ |
| `GRASP` | `arm.move_to_cartesian()` · `set_gripper()` · `get_load()` | ✅ |
| `TRANSPORT` | `perception.find_box()` · `base.drive_to()` · `base.align_to_box()` · `arm.fold_to_cradle()` | ✅ |
| `POSE_PLAN` | `perception.measure_opening()` | ✅ |
| `INSERT` | `arm.reorient()` · `move_to_cartesian()` · `set_gripper()` | ✅ |
| `DELIVER` | `base.drive_to()` · `arm.fold_to_cradle()` | ✅ |
| `HANDOVER` | `arm.set_gripper()` · `get_load()` | ✅ |
| `REJECT` | `arm.move_to_cartesian()` · `set_gripper()` | ✅ |
| `DONE` | — | ✅ |
| `ESTOP` | `base.stop()` · `arm.hold_position()` | ✅ (`hold_position` 이 이래서 필요) |

---

## 3. 이 freeze 가 해소하는 결정

| # | 항목 | 결정 | 근거 |
|---|---|---|---|
| D-1 | `set_gripper` 단위 | **`width_mm` (개구 폭 mm)** | 단위 규약이 "개구 폭은 mm, 각도 아님". 서보 각도 변환은 `FeetechArm` 캘리브레이션 테이블 (미결 #4 실측 결과) |
| D-2 | `move_to_cartesian` 기준 프레임 | **`base_link`** | HLD 미결 #2. 팔 기준으로 두면 주행 결과가 팔 좌표에 섞임 |
| D-3 | 각도 단위 | **rad 통일.** deg 는 GUI 표시·문서만 | HLD 미결 #3 |
| D-4 | 원시값 전달 | `xyz_m` 는 `list[float]` → **`Point3`** | 코드 리뷰 기준 "원시값 직접 전달 금지" |
| D-5 | `move_to_cartesian` 의 `grip` 인자 | **삭제** (`set_gripper` 로 분리) | 인자 1개가 두 액추에이터를 건드리면 실패 원인이 섞임 |
| D-6 | 실패 표현 | **예외가 아니라 반환값** (`bool` · `None`) | 루프 FSM에서 실패는 정상 경로(`SCAN` 복귀 + 보류 등록). 예외로 올리면 State 밖에서 흐름이 갈림 |
| D-7 | 안전 기본값 | `monitor_clearance()` → `contact_risk=True`, `scan_floor()` → `[]`, `find_box()` → `None` | "모르면 멈춘다" |
| D-8 | 음성 | **포트 아님** | `voice_io` 가 명령 토픽에 텍스트를 발행할 뿐 — 도메인 diff 0줄 |
| D-9 | `parse()` 실패 | **`None` 반환** | STT 오인식이 확인 없이 실행되는 경로를 만들지 않음 (오실행률 목표 0%) |

### 삭제되는 시그니처 (이전 주제 잔재)

`detect_target()` · `measure_gap()` · `set_light_profile()` · `align_to_centerline()` ·
`set_gripper(deg)` — 조명 도메인 전환이 없어졌고, 대상이 1개에서 N개로 바뀌었습니다.

---

## 4. 합의가 필요한 미결 — 서명 전 처리

> **아래 6건은 초안 작성자(이승용) 판단으로 채워 넣은 값입니다. 서명 전에 확인해 주세요.**

| # | 항목 | 초안의 선택 | 대안 | 결정 필요 |
|---|---|---|---|---|
| Q-1 | `ObjectClass` 이름 | **`TOY` · `CHESS`** | `GABE` · `CHESS_PIECE` | README 는 8/14 에 `toy`/`chess` 로 확정했고 `class_diagram.md` §1 은 아직 `GABE`/`CHESS_PIECE` 입니다. **가베가 3D 프린트로 바뀌어 `GABE` 는 더 이상 물건을 가리키지 않으므로 `TOY` 채택을 제안합니다.** 확정 시 `class_diagram.md` 정정 필요 |
| Q-2 | **`Detection.pose_m` 의 프레임** | **`map`** | `base_link` | `SELECT` 는 "`base_link` 기준 최단거리"로 고르는데 `drive_to()` 는 `map` 을 받습니다. 어느 쪽이든 **변환이 한 군데서만** 일어나야 합니다. 값 객체에 프레임 ID가 없으므로 **문서로만 고정됩니다** |
| Q-3 | **`track_id` 의 안정성** | 어댑터가 **프레임 간 동일 물체에 동일 id 를 보장** | 매 스캔 새 id + 위치 기반 매칭을 도메인에서 수행 | **재진입 방지(`done_ids`/`held_ids`) 전체가 이 가정 위에 있습니다.** 보장할 수 없으면 무한 루프 방어선이 무너지므로, 보장 주체를 지금 정해야 합니다 |
| Q-4 | `measure_opening(box)` 인자 | `BoxObservation` 을 받음 | 인자 없음 (현재 정렬된 상자 가정) | 인자를 받으면 어느 상자를 쟀는지가 호출부에 남습니다 |
| Q-5 | `reorient()` 유지 여부 | **유지** (`POSE_PLAN` 보류 중이라 항상 `phi_rad=0.0`) | 삭제 후 재도입 시 추가 | 지금 지우면 Fake·Real 어댑터 4종을 나중에 다시 건드려야 합니다 |
| Q-6 | `placement_rule` 의 ROS2 표현 | 도메인은 `Mapping[ObjectClass, BoxColor]` | 메시지에서는 **병렬 배열 2개**로 평탄화 | msg 정의는 배포판 단일화([#96](https://github.com/grippers-intel/grippers/issues/96)) 이후 |

---

## 5. 머지 순서와 CI 빨간불 구간

**이 PR 단독으로는 CI 가 그린이 되지 않습니다.** 숨기지 않고 적습니다.

포트 ABC 가 바뀌면 기존 Fake 어댑터는 추상 메서드 미구현이 되어 **인스턴스 생성 시점에 실패**하고,
`Pose2D` 필드명 변경(`x` → `x_m`)으로 아래가 함께 깨집니다.

| 파일 | 깨지는 이유 | 담당 이슈 |
|---|---|---|
| `domain/task/states.py` | 이전 주제 FSM 전체 | 마이그레이션 #6 |
| `domain/adapters/fake/*.py` | 구 ABC 구현 | [#98](https://github.com/grippers-intel/grippers/issues/98) |
| `domain/adapters/real/*.py` | 구 ABC 구현 · `Pose2D` 필드명 | 마이그레이션 #9 |
| `tests/test_fake_perception.py` · `tests/test_mission_task.py` | 구 시그니처 전제 | 마이그레이션 #10 |

**권장 — #97 과 [#98](https://github.com/grippers-intel/grippers/issues/98) 을 하나의 머지 트레인으로 묶습니다.**
`class_diagram.md` §5 의 순서(1 → 2·3·4·5 → 8 → 6 → 7 → 9 → 10)를 지키되, **같은 날 안에** 8까지 넣습니다.
같은 날에 못 넣으면 구 테스트 2개에 `pytest.mark.skip(reason="#98 대기")` 를 달아 빨간불 구간을 명시적으로 표시하세요 —
**조용히 빨간 채로 두면 [#99](https://github.com/grippers-intel/grippers/issues/99)(CI Fake 전 파이프라인 통과)의 판정 기준이 사라집니다.**

⛔ `domain/ports/perception.py` 는 **[#96](https://github.com/grippers-intel/grippers/issues/96)(배포판 단일화) 이후**에 머지합니다.
`scan_floor()` 가 목록을 반환하므로 `Detection.msg` / `DetectionArray.msg` 가 필요하고,
Humble(3.10) ↔ Jazzy(3.12) 는 타입 해시가 달라 통신이 되지 않습니다.

---

## 6. 변경 절차

freeze 이후 Tier-1 을 바꾸려면:

1. 이 문서에 **변경 이력 행을 먼저 추가**하는 PR 을 엽니다 (코드보다 문서가 먼저)
2. **3인 합의** — 이승용 · 조현우 + 해당 포트 오너
3. 같은 PR 에서 Fake 어댑터와 계약 테스트를 함께 고칩니다

계약 테스트 [`tests/test_port_contract.py`](../../tests/test_port_contract.py) 가 메서드 이름과
인자 이름·순서를 검사합니다. **이 테스트를 고치는 diff 가 곧 freeze 를 깨는 diff 입니다** —
리뷰에서 그 파일이 변경 목록에 있으면 3인 합의를 확인하세요.

---

## 7. 승인

freeze 는 아래 3인 서명으로 성립합니다.

| 역할 | 이름 | 승인 | 일자 |
|---|---|---|---|
| 설계 총괄 | 이승용 (@sysy009) | ☐ | |
| 코드 수장 · 최종 판단 | 조현우 (@kica927) | ☐ | |
| 포트 오너 (Arm) | 임성혁 (@alex7663) | ☐ | |
| 포트 오너 (Perception) | 김동혁 (@Feroninn) · 김희수 (@Hease) | ☐ | |

---

## 8. 변경 이력

| 날짜 | 버전 | 변경 | PR | 승인 |
|---|---|---|---|---|
| 2026-08-17 | 0.1 | 초안 — 포트 4종 시그니처 · 값 객체 · 미결 6건 (#97) | | |

---

## 참고

| 문서 | 내용 |
|---|---|
| [`state_machine.md`](state_machine.md) | FSM 전이 단일 소스 — 상태별 포트 호출 계약 |
| [`class_diagram.md`](class_diagram.md) | 값 객체 · 포트 클래스 다이어그램 · 마이그레이션 PR 10건 |
| [`hld.md`](hld.md) | 인터페이스 명세 (§4.5 는 이 문서로 대체 예정) |
| [`milestones.md`](../ops/milestones.md) | Tier-1 / Tier-2 freeze 정책 |

---

[← Documentation](../README.md) · [design](README.md)
