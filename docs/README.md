# Grippers — 문서 색인

프로젝트 개요는 [최상위 README](../README.md) 를 먼저 보세요.

| 폴더 | 성격 | 내용 |
|---|---|---|
| 📐 [`design/`](design/) | **코드보다 앞서는 계약** — 변경은 PR + 3인 합의 | FSM · 클래스 · 시퀀스 · 인터페이스 명세 |
| 🧩 [`subsystems/`](subsystems/) | 담당 영역별 설계와 제약 | 물체 · 인식 · 콘솔 · AI |
| 🛠 [`ops/`](ops/) | 프로젝트를 굴리는 문서 | 일정 · 실행 · 실측 기록 |

> **파일명 규약** — snake_case(언더스코어).
> 새 문서는 위 세 폴더 중 하나에 넣고, 해당 폴더의 `README.md` 표에 한 줄 추가하세요.

---

## 어디부터 읽을까

| 상황 | 순서 |
|---|---|
| **처음 합류** | [README](../README.md) → [`state_machine.md`](design/state_machine.md) → [`class_diagram.md`](design/class_diagram.md) |
| **코드 작업 시작** | [`class_diagram.md`](design/class_diagram.md) §5 → [CONTRIBUTING](../CONTRIBUTING.md) → [`setup.md`](ops/setup.md) |
| 인식 담당 | [`perception.md`](subsystems/perception.md) → [`objects.md`](subsystems/objects.md) → [`ai_components.md`](subsystems/ai_components.md) |
| 콘솔·음성 담당 | [`console.md`](subsystems/console.md) → [`sequences.md`](design/sequences.md) §5 |
| 하드웨어·발주 | [`objects.md`](subsystems/objects.md) → [`perception.md`](subsystems/perception.md) (해상도 요구사항) |
| 일정 확인 | [`milestones.md`](ops/milestones.md) |
| 측정·시연 집계 | [`failure_definition.md`](ops/failure_definition.md) → [`measurements.md`](ops/measurements.md) |

## 지금 주의할 것

- **FSM 전이는 [`state_machine.md`](design/state_machine.md) 가 단일 소스** — 다른 문서에 중복 정의 금지.
  **2026-09-03 실제 코드(`domain/task/baseline_mission.py`) 기준으로 갱신 완료.**
- **[`hld.md`](design/hld.md) · [`error_budget.md`](design/error_budget.md) · [`class_diagram.md`](design/class_diagram.md) ·
  [`sequences.md`](design/sequences.md) · [`architecture.puml`](design/architecture.puml) 도 전부
  2026-09-04에 같은 실제 코드 기준으로 갱신 완료** — 더 이상 구 SCAN/SELECT 루프 설계가 아니다.
  `design/` 아래 유일하게 남은 구 설계 문서는 [`pose_planning.md`](subsystems/pose_planning.md)뿐이며,
  이건 ⏸ 보류 표시가 이미 붙어 있다.

---

[← README](../README.md)
