# 설계 다이어그램

시스템 구조와 동작을 정의하는 문서. **코드보다 앞서는 계약**이며 변경은 PR + 3인 합의.

| 문서 | 내용 |
|---|---|
| [`state_machine.md`](state_machine.md) | ⭐ **FSM 전이 단일 소스** — 루프 구조, 상태별 계약, 재진입 방지 |
| [`class_diagram.md`](class_diagram.md) | 값 객체 · 포트 · State · 노드 계층 + **as-is→to-be 마이그레이션 PR 10건** |
| [`sequences.md`](sequences.md) | TIDY 루프 · 파지 재시도 · FETCH · 음성 대화 |
| [`architecture.puml`](architecture.puml) | 같은 구조의 PlantUML 버전 |
| [`hld.md`](hld.md) | 인터페이스 명세 ⚠️ **갱신 대기** (M2, 8/20) |
| [`error_budget.md`](error_budget.md) | 오차 전파 분석 ⚠️ **갱신 대기** — 단안 구성 반영 필요 |

> FSM 전이 그래프는 `state_machine.md` 가 단일 소스입니다. 다른 문서에 중복 정의하지 말고 링크하세요.

---

[← 문서 색인](../README.md) · [← README](../../README.md)
