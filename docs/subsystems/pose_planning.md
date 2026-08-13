# 자세 재조정 (⏸ 보류)


> **상태: 대상 클래스 미정으로 보류 (2026-08-13).**
> 긴 물체를 정리 대상에서 제외하면서 이 경로를 실행할 물체가 없어졌습니다.
> **`POSE_PLAN` · `REJECT` 상태와 `_solve_phi()` 는 설계·문서에 그대로 유지**하며,
> 미배정 상자(⚫ 또는 🔴)에 긴 물체를 배정하기로 하면 즉시 되살릴 수 있습니다.

긴 물체(0.50 m)를 좁은 상자 입구(0.40 m)에 넣으려면 세워야 합니다.

```
H_proj(φ) = L·|cos φ| + w·|sin φ|  ≤  W_open − margin
→ φ ≥ 0.83 rad (48°)   — W_open = 0.40 m 기준
```

| W_open | 최소 φ |
|---|---|
| 0.35 m | 55.5° |
| 0.40 m | 47.7° |
| 0.45 m | 41.8° |

- 요(yaw)는 `align_to_box()` 로 선행 정렬 → **1자유도 문제로 축소**
- 피치 회전이므로 **손목 단독 불가** — IK 전체 관여
- 자세 전환은 **반드시 정지 상태에서** — 주행 중 전환 시 전복
- 해 구간이 없으면 `REJECT` — "못 넣습니다"라고 판단하는 능력

> [!NOTE]
> **되살릴 때 부등호 방향에 주의하세요.** 최초 주제(암실)는 낮은 개구부 **밑을 지나느라 눕혔고**
> (`sin`/`cos` 위치가 반대, `φ ≲ 27°`), 이 설계는 좁은 입구에 **넣느라 세웁니다.**

## 재도입 절차

1. 미배정 상자(⚫ 또는 🔴)에 긴 물체 범주를 배정 → [`objects.md`](objects.md)
2. `ObjectClass` 에 항목 추가 → [`class_diagram.md`](../design/class_diagram.md)
3. `PosePlanState._solve_phi()` 구현 (현재 스텁)
4. `RejectState` 경로 테스트 추가
5. 상자 입구 실측 → `W_open` 확정

FSM 상태(`POSE_PLAN` · `REJECT`)와 시퀀스는 이미 문서화되어 있으므로 설계 작업은 불필요합니다.
→ [`state_machine.md`](../design/state_machine.md) · [`sequences.md`](../design/sequences.md) §3


---

[← README](../../README.md) · [Documentation](../README.md#-documentation)
