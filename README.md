<div align="center">

# 🤖 Grippers

**바닥에 흩어진 물건을 스스로 정리하고, 요청하면 가져다주는 모바일 매니퓰레이터**

⭐ 팀 프로젝트입니다 — 이슈와 제안 환영합니다 🙏

</div>

---

> [!IMPORTANT]
> **주제 변경 (8/12)** — *무인 멸균 암실 장물 반출* → *바닥 물건 정리 + 요청 배달*.
>
> ⚠️ **마이그레이션은 이 README가 그리던 설계(Pi 자율 `SCAN`/`SELECT` 루프)대로 끝나지 않았습니다.**
> 실제로 팀이 확정·구현한 것은 **Host(관제 콘솔)가 좌표·목표·경로를 전부 갖고 Pi는 지시를
> 실행 + 자기 센서 판단만 보고하는** 더 단순한 구조(`domain/task/baseline_mission.py`)입니다.
> 아래 [🧱 Architecture](#-architecture)와 [`state_machine.md`](docs/design/state_machine.md)가
> 실제 구현 기준이고, 이 README의 나머지(Mission 서술·아키텍처 다이어그램 이전 버전)는
> **당시 to-be 설계로서 남겨둔 것**입니다 — 물체·클래스·작업 공간 치수 같은 물리적 사실은
> 유효하지만, FSM·노드 구성은 실제와 다릅니다.
>
> 📅 **일정 조정 (8/13)** — M1 재시작으로 **범위 축소**. 파지 정책 학습을 Stretch로 강등하고
> freeze를 2단으로 분리했습니다. → [`milestones.md`](docs/ops/milestones.md)

## Table of Contents

- [🚀 About](#-about)
- [📖 Documentation](#-documentation)
- [🎯 Mission](#-mission)
- [✨ Key Features](#-key-features)
- [🧱 Architecture](#-architecture)
- [🔩 Hardware](#-hardware)
- [📁 Repository Structure](#-repository-structure)
- [⚡ Quick Start](#-quick-start)
- [📅 Milestones](#-milestones)
- [👥 Team](#-team)
- [🤝 Contributing](#-contributing)
- [📃 License](#-license)
- [📚 References](#-references)

> **파일명 규약** — `docs/` 하위 문서는 **snake_case(언더스코어)** 로 통일합니다.
>
> **용어** — **Grippers**는 시스템 전체(모바일 매니퓰레이터)를 가리킵니다.
> 팔 끝단 부품은 **엔드이펙터**로 표기해 구분합니다.

---

## 🚀 About

> **확정 문장**
>
> 사용자가 노트북 관제 콘솔에서 **음성 또는 텍스트로 명령**하면, 로봇이 바닥을 스스로 관측해 흩어진 물건을 찾고,
> **범주에 맞는 색 상자**에 넣어 정리한다. **형상이 달라도 같은 범주면 같은 상자로 간다.**
> 사용자가 특정 물건을 요청하면 그것을 찾아 **가져다준다.**

### 왜 이 문제인가

정리는 로봇 태스크로서 특이한 성질이 하나 있습니다 — **목표 상태가 목록으로 주어지지 않습니다.**

"A를 B로 옮겨라"는 목표가 명시된 문제이고, 경로만 풀면 됩니다. 반면 "정리해라"는 **무엇이 몇 개 있는지조차 모르는 상태에서 시작**하고,
로봇이 물체 하나를 옮길 때마다 바닥 상태가 바뀝니다. 그래서 **관측 → 판단 → 행동이 매 사이클 닫혀야** 하고,
이것이 이 프로젝트가 선형 시퀀스가 아니라 **루프 FSM**을 쓰는 이유입니다.

한 번 실행하고 끝나는 대본은 이 문제를 풀 수 없습니다. 그 차이가 시연에서 그대로 드러납니다.

### AI가 필요한 이유

세 가지가 겹칩니다.

| 조건 | 규칙 기반으로 안 되는 이유 |
|---|---|
| **물체 종류를 형상으로 분류** | 목적지가 종류에 따라 갈리므로 분류가 판단의 근거. 색으로 풀면 학습이 필요 없어짐 |
| **범주 내 형상이 다양함** | 정육면체·축구공형 다면체·오각별기둥이 전부 `toy`. **각진 것 · 둥근 것 · 별 모양이 같은 상자로** 갑니다 — 개별 형상을 외우는 방식으로는 처음 보는 형상에서 실패 |
| **자연어가 미션 파라미터를 변경** | "블록은 파란 상자에 넣어줘" → 배치 규칙 자체가 바뀜. 고정 대본과 구분되는 지점 |

> [!NOTE]
> **물체 색과 상자 색을 일부러 맞추지 않습니다.** 맞추면 색 세그멘테이션만으로 전체 문제가 풀려
> 학습 기반 인식의 명분이 사라집니다. **물체는 형상·종류로 분류(학습), 상자는 색 랜드마크로 탐색(LAB 세그멘테이션 · 견고성 우선)** —
> 두 축을 의도적으로 분리했습니다.

### 정직한 서술 — end-to-end VLA가 아닙니다

이 프로젝트는 **Vision-Language-Action을 모듈형으로 분해한 파이프라인**입니다.

| | 본 프로젝트 | end-to-end VLA |
|---|---|---|
| 모듈 간 전달 | **심볼** (`toy`, `BLUE`) | 학습된 특징 벡터 |
| 행동 결정 | 상태 머신 + 기하 계획 | 모델 가중치 |
| 학습 대상 | 검출기 · 파지 정책 | 정책 전체 |

발표에서 "VLA 모델을 구현했다"고 말하지 않습니다. 5주 안에 정책 학습 데이터를 모을 수 없었고,
모듈형이 **부분 검증과 병목 측정에 유리해 의도적으로 분해**했습니다. 모듈 명칭도 `VLA-V/L/A` 에서
**`perception` / `language` / `action`** 으로 바꿔, 이름이 실제보다 큰 주장을 하지 않게 했습니다.

---

## 📖 Documentation

README는 개요만 담고, 상세는 아래 문서로 나눠져 있습니다.

| 문서 | 내용 |
|---|---|
| [`objects.md`](docs/subsystems/objects.md) | 물체 구성 · 클래스 체계 · 3D 프린팅 체스말 · 시연 구성 |
| [`perception.md`](docs/subsystems/perception.md) | 인식 구성 — **해상도 요구사항 · 호모그래피 · 검정 상자 · 가림** |
| [`console.md`](docs/subsystems/console.md) | 노트북 관제 콘솔 — GUI · 음성 · **네트워크 리스크** |
| [`ai_components.md`](docs/subsystems/ai_components.md) | 학습 범위 · 데이터 · 가속기 선택 근거 · HEF 파이프라인 |
| [`setup.md`](docs/ops/setup.md) | 설치 · 실행 · 테스트 · 트러블슈팅 |
| [`milestones.md`](docs/ops/milestones.md) | 일정 · 미결 사항 · 리스크 · 측정 결과 |
| [`failure_definition.md`](docs/ops/failure_definition.md) | **무엇을 실패로 셀 것인가** — 지표 계수 규칙 |
| [`pose_planning.md`](docs/subsystems/pose_planning.md) | ⏸ 보류된 자세 재조정 설계 (재도입 절차 포함) |
| **설계 다이어그램** | |
| [`state_machine.md`](docs/design/state_machine.md) | **FSM 전이 단일 소스 · ✅ 실제 코드(`baseline_mission.py`) 기준 갱신됨(9/3)** |
| [`class_diagram.md`](docs/design/class_diagram.md) | 값 객체 · 포트 · State · 노드 계층 · 죽은 코드 목록 · ✅ 갱신됨(9/4) |
| [`sequences.md`](docs/design/sequences.md) | 시퀀스 다이어그램 · ✅ 갱신됨(9/4) |
| [`architecture.puml`](docs/design/architecture.puml) | PlantUML 버전 · ✅ 갱신됨(9/4) |
| [`hld.md`](docs/design/hld.md) | High Level Design · ✅ 갱신됨(9/4) |
| [`error_budget.md`](docs/design/error_budget.md) | 판정 문턱값 실측 · ✅ 갱신됨(9/4) |
| [`workspace_layout.html`](docs/design/workspace_layout.html) | **작업 공간 배치도 (Rev.I · 8/20 팀 확정)** — 평면도 · 지점별 기하 · 커버리지 · 배치 규칙 |
| **Host(관제 콘솔)** | |
| [`grippers-host-mac`](https://github.com/kica927/grippers-host-mac) | Host 실행 코드(좌표·경로·미션 진행) — macOS 포팅, 설치·실행·"테스트 준비" 절차는 그쪽 README |

각 문서의 최신/구 설계 여부 요약은 [`docs/README.md`](docs/README.md#지금-주의할-것) 참고.
기여 방법은 [CONTRIBUTING](CONTRIBUTING.md) 참고.

---

## 🎯 Mission

> ⚠️ **FETCH 모드는 빠졌습니다.** 2026-08-23 확정 미션 명세서에 **TIDY(규칙 기반 정리)만
> 남았습니다** — "체스말 가져와" 같은 사람 지시 기반 FETCH는 이전 설계였고 지금은 구현하지
> 않습니다(`grippers_language/language_node.py` 주석 참고). 각 물체는 라벨에 따라 고정된
> 상자로 갑니다(`toy`류 → 초록 상자, `chess`류 → 파랑 상자 — Host의 `PIECE_DEST_BOX`).

### 작업 공간

**울타리 1.80 × 1.80 m 정사각형 · 탑뷰 웹캠 2대 대향 (8/20 팀 확정).**

```
  웹캠 1                  1.80 m                  웹캠 2
  H 1.65  ┌──▶  ┌───────────────────┐  ◀──┐  H 1.65
  후퇴 .95│     │ 🟢               🔵 │     │후퇴 .95
  피치 44°└──   │                   │   ──┘  피치 44°
                │    ▪   ⬡    ■     │ 1.80 m
                │         ♞         │
                │  🤖 IDLE          │
                └───────────────────┘
       바닥 소요 3.70 × 1.80 m · 2대 이중 관측 76%

  🟢🔵 상자   0.29 × 0.35 m · 높이 0.22 m · 먼 쪽 양쪽 모서리 (실물 보유)
  ■●★ toy     정육면체 · 축구공형 다면체 · 오각별기둥 (폭 40 mm)
  ♞   체스말   나이트 · 퀸 · 룩 — 3D 프린팅
              물체 최소 이격 0.30 m · 상자 앞 정렬 통로 0.50 m
```

> ⚠️ **아래 표는 검토 당시 C270 실측 기준입니다.** 실제 사용 중인 카메라는 이후 **C920**으로
> 바뀌었습니다([🔩 Hardware](#-hardware) 참고) — 초점거리(`f`)가 달라지므로 mm/px·고도각 등
> 픽셀 단위 수치는 **C920 기준으로 재실측 전까지는 그대로 믿지 마세요.** 울타리·상자·물체
> 치수 같은 물리적 값은 카메라와 무관하므로 유효합니다.

| 항목 | 확정값 |
|---|---|
| 울타리 | **1.80 × 1.80 m** · 높이 0.25 m 권장 |
| 탑뷰 웹캠 | ~~C270 ×2 대향 · 1280×720 · 실측 `f = 1410 px`~~ → **C920으로 교체, 재실측 필요** |
| 설치 | 높이 **1.65 m** · 후퇴 **0.95 m** · 피치 44.1° (C270 기준 산출값) |
| 고도각 | 29.7 ~ 51.6° (C270 기준 산출값) |
| 최악점 | **1.87 mm/px** — **40 mm 물체가 21.4 px** (검출 하한 20 px) · 45 mm 면 24.1 px (C270 기준 산출값) |
| 물체 폭 | **40 mm** — 검출 하한 37.4 mm 와 그리퍼 설계 상한 45 mm 사이 |
| 실효 커버 시작 | 0.21 m (울타리 0.25 기준) |

> [!NOTE]
> **"작업 공간 네 모서리가 한 화면에" 는 이 기하에서 성립하지 않습니다** — 가로 7% 초과입니다.
> 2대 대향이라 근거리 모서리는 맞은편 카메라가 담당하고, 합집합으로 전 구역이 덮이므로 운용에는 문제가 없습니다.
> **세팅 확인 절차는 "네 모서리" 가 아니라 카메라별 담당 구역 기준으로 판정합니다.**
>
> ⏸ **네 모서리 ArUco 기지점 방식은 채택하지 않습니다 (8/20).** A2 호모그래피는 **카메라별로 기지점을 따로** 잡아 후퇴 0.95 m 를 유지합니다.
> 재도입하려면 **후퇴 1.20 m · 바닥 4.20 × 1.80 m** 가 필요합니다 — 설치 공간에 여유가 생기면 그때 판단합니다.

**도면 · 지점별 기하 · 커버리지 · 배치 규칙 전문 → [`workspace_layout.html`](docs/design/workspace_layout.html)**
*(GitHub 은 HTML 을 렌더하지 않습니다 — 내려받아 브라우저로 열어 보세요)*

### 클래스 체계

**수거 대상은 2 클래스 × 형상 3종으로 확정했습니다 (8/14).**

| 클래스 | 포함 개체 = 학습 대상 | 상자 | 상태 |
|---|---|---|---|
| `toy` | **정육면체 · 축구공형 다면체 · 오각별기둥** — 3D 프린팅 | 🟢 초록 | **개정 (8/20)** |
| `chess` | **나이트 · 퀸 · 룩** — 3D 프린팅, 스케일 확대 | 🔵 파랑 | **확정 (8/14)** |
| *(⏸ 보류)* | 방해 선택지 — ⚫ 검정 · 🔴 빨강 | — | **미사용 (8/20)** |

- `toy` **3형상을 개정했습니다 (8/20)** — 원기둥 · 육각기둥을 빼고 **축구공형 다면체 · 오각별기둥**을 넣었습니다. 시판 가베는 3×3 cm 로 검출 하한에 미달해 8/14 에 이미 3D 프린트로 전환했고, 이제 형상도 가베 교구를 따르지 않습니다.
- **원기둥을 뺀 것이 핵심입니다** — 세운 원기둥은 체스말(룩)과 측면 실루엣·종횡비가 겹쳐 문서상 유일한 🔴 혼동 쌍이었습니다. 그 쌍이 사라집니다.
- **축구공형은 유일하게 둥근 실루엣**이라, 모델이 *"`toy` = 각진 것"* 이라는 지름길을 학습하는 걸 막습니다.
- `chess` 는 **나이트 · 퀸 · 룩 3종을 우선** 학습합니다.
- 두 클래스 모두 **학습 대상과 시연 투입 물체가 동일**합니다 — 별도의 미학습 형상을 두지 않습니다.
- **상자는 🟢 초록 · 🔵 파랑 2개로 확정했습니다 (8/20).** ⚫ 검정 · 🔴 빨강 방해 선택지는 쓰지 않습니다.
- 🔴 **검정을 뺀 이유는 원거리 오검출입니다.** 먼 지점에서는 **🟢 초록 · 🔵 파랑이 모두 검정으로 잡힙니다.** 방해 선택지로 두려던 상자가 실제 목적지 두 개를 삼키는 구조라, 색 랜드마크 탐색의 전제가 무너집니다.
- ⏸ 클래스를 늘릴 때 상자 · `placement_rule` 을 함께 추가할 수 있도록 구조는 유지합니다. **⚫ 를 되살리려면 원거리 색 분리가 먼저 해결돼야 하고**, 밝은 테두리 · ArUco 병행 대책도 폐기가 아니라 ⏸ 보류입니다.

> **`toy` 안에 형상이 다른 개체가 여러 개 있는 것이 핵심 주장입니다.** 정육면체와 축구공형은
> 실루엣이 다른데 같은 상자로 갑니다 — 색 매칭이나 대본으로는 만들 수 없는 장면이며,
> 모델이 **범주를 배웠다는 증거**입니다.

상세 → [`objects.md`](docs/subsystems/objects.md)

### FSM — Host 지시 실행형

> ⚠️ 이 자리에 있던 `IDLE → SCAN → SELECT → ... → DONE` 루프 다이어그램은 **채택되지
> 않은 to-be 설계**였습니다. 실제 구현은 Pi가 자율로 바닥을 스캔·순회하지 않고,
> **Host가 목표·좌표·경로를 정해 지시하면 Pi는 그 지시를 실행 + 자기 센서 판단만
> 보고**하는 구조입니다(`IDLE/APPROACH/GRASP/CARRY/INSERT/DONE`, ESTOP 인터럽트).
> 전이 그래프·상태별 계약·왜 이렇게 갈렸는지는 **[`docs/design/state_machine.md`](docs/design/state_machine.md)
> 하나에만** 두고 여기서는 중복하지 않습니다.

### 유즈케이스

| # | 시나리오 | 검증 대상 |
|---|---|---|
| 1 | 정상 정리 — N개 모두 올바른 상자에 | 전체 파이프라인 + **반복 루프** |
| 2 | **범주 내 형상 변화** — 정육면체 ↔ 축구공형, 나이트 ↔ 룩 이 같은 상자로 | **범주를 배웠다는 증거** ★ |
| 3 | 파지 실패 후 복구 | 재시도 루프, **실패해도 미션 계속** |

> **유즈케이스 2가 이 프로젝트의 중심 주장입니다.** 자세 재조정이 보류되면서
> "판단하는 능력"의 근거가 기하 계산에서 **범주 일반화**로 옮겨갔습니다.
> 색 매칭·대본·하드코딩 어느 것으로도 만들 수 없는 유일한 장면입니다.

### 성공 기준 (M4 측정 대상)

| 지표 | 목표 | 측정 방법 |
|---|---|---|
| 정리 완료율 | **90% 이상** | **20 인스턴스**(4개 × 5회) 중 올바른 상자에 들어간 개수 |
| 오배치 횟수 | **0회** | **상자 2개 (8/20 확정)** → 무작위 기준선 **50%.** 원기둥 제거로 최대 위협이던 원기둥 ↔ 룩 쌍이 사라졌고, **남은 관찰 대상은 오각별기둥 ↔ 룩** (둘 다 세운 기둥) |
| 가구·상자 접촉 | **0회** | 🔴 측정 수단 미정 |
| 물체당 사이클 시간 | 60초 이내 | 2×2 m 기준 추정 58초 — 울타리 1.80 확정으로 보수적인 값 |
| FETCH 대상 정확도 | 미정 | 지시한 종류를 가져온 비율 |
| 음성 명령 인식률 | 미정 | STT 결과가 의도와 일치한 비율 |
| **오실행률 (음성)** | **0%** | STT 오인식이 **확인 없이** 실행된 횟수 |

> [!IMPORTANT]
> **오배치 0회에 무작위 기준선을 병기하는 이유** — 상자가 2개이므로 **아무렇게나 넣어도 50%는 맞습니다.**
> 8/20 확정으로 기준선이 25% → **50%** 로 올라가, **이 지표 하나만으로는 분류가 동작한다는 근거가 되지 못합니다.**
> 발표에서는 **분류 정확도(2클래스)와 함께 제시**하고, 오배치는 "기준선 대비" 가 아니라 **절대 0회** 를 목표로 서술합니다.
> 기준선 없이 "정확도 90%"라고 쓰면 성능 주장이 성립하지 않습니다.
>
> **완료율은 20 인스턴스(4개 × 5회) 기준**입니다. 4개 단회로는 90%를 측정할 수 없습니다.

### 성공 등급

| 등급 | 범위 | 목표 시점 |
|---|---|---|
| 🥉 **Minimum** | 가베 1개 · 고정 위치 → 파지 → 상자 투입 | M2 종료 (8/23) |
| 🥈 **Target** | 4개 자율 반복 정리 + **범주 일반화(`toy` 형상 3종 · `chess` 기물 3종)** + 파지 재시도 + FETCH + 음성 | M3–M4 (8/30–9/4) |
| 🥇 **Stretch** | **파지 정책 시연 학습**, 학습에 쓰지 않은 체스 세트 투입, **3~4번째 범주 추가(자세 재조정 부활)**, 밀집·겹침, 동적 장애물, 자유 문형, 웨이크워드 | 여유 시 |

---

## ✨ Key Features

| | 기능 | 상세 |
|---|---|---|
| 🎲 | **범주 일반화** — 형상이 달라도 같은 범주면 같은 상자 | [objects](docs/subsystems/objects.md) |
| 🔁 | **재관측 루프** — 행동으로 바뀐 바닥을 매 사이클 다시 관측, 실패해도 미션 계속 | [state_machine](docs/design/state_machine.md) |
| 🗣️ | **명령이 미션 파라미터를 변경** — "체스말은 검은 상자에" → `placement_rule` 갱신 | [console](docs/subsystems/console.md) |
| 🦾 | **부하 기반 파지 검증** — 힘센서 없이 서보 부하로 판정, 실패 시 재스캔 후 재시도 | [sequences](docs/design/sequences.md) |
| 🖥️ | **노트북 관제 콘솔** — GUI + 음성. 실행 위치를 옮겨도 도메인 diff 0줄 | [console](docs/subsystems/console.md) |
| 📷 | **단안 인식** — 모서리 웹캠 + 바닥면 호모그래피. 깊이 카메라 없음 | [perception](docs/subsystems/perception.md) |
| 🔌 | **전원 도메인 분리** — 팔 서보는 3S LiPo 전용, GND만 스타 접지 | — |

---

## 🧱 Architecture

### Ports & Adapters

도메인 로직을 하드웨어에서 분리합니다. 태스크 로직은 ROS2도 서보 SDK도 알지 못합니다.
**실제 구현 기준**(`domain/ports/`)으로 갱신했습니다 — 아래는 이전 표(`Perception.scan_floor()`
자율 탐색 전제, `CommandInterpreter` 직결)와 갈라진 지점입니다.

| Port | 책임 | Real Adapter | Fake Adapter |
|---|---|---|---|
| `BaseDriver` | 주행 실행(Host가 준 속도를 그대로), 상자 정렬 보조 | `Ros2MecanumBase` | `FakeBase` |
| `ArmDriver` | 관절/직교 이동, 엔드이펙터, 부하 읽기 | `FeetechArm` | `FakeArm` |
| `Perception` | **자기 뎁스캠으로 정면 라벨 확인**, 파지 확인(사라짐 판정) | `LearnedPerception` | `ScriptedPerception` |
| `Lidar` | 바구니 정면 판독(거리·yaw·좌우 오프셋) | (라이다 real 어댑터) | `FakeLidar` |
| `HostLink` | Host 명령 수신 + 상태/결과 보고 (UDP+JSON) | (real 링크) | `FakeHostLink` |
| `CommandInterpreter` | 텍스트 → 배치 규칙(`grippers_language`가 별도 소유, baseline FSM은 직접 안 씀) | `LanguageAdapter` | `ScriptedInterpreter` |

- **목표 선정·좌표·경로는 어떤 포트에도 없습니다** — 전부 Host(`grippers-host-mac`) 소유이고,
  Pi FSM은 `HostLink`로 받은 명령을 그대로 실행합니다
- 각 포트는 **Real 어댑터와 Fake 어댑터를 둘 다** 가집니다
- **CI는 매 push마다 Fake 어댑터로 전체 미션 파이프라인을 실행**합니다 — 인터페이스 불일치를 통합 시점이 아니라 커밋 시점에 검출

### ROS2 노드 분할

> [!WARNING]
> **기능 축으로 나누지 않습니다.** 주행·검출·파지는 동시에 도는 것이 아니라 **순차 단계**입니다.
> 순차 단계를 노드로 쪼개면 동시성 이득 없이 직렬화 지연·분산 상태·브레이크포인트 불가만 남습니다.
>
> **분할 기준은 둘 — 동시에 도는가, 하드웨어를 소유하는가.**

실제 존재하는 패키지(`ros2_ws/src/grippers_*`) 기준입니다. 이전 표에 있던 `grippers_inference`·
`grippers_console`은 **만들어진 적이 없습니다** — 추론은 `perception` 안에 흡수됐고, 관제 콘솔은
ROS2 패키지가 아니라 **별도 저장소의 순수 Python 앱**(`grippers-host-mac`)이 됐습니다.

| 노드/패키지 | 실행 위치 | 책임 |
|---|---|---|
| `grippers_mission` (`mission_orchestrator_node`) | Pi | `baseline_mission.py` FSM 실행, `HostLink`로 Host와 통신 |
| `grippers_perception` (`perception_node`, `depth_cam_rotate_node`, `gripper_cam_publisher_node`) | Pi | 뎁스캠 소유·회전보정, YOLO 검출(CPU/Hailo), 그리퍼캠 모니터링 스트림 |
| `grippers_arm` (`arm_driver_node`) | Pi | Feetech SDK, 관절/직교 이동, 부하, **EEPROM Homing_Offset 교시값 대조** |
| `grippers_base` (`base_driver_node`) | Pi | 메카넘 주행 실행, LiDAR |
| `grippers_language` (`language_node`) | Pi | 텍스트 → 배치 규칙(TIDY 전용, Claude 구조화 출력 + 키워드 폴백) |
| `grippers_bringup` | Pi | launch 재조합 |
| `grippers_vla` | Pi | SmolVLA/ACT VLA 실험 — **stretch 브랜치, baseline에 병합 안 함** |
| **Host — `grippers-host-mac`** | **노트북(Mac)** | 오버헤드 웹캠 2대로 좌표 추정, 목표 선정, 경로 계산, 미션 진행 — **ROS2 패키지가 아니라 순수 Python** |

> [!CAUTION]
> **`/cmd_vel` 발행 주체는 언제나 `base_driver` 하나뿐입니다.** 둘이 되면 명령이 경합해
> 로봇이 떨리거나 타이밍에 따라만 재현되는 버그가 납니다.

### 관측성 3원칙

- **모든 상태 전이를 Host에 보고** — `HostLink.report()`로 UDP+JSON을 매 사이클 보낸다
  (`domain/ports/baseline_ports.py`의 `Report` 종류 전부). `hud`/`voice_io` ROS 노드로
  구독시키는 설계는 채택되지 않았다 — Host 쪽에서 어떻게 쓰는지는 `grippers-host-mac` 참고
- **포트 호출을 전부 로깅** — 인자와 반환값을 남기면 그것이 곧 재현 가능한 시나리오
- **`ros2 bag record -a` 습관화**(단, depth/points 등 무거운 토픽은 반드시 제외 — 디스크가
  20분 안에 찬다) — 실기 시행은 되돌릴 수 없고, 로봇 1대를 여럿이 나눠 쓰므로 **녹화가 곧 시간**

### 코드 리뷰 기준

| 제약 | 목적 |
|---|---|
| 원시값(`float`, `tuple`) 직접 전달 금지 | 단위·좌표계 혼동 차단 |
| `else` 사용 지양 | 조건 분기 대신 State 객체가 다음 상태를 반환 |
| 클래스당 인스턴스 변수 2개 이하 지향 | God Node 방지 |
| **호모그래피 입력은 바운딩 박스 아래쪽 모서리** | 비스듬한 시점에서 중심점 사용 시 5~19 cm 위치 오차 |

### 단위 규약

```
각도: radian (필드명 _rad)      길이: m (_m)
개구 폭: mm (_mm) — 각도 아님    부하: 0.0~1.0
GUI 표시·문서만 도(°) 허용 — 변환은 경계에서 한 번
```

### 운영 규칙

- 파라미터 선언은 **launch 또는 노드 중 한 쪽에서만** — 중복 시 `ParameterAlreadyDeclaredException`
- `ROS_DOMAIN_ID` 는 팀 전체 동일값 고정 (노트북·로봇 간 통신 전제)
- 노드 경계 = 파일 경계 = 오너 경계

---

## 🔩 Hardware

**예산 30만원.** 주요 장비는 교육장 보유이며 발주는 웹캠·상자·물체·소모품 중심입니다.

| 구분 | 사양 | 상태 |
|---|---|---|
| 이동 베이스 | **MentorPi** (메카넘 4륜) | ✅ 보유 |
| 컴퓨트 | **Raspberry Pi 5** / Ubuntu 24.04 · 빌드는 `IntelPi` 컨테이너 | ✅ 보유 |
| AI 가속기 | **Raspberry Pi AI HAT+ 2** (Hailo-10H, 8GB) | ✅ **8/11 장착**, HailoRT 5.1.1 연동 완료(8/19) |
| 로봇 암 | **SO-ARM101** 리더/팔로워 2대 · Feetech STS3215 ×6 | ✅ 보유 |
| **오버헤드 웹캠 2대** | **Logitech HD Pro Webcam C920 ×2 대향** — Host(노트북)가 직접 물려 아레나 좌표·목표를 산출(ArUco 기반). [🎯 Mission](#-mission)의 작업 공간 표는 이전 검토안(C270) 기준이라 픽셀 단위 수치는 재실측 필요 | ✅ 사용 중 |
| **뎁스 카메라** | ascamera SDK 구동, **HP60C 계열**(`ascamera_node`, `depth_cam_rotate_node`가 180° 회전 보정) — Pi 탑재, 바구니 정면(라이다 보조)·GRASP confirm 신호 | ✅ 사용 중 |
| **LiDAR** | **LDRobot LD19** — 정면 11.3° 하향틸트, 바구니 정면 거리·yaw 판정 전용 | ✅ 보유 |
| **그리퍼 캠(엔드이펙터 탑재)** | USB, `gripper_cam_publisher_node`로 ROS2 브리지 — 모니터링 전용(자동 GRASP 판정에는 미사용) | ✅ 사용 중 |
| **웹캠 거치 수단** | **보유 스탠드로 높이 1.65 m 거치 (8/20 확정)** — 발주 없음. 카메라 고정이 호모그래피의 전제 | ✅ 확정 |
| **상자 2개** | 🟢 초록 · 🔵 파랑 · **0.29 × 0.35 m · 높이 0.22 m** · **먼 쪽 양쪽 모서리** · ⚫🔴 는 ⏸ 미사용 | ✅ 보유 |
| **`toy` 3형상** | **정육면체 · 축구공형 다면체 · 오각별기둥 (8/20 개정)** · 3D 프린팅 · **폭 40 mm** (검출 하한 37.4 · 그리퍼 설계 상한 45) | 자작 |
| **체스말** | **나이트 · 퀸 · 룩 3종** · 3D 프린팅 · 스케일 3종 × 색 3~4종 | 자작 (실측 후) |
| 시연용 노트북 | 관제 콘솔 + STT/TTS. ROS 2 통신 가능해야 함 | ⚠️ 사양 확인 |
| 접촉 감지 · E-STOP | 성공 기준 측정 · 안전 | 🔴 결정 필요 |
| 팔 전용 전원 · 크래들 | 3S LiPo · Transport Pose 안착 | 발주 / 자작 |
| 전용 라우터 / 핫스팟 | 시연장 WiFi AP 격리 대비 | ⚠️ 권장 |

**발주 전 확인 필수**

- 🔴 **추론 입력 해상도** — `imgsz=640` 으로 넣으면 물체가 절반으로 줄어 **11~15 px** 이 됩니다. **네이티브 1280 으로 추론하고 학습도 같은 해상도**로 맞춰야 합니다. `scan_floor()` 는 사이클당 1~4 회라 연산은 감당됩니다 → [perception](docs/subsystems/perception.md)
- **설치 높이 1.65 m · 후퇴 0.95 m (8/20 팀 확정)** — 이전 1.6 m 는 Houdini 합성 데이터 생성 높이가 유일한 근거였지만, 지금 값은 **C270 실측 화각(`f = 1410 px`)에서 역산한 기하**입니다. 검토안이던 2.10 m 는 채택하지 않았습니다 → [`workspace_layout.html`](docs/design/workspace_layout.html)
- **울타리 1.80 × 1.80 m 확정 (8/20)** — 2×2 m 안 폐기. 바닥 소요가 **3.70 × 1.80 m** 로 팀 초안(4.88)보다 1.2 m 작습니다. 고도각 · 슬랜트 · 검출 픽셀 수의 기준이 확정되어 M2 데이터 생성 기준이 고정됩니다
- ⚠️ **한 변 치수는 재확인 대상입니다** — 팀 확정은 **1.80**, 프레임 역산은 **1.88** 이었습니다. 가벽 조립 후 실측할 것. 80 mm 차이가 근거리 모서리 `u` 를 30 px 움직입니다
- ⚠️ **가림 대책은 2대 대향과 배치 규칙입니다.** 고도각이 **29.7 ~ 51.6°** 로 이전 단일 카메라(30°)보다 낫지만, 최악점(중앙선 측면 · 38.7°)에서 0.10 m 물체의 가림이 0.125 m 입니다. **물체 최소 이격 0.30 m** 를 지키고 A2 실측에 가림 케이스를 포함할 것
- **`toy` 3형상은 3D 프린트로 전환 (8/14)** — 시판 가베 실측이 **3×3 cm** 로 검출 하한에 미달했습니다. 체스말과 같은 파이프라인이라 크기 · 색을 직접 통제할 수 있고, Asset SSoT 가 `toy` 까지 확장됩니다. 발주에서는 제외
- ⚠️ **오각별기둥은 파지 검증이 필요합니다** — 5회 대칭이라 **마주보는 평행면이 0쌍**입니다(짝수각 기둥은 n/2쌍). 평행 조가 닫힐 때 접촉점이 어긋나 물체가 회전할 수 있고, 폭 40 mm 기준 별 끝의 두께가 얇아 인필 15~20% 에서 파손 위험이 있습니다. **TPU 조의 순응·마찰이 이를 얼마나 흡수하는지 M2-P1 출력 후 실측**할 것
- ✅ **그리퍼 개구 폭 실측 완료 (8/20)** — TPU 인쇄본 **파지 상한 50 mm**, 접근 클리어런스를 보면 설계 상한 **45 mm** 입니다. `toy` 폭 40 mm 와 체스말 넥 지름이 이 값에서 확정됩니다. **바닥 파지 리치는 계속 진행** — 리더암 텔레오퍼레이션 30분

전원 도메인·실측 항목 → [`milestones.md`](docs/ops/milestones.md)

---

## 📁 Repository Structure

```
grippers/
├── README.md               # 개요 + 문서 지도
├── CONTRIBUTING.md         # 브랜치·PR·품질 기준        ← 이슈/PR 화면에 자동 링크
├── LICENSE                 # MIT (+ LeRobot Apache 2.0 고지)  ← GitHub 탭 생성
├── pyproject.toml          # ruff / black 설정 — 벤더 코드는 lint 대상에서 제외
├── .gitignore
├── .gitmodules             # third_party/soarm_provided_d
├── .github/
│   ├── CODEOWNERS          #   경로별 리뷰어 자동 지정
│   ├── pull_request_template.md
│   ├── ISSUE_TEMPLATE/     #   task.yml, bug.yml, config.yml
│   └── workflows/
│       └── ci.yml          #   lint → test → colcon build → docker build
├── domain/                 # 순수 Python. 하드웨어 의존성 없음 (ROS2 import 금지)
│   ├── task/               #   State 제너레이터 기반 루프 FSM
│   ├── values.py           #   Detection, BoxObservation, MissionSpec, MissionContext 등
│   ├── ports/              #   인터페이스 정의 (ABC) — 포트 4종
│   └── adapters/
│       ├── real/           #   ROS2 액션/서비스 클라이언트
│       └── fake/           #   테스트용 구현 (CI에서 사용)
├── ros2_ws/
│   └── src/
│       ├── grippers_interfaces/  # 공통 msg/srv/action (노드 간 유일한 접점)
│       ├── grippers_base/        # base_driver_node
│       ├── grippers_arm/         # arm_driver_node — soarm_lab(third_party) 래핑, EEPROM 교시값 대조
│       ├── grippers_perception/  # perception_node · depth_cam_rotate_node · gripper_cam_publisher_node
│       ├── grippers_mission/     # mission_orchestrator_node — domain FSM(baseline_mission.py) 실행
│       ├── grippers_language/    # language_node — 텍스트 → 배치 규칙 (TIDY 전용)
│       ├── grippers_vla/         # ⏸ SmolVLA/ACT 실험 — stretch, baseline에 병합 안 함
│       ├── grippers_bringup/     # launch 재조합
│       └── (app/ bringup/ driver/ interfaces/ navigation/ peripherals/
│            simulations/ slam/ yolov5_ros2/ 등 — MentorPi 벤더 소스 보존. lint 제외)
│
│       ⚠️ `grippers_inference`·`grippers_console`은 계획만 있었고 만들어지지 않았다 —
│       추론은 `grippers_perception`에 흡수됐고, 관제 콘솔은 ROS2 패키지가 아니라
│       별도 저장소(`grippers-host-mac`)의 순수 Python 앱이 됐다.
├── third_party/
│   └── soarm_provided_d/   # git submodule — soarm_lab (FK/IK/시뮬/실물 백엔드)
├── tests/                  # pytest — 하드웨어·ROS2 불필요, domain/ + Fake 어댑터만 사용
├── docs/                   # snake_case 통일 · 각 폴더에 README.md(폴더 안내)
│   ├── design/             #   ── 설계 ──
│   │   ├── state_machine.md    #   ⭐ FSM 전이 단일 소스 · ✅ 실 코드 기준 갱신됨(9/3)
│   │   ├── class_diagram.md    #   클래스 다이어그램 (Mermaid) + 마이그레이션 계획 ⚠️ 갱신 대기
│   │   ├── sequences.md        #   시퀀스 다이어그램 ⚠️ 갱신 대기
│   │   ├── architecture.puml   #   같은 구조의 PlantUML 버전 ⚠️ 갱신 대기
│   │   ├── hld.md              #   High Level Design — 인터페이스 명세 ⚠️ 갱신 대기
│   │   └── error_budget.md     #   오차 전파 분석 ⚠️ 갱신 대기
│   ├── subsystems/         #   ── 서브시스템 ──
│   │   ├── objects.md          #   물체 구성 · 클래스 체계 · 3D 프린팅 · 시연 구성
│   │   ├── perception.md       #   인식 — 해상도 · 호모그래피 · 검정 상자 · 가림
│   │   ├── console.md          #   노트북 관제 콘솔 — GUI · 음성 · 네트워크
│   │   ├── ai_components.md    #   학습 범위 · 데이터 · 가속기 근거 · HEF 파이프라인
│   │   └── pose_planning.md    #   ⏸ 보류된 자세 재조정 (재도입 절차 포함)
│   └── ops/                #   ── 운영 ──
│       ├── setup.md            #   설치 · 실행 · 테스트 · 트러블슈팅
│       ├── milestones.md       #   일정 · 미결 사항 · 리스크 · 측정 결과
│       ├── measurements.md     #   실측 리포트
│       ├── failure_definition.md # 무엇을 실패로 셀 것인가 — 계수 규칙
│       ├── purchase_ledger.md  #   구매 장부
│       └── rejected_designs.md #   채택하지 않은 설계와 근거
└── hardware/               # 마운트·크래들 도면, BOM, 배선도
```

> **`grippers_console` 이 노트북에서 도는 유일한 패키지입니다.** 나머지는 전부 로봇 온보드입니다.
> 이 경계가 곧 "노트북이 다 하는 것 아니냐"에 대한 답입니다.

> **루트 2개 파일은 GitHub가 특별 취급합니다.** `LICENSE` 는 저장소 첫 화면 README 위에
> **탭으로 표시**되고, `CONTRIBUTING.md` 는 이슈·PR 생성 화면에 자동 링크됩니다.
> 그래서 이 둘만 `docs/` 가 아니라 루트에 둡니다.

> **lint 범위**: `pyproject.toml` 이 MentorPi 벤더 패키지와 `third_party/` 를 ruff·black 대상에서 제외합니다.
> 검사 대상은 `domain/`, `tests/`, `ros2_ws/src/grippers_*` 입니다.

---

## ⚡ Quick Start

```bash
# 하드웨어 없이 도메인 로직 검증 (ROS2도 불필요)
git clone --recurse-submodules https://github.com/grippers-intel/grippers.git
cd grippers && export PYTHONPATH="$PWD:$PYTHONPATH"
python3 -m pytest tests/ -v
```

`tests/`는 하드웨어·ROS2 없이 `domain/` + Fake 어댑터만으로 FSM 전체(`IDLE → APPROACH →
GRASP → CARRY → INSERT → IDLE`)를 검증합니다. 전이 상세는 [`state_machine.md`](docs/design/state_machine.md).

실기 실행·컨테이너 설정·트러블슈팅 → [`setup.md`](docs/ops/setup.md)

---

## 📅 Milestones

> **최종 발표: 2026년 9월 8일 (화)**

| 마일스톤 | 기간 | 핵심 완료 조건 |
|---|---|---|
| **M1 · 설계 (축소)** | ~8/14 | 바닥 파지 리치 실측(게이트) · 발주 · 배포판 확정 · Tier-1 freeze |
| **M2 · 기반 재구축** | 8/15–8/22 | 마이그레이션 10 PR · 호모그래피 · 2범주 검출 · CI 그린 |
| **M3 · 실기 통합** | 8/23–8/30 | **1개 End-to-end 성공** · 네트워크 실측 · 컷오프 |
| **M4 · 확장 + 측정** | 8/31–9/4 | **4개 반복 정리** · FETCH · 측정 20회 · 9/4 freeze |
| **M5 · 발표 준비** | 9/1–9/8 | 9/6 포스터 제출 · **9/8 발표** |

담당별 작업·미결 사항·리스크 → [`milestones.md`](docs/ops/milestones.md)

---

## 👥 Team

| 이름 | GitHub | 핵심 롤 | 세부 담당 |
|---|---|---|---|
| **이승용** | [@sysy009](https://github.com/sysy009) | Git Master · Language · 전체 설계 총괄 | Milestone 선언, issue 발행·할당, HLD, 명령 해석·문형 설계 |
| **김동혁** | [@Feroninn](https://github.com/Feroninn) | Git Slave · Vision · 발주 | Conflict 해결, `git blame` 추적, 검출 모델, 부품 발주·장부 |
| **임성혁** | [@alex7663](https://github.com/alex7663) | Action · 하드웨어 총괄 | 파지 정책, 액션 시퀀스, 기구부/전장, 조립·배선 |
| **조현우** | [@kica927](https://github.com/kica927) | 중심잡기 · 코드 수장 · ROS2 | 구조 설계, 자세 안정화, ROS2 노드·통신, 코드 리뷰 |
| **김희수** | [@Hease](https://github.com/Hease) | Perception · 데이터 · UI/UX · 음성 | 데이터셋, 검출 학습, **노트북 관제 콘솔**, STT/TTS, 시각화 |

**공통 책임** — 본인 영역 unittest 작성 · 타인 PR 1일 내 review · 금요일 마일스톤 progress 업데이트 · 막히면 24시간 내 공유

---

## 🤝 Contributing

`main` 직접 push 금지, topic branch → PR → review → merge.
브랜치·품질 기준·Git 운영 체계 → [CONTRIBUTING.md](CONTRIBUTING.md)

---

## 📃 License

본 프로젝트 코드는 MIT License로 배포합니다. 자세한 내용은 [LICENSE](LICENSE) 참고.

LeRobot 기반 구성 요소는 **Apache License 2.0** 을 따릅니다. 해당 코드를 포함하거나 파생한 파일에는 원 라이선스 고지를 유지합니다.

---

## 📚 References

- [LeRobot / SO-ARM101](https://github.com/huggingface/lerobot)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) · [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [ROS 2 DDS 튜닝](https://docs.ros.org/en/jazzy/How-To-Guides/DDS-tuning.html)
- [Raspberry Pi AI HAT+ 2](https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html)
- [Hailo Model Zoo — DFC 요구 사양](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/GETTING_STARTED.rst)
- [whisper.cpp](https://github.com/ggerganov/whisper.cpp) · [Vosk](https://alphacephei.com/vosk/)
- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)

<div align="center">

[⬆ Back to top](#-grippers)

</div>
