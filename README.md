<div align="center">

# 🤖 Grippers

**바닥에 흩어진 물건을 스스로 정리하고, 요청하면 가져다주는 모바일 매니퓰레이터**

⭐ 팀 프로젝트입니다 — 이슈와 제안 환영합니다 🙏

</div>

---

> [!IMPORTANT]
> **주제 변경 (8/12)** — *무인 멸균 암실 장물 반출* → *바닥 물건 정리 + 요청 배달*.
> 하드웨어·아키텍처는 유지, 미션 시나리오와 FSM 구조가 바뀌었습니다.
>
> ⚠️ **`domain/` 코드는 아직 이전 주제 기준입니다.** 이 README는 to-be 설계이며
> 코드 마이그레이션은 [`class_diagram.md` §5](docs/design/class_diagram.md) 의 PR 10건으로 진행합니다 (M2, 8/22).
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
| **범주 내 형상이 다양함** | 정육면체·육각기둥·원기둥이 전부 `toy`. 개별 형상을 외우는 방식으로는 처음 보는 형상에서 실패 |
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
| [`pose_planning.md`](docs/subsystems/pose_planning.md) | ⏸ 보류된 자세 재조정 설계 (재도입 절차 포함) |
| **설계 다이어그램** | |
| [`state_machine.md`](docs/design/state_machine.md) | **FSM 전이 단일 소스** |
| [`class_diagram.md`](docs/design/class_diagram.md) | 값 객체 · 포트 · State · 노드 계층 · 마이그레이션 |
| [`sequences.md`](docs/design/sequences.md) | 시퀀스 다이어그램 |
| [`architecture.puml`](docs/design/architecture.puml) | PlantUML 버전 |

기여 방법은 [CONTRIBUTING](CONTRIBUTING.md) 참고.

---

## 🎯 Mission

| 모드 | 행동을 결정하는 것 | 진입 명령 |
|---|---|---|
| **TIDY** | 규칙 (`placement_rule`) | "장난감 정리해줘" |
| **FETCH** | 사람의 지시 | "체스말 가져와" |

두 모드는 **같은 하드웨어·포트·파지 로직**을 쓰고, **파지 이후 목적지 결정에서만 갈라집니다.**
분기는 `GraspState.execute()` 의 마지막 한 줄입니다.

### 작업 공간

```
        2 m
  ┌───────────────┐
  │ ⬜          🔵 │   상자 4개 — 검정 · 빨강 · 파랑 · 초록
  │               │
  │   ▪  ⬡   ■    │2 m 🟢 가베      (원기둥·정육면체·육각기둥)
  │        ♞      │   🔵 체스말     (나이트·퀸·룩 — 3D 프린팅)
  │ 🟢          ⬜ │   ⬜ 2개는 **방해 선택지** — 명령으로만 활성화
  └───────────────┘   상자는 코너 · 벽에서 30~40cm 이격 (팔 도달성)
```

### 클래스 체계

**수거 대상은 2 클래스 × 형상 3종으로 확정했습니다 (8/14).**

| 클래스 | 포함 개체 = 학습 대상 | 상자 | 상태 |
|---|---|---|---|
| `toy` | **원기둥 · 정육면체 · 육각기둥** — 가베 교구 | 🟢 초록 | **확정 (8/14)** |
| `chess` | **나이트 · 퀸 · 룩** — 3D 프린팅, 스케일 확대 | 🔵 파랑 | **확정 (8/14)** |
| *(방해 선택지)* | — · 기본 TIDY 에서는 목적지가 아님 | ⚫ 검정 · 🔴 빨강 | **명령으로 활성화** |

- `toy` 는 **가베 형상을 따르되 3D 프린트로 제작**합니다 (8/14 전환) — 시판 가베가 3×3 cm 로 검출 하한에 미달했습니다.
- `chess` 는 **나이트 · 퀸 · 룩 3종을 우선** 학습합니다.
- 두 클래스 모두 **학습 대상과 시연 투입 물체가 동일**합니다 — 별도의 미학습 형상을 두지 않습니다.
- ⚫ 검정 · 🔴 빨강은 **방해 선택지**입니다. 기본 TIDY 에서는 비어 있지만 **네 상자 모두 물리적으로 놓여 있어 투입 가능한 후보**이고, "체스말은 검은 상자에" 같은 명령이 들어오면 `placement_rule` 이 갱신되어 목적지가 됩니다.

> **`toy` 안에 형상이 다른 개체가 여러 개 있는 것이 핵심 주장입니다.** 정육면체와 육각기둥은
> 실루엣이 다른데 같은 상자로 갑니다 — 색 매칭이나 대본으로는 만들 수 없는 장면이며,
> 모델이 **범주를 배웠다는 증거**입니다.

상세 → [`objects.md`](docs/subsystems/objects.md)

### FSM — 루프 구조

```
IDLE → SCAN ─┬─ (미처리 대상 0개) ──────────────────────→ DONE
             │
             └─ SELECT → APPROACH → GRASP ─┬─ TIDY  → TRANSPORT → POSE_PLAN ─┬─ INSERT ─┐
                                            │                                 └─ REJECT ─┤
                                            └─ FETCH → DELIVER → HANDOVER ───────────────┤
                          ┌──────────────────────────────────────────────────────────────┘
                          └──→ SCAN 복귀 (루프)
```

**루프가 핵심입니다.** 로봇 자신의 행동으로 바뀐 바닥 상태를 매 사이클 재관측하며,
파지에 실패해도 미션이 끝나지 않고 다음 물체로 진행합니다.

전이 그래프의 단일 소스는 [`docs/design/state_machine.md`](docs/design/state_machine.md) 입니다.

### 유즈케이스

| # | 시나리오 | 검증 대상 |
|---|---|---|
| 1 | 정상 정리 — N개 모두 올바른 상자에 | 전체 파이프라인 + **반복 루프** |
| 2 | **범주 내 형상 변화** — 원기둥 ↔ 육각기둥, 나이트 ↔ 룩 이 같은 상자로 | **범주를 배웠다는 증거** ★ |
| 3 | 파지 실패 후 복구 | 재시도 루프, **실패해도 미션 계속** |

> **유즈케이스 2가 이 프로젝트의 중심 주장입니다.** 자세 재조정이 보류되면서
> "판단하는 능력"의 근거가 기하 계산에서 **범주 일반화**로 옮겨갔습니다.
> 색 매칭·대본·하드코딩 어느 것으로도 만들 수 없는 유일한 장면입니다.

### 성공 기준 (M4 측정 대상)

| 지표 | 목표 | 측정 방법 |
|---|---|---|
| 정리 완료율 | **90% 이상** | **20 인스턴스**(4개 × 5회) 중 올바른 상자에 들어간 개수 |
| 오배치 횟수 | **0회** | **상자 4개 모두 투입 가능한 후보** → 무작위 기준선 **25%.** 최대 위협은 원기둥 가베 ↔ 체스말 혼동 |
| 가구·상자 접촉 | **0회** | 🔴 측정 수단 미정 |
| 물체당 사이클 시간 | 60초 이내 | 2×2 m 기준 추정 58초 |
| FETCH 대상 정확도 | 미정 | 지시한 종류를 가져온 비율 |
| 음성 명령 인식률 | 미정 | STT 결과가 의도와 일치한 비율 |
| **오실행률 (음성)** | **0%** | STT 오인식이 **확인 없이** 실행된 횟수 |

> **오배치 0회에 무작위 기준선을 병기하는 이유** — ⚫🔴 를 포함해 **상자 4개가 모두 투입 가능한 후보**이므로 아무렇게나 넣어도 25%는 맞습니다.
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

| Port | 책임 | Real Adapter | Fake Adapter |
|---|---|---|---|
| `BaseDriver` | 주행, 상자 정렬, 오도메트리 | `Ros2MecanumBase` | `FakeBase` |
| `ArmDriver` | 관절/직교 이동, 엔드이펙터, 부하, 자세 재조정 | `FeetechArm` | `FakeArm` |
| `Perception` | **바닥 스캔**, 검출, 치수, 상자 색 탐색 | `LearnedPerception` | `ScriptedPerception` |
| `CommandInterpreter` | 텍스트 → `MissionSpec` | `LanguageAdapter` | `ScriptedInterpreter` |

- 각 포트는 **Real 어댑터와 Fake 어댑터를 둘 다** 가집니다
- **CI는 매 push마다 Fake 어댑터로 전체 미션 파이프라인을 실행**합니다 — 인터페이스 불일치를 통합 시점이 아니라 커밋 시점에 검출
- 로봇 1대를 5명이 나눠 쓰는 병목을 구조로 해소하기 위한 설계입니다

> `Perception.scan_floor()` 는 **목록을 반환**하므로 원소 타입(`Detection`) 정의가 선행되어야 합니다.
> **음성은 포트를 추가하지 않습니다** — `voice_io` 가 기존 명령 토픽에 발행할 뿐입니다.

### ROS2 노드 분할

> [!WARNING]
> **기능 축으로 나누지 않습니다.** 주행·검출·파지는 동시에 도는 것이 아니라 **순차 단계**입니다.
> 순차 단계를 노드로 쪼개면 동시성 이득 없이 직렬화 지연·분산 상태·브레이크포인트 불가만 남습니다.
>
> **분할 기준은 둘 — 동시에 도는가, 하드웨어를 소유하는가.**

| 노드 | 실행 위치 | 책임 | 분리 근거 | 오너 |
|---|---|---|---|---|
| `mission_orchestrator` | 로봇 | FSM 전체. 포트를 호출해 순차 로직 진행 | 순차 로직은 한 프로세스에 — 디버거 추적 가능 | 이승용 |
| `perception` | 로봇 | **웹캠 2대 소유**, 바닥면 호모그래피, 상자 색 탐색, 클리어런스 | 상시 구동 + 디바이스 소유 | 김동혁 · 김희수 |
| `inference` | 로봇 | 검출 추론 전담 | **Hailo-10H 독점 소유** | 김동혁 |
| `arm_driver` | 로봇 | Feetech SDK, 관절/직교 이동, 엔드이펙터, 부하 | 상시 구동 + 디바이스 소유 | 임성혁 |
| `base_driver` | 로봇 | 메카넘 주행, LiDAR | 상시 구동 + 디바이스 소유 | 조현우 |
| **`voice_io`** | **노트북** | STT → `/command` 발행, `/mission/state` → TTS | **노트북 오디오 장치 소유** | 김희수 |
| `hud` | 노트북 | 대시보드 — `/mission/state` 만 구독 | 시연 중 끊겨도 미션에 영향 없어야 함 | 김희수 |

> [!CAUTION]
> **`/cmd_vel` 발행 주체는 언제나 `base_driver` 하나뿐입니다.** 둘이 되면 명령이 경합해
> 로봇이 떨리거나 타이밍에 따라만 재현되는 버그가 납니다.

### 관측성 3원칙

- **상태 전이를 토픽으로 발행** — `/mission/state` 를 `transient_local` QoS로. 중간에 붙어도 현재 상태 즉시 파악. HUD·TTS 모두 이 토픽 하나만 구독
- **포트 호출을 전부 로깅** — 인자와 반환값을 남기면 그것이 곧 재현 가능한 시나리오
- **`ros2 bag record -a` 습관화** — 실기 시행은 되돌릴 수 없고, 로봇 1대를 5명이 나눠 쓰므로 **녹화가 곧 시간**

### 코드 리뷰 기준

| 제약 | 목적 |
|---|---|
| 원시값(`float`, `tuple`) 직접 전달 금지 | 단위·좌표계 혼동 차단 |
| `else` 사용 지양 | 조건 분기 대신 State 객체가 다음 상태를 반환 |
| 클래스당 인스턴스 변수 2개 이하 지향 | God Node 방지 (루프 상태는 `MissionContext` 하나로 묶음) |
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
| AI 가속기 | **Raspberry Pi AI HAT+ 2** (Hailo-10H, 8GB) | ✅ **8/11 장착** |
| 로봇 암 | **SO-ARM101** 리더/팔로워 2대 · Feetech STS3215 ×6 | ✅ 보유 |
| **모서리 웹캠** | USB **1080p 이상 · HFOV 60~70°** · **환경 고정 · 설치 높이 1.6 m (8/14 확정)** | 발주 |
| **로봇 탑재 웹캠** | USB — 근거리 파지 확인 · 클리어런스 | 발주 |
| **웹캠 거치 수단** | **환경 고정물에 1.6 m 로 거치 (8/17 확정)** — 삼각대 · 폴대 · 클램프 발주 없음 | ✅ 확정 |
| **상자 4개** | ⚫🔴🔵🟢 · 입구 짧은 변 0.40 m · **검정은 밝은 테두리 필수** | 발주 |
| **`toy` 3형상** | **원기둥 · 정육면체 · 육각기둥** · 3D 프린팅 · 최소 폭은 그리퍼 개구 폭 실측 후 확정 | 자작 (실측 후) |
| **체스말** | **나이트 · 퀸 · 룩 3종** · 3D 프린팅 · 스케일 3종 × 색 3~4종 | 자작 (실측 후) |
| 시연용 노트북 | 관제 콘솔 + STT/TTS. ROS 2 통신 가능해야 함 | ⚠️ 사양 확인 |
| 접촉 감지 · E-STOP | 성공 기준 측정 · 안전 | 🔴 결정 필요 |
| 팔 전용 전원 · 크래들 | 3S LiPo · Transport Pose 안착 | 발주 / 자작 |
| 전용 라우터 / 핫스팟 | 시연장 WiFi AP 격리 대비 | ⚠️ 권장 |

**발주 전 확인 필수**

- **웹캠 1080p** — 640×480은 먼 모서리에서 9 px로 검출 불가 → [perception](docs/subsystems/perception.md)
- **설치 높이 1.6 m (8/14 확정) · 환경 고정물 거치 (8/17 확정)** — 높이의 근거는 **Houdini 합성 데이터가 이 높이로 생성 중**이라는 것 하나입니다. 천장 · 거치 물리 제약에서 나온 값이 아니므로, 별도 제약이 생기면 기록하고 재검토합니다. 거치는 **환경에 고정**하므로 삼각대·폴대·클램프는 발주하지 않습니다 — **카메라 고정이 호모그래피의 전제**이고, 그 전제가 유지되므로 A2 판정 기준도 그대로입니다
- **작업 공간 2×2 m 확정 (8/17)** — 2.5×2.5 m 안 폐기. 고도각 · 슬랜트 · 검출 픽셀 수의 기준이 확정되어 M2 데이터 생성 기준이 고정됩니다
- ⚠️ **1.6 m 의 대가는 가림입니다.** 먼 모서리 고도각이 **30°** (슬랜트 3.25 m) 입니다. 로봇(0.3 m) · 상자(0.4 m) 가 시야를 가리므로 **밀집 · 겹침 배치를 피하고 A2 실측에 가림 케이스를 반드시 포함**할 것
- **`toy` 3형상은 3D 프린트로 전환 (8/14)** — 시판 가베 실측이 **3×3 cm** 로 검출 하한에 미달했습니다. 체스말과 같은 파이프라인이라 크기 · 색을 직접 통제할 수 있고, Asset SSoT 가 `toy` 까지 확장됩니다. 발주에서는 제외
- **바닥 파지 리치 · 그리퍼 개구 폭** — 체스말 STL이 여기 종속. 리더암 텔레오퍼레이션 30분. **프린트 시간을 감안하면 8/15 까지 실측을 끝내야 M2 에 맞습니다**

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
│       ├── grippers_arm/         # arm_driver_node — soarm_lab(third_party) 래핑
│       ├── grippers_perception/  # perception_node — 카메라 소유, 검출, 상자 탐색
│       ├── grippers_inference/   # inference_node — Hailo-10H 전담
│       ├── grippers_mission/     # mission_orchestrator_node — domain FSM을 스레드 분리 실행
│       ├── grippers_console/     # ⭐ 노트북 실행 — voice_io_node, hud_node
│       ├── grippers_bringup/     # launch 재조합
│       └── (app/ bringup/ driver/ interfaces/ navigation/ peripherals/
│            simulations/ slam/ yolov5_ros2/ 등 — MentorPi 벤더 소스 보존. lint 제외)
├── third_party/
│   └── soarm_provided_d/   # git submodule — soarm_lab (FK/IK/시뮬/실물 백엔드)
├── tests/                  # pytest — 하드웨어·ROS2 불필요, domain/ + Fake 어댑터만 사용
├── docs/                   # snake_case 통일 · 각 폴더에 README.md(폴더 안내)
│   ├── design/             #   ── 설계 ──
│   │   ├── state_machine.md    #   ⭐ FSM 전이 단일 소스
│   │   ├── class_diagram.md    #   클래스 다이어그램 (Mermaid) + 마이그레이션 계획
│   │   ├── sequences.md        #   시퀀스 다이어그램
│   │   ├── architecture.puml   #   같은 구조의 PlantUML 버전
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

`IDLE → SCAN → ... → DONE` 루프가 **유한 스텝 안에 종료되면** 도메인 로직은 정상입니다.

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
