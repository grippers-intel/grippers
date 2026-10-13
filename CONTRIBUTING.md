# Contributing

Grippers 프로젝트에 기여해주셔서 감사합니다. 팀 내부 규칙이지만 외부 제안도 환영합니다.

### 브랜치 & PR

- `main` 브랜치에 **직접 push 금지** — 저장소 규칙(ruleset)으로 막혀 있다(직접 push · 강제 push · 삭제 금지, 일렬 기록)
- topic branch → PR 등록 → **peer review 후 approval(1명)** → merge
- 브랜치 네이밍: `feat/`, `fix/`, `docs/`, `refactor/`, `test/`, `ci/` + 짧은 설명
- 커밋 메시지에 issue 번호 참조, PR 본문에 `Closes #N`
- 한 PR 에는 한 주제만 — 코드 · 문서 · CI 변경을 섞지 않는다

### 병합

- 방식: **squash** 기본(한 주제 = main 의 커밋 하나). 커밋을 그대로 남길 가치가 있으면 rebase. 병합 커밋은 저장소 설정상 쓸 수 없다.
- 리뷰어가 없을 때: **Repository admin 만** PR 화면에서 규칙을 우회해 병합할 수 있다(직접 push 는 admin 도 막힘).
  우회 병합은 기록이 남으니 PR 본문에 이유를 적는다.
  ```bash
  gh pr merge <번호> -R grippers-intel/grippers --squash --admin --delete-branch
  ```

### Git 운영 (Master / Slave 체계)

| 역할 | 담당 | 책임 |
|---|---|---|
| **Master** | 이승용 | Milestone 선언, issue 발행·할당, 머지 순서 조율, 릴리즈 태깅 |
| **Slave** | 김동혁 | Conflict 해결 주도, 커밋 로그·`git blame` 추적, 사고 시 원인 커밋 특정 |

대형 리팩터링은 사전 공지 후 단독 PR로 분리합니다 (conflict 지옥 방지).

### 품질

| 항목 | 도구 / 규칙 |
|---|---|
| 정적분석 | **ruff** — 저장소 루트에서 `ruff check .` (버전 0.17.0, CI 와 같게) |
| 포맷 | **black**(26.10.1) — 루트 · `legacy/` 만. `vla_robot/` 은 black 을 쓰지 않고 자체 규칙 `vla_robot/ruff.toml`(Python 3.10 · 140자 · E/F/B) |
| CI | push · PR 마다 4개 작업: 린트 · legacy 시험 · vla_robot Host 시험(Python 3.11) · vla_robot Pi 쪽 시험(Python 3.10) |
| 코드 리뷰 최종 판단 | 조현우 |
| 개발 환경 | Host = Windows · Python 3.11 / Pi = Raspberry Pi 5 · Docker(`IntelPi`) · ROS 2 Humble · Python 3.10 |

```bash
ruff check .                                  # 저장소 루트
cd vla_robot/host && pytest                   # Host 시험
```

- **Pi 에서 도는 코드는 Python 3.10 문법까지만** 쓴다(ROS 2 Humble). `vla_robot/ruff.toml` 이 3.10 기준으로 검사한다.
- 설정 파일(`host.yaml` · `robot.yaml`)은 모르는 키가 있으면 기동을 거부한다 — 키를 더하면 로더(`host_config.py` · `vla_common/config.py`)와 시험을 같이 고친다.
- 실행 코드(Pi 노드 · Host 미션 · 설정 값)의 **동작을 바꾸는** 변경은 실기 확인이 필요하다 — PR 본문에 확인 여부를 적는다.
  동작이 그대로인 정리는 시험 · `vla_robot/host/tools/sim_regress.py` 로 확인한다.

## 이슈 · PR

- 이슈 템플릿: `.github/ISSUE_TEMPLATE/` (task · bug)
- PR 본문에 `Closes #N` 표기
- `.github/CODEOWNERS` 가 경로별 리뷰어를 자동 지정합니다

## 문서

- 현재 시스템 문서는 `docs/`, 목차는 [`docs/README.md`](docs/README.md). 파일명은 **snake_case**
- 상태 · 전이는 [`docs/state_machine.md`](docs/state_machine.md), Host ↔ Pi 규격은 [`docs/interfaces.md`](docs/interfaces.md) 가 기준 — 그림과 표는 코드에서 확인해 쓰고, 코드를 바꾸면 같이 고친다
- 설계 변경 시 README 문서 표와 해당 docs 문서를 함께 갱신
- `legacy/` 는 2026년 8월 초기 설계 기록 — 고치지 않는다
