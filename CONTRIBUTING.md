# Contributing

Grippers 프로젝트에 기여해주셔서 감사합니다. 팀 내부 규칙이지만 외부 제안도 환영합니다.

### 브랜치 & PR

- `main` 브랜치에 **직접 push 금지**
- topic branch → PR 등록 → **peer review 후 approval** → merge
- 브랜치 네이밍: `feat/`, `fix/`, `docs/`, `refactor/` + 짧은 설명
- 커밋 메시지에 issue 번호 참조, PR 본문에 `Closes #N`

### Git 운영 (Master / Slave 체계)

| 역할 | 담당 | 책임 |
|---|---|---|
| **Master** | 이승용 | Milestone 선언, issue 발행·할당, 머지 순서 조율, 릴리즈 태깅 |
| **Slave** | 김동혁 | Conflict 해결 주도, 커밋 로그·`git blame` 추적, 사고 시 원인 커밋 특정 |

대형 리팩터링은 사전 공지 후 단독 PR로 분리합니다 (conflict 지옥 방지).
**주제 전환 마이그레이션은 [`docs/design/class_diagram.md` §5](docs/design/class_diagram.md) 의 순서를 따릅니다** —
값 객체 → 포트 → Fake → `states.py` 순으로, 순서를 바꾸면 재작성 내내 CI가 빨간불입니다.

### 품질

| 항목 | 도구 / 규칙 |
|---|---|
| 정적분석 | **ruff** |
| 포맷 | **black** |
| CI/CD | PR마다 lint + unittest 자동 실행 |
| 코드 리뷰 최종 판단 | 조현우 |
| 개발 환경 | Linux 기준 |

```bash
ruff check .
black .
```

---

---

## 주제 전환 마이그레이션

`domain/` 코드는 아직 이전 주제(암실 반출) 기준입니다. 재작성은
[`docs/design/class_diagram.md` §5](docs/design/class_diagram.md) 의 **PR 10건 순서를 반드시 따르세요.**

```
값 객체 → 포트 → Fake 어댑터 → states.py → 실제 어댑터 → 테스트
```

Fake 어댑터가 `states.py` 보다 먼저 들어가지 않으면 재작성 내내 CI가 빨간불입니다.

## 이슈 · PR

- 이슈 템플릿: `.github/ISSUE_TEMPLATE/` (task · bug)
- PR 본문에 `Closes #N` 표기
- `.github/CODEOWNERS` 가 경로별 리뷰어를 자동 지정합니다

## 문서

- `docs/` 하위는 **snake_case**
- FSM 전이는 [`docs/design/state_machine.md`](docs/design/state_machine.md) 가 **단일 소스** — 다른 문서에 중복 정의하지 마세요
- 설계 변경 시 README 문서 지도와 해당 docs 문서를 함께 갱신

