# legacy — 초기 설계 코드 (기록용)

2026년 8월 팀이 설계·작성한 첫 구조입니다. **최종 시스템은 [`vla_robot/`](../vla_robot/)** 이고(탑뷰 ArUco + 메카넘 + ACT),
여기 코드는 더 이상 실행하거나 고치지 않습니다. 2026-10-13 저장소 정리 때 루트에서 이 폴더로 옮겼습니다.

| 경로 | 내용 (원래 위치) |
|---|---|
| `domain/` | 육각형 구조 미션 FSM — 값 객체 · 포트 · Fake/real 어댑터 (`domain/`) |
| `tests/` | 위 코드의 하드웨어 없는 시험 — CI 의 "Legacy unit tests" 가 계속 돌린다 (`tests/`) |
| `ros2_ws/src/grippers_*` | 초기 설계의 ROS 2 패키지 7개 (`ros2_ws/src/grippers_*`) |
| `tools/` | 바닥 호모그래피 실측(a2) · Hailo DFC 환경 · 팔 실기 시험 스크립트 (`tools/`) |

- 폴더 안의 상대 위치는 원래 그대로라, 시험이 쓰는 경로(`legacy/tests` → `legacy/domain`, `legacy/ros2_ws/...`)가 그대로 맞습니다.
- 커밋 기록은 git 에 그대로 남아 있습니다(`git log --follow legacy/domain/...`).
- 루트에 남은 `ros2_ws/src` 의 다른 패키지 · `docker/` · `third_party/` 는 **MentorPi 제조사 · 외부 원본**이라 옮기지 않았습니다
  (Pi 컨테이너가 같은 제조사 패키지를 지금도 쓴다).
