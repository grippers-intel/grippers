# 문서

Grippers 최종 시스템(코드: [`vla_robot/`](../vla_robot/)) 문서입니다. 처음이면 **[handover.md](handover.md)** 한 장부터 보십시오.

## 현재 시스템

| 문서 | 내용 |
|---|---|
| [handover.md](handover.md) | **한 장 요약** — 무엇이 되나 · 하루 순서 · 설정 · 알려진 한계 · 다음 단계 |
| [behavior.md](behavior.md) | 주행·미션 동작과 그것을 정하는 설정 키 |
| [pi_deploy.md](pi_deploy.md) | Pi 배포 · 기동 · 하루 마무리(안전 종료) · 무응답 대처 · 배터리 측정 |
| [map_rebuild.md](map_rebuild.md) | 장판·마커를 옮겼을 때 맵 다시 만들기 |
| [map_rebuild.md](map_rebuild.md) 의 표 | **현재 작업장 배치**(REV.2) — 장판 · 마커 · 바구니 · 카메라 |
| [layout/workspace_layout.md](layout/workspace_layout.md) | 옛 세트장(REV.1, 09-05) 도면 기록 — 지금 배치와 다름 |
| [../vla_robot/README.md](../vla_robot/README.md) | 구성 · 디렉터리 · 설치 · 캘리브레이션 · 첫 기동 |
| [../vla_robot/host/README.md](../vla_robot/host/README.md) | Host(노트북) 실행 · 도구 |

## 설계

| 문서 | 내용 |
|---|---|
| [architecture.md](architecture.md) | **시스템 구성** — 역할 분담(Host · Pi) · 구성도 · 어디서 무엇이 도나 · 한 사이클 · 설계 원칙 · 설정 파일 |
| [interfaces.md](interfaces.md) | **인터페이스** — Host ↔ Pi UDP(HostCommand · PiStatus · 완료 보고 방식) · ROS 2 노드 · 토픽 · 서비스 · 액션 |
| [modules.md](modules.md) | **폴더 · 모듈** — 저장소 구조 · 의존 관계 · 모듈별 역할 · 시험 |
| [state_machine.md](state_machine.md) | **상태도** — Host 미션 상태 8개 · 전이 조건 · 겹쳐 도는 감시(ESTOP · 무응답 · 폭주) · Pi 쪽 |
| [sequences.md](sequences.md) | **시퀀스** — 정리 한 기물 · 가져와(손에 건네기) · 차체 무응답 복구 |
| [decisions.md](decisions.md) | **결정 기록** — 무엇을 왜 정했고 무엇을 뺐나(구조 · 인식 · 파지 · 주행 · 운영) |
| [results.md](results.md) | **결과 · 알려진 한계** — 실기 성공률 · 파지 · 자원(CPU · 전력) · 한계 |

## 운영

| 문서 | 내용 |
|---|---|
| [ops/purchase_ledger.md](ops/purchase_ledger.md) | 구매 장부 운영 규칙 |

## 초기 설계 (기록용)

2026년 8월 초기 설계의 코드와 문서는 [`legacy/`](../legacy/) 에 그대로 있습니다(지금 코드와 다름). 지금도 읽을 만한 것:

| 문서 | 내용 |
|---|---|
| [measurements.md](../legacy/docs/ops/measurements.md) | 측정 원칙(시행 횟수 · 신뢰구간 · 조건 통제)과 M1 하드웨어 실측 |
| [failure_definition.md](../legacy/docs/ops/failure_definition.md) | 무엇을 실패로 셀 것인가 — 계수 규칙 |
| [rejected_designs.md](../legacy/docs/ops/rejected_designs.md) | 판단해서 뺀 설계와 근거 |
