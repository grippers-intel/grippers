"""포트(ABC) 계층 — ROS2를 전혀 모르는 순수 인터페이스.

**실패를 어떻게 표현하는가도 포트 계약이다.** 각 메서드 docstring에 실패 시
반환값을 명시한다. 계약이 코드 어디에도 적혀 있지 않아 Fake와 real 구현이
서로 다른 표현을 쓰다가 두 번 사고가 났다:

- `ScriptedInterpreter.parse()` 는 `ValueError` 를 올리는데
  `Ros2CommandInterpreter.parse()` 는 `None` 을 반환했다 (PR #9 리뷰 B항)
- `FakeArm.get_load()` 는 0~1 정규화 비율을, `Ros2ArmDriver.get_load()` 는
  서보 원시값(0~1023)을 돌려줬다 (PR #136)

둘 다 CI는 통과하고 실기에서만 깨지는 종류다 — Fake로 도는 도메인 테스트는
Fake의 표현만 보기 때문이다.

**실패는 예외가 아니라 값이다.** 루프 FSM은 실패를 미션 종료가 아니라 보류
등록 후 `SCAN` 복귀로 흡수하므로(docs/design/state_machine.md §3), 어댑터가
예외를 던지면 그 흡수 경로를 건너뛰고 미션 스레드가 죽는다. 서비스 부재·응답
없음·인식 실패는 전부 각 포트가 정한 실패값으로 돌아와야 한다.
"""
