"""CommandInterpreter 포트 — ROS2를 전혀 모르는 순수 ABC.
grippers_vla의 LanguageAdapter가 이걸 구현한다.

이전 주제에서 명령은 미션 시작 시 한 번 들어오는 입력이라 포트가 필요 없었지만,
새 주제에서는 **자연어가 미션 도중에도 `placement_rule` 을 실제로 바꾼다** —
이게 이 포트가 신설된 이유다 (docs/design/class_diagram.md §2 '포트가 4종이 된 이유').

    "체스말은 검은 상자에 넣어줘"
        → placement_rule[ObjectClass.CHESS_PIECE] = BoxColor.BLACK

미션 파라미터를 바꾸는 것은 도메인 로직이므로 포트 뒤에 있어야 하고,
`ScriptedInterpreter` 로 Fake 대체가 되어야 CI에서 명령 문형 회귀 테스트가 돌아간다."""

from abc import ABC, abstractmethod

from domain.values import MissionSpec


class CommandInterpreter(ABC):
    @abstractmethod
    def parse(self, text: str) -> MissionSpec:
        """자연어 명령 text를 MissionSpec으로 해석한다.

        **해석하지 못했거나 서비스가 응답하지 않으면 `None`** — 예외를 올리지
        않는다. 실패를 예외로 표현하면 루프 FSM이 흡수할 수 없고 미션 스레드가
        그대로 죽는다. `IDLE` 이 대기를 유지하는 편이 명령을 못 알아들은 채로
        미션을 시작하는 것보다 낫다. (반환 타입 선언은 아직 `MissionSpec` 이다 —
        시그니처는 후속 PR에서 한꺼번에 맞춘다.)"""

    @abstractmethod
    def confirm_phrase(self, spec: MissionSpec) -> str:
        """spec을 사람이 확인할 수 있는 복창 문구로 변환한다.

        **실패는 빈 문자열.** 복창만 누락되고 미션은 그대로 진행된다 — 복창은
        사용자에게 들려주는 확인 문구이지 전이 조건이 아니다."""
