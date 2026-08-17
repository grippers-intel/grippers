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
        """자연어 명령 text를 MissionSpec으로 해석한다."""

    @abstractmethod
    def confirm_phrase(self, spec: MissionSpec) -> str:
        """spec을 사람이 확인할 수 있는 복창 문구로 변환한다."""
