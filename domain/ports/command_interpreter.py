"""CommandInterpreter 포트 — 신규 (주제 전환으로 추가된 4번째 포트).
Real: `LanguageAdapter` · Fake: `ScriptedInterpreter`

⚠️ Tier-1 freeze 대상 (#97).

자연어가 **미션 파라미터를 실제로 바꾸기 때문에** 포트다.

    "체스말은 검은 상자에 넣어줘"  →  placement_rule[ObjectClass.CHESS] = BoxColor.BLACK

음성은 포트가 아니다 — `voice_io` 노드가 STT 결과를 기존 명령 토픽에 텍스트로
발행할 뿐이고, 도메인 코드는 0줄 바뀐다.
"""

from abc import ABC, abstractmethod

from domain.values import MissionSpec


class CommandInterpreter(ABC):
    @abstractmethod
    def parse(self, text: str) -> MissionSpec | None:
        """명령 텍스트 1건을 `MissionSpec` 으로. 해석 불가면 None.

        None 은 실패가 아니라 "실행하지 않는다"이다 — STT 오인식이 확인 없이
        실행되는 경로를 만들지 않기 위해, 애매하면 None 을 반환한다 (오실행률 목표 0%).
        """

    @abstractmethod
    def confirm_phrase(self, spec: MissionSpec) -> str:
        """실행 전 사용자에게 복창할 문장. TTS · HUD 가 그대로 쓴다.

        확인 절차가 도메인 계약인 이유 — 확인 없는 실행 경로를 만들지 않는다.
        """
