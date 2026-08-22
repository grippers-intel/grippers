"""Ros2CommandInterpreter — mission_orchestrator가 쓰는 CommandInterpreter 포트 구현
(class_diagram.md §2의 LanguageAdapter). language 노드에 서비스로 말을 건다.

placement_rule(dict[ObjectClass, Destination])은 MissionSpec.msg에서 병렬 배열
2개로 평탄화돼 있다 — 여기서 dict ↔ 배열 왕복 변환을 전담한다.

⚠️ 2026-08-23: domain.values.BoxColor가 Destination(LEFT/RIGHT)으로 바뀌었지만,
MissionSpec.msg의 배열 필드명은 아직 `placement_colors`다 — .msg 필드명을
바꾸는 건 인터페이스 재빌드가 필요한 별도 변경이라 이번 범위 밖에 둔다.
같은 문자열 배열에 Destination의 이름을 담는 것으로만 맞춘다."""

from grippers_interfaces.msg import MissionSpec as RosMissionSpec
from grippers_interfaces.srv import ConfirmPhrase, Parse

from domain.adapters.real._ros_call import call_service
from domain.ports.command_interpreter import CommandInterpreter
from domain.values import Destination, MissionMode, MissionSpec, ObjectClass

# 복창 문구를 받지 못했을 때의 값. 보고가 누락될 뿐 미션은 계속된다 —
# 복창은 사용자에게 들려주는 확인 문구이지 전이 조건이 아니다.
NO_CONFIRM_PHRASE = ""


def _mission_spec_from_msg(msg) -> MissionSpec:
    placement_rule = {
        ObjectClass[cls]: Destination[dest]
        for cls, dest in zip(msg.placement_classes, msg.placement_colors, strict=True)
    }
    return MissionSpec(
        mode=MissionMode[msg.mode],
        target_cls=ObjectClass[msg.target_cls] if msg.target_cls else None,
        placement_rule=placement_rule,
        raw_text=msg.raw_text,
    )


def _mission_spec_to_msg(spec: MissionSpec) -> RosMissionSpec:
    return RosMissionSpec(
        mode=spec.mode.name,
        target_cls=spec.target_cls.name if spec.target_cls is not None else "",
        placement_classes=[cls.name for cls in spec.placement_rule],
        placement_colors=[dest.name for dest in spec.placement_rule.values()],
        raw_text=spec.raw_text,
    )


class Ros2CommandInterpreter(CommandInterpreter):
    def __init__(self, node):
        self._node = node
        self._parse_client = node.create_client(Parse, "language/parse")
        self._confirm_client = node.create_client(ConfirmPhrase, "language/confirm_phrase")

    def parse(self, text: str) -> MissionSpec | None:
        """해석하지 못한 문장이면 None을 반환한다 (Parse.srv의 understood 규약,
        Ros2Perception.find_box()가 found를 다루는 방식과 같다).

        반환 타입이 포트 ABC의 선언(-> MissionSpec)보다 넓은데, 의도된 과도기다.
        포트 시그니처와 ScriptedInterpreter(현재 미등록 문형에 ValueError)와
        IdleState의 None 처리는 셋이 원자적으로 움직여야 해서 후속 rename PR에서
        한꺼번에 맞춘다. 여기서 먼저 고치는 이유는 .srv를 나중에 바꾸면
        인터페이스 재빌드와 language 노드 재구현이 따라오기 때문이다.

        서비스가 없거나 응답이 없을 때도 **None** 이다 — 해석하지 못한 문장과
        같은 취급이라 `IDLE` 이 대기를 유지한다. 명령을 못 알아들은 채 미션을
        시작하는 것보다 낫다."""
        res = call_service(self._node, self._parse_client, Parse.Request(text=text), label="parse")
        if res is None or not res.understood:
            return None
        return _mission_spec_from_msg(res.spec)

    def confirm_phrase(self, spec: MissionSpec) -> str:
        """복창 문구. 서비스가 없거나 응답이 없으면 **`NO_CONFIRM_PHRASE`(빈 문자열)** —
        복창만 누락되고 미션은 그대로 진행된다."""
        req = ConfirmPhrase.Request(spec=_mission_spec_to_msg(spec))
        res = call_service(self._node, self._confirm_client, req, label="confirm_phrase")
        if res is None:
            return NO_CONFIRM_PHRASE
        return res.phrase
