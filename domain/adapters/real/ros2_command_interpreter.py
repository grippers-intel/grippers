"""Ros2CommandInterpreter — mission_orchestrator가 쓰는 CommandInterpreter 포트 구현
(class_diagram.md §2의 LanguageAdapter). language 노드에 서비스로 말을 건다.

placement_rule(dict[ObjectClass, BoxColor])은 MissionSpec.msg에서 병렬 배열
2개로 평탄화돼 있다 — 여기서 dict ↔ 배열 왕복 변환을 전담한다."""

import rclpy
from grippers_interfaces.msg import MissionSpec as RosMissionSpec
from grippers_interfaces.srv import ConfirmPhrase, Parse

from domain.ports.command_interpreter import CommandInterpreter
from domain.values import BoxColor, MissionMode, MissionSpec, ObjectClass


def _mission_spec_from_msg(msg) -> MissionSpec:
    placement_rule = {
        ObjectClass[cls]: BoxColor[color]
        for cls, color in zip(msg.placement_classes, msg.placement_colors, strict=True)
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
        placement_colors=[color.name for color in spec.placement_rule.values()],
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
        인터페이스 재빌드와 language 노드 재구현이 따라오기 때문이다."""
        self._parse_client.wait_for_service()
        future = self._parse_client.call_async(Parse.Request(text=text))
        rclpy.spin_until_future_complete(self._node, future)
        res = future.result()
        if not res.understood:
            return None
        return _mission_spec_from_msg(res.spec)

    def confirm_phrase(self, spec: MissionSpec) -> str:
        self._confirm_client.wait_for_service()
        req = ConfirmPhrase.Request(spec=_mission_spec_to_msg(spec))
        future = self._confirm_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().phrase
