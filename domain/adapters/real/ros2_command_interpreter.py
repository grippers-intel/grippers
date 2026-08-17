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

    def parse(self, text: str) -> MissionSpec:
        self._parse_client.wait_for_service()
        future = self._parse_client.call_async(Parse.Request(text=text))
        rclpy.spin_until_future_complete(self._node, future)
        return _mission_spec_from_msg(future.result().spec)

    def confirm_phrase(self, spec: MissionSpec) -> str:
        self._confirm_client.wait_for_service()
        req = ConfirmPhrase.Request(spec=_mission_spec_to_msg(spec))
        future = self._confirm_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().phrase
