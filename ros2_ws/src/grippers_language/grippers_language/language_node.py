"""language_node — 채팅 명령을 미션 규칙으로 해석한다 (미션 명세서 파이프라인 01번).

Ros2CommandInterpreter(domain/adapters/real/ros2_command_interpreter.py)가
`language/parse`·`language/confirm_phrase` 서비스로 이 노드에 말을 건다.

⚠️ 아직 실기로 검증 안 됨 — Pi가 이번 세션 내내 연결 안 된 상태라 이 노드
자체를 ROS2 환경에서 띄워 본 적이 없다. 순수 로직(rule_parsing.py)만
pytest로 검증했다. 연결되면 다음을 확인할 것:
- `ANTHROPIC_API_KEY`(또는 `ant auth login` 프로필)가 실제로 이 프로세스
  환경에 보이는지
- Claude 실패 → 키워드 폴백 전환이 실제 지연 없이 일어나는지(시연장
  네트워크 조건에서 REQUEST_TIMEOUT_SEC=8초가 적절한지)
"""

import rclpy
from grippers_interfaces.msg import MissionSpec as RosMissionSpec
from grippers_interfaces.srv import ConfirmPhrase, Parse
from rclpy.node import Node

from grippers_language.claude_rule_parser import build_client, parse_via_claude
from grippers_language.rule_parsing import confirm_phrase_text, normalize_rules, parse_via_keywords

CLAUDE_MODEL_DEFAULT = "claude-opus-5"


class LanguageNode(Node):
    def __init__(self):
        super().__init__("language_node")

        self.declare_parameter("claude_enabled", True)
        self.declare_parameter("claude_model", CLAUDE_MODEL_DEFAULT)

        self._claude_client = build_client() if self.get_parameter("claude_enabled").value else None
        backend_state = "Claude API + 키워드 폴백" if self._claude_client is not None else "키워드 폴백만"

        self.create_service(Parse, "language/parse", self._on_parse)
        self.create_service(ConfirmPhrase, "language/confirm_phrase", self._on_confirm_phrase)

        self.get_logger().info(f"language_node ready ({backend_state})")

    def _on_parse(self, request, response):
        raw_rules = None
        if self._claude_client is not None:
            model = self.get_parameter("claude_model").value
            raw_rules = parse_via_claude(self._claude_client, request.text, model=model)
            if raw_rules is None:
                self.get_logger().warn("parse: Claude 실패 — 키워드 폴백으로 전환")

        if raw_rules is None:
            raw_rules = parse_via_keywords(request.text)

        placement_rule = normalize_rules(raw_rules) if raw_rules is not None else None
        if placement_rule is None:
            self.get_logger().warn(f"parse(text={request.text!r}): 이해 못함 — understood=False 반환")
            response.understood = False
            response.spec = RosMissionSpec()
            return response

        response.understood = True
        # TIDY만 만든다 — FETCH는 2026-08-23 확정 미션 명세서엔 없는 이전
        # 주제("암실 반출") 잔재라 여기서 새로 만들지 않는다
        # (domain/adapters/fake/scripted_interpreter.py 상단 경고 참고).
        response.spec = RosMissionSpec(
            mode="TIDY",
            target_cls="",
            placement_classes=list(placement_rule.keys()),
            placement_colors=list(placement_rule.values()),
            raw_text=request.text,
        )
        self.get_logger().info(f"parse(text={request.text!r}): {placement_rule}")
        return response

    def _on_confirm_phrase(self, request, response):
        placement_rule = dict(
            zip(request.spec.placement_classes, request.spec.placement_colors, strict=True)
        )
        response.phrase = confirm_phrase_text(placement_rule) if placement_rule else ""
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LanguageNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
