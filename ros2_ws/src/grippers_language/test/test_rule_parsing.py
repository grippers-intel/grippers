"""rule_parsing.py 순수 로직 테스트. rclpy/anthropic 없이도 돈다 — Claude API
호출은 claude_rule_parser.py가 전담하고 여기는 그 결과를 검증·정규화하는
로직만 본다(grippers_perception의 hailo_scan_mapping.py와 같은 이유)."""

from grippers_language.rule_parsing import (
    confirm_phrase_text,
    normalize_rules,
    parse_via_keywords,
)


def test_normalize_rules_maps_spec_example_to_domain_classes():
    """확정 미션 명세서 예문 그대로 — 체스는 왼쪽, 장난감은 오른쪽."""
    raw_rules = [
        {"classes": ["knight", "queen", "rook"], "dest": "left"},
        {"classes": ["box", "soccer", "star"], "dest": "right"},
    ]

    assert normalize_rules(raw_rules) == {"CHESS_PIECE": "LEFT", "GABE": "RIGHT"}


def test_normalize_rules_rejects_mixed_classes_in_one_rule():
    """한 규칙에 체스 기물과 장난감이 섞여 있으면 불분명한 명령으로 접는다."""
    raw_rules = [{"classes": ["knight", "soccer"], "dest": "left"}]

    assert normalize_rules(raw_rules) is None


def test_normalize_rules_rejects_contradictory_duplicate_rules():
    """같은 domain 클래스에 서로 다른 목적지를 지정하면 모순으로 접는다."""
    raw_rules = [
        {"classes": ["knight"], "dest": "left"},
        {"classes": ["queen"], "dest": "right"},
    ]

    assert normalize_rules(raw_rules) is None


def test_normalize_rules_allows_consistent_duplicate_rules():
    """같은 domain 클래스를 두 규칙이 같은 목적지로 가리키면 문제없다."""
    raw_rules = [
        {"classes": ["knight"], "dest": "left"},
        {"classes": ["queen"], "dest": "left"},
    ]

    assert normalize_rules(raw_rules) == {"CHESS_PIECE": "LEFT"}


def test_normalize_rules_rejects_unknown_class_name():
    assert normalize_rules([{"classes": ["dragon"], "dest": "left"}]) is None


def test_normalize_rules_rejects_unknown_dest():
    assert normalize_rules([{"classes": ["knight"], "dest": "up"}]) is None


def test_normalize_rules_rejects_empty_classes():
    assert normalize_rules([{"classes": [], "dest": "left"}]) is None


def test_normalize_rules_returns_none_for_empty_rule_list():
    assert normalize_rules([]) is None


def test_confirm_phrase_text_lists_each_rule():
    phrase = confirm_phrase_text({"CHESS_PIECE": "LEFT", "GABE": "RIGHT"})

    assert "체스 기물은 왼쪽 바구니에" in phrase
    assert "장난감은 오른쪽 바구니에" in phrase


def test_parse_via_keywords_matches_spec_example():
    rules = parse_via_keywords("모든 체스 기물을 왼쪽 박스에, 장난감들은 오른쪽 박스에 정리해주세요")

    assert normalize_rules(rules) == {"CHESS_PIECE": "LEFT", "GABE": "RIGHT"}


def test_parse_via_keywords_matches_reversed_example():
    """같은 물체 구성이라도 명령이 다르면 다르게 행동한다(명세서 핵심 성질)."""
    rules = parse_via_keywords("체스 기물은 오른쪽 박스에, 장난감은 왼쪽 박스에 정리해주세요")

    assert normalize_rules(rules) == {"CHESS_PIECE": "RIGHT", "GABE": "LEFT"}


def test_parse_via_keywords_handles_partial_command():
    """한 범주만 언급한 명령은 그 범주만 규칙으로 만든다."""
    rules = parse_via_keywords("체스 기물만 왼쪽 박스에 정리해줘")

    assert normalize_rules(rules) == {"CHESS_PIECE": "LEFT"}


def test_parse_via_keywords_returns_none_for_unrelated_text():
    assert parse_via_keywords("오늘 날씨 어때") is None


def test_parse_via_keywords_returns_none_for_ambiguous_clause():
    """한 구절에 체스와 장난감이 같이 언급되면 그 구절에서는 규칙을 만들지
    않는다 — 결과적으로 규칙이 하나도 없으면 전체가 None이다."""
    assert parse_via_keywords("체스 기물과 장난감을 왼쪽에 정리해줘") is None
