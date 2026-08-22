"""Claude API로 자연어 명령을 미션 규칙(JSON)으로 구조화 출력한다.

미션 명세서(2026-08-23) 파이프라인 01번 — "Claude API·구조화 출력". LLM은
여기서만 쓰이고 제어 루프에는 관여하지 않는다.

⚠️ 시연장 네트워크를 못 믿는다(명세서 원문). 그래서 이 모듈의 모든 실패
경로는 예외를 올리지 않고 **`None`**을 반환한다 — 호출자(language_node.py)가
`rule_parsing.parse_via_keywords()` 키워드 폴백으로 넘어가야 한다는 신호다.
다른 관측 포트의 "모르면 실패" 관례와 같다.

rclpy는 안 쓰지만 `anthropic` 패키지가 있어야 실제로 동작한다 — 없으면
호출 전에 바로 `None`(호출부가 즉시 폴백한다)."""

import json

from grippers_perception.cpu_yolo_scan_mapping import CPU_YOLO_CLASS_NAMES

try:
    import anthropic

    _ANTHROPIC_AVAILABLE = True
except ImportError:
    _ANTHROPIC_AVAILABLE = False

# claude-api 스킬 기본값을 따른다 — 사용자가 다른 모델을 명시하지 않는 한
# claude-opus-5를 쓴다.
MODEL_DEFAULT = "claude-opus-5"
# 시연장에서 응답 없는 네트워크를 오래 붙잡지 않는다 — 명세서가 "네트워크를
# 못 믿는다"고 못박은 이유가 바로 이거다. 8초 안에 답이 없으면 폴백으로
# 넘어가는 게, 무대 위에서 몇십 초 침묵하는 것보다 낫다.
REQUEST_TIMEOUT_SEC = 8.0
MAX_TOKENS = 1024

RULES_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "rules": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "classes": {
                        "type": "array",
                        "items": {"type": "string", "enum": list(CPU_YOLO_CLASS_NAMES)},
                    },
                    "dest": {"type": "string", "enum": ["left", "right"]},
                },
                "required": ["classes", "dest"],
                "additionalProperties": False,
            },
        }
    },
    "required": ["rules"],
    "additionalProperties": False,
}

SYSTEM_PROMPT = f"""당신은 로봇 팔·베이스 미션 시스템의 명령 해석기입니다.
사용자가 한국어로 정리 명령을 내리면, 그 명령을 규칙 목록으로 바꾸세요.

로봇 카메라가 인식하는 물체 클래스는 정확히 이 6종뿐입니다:
{", ".join(CPU_YOLO_CLASS_NAMES)}

"체스 기물"/"체스말"처럼 상위 범주로 말하면 knight, queen, rook 전부로
펼치세요. "장난감"처럼 말하면 box, soccer, star 전부로 펼치세요. 목적지는
"left" 또는 "right"만 있습니다(바구니는 2개, 왼쪽/오른쪽뿐입니다).

한 규칙의 classes는 전부 같은 상위 범주(체스 기물 전부이거나 장난감
전부)여야 합니다 — 섞지 마세요.

예시 — "모든 체스 기물을 왼쪽 박스에, 장난감들은 오른쪽 박스에
정리해주세요":
{{"rules": [
  {{"classes": ["knight", "queen", "rook"], "dest": "left"}},
  {{"classes": ["box", "soccer", "star"], "dest": "right"}}
]}}

명령이 언급하지 않은 범주는 규칙에 넣지 마세요 — 언급된 것만 규칙으로
만듭니다."""


def parse_via_claude(client, text, model=MODEL_DEFAULT):
    """`text`를 Claude 구조화 출력으로 파싱해 원시 규칙 목록
    (`rule_parsing.normalize_rules()`가 받는 형태)을 반환한다.

    `client`가 `None`이거나(anthropic 미설치·초기화 실패) 요청이 어떤
    이유로든(네트워크·인증·레이트리밋·타임아웃·응답 형식 이상) 실패하면
    **`None`** — 원인을 구분하지 않는다. 시연 중엔 "왜 실패했는지"보다
    "즉시 폴백하는지"가 더 중요하다."""
    if client is None or not _ANTHROPIC_AVAILABLE:
        return None

    try:
        response = client.with_options(timeout=REQUEST_TIMEOUT_SEC).messages.create(
            model=model,
            max_tokens=MAX_TOKENS,
            system=SYSTEM_PROMPT,
            messages=[{"role": "user", "content": text}],
            output_config={"format": {"type": "json_schema", "schema": RULES_JSON_SCHEMA}},
        )
    except Exception:  # noqa: BLE001 -- 네트워크/인증/레이트리밋 등 원인 불문 폴백
        return None

    try:
        text_block = next(b.text for b in response.content if b.type == "text")
        return json.loads(text_block)["rules"]
    except (StopIteration, KeyError, ValueError, TypeError):
        return None


def build_client():
    """anthropic 클라이언트를 만든다. 미설치·초기화 실패 시 **`None`** —
    호출자는 이걸 곧장 `parse_via_claude(client=None, ...)`에 넘기면 되고,
    그러면 즉시 폴백 경로를 탄다(별도 분기가 필요 없다)."""
    if not _ANTHROPIC_AVAILABLE:
        return None
    try:
        return anthropic.Anthropic()
    except Exception:  # noqa: BLE001 -- 자격증명 부재 등 다양한 초기화 실패를 전부 접는다
        return None
