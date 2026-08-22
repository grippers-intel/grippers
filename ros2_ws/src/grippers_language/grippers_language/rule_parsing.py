"""자연어 명령 → 미션 규칙 파싱의 순수 로직 (검증·정규화·복창 문구).

미션 명세서(2026-08-23) 파이프라인 01번 — "채팅 입력을 미션 스펙으로 바꾼다.
대상 클래스와 목적지를 규칙 목록으로 뽑고, toy 같은 상위 범주를 개별 클래스로
펼친다. LLM은 여기서만 쓰이고 제어 루프에는 관여하지 않는다. 시연장 네트워크를
못 믿으므로 키워드 폴백을 함께 둔다."

이 파일은 rclpy도 anthropic도 import하지 않는다 — Claude API 호출 자체는
claude_rule_parser.py가 전담하고, 여기는 그 결과(구조화 출력이든 키워드
폴백이든)를 검증·정규화하는 로직만 담아 rclpy·네트워크 없이 pytest로
검증한다(grippers_perception의 hailo_scan_mapping.py와 같은 이유).

양쪽 파서가 공통으로 만들어야 하는 "원시 규칙" 형태는 명세서 원문 그대로다:
    {"rules": [{"classes": ["knight", "queen", "rook"], "dest": "left"}, ...]}
"""

from grippers_perception.cpu_yolo_scan_mapping import (
    CPU_YOLO_CLASS_NAMES,
    object_class_for_cpu_yolo_class_name,
)

KNOWN_DESTS = ("left", "right")

# 명세서 예문의 대표 클래스 — "체스 기물"/"장난감"처럼 상위 범주로 말한
# 명령을 원시 클래스 목록으로 펼칠 때 쓴다(파이프라인 01번 "toy 같은 상위
# 범주를 개별 클래스로 펼친다"). 셋 다 CPU_YOLO_CLASS_TO_OBJECT_CLASS에서
# 각각 CHESS_PIECE/GABE로 접히므로, 여기서 어떤 그룹으로 펼치는지는 결과에
# 영향을 주지 않는다 — 어느 대표 원소를 넣어도 normalize_rules()가 같은
# domain 클래스로 접는다.
CHESS_PIECE_CLASSES = ("knight", "queen", "rook")
GABE_CLASSES = ("box", "soccer", "star")

# 키워드 폴백(부록)이 쓰는 한국어 어휘. Claude 구조화 출력 프롬프트도 같은
# 어휘를 예시로 든다 — 실제 판단 기준은 여기 이 표 하나다.
_CHESS_KEYWORDS = ("체스", "기물", "나이트", "퀸", "룩", "chess")
# ⚠️ "박스"는 여기 넣지 않는다 — 명세서 예문 자체가 "왼쪽 박스에/오른쪽
# 박스에"처럼 "박스"를 목적지(바구니)를 가리키는 일반 명사로 쓴다. YOLO의
# box(장난감) 서브클래스와 이름이 같아서 넣으면 "체스 기물만 왼쪽 박스에
# 정리해줘"류 문장까지 장난감 언급으로 오판해 전부 불분명 처리된다.
_GABE_KEYWORDS = ("장난감", "가베", "축구공", "스타", "toy")
_LEFT_KEYWORDS = ("왼쪽", "좌측", "left")
_RIGHT_KEYWORDS = ("오른쪽", "우측", "right")

_DEST_LABEL_KO = {"LEFT": "왼쪽", "RIGHT": "오른쪽"}
# 조사("은")까지 미리 붙여둔다 — 둘 다 받침 있는 명사(물/감)라 "은"으로
# 고정해도 맞지만, 새 domain 클래스가 추가되면 이 표를 새로 확인해야 한다.
_OBJECT_CLASS_PHRASE_KO = {"CHESS_PIECE": "체스 기물은", "GABE": "장난감은"}


def normalize_rules(raw_rules):
    """원시 규칙 목록을 domain `ObjectClass` 이름("GABE"/"CHESS_PIECE")을 키로,
    `Destination` 이름("LEFT"/"RIGHT")을 값으로 하는 placement_rule dict로
    정규화한다.

    다음 중 하나라도 있으면 **`None`** — 호출자가 "이해 못함"으로 접어야
    한다는 신호다(다른 관측/해석 포트와 같은 "모르면 실패" 관례):
    - `classes`가 비어 있음
    - 알 수 없는 클래스 이름(YOLO 6종 어휘 밖) 또는 매핑 안 되는 클래스
    - `dest`가 "left"/"right"가 아님
    - 한 규칙 안에 CHESS_PIECE와 GABE 원시 클래스가 섞여 있음 — 명세서의
      "toy 클래스는 미션 계층에서 처리한다" 원칙상 한 규칙은 항상 단일
      domain 클래스만 가리켜야 한다(뒤섞이면 명령이 불분명하다는 뜻)
    - 같은 domain 클래스에 서로 다른 목적지를 지정하는 규칙이 두 개 이상
      있음(모순된 명령 — "체스는 왼쪽... 체스는 오른쪽")
    """
    placement_rule = {}
    for rule in raw_rules:
        classes = rule.get("classes") or []
        dest = rule.get("dest")
        if not classes or dest not in KNOWN_DESTS:
            return None

        object_classes = set()
        for class_name in classes:
            if class_name not in CPU_YOLO_CLASS_NAMES:
                return None
            object_class = object_class_for_cpu_yolo_class_name(class_name)
            if object_class is None:
                return None
            object_classes.add(object_class)

        if len(object_classes) != 1:
            return None
        (object_class,) = object_classes
        dest_name = dest.upper()

        if object_class in placement_rule and placement_rule[object_class] != dest_name:
            return None
        placement_rule[object_class] = dest_name

    return placement_rule if placement_rule else None


def confirm_phrase_text(placement_rule):
    """`placement_rule`(normalize_rules()의 결과)을 사람이 들을 복창 문구로
    바꾼다. 실패는 이 함수의 책임이 아니다 — `confirm_phrase` 포트 계약상
    실패는 빈 문자열이고, 그건 호출자(language_node.py)가 서비스 자체가
    죽었을 때만 쓴다."""
    parts = [
        f"{_OBJECT_CLASS_PHRASE_KO.get(cls, cls)} {_DEST_LABEL_KO.get(dest, dest)} 바구니에"
        for cls, dest in placement_rule.items()
    ]
    return ", ".join(parts) + " 정리할게요"


def _clause_rule(clause):
    """문장 한 조각(쉼표로 나눈 한 구절)에서 규칙 하나를 뽑는다. 클래스
    그룹과 목적지 둘 다 언급돼야 하고, 그룹이 두 개 다 언급되면(문장이
    뒤섞여 있어 어느 쪽 목적지인지 불분명) 규칙을 만들지 않는다 — "모르면
    포함하지 않는다"가 이 함수의 원칙이다."""
    mentions_chess = any(kw in clause for kw in _CHESS_KEYWORDS)
    mentions_gabe = any(kw in clause for kw in _GABE_KEYWORDS)
    if mentions_chess == mentions_gabe:  # 둘 다 언급 또는 둘 다 없음 — 불분명
        return None

    mentions_left = any(kw in clause for kw in _LEFT_KEYWORDS)
    mentions_right = any(kw in clause for kw in _RIGHT_KEYWORDS)
    if mentions_left == mentions_right:  # 둘 다 언급 또는 둘 다 없음 — 불분명
        return None

    classes = list(CHESS_PIECE_CLASSES) if mentions_chess else list(GABE_CLASSES)
    dest = "left" if mentions_left else "right"
    return {"classes": classes, "dest": dest}


def parse_via_keywords(text):
    """Claude API를 못 믿을 때(네트워크 부재·타임아웃·구조화 출력 검증
    실패) 쓰는 키워드 기반 폴백. 명세서 원문 — "시연장 네트워크를 못
    믿으므로 키워드 폴백을 함께 둔다."

    문장을 쉼표로 조각내 조각마다 규칙을 뽑는다 — 명세서 예문("모든 체스
    기물을 왼쪽 박스에, 장난감들은 오른쪽 박스에 정리해주세요")이 정확히
    이 구조다. 규칙을 하나도 못 뽑으면 **`None`**."""
    clauses = [c.strip() for c in text.replace("，", ",").split(",") if c.strip()]
    rules = [rule for clause in clauses if (rule := _clause_rule(clause)) is not None]
    return rules or None
