"""ScriptedInterpreter — CommandInterpreter 포트의 테스트 더블.
실제 언어모델/파서 대신 정해진 문형 목록(`_table`)을 조회해 MissionSpec을 반환한다 —
`ScriptedInterpreter` 로 Fake 대체가 되어야 CI에서 명령 문형 회귀 테스트가 돌아간다
(docs/design/class_diagram.md §2 '포트가 4종이 된 이유').

기본 배치 규칙은 docs/subsystems/objects.md 확정값을 따른다:
GABE → GREEN, CHESS_PIECE → BLUE. BLACK/RED는 방해 선택지로 기본은 비어 있고,
"체스말은 검은 상자에" 류의 규칙 변경 문형이 들어와야 채워진다."""

from dataclasses import replace

from domain.ports.command_interpreter import CommandInterpreter
from domain.values import BoxColor, MissionMode, MissionSpec, ObjectClass

DEFAULT_PLACEMENT_RULE = {
    ObjectClass.GABE: BoxColor.GREEN,
    ObjectClass.CHESS_PIECE: BoxColor.BLUE,
}


def _tidy(raw_text: str, placement_rule: dict | None = None) -> MissionSpec:
    # TIDY는 특정 대상이 없다 — target_cls는 FETCH의 SELECT 필터링에만 쓰인다
    # (state_machine.md §3 'SELECT의 선정 기준' 3번 조건).
    rule = DEFAULT_PLACEMENT_RULE if placement_rule is None else placement_rule
    return MissionSpec(
        mode=MissionMode.TIDY,
        target_cls=None,
        placement_rule=dict(rule),
        raw_text=raw_text,
    )


def _fetch(raw_text: str, target_cls: ObjectClass) -> MissionSpec:
    return MissionSpec(
        mode=MissionMode.FETCH,
        target_cls=target_cls,
        placement_rule=dict(DEFAULT_PLACEMENT_RULE),
        raw_text=raw_text,
    )


# 문형은 architecture.puml의 sequences.md에서 실제 쓰인 예문을 그대로 옮겼다.
DEFAULT_TABLE = {
    "장난감 정리해줘": _tidy("장난감 정리해줘"),
    "정리해줘": _tidy("정리해줘"),
    "체스말은 검은 상자에": _tidy(
        "체스말은 검은 상자에",
        {**DEFAULT_PLACEMENT_RULE, ObjectClass.CHESS_PIECE: BoxColor.BLACK},
    ),
    "체스말은 검은 상자에 넣어줘": _tidy(
        "체스말은 검은 상자에 넣어줘",
        {**DEFAULT_PLACEMENT_RULE, ObjectClass.CHESS_PIECE: BoxColor.BLACK},
    ),
    "블록은 파란 상자에 넣어줘": _tidy(
        "블록은 파란 상자에 넣어줘",
        {**DEFAULT_PLACEMENT_RULE, ObjectClass.GABE: BoxColor.BLUE},
    ),
    "체스말 가져와": _fetch("체스말 가져와", ObjectClass.CHESS_PIECE),
    "가베 가져와": _fetch("가베 가져와", ObjectClass.GABE),
}


class ScriptedInterpreter(CommandInterpreter):
    def __init__(self, table: dict[str, MissionSpec] | None = None):
        self._table = dict(DEFAULT_TABLE if table is None else table)

    def parse(self, text: str) -> MissionSpec | None:
        """등록되지 않은 문형이면 **None** — 포트 계약 그대로다
        (domain/ports/command_interpreter.py).

        예전에는 ValueError를 올렸다. real 구현(Ros2CommandInterpreter)은
        understood=False 일 때 None을 돌려주므로 **같은 상황을 Fake는 예외로,
        real은 값으로 표현**하고 있었다 — Fake로 도는 도메인 테스트가 real의
        실패 경로를 한 번도 밟지 않는다는 뜻이다 (PR #9 리뷰 B항)."""
        spec = self._table.get(text)
        if spec is None:
            return None
        # placement_rule은 dict라 얕은 참조를 그대로 넘기면 한 테스트의 변경이
        # _table의 다음 조회에 새어 들어간다 — 매 호출마다 복사본을 반환한다.
        return replace(spec, placement_rule=dict(spec.placement_rule))

    def confirm_phrase(self, spec: MissionSpec) -> str:
        if spec.mode is MissionMode.FETCH:
            return f"{spec.target_cls.name}을(를) 가져올게요"
        return "정리를 시작할게요"
