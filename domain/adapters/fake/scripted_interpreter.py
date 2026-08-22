"""ScriptedInterpreter — CommandInterpreter 포트의 테스트 더블.
실제 언어모델/파서 대신 정해진 문형 목록(`_table`)을 조회해 MissionSpec을 반환한다 —
`ScriptedInterpreter` 로 Fake 대체가 되어야 CI에서 명령 문형 회귀 테스트가 돌아간다
(docs/design/class_diagram.md §2 '포트가 4종이 된 이유').

⚠️ 2026-08-23 확정 미션 명세서 기준으로 다시 썼다. 이전 버전은
docs/subsystems/objects.md의 "GABE→GREEN, CHESS_PIECE→BLUE" 고정 기본값과
BoxColor(BLACK/RED/BLUE/GREEN) 색상 탐색 모델을 썼는데, 확정 명세서는
"좌표를 하드코딩한다. 색 탐색은 하지 않는다"·"목적지는 명령이 정한다 —
'체스는 왼쪽'이 고정이 아니라, 매 명령이 대상과 목적지를 함께 지정한다"고
확정했다(docs/는 권위 없음 — 팀 repo 문서와 실제 구현이 여러 세션에 걸쳐
어긋나 있었다는 게 사용자 확인 사항). 그래서:
- BoxColor → Destination(LEFT/RIGHT) — 바구니는 색이 아니라 좌우 위치로
  구분되는 2개뿐이다.
- 고정 DEFAULT_PLACEMENT_RULE을 없앴다 — 문형마다 규칙을 통째로 지정한다."""

from dataclasses import replace

from domain.ports.command_interpreter import CommandInterpreter
from domain.values import Destination, MissionMode, MissionSpec, ObjectClass


def _tidy(raw_text: str, placement_rule: dict) -> MissionSpec:
    # TIDY는 특정 대상이 없다 — target_cls는 FETCH의 SELECT 필터링에만 쓰인다
    # (state_machine.md §3 'SELECT의 선정 기준' 3번 조건).
    return MissionSpec(
        mode=MissionMode.TIDY,
        target_cls=None,
        placement_rule=dict(placement_rule),
        raw_text=raw_text,
    )


def _fetch(raw_text: str, target_cls: ObjectClass, dest: Destination) -> MissionSpec:
    # FETCH는 2026-08-23 확정 미션 명세서엔 없다 — 이전 주제("암실 반출")의
    # 잔재다. select_candidates()는 모드와 무관하게 placement_rule에 목적지가
    # 있어야 후보로 본다는 계약(states.py select_candidates 2번 조건)이라,
    # 그걸 지키려고 target_cls 하나짜리 규칙을 형식상 채운다.
    return MissionSpec(
        mode=MissionMode.FETCH,
        target_cls=target_cls,
        placement_rule={target_cls: dest},
        raw_text=raw_text,
    )


# 문형은 확정 미션 명세서(2026-08-23)의 시연 예문과 "핵심 성질 — 목적지는
# 명령이 정한다. 같은 장면에서도 명령이 다르면 다르게 행동한다"를 그대로
# 옮겼다. 좌우가 뒤집힌 문형을 나란히 둬서 그 성질을 문형 테이블 자체로
# 보여준다.
#
# "장난감 정리해줘"는 확정 명세서 예문에는 없지만, 여러 세션에 걸쳐 도메인
# FSM 테스트 전반(tests/conftest.py의 run_to_completion 기본 raw_text)이 이
# 문형을 "체스+장난감 전부 정리" 기본 트리거로 써 왔다 — 지우면 그 테스트들이
# 전부 IDLE에 멈춘다. 규칙 자체는 새 모델(Destination)로 맞추고 문형은 그대로
# 남긴다.
DEFAULT_TABLE = {
    "장난감 정리해줘": _tidy(
        "장난감 정리해줘",
        {ObjectClass.CHESS_PIECE: Destination.LEFT, ObjectClass.GABE: Destination.RIGHT},
    ),
    "모든 체스 기물을 왼쪽 박스에, 장난감들은 오른쪽 박스에 정리해주세요": _tidy(
        "모든 체스 기물을 왼쪽 박스에, 장난감들은 오른쪽 박스에 정리해주세요",
        {ObjectClass.CHESS_PIECE: Destination.LEFT, ObjectClass.GABE: Destination.RIGHT},
    ),
    "체스 기물은 오른쪽 박스에, 장난감은 왼쪽 박스에 정리해주세요": _tidy(
        "체스 기물은 오른쪽 박스에, 장난감은 왼쪽 박스에 정리해주세요",
        {ObjectClass.CHESS_PIECE: Destination.RIGHT, ObjectClass.GABE: Destination.LEFT},
    ),
    # 언급 안 된 클래스는 placement_rule에 아예 없다 — select_candidates()가
    # "목적지가 정의돼 있을 것"으로 걸러서, 장난감은 후보에서 빠진 채
    # 체스 기물만 정리된다(명세서 "toy 클래스는 미션 계층에서 처리" 원칙과
    # 같은 결의 부분 정리).
    "체스 기물만 왼쪽 박스에 정리해줘": _tidy(
        "체스 기물만 왼쪽 박스에 정리해줘",
        {ObjectClass.CHESS_PIECE: Destination.LEFT},
    ),
    "체스말 가져와": _fetch("체스말 가져와", ObjectClass.CHESS_PIECE, Destination.LEFT),
    "가베 가져와": _fetch("가베 가져와", ObjectClass.GABE, Destination.RIGHT),
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
