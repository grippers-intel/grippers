"""명령 문형 회귀 테스트. docs/design/class_diagram.md §2: 'ScriptedInterpreter로
Fake 대체가 되어야 CI에서 명령 문형 회귀 테스트가 돌아간다.'"""

from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.scripted_interpreter import ScriptedInterpreter
from domain.adapters.fake.scripted_perception import ScriptedPerception
from domain.task.states import TransportState
from domain.values import (
    Destination,
    Detection,
    MissionContext,
    MissionMode,
    MissionSpec,
    ObjectClass,
    Point3,
)


def test_command_phrase_sets_rule_it_states():
    """확정 미션 명세서 예문 — 명령이 지정한 대로 규칙이 나온다."""
    spec = ScriptedInterpreter().parse(
        "모든 체스 기물을 왼쪽 박스에, 장난감들은 오른쪽 박스에 정리해주세요"
    )

    assert spec.mode is MissionMode.TIDY
    assert spec.placement_rule[ObjectClass.CHESS_PIECE] is Destination.LEFT
    assert spec.placement_rule[ObjectClass.GABE] is Destination.RIGHT


def test_reversed_command_flips_destinations():
    """같은 물체 구성이라도 명령이 다르면 다르게 행동한다(명세서 핵심 성질) —
    좌우를 뒤집은 명령은 정반대 규칙을 내야 한다."""
    spec = ScriptedInterpreter().parse(
        "체스 기물은 오른쪽 박스에, 장난감은 왼쪽 박스에 정리해주세요"
    )

    assert spec.placement_rule[ObjectClass.CHESS_PIECE] is Destination.RIGHT
    assert spec.placement_rule[ObjectClass.GABE] is Destination.LEFT


def test_partial_rule_omits_unmentioned_class():
    """명령이 언급하지 않은 클래스는 placement_rule에 아예 없다 — SELECT가
    '목적지가 정의돼 있을 것' 조건으로 걸러 후보에서 제외한다."""
    spec = ScriptedInterpreter().parse("체스 기물만 왼쪽 박스에 정리해줘")

    assert spec.placement_rule[ObjectClass.CHESS_PIECE] is Destination.LEFT
    assert ObjectClass.GABE not in spec.placement_rule


def test_fetch_phrase_sets_mode_and_target_cls():
    spec = ScriptedInterpreter().parse("체스말 가져와")

    assert spec.mode is MissionMode.FETCH
    assert spec.target_cls is ObjectClass.CHESS_PIECE


def test_unknown_phrase_returns_none():
    """등록되지 않은 문형은 **None** — 예전에는 ValueError였다.

    real 구현(Ros2CommandInterpreter)이 understood=False 일 때 None을 돌려주므로,
    Fake가 예외를 던지면 같은 상황을 두 구현이 다르게 표현하게 된다 —
    Fake로 도는 도메인 테스트가 real의 실패 경로를 한 번도 밟지 않는다
    (PR #9 리뷰 B항, domain/ports/command_interpreter.py 계약)."""
    assert ScriptedInterpreter().parse("알 수 없는 명령") is None


def test_parse_does_not_leak_mutations_into_table():
    """반환된 MissionSpec.placement_rule을 호출자가 고쳐도, 같은 문형을 다시
    parse()했을 때 오염되면 안 된다 (얕은 dict 참조를 그대로 넘기면 샌다)."""
    interpreter = ScriptedInterpreter()

    first = interpreter.parse("체스 기물만 왼쪽 박스에 정리해줘")
    first.placement_rule[ObjectClass.GABE] = Destination.RIGHT

    second = interpreter.parse("체스 기물만 왼쪽 박스에 정리해줘")
    assert ObjectClass.GABE not in second.placement_rule


def test_custom_table_overrides_default():
    custom_spec = MissionSpec(
        mode=MissionMode.TIDY, target_cls=None, placement_rule={}, raw_text="테스트"
    )
    interpreter = ScriptedInterpreter(table={"테스트": custom_spec})

    assert interpreter.parse("테스트") is not custom_spec  # 매번 복사본
    assert interpreter.parse("테스트").raw_text == "테스트"


class _RecordingPerception(ScriptedPerception):
    """find_box()에 실제로 어떤 목적지가 넘어오는지 기록하는 스파이."""

    def __init__(self):
        super().__init__()
        self.requested_dests = []

    def find_box(self, dest):
        self.requested_dests.append(dest)
        return super().find_box(dest)


def test_command_destination_flows_into_transport_find_box(make_ports):
    """단위 테스트를 넘어 — 명령으로 얻은 placement_rule이 실제로 TRANSPORT의
    find_box() 호출에 반영되는지 FSM 레벨에서 확인한다."""
    spec = ScriptedInterpreter().parse(
        "모든 체스 기물을 왼쪽 박스에, 장난감들은 오른쪽 박스에 정리해주세요"
    )
    target = Detection(
        track_id=1,
        cls=ObjectClass.CHESS_PIECE,
        pose_m=Point3(x=0.2, y=0.0, z=0.0),
        dims_m=Point3(x=0.03, y=0.03, z=0.03),
        yaw_rad=0.0,
        confidence=0.9,
    )
    ctx = MissionContext(spec=spec)
    perception = _RecordingPerception()
    ports = make_ports(base=FakeBase(), perception=perception)

    TransportState(ctx, target).execute(ports)

    assert perception.requested_dests == [Destination.LEFT]
