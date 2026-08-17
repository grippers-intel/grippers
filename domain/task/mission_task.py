from dataclasses import dataclass

from domain.task.states import EstopState, IdleState
from domain.values import MissionContext, MissionMode, MissionSpec


@dataclass
class Ports:
    base: object
    arm: object
    perception: object
    interpreter: object
    estop: object  # threading.Event 유사 객체 (is_set() 지원)


class MissionTask:
    def __init__(self, ports: Ports):
        self.ports = ports

    def run(self, raw_text: str = ""):
        """raw_text가 있으면 IDLE이 interpreter.parse()로 해석해 SCAN으로 넘어간다.
        비어 있으면(기본값) IDLE에서 대기한다 (domain/task/states.py IdleState 계약).

        여기서 만드는 MissionSpec은 IDLE이 ctx.spec.raw_text를 읽을 자리표시자일
        뿐이다 — mode/placement_rule은 interpreter.parse() 결과로 곧바로 대체된다."""
        placeholder_spec = MissionSpec(
            mode=MissionMode.TIDY, target_cls=None, placement_rule={}, raw_text=raw_text
        )
        state = IdleState(MissionContext(spec=placeholder_spec))
        while state is not None:
            if self.ports.estop.is_set():
                state = EstopState()
            yield state
            state = state.execute(self.ports)
