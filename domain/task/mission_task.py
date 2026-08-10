# -*- coding: utf-8 -*-
from dataclasses import dataclass
from domain.task.states import IdleState, EstopState


@dataclass
class Ports:
    base: object
    arm: object
    perception: object
    estop: object  # threading.Event 유사 객체 (is_set() 지원)


class MissionTask:
    def __init__(self, ports: Ports):
        self.ports = ports

    def run(self):
        state = IdleState()
        while state is not None:
            if self.ports.estop.is_set():
                state = EstopState()
            yield state
            state = state.execute(self.ports)
