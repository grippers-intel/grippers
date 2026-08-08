# -*- coding: utf-8 -*-
from abc import ABC, abstractmethod


class State(ABC):
    name = "base"

    @abstractmethod
    def execute(self, ports):
        """다음 State 인스턴스를 반환. 미션 종료면 None."""
