# -*- coding: utf-8 -*-
from types import SimpleNamespace
from domain.ports.perception import Perception


class FakePerception(Perception):
    def detect_target(self):
        return True, SimpleNamespace(x=0.2, y=0.0, z=0.05), SimpleNamespace(x=0.04, y=0.04, z=0.04)

    def measure_gap(self):
        return SimpleNamespace(h_gap=0.3, centerline=SimpleNamespace(x=0, y=0, theta=0))

    def set_light_profile(self, profile):
        return True

    def monitor_clearance(self):
        return SimpleNamespace(front=1.0, left=1.0, right=1.0, top=1.0, contact_risk=False)
