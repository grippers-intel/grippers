from types import SimpleNamespace

from domain.ports.perception import Perception
from domain.values import Point3, Pose2D


class FakePerception(Perception):
    def __init__(self, found: bool = True, contact_risk: bool = False, h_gap: float = 0.3):
        self._found = found
        self._contact_risk = contact_risk
        self._h_gap = h_gap

    def detect_target(self):
        if not self._found:
            return False, None, None
        return (
            True,
            SimpleNamespace(x=0.2, y=0.0, z=0.05),
            SimpleNamespace(x=0.04, y=0.04, z=0.04),
        )

    def measure_gap(self):
        return SimpleNamespace(
            h_gap=self._h_gap,
            centerline=Pose2D(x=0.0, y=0.0, theta=0.0),
        )

    def set_light_profile(self, profile):
        return True

    def monitor_clearance(self):
        return SimpleNamespace(
            front=1.0, left=1.0, right=1.0, top=1.0, contact_risk=self._contact_risk
        )
