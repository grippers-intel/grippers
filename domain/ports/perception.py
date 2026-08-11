from abc import ABC, abstractmethod


class Perception(ABC):
    @abstractmethod
    def detect_target(self):
        """-> (found: bool, pose: Point3, dims: Point3)"""

    @abstractmethod
    def measure_gap(self):
        """-> object(h_gap: float, centerline: Pose2D)"""

    @abstractmethod
    def set_light_profile(self, profile: str) -> bool: ...

    @abstractmethod
    def monitor_clearance(self):
        """-> object(front, left, right, top: float, contact_risk: bool)"""
