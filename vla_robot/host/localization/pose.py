"""로봇 pose 값 객체. cv2 없이 import 된다(FSM·시뮬레이터·테스트가 쓴다)."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw_deg: float = 0.0
    ok: bool = False            # False 면 로봇 위치를 모른다 -> Host 는 stop 을 보낸다
    n_cams: int = 0             # 이번 프레임에 마커를 본 카메라 수. HOLD 중에는 0
    age_s: float = 0.0          # 마지막 실제 관측 후 경과 시간
    fresh: bool = False         # 이번 프레임에 실제로 봤는가
    per_cam: dict = field(default_factory=dict)

    @property
    def xy(self) -> tuple[float, float]:
        return (self.x, self.y)

    def __str__(self) -> str:
        if not self.ok:
            return "pose: LOST"
        tag = "" if self.fresh else f" (HOLD {self.age_s:.2f}s)"
        return (f"x={self.x * 1000:7.1f}mm y={self.y * 1000:7.1f}mm "
                f"yaw={self.yaw_deg:6.1f}deg cams={self.n_cams}{tag}")
