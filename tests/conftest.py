"""Host FSM 을 하드웨어 없이 돌리기 위한 시늉 장치.

이 저장소에는 테스트가 없었다. 2026-08-28 실기에서 GRASP 는 성공했는데
INSERT 는 95회 연속 거부됐고, 그 원인(바구니가 0.196m 멀다)을 고친 코드를
검증할 방법이 로봇밖에 없었다. 로봇 없이 그 경로를 돌릴 수 있어야 한다.

`PiSim` 은 두 가지를 한꺼번에 흉내낸다.

  * **차량** — Host 가 보낸 명령을 실제 속도로 적분해 자세를 옮긴다.
    Host 는 `moved` 를 오버헤드 자세로 재므로, 자세가 명령대로 움직여야
    미세이동 완료 판정이 실기와 같은 방식으로 걸린다.
  * **Pi** — 바구니 판정을 실기와 **같은 문장**으로 돌려준다. Host 는
    그 문장을 정규식으로 파싱하므로(vehicle_link.parse_basket_fix),
    문장이 다르면 시늉이 아니라 딴 것을 시험하게 된다.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import pytest

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

import config as cfg              # noqa: E402
from localizer import Pose        # noqa: E402
from vehicle_link import MissionCommand, VehicleLink  # noqa: E402

# 라이다가 차체 기준점보다 얼마나 앞에 있는가.
#
# 실측에서 역산한 값이다(2026-08-28). Host 가 자연스럽게 서는 자리는
# 목적지에서 PLACE_TRIGGER_DIST_M(0.35) - BOX_NUDGE_M(0.05) = 0.30m 이고,
# 그때 로봇 y 는 약 1.00m 였다. 바구니 앞면은 y = 1.625 - BOX_L/2 = 1.45m
# 이므로 차체에서 앞면까지는 0.45m 인데 라이다는 0.351m 를 읽었다.
LIDAR_AHEAD_M = 0.45 - 0.351

# Pi 가 APPROACH_BOX 에서 쓰는 속도 상한(domain/task/motion.py 합의값).
BASKET_APPROACH_MPS = 0.06

# ⚠️ 여기서부터는 **Pi 쪽 값을 그대로 베껴 둔다** — Host 의 mission_config
# 를 참조하지 않는다. 참조하면 Host 가 상수를 잘못 바꿔도 시늉이 같이
# 바뀌어 테스트가 통과해 버린다. 이 시늉의 존재 이유는 Host 가 Pi 의
# 수용 창 안에 서는지 보는 것이므로, 창은 Pi 의 것이어야 한다.
# 출처: domain/task/baseline_constants.py
PI_STOP_LIDAR_M = 0.140
PI_STOP_TOLERANCE_M = 0.015
PI_LATERAL_TOLERANCE_M = 0.070
AGREED_LINEAR_MPS = 0.10
AGREED_ROTATION_RAD_S = 0.25


@dataclass
class PiSim(VehicleLink):
    """Host FSM 이 말을 거는 상대. 차량 운동학 + Pi 바구니 판정."""

    x: float = 1.271
    y: float = 1.000
    yaw_deg: float = 90.0
    dt: float = 1.0 / 14.0        # 실측 Host 루프 주기
    box: str = "chess"
    #: 좌우 판독을 아예 못 하는 경우(창을 양쪽 다 채움)를 흉내낼 때 False.
    lateral_known: bool = True
    #: servo 1 이 안 듣는 상황처럼, 전진해도 좌우가 안 줄어드는 경우.
    freeze_lateral: bool = False

    sent: list = field(default_factory=list)
    _pending: str | None = None
    _last_detail: str = ""

    # --- 기하 -----------------------------------------------------------
    @property
    def _face_y(self) -> float:
        return cfg.BOXES[self.box][1] - cfg.BOX_L / 2.0

    @property
    def lidar_m(self) -> float:
        """라이다가 읽는 바구니 앞면까지의 거리."""
        return (self._face_y - self.y) - LIDAR_AHEAD_M

    @property
    def lateral_m(self) -> float:
        """바구니 중심이 로봇 기준 어디 있는가(+가 왼쪽).

        로봇이 +y(BOX_FACE_YAW_DEG=90도)를 보고 있을 때 로봇의 왼쪽은
        월드 -x 다. 그래서 부호가 뒤집힌다."""
        return self.x - cfg.BOXES[self.box][0]

    def pose(self) -> Pose:
        return Pose(x=self.x, y=self.y, yaw_deg=self.yaw_deg,
                    ok=True, n_cams=2, fresh=True)

    # --- VehicleLink -----------------------------------------------------
    def send(self, cmd: MissionCommand) -> None:
        self.sent.append((cmd.cmd, cmd.status))
        self._move(cmd)
        if cmd.status == "PLACE":
            self._judge_insert()

    def poll_status(self) -> str:
        status, self._pending = self._pending, None
        return status or "IDLE"

    # --- 차량 운동학 ------------------------------------------------------
    def _move(self, cmd: MissionCommand) -> None:
        # Pi 는 바구니 접근 구간에서만 상한을 낮춘다.
        v = (BASKET_APPROACH_MPS if cmd.status == "NUDGE_BOX"
             else AGREED_LINEAR_MPS)
        step = v * self.dt
        heading = math.radians(self.yaw_deg)
        left = heading + math.pi / 2.0
        if cmd.cmd == "go":
            self.x += step * math.cos(heading)
            self.y += step * math.sin(heading)
        elif cmd.cmd == "back":
            self.x -= step * math.cos(heading)
            self.y -= step * math.sin(heading)
        elif cmd.cmd in ("left", "right"):
            if self.freeze_lateral:
                return
            sign = 1.0 if cmd.cmd == "left" else -1.0
            self.x += sign * step * math.cos(left)
            self.y += sign * step * math.sin(left)
        elif cmd.cmd in ("yaw+", "yaw-"):
            sign = 1.0 if cmd.cmd == "yaw+" else -1.0
            self.yaw_deg += sign * math.degrees(AGREED_ROTATION_RAD_S * self.dt)

    # --- Pi 바구니 판정 ---------------------------------------------------
    def _judge_insert(self) -> None:
        """`domain/task/preconditions.check_insert` 의 문장을 그대로 낸다.

        ⚠️ 문구를 바꾸면 Host 의 파서가 조용히 실패한다 — 그게 이 시늉의
        요점이므로 리터럴을 Pi 쪽과 같이 유지할 것."""
        reasons = []
        upper = PI_STOP_LIDAR_M + PI_STOP_TOLERANCE_M
        if self.lidar_m > upper:
            reasons.append(
                f"바구니가 멀다 (라이다 {self.lidar_m:.3f}m > {upper:.3f}m)")
        if self.lateral_known and abs(self.lateral_m) > PI_LATERAL_TOLERANCE_M:
            reasons.append(
                f"좌우로 밀려 있다 ({self.lateral_m * 1000:+.0f}mm > "
                f"±{PI_LATERAL_TOLERANCE_M * 1000:.0f}mm)")
        if reasons:
            self._last_detail = " / ".join(reasons)
            from vehicle_link import parse_basket_fix
            fix = parse_basket_fix(self._last_detail)
            if fix is not None:
                self.last_basket_fix = fix
            self._pending = "BUSY"
        else:
            self._last_detail = ""
            self._pending = "PLACE_DONE"


@pytest.fixture
def pi_sim():
    return PiSim
