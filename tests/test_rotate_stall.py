"""ROTATE 가 진전 없이 멈춰도 미션이 거기서 영원히 안 서게 — 2026-08-31 실기.

그날 APPROACH_PIECE 에서 yaw- 명령을 58초(602사이클) 연속 보냈는데
pose.yaw 가 ArUco 잡음(<1도) 안에서만 흔들리고 x/y 도 mm 단위로 그대로였다
— 팔이 가벽에 걸렸을 가능성이 유력하다(mission_config.ROTATE_STALL_SEC
주석 참고). navigator.DriveSequencer 의 회전 정지-감시가 그 사고를
재현하지 않는지 로봇 없이 확인한다.

시계는 monkeypatch 로 직접 앞당긴다 — 실제로 8초씩 자지 않는다.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

import mission_config as mcfg                       # noqa: E402
import navigator                                     # noqa: E402
from domain.task.motion import AGREED_ROTATION_RAD_S  # noqa: E402
from localizer import Pose                           # noqa: E402
from mission import MissionFSM, State                # noqa: E402
from navigator import DriveMode, DriveSequencer      # noqa: E402
from vehicle_link import ConsoleVehicleLink           # noqa: E402


class _FakeClock:
    """time.monotonic() 대신 쓴다 — 값은 테스트가 t 를 직접 정한다."""

    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


@pytest.fixture
def fake_clock(monkeypatch):
    clock = _FakeClock()
    monkeypatch.setattr(navigator.time, "monotonic", clock)
    return clock


def test_회전이_전혀_안_되면_STALL_SEC_뒤에_stalled_이_뜬다(fake_clock):
    """2026-08-31 그대로 — 로봇이 물리적으로 안 도는데 명령만 계속 나간다."""
    drive = DriveSequencer()
    robot_xy = (0.827, 0.349)
    robot_yaw = 90.0
    target_xy = (robot_xy[0] - 1.0, robot_xy[1])   # 반대쪽 — 계속 ROTATE 여야 함

    dt = 0.1
    seen_stalled_at = None
    t = 0.0
    max_t = mcfg.ROTATE_STALL_SEC + 5.0
    while t <= max_t:
        fake_clock.t = t
        cmd = drive.update(robot_xy, robot_yaw, target_xy, [])
        assert cmd.mode == DriveMode.ROTATE, "이 시나리오는 끝까지 ROTATE 여야 한다"
        if cmd.rotate_stalled and seen_stalled_at is None:
            seen_stalled_at = t
        t += dt

    assert seen_stalled_at is not None, "정지가 끝까지 감지되지 않았다"
    assert seen_stalled_at >= mcfg.ROTATE_STALL_SEC, "너무 일찍 오탐했다"
    assert seen_stalled_at <= mcfg.ROTATE_STALL_SEC + 1.0, "너무 늦게 잡았다"


def test_실제로_돌고_있으면_오탐하지_않는다(fake_clock):
    """느려도 진전이 있으면(회전각이 커서 오래 걸리는 경우) 정지로 안 본다."""
    drive = DriveSequencer()
    robot_xy = (0.0, 0.0)
    target_xy = (-1.0, 0.0)          # target_yaw ≈ 180도 — 오래 걸리는 큰 회전
    yaw = 90.0
    deg_per_s = math.degrees(AGREED_ROTATION_RAD_S)

    dt = 0.1
    t = 0.0
    max_t = mcfg.ROTATE_STALL_SEC * 3
    while t <= max_t:
        fake_clock.t = t
        cmd = drive.update(robot_xy, yaw, target_xy, [])
        assert not cmd.rotate_stalled, f"t={t:.1f}s — 실제로 도는데 정지로 오판했다"
        if cmd.mode == DriveMode.ROTATE:
            yaw += deg_per_s * dt    # 실제로 그만큼 돌았다고 시늉
        t += dt


def test_APPROACH_PIECE에서_회전이_막히면_포기하고_다음으로_간다(fake_clock):
    """실물에서 일어난 것 그대로: 걸려서 58초 동안 못 돈다. 그대로 두면
    미션이 거기서 영원히 멈춘다 — 이 기물은 보류하고 SEARCH_TARGET 으로
    돌아가야 나머지 기물이라도 마저 옮겨 '완주'할 수 있다."""
    fsm = MissionFSM()
    link = ConsoleVehicleLink(auto_complete=False)
    stuck_xy = (0.900, 0.900)   # 작업영역(WORKSPACE_X/Y) 중앙권 — 경계 효과 배제
    # 로봇 남쪽(180도 가까운 큰 회전이 필요한 자리, 그래도 작업영역 안)에 rook.
    piece_map = {"rook": [(stuck_xy[0], 0.420)]}
    # pose 는 끝까지 이 값 그대로다 — 실제로 하나도 안 움직였다는 뜻.
    pose = Pose(x=stuck_xy[0], y=stuck_xy[1], yaw_deg=90.0,
                ok=True, n_cams=2, fresh=True)

    dt = 0.1
    t = 0.0
    max_t = mcfg.ROTATE_STALL_SEC + 5.0
    entered_approach = False
    gave_up_at = None
    while t <= max_t:
        fake_clock.t = t
        fsm.step(pose, piece_map, link)
        if fsm.state == State.APPROACH_PIECE:
            entered_approach = True
        if entered_approach and fsm.state == State.SEARCH_TARGET and gave_up_at is None:
            gave_up_at = t
        t += dt

    assert entered_approach, "애초에 rook 를 쫓아가지도 않았다 — 시나리오 자체가 안 맞다"
    assert gave_up_at is not None, (
        f"{max_t:.0f}초 안에 포기하지 않았다 — 완주 못 하고 여기서 영원히 멈춘다")
    assert gave_up_at >= mcfg.ROTATE_STALL_SEC, "너무 빨리 포기했다(오탐)"
    assert len(fsm.skipped) == 1, "보류 목록에 안 남았다 — 다음 SEARCH_TARGET 에서 같은 걸 또 고른다"
