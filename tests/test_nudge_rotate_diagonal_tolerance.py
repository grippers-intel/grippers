"""NUDGE_BOX 회전판이 정면(BASKET_YAW_DEADBAND_RAD)까지 안 맞춰도
NUDGE_ROTATE_DIAGONAL_TOLERANCE_RAD 안이면 그만 도는지 (2026-09-05, 사용자
지시 — "무조건 정면은 좀 위험해").

⚠️ 이건 gate_ok(Host 전용 ArUco 판정)로 회전을 우회하는 것과 다르다 —
여전히 **Pi 라이다 실측값**(fix.yaw_rad)을 본다. 2026-09-03/09-04에 문제됐던
"Host만의 느슨한 판정으로 조기종료"(test_nudge_box_rotate_fix.py가 그
회귀를 지킨다)와는 신호 자체가 다르므로 같이 재발하지 않는다."""

from __future__ import annotations

import sys
from pathlib import Path

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

import mission_config as mcfg               # noqa: E402
from mission import MissionFSM, State        # noqa: E402
from vehicle_link import BasketFix           # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from conftest import PiSim                   # noqa: E402


def _rotate_fsm_with_fix(axis: str, yaw_rad: float) -> tuple[MissionFSM, PiSim]:
    fsm = MissionFSM()
    fsm.state = State.NUDGE_BOX
    fsm.target_label = "rook"
    fsm._nudge_plan = (0.15, axis)
    link = PiSim(x=1.0, y=1.0, yaw_deg=mcfg.BOX_FACE_YAW_DEG)
    link.last_basket_fix = BasketFix(yaw_rad=yaw_rad)
    return fsm, link


def test_허용각_안이면_정면까지_안_맞춰도_그만_돈다():
    yaw_rad = mcfg.NUDGE_ROTATE_DIAGONAL_TOLERANCE_RAD * 0.9
    assert yaw_rad > mcfg.BASKET_YAW_DEADBAND_RAD, "전제: 타이트한 데드밴드는 이미 벗어나 있어야 한다"
    fsm, link = _rotate_fsm_with_fix("rotate_left", yaw_rad)

    fsm.step(link.pose(), {}, link)

    assert fsm.state == State.PLACE, "허용각 안인데도 계속 돌았다"


def test_허용각_밖이면_계속_돈다():
    yaw_rad = mcfg.NUDGE_ROTATE_DIAGONAL_TOLERANCE_RAD * 1.5
    fsm, link = _rotate_fsm_with_fix("rotate_left", yaw_rad)

    fsm.step(link.pose(), {}, link)

    assert fsm.state == State.NUDGE_BOX, "허용각 밖인데 그만 돌았다"
    cmds = {c for c, status in link.sent if status == "NUDGE_BOX"}
    assert cmds == {"yaw+"}


def test_허용각_판정은_debounce_없이_즉시_확정된다():
    """gate_ok(Host 전용 판정)와 달리 이건 Pi 실측이라, NUDGE_GATE_CONFIRM_
    CYCLES 만큼 기다릴 필요가 없다 — 한 사이클 만에 PLACE로 넘어가야 한다."""
    yaw_rad = mcfg.NUDGE_ROTATE_DIAGONAL_TOLERANCE_RAD * 0.5
    fsm, link = _rotate_fsm_with_fix("rotate_left", yaw_rad)

    fsm.step(link.pose(), {}, link)

    assert fsm.state == State.PLACE
