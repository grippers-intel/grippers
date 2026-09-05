"""PLACE가 safe_300(드랍 직전 servo 1 요 보정)용 yaw_correction_deg를 실제
미션 경로에도 실어 보내는지 확인한다 (사용자 지시, 2026-09-05 —
"run_mission에 해당 로직을 반영해주고").

manual_insert_probe.py로 실기 검증까지 마친 뒤 나온 지시다 — FACE_BOX/
NUDGE_BOX(회전·정렬 루프)는 그대로 두고, PLACE가 Pi로 보내는 매 사이클
명령에 그 순간의 잔여 지향오차(basket_target.check_basket_insert_gate의
facing_error_deg)를 얹기만 한다. FACE_BOX/NUDGE_BOX가 이미 잘 맞춰 왔으면
이 값은 0에 가까워 safe_300 자체가 사실상 건너뛰어진다
(BaselineInsertState 참고) — 즉 기존 동작을 깨지 않아야 한다.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path

import pytest

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

import basket_target                                               # noqa: E402
from localizer import Pose                                         # noqa: E402
from mission import MissionFSM, State                              # noqa: E402
from vehicle_link import MissionCommand, VehicleLink                # noqa: E402


@dataclass
class RecordingLink(VehicleLink):
    """PLACE가 보내는 MissionCommand를 있는 그대로 기록만 한다 — Pi 판정은
    시늉하지 않는다(이 파일은 yaw_correction_deg 필드 자체만 본다)."""

    sent: list[MissionCommand] = field(default_factory=list)

    def send(self, cmd: MissionCommand) -> None:
        self.sent.append(cmd)

    def poll_status(self) -> str:
        return "BUSY"


def _place_fsm(x: float, y: float, yaw_deg: float, target_label: str) -> tuple[MissionFSM, RecordingLink]:
    fsm = MissionFSM()
    fsm.state = State.PLACE
    fsm.target_label = target_label
    link = RecordingLink()
    return fsm, link, Pose(x=x, y=y, yaw_deg=yaw_deg, ok=True, n_cams=2, fresh=True)


def test_상자_목적지면_그_순간의_지향오차를_실어_보낸다():
    """rook -> chess 상자. 로봇 자세로 basket_target.check_basket_insert_gate가
    내는 facing_error_deg와 정확히 같은 값이 PLACE 명령에 실려야 한다."""
    x, y, yaw_deg = 1.30, 1.20, 60.0
    fsm, link, pose = _place_fsm(x, y, yaw_deg, "rook")

    fsm.step(pose, {}, link)

    expected = basket_target.check_basket_insert_gate((x, y), yaw_deg, "chess").facing_error_deg
    assert link.sent and link.sent[-1].status == "PLACE"
    assert link.sent[-1].yaw_correction_deg == pytest.approx(expected)
    assert expected != 0.0   # 이 자세가 실제로 오차를 만드는지부터 확인


def test_정면으로_잘_들어왔으면_보정값이_0에_가깝다():
    """FACE_BOX/NUDGE_BOX가 이미 잘 맞춰 온 경우 — 기존 동작(보정 없음)과
    사실상 같아야 한다."""
    center_x, center_y = basket_target.target_center("chess")
    fsm, link, pose = _place_fsm(center_x, center_y - 0.20, 90.0, "rook")

    fsm.step(pose, {}, link)

    assert link.sent[-1].yaw_correction_deg == pytest.approx(0.0, abs=1.0)


def test_상자_목적지가_아니면_보정을_안_보낸다():
    """PIECE_DEST_BOX에 없는 라벨(사람에게 직접 가져다주는 경우)은 상자가
    없으니 safe_300 보정도 의미가 없다 — 0으로 고정."""
    fsm, link, pose = _place_fsm(1.30, 1.20, 60.0, "unknown_label_no_box")

    fsm.step(pose, {}, link)

    assert link.sent[-1].yaw_correction_deg == 0.0
