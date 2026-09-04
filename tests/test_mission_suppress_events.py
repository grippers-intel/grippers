"""GRASP_DONE/PLACE_DONE 시점에 MissionFSM이 "방금 파지/투입한 그 개체"를
run_mission.py가 piece_map.PieceTracker.suppress_at()으로 넘길 수 있게
1회성 이벤트(last_grasp_event/last_place_event)를 남기는지 (2026-09-05,
사용자 지시).

라벨 전체가 아니라 정확히 그 좌표의 그 개체만 숨기려는 것이므로, 이
이벤트가 실려 보내는 좌표가 맞는지가 핵심이다 — GRASP는 파지 직전 원래
있던 자리(target_xy), PLACE는 목적지 바구니 좌표(box_pose)다."""

from __future__ import annotations

import sys
from pathlib import Path

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

import mission_config as mcfg          # noqa: E402
from localizer import box_pose         # noqa: E402
from mission import MissionFSM, State  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from conftest import PiSim              # noqa: E402


class _GraspDonePi(PiSim):
    def poll_status(self) -> str:
        return "GRASP_DONE"


class _PlaceDonePi(PiSim):
    def poll_status(self) -> str:
        return "PLACE_DONE"


class _PlaceFailedPi(PiSim):
    def poll_status(self) -> str:
        return "FAILED"


def test_GRASP_DONE_시점에_라벨과_원래_좌표를_이벤트로_남긴다():
    fsm = MissionFSM()
    fsm.state = State.GRASP
    fsm.target_label = "rook"
    fsm._target_xy = (1.0, 0.6)
    link = _GraspDonePi(x=1.0, y=1.0, yaw_deg=90.0)

    fsm.step(link.pose(), {}, link)

    assert fsm.last_grasp_event == ("rook", (1.0, 0.6))


def test_PLACE_DONE_시점에_목적지_바구니_좌표를_이벤트로_남긴다():
    dest_box_name = mcfg.PIECE_DEST_BOX["rook"]
    fsm = MissionFSM()
    fsm.state = State.PLACE
    fsm.target_label = "rook"
    link = _PlaceDonePi(x=1.0, y=1.0, yaw_deg=90.0, box=dest_box_name)

    fsm.step(link.pose(), {}, link)

    assert fsm.last_place_event is not None
    label, xy = fsm.last_place_event
    assert label == "rook"
    assert xy == box_pose(dest_box_name)[:2]


def test_PLACE_FAILED에서는_이벤트를_남기지_않는다():
    """Pi의 FAILED는 오탐일 수 있어 실제 안착 여부를 "다음 SEARCH_TARGET
    에서 다시 보이는가"로 판단하게 이미 설계돼 있다(mission.py의 같은
    분기 코멘트 참고) — 여기서 숨겨 버리면 정말 못 들어가 바닥에 남은
    기물을 다시 못 찾는다."""
    dest_box_name = mcfg.PIECE_DEST_BOX["rook"]
    fsm = MissionFSM()
    fsm.state = State.PLACE
    fsm.target_label = "rook"
    link = _PlaceFailedPi(x=1.0, y=1.0, yaw_deg=90.0, box=dest_box_name)

    fsm.step(link.pose(), {}, link)

    assert fsm.last_place_event is None
