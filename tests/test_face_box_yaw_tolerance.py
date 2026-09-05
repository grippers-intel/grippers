"""FACE_BOX/NUDGE_BOX가 박스 목표 방위를 맞출 때 쓰는 허용각이 일반
주행용(DRIVE_YAW_TOLERANCE_DEG, 12도)에서 전용
BOX_FACE_YAW_TOLERANCE_DEG(45도)로 분리됐는지 확인한다 (사용자 지시,
2026-09-05 — "FACE_BOX도 고칠 수 있으면 고쳐" + "방향무관 즉시정지").

safe_300(servo 1 요 보정, ±60도까지 실기 검증됨)이 이제 드랍 직전
잔여 오차를 팔로 흡수하므로, 차량이 회전으로 몇 도까지 좁혀야 하는지를
다시 낮출 이유가 없다 — FACE_BOX가 더 넓은 각도에서도 빨리 통과해야
한다."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "host"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "host" / "aruco"))

import mission_config as mcfg                                    # noqa: E402
from mission import MissionFSM, State                              # noqa: E402

from conftest import PiSim                                         # noqa: E402


def _fsm_facing(target_yaw_deg: float, actual_yaw_deg: float) -> tuple[MissionFSM, PiSim]:
    fsm = MissionFSM()
    fsm.state = State.FACE_BOX
    fsm._face_target_yaw_deg = target_yaw_deg
    link = PiSim(x=1.27, y=1.0, yaw_deg=actual_yaw_deg)
    return fsm, link


def test_일반_주행_허용각을_넘지만_박스_허용각_안이면_바로_통과한다():
    """30도 오차 — DRIVE_YAW_TOLERANCE_DEG(12도)는 넘지만
    BOX_FACE_YAW_TOLERANCE_DEG(45도) 안이다."""
    assert 12.0 < 30.0 < mcfg.BOX_FACE_YAW_TOLERANCE_DEG
    fsm, link = _fsm_facing(target_yaw_deg=90.0, actual_yaw_deg=60.0)

    fsm.step(link.pose(), {}, link)

    assert fsm.state == State.NUDGE_BOX, "박스 전용 허용각 안인데도 계속 돌았다"


def test_박스_허용각도_넘으면_계속_돈다():
    fsm, link = _fsm_facing(target_yaw_deg=90.0, actual_yaw_deg=0.0)

    fsm.step(link.pose(), {}, link)

    cmds = {c for c, status in link.sent if status == "FACE_BOX"}
    assert fsm.state == State.FACE_BOX
    assert cmds and "stop" not in cmds, "박스 허용각 밖인데도 회전하지 않았다"


def test_일반_주행_허용각_상수는_그대로다():
    """FACE_BOX 전용 허용각을 새로 만들면서 일반 주행(GRASP 접근 등)이
    쓰는 DRIVE_YAW_TOLERANCE_DEG까지 같이 넓어지면 안 된다."""
    assert mcfg.DRIVE_YAW_TOLERANCE_DEG == 12.0
