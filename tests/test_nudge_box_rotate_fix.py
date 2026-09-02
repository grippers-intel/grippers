"""NUDGE_BOX가 회전(yaw) 보정 계획을 실제로 실행한다 (10:18 실기, 2026-09-02).

## 왜 이 기능이 생겼나

10:18 실기: GRASP·CARRY까지 잘 끝난 뒤 바구니 앞에서

    INSERT_BLOCKED [APPROACH_BOX] 정렬이 틀어졌다 (yaw +0.17rad > 0.087rad)

가 수백 번 반복되며 미션이 멈췄다. `corrections.from_insert()`는 거리
다음으로 yaw(라이다 평면 자체가 정면이 아님)를 보는데, Host의
`_plan_basket_fix`는 여태 `lateral_m`만 읽어서 이 경우 아무 계획도 못
세웠다 — PLACE가 `_nudge_plan`을 못 채우니 NUDGE_BOX로 못 돌아가고,
`link.take_basket_fix()`로 매번 소비만 하고 버리며 그 자리에 갇혔다.

이 파일은 `_plan_basket_fix`가 만든 회전판 계획(`rotate_left`/
`rotate_right`)을 NUDGE_BOX가 실제로 "yaw+"/"yaw-"로 실행하고, 목표
회전량만큼 돌면 멈추고 PLACE로 넘어가는지 검증한다. `_plan_basket_fix`
자체의 계획 생성(축 선택·부호·예산)은 test_pi_fix_channel.py가 따로
본다."""

from __future__ import annotations

import sys
from pathlib import Path

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

import mission_config as mcfg                                    # noqa: E402
from mission import MissionFSM, State                             # noqa: E402

from conftest import PiSim                                        # noqa: E402

MAX_STEPS = 400


def _rotate_fsm(axis: str, amount_rad: float = 0.15,
               start_yaw_deg: float = None) -> tuple[MissionFSM, PiSim]:
    fsm = MissionFSM()
    fsm.state = State.NUDGE_BOX
    fsm.target_label = "rook"
    fsm._nudge_plan = (amount_rad, axis)
    # BOX_FACE_YAW_DEG와 이미 정렬된 자리에서 시작한다 — 그래도 회전판이
    # 우선이어야 한다(그렇지 않으면 "정렬됐다"고 보고 그냥 멈춰 버린다).
    yaw = start_yaw_deg if start_yaw_deg is not None else mcfg.BOX_FACE_YAW_DEG
    link = PiSim(x=1.0, y=1.0, yaw_deg=yaw)
    return fsm, link


def _run(fsm: MissionFSM, link: PiSim, max_steps: int = MAX_STEPS) -> int:
    for n in range(1, max_steps + 1):
        fsm.step(link.pose(), {}, link)
        if fsm.state == State.PLACE:
            return n
    raise AssertionError(
        f"{max_steps} 사이클 안에 회전을 못 끝냈다 — 상태 {fsm.state.name}")


def test_rotate_left_계획은_yaw_플러스_명령을_보낸다():
    fsm, link = _rotate_fsm("rotate_left")

    fsm.step(link.pose(), {}, link)

    cmds = {c for c, status in link.sent if status == "NUDGE_BOX"}
    assert cmds == {"yaw+"}


def test_rotate_right_계획은_yaw_마이너스_명령을_보낸다():
    fsm, link = _rotate_fsm("rotate_right")

    fsm.step(link.pose(), {}, link)

    cmds = {c for c, status in link.sent if status == "NUDGE_BOX"}
    assert cmds == {"yaw-"}


def test_이미_BOX_FACE_YAW_DEG에_정렬돼_있어도_회전판이_우선이다():
    """평소 NUDGE_BOX의 "정렬됐으면 그만 돌아라" 판정과 다투면 안 된다 —
    이 회전은 ArUco 기준 BOX_FACE_YAW_DEG가 아니라 Pi 라이다 판독을
    맞추려는 것이라, 이미 BOX_FACE_YAW_DEG에 서 있어도 계속 돌아야 한다."""
    fsm, link = _rotate_fsm("rotate_left", start_yaw_deg=mcfg.BOX_FACE_YAW_DEG)

    fsm.step(link.pose(), {}, link)

    cmds = [c for c, status in link.sent if status == "NUDGE_BOX"]
    assert cmds and cmds[0] == "yaw+", "정렬됐다고 보고 멈춰 버렸다"


def test_목표_회전량만큼_돌면_멈추고_PLACE로_넘어간다():
    fsm, link = _rotate_fsm("rotate_left", amount_rad=0.15)

    steps = _run(fsm, link)

    assert steps < MAX_STEPS
    cmds = {c for c, status in link.sent if status == "NUDGE_BOX"}
    # 회전만 썼다 — 전후·좌우 이동으로 새지 않았다.
    assert cmds <= {"yaw+", "stop"}


def test_회전판은_거리_기반_goal을_계산하지_않는다():
    """제자리 회전이라 위치가 안 바뀐다 — forward 축처럼 좌표를 밀어
    투영하면 안 된다(축 값(rad)을 거리(m)로 잘못 쓰면 goal이 엉뚱한 곳을
    가리킨다)."""
    fsm, link = _rotate_fsm("rotate_left", amount_rad=0.15)
    start_xy = (link.x, link.y)

    fsm.step(link.pose(), {}, link)

    assert fsm.last_nav is not None
    assert fsm.last_nav.waypoint == start_xy
