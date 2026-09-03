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
from vehicle_link import BasketFix                                # noqa: E402

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


def test_거리가_이미_목표창_안이어도_회전판은_그것만으로_끝나지_않는다():
    """2026-09-03 실기(rook/box, 두 바구니 다) — 진짜 원인.

    APPROACH_BOX_READY(basket_ready_early)는 Pi의 "라이다 거리가 이미
    목표창 안"이라는 **전후 축** 신호일 뿐인데, 회전판의 "끝났다" 판정에도
    그대로 섞여 있었다. NUDGE_BOX가 바구니 가까이 붙은 뒤에는 이 신호가
    사실상 항상 참이라, 회전판을 새로 계획해 NUDGE_BOX에 들어가는 바로 그
    첫 사이클에 "이미 끝났다"고 오판하고 즉시 PLACE로 돌아갔다 — 실제
    회전은 거의 안 하고서. want_m(~0.1rad)을 AGREED_ROTATION_RAD_S로
    돌면 최소 0.4초는 걸려야 하는데, 실기 로그의 NUDGE_BOX→PLACE 왕복은
    0.1~0.2초 만에 끝나 있었다 — 이 테스트가 바로 그 오판을 재현한다."""
    fsm, link = _rotate_fsm("rotate_left", amount_rad=0.15)
    link.basket_ready_early = True   # Pi가 조금 전 "거리는 이미 됐다"고 알려온 상태

    fsm.step(link.pose(), {}, link)

    cmds = [c for c, status in link.sent if status == "NUDGE_BOX"]
    assert cmds and cmds[0] == "yaw+", (
        "거리 신호(ready_early)만으로 회전판을 끝내 버렸다 — 실제로는 거의 안 돌았다")
    assert fsm.state == State.NUDGE_BOX


def test_ArUco상_계획량을_다_돌아도_Pi_라이다가_안_맞으면_안_끝난다():
    """2026-09-03 실기(toy/box): ArUco로 잰 회전량(moved)이 want_m을
    채워도, Pi 라이다가 보는 정렬(check_insert가 실제로 판정하는 기준)이
    그대로면 "끝났다"로 치면 안 된다 — 그렇게 치면 다음 PLACE에서 똑같은
    크기의 INSERT_BLOCKED가 반복된다(실측: yaw 오차가 수십 번 보정에도
    전혀 안 줄었다)."""
    fsm, link = _rotate_fsm("rotate_left", amount_rad=0.05)
    link.last_basket_fix = BasketFix(yaw_rad=0.15)   # 여전히 크게 어긋나 있다

    for _ in range(5):   # 5스텝이면 ArUco상 moved(약 0.09rad)가 want_m(0.05)을 넘는다
        fsm.step(link.pose(), {}, link)

    assert fsm.state == State.NUDGE_BOX, "Pi 라이다가 안 맞는데 PLACE로 넘어갔다"


def test_Pi_라이다가_먼저_맞으면_ArUco_계획량_전에도_끝난다():
    """반대 방향 회귀도 지킨다 — Pi가 이미 맞다고 하면 ArUco 계획량을 다
    안 돌았어도 과잉 회전 없이 바로 멈춘다."""
    fsm, link = _rotate_fsm("rotate_left", amount_rad=0.5)
    link.last_basket_fix = BasketFix(yaw_rad=0.01)   # 이미 데드밴드 안

    fsm.step(link.pose(), {}, link)

    assert fsm.state == State.PLACE


def test_회전판은_거리_기반_goal을_계산하지_않는다():
    """제자리 회전이라 위치가 안 바뀐다 — forward 축처럼 좌표를 밀어
    투영하면 안 된다(축 값(rad)을 거리(m)로 잘못 쓰면 goal이 엉뚱한 곳을
    가리킨다)."""
    fsm, link = _rotate_fsm("rotate_left", amount_rad=0.15)
    start_xy = (link.x, link.y)

    fsm.step(link.pose(), {}, link)

    assert fsm.last_nav is not None
    assert fsm.last_nav.waypoint == start_xy
