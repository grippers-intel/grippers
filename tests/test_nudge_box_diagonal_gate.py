"""NUDGE_BOX가 새 목표영역 게이트로 사선 진입을 조기 종료하는지 (2026-09-04,
사용자 지시).

`basket_target.check_basket_insert_gate()`가 이미 통과라고 답하는
위치·방향이면, 기존처럼 BOX_FACE_YAW_DEG(90도) 정면 정렬이나 남은
계획거리(want_m)를 다 채우기를 기다리지 않고 즉시 PLACE로 넘어가야
한다 — 최종 확인은 여전히 Pi의 check_insert가 한다. 이 조기종료는
**전후/좌우 축에만** 적용된다 — 회전(rotate) 축은 아래 참고.

⚠️ 2026-09-04 밤: 이 조기종료는 LIDAR_INSERT_CHECK_ENABLED가 꺼져 있을
때만(=Pi가 라이다로 거절·보정해 주지 않을 때만) 걸리도록 바꿨다 — 라이다가
켜져 있는데도 이 느슨한 Host 판정만으로 회전/좌우까지 건너뛰다가 toy
바구니 투하가 입구 밖으로 나가는 사고가 났다(mission.py의 gate_ok 관련
정정 커밋 참고). 그래서 이 파일의 시험은 그 스위치를 명시적으로 꺼 두고
검증한다.

⚠️ 2026-09-05: 회전 축까지 이 게이트를 라이다 스위치와 무관하게 쓰도록
분리해 보려 했으나(사선이면 정면까지 안 돌아도 되게), NUDGE_BOX 회전
진입 시점엔 로봇이 이미 대략 바구니 쪽을 보고 있는 게 보통이라 그
시도가 Pi의 정밀 라이다 정렬을 시작하기도 전에 끝내 버려 2026-09-03
사고(test_nudge_box_rotate_fix.py)와 같은 모양으로 재발했다 — 되돌렸다.
같은 날, 게이트만으로 PLACE 전환을 확정하기 전에 "실제로 멈춘 뒤 다시
재본 좌표"인지 두 사이클 연속으로 확인하는 debounce도 추가됐다
(mcfg.NUDGE_GATE_CONFIRM_CYCLES, mission.py의 host_gate_hit 참고) — 아래
테스트가 `fsm.step()`을 그 횟수만큼 반복 호출하는 이유다."""

from __future__ import annotations

import math
import sys
from pathlib import Path

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

import basket_target as bt         # noqa: E402
import config as cfg               # noqa: E402
import mission_config as mcfg      # noqa: E402
from mission import MissionFSM, State   # noqa: E402
from localizer import box_pose     # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from conftest import PiSim          # noqa: E402


def _edge_center(box_name: str) -> tuple[float, float]:
    bx, by, _ = box_pose(box_name)
    edge_y = by - cfg.BOX_L / 2.0
    if box_name == "chess":
        edge_y += mcfg.CHESS_APPROACH_EXTRA_DEPTH_M
    return bx, edge_y


def test_45도_사선에서_게이트가_통과하면_정면정렬_없이_바로_PLACE로(monkeypatch):
    monkeypatch.setattr(mcfg, "LIDAR_INSERT_CHECK_ENABLED", False)
    ex, ey = _edge_center("chess")
    dist = 0.12
    robot_xy = (ex - dist * math.sin(math.radians(45.0)),
                ey - dist * math.cos(math.radians(45.0)))
    dx, dy = ex - robot_xy[0], ey - robot_xy[1]
    yaw_deg = math.degrees(math.atan2(dy, dx))   # 목표 중심을 정면으로 바라본다

    # 전제 확인 — 이 자리가 실제로 정면(90도)에서 45도 넘게 틀어져 있어야
    # "정면 정렬 없이" 라는 주장이 의미가 있다.
    assert abs(((yaw_deg - 90.0 + 180.0) % 360.0) - 180.0) > 30.0
    gate = bt.check_basket_insert_gate(robot_xy, yaw_deg, "chess")
    assert gate.ok, gate.reason

    fsm = MissionFSM()
    fsm.state = State.NUDGE_BOX
    fsm.target_label = "rook"                 # -> PIECE_DEST_BOX["rook"] == "chess"
    fsm.dest_xy = (ex, ey - 0.30)
    # 남은 계획 거리를 크게 줘서, 게이트가 없다면 이번 사이클엔 절대
    # "다 갔다"고 안 나올 상황을 만든다.
    fsm._nudge_plan = (0.30, "forward")
    link = PiSim(x=robot_xy[0], y=robot_xy[1], yaw_deg=yaw_deg, box="chess")

    # 2026-09-05: Host 게이트만으로 끝났다고 볼 때는 "정지 명령을 막 보낸
    # 순간의 좌표"가 아니라 "실제로 멈춘 뒤 다시 재본 좌표"로 확정한다
    # (mcfg.NUDGE_GATE_CONFIRM_CYCLES). 첫 사이클은 정지만 하고, 같은
    # 자리에서 한 번 더 게이트가 통과해야 PLACE로 넘어간다 — 첫 사이클에
    # 이미 "stop"을 보내므로 PiSim은 움직이지 않고 같은 좌표를 유지한다.
    for _ in range(mcfg.NUDGE_GATE_CONFIRM_CYCLES):
        fsm.step(link.pose(), {}, link)

    assert fsm.state == State.PLACE, (
        f"게이트가 통과인데도 PLACE로 안 넘어갔다 — state={fsm.state.name}")


def test_게이트가_실패면_기존대로_계속_다듬는다():
    """대조군 — 목표영역에서 멀거나 안 보고 있으면 게이트가 조기종료를
    유발하면 안 된다(기존 동작 보존 확인)."""
    ex, ey = _edge_center("chess")
    robot_xy = (ex, ey - 0.40)   # 40cm — 게이트 반경(15cm) 밖
    yaw_deg = 90.0

    gate = bt.check_basket_insert_gate(robot_xy, yaw_deg, "chess")
    assert not gate.ok

    fsm = MissionFSM()
    fsm.state = State.NUDGE_BOX
    fsm.target_label = "rook"
    fsm.dest_xy = (ex, ey - 0.30)
    fsm._nudge_plan = (0.30, "forward")
    link = PiSim(x=robot_xy[0], y=robot_xy[1], yaw_deg=yaw_deg, box="chess")

    fsm.step(link.pose(), {}, link)

    assert fsm.state == State.NUDGE_BOX
    assert fsm.last_cmd == "go"
