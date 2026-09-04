"""LIDAR_INSERT_CHECK_ENABLED=False일 때 NUDGE_BOX가 스스로 목표영역까지
계획을 세우는지 (2026-09-04, host+Pi 연동 실기 사고 회귀 방지).

실기 재현: Pi의 라이다 게이트를 끄고 나니, NUDGE_BOX가 옛 고정 5cm
넛지만 한 번 하고 바로 INSERT를 시도해 버렸다 — 예전엔 Pi가 "너무
멀다"고 거절하며 보정 계획을 돌려줘서 여러 번 왕복하며 좁혀졌는데,
그 거절 자체가 사라졌기 때문이다. 라이다 실측이 0.373m/0.215m/0.164m
로 제각각인데도(옛 목표 0.140m) 매번 정면에서 헛되이 투하됐다.

고친 뒤에는: 이 플래그가 꺼져 있으면 NUDGE_BOX 첫 진입 때 basket_target
게이트의 실제 남은 거리로 계획을 세운다 — 고정 5cm보다 훨씬 커야
정상이다(_box_front_xy가 이미 BOX_APPROACH_MARGIN_M=0.15m 여유를 두고
멈추게 만들어 두기 때문)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

import basket_target                                # noqa: E402
import mission_config as mcfg                       # noqa: E402
from mission import MissionFSM, State, _box_front_xy  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from conftest import PiSim                           # noqa: E402


def test_라이다_게이트가_꺼져있으면_목표거리로_계획을_세운다(monkeypatch):
    monkeypatch.setattr(mcfg, "LIDAR_INSERT_CHECK_ENABLED", False)

    # _box_front_xy가 멈추는 자리(BOX_APPROACH_MARGIN_M=0.15m 여유)가
    # 공교롭게도 게이트의 MAX_APPROACH_DIST_M(0.15m)과 정확히 같아서,
    # 그 자리에서 그대로 시작하면 게이트가 이미 통과 상태라 NUDGE_BOX가
    # 이번 사이클에 바로 PLACE로 넘어가 버린다(_nudge_plan이 세워지는
    # 걸 보기도 전에 리셋됨) — 실측 노이즈를 흉내내 그보다 0.2m 더
    # 짧게 와서 선 것으로 시작한다.
    front_xy = _box_front_xy("chess")
    heading_rad = math.radians(mcfg.BOX_FACE_YAW_DEG)
    dest_xy = (front_xy[0] - 0.20 * math.cos(heading_rad),
               front_xy[1] - 0.20 * math.sin(heading_rad))
    fsm = MissionFSM()
    fsm.state = State.NUDGE_BOX
    fsm.target_label = "rook"          # -> PIECE_DEST_BOX["rook"] == "chess"
    fsm.dest_xy = front_xy
    link = PiSim(x=dest_xy[0], y=dest_xy[1], yaw_deg=mcfg.BOX_FACE_YAW_DEG)

    fsm.step(link.pose(), {}, link)

    assert fsm.state == State.NUDGE_BOX, (
        f"게이트에서 아직 0.2m 먼데 벌써 {fsm.state.name}로 넘어갔다")
    assert fsm._nudge_plan is not None
    want_m, axis = fsm._nudge_plan
    assert axis == "forward"

    expected = basket_target.check_basket_insert_gate(
        dest_xy, mcfg.BOX_FACE_YAW_DEG, "chess").distance_m
    assert want_m == expected
    # 옛 고정 5cm 넛지보다 뚜렷하게 커야 한다 — 그래야 "한 번 찔끔 밀고
    # 바로 INSERT" 사고가 재현되지 않는다.
    assert want_m > mcfg.BOX_NUDGE_M * 2

    # 5cm(옛 고정값)만큼만 움직인 시점엔 아직 도착이 아니어야 한다.
    heading_rad = math.radians(mcfg.BOX_FACE_YAW_DEG)
    still_short_xy = (dest_xy[0] + mcfg.BOX_NUDGE_M * math.cos(heading_rad),
                       dest_xy[1] + mcfg.BOX_NUDGE_M * math.sin(heading_rad))
    link2 = PiSim(x=still_short_xy[0], y=still_short_xy[1], yaw_deg=mcfg.BOX_FACE_YAW_DEG)
    fsm.step(link2.pose(), {}, link2)
    assert fsm.state == State.NUDGE_BOX, (
        "5cm만 움직였는데 벌써 PLACE로 넘어갔다 — 옛 고정 5cm 넛지로 되돌아간 것으로 보인다")


def test_라이다_게이트가_켜져있어도_스스로_계획을_세운다(monkeypatch):
    """2026-09-05, 사용자 지시("라이다 뺀 상황으로 전제하고 다시 수정해")로
    바뀐 대조군 — 예전에는 플래그가 True(Pi가 라이다로 거절·보정)면 이
    자체 계획 로직이 끼어들지 않고 고정 5cm만 썼다. 그 5cm는 CARRY_TO_DEST가
    dest_xy(상자 중심에서 0.325m)에서 출발한다고 보던 옛 판정 기준이었다.
    지금은 CARRY_TO_DEST가 그보다 훨씬 가까운 접근 부채꼴(basket_target.
    SOUTH_APPROACH_SECTOR_RADIUS_M=0.15, 목표중심 기준)에서 바로 NUDGE_BOX로
    넘어오므로, 옛 5cm 전제가 더는 안 맞는다(그대로 두면 실측상 라이다가
    이미 바구니 앞면을 지나친 채 멈춘다) — 그래서 이 플래그 값과 무관하게
    항상 Host 스스로 목표영역까지 남은 거리로 계획을 세우도록 고쳤다. Pi
    라이다는 이제 그 위에 얹히는 보너스 보정일 뿐이다(mission.py NUDGE_BOX
    첫 진입 블록의 같은 날짜 주석 참고)."""
    monkeypatch.setattr(mcfg, "LIDAR_INSERT_CHECK_ENABLED", True)

    dest_xy = _box_front_xy("chess")
    fsm = MissionFSM()
    fsm.state = State.NUDGE_BOX
    fsm.target_label = "rook"
    fsm.dest_xy = dest_xy
    link = PiSim(x=dest_xy[0], y=dest_xy[1], yaw_deg=mcfg.BOX_FACE_YAW_DEG)

    fsm.step(link.pose(), {}, link)

    assert fsm._nudge_plan is not None
    want_m, axis = fsm._nudge_plan
    assert axis == "forward"
    expected = basket_target.check_basket_insert_gate(
        dest_xy, mcfg.BOX_FACE_YAW_DEG, "chess").distance_m
    assert want_m == max(mcfg.BOX_NUDGE_M, expected)
