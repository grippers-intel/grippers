"""NUDGE_BOX의 axis="back" 계획이 live_too_close 때문에 시작하자마자 스스로
끝나 버리던 버그 (2026-09-05 실기).

## 왜 이 버그가 생겼나

`_plan_basket_fix`는 라이다가 안전 최소거리보다 가까우면(`retreat_if_too_close`)
axis="back" 계획을 낸다 — "너무 가까우니 물러나라"는 뜻이다. 그런데
NUDGE_BOX의 done 판정에서 axis="back"도 다른 축(forward/left/right)과
똑같이 `live_too_close`(Pi가 매 사이클 실어 보내는 "아직도 너무 가깝다"
라이브 신호)를 "끝났다"로 셌다. back을 계획하는 이유 자체가 "너무
가깝다"이므로, back이 막 시작되는 그 사이클에 이미 live_too_close=True라
`moved`가 0인 채로 즉시 done=True가 나 버렸다 — "back" cmd를 단 한 번도
내보내지 못하고 곧바로 PLACE로 되돌아갔다.

실기(2026-09-05)에서 요(yaw)가 크게 틀어져(0.2~0.48rad) 라이다가 바구니
테두리를 비스듬히 봐서 실제보다 가깝게 잘못 읽히는 상황이 벌어졌고, 이
버그 때문에 154회 연속 NUDGE_BOX<->PLACE만 오가며 완전히 멎었다(사용자
보고 — "파지 미세전진 제대로 안됨, INSERT 버그").
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "host"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "host" / "aruco"))

import mission_config as mcfg                                    # noqa: E402
from localizer import box_pose                                    # noqa: E402
from mission import MissionFSM, State                              # noqa: E402
from vehicle_link import BasketFix                                 # noqa: E402

from conftest import PiSim                                         # noqa: E402

TOY_X, TOY_Y, _TOY_YAW = box_pose("toy")


def _fsm_backing_away(forward_m: float) -> tuple[MissionFSM, PiSim]:
    """toy 바구니 정면에서 이미 안전 최소거리보다 가까워(forward_m<0)
    axis="back" 계획을 실행 중인 상황을 만든다. 하드스톱 반경 밖에서
    벌어지는 상황임을 분명히 하려고 정상 INSERT 거리 근처에 둔다 —
    이 버그는 hard_stop과 무관하게 live_too_close 하나만으로도 난다."""
    fsm = MissionFSM()
    fsm.state = State.NUDGE_BOX
    fsm.target_label = "box"   # mission_config.PIECE_DEST_BOX["box"] == "toy"
    fsm._nudge_plan = (abs(forward_m), "back")
    robot_y = TOY_Y - mcfg.BASKET_TARGET_LIDAR_M
    link = PiSim(x=TOY_X, y=robot_y, yaw_deg=mcfg.BOX_FACE_YAW_DEG)
    link.last_basket_fix = BasketFix(forward_m=forward_m)
    return fsm, link


def test_너무_가까워_후진을_시작한_순간_바로_끝났다고_보면_안_된다():
    """live_too_close(아직도 너무 가깝다)는 back을 계획한 바로 그 이유다 —
    시작하자마자 그걸로 끝났다고 보면 실제로는 단 한 번도 물러나지 못한다."""
    fsm, link = _fsm_backing_away(forward_m=-0.02)

    fsm.step(link.pose(), {}, link)

    cmds = {c for c, status in link.sent if status == "NUDGE_BOX"}
    assert cmds == {"back"}, (
        "live_too_close 때문에 back을 한 번도 못 내보내고 곧바로 멈췄다")
    assert fsm.state == State.NUDGE_BOX, (
        "back을 실행하기도 전에 PLACE로 돌아가 버렸다")


def test_충분히_물러나면_그때는_정상적으로_끝난다():
    """버그를 고친다고 back이 영원히 안 끝나면 안 된다 — moved가 계획한
    만큼 쌓이면 정상적으로 완료돼야 한다."""
    fsm, link = _fsm_backing_away(forward_m=-0.02)
    fsm._nudge_from = (link.x, link.y)
    fsm._nudge_yaw_from = link.yaw_deg
    fsm._nudge_best = 0.0
    fsm._nudge_gate_streak = 0
    import time
    fsm._nudge_stall_at = time.monotonic() + 999.0
    fsm._nudge_stall_warned = False
    # 계획 거리를 이미 다 물러난 것처럼 로봇 위치를 뒤로 옮겨 둔다.
    link.y += fsm._nudge_plan[0] + 0.01

    fsm.step(link.pose(), {}, link)

    cmds = {c for c, status in link.sent if status == "NUDGE_BOX"}
    assert cmds == {"stop"}, "충분히 물러났는데도 계속 후진했다"
    assert fsm.state == State.PLACE, "충분히 물러났는데도 PLACE로 안 돌아갔다"
