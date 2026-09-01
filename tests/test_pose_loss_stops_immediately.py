"""포즈를 잃는 순간 이동 명령이면 즉시 stop을 보낸다 (2026-09-02 실기).

## 왜 이 기능이 생겼나

09-02 실기: NUDGE_BOX로 바구니에 접근하던 중 ArUco 마커를 놓쳐
`pose.ok`가 False가 됐다. 그때까지 `MissionFSM.step()`은 pose가 없으면
**아무 명령도 보내지 않고** 그냥 이번 사이클을 건너뛰었다 — "마지막 좌표로
계속 가면 안 된다"는 의도였지만, 아무 것도 안 보내는 것과 정지를 보내는
것은 다르다. 차량 쪽은 새 명령이 올 때까지 마지막 명령("go")을 그대로
래치해 두므로, Pi의 워치독(HOST_COMMAND_TIMEOUT_CYCLES, 6사이클 ≈0.4초)이
대신 멈출 때까지 블라인드로 계속 전진했다 — 그 구간에 바구니와 충돌했다.

"stop"은 좌표 계산이 필요 없는 명령이라(HostCommand에는 애초에 좌표가
없다 — vehicle_link.py 참고) pose 없이도 낼 수 있다. 이 파일은 그 새
동작만 검증한다 — 정지 상태(last_cmd가 None/"stop")에서 포즈를 잃으면
여전히 아무 것도 안 보내는지(불필요한 stop 스팸 방지)도 함께 본다."""

from __future__ import annotations

import sys
from pathlib import Path

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

from localizer import Pose                       # noqa: E402
from mission import MissionFSM, State            # noqa: E402

from conftest import PiSim                        # noqa: E402

_LOST_POSE = Pose(x=1.0, y=0.6, yaw_deg=30.0, ok=False, n_cams=0, fresh=False)


def test_전진_중_포즈를_잃으면_즉시_정지를_보낸다():
    fsm = MissionFSM()
    link = PiSim()

    fsm.state = State.NUDGE_BOX
    fsm.last_cmd = "go"

    sent_before = len(link.sent)
    fsm.step(_LOST_POSE, {}, link)

    assert len(link.sent) == sent_before + 1
    assert link.sent[-1][0] == "stop"
    assert fsm.last_cmd == "stop"
    # 상태 자체는 안 바뀐다 — 정지만 내고 판단은 포즈가 돌아온 뒤로 미룬다.
    assert fsm.state == State.NUDGE_BOX


def test_회전_중_포즈를_잃어도_즉시_정지를_보낸다():
    fsm = MissionFSM()
    link = PiSim()

    fsm.state = State.APPROACH_PIECE
    fsm.last_cmd = "yaw-"

    fsm.step(_LOST_POSE, {}, link)

    assert link.sent[-1][0] == "stop"


def test_이미_정지_중이면_포즈를_잃어도_또_보내지_않는다():
    """차 있고 GRASP 중처럼 원래도 계속 stop을 보내는 상태에서 포즈까지
    잃었다고 매 사이클 stop을 또 보낼 필요는 없다 — 스팸만 늘어난다."""
    fsm = MissionFSM()
    link = PiSim()

    fsm.state = State.GRASP
    fsm.last_cmd = "stop"

    sent_before = len(link.sent)
    fsm.step(_LOST_POSE, {}, link)

    assert len(link.sent) == sent_before


def test_처음부터_포즈가_없으면_아무_것도_보내지_않는다():
    """last_cmd가 아직 None인 시작 시점(미션 시작 전)에는 정지할 이동
    자체가 없었으므로 stop을 보낼 이유가 없다."""
    fsm = MissionFSM()
    link = PiSim()

    assert fsm.last_cmd is None
    fsm.step(_LOST_POSE, {}, link)

    assert link.sent == []
