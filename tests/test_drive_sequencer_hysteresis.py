"""주행 방위 제어가 목표 근처에서 떨지 않는지 지킨다.

## 왜 필요한가

DriveSequencer 는 FORWARD / STOP / ROTATE 를 오가는 on-off 제어다. 회전을
끝내는 기준과 직진을 그만두는 기준이 같은 값이면, 목표 방위 근처에서
정지<->회전을 반복하며 좌우로 떤다.

2026-09-05 실기에서 그게 보였다 — 차가 좌우로 흔들리며 앞으로 갔다.
회전 속도를 0.25(제자리에서 안 돌던 값)에서 0.6 으로 올리자 회전이 실제로
되기 시작하면서 드러났다. 그 전에는 ROTATE 가 아무 일도 안 해서 떨림도
없었다.

지연을 넣은 모의 주행으로 재 봤을 때(실측 회전 20도/s, 지연 300~500ms)
2m 한 번 가는 동안 회전 전환이 19~52회였고, 이력을 넣으면 3~7회가 됐다.

이 파일은 그 이력이 살아 있는지만 본다. 모의 주행 전체를 테스트로 옮기지
않은 이유는 그 결과가 지연·잡음 모형에 민감해서, 상태 기계의 계약이 아니라
모형을 검증하게 되기 때문이다.
"""

import importlib.util
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
HOST = ROOT / "host"


def _load_navigator():
    pytest.importorskip("numpy")
    if str(HOST) not in sys.path:
        sys.path.insert(0, str(HOST))
    spec = importlib.util.spec_from_file_location(
        "host_navigator", HOST / "navigator.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _step(seq, yaw_err_deg):
    """로봇을 원점·yaw 0 에 두고, 목표를 원하는 방위각에 놓아 한 번 돌린다."""
    rad = math.radians(yaw_err_deg)
    target = (2.0 * math.cos(rad), 2.0 * math.sin(rad))
    return seq.update((0.0, 0.0), 0.0, target, []).mode


def test_들어가는_문턱이_나오는_문턱보다_넓다():
    nav = _load_navigator()
    seq = nav.DriveSequencer()

    assert seq.yaw_enter_deg > seq.yaw_tolerance_deg


def test_직진_중_작은_오차로는_회전으로_안_돌아간다():
    """허용치는 넘었지만 들어가는 문턱 아래인 오차. 여기서 회전으로 넘어가면
    오버슛 때문에 반대편으로 넘어가고, 그게 좌우 떨림이 된다."""
    nav = _load_navigator()
    seq = nav.DriveSequencer()

    assert _step(seq, 0.0) == nav.DriveMode.FORWARD          # 정렬된 채 시작
    between = (seq.yaw_tolerance_deg + seq.yaw_enter_deg) / 2.0
    for _ in range(5):
        assert _step(seq, between) == nav.DriveMode.FORWARD


def test_크게_틀어지면_정지를_거쳐_회전한다():
    nav = _load_navigator()
    seq = nav.DriveSequencer()

    assert _step(seq, 0.0) == nav.DriveMode.FORWARD
    big = seq.yaw_enter_deg + 3.0
    # 전이는 이번 사이클을 내보낸 **뒤에** 일어난다 — 그래서 한 번 더 돌린다.
    assert _step(seq, big) == nav.DriveMode.FORWARD
    assert _step(seq, big) == nav.DriveMode.STOP
    assert _step(seq, big) == nav.DriveMode.ROTATE


def test_회전은_좁은_허용치_안에_들어와야_끝난다():
    """나올 때는 좁게 — 넓게 잡으면 정렬이 덜 된 채 직진해 경로가 휜다."""
    nav = _load_navigator()
    seq = nav.DriveSequencer()

    assert _step(seq, 0.0) == nav.DriveMode.FORWARD
    big = seq.yaw_enter_deg + 3.0
    for _ in range(3):
        _step(seq, big)                       # FORWARD -> STOP -> ROTATE

    between = (seq.yaw_tolerance_deg + seq.yaw_enter_deg) / 2.0
    assert _step(seq, between) == nav.DriveMode.ROTATE       # 아직 안 끝난다
    assert _step(seq, 0.0) == nav.DriveMode.ROTATE           # 이번 사이클까지 회전
    assert _step(seq, 0.0) == nav.DriveMode.STOP
    assert _step(seq, 0.0) == nav.DriveMode.FORWARD
