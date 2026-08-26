"""Host 속도 명령 해석의 계약을 고정한다 (팀 확정, 2026-08-26).

주행 안전의 마지막 한 겹이다. 여기서 새는 값은 그대로 바퀴로 간다."""

import pytest

from domain.task import motion as mo
from domain.ports.baseline_ports import HostCommand, MissionState
from domain.task.motion import (
    AGREED_LINEAR_MPS,
    AGREED_ROTATION_RAD_S,
    resolve_motion,
)


def _command(**kwargs):
    return HostCommand(state=MissionState.APPROACH, **kwargs)


def test_합의된_속도는_그대로_통과한다():
    decision = resolve_motion(_command(linear_x=AGREED_LINEAR_MPS))

    assert decision.ok
    assert decision.motion.linear_x == AGREED_LINEAR_MPS


@pytest.mark.parametrize("sent,expected", [
    (1.0, AGREED_LINEAR_MPS),
    (-1.0, -AGREED_LINEAR_MPS),
    (0.05, 0.05),          # 합의보다 느린 것은 자르지 않는다
])
def test_직진은_크기만_자르고_부호는_지킨다(sent, expected):
    decision = resolve_motion(_command(linear_x=sent))

    assert decision.ok
    assert decision.motion.linear_x == pytest.approx(expected)


def test_수평이동도_같은_한계를_쓴다():
    """팀 합의: linear.x와 linear.y가 같은 0.1이다."""
    decision = resolve_motion(_command(linear_y=-5.0))

    assert decision.motion.linear_y == -AGREED_LINEAR_MPS


def test_제자리회전은_별도_한계를_쓴다():
    decision = resolve_motion(_command(angular_z=3.0))

    assert decision.motion.angular_z == AGREED_ROTATION_RAD_S


def test_제자리정지가_다른_모든_필드를_이긴다():
    """정지 의도가 다른 필드의 잔여값에 지면 안 된다."""
    decision = resolve_motion(
        _command(linear_x=0.1, linear_y=0.1, angular_z=0.25, stop=True))

    assert decision.ok
    assert decision.motion.is_stop


def test_직진과_수평이동은_함께_와도_된다():
    """메카넘에서 대각선 이동은 한 동작이다 — '제자리'라는 단서가 붙은 건 회전뿐."""
    decision = resolve_motion(_command(linear_x=0.1, linear_y=0.1))

    assert decision.ok
    assert decision.motion.linear_x == AGREED_LINEAR_MPS
    assert decision.motion.linear_y == AGREED_LINEAR_MPS


def test_제자리회전에_병진이_섞이면_거부한다():
    """합의된 네 가지 명령 중 무엇도 아니다 — 추측해 실행하지 않는다."""
    decision = resolve_motion(_command(linear_x=0.1, angular_z=0.25))

    assert not decision.ok
    assert decision.motion.is_stop
    assert "제자리회전" in decision.reason


def test_부동소수_잡음은_회전으로_읽지_않는다():
    """UDP+JSON을 거치며 0.0이 1e-17로 오는 경우가 있다."""
    decision = resolve_motion(_command(linear_x=0.1, angular_z=1e-17))

    assert decision.ok
    assert decision.motion.angular_z == 0.0


def test_명령이_없으면_거부하고_정지한다():
    decision = resolve_motion(None)

    assert not decision.ok
    assert decision.motion.is_stop


# ── 바구니 접근 구간 속도 상한 (2026-08-26) ────────────────────────────────
# 지연이 허용폭보다 크다는 계산이 근거다. Host가 "멈춰"라고 판단한 순간부터
# 바퀴가 실제로 서기까지 235ms가 쌓이는데(Host 루프 125 + 링크 10 + Pi
# 사이클 100), 0.1 m/s면 그동안 23.5mm를 더 간다. INSERT 허용폭은 ±15mm다.

def test_APPROACH_BOX에서는_더_느리게_자른다():
    """0.1로 오면 0.06으로 실행된다 — 그러지 않으면 오버슈트가 창을 넘는다."""
    decision = resolve_motion(
        HostCommand(state=MissionState.APPROACH_BOX, linear_x=0.1))

    assert decision.ok
    assert decision.motion.linear_x == pytest.approx(mo.BASKET_APPROACH_MPS)


def test_낮췄다는_사실을_사유에_남긴다():
    """Host는 0.1을 보냈는데 0.06으로 도는 것을 모르면 안 된다."""
    decision = resolve_motion(
        HostCommand(state=MissionState.APPROACH_BOX, linear_x=0.1))

    assert "낮췄다" in decision.reason


def test_이미_느리면_그대로_둔다():
    """Host가 스스로 0.06을 보냈으면 손대지 않고, 사유도 안 남긴다."""
    decision = resolve_motion(
        HostCommand(state=MissionState.APPROACH_BOX, linear_x=0.06))

    assert decision.motion.linear_x == pytest.approx(0.06)
    assert decision.reason == ""


def test_다른_상태는_안_낮춘다():
    """주행 구간까지 느리게 하면 시연이 하염없이 길어진다."""
    for state in (MissionState.APPROACH, MissionState.CARRY):
        decision = resolve_motion(HostCommand(state=state, linear_x=0.1))

        assert decision.motion.linear_x == pytest.approx(mo.AGREED_LINEAR_MPS)


def test_회전은_안_낮춘다():
    """회전은 한 사이클에 1.8도라 이미 허용치(5도)의 3분의 1이다."""
    decision = resolve_motion(
        HostCommand(state=MissionState.APPROACH_BOX, angular_z=0.25))

    assert decision.motion.angular_z == pytest.approx(mo.AGREED_ROTATION_RAD_S)


def test_상한이_데드밴드_위에_있다():
    """0.05 아래는 아무리 오래 줘도 안 움직인다 — 낮출 수 있는 바닥이다."""
    assert mo.BASKET_APPROACH_MPS > 0.05


def test_상태_문자열이_포트와_같다():
    """motion은 순환 import를 피하려고 문자열을 직접 들고 있다. 둘이
    갈라지면 상한이 조용히 안 걸린다."""
    assert mo._APPROACH_BOX == MissionState.APPROACH_BOX
