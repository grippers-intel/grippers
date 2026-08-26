"""Host 속도 명령 해석의 계약을 고정한다 (팀 확정, 2026-08-26).

주행 안전의 마지막 한 겹이다. 여기서 새는 값은 그대로 바퀴로 간다."""

import pytest

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
