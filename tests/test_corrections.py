"""Host가 기계적으로 실행할 수 있는 보정 요구 (2026-08-26).

2026-08-26 Host 저장소 대조에서 드러난 공백을 메운다 — Host 어휘에는
`FAILED`만 있고 "이렇게 고쳐 달라"가 없어서, 막힌 보고를 받아도 재시도밖에
못 한다. 재시도는 같은 자리에서 같은 이유로 또 막힌다.
"""

import pytest

from domain.task import baseline_constants as bc
from domain.task import corrections as cx
from domain.task import grasp_alignment as ga
from domain.task.preconditions import InsertInputs


def _insert(**overrides):
    base = dict(
        estop_set=False, base_stopped=True, gripper_load=0.07,
        face_ok=True, face_distance_m=bc.BASKET_STOP_LIDAR_M,
        face_yaw_error_rad=0.0, profile="chess_queen",
        face_point_count=90, face_lateral_offset_m=0.0, face_lateral_known=True,
        distance_change_m=0.001, load_change=0.0,
    )
    base.update(overrides)
    return InsertInputs(**base)


# ── 형식 ───────────────────────────────────────────────────────────────────

def test_모르는_동작은_거부한다():
    """오타 하나가 Host를 엉뚱하게 움직이게 두지 않는다."""
    with pytest.raises(ValueError, match="모르는 보정 동작"):
        cx.Correction("go_left")


def test_JSON으로_보낼_수_있다():
    fix = cx.Correction(cx.ADVANCE, forward_m=0.31)

    assert fix.as_dict() == {"action": "advance", "lateral_m": 0.0,
                             "forward_m": 0.31, "yaw_rad": 0.0}


# ── GRASP ──────────────────────────────────────────────────────────────────

def test_정렬이_맞으면_보정이_없다():
    assert cx.from_alignment(ga.AlignmentVerdict(ga.READY)) is None


def test_Pi가_스스로_고치는_중이면_기다리라고_한다():
    """Host가 이때 차를 움직이면 Pi의 servo 1 보정과 겹쳐 오히려 어긋난다."""
    verdict = ga.AlignmentVerdict(ga.PI_CENTER, lateral_error_m=0.03)

    fix = cx.from_alignment(verdict)

    assert fix.action == cx.WAIT
    assert fix.lateral_m == 0.03


def test_턱_폭_밖이면_재회전을_요구한다():
    verdict = ga.AlignmentVerdict(ga.HOST_CORRECTION, lateral_error_m=0.095)

    fix = cx.from_alignment(verdict)

    assert fix.action == cx.ROTATE
    assert fix.lateral_m == 0.095
    assert fix.forward_m == 0.0


def test_전진_거리_밖이면_전진을_요구한다():
    """좌우가 아니라 전후가 원인이면 Host가 할 일이 다르다."""
    verdict = ga.AlignmentVerdict(ga.HOST_CORRECTION, forward_error_m=0.04)

    fix = cx.from_alignment(verdict)

    assert fix.action == cx.ADVANCE
    assert fix.forward_m == pytest.approx(0.04)


def test_턱_선보다_가까우면_후진을_요구한다():
    verdict = ga.AlignmentVerdict(ga.HOST_CORRECTION, forward_error_m=-0.02)

    fix = cx.from_alignment(verdict)

    assert fix.action == cx.RETREAT
    assert fix.forward_m == pytest.approx(-0.02)


def test_판정_불가면_다시_보이게_해달라고_한다():
    """UNKNOWN은 '고칠 방향을 모른다'이므로 수치를 지어내지 않는다."""
    fix = cx.from_alignment(ga.AlignmentVerdict(ga.UNKNOWN))

    assert fix.action == cx.REACQUIRE
    assert fix.lateral_m == 0.0 and fix.forward_m == 0.0


# ── INSERT ─────────────────────────────────────────────────────────────────

def test_조건이_다_맞으면_보정이_없다():
    assert cx.from_insert(_insert()) is None


def test_바구니가_멀면_전진량을_준다():
    far = bc.BASKET_STOP_LIDAR_M + 0.30
    fix = cx.from_insert(_insert(face_distance_m=far))

    assert fix.action == cx.ADVANCE
    assert fix.forward_m == pytest.approx(0.30)


def test_절벽_아래는_전진이_아니라_후진이다():
    """약 0.125m에서 빔이 테두리를 넘어 바구니를 놓친다. 그 아래로는 판독이
    **커지는 방향으로** 틀리므로, 더 붙이면 상황이 나빠지기만 한다."""
    fix = cx.from_insert(_insert(face_distance_m=bc.BASKET_MIN_LIDAR_M - 0.005))

    assert fix.action == cx.RETREAT
    assert fix.forward_m < 0


def test_정면을_못_잡으면_다시_세워달라고_한다():
    fix = cx.from_insert(_insert(face_ok=False))

    assert fix.action == cx.REACQUIRE


def test_거리가_맞고_yaw만_틀어지면_회전을_요구한다():
    fix = cx.from_insert(_insert(face_yaw_error_rad=0.20))

    assert fix.action == cx.ROTATE
    assert fix.yaw_rad == pytest.approx(0.20)


def test_좌우로_밀렸으면_회전을_요구한다():
    """사선 진입이면 거리와 yaw가 멀쩡해도 팔이 바구니 끝을 벗어난 곳을
    겨눌 수 있다(사용자 지시 2026-08-26)."""
    offset = bc.BASKET_LATERAL_TOLERANCE_M + 0.02
    fix = cx.from_insert(_insert(face_lateral_offset_m=offset))

    assert fix.action == cx.ROTATE
    assert fix.lateral_m == pytest.approx(offset)


def test_Host가_못_고치는_미충족에는_보정을_안_준다():
    """점 개수·판독 안정성·부하는 서 있는 자리 문제가 아니다. 지어낸 보정을
    주면 Host가 엉뚱하게 움직인다."""
    assert cx.from_insert(_insert(face_point_count=3)) is None
    assert cx.from_insert(_insert(distance_change_m=-0.02)) is None
    assert cx.from_insert(_insert(load_change=-0.05)) is None
