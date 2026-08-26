"""턱이 쓸고 갈 영역 판정의 계약을 고정한다 (사용자 지시, 2026-08-26).

여기서 지키려는 성질: **"가운데"가 아니라 "영역 안"이 통과 기준이고,
영역 안/밖이 누가 고치는지를 가른다.**"""

import math

import pytest

from domain.task import baseline_constants as bc
from domain.task import grasp_alignment as ga
from domain.values import TargetObservation

JAW_LINE_M = 0.36
SERVO1_REACH_MM = 240.0
TEST_LABEL = "테스트말"


@pytest.fixture(autouse=True)
def _measured_geometry(monkeypatch):
    # 실측 상수를 건드리지 않도록 가상의 클래스를 하나 넣어 쓴다 — 영점을
    # 0으로 두면 읽은 좌우 값이 그대로 중심선 기준 오차가 된다.
    monkeypatch.setattr(bc, "JAW_LINE_DEPTH_FORWARD_M",
                        {**bc.JAW_LINE_DEPTH_FORWARD_M, TEST_LABEL: JAW_LINE_M})
    monkeypatch.setattr(bc, "DEPTH_LATERAL_TO_JAW_CENTER_M",
                        {**bc.DEPTH_LATERAL_TO_JAW_CENTER_M, TEST_LABEL: 0.0})
    monkeypatch.setattr(bc, "SERVO1_AXIS_TO_JAW_MM", SERVO1_REACH_MM)


def _obs(forward_m=JAW_LINE_M + 0.02, lateral_m=0.0, metric_ok=True):
    return TargetObservation(TEST_LABEL, forward_m, lateral_m, metric_ok)


# ── 영역의 모양 ────────────────────────────────────────────────────────────


def test_좌우_허용치는_물체_폭만큼_줄어든다():
    """턱이 물체를 스치기만 하면 밀려 넘어진다 — 폭의 절반씩 뺀다."""
    narrow = ga.capture_half_width_m(17.0)
    wide = ga.capture_half_width_m(46.0)

    assert narrow == pytest.approx((bc.GRIPPER_OPEN_MM - 17.0) / 2000.0)
    assert wide < narrow


def test_열린_폭보다_넓은_물체는_들어올_수_없다():
    assert ga.capture_half_width_m(bc.GRIPPER_OPEN_MM + 10.0) == 0.0


def test_깊이_구간은_턱_선부터_전진_거리까지다():
    near, far = ga.capture_depth_range_m(TEST_LABEL)

    assert near == pytest.approx(JAW_LINE_M)
    assert far == pytest.approx(JAW_LINE_M + bc.GRASP_CREEP_FORWARD_MM / 1000.0)


# ── 세 갈래 판정 ───────────────────────────────────────────────────────────


def test_영역_안_중앙이면_그대로_내려간다():
    assert ga.judge(_obs(lateral_m=0.005), 17.0).action == ga.READY


def test_영역_안_치우침은_Pi가_고친다():
    verdict = ga.judge(_obs(lateral_m=0.040), 17.0)

    assert verdict.action == ga.PI_CENTER
    assert verdict.servo1_offset_rad == pytest.approx(
        math.atan2(0.040, SERVO1_REACH_MM / 1000.0))


def test_턱_폭_밖은_Host가_다시_세운다():
    verdict = ga.judge(_obs(lateral_m=0.090), 17.0)

    assert verdict.action == ga.HOST_CORRECTION
    assert "재회전" in verdict.reason


def test_전진_거리_밖은_재직진이다():
    far = JAW_LINE_M + bc.GRASP_CREEP_FORWARD_MM / 1000.0 + 0.05
    verdict = ga.judge(_obs(forward_m=far), 17.0)

    assert verdict.action == ga.HOST_CORRECTION
    assert "재직진" in verdict.reason


def test_턱_선보다_가까우면_후진이다():
    """이미 턱 선 안쪽이면 전진해도 안 들어오고 밀려날 뿐이다."""
    verdict = ga.judge(_obs(forward_m=JAW_LINE_M - 0.05), 17.0)

    assert verdict.action == ga.HOST_CORRECTION
    assert "후진" in verdict.reason


def test_보정_방향이_치우친_쪽을_따라간다():
    left = ga.judge(_obs(lateral_m=0.040), 17.0).servo1_offset_rad
    right = ga.judge(_obs(lateral_m=-0.040), 17.0).servo1_offset_rad

    assert left > 0.0 > right
    assert left == pytest.approx(-right)


# ── 모르면 실패 ────────────────────────────────────────────────────────────


def test_미터_환산_실패는_판정하지_않는다():
    """0.0을 그대로 쓰면 '바로 앞 정중앙'으로 읽혀 가장 위험한 쪽으로 틀린다."""
    assert ga.judge(_obs(metric_ok=False), 17.0).action == ga.UNKNOWN


def test_관측이_없으면_판정하지_않는다():
    assert ga.judge(None, 17.0).action == ga.UNKNOWN


def test_턱_선_미실측이면_판정하지_않는다(monkeypatch):
    monkeypatch.setattr(bc, "JAW_LINE_DEPTH_FORWARD_M", {})

    verdict = ga.judge(_obs(), 17.0)

    assert verdict.action == ga.UNKNOWN
    assert "JAW_LINE_DEPTH_FORWARD_M" in verdict.reason


def test_팔_길이_미실측이면_보정_대신_Host에_넘긴다(monkeypatch):
    """각도를 지어내면 엉뚱한 곳으로 턱을 돌린다."""
    monkeypatch.setattr(bc, "SERVO1_AXIS_TO_JAW_MM", None)

    verdict = ga.judge(_obs(lateral_m=0.040), 17.0)

    assert verdict.action == ga.HOST_CORRECTION
    assert "SERVO1_AXIS_TO_JAW_MM" in verdict.reason


def test_카메라_광축_어긋남을_먼저_지운다(monkeypatch):
    """카메라가 가운데가 아니면 보정이 늘 한쪽으로 치우친다."""
    monkeypatch.setattr(bc, "DEPTH_LATERAL_TO_JAW_CENTER_M", {TEST_LABEL: 0.040})

    assert ga.judge(_obs(lateral_m=0.040), 17.0).action == ga.READY


# ── 미세 전진 거리 ─────────────────────────────────────────────────────────


def test_전진_거리는_관측에서_나온다():
    """상수를 그대로 밀면 이미 가까운 물체를 턱 안쪽으로 처박는다."""
    assert ga.creep_distance_m(_obs(forward_m=JAW_LINE_M + 0.024)) == pytest.approx(0.024)


def test_전진_거리에_상한이_걸린다():
    """관측이 튀었을 때 크게 밀고 나가지 않게 한다."""
    far = JAW_LINE_M + 5.0
    assert ga.creep_distance_m(_obs(forward_m=far)) == pytest.approx(
        bc.GRASP_CREEP_FORWARD_MM / 1000.0)


def test_이미_턱_선_안쪽이면_전진하지_않는다():
    assert ga.creep_distance_m(_obs(forward_m=JAW_LINE_M - 0.01)) is None


def test_환산_실패면_전진_거리를_내지_않는다():
    assert ga.creep_distance_m(_obs(metric_ok=False)) is None


# ── 턱 선은 클래스마다 다르다 ─────────────────────────────────────────────


def test_안_잰_클래스는_판정하지_않는다(monkeypatch):
    """클래스별 K의 배율 오차가 커서 한 값을 공용하면 안 된다 —
    같은 물리 18cm를 queen 14.4 / soccer 25.6cm로 읽는다(2026-08-25 실측)."""
    monkeypatch.setattr(bc, "JAW_LINE_DEPTH_FORWARD_M", {"rook": 0.36})

    verdict = ga.judge(TargetObservation("queen", 0.38, 0.0, True), 17.0)

    assert verdict.action == ga.UNKNOWN
    assert "queen" in verdict.reason


def test_클래스마다_다른_턱_선을_쓴다(monkeypatch):
    monkeypatch.setattr(bc, "JAW_LINE_DEPTH_FORWARD_M",
                        {"queen": 0.30, "soccer": 0.50})

    assert ga.creep_distance_m(
        TargetObservation("queen", 0.32, 0.0, True)) == pytest.approx(0.02)
    assert ga.creep_distance_m(
        TargetObservation("soccer", 0.52, 0.0, True)) == pytest.approx(0.02)


# ── 실측된 rook 기하 (2026-08-26) ─────────────────────────────────────────


def test_rook_턱_선이_실측값이다():
    """0.1985에서도 물리기는 했지만 턱 끝에 걸려 미끄러졌다 — 목표는
    '물리는 자리'가 아니라 '제대로 앉는 자리'다."""
    assert bc.JAW_LINE_DEPTH_FORWARD_M["rook"] == 0.1757


def test_턱_끝에_걸리는_거리에서는_전진이_남아있다():
    """1차 실측 자리(0.1985)에서 판정하면 아직 전진해야 한다고 나와야 한다."""
    creep = ga.creep_distance_m(TargetObservation("rook", 0.1985, 0.0, True))

    assert creep == pytest.approx(0.0228, abs=0.0005)


def test_제대로_앉은_자리에서는_전진할_것이_없다():
    """이미 턱 선이면 전진 거리가 0 이하라 판정이 '너무 가깝다'로 간다."""
    assert ga.creep_distance_m(TargetObservation("rook", 0.1757, 0.0, True)) is None


# ── 좌우 영점은 클래스마다 다르다 ─────────────────────────────────────────


def test_좌우_영점을_안_잰_클래스는_판정하지_않는다(monkeypatch):
    monkeypatch.setattr(bc, "DEPTH_LATERAL_TO_JAW_CENTER_M", {"rook": 0.0295})

    verdict = ga.judge(_obs(lateral_m=0.0), 17.0)

    assert verdict.action == ga.UNKNOWN
    assert "좌우 영점" in verdict.reason


def test_실측된_좌우_영점이_클래스마다_다르다():
    """겉보기 좌우 값에 그 클래스의 거리 배율 오차가 실려 있다 —
    같은 물리 30mm를 rook 29.5 / queen 24.0 / knight 31.4로 읽었다."""
    zeros = bc.DEPTH_LATERAL_TO_JAW_CENTER_M

    assert zeros["rook"] == 0.0295
    assert zeros["queen"] == 0.0240
    assert zeros["knight"] == 0.0314


def test_영점을_빼면_중앙이_0이_된다(monkeypatch):
    """영점 자리에 놓인 물체는 '가운데'로 판정돼야 한다."""
    monkeypatch.setattr(bc, "JAW_LINE_DEPTH_FORWARD_M", {"rook": 0.1757})

    verdict = ga.judge(TargetObservation("rook", 0.1957, 0.0295, True), 24.5)

    assert verdict.action == ga.READY


def test_knight_턱_선이_실측값이다():
    """닫음/들어올림/CARRY 부하가 0.0782로 한 번도 안 떨어진 실측이다."""
    assert bc.JAW_LINE_DEPTH_FORWARD_M["knight"] == 0.1881


def test_같은_물리_자리를_클래스마다_다르게_읽는다():
    """턱 선은 정의상 모든 클래스에서 물리적으로 같은 자리다. 그런데 판독은
    7% 다르다 — 턱 선을 클래스마다 따로 두는 이유가 이것이다."""
    rook = bc.JAW_LINE_DEPTH_FORWARD_M["rook"]
    knight = bc.JAW_LINE_DEPTH_FORWARD_M["knight"]

    assert knight > rook
    assert 0.06 < knight / rook - 1.0 < 0.08
