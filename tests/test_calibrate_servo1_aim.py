"""servo 1 조준 캘리브레이션의 계산부를 고정한다.

도구의 값어치는 **두 점 이상에서 부호(a)와 영점(b)을 갈라내는 것**에 있다.
한 점만 재면 둘이 섞여서 못 가른다 — 2026-09-06 의 눈대중 캘리브레이션이
0 -> 6.8 -> 4.5 로 두 번 반복하고도 1~2cm 를 남긴 이유가 그것이다.
"""

import importlib.util
import math
import pathlib

import pytest

_PATH = (pathlib.Path(__file__).resolve().parents[1]
         / "tools" / "arm" / "calibrate_servo1_aim.py")
_spec = importlib.util.spec_from_file_location("calibrate_servo1_aim", _PATH)
cal = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cal)


# ── 방위각 ────────────────────────────────────────────────────────────────


def test_좌우_부호를_그대로_따른다():
    """run_mission 의 `[파지 진입]` 줄과 같은 규약이어야 한다 — 그 줄을 보고
    사람이 숫자를 옮겨 적기 때문이다."""
    assert cal.bearing_deg(300.0, 0.0) == pytest.approx(0.0)
    assert cal.bearing_deg(300.0, 30.0) > 0      # 왼쪽
    assert cal.bearing_deg(300.0, -30.0) < 0     # 오른쪽


def test_방위각은_atan2다():
    assert cal.bearing_deg(300.0, 30.0) == pytest.approx(
        math.degrees(math.atan2(30.0, 300.0)))


def test_전방이_0이하면_거부한다():
    """기물이 뒤에 있다는 뜻이라 조준으로 풀 문제가 아니다."""
    with pytest.raises(ValueError):
        cal.bearing_deg(0.0, 10.0)


# ── 직선 맞추기 ───────────────────────────────────────────────────────────


def test_부호가_반대인_1대1을_되찾는다():
    """코드가 여태 가정만 하던 -θ 가 맞다면 이렇게 나와야 한다."""
    samples = [(t, -t + 4.5) for t in (-8.0, -3.0, 0.0, 3.0, 8.0)]

    a, b, rms, _resid = cal.fit_line(samples)

    assert a == pytest.approx(-1.0, abs=1e-6)
    assert b == pytest.approx(4.5, abs=1e-6)
    assert rms == pytest.approx(0.0, abs=1e-9)


def test_영점만_있고_기울기가_없는_경우도_구분한다():
    """servo 1 이 θ 와 무관하게 상수라면 a=0 이 나와야 한다 — 그때는 조준이
    아니라 고정 오프셋만 필요하다는 뜻이다."""
    samples = [(t, 4.5) for t in (-8.0, -3.0, 0.0, 3.0, 8.0)]

    a, b, _rms, _resid = cal.fit_line(samples)

    assert a == pytest.approx(0.0, abs=1e-6)
    assert b == pytest.approx(4.5, abs=1e-6)


def test_한_자리에서만_재면_거부한다():
    """⚠️ 이것이 이 도구의 존재 이유다. θ 가 전부 같으면 a 와 b 가 섞여
    무한히 많은 답이 나온다 — 조용히 아무 값이나 주는 대신 거부한다."""
    samples = [(3.0, 1.0), (3.0, 1.2), (3.0, 0.8)]

    with pytest.raises(ValueError, match="좌우로"):
        cal.fit_line(samples)


def test_점이_하나면_거부한다():
    with pytest.raises(ValueError):
        cal.fit_line([(3.0, -3.0)])


def test_흩어진_점도_직선을_주고_잔차로_알린다():
    """사람이 '정면으로 겨눴다'를 판정하므로 흔들린다. 값을 안 주는 것보다
    잔차를 같이 주는 편이 낫다."""
    samples = [(-8.0, 12.0), (-3.0, 8.0), (0.0, 4.0), (3.0, 2.0), (8.0, -4.0)]

    a, _b, rms, resid = cal.fit_line(samples)

    assert a < 0
    assert rms > 0.0
    assert len(resid) == len(samples)


def test_한계각은_운영_한계와_같은_값이다():
    """잰 값이 이보다 크면 조준으로 풀 문제가 아니다 —
    arm_driver.MAX_BASE_YAW_OFFSET_RAD 와 같은 값이어야 그 경고가 뜻을 갖는다."""
    assert cal.LIMIT_DEG == 15.0


# ── 손으로 맞추고 읽는 방식 (2026-09-07 사용자 지시) ──────────────────────


def test_좌우_사다리가_양쪽으로_충분히_벌어져_있다():
    """한쪽에만 몰리면 a 와 b 가 다시 섞인다. 기본 사다리 자체가 그 조건을
    만족해야 한다 — 사람이 매번 신경 쓰게 두면 안 된다."""
    thetas = [cal.bearing_deg(cal.DEFAULT_FORWARD_MM, v)
              for v in cal.LATERAL_LADDER_MM]

    assert min(thetas) < -6.0 and max(thetas) > 6.0
    assert max(thetas) - min(thetas) >= 12.0


def test_전방_거리가_파지_진입_거리와_같다():
    """다른 거리에서 재면 같은 mm 가 다른 각이 된다 —
    host/mission_config.GRASP_TRIGGER_DIST_M(0.32m) 과 맞춰 둔다."""
    assert cal.DEFAULT_FORWARD_MM == 320.0


def test_raw를_도로_바꾸는_환산이_driver_sdk와_같다():
    """4096 카운트 = 360도, 중앙 2048."""
    assert cal.raw_to_deg(cal.POS_CENTER) == 0.0
    assert cal.raw_to_deg(cal.POS_CENTER + 4095) == pytest.approx(360.0)
    assert cal.raw_to_deg(2066) == pytest.approx((2066 - 2048) / 4095 * 360)
