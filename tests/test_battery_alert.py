"""battery_alert — 순수 계산만 검증한다. ROS도 하드웨어도 모른다."""

from __future__ import annotations

from domain.task import battery_alert as ba


def test_문턱_위에서는_경고_없음():
    state = ba.BatteryAlertState()
    state, cmd = ba.check_battery(8000, now=0.0, state=state)
    assert state.warning_active is False
    assert cmd is None


def test_문턱_아래로_내려가면_바로_한번_울린다():
    state = ba.BatteryAlertState()
    state, cmd = ba.check_battery(ba.WARN_MV - 1, now=0.0, state=state)
    assert state.warning_active is True
    assert cmd is not None
    assert cmd.freq == ba.BUZZER_FREQ_HZ
    assert cmd.on_time == ba.BUZZER_ON_S
    assert cmd.repeat == ba.BUZZER_REPEAT


def test_최소_간격_안에는_다시_안_울린다():
    state = ba.BatteryAlertState()
    state, cmd1 = ba.check_battery(ba.WARN_MV - 1, now=0.0, state=state)
    assert cmd1 is not None
    state, cmd2 = ba.check_battery(ba.WARN_MV - 1, now=ba.MIN_REPEAT_INTERVAL_S / 2, state=state)
    assert cmd2 is None
    assert state.warning_active is True  # 경고 자체는 계속 유효


def test_최소_간격이_지나면_다시_울린다():
    state = ba.BatteryAlertState()
    state, _ = ba.check_battery(ba.WARN_MV - 1, now=0.0, state=state)
    state, cmd = ba.check_battery(
        ba.WARN_MV - 1, now=ba.MIN_REPEAT_INTERVAL_S + 0.1, state=state
    )
    assert cmd is not None


def test_회복_문턱_위로_올라가야_경고가_꺼진다():
    state = ba.BatteryAlertState()
    state, _ = ba.check_battery(ba.WARN_MV - 1, now=0.0, state=state)
    # WARN_MV와 RECOVER_MV 사이 — 아직 회복 아님, 경고 유지.
    mid = (ba.WARN_MV + ba.RECOVER_MV) / 2
    state, cmd = ba.check_battery(mid, now=1.0, state=state)
    assert state.warning_active is True
    assert cmd is None  # 방금 울렸으니 간격 안 지남
    state, _ = ba.check_battery(ba.RECOVER_MV, now=2.0, state=state)
    assert state.warning_active is False


def test_히스테리시스로_문턱_근처_떨림에_안_흔들린다():
    """WARN_MV 바로 아래로 한 번 떨어졌다가 WARN_MV~RECOVER_MV 사이로
    돌아와도 경고가 계속 유지돼야 한다 — 매초 켜졌다 꺼졌다 하면 안 된다."""
    state = ba.BatteryAlertState()
    state, _ = ba.check_battery(ba.WARN_MV - 1, now=0.0, state=state)
    for i in range(1, 5):
        state, _ = ba.check_battery(ba.WARN_MV + 1, now=float(i), state=state)
        assert state.warning_active is True


def test_부저_주파수는_압전_공진대_밖이다():
    """사용자 지시(2026-08-28) — 2~4kHz 공진대를 피한다."""
    assert not (2000 <= ba.BUZZER_FREQ_HZ <= 4000)


def test_on_time은_짧다():
    assert ba.BUZZER_ON_S <= 0.1
