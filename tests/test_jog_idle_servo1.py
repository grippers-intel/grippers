"""Hardware-free unit tests for tools/jog_idle_servo1.py — mirrors
tests/test_align_to_idle.py's approach (fake driver, no pyserial needed)."""

import importlib.util
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "tools" / "jog_idle_servo1.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("jog_idle_servo1", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


jog = _load_module()


class FakeDriver:
    def __init__(self, position, online=True):
        self.position = position
        self.online = online
        self.calls = []
        self.disconnected = False

    def ping(self, servo_id):
        return self.online

    def get_position(self, servo_id):
        return self.position

    def set_position(self, servo_id, position):
        self.calls.append(("set_position", servo_id, position))
        self.position = position
        return True

    def set_speed(self, servo_id, speed):
        self.calls.append(("set_speed", servo_id, speed))
        return True

    def set_acceleration(self, servo_id, acc):
        self.calls.append(("set_acceleration", servo_id, acc))
        return True

    def disconnect(self):
        self.disconnected = True


def _inputs(monkeypatch, lines):
    it = iter(lines)
    monkeypatch.setattr("builtins.input", lambda prompt="": next(it))


# ── pure functions ──────────────────────────────────────────────────────────


def test_raw_after_delta_one_degree_matches_protocol_ratio():
    # 4095 raw counts == 360 degrees for STS3215 — one degree is 4095/360 raw.
    assert jog.raw_after_delta(2045, 1.0) == 2045 + round(4095 / 360.0)


def test_raw_after_delta_negative_moves_the_other_way():
    assert jog.raw_after_delta(2045, -10.0) < 2045


def test_clamp_check_within_bounds_is_none():
    assert jog.clamp_check(15.0, max_offset_deg=30.0) is None


def test_clamp_check_over_bounds_is_rejected():
    problem = jog.clamp_check(45.0, max_offset_deg=30.0)
    assert problem is not None
    assert "30" in problem


def test_parse_command_blank_is_confirm():
    assert jog.parse_command("") == ("confirm", None)
    assert jog.parse_command("   ") == ("confirm", None)


def test_parse_command_q_is_quit():
    assert jog.parse_command("q") == ("quit", None)
    assert jog.parse_command("QUIT") == ("quit", None)


def test_parse_command_number_is_move():
    assert jog.parse_command("-3.5") == ("move", -3.5)


def test_parse_command_garbage_is_invalid():
    kind, value = jog.parse_command("abc")
    assert kind == "invalid"
    assert value == "abc"


# ── run() end-to-end against a fake driver ──────────────────────────────────


def test_confirm_immediately_does_not_move_servo(monkeypatch):
    driver = FakeDriver(position=2045)
    monkeypatch.setattr(jog, "_connect", lambda port: driver)
    _inputs(monkeypatch, [""])  # confirm right away

    code = jog.run("/dev/fake", max_offset_deg=30.0)

    assert code == 0
    # only the initial torque-latch write (goal <- present), no actual motion
    moves = [c for c in driver.calls if c[0] == "set_position"]
    assert len(moves) == 1
    assert moves[0] == ("set_position", jog.SERVO_ID, 2045)
    assert driver.disconnected


def test_move_then_confirm_applies_relative_offset(monkeypatch):
    driver = FakeDriver(position=2045)
    monkeypatch.setattr(jog, "_connect", lambda port: driver)
    _inputs(monkeypatch, ["10", ""])  # move +10deg, then confirm

    code = jog.run("/dev/fake", max_offset_deg=30.0)

    assert code == 0
    expected_raw = 2045 + round(10.0 * jog.RAW_PER_DEG)
    assert driver.position == expected_raw


def test_quit_reverts_to_start_position(monkeypatch):
    driver = FakeDriver(position=2045)
    monkeypatch.setattr(jog, "_connect", lambda port: driver)
    _inputs(monkeypatch, ["10", "-5", "q"])

    code = jog.run("/dev/fake", max_offset_deg=30.0)

    assert code == 2
    assert driver.position == 2045  # reverted, not left at the +5 net offset


def test_over_limit_move_is_rejected_and_position_unchanged(monkeypatch):
    driver = FakeDriver(position=2045)
    monkeypatch.setattr(jog, "_connect", lambda port: driver)
    _inputs(monkeypatch, ["45", ""])  # exceeds default-style 30° cap, then confirm

    code = jog.run("/dev/fake", max_offset_deg=30.0)

    assert code == 0
    assert driver.position == 2045  # rejected move never applied


def test_offline_servo_aborts_before_any_write(monkeypatch):
    driver = FakeDriver(position=2045, online=False)
    monkeypatch.setattr(jog, "_connect", lambda port: driver)

    code = jog.run("/dev/fake", max_offset_deg=30.0)

    assert code == 1
    assert driver.calls == []
