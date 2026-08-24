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
    """position_reads lets a test simulate the joint being physically turned
    by hand between the release Enter and the confirm Enter — each
    get_position() call pops the next scripted reading."""

    def __init__(self, position_reads, online=True):
        self.position_reads = list(position_reads)
        self.online = online
        self.torque_enabled = True
        self.calls = []
        self.disconnected = False

    def ping(self, servo_id):
        return self.online

    def get_position(self, servo_id):
        return self.position_reads.pop(0)

    def set_torque(self, servo_id, enable):
        self.calls.append(("set_torque", servo_id, enable))
        self.torque_enabled = enable
        return True

    def set_position(self, servo_id, position):
        self.calls.append(("set_position", servo_id, position))
        self.torque_enabled = True  # STS3215: goal write auto-enables torque
        return True

    def disconnect(self):
        self.disconnected = True


def _inputs(monkeypatch, lines):
    it = iter(lines)
    monkeypatch.setattr("builtins.input", lambda prompt="": next(it))


def test_first_enter_releases_torque(monkeypatch):
    driver = FakeDriver(position_reads=[2045, 2045])
    monkeypatch.setattr(jog, "_connect", lambda port: driver)
    _inputs(monkeypatch, ["", ""])  # release, then confirm at the same spot

    jog.run("/dev/fake")

    assert ("set_torque", jog.SERVO_ID, False) in driver.calls


def test_manual_move_between_enters_is_latched_on_confirm(monkeypatch):
    # first get_position() call is the initial read before release; second is
    # read again after the user has (in this simulation) turned it by hand.
    driver = FakeDriver(position_reads=[2045, 2210])
    monkeypatch.setattr(jog, "_connect", lambda port: driver)
    _inputs(monkeypatch, ["", ""])

    code = jog.run("/dev/fake")

    assert code == 0
    assert ("set_position", jog.SERVO_ID, 2210) in driver.calls
    assert driver.torque_enabled is True
    assert driver.disconnected


def test_q_before_release_does_nothing(monkeypatch):
    driver = FakeDriver(position_reads=[2045])
    monkeypatch.setattr(jog, "_connect", lambda port: driver)
    _inputs(monkeypatch, ["q"])

    code = jog.run("/dev/fake")

    assert code == 2
    assert driver.calls == []  # torque never touched


def test_q_after_release_reverts_to_start_position(monkeypatch):
    driver = FakeDriver(position_reads=[2045])
    monkeypatch.setattr(jog, "_connect", lambda port: driver)
    _inputs(monkeypatch, ["", "q"])  # release, then cancel

    code = jog.run("/dev/fake")

    assert code == 2
    assert ("set_torque", jog.SERVO_ID, False) in driver.calls
    assert ("set_position", jog.SERVO_ID, 2045) in driver.calls  # driven back to start


def test_offline_servo_aborts_before_any_write(monkeypatch):
    driver = FakeDriver(position_reads=[2045], online=False)
    monkeypatch.setattr(jog, "_connect", lambda port: driver)

    code = jog.run("/dev/fake")

    assert code == 1
    assert driver.calls == []
