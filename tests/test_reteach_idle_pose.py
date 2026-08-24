"""Hardware-free unit tests for tools/reteach_idle_pose.py — mirrors
tests/test_align_to_idle.py's approach (fake driver, no pyserial needed)."""

import importlib.util
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "tools" / "reteach_idle_pose.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("reteach_idle_pose", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


reteach = _load_module()

START = {1: 2054, 2: 823, 3: 3099, 4: 2272, 5: 3088}


class FakeDriver:
    """position_reads[sid] is a per-servo queue popped on each get_position()
    call — lets a test simulate the arm being physically re-posed by hand
    between the release Enter and the confirm Enter."""

    def __init__(self, position_reads, online=None, fail_torque_on=None, fail_set_position_on=None):
        self.position_reads = {sid: list(vals) for sid, vals in position_reads.items()}
        self.online = online or dict.fromkeys(position_reads, True)
        self.fail_torque_on = fail_torque_on or set()
        self.fail_set_position_on = fail_set_position_on or set()
        self.torque_enabled = dict.fromkeys(position_reads, True)
        self.calls = []
        self.disconnected = False

    def ping(self, servo_id):
        return self.online.get(servo_id, True)

    def get_position(self, servo_id):
        return self.position_reads[servo_id].pop(0)

    def set_torque(self, servo_id, enable):
        self.calls.append(("set_torque", servo_id, enable))
        if servo_id in self.fail_torque_on:
            return False
        self.torque_enabled[servo_id] = enable
        return True

    def set_position(self, servo_id, position):
        self.calls.append(("set_position", servo_id, position))
        if servo_id in self.fail_set_position_on:
            return False
        self.torque_enabled[servo_id] = True  # STS3215: goal write auto-enables torque
        return True

    def disconnect(self):
        self.disconnected = True


def _inputs(monkeypatch, lines):
    it = iter(lines)
    monkeypatch.setattr("builtins.input", lambda prompt="": next(it))


def test_first_enter_releases_all_five_joints(monkeypatch):
    reads = {sid: [pos, pos] for sid, pos in START.items()}
    driver = FakeDriver(reads)
    monkeypatch.setattr(reteach, "_connect", lambda port: driver)
    _inputs(monkeypatch, ["", ""])

    reteach.run("/dev/fake")

    for sid in reteach.SERVO_IDS:
        assert ("set_torque", sid, False) in driver.calls
    assert 6 not in driver.torque_enabled  # gripper never touched


def test_manual_repose_of_all_joints_is_latched_on_confirm(monkeypatch):
    moved = {1: 2060, 2: 900, 3: 3050, 4: 2300, 5: 3100}
    reads = {sid: [START[sid], moved[sid]] for sid in START}
    driver = FakeDriver(reads)
    monkeypatch.setattr(reteach, "_connect", lambda port: driver)
    _inputs(monkeypatch, ["", ""])

    code = reteach.run("/dev/fake")

    assert code == 0
    for sid, pos in moved.items():
        assert ("set_position", sid, pos) in driver.calls
        assert driver.torque_enabled[sid] is True
    assert driver.disconnected


def test_q_before_release_touches_nothing(monkeypatch):
    reads = {sid: [pos] for sid, pos in START.items()}
    driver = FakeDriver(reads)
    monkeypatch.setattr(reteach, "_connect", lambda port: driver)
    _inputs(monkeypatch, ["q"])

    code = reteach.run("/dev/fake")

    assert code == 2
    assert driver.calls == []


def test_q_after_release_drives_all_joints_back_to_start(monkeypatch):
    reads = {sid: [pos] for sid, pos in START.items()}
    driver = FakeDriver(reads)
    monkeypatch.setattr(reteach, "_connect", lambda port: driver)
    _inputs(monkeypatch, ["", "q"])

    code = reteach.run("/dev/fake")

    assert code == 2
    for sid, pos in START.items():
        assert ("set_position", sid, pos) in driver.calls


def test_offline_servo_aborts_before_any_write(monkeypatch):
    reads = {sid: [pos] for sid, pos in START.items()}
    online = {sid: True for sid in START}
    online[3] = False
    driver = FakeDriver(reads, online=online)
    monkeypatch.setattr(reteach, "_connect", lambda port: driver)

    code = reteach.run("/dev/fake")

    assert code == 1
    assert driver.calls == []


def test_partial_torque_release_failure_relocks_already_released_joints(monkeypatch):
    reads = {sid: [pos] for sid, pos in START.items()}
    driver = FakeDriver(reads, fail_torque_on={3})
    monkeypatch.setattr(reteach, "_connect", lambda port: driver)
    _inputs(monkeypatch, [""])

    code = reteach.run("/dev/fake")

    assert code == 1
    # servo 1 and 2 were released before servo 3 failed — both must be
    # driven back to their start position, not left dangling and unlocked.
    assert ("set_position", 1, START[1]) in driver.calls
    assert ("set_position", 2, START[2]) in driver.calls
