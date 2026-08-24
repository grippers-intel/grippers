"""Hardware-free unit tests for tools/idle_align_daemon.py.

Mirrors tests/test_align_to_idle.py's approach: load the script by file path,
monkeypatch align_to_idle._connect with a fake driver, and monkeypatch
subprocess.run so no real pgrep/serial I/O ever happens."""

import importlib.util
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
GRIPPERS_ARM_SRC = ROOT / "ros2_ws" / "src" / "grippers_arm"
TOOLS = ROOT / "tools"

if str(GRIPPERS_ARM_SRC) not in sys.path:
    sys.path.insert(0, str(GRIPPERS_ARM_SRC))
if str(TOOLS) not in sys.path:
    # idle_align_daemon.py does `import align_to_idle as ai` expecting tools/
    # to be on sys.path, same as when it's run directly with `python3
    # tools/idle_align_daemon.py` (the script's own directory is auto-added).
    sys.path.insert(0, str(TOOLS))


def _load_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, TOOLS / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


align = _load_module("align_to_idle", "align_to_idle.py")
daemon = _load_module("idle_align_daemon", "idle_align_daemon.py")
daemon.ai = align  # daemon imported its own copy of align_to_idle at exec time; point it at ours


class FakeStatus:
    def __init__(self, online=True, position=None, temperature=25):
        self.online = online
        self.position = position
        self.temperature = temperature


class FakeDriver:
    """Duck-types just the STS3215Driver surface align_to_idle.py calls, plus
    ping() for the daemon's own online probe."""

    def __init__(self, positions, online=None):
        self.positions = dict(positions)
        self.online = online or dict.fromkeys(positions, True)
        self.temperatures = dict.fromkeys(positions, 25)
        self.calls = []
        self.disconnected = False

    def ping(self, servo_id):
        return self.online.get(servo_id, True)

    def get_position(self, servo_id):
        return self.positions.get(servo_id)

    def set_position(self, servo_id, position):
        self.calls.append(("set_position", servo_id, position))
        self.positions[servo_id] = position
        return True

    def set_speed(self, servo_id, speed):
        self.calls.append(("set_speed", servo_id, speed))
        return True

    def set_acceleration(self, servo_id, acc):
        self.calls.append(("set_acceleration", servo_id, acc))
        return True

    def get_all_status(self):
        return {
            servo_id: FakeStatus(
                online=self.online.get(servo_id, True),
                position=self.positions.get(servo_id),
                temperature=self.temperatures.get(servo_id, 25),
            )
            for servo_id in self.positions
        }

    def disconnect(self):
        self.disconnected = True


def _at_target():
    targets = align.idle_targets()
    return targets, dict(targets)


def _no_owner(monkeypatch):
    monkeypatch.setattr(daemon.subprocess, "run", lambda *a, **k: type("R", (), {"returncode": 1})())


def _owner_present(monkeypatch):
    monkeypatch.setattr(daemon.subprocess, "run", lambda *a, **k: type("R", (), {"returncode": 0})())


def test_owned_elsewhere_skips_probe_entirely(monkeypatch, tmp_path):
    _owner_present(monkeypatch)
    probed = []
    monkeypatch.setattr(daemon, "_probe_online", lambda port: probed.append(port) or True)

    was_online = daemon._tick("/dev/fake", str(tmp_path / "log.jsonl"), was_online=False, settle=0)

    assert was_online is False
    assert probed == []  # never even tried to open the port


def test_fresh_connect_triggers_one_alignment(monkeypatch, tmp_path):
    _no_owner(monkeypatch)
    targets, positions = _at_target()
    positions[2] += 50
    driver = FakeDriver(positions)
    monkeypatch.setattr(align, "_connect", lambda port: driver)
    monkeypatch.setattr(daemon, "_probe_online", lambda port: True)
    log_path = str(tmp_path / "log.jsonl")

    was_online = daemon._tick("/dev/fake", log_path, was_online=False, settle=0)

    assert was_online is True
    assert any(name == "set_position" for name, *_ in driver.calls)
    assert driver.disconnected
    events = [line for line in pathlib.Path(log_path).read_text().splitlines()]
    assert any('"align_done"' in line or '"align_incomplete"' in line for line in events)


def test_staying_online_does_not_realign(monkeypatch, tmp_path):
    _no_owner(monkeypatch)
    aligned = []
    monkeypatch.setattr(daemon, "_align_once", lambda port, log_path: aligned.append(1))
    monkeypatch.setattr(daemon, "_probe_online", lambda port: True)
    log_path = str(tmp_path / "log.jsonl")

    was_online = daemon._tick("/dev/fake", log_path, was_online=True, settle=0)

    assert was_online is True
    assert aligned == []  # already online last tick — no new alignment triggered


def test_large_offset_is_rejected_without_moving(monkeypatch, tmp_path):
    _no_owner(monkeypatch)
    targets, positions = _at_target()
    positions[4] += 900
    driver = FakeDriver(positions)
    monkeypatch.setattr(align, "_connect", lambda port: driver)
    monkeypatch.setattr(daemon, "_probe_online", lambda port: True)
    log_path = str(tmp_path / "log.jsonl")

    daemon._tick("/dev/fake", log_path, was_online=False, settle=0)

    assert driver.calls == []
    assert '"align_rejected"' in pathlib.Path(log_path).read_text()


def test_owner_appearing_during_settle_cancels_alignment(monkeypatch, tmp_path):
    calls = {"n": 0}

    def flaky_owner_check():
        calls["n"] += 1
        return calls["n"] > 1  # unowned on first check, owned by the time settle ends

    monkeypatch.setattr(daemon, "_port_owned_elsewhere", flaky_owner_check)
    monkeypatch.setattr(daemon, "_probe_online", lambda port: True)
    aligned = []
    monkeypatch.setattr(daemon, "_align_once", lambda port, log_path: aligned.append(1))
    log_path = str(tmp_path / "log.jsonl")

    daemon._tick("/dev/fake", log_path, was_online=False, settle=0)

    assert aligned == []
    assert '"align_skip_owner_appeared"' in pathlib.Path(log_path).read_text()
