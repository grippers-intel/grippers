"""Hardware-free unit tests for tools/align_to_idle.py — a fake driver stands
in for STS3215Driver so these run without pyserial or a connected robot."""

import importlib.util
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
GRIPPERS_ARM_SRC = ROOT / "ros2_ws" / "src" / "grippers_arm"
SCRIPT = ROOT / "tools" / "align_to_idle.py"

# floor_grasp_profiles/gripper_calibration have no ROS dependency, so the
# package is importable on a plain dev machine once its src dir is on
# sys.path — align_to_idle.py imports it the same way the other tools/*
# hardware scripts do.
if str(GRIPPERS_ARM_SRC) not in sys.path:
    sys.path.insert(0, str(GRIPPERS_ARM_SRC))


def _load_module():
    spec = importlib.util.spec_from_file_location("align_to_idle", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


align = _load_module()


class FakeStatus:
    def __init__(self, online=True, position=None, temperature=25):
        self.online = online
        self.position = position
        self.temperature = temperature


class FakeDriver:
    """Duck-types just the STS3215Driver surface align_to_idle.py calls."""

    def __init__(
        self, positions, temperatures=None, online=None, jam_servo=None, jam_after_calls=0
    ):
        self.positions = dict(positions)
        self.temperatures = temperatures or dict.fromkeys(positions, 25)
        self.online = online or dict.fromkeys(positions, True)
        self.jam_servo = jam_servo
        self.jam_after_calls = jam_after_calls
        self._set_position_calls = dict.fromkeys(positions, 0)
        self.calls = []

    def get_position(self, servo_id):
        return self.positions.get(servo_id)

    def set_position(self, servo_id, position):
        self.calls.append(("set_position", servo_id, position))
        self._set_position_calls[servo_id] = self._set_position_calls.get(servo_id, 0) + 1
        if servo_id == self.jam_servo and self._set_position_calls[servo_id] > self.jam_after_calls:
            return True  # write accepted by the servo, but it's mechanically blocked
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


def _at_target(offset_by=None):
    """servo positions equal to idle_targets(), optionally offset per servo."""
    targets = align.idle_targets()
    positions = dict(targets)
    for servo_id, delta in (offset_by or {}).items():
        positions[servo_id] += delta
    return targets, positions


def test_offset_over_800_is_rejected_by_the_pure_check():
    targets, positions = _at_target({4: 900})
    driver = FakeDriver(positions)

    problems = align.check_safe_to_align(driver.get_all_status(), targets)

    assert any("servo 4" in problem for problem in problems)
    assert driver.calls == []


def test_hot_servo2_is_rejected_by_the_pure_check():
    targets, positions = _at_target()
    driver = FakeDriver(positions, temperatures={sid: 25 for sid in positions} | {2: 41})

    problems = align.check_safe_to_align(driver.get_all_status(), targets)

    assert any("servo 2" in problem and "41" in problem for problem in problems)


def test_offline_servo_is_rejected_by_the_pure_check():
    targets, positions = _at_target()
    driver = FakeDriver(positions, online={sid: True for sid in positions} | {3: False})

    problems = align.check_safe_to_align(driver.get_all_status(), targets)

    assert any("3" in problem for problem in problems)


def test_large_offset_is_rejected_before_any_write(monkeypatch):
    targets, positions = _at_target({4: 900})
    driver = FakeDriver(positions)
    monkeypatch.setattr(align, "_connect", lambda port: driver)

    code = align.main(["--port", "/dev/fake"])

    assert code == 1
    assert driver.calls == []


def test_hot_servo2_is_rejected_before_any_write(monkeypatch):
    targets, positions = _at_target()
    driver = FakeDriver(positions, temperatures={sid: 25 for sid in positions} | {2: 41})
    monkeypatch.setattr(align, "_connect", lambda port: driver)

    code = align.main(["--port", "/dev/fake"])

    assert code == 1
    assert driver.calls == []


def test_dry_run_never_writes(monkeypatch):
    targets, positions = _at_target({4: 50})
    driver = FakeDriver(positions)
    monkeypatch.setattr(align, "_connect", lambda port: driver)

    code = align.main(["--port", "/dev/fake", "--dry-run"])

    assert code == 0
    assert driver.calls == []


def test_goal_latches_to_present_before_interpolated_motion(monkeypatch):
    targets, positions = _at_target({2: 300})
    original_positions = dict(positions)
    driver = FakeDriver(positions)
    monkeypatch.setattr(align, "_connect", lambda port: driver)

    code = align.main(["--port", "/dev/fake", "--settle", "0"])

    assert code == 0
    set_position_calls = [(sid, pos) for name, sid, pos in driver.calls if name == "set_position"]
    latch_calls = set_position_calls[: len(targets)]
    # first write per servo is goal <- present — zero-motion torque latch,
    # not a step toward the target.
    assert dict(latch_calls) == original_positions
    later_calls = set_position_calls[len(targets) :]
    assert later_calls, "interpolated glide never ran after the latch"
    assert any(pos != original_positions[sid] for sid, pos in later_calls)


def test_jam_stops_after_two_stalled_steps(monkeypatch):
    targets, positions = _at_target({1: 180, 4: -400})
    driver = FakeDriver(positions, jam_servo=4, jam_after_calls=1)
    monkeypatch.setattr(align, "_connect", lambda port: driver)

    code = align.main(["--port", "/dev/fake", "--steps", "6", "--settle", "0"])

    assert code == 2
    # servo 4 never got anywhere near its target — the glide aborted early
    # instead of running all 6 steps.
    assert driver.positions[4] != targets[4]
    assert abs(driver.positions[4] - targets[4]) > 300
    # a non-jammed servo also stopped short of the full glide, proving the
    # whole motion — not just the stuck joint — was cut off.
    assert driver.positions[1] != targets[1]
