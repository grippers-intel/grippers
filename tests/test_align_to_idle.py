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


from grippers_arm.floor_grasp_profiles import TAUGHT_HOMING_OFFSETS


class FakeStatus:
    def __init__(self, online=True, position=None, temperature=25):
        self.online = online
        self.position = position
        self.temperature = temperature


class FakeDriver:
    """Duck-types just the STS3215Driver surface align_to_idle.py calls."""

    def __init__(
        self, positions, temperatures=None, online=None, jam_servo=None, jam_after_calls=0,
        homing_offsets=None,
    ):
        self.positions = dict(positions)
        # 프레임 검사가 읽는 값. 기본은 교시 오프셋 - 이 테스트들은 교시 IDLE 경로를
        # 보므로, 팔이 교시 프레임에 있다는 것이 이들의 전제다.
        self.homing_offsets = (
            dict(homing_offsets) if homing_offsets is not None
            else dict(TAUGHT_HOMING_OFFSETS)
        )
        self.temperatures = temperatures or dict.fromkeys(positions, 25)
        self.online = online or dict.fromkeys(positions, True)
        self.jam_servo = jam_servo
        self.jam_after_calls = jam_after_calls
        self._set_position_calls = dict.fromkeys(positions, 0)
        self.calls = []

    def get_position(self, servo_id):
        return self.positions.get(servo_id)

    def get_homing_offset(self, servo_id):
        return self.homing_offsets.get(servo_id)

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


def test_large_offset_warns_but_does_not_block(monkeypatch):
    """2026-08-24 사용자 지시로 편차 상한 거부를 껐다 — 큰 편차는 경고만
    하고 정렬은 그대로 진행한다(align_to_idle.py LARGE_OFFSET_WARN_RAW 주석)."""
    targets, positions = _at_target({4: 900})
    driver = FakeDriver(positions)

    assert align.check_safe_to_align(driver.get_all_status(), targets) == []
    warnings = align.large_offsets(driver.get_all_status(), targets)
    assert any("servo 4" in w for w in warnings)


def test_hot_servo2_is_rejected_by_the_pure_check():
    targets, positions = _at_target()
    driver = FakeDriver(positions, temperatures={sid: 25 for sid in positions} | {2: 51})

    problems = align.check_safe_to_align(driver.get_all_status(), targets)

    assert any("servo 2" in problem and "51" in problem for problem in problems)


def test_offline_servo_is_rejected_by_the_pure_check():
    targets, positions = _at_target()
    driver = FakeDriver(positions, online={sid: True for sid in positions} | {3: False})

    problems = align.check_safe_to_align(driver.get_all_status(), targets)

    assert any("3" in problem for problem in problems)


def test_large_offset_still_aligns_end_to_end(monkeypatch):
    """거부 가드를 끈 뒤에도 통신·과열 가드와 끼임 감지는 그대로다 — 여기서
    고정하는 건 "큰 편차여도 실제로 목표까지 간다"이다."""
    targets, positions = _at_target({4: 900})
    driver = FakeDriver(positions)
    monkeypatch.setattr(align, "_connect", lambda port: driver)

    code = align.main(["--port", "/dev/fake", "--settle", "0"])

    assert code == 0
    assert driver.positions[4] == targets[4]


def test_hot_servo2_is_rejected_before_any_write(monkeypatch):
    targets, positions = _at_target()
    driver = FakeDriver(positions, temperatures={sid: 25 for sid in positions} | {2: 51})
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


def test_small_offset_within_tolerance_does_not_false_jam(monkeypatch):
    # Real-hardware regression (2026-08-21): a 6-raw offset over the default
    # 12-step glide rounds to identical consecutive waypoints, so a servo
    # that never visibly moves used to trip JamDetected even though 6 is
    # nowhere near the 120-raw acceptance tolerance. Simulate the worst
    # case — the servo never moves at all, from the very first write.
    targets, positions = _at_target({2: 6})
    driver = FakeDriver(positions, jam_servo=2, jam_after_calls=0)
    monkeypatch.setattr(align, "_connect", lambda port: driver)

    code = align.main(["--port", "/dev/fake", "--settle", "0"])

    assert code == 0


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


class LaggingDriver(FakeDriver):
    """A servo that closes only part of the commanded distance per write.

    This is what the real STS3215s did on 2026-08-24: with servo 2 offset by
    +1668 raw, the 12-step glide wrote waypoints faster than the joints could
    follow, so the last waypoint landed while the arm was still 593 raw short.
    The glide then read that lagging position as its "final" answer and main()
    reported failure (exit 3) even though nothing was wrong mechanically — the
    arm just needed a moment more. ``catch_up`` is the fraction of the
    remaining distance the joint covers per write.
    """

    def __init__(self, positions, catch_up=0.45, **kwargs):
        super().__init__(positions, **kwargs)
        self.catch_up = catch_up

    def set_position(self, servo_id, position):
        self.calls.append(("set_position", servo_id, position))
        current = self.positions[servo_id]
        self.positions[servo_id] = round(current + self.catch_up * (position - current))
        return True


def test_lagging_servos_are_waited_out_instead_of_reported_as_failure(monkeypatch):
    """2026-08-24 실기 회귀 — 보간 끝 = 도달 아님(converge_at_targets 참고).

    servo 2가 +1668 raw 떨어진 상태에서 12스텝 보간을 돌리면 마지막 스텝
    시점에는 아직 수백 raw가 남는다. 예전 코드는 그 값을 최종값으로 읽어
    "허용치 초과"로 3을 반환했다. 이제는 goal이 이미 목표에 박혀 있으므로
    도달할 때까지 기다렸다가 판정한다."""
    targets, positions = _at_target({2: 1668, 4: -1602})
    driver = LaggingDriver(positions)
    monkeypatch.setattr(align, "_connect", lambda port: driver)

    code = align.main(["--port", "/dev/fake", "--settle", "0"])

    assert code == 0
    for servo_id, target in targets.items():
        assert abs(driver.positions[servo_id] - target) <= align.DEFAULT_TOLERANCE_RAW


def test_converge_gives_up_after_the_timeout_without_raising(monkeypatch):
    """도달을 못 해도 예외로 터지지 않고 마지막 present를 돌려준다 — 최종
    판정과 종료 코드는 main()의 리포트가 담당한다."""
    targets, positions = _at_target({3: 900})
    # catch_up=0 → the joint accepts every write but never actually moves.
    driver = LaggingDriver(positions, catch_up=0.0)

    final = align.converge_at_targets(driver, targets, timeout=0.0, poll=0.0)

    assert final[3] == positions[3]


# ── 프레임 검사 ────────────────────────────────────────────────────────────
#
# `latch_torque_at_present` 는 "지금 자세에서 출발한다"만 보장한다. 목표 raw 가
# 다른 캘리브레이션 프레임의 숫자면 출발이 안전해도 도착이 엉뚱하다 - 교시
# 상태에서 --vla 를 돌리면 wrist_roll 이 994틱(87도) 돈다. 온도·통신 검사는
# 전부 통과하므로 이 검사가 없으면 아무것도 막지 못한다.


def _vla_frame_driver():
    """VLA 오프셋이 실린 팔. 교시 IDLE 을 요구하면 거부돼야 한다."""
    return FakeDriver(
        dict.fromkeys(range(1, 7), 2048),
        homing_offsets=align.vla_homing_offsets(),
    )


def test_taught_align_is_refused_when_arm_carries_vla_offsets(monkeypatch):
    driver = _vla_frame_driver()
    monkeypatch.setattr(align, "_connect", lambda port: driver)

    assert align.main([]) == 3
    assert driver.calls == []      # 검사 전에 아무것도 쓰지 않았다


def test_vla_align_is_refused_when_arm_carries_taught_offsets(monkeypatch):
    driver = FakeDriver(dict.fromkeys(range(1, 7), 2048))   # 기본 = 교시 오프셋
    monkeypatch.setattr(align, "_connect", lambda port: driver)

    assert align.main(["--vla"]) == 3
    assert driver.calls == []


def test_vla_align_proceeds_when_frames_agree(monkeypatch):
    driver = _vla_frame_driver()
    monkeypatch.setattr(align, "_connect", lambda port: driver)

    assert align.main(["--vla", "--dry-run"]) == 0


def test_unreadable_offset_is_refused_rather_than_assumed_equal(monkeypatch):
    """못 읽은 것은 '같다'가 아니다 - calib_identity 가 UNREADABLE 로 가르는 이유."""
    driver = FakeDriver(dict.fromkeys(range(1, 7), 2048))
    driver.homing_offsets[3] = None
    monkeypatch.setattr(align, "_connect", lambda port: driver)

    assert align.main([]) == 3
    assert driver.calls == []


def test_skip_frame_check_lets_a_mismatched_frame_through(monkeypatch):
    """오프셋을 일부러 바꾸는 중이라면 빠져나갈 문이 있어야 한다."""
    driver = _vla_frame_driver()
    monkeypatch.setattr(align, "_connect", lambda port: driver)

    assert align.main(["--dry-run", "--skip-frame-check"]) == 0
