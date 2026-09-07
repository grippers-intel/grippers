"""Min/Max_Angle_Limit 레지스터 읽기·쓰기 (2026-09-01 그리퍼 먹통 사고).

## 배경

`tests/test_calib_identity.py` 의 배경 설명과 짝이다: Homing_Offset 을
복구하는 것만으로는 부족했다. 2026-09-01 실기에서 그리퍼(servo 6)가 어떤
폭을 명령해도 전혀 안 닫히는 사고가 났는데, 그 시점 Homing_Offset 은 이미
`restore_taught_offsets.py` 로 복구돼 있었다. 원인은 별개의 EEPROM
레지스터인 Min/Max_Angle_Limit(서보의 물리적 가동 범위, 주소 9/11)이었다 —
LeRobot/VLA 캘리브레이션이 이것도 Homing_Offset 과 같이 덮어쓰는데, 그
시점의 복구 도구는 이 레지스터를 아예 보지도 쓰지도 않았다.

그날은 스크래치패드 즉석 스크립트로 응급 복구했고 저장소에는 반영되지
않았다(`grippers_handover_20260901.md` §2-4). 이 테스트는 그 즉석 스크립트를
옮겨 온 `position_limit_registers.py` 가 실기 없이 검증되게 한다.
"""

from __future__ import annotations

import importlib.util
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
GRIPPERS_ARM_SRC = ROOT / "ros2_ws" / "src" / "grippers_arm"
BACKUP = ROOT / "tools" / "arm" / "servo_backup" / "servo_COM8_20260829_181124.json"

if str(GRIPPERS_ARM_SRC) not in sys.path:
    sys.path.insert(0, str(GRIPPERS_ARM_SRC))


def _load(name):
    path = GRIPPERS_ARM_SRC / "grippers_arm" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


poslim = _load("position_limit_registers")
ci = _load("calib_identity")

TAUGHT = {
    1: (932, 3425), 2: (817, 3196), 3: (889, 3101),
    4: (870, 3224), 5: (129, 3995), 6: (1140, 2090),
}


class FakeDriver:
    """레지스터 두 개(min/max)와 EEPROM 잠금 상태만 흉내낸다."""

    def __init__(self, limits: dict[int, tuple[int, int]] | None = None,
                 fail_read: set[int] | None = None,
                 fail_write: bool = False):
        self.limits = dict(limits or {})
        self.fail_read = fail_read or set()
        self.fail_write = fail_write
        self.locked: dict[int, bool] = {}
        self.write_calls: list[tuple[int, int, int]] = []

    def _read_u16(self, servo_id, addr):
        if servo_id in self.fail_read:
            return None
        lo, hi = self.limits.get(servo_id, (None, None))
        if addr == poslim.ADDR_MIN_ANGLE_LIMIT:
            return lo
        if addr == poslim.ADDR_MAX_ANGLE_LIMIT:
            return hi
        raise AssertionError(f"모르는 주소 {addr}")

    def _write_u16(self, servo_id, addr, value):
        self.write_calls.append((servo_id, addr, value))
        if self.fail_write:
            return False
        lo, hi = self.limits.get(servo_id, (0, 0))
        if addr == poslim.ADDR_MIN_ANGLE_LIMIT:
            self.limits[servo_id] = (value, hi)
        elif addr == poslim.ADDR_MAX_ANGLE_LIMIT:
            self.limits[servo_id] = (lo, value)
        else:
            raise AssertionError(f"모르는 주소 {addr}")
        return True

    def set_eeprom_lock(self, servo_id, locked):
        self.locked[servo_id] = locked
        return True


# ── 읽기 ─────────────────────────────────────────────────────────────────


def test_min_max를_한_벌로_읽는다():
    drv = FakeDriver(limits={6: (1140, 2090)})

    assert poslim.get_position_limits(drv, 6) == (1140, 2090)


def test_하나라도_못_읽으면_None이다():
    """min은 읽혔는데 max를 못 읽은 경우까지 '읽었다'로 치면 안 된다."""
    drv = FakeDriver(limits={6: (1140, 2090)}, fail_read={6})

    assert poslim.get_position_limits(drv, 6) is None


def test_read_all이_실패한_서보만_재시도한다():
    class Flaky:
        def __init__(self):
            self.calls = 0

        def _read_u16(self, servo_id, addr):
            self.calls += 1
            if self.calls <= 2:      # 첫 서보의 min/max 읽기 한 번씩 실패
                return None
            return TAUGHT[servo_id][0 if addr == poslim.ADDR_MIN_ANGLE_LIMIT else 1]

    out = poslim.read_all(Flaky(), [1], attempts=3)

    assert out == {1: TAUGHT[1]}


def test_끝까지_못_읽으면_None이다():
    drv = FakeDriver(fail_read={6})

    assert poslim.read_all(drv, [6], attempts=2) == {6: None}


# ── 쓰기 ─────────────────────────────────────────────────────────────────


def test_쓰기_전후로_잠금을_풀고_잠근다():
    """set_homing_offset과 같은 계약 — EEPROM은 잠금을 풀어야 쓸 수 있다."""
    drv = FakeDriver(limits={6: (1960, 2378)})

    assert poslim.set_position_limits(drv, 6, 1140, 2090)

    assert drv.limits[6] == (1140, 2090)
    # 마지막 상태는 다시 잠긴 채여야 한다.
    assert drv.locked[6] is True


def test_쓰기_실패는_False를_돌려준다():
    drv = FakeDriver(limits={6: (1960, 2378)}, fail_write=True)

    assert not poslim.set_position_limits(drv, 6, 1140, 2090)


# ── verdict 재사용 ───────────────────────────────────────────────────────


def test_min_max를_펼쳐서_기존_verdict로_판정할_수_있다():
    """새 판정 로직을 만들지 않는다 — calib_identity.verdict를 그대로 쓴다."""
    current = {6: (1960, 2378)}
    taught = {6: (1140, 2090)}

    current_min, current_max = poslim.split_limits(current)
    taught_min, taught_max = poslim.split_taught(taught)

    assert not ci.verdict(current_min, taught_min).ok
    assert not ci.verdict(current_max, taught_max).ok


def test_교시값과_같으면_통과한다():
    current_min, current_max = poslim.split_limits(dict(TAUGHT))
    taught_min, taught_max = poslim.split_taught(TAUGHT)

    assert ci.verdict(current_min, taught_min).ok
    assert ci.verdict(current_max, taught_max).ok


def test_못_읽은_것도_split_후_None으로_남는다():
    current_min, current_max = poslim.split_limits({6: None})

    assert current_min == {6: None}
    assert current_max == {6: None}


# ── 사본이 원본과 갈라지지 않는지 ──────────────────────────────────────────


def test_교시_각도제한이_백업_파일과_같다():
    """TAUGHT_POSITION_LIMITS는 TAUGHT_HOMING_OFFSETS와 같은 백업 JSON의
    사본이다 — 같은 순간의 같은 팔 상태이므로 갈라지면 안 된다."""
    profiles = _load("floor_grasp_profiles")
    data = json.loads(BACKUP.read_text(encoding="utf-8"))

    ids = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
           "wrist_flex": 4, "wrist_roll": 5, "gripper": 6}
    expected = {ids[n]: (r["Min_Position_Limit"], r["Max_Position_Limit"])
                for n, r in data["motors"].items()}

    # ⚠️ 2026-09-07: servo 6 하한만 백업(1140)에서 **일부러 벗어났다**.
    # 두 번 움직였고, 두 번째가 첫 번째 근거를 뒤집었다.
    #
    #   1140 -> 1090  1140 은 기계적 한계가 아니라 소프트웨어 제한이었다.
    #                 토크를 끄고 손으로 누르니 1106 까지 닫혔다. "도달
    #                 가능한 자리보다 조금 아래" 를 기준으로 1090 을 잡았다.
    #   1090 -> 1007  그 기준이 틀렸다. 무는 힘은 P x (목표 - 현재) 라
    #                 **도달하지 못하는 거리**에서 나오므로, 하한을 기계적
    #                 스토퍼 근처에 두면 밀 여지가 사라진다. 사용자 관찰이
    #                 이것을 드러냈다 — "텔레옵 때는 리더암 같은 파지력이
    #                 나왔는데 지금은 안 나온다."
    #
    # 1007 은 지어낸 값이 아니라 텔레옵이 실제로 쓰던 자리다. lerobot
    # 캘리브레이션의 그리퍼 range_min=1960 을 지금 프레임으로 옮긴 값이고
    # (390 - 1343 = -953), 텔레옵이 그 하한으로 몇 달을 돌았다.
    #
    # 백업 JSON 은 **교시 순간의 기록**이므로 고치지 않는다. 대신 이탈을
    # 여기에 적어 둔다 — 그래야 나머지 다섯 관절의 드리프트는 계속 잡히고,
    # 이 하나는 "왜 다른지"가 코드에 남는다. 새 이탈이 생기면 반드시 여기에
    # 근거와 함께 적을 것.
    DELIBERATE = {6: ((1140, 2090), (1007, 2090))}   # {servo: (백업, 지금)}

    for sid, (backup, now) in DELIBERATE.items():
        assert expected[sid] == backup, (
            f"servo {sid} 백업이 바뀌었다 — 이탈 기록을 다시 볼 것")
        assert profiles.TAUGHT_POSITION_LIMITS[sid] == now
        expected[sid] = now

    assert profiles.TAUGHT_POSITION_LIMITS == expected
