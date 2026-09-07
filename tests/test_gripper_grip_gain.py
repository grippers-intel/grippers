"""gripper_grip_gain 이 EEPROM 을 열어 놓은 채 끝나지 않는지 고정한다.

P 게인은 EEPROM 이라 쓰기 전에 Lock(55) 을 풀어야 한다. 푼 채로 두면 그
뒤의 **어떤** 쓰기도 조용히 EEPROM 에 남는다 — 서보 수명이 걸린 자리다.
그래서 "쓰기가 실패해도 다시 잠근다"를 시험으로 못 박는다.
"""

import importlib.util
import pathlib

import pytest

_PATH = pathlib.Path(__file__).resolve().parents[1] / "tools" / "arm" / "gripper_grip_gain.py"
_spec = importlib.util.spec_from_file_location("gripper_grip_gain", _PATH)
ggg = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ggg)


class _FakeLink:
    """레지스터 몇 개짜리 서보. 쓰기 실패를 주소별로 주입한다."""

    def __init__(self, gain=16, fail_on=(), min_limit=1090, max_limit=2090,
                 homing=1343, max_torque=1000, temp_limit=70, temp_now=43):
        self.registers = {
            ggg.ADDR_POSITION_P: gain,
            ggg.ADDR_LOCK: 1,
            ggg.ADDR_MIN_POSITION_LIMIT: min_limit,
            11: max_limit,
            ggg.ADDR_HOMING_OFFSET: homing,
            ggg.ADDR_MAX_TORQUE_LIMIT: max_torque,
            48: max_torque,
            ggg.ADDR_MAX_TEMPERATURE_LIMIT: temp_limit,
            ggg.ADDR_PRESENT_TEMPERATURE: temp_now,
        }
        self.fail_on = set(fail_on)
        self.writes = []

    def read(self, addr, size):
        return self.registers.get(addr)

    def write(self, addr, size, value):
        self.writes.append((addr, value))
        if addr in self.fail_on:
            return False
        self.registers[addr] = value
        return True


def _lock_writes(link):
    return [value for addr, value in link.writes if addr == ggg.ADDR_LOCK]


def test_P를_바꾸면_잠금을_풀었다_다시_잠근다():
    link = _FakeLink(gain=16)

    assert ggg.set_gain(link, 32) == 0

    assert link.registers[ggg.ADDR_POSITION_P] == 32
    assert _lock_writes(link) == [0, 1], "잠금 해제 -> 쓰기 -> 재잠금 이어야 한다"
    assert link.registers[ggg.ADDR_LOCK] == 1


def test_쓰기가_실패해도_다시_잠근다():
    """여기가 이 시험의 존재 이유다 — 실패 경로에서 EEPROM 이 열린 채 남으면
    안 된다."""
    link = _FakeLink(gain=16, fail_on={ggg.ADDR_POSITION_P})

    assert ggg.set_gain(link, 32) == 1

    assert link.registers[ggg.ADDR_LOCK] == 1
    assert _lock_writes(link) == [0, 1]


def test_같은_값이면_잠금을_아예_안_건드린다():
    link = _FakeLink(gain=16)

    assert ggg.set_gain(link, 16) == 0

    assert link.writes == []


@pytest.mark.parametrize("bad", [-1, 255, 1000])
def test_범위_밖_P는_거부한다(bad):
    link = _FakeLink(gain=16)

    assert ggg.set_gain(link, bad) == 1

    assert link.writes == []


def test_목표는_set_gripper_0mm_이_만드는_값과_같다():
    """FULL_CLOSE_RAW 가 gripper_calibration 과 어긋나면 --probe 가 실기와
    다른 조건을 재게 된다."""
    import sys

    sys.path.insert(0, str(_PATH.parents[2] / "ros2_ws" / "src" / "grippers_arm"))
    from grippers_arm.gripper_calibration import position_from_width

    assert ggg.FULL_CLOSE_RAW == position_from_width(0.0, min_width_mm=0.0)


# ── 무는 힘의 진짜 손잡이: Min_Position_Limit ──────────────────────────────


def test_텔레옵_하한은_살아있는_오프셋으로_환산한다():
    """상수로 박으면 오프셋이 바뀐 팔에서 조용히 틀린다. 2026-09-07 실측
    오프셋 1343 에서 lerobot range_min 1960 은 1007 이 된다."""
    assert ggg.teleop_min_limit(_FakeLink(homing=1343))[0] == 1007
    # 오프셋이 lerobot 것(390)이면 환산할 것이 없다.
    assert ggg.teleop_min_limit(_FakeLink(homing=390))[0] == 1960


def test_하한을_텔레옵_자리로_되돌린다():
    link = _FakeLink(min_limit=1090)

    assert ggg.set_min_limit(link, 1007) == 0

    assert link.registers[ggg.ADDR_MIN_POSITION_LIMIT] == 1007
    assert _lock_writes(link) == [0, 1]


def test_텔레옵보다_깊은_하한은_거부한다():
    """그 아래는 검증된 적이 없다 — 빈 턱이 스토퍼를 미는 힘만 커진다."""
    link = _FakeLink(min_limit=1090)

    assert ggg.set_min_limit(link, 900) == 1

    assert link.writes == []
    assert link.registers[ggg.ADDR_MIN_POSITION_LIMIT] == 1090


def test_상한보다_큰_하한은_거부한다():
    link = _FakeLink(min_limit=1090, max_limit=2090)

    assert ggg.set_min_limit(link, 2500) == 1

    assert link.writes == []


def test_하한_쓰기가_실패해도_다시_잠근다():
    link = _FakeLink(min_limit=1090, fail_on={ggg.ADDR_MIN_POSITION_LIMIT})

    assert ggg.set_min_limit(link, 1007) == 1

    assert link.registers[ggg.ADDR_LOCK] == 1
    assert _lock_writes(link) == [0, 1]


# ── 하한과 토크 상한은 짝이다 ──────────────────────────────────────────────


def test_텔레옵_복원은_하한과_토크상한을_같이_바꾼다():
    """2026-09-07 실기: 하한만 1007 로 내리고 상한을 1000 그대로 뒀더니
    정책이 물체를 조이는 순간 servo 6 이 버스에서 떨어졌다. 둘은 짝이다."""
    link = _FakeLink(min_limit=1090, max_torque=1000)

    assert ggg.restore_teleop_grip(link) == 0

    assert link.registers[ggg.ADDR_MIN_POSITION_LIMIT] == 1007
    assert link.registers[ggg.ADDR_MAX_TORQUE_LIMIT] == ggg.TELEOP_MAX_TORQUE


def test_토크_뚜껑을_하한보다_먼저_씌운다():
    """순서가 뒤집히면 하한만 깊어진 짧은 창이 생긴다 — 그 사이에 정책이
    조이면 같은 사고가 난다."""
    link = _FakeLink(min_limit=1090, max_torque=1000)

    ggg.restore_teleop_grip(link)

    addrs = [addr for addr, _v in link.writes]
    assert addrs.index(ggg.ADDR_MAX_TORQUE_LIMIT) < addrs.index(ggg.ADDR_MIN_POSITION_LIMIT)


def test_토크_상한_쓰기가_실패하면_하한은_안_건드린다():
    link = _FakeLink(min_limit=1090, max_torque=1000,
                     fail_on={ggg.ADDR_MAX_TORQUE_LIMIT})

    assert ggg.restore_teleop_grip(link) == 1

    assert link.registers[ggg.ADDR_MIN_POSITION_LIMIT] == 1090
    assert link.registers[ggg.ADDR_LOCK] == 1


def test_토크_상한은_0에서_1000_밖을_거부한다():
    link = _FakeLink(max_torque=1000)

    assert ggg.set_max_torque(link, 1001) == 1
    assert link.writes == []


# ── 온도 상한 ──────────────────────────────────────────────────────────────


def test_온도_상한을_올린다():
    link = _FakeLink(temp_limit=70)

    assert ggg.set_temp_limit(link, 80) == 0

    assert link.registers[ggg.ADDR_MAX_TEMPERATURE_LIMIT] == 80
    assert _lock_writes(link) == [0, 1]


def test_상한_최대치_위는_거부한다():
    """코일 절연·플라스틱 기어가 감당하는 범위 밖이다. 상한을 올린다고
    서보가 더 잘 견디는 것이 아니라 보호가 늦게 켜질 뿐이다."""
    link = _FakeLink(temp_limit=70)

    assert ggg.set_temp_limit(link, 100) == 1

    assert link.writes == []
    assert link.registers[ggg.ADDR_MAX_TEMPERATURE_LIMIT] == 70


def test_온도_상한_쓰기가_실패해도_다시_잠근다():
    link = _FakeLink(temp_limit=70, fail_on={ggg.ADDR_MAX_TEMPERATURE_LIMIT})

    assert ggg.set_temp_limit(link, 80) == 1

    assert link.registers[ggg.ADDR_LOCK] == 1
    assert _lock_writes(link) == [0, 1]


def test_온도_상한은_힘_레지스터가_아니다():
    """올려도 하한·토크 상한·P 는 그대로여야 한다 — 파지력과 무관한 자리다."""
    link = _FakeLink(min_limit=1090, max_torque=1000, gain=16, temp_limit=70)

    ggg.set_temp_limit(link, 80)

    assert link.registers[ggg.ADDR_MIN_POSITION_LIMIT] == 1090
    assert link.registers[ggg.ADDR_MAX_TORQUE_LIMIT] == 1000
    assert link.registers[ggg.ADDR_POSITION_P] == 16


# ── 뚜껑 값을 골라 걸 수 있다 (2026-09-07 사용자 지시: 최대의 70%) ─────────


def test_뚜껑_값을_골라도_짝과_순서는_그대로다():
    """텔레옵의 500 이 아니라 700 을 걸어도, 하한과 함께 가고 뚜껑이 먼저다.
    순서가 뒤집히면 하한만 깊은 창이 열린다."""
    link = _FakeLink(min_limit=1090, max_torque=1000)

    assert ggg.restore_teleop_grip(link, 700) == 0

    assert link.registers[ggg.ADDR_MAX_TORQUE_LIMIT] == 700
    assert link.registers[ggg.ADDR_MIN_POSITION_LIMIT] == 1007
    addrs = [addr for addr, _v in link.writes]
    assert addrs.index(ggg.ADDR_MAX_TORQUE_LIMIT) < addrs.index(ggg.ADDR_MIN_POSITION_LIMIT)


def test_뚜껑_기본값은_텔레옵_값이다():
    """인자를 안 주면 검증된 값으로 간다 — 모르는 값이 기본이면 안 된다."""
    link = _FakeLink(min_limit=1090, max_torque=1000)

    ggg.restore_teleop_grip(link)

    assert link.registers[ggg.ADDR_MAX_TORQUE_LIMIT] == ggg.TELEOP_MAX_TORQUE


def test_뚜껑은_1000을_못_넘는다():
    link = _FakeLink(min_limit=1090, max_torque=500)

    assert ggg.restore_teleop_grip(link, 1200) == 1

    assert link.registers[ggg.ADDR_MIN_POSITION_LIMIT] == 1090, "하한은 안 건드린다"
