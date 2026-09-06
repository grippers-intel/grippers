"""VLA 파지의 좌우 조준 — 차체 대신 servo 1 로 흡수한다 (2026-09-06).

## 왜 차체로는 안 되는가

주행 yaw 허용오차가 12도다. 회전이 bang-bang 이라 정지 명령 뒤 관성으로 약
10도를 더 돌고, 그래서 그보다 좁히면 헌팅이 난다(mission_config 의
DRIVE_YAW_TOLERANCE_DEG 주석). 그런데 그리퍼-기물 20cm 에서 12도면 좌우
42mm 로, VLA 허용치(±41mm)를 이미 넘는다.

## 왜 부호를 뒤집는가

⚠️ 2026-09-05 실기에서 그대로 넘겼다가 **반대로 돌았다** — 사용자 보고
"servo1이 돌았는데, 반대방향으로 돌았어". `yaw_correction_deg` 는 차량
좌표계이고 servo 1 의 + 는 팔 베이스 좌표계라 부호축이 반대다. INSERT 의
safe_300 이 같은 필드로 같은 것을 겪었고 거기서도 뒤집어 흡수한다.

**이 파일이 존재하는 이유가 그 부호다.** 실기에서 한 번 데인 값이라 코드가
조용히 뒤집히면 안 된다.

## 왜 한계를 넘으면 **자르는가** (버리지 않는가)

⚠️ 2026-09-07 에 계약이 뒤집혔다. 원래는 한계를 넘으면 보정을 통째로
버렸다 — "분포 밖으로 밀어 넣느니 안 밀어 넣는 게 낫다"는 논리였다.
실기가 그것을 반박했다.

그날 세 번 연속으로 `run_grasp('queen', 0.0)` 이 나갔다. Host 가 보낸 값은
`PIECE_AIM_YAW_TRIM_DEG`(4.5도, 그리퍼·마커의 **고정** 장착 오차)와 차체
잔차(±8도, 이번 회차에서만 생긴 값)의 합이라, 합이 한계를 넘는 일이
흔했다. 통째로 버리니 **항상 필요한 트림까지 같이 버려졌고**, 조준이 차체가
우연히 멈춘 각도에 그대로 맡겨졌다 — r=0.38m 에서 ±8도면 좌우 ±5.3cm 가
무작위로 남는다. 사용자가 본 "어느 날은 좌편향, 어느 날은 우편향"이 이것이다.

자르면 분포 밖으로는 안 나가면서 방향은 맞는다. **±8도는 0도보다 항상
가깝다** — 보정이 모자란 것과 반대로 가는 것은 다르다.
"""

import pytest

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.fake_host_link import FakeHostLink
from domain.ports.baseline_ports import HostCommand, MissionState
from domain.task import grasp_alignment as ga
from domain.task.baseline_mission import BaselineGraspState, BaselinePorts


class _SpyVla:
    """run_grasp 가 무엇을 받았는지만 기록한다."""

    def __init__(self, ok=True):
        self.ok = ok
        self.calls = []

    def run_grasp(self, label, pan_bias_deg=0.0):
        self.calls.append((label, pan_bias_deg))
        return self.ok


class _Profile:
    release_width_mm = 40.0
    profile = "queen"


def _run(yaw_correction_deg, ok=True):
    """주어진 Host 보정각으로 _grasp_vla 를 돌리고 (전달된 바이어스, 보고) 반환."""
    host = FakeHostLink(script=[HostCommand(
        state=MissionState.GRASP, yaw_correction_deg=yaw_correction_deg)])
    vla = _SpyVla(ok=ok)
    ports = BaselinePorts(base=FakeBase(), arm=FakeArm(), perception=None,
                          host=host, lidar=None, estop=None,
                          grasp_backend="vla", vla=vla)
    state = BaselineGraspState("queen", creep_m=0.0)
    state._grasp_vla(ports, _Profile())
    assert vla.calls, "run_grasp 가 아예 안 불렸다"
    return vla.calls[0][1], " ".join(str(r) for r in host.reports)


# ── 부호 ───────────────────────────────────────────────────────────────────


def test_부호를_뒤집어_넘긴다():
    """⚠️ 2026-09-05 실기: 그대로 넘겼다가 servo 1 이 반대로 돌았다."""
    bias, _ = _run(+5.0)
    assert bias == -5.0


def test_반대쪽도_뒤집는다():
    bias, _ = _run(-3.0)
    assert bias == +3.0


# ── 한계 ───────────────────────────────────────────────────────────────────


def test_한계_안이면_적용한다():
    bias, report = _run(ga.VLA_PAN_LIMIT_DEG - 0.5)
    assert bias == pytest.approx(-(ga.VLA_PAN_LIMIT_DEG - 0.5))
    assert "보정" in report


def test_한계_경계는_적용한다():
    """<= 이지 < 가 아니다 — 경계에서 조용히 포기하면 원인을 못 찾는다."""
    bias, _ = _run(ga.VLA_PAN_LIMIT_DEG)
    assert bias == pytest.approx(-ga.VLA_PAN_LIMIT_DEG)


@pytest.mark.parametrize("deg", [12.0, -12.0, 45.0])
def test_한계_밖이면_한계까지_잘라서_넣는다(deg):
    """⚠️ 2026-09-07 에 뒤집힌 계약이다 — 파일 상단 설명 참고.

    0 으로 버리면 고정 트림까지 사라져 조준이 무작위가 된다."""
    bias, report = _run(deg)
    assert bias == pytest.approx(-ga.VLA_PAN_LIMIT_DEG * (1 if deg > 0 else -1))
    assert "분포 밖" in report


@pytest.mark.parametrize("deg", [12.0, -12.0, 45.0])
def test_잘린_보정도_방향은_맞다(deg):
    """자르기의 존재 이유 — 크기는 모자라도 부호는 옳아야 한다."""
    bias, _ = _run(deg)
    assert bias * deg < 0, "부호가 Host 보정과 반대여야 한다(좌표계가 반대)"


def test_잘려도_0보다는_가깝다():
    """'모자란 보정'과 '보정 없음'은 다르다는 것을 수치로 못 박는다."""
    wanted = -20.0                      # Host 가 원한 servo 1 각
    bias, _ = _run(20.0)
    assert abs(bias - wanted) < abs(0.0 - wanted)


# ── 없을 때 ────────────────────────────────────────────────────────────────


def test_보정이_0이면_아무것도_안_한다():
    """기존 경로와 100% 같아야 한다 — 이 기능을 껐을 때의 동작이다."""
    bias, report = _run(0.0)
    assert bias == 0.0
    assert "보정" not in report


def test_Host_명령이_없어도_안_죽는다():
    """파지 도중에 예외로 죽는 것이 최악이다."""
    vla = _SpyVla()
    ports = BaselinePorts(base=FakeBase(), arm=FakeArm(), perception=None,
                          host=FakeHostLink(script=[]), lidar=None, estop=None,
                          grasp_backend="vla", vla=vla)
    BaselineGraspState("queen", creep_m=0.0)._grasp_vla(ports, _Profile())
    assert vla.calls[0][1] == 0.0


# ── 실패 경로 ──────────────────────────────────────────────────────────────


def test_실패하면_물체를_놓고_접는다():
    """⚠️ 2026-09-06: 놓지 않고 접으면 fold 의 자동 닫기가 물체를 물어서,
    정책은 실패했는데 물건은 들려 있는 상태가 된다."""
    vla = _SpyVla(ok=False)
    arm = FakeArm()
    ports = BaselinePorts(base=FakeBase(), arm=arm, perception=None,
                          host=FakeHostLink(script=[]), lidar=None, estop=None,
                          grasp_backend="vla", vla=vla)
    BaselineGraspState("queen", creep_m=0.0)._grasp_vla(ports, _Profile())
    assert arm.gripper_widths and arm.gripper_widths[-1] == pytest.approx(
        _Profile.release_width_mm), "실패 뒤 그리퍼를 release 폭으로 열어야 한다"


# ── 소비 계약 ──────────────────────────────────────────────────────────────


def test_다른_곳에서_먼저_읽어도_보정이_살아_있다():
    """⚠️ `latest_command()` 는 **한 번 읽으면 소비**된다(아직 안 읽은 새
    명령이 없으면 None). 주행 루프에는 맞는 계약이지만, 파지는 그 뒤로
    `fold_to_cradle` 등 몇 초를 보내고 나서 한 번 읽는다 — 그 사이 새 패킷이
    안 왔으면 조준 보정이 조용히 0 이 된다.

    그래서 `_grasp_vla` 는 소비하지 않는 `last_command()` 를 쓴다. 조준
    보정은 못 읽으면 멈춰야 하는 값이 아니라 마지막 값을 쓰면 되는 값이다 —
    차체는 이미 그 자리에 서 있다."""
    host = FakeHostLink(script=[HostCommand(
        state=MissionState.GRASP, yaw_correction_deg=+5.0)])
    host.latest_command()                    # 주행 루프가 먼저 소비했다

    vla = _SpyVla()
    ports = BaselinePorts(base=FakeBase(), arm=FakeArm(), perception=None,
                          host=host, lidar=None, estop=None,
                          grasp_backend="vla", vla=vla)
    BaselineGraspState("queen", creep_m=0.0)._grasp_vla(ports, _Profile())

    assert vla.calls[0][1] == -5.0


def test_last_command는_읽어도_안_사라진다():
    """계약 자체 — 몇 번을 읽어도 같은 값이다."""
    host = FakeHostLink(script=[HostCommand(
        state=MissionState.GRASP, yaw_correction_deg=+5.0)])
    assert [host.last_command().yaw_correction_deg for _ in range(3)] == [5.0] * 3
