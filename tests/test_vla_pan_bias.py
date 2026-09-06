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

## 왜 한계를 넘으면 포기하는가

학습 분포 밖으로 팔을 밀어 넣느니 안 밀어 넣는 편이 낫다 — 분포 밖은
"조금 나쁨"이 아니라 그냥 실패다(grasp_alignment.VLA_PAN_LIMIT_DEG 주석).
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
def test_한계_밖이면_보정을_포기하고_그대로_간다(deg):
    """분포 밖으로 밀어 넣느니 안 밀어 넣는다. 다만 조용히 넘어가면 안 된다."""
    bias, report = _run(deg)
    assert bias == 0.0
    assert "분포 밖" in report


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
