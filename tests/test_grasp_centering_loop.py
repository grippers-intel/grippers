"""GRASP 좌우 정렬 폐루프 — 2026-08-28 실기에서 두 번 막혔던 지점.

기존 테스트는 **한 사이클**만 본다(`test_baseline_mission.py`). 그런데 실기에서
막힌 것은 여러 사이클에 걸친 루프였고, 한 사이클씩 보면 매번 정상이었다.

  * run1: 같은 +27mm 를 9번 반복하고 끝났다. 보정이 서보 격자에 걸려
    통째로 버려졌는데 서비스는 성공을 보고했다.
  * run6: 보정이 실제로 먹었는데(servo 1 이 +10.1도 돌고 잔차 -10 raw)
    다음 관측이 여전히 +53mm 였다. 두 번째 보정은 누적 +19.6도가 되어
    한계에 막혔고, Host 가 차를 다시 세운 뒤 뎁스캠이 목표를 놓쳤다.

두 경우 모두 **루프가 끝나는가**가 핵심이다. 여기서는 하드웨어 없이
그것만 본다. 서보 격자 문제 자체는 `test_arm_hardware_contract.py` 의
정적 검사가 막는다.
"""

import math

import pytest

from domain.adapters.fake.fake_arm import FakeArm
from domain.ports.baseline_ports import HostCommand, MissionState, Report
from domain.task import baseline_constants as bc
from domain.task.baseline_mission import BaselineApproachState, BaselineGraspState
from domain.values import TargetObservation

from test_baseline_mission import EMPTY_LOAD, _ports  # noqa: F401

# queen 의 턱 쓸기 구간은 [196.9mm, 266.9mm] 다(JAW_LINE_DEPTH_FORWARD_M +
# GRASP_CREEP_FORWARD_MM). 그 안쪽 값을 써야 전후 판정에서 먼저 걸리지 않고
# 좌우 판정까지 간다.
FORWARD_M = 0.210
from domain.adapters.fake.fake_host_link import FakeHostLink
from domain.adapters.fake.scripted_perception import ScriptedPerception

# arm_driver_node.MAX_BASE_YAW_OFFSET_RAD 와 같은 값.
MAX_YAW_RAD = math.radians(15.0)
# servo 1 축에서 턱까지. `grasp_alignment.servo1_offset_for` 가 쓰는 것과
# 같은 실측값이어야 결합 모형이 판정과 어긋나지 않는다.
ARM_REACH_M = bc.SERVO1_AXIS_TO_JAW_MM / 1000.0
# 무한 루프는 이 프로젝트의 최대 리스크다 — 상한 안에 못 끝나면 실패다.
MAX_CYCLES = 40


class LimitedYawArm(FakeArm):
    """servo 1 한계각을 **누적**으로 집행하는 팔.

    기본 `FakeArm` 은 `offset_base_yaw` 가 항상 성공한다. 그러면 실기에서
    루프를 끝낸 바로 그 장치(한계각 거부)가 테스트에 없어서, 안 끝나는
    루프가 통과해 버린다. 실물은 교시 정면 기준 절대각으로 막는다
    (`arm_driver_node._on_offset_base_yaw`).
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.yaw_total_rad = 0.0

    def offset_base_yaw(self, offset_rad: float) -> bool:
        if abs(self.yaw_total_rad + offset_rad) > MAX_YAW_RAD:
            return False
        self.yaw_total_rad += offset_rad
        self.yaw_offsets.append(offset_rad)
        return True


class CoupledPerception(ScriptedPerception):
    """servo 1 이 돌면 좌우 관측이 **따라 줄어드는** 뎁스캠.

    `arm` 의 누적 각도를 보고 좌우값을 다시 계산한다. `coupling` 이 1.0 이면
    각도가 그대로 좌우 오차를 지우고, 0.0 이면 아무리 돌려도 관측이 안
    바뀐다(run6 에서 실제로 본 모습).
    """

    def __init__(self, arm, label, forward_m, lateral_m, coupling=1.0):
        super().__init__(label=label, forward_m=forward_m,
                         lateral_m=lateral_m, metric_ok=True)
        self._arm = arm
        self._lateral0 = lateral_m
        self._coupling = coupling

    def identify_target(self):
        self.identify_calls += 1
        # 턱은 servo 1 축을 중심으로 호를 그린다 — 판정이 쓰는
        # `atan2(오차, 팔 길이)` 의 역이다.
        moved = math.sin(self._arm.yaw_total_rad) * ARM_REACH_M * self._coupling
        return TargetObservation(self._label, self._forward_m,
                                 self._lateral0 - moved, True)


def _run_approach(ports, cycles=MAX_CYCLES):
    """APPROACH 에 GRASP 지시를 계속 주며 상태 전이를 따라간다."""
    state = BaselineApproachState()
    for n in range(1, cycles + 1):
        state = state.execute(ports)
        if not isinstance(state, BaselineApproachState):
            return state, n
    return state, cycles


def _grasp_ports(arm, perception, host=None):
    host = host or FakeHostLink(
        [HostCommand(MissionState.GRASP, stop=True)] * (MAX_CYCLES + 1))
    return _ports(host=host, arm=arm, perception=perception)


def test_보정이_먹으면_몇_사이클_안에_내려간다():
    """정상 결합. 폐루프가 닫히고 GRASP 로 넘어가야 한다."""
    arm = LimitedYawArm(load_ratio=EMPTY_LOAD)
    zero = bc.DEPTH_LATERAL_TO_JAW_CENTER_M["queen"]
    perception = CoupledPerception(arm, "queen", FORWARD_M,
                                   zero + 0.030, coupling=1.0)
    ports = _grasp_ports(arm, perception)

    state, cycles = _run_approach(ports)

    assert isinstance(state, BaselineGraspState), (
        f"{cycles} 사이클 안에 GRASP 로 못 넘어갔다")
    assert arm.yaw_offsets, "보정을 한 번도 안 걸었다"


def test_보정이_안_먹으면_영원히_반복하지_않는다():
    """run6 재현 — 돌려도 관측이 안 변한다. 한계각이 루프를 끝내야 한다."""
    arm = LimitedYawArm(load_ratio=EMPTY_LOAD)
    zero = bc.DEPTH_LATERAL_TO_JAW_CENTER_M["queen"]
    perception = CoupledPerception(arm, "queen", FORWARD_M,
                                   zero + 0.030, coupling=0.0)
    host = FakeHostLink([HostCommand(MissionState.GRASP, stop=True)] * (MAX_CYCLES + 1))
    ports = _grasp_ports(arm, perception, host=host)

    for _ in range(MAX_CYCLES):
        BaselineApproachState().execute(ports)

    # 한계에 걸린 뒤로는 팔이 더 돌지 않아야 한다.
    assert abs(arm.yaw_total_rad) <= MAX_YAW_RAD + 1e-9
    # 그리고 Host 에게 "차를 다시 세워라"가 나가야 한다 — 그게 유일한 출구다.
    assert Report.GRASP_BLOCKED in host.reported_kinds
    assert any("재회전" in detail for _kind, _state, detail, *_ in host.reports), (
        f"재회전 요구가 없다 — 보고 {[r[2] for r in host.reports[-3:]]}")


def test_한계각_안에서_고칠_수_있는_치우침은_Host를_안_부른다():
    """±15도(약 64mm) 안쪽은 팔이 혼자 끝내야 한다. 매번 차를 다시 세우면
    접근이 끝나지 않는다."""
    arm = LimitedYawArm(load_ratio=EMPTY_LOAD)
    zero = bc.DEPTH_LATERAL_TO_JAW_CENTER_M["queen"]
    perception = CoupledPerception(arm, "queen", FORWARD_M,
                                   zero + 0.020, coupling=1.0)
    host = FakeHostLink([HostCommand(MissionState.GRASP, stop=True)] * (MAX_CYCLES + 1))
    ports = _grasp_ports(arm, perception, host=host)

    state, _ = _run_approach(ports)

    assert isinstance(state, BaselineGraspState)
    assert not any("재회전" in detail for _k, _s, detail, *_ in host.reports)


@pytest.mark.parametrize("lateral_mm", [-30.0, 30.0])
def test_보정_방향이_양쪽_모두_수렴한다(lateral_mm):
    """부호를 뒤집어도 같은 사이클 수 안에 끝나야 한다."""
    arm = LimitedYawArm(load_ratio=EMPTY_LOAD)
    zero = bc.DEPTH_LATERAL_TO_JAW_CENTER_M["queen"]
    perception = CoupledPerception(arm, "queen", FORWARD_M,
                                   zero + lateral_mm / 1000.0, coupling=1.0)
    ports = _grasp_ports(arm, perception)

    state, _ = _run_approach(ports)

    assert isinstance(state, BaselineGraspState)
