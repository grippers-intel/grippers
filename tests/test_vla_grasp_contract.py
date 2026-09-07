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
    # 실제 프로파일과 같은 값이어야 한다 — 40mm 로는 release_until_open 의
    # 위치 확인(GRIPPER_RELEASED_MIN_RAW)을 못 넘어 "못 놓았다"가 된다.
    release_width_mm = 168.0
    profile = "queen"


# ── 좌우 조준은 안 한다 (2026-09-07 사용자 지시) ──────────────────────────
#
# 예전에는 Host 의 yaw_correction_deg 를 부호 뒤집고 ±8도로 잘라 정책의 pan
# 출력에 더했다. 그 계약을 고정하던 시험 열 개가 여기 있었는데, 계약 자체가
# 없어졌으므로 같이 지우고 **안 한다는 것**을 대신 고정한다.


def _ports_with_correction(deg):
    host = FakeHostLink(script=[HostCommand(
        state=MissionState.GRASP, yaw_correction_deg=deg)])
    return BaselinePorts(base=FakeBase(), arm=FakeArm(), perception=None,
                         host=host, lidar=None, estop=None, vla=_SpyVla())


@pytest.mark.parametrize("deg", [0.0, 3.5, -3.5, 20.0, -20.0])
def test_Host_가_보정을_보내도_정책에_안_넣는다(deg):
    """Host 는 여전히 계산해 보낸다 — Pi 가 안 읽을 뿐이다."""
    ports = _ports_with_correction(deg)

    BaselineGraspState("queen")._grasp_vla(ports, _Profile())

    assert ports.vla.calls == [("queen", 0.0)], (
        f"보정 {deg}도가 정책에 흘러들어갔다: {ports.vla.calls}")


def test_보정을_읽지도_않는다():
    """읽기만 해도 소비 순서가 얽힌다 — 아예 안 본다."""
    import inspect

    from domain.task.baseline_mission import BaselineGraspState as G

    # 주석에는 "왜 안 하는지"가 남아 있어야 하므로 코드 줄만 본다.
    code = chr(10).join(line for line in inspect.getsource(G._grasp_vla).splitlines()
                        if not line.strip().startswith("#"))
    assert "yaw_correction_deg" not in code
    assert "last_command" not in code
    assert "pan_bias" not in code


# ── 실패 경로 ──────────────────────────────────────────────────────────────


def test_실패하면_물체를_놓고_접는다():
    """⚠️ 2026-09-06: 놓지 않고 접으면 fold 의 자동 닫기가 물체를 물어서,
    정책은 실패했는데 물건은 들려 있는 상태가 된다."""
    vla = _SpyVla(ok=False)
    arm = FakeArm()
    # 턱을 비워 둔다 — 물고 있으면 놓지 않고 판정으로 넘긴다(2026-09-07).
    arm.jaw_blocked_raw = None
    arm.gripper_position_raw_value = FakeArm.EMPTY_RAW
    ports = BaselinePorts(base=FakeBase(), arm=arm, perception=None,
                          host=FakeHostLink(script=[]), lidar=None, estop=None, vla=vla)
    BaselineGraspState("queen")._grasp_vla(ports, _Profile())
    assert _Profile.release_width_mm in arm.gripper_widths, (
        "실패 뒤 그리퍼를 release 폭으로 열어야 한다")
    # 놓은 것을 확인한 **뒤에** 접기용으로 닫는다(_release_and_fold 주석).
    assert arm.gripper_widths[-1] == pytest.approx(9.0)


# ── 루프 실패 != 못 잡았다 (2026-09-07 실기) ──────────────────────────────


def test_루프가_끝을_못_봐도_물고_있으면_안_놓는다():
    """실기 로그:

        16청크(33.6s)를 다 썼는데 복귀를 못 봤습니다   -> run_grasp=False
        그 직후 그리퍼 위치 1067                        -> 물고 있었다

    정책은 집었는데 노드의 "복귀" 신호만 안 떴다. 그걸 실패로 접으면 물체를
    도로 놓고 처음부터 다시 한다 — 사용자 보고 "이번에는 잡았는데도
    approach_piece 로 돌아갔다"."""
    from domain.task import baseline_constants as bc

    vla = _SpyVla(ok=False)
    arm = FakeArm()
    arm.gripper_position_raw_value = bc.held_threshold_raw(0.0) + 10
    arm.jaw_blocked_raw = arm.gripper_position_raw_value
    ports = BaselinePorts(base=FakeBase(), arm=arm, perception=None,
                          host=FakeHostLink(script=[]), lidar=None, estop=None, vla=vla)

    assert BaselineGraspState("queen")._grasp_vla(ports, _Profile()) is True
    assert _Profile.release_width_mm not in arm.gripper_widths, (
        "물고 있는데 놓았다 — 성공한 파지를 버린다")


def test_루프도_실패하고_턱도_비었으면_놓고_접는다():
    """위와 짝. 문턱 아래면 그대로 실패다 — 규칙이 한쪽으로만 느슨해지면
    빈 턱을 물었다고 보고하게 된다."""
    from domain.task import baseline_constants as bc

    vla = _SpyVla(ok=False)
    arm = FakeArm()
    arm.jaw_blocked_raw = None
    arm.gripper_position_raw_value = bc.held_threshold_raw(0.0) - 10
    ports = BaselinePorts(base=FakeBase(), arm=arm, perception=None,
                          host=FakeHostLink(script=[]), lidar=None, estop=None, vla=vla)

    assert BaselineGraspState("queen")._grasp_vla(ports, _Profile()) is False
    assert _Profile.release_width_mm in arm.gripper_widths


# ── 서보가 죽었을 때 초당 열 번씩 덤비지 않는다 (2026-09-07 실기) ─────────


def test_서보가_응답_안_하면_다음_시도까지_쉰다():
    """실기: servo 6 이 과전류로 떨어지자 fold 가 매번 실패했고, Host 가
    실패를 받자마자 리셋해 0.35초 주기로 **37번째 시도**까지 갔다.

    재시도가 틀린 게 아니라 간격이 없는 것이 틀렸다 — 같은 로그에서 버스는
    4초쯤 뒤에 스스로 돌아왔다."""
    slept = []

    class DeadBusArm(FakeArm):
        def fold_to_cradle(self) -> bool:
            return False

        def gripper_position_raw(self) -> int:
            return -1          # 읽기 실패 = 버스가 나갔다

    arm = DeadBusArm()
    host = FakeHostLink(script=[])
    ports = BaselinePorts(base=FakeBase(), arm=arm, perception=None,
                          host=host, lidar=None, estop=None, vla=_SpyVla())

    state = BaselineGraspState("queen")
    state.HARDWARE_FAULT_DWELL_SEC = 0.0
    import domain.task.baseline_mission as bm
    real_sleep = bm.time.sleep
    bm.time.sleep = lambda s: slept.append(s)
    try:
        assert state._grasp_vla(ports, _Profile()) is False
    finally:
        bm.time.sleep = real_sleep

    assert slept, "서보가 죽었는데 곧바로 실패를 돌려줬다 — 스핀이 된다"
    detail = " ".join(d for _k, _s, d, _f in host.reports)
    assert "servo 6" in detail, f"무엇이 죽었는지 보고에 없다: {detail}"


def test_자세만_못_잡은_것은_안_쉰다():
    """버스가 살아 있으면 기다릴 이유가 없다 — 그때는 다시 해 보는 게 맞다."""
    slept = []

    class FoldFailsArm(FakeArm):
        def fold_to_cradle(self) -> bool:
            return False

    ports = BaselinePorts(base=FakeBase(), arm=FoldFailsArm(), perception=None,
                          host=FakeHostLink(script=[]), lidar=None, estop=None,
                          vla=_SpyVla())

    import domain.task.baseline_mission as bm
    real_sleep = bm.time.sleep
    bm.time.sleep = lambda s: slept.append(s)
    try:
        assert BaselineGraspState("queen")._grasp_vla(ports, _Profile()) is False
    finally:
        bm.time.sleep = real_sleep

    assert not slept


def test_위치를_못_읽으면_놓지_않는다():
    """⚠️ 2026-09-07 실기. servo 6 이 과전류로 응답을 멈춘 채 물체를 물고
    있었는데 위치가 -1 로 와서 "안 물었다"로 읽혔다. 놓기도 같은 서보라
    4회 전부 실패해 "놓지 못했다"를 보고하고 파지까지 실패로 접었다.

    2026-09-05 의 부하 0.0 사고와 같은 실수 — 모르는 것을 아니라고 단정했다.
    놓는 것은 되돌릴 수 없으니 확신이 있을 때만 한다."""
    vla = _SpyVla(ok=False)

    class UnreadableArm(FakeArm):
        def gripper_position_raw(self) -> int:
            return -1

    arm = UnreadableArm()
    ports = BaselinePorts(base=FakeBase(), arm=arm, perception=None,
                          host=FakeHostLink(script=[]), lidar=None, estop=None, vla=vla)

    assert BaselineGraspState("queen")._grasp_vla(ports, _Profile()) is True
    assert _Profile.release_width_mm not in arm.gripper_widths, (
        "위치를 못 읽었는데 놓았다 — 물고 있으면 물체를 떨어뜨린다")
