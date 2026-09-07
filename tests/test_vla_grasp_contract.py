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
from domain.adapters.fake.fake_host_link import FakeHostLink, FakeLidar
from domain.adapters.fake.scripted_perception import ScriptedPerception
from domain.ports.baseline_ports import HostCommand, MissionState, Report
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


# ── 좌우 조준: 탑뷰 θ 를 servo 1 로 지운다 (2026-09-08 실측으로 복원) ────
#
# servo 1 을 스윕하며 탑뷰를 읽어 확정한 관계:
#
#     yaw = -0.975 * servo1 + 8.65도    잔차 RMS 0.18도
#
# 마커가 servo 1 회전축 위에 있어 위치는 제자리이고 yaw 만 1:1 로 돈다.
# 그래서 dθ/d(servo1) = +1 이고, θ 를 지우려면 servo 1 을 -θ 만큼 돌린다.


def _ports_with_correction(deg):
    host = FakeHostLink(script=[HostCommand(
        state=MissionState.GRASP, yaw_correction_deg=deg)])
    return BaselinePorts(base=FakeBase(), arm=FakeArm(), perception=None,
                         host=host, lidar=None, estop=None, vla=_SpyVla())


@pytest.mark.parametrize("theta", [3.5, -3.5, 0.0])
def test_한계_안이면_부호를_뒤집어_그대로_넣는다(theta):
    """실측으로 확정된 부호다 — 여태 가정만 하던 -θ 가 맞았다."""
    ports = _ports_with_correction(theta)

    BaselineGraspState("queen")._grasp_vla(ports, _Profile())

    assert ports.vla.calls == [("queen", pytest.approx(-theta))]


@pytest.mark.parametrize("theta, expected", [(20.0, -8.0), (-20.0, 8.0)])
def test_한계_밖이면_자른다_버리지_않는다(theta, expected):
    """⚠️ 2026-09-07 실기에서 세 번 연속 0.0 이 나갔다. 예전 코드는 한계를
    넘으면 보정을 **통째로 버렸는데**, 그러면 조준이 차체가 우연히 멈춘
    각도에 그대로 맡겨진다. ±8도가 0도보다 항상 가깝다."""
    ports = _ports_with_correction(theta)

    BaselineGraspState("queen")._grasp_vla(ports, _Profile())

    assert ports.vla.calls == [("queen", pytest.approx(expected))]


def test_트림은_기본이_0이고_그대로_더해진다():
    """b(마커 정면과 그리퍼 방향의 고정 각도)는 아직 못 쟀다 — 탑뷰 두 대가
    158mm 어긋나 있어 그 위에서는 1도 정밀도로 못 잡는다. 0 으로 두고 실기에서
    잡되, 더해지는 자리는 지금 고정해 둔다."""
    import domain.task.baseline_mission as bm

    assert bm.VLA_PAN_TRIM_DEG == 0.0

    ports = _ports_with_correction(2.0)
    try:
        bm.VLA_PAN_TRIM_DEG = 1.5
        BaselineGraspState("queen")._grasp_vla(ports, _Profile())
    finally:
        bm.VLA_PAN_TRIM_DEG = 0.0

    assert ports.vla.calls == [("queen", pytest.approx(-2.0 + 1.5))]


def test_vla_only면_조준을_아예_안_넣는다():
    """정책만 돌려 보는 진단 모드다 — 우리가 얹은 것은 전부 뺀다."""
    host = FakeHostLink(script=[HostCommand(
        state=MissionState.GRASP, yaw_correction_deg=6.0)])
    ports = BaselinePorts(base=FakeBase(), arm=FakeArm(), perception=None,
                          host=host, lidar=None, estop=None, vla=_SpyVla(),
                          vla_only=True)

    BaselineGraspState("queen")._grasp_vla(ports, _Profile())

    assert ports.vla.calls == [("queen", 0.0)]


# ── 실패 경로 ──────────────────────────────────────────────────────────────


# ── 루프 실패 != 못 잡았다 (2026-09-07 실기) ──────────────────────────────


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




# ── 파지 성공/실패 판정은 없다 (2026-09-07 사용자 지시) ───────────────────


def _grasp_ports(arm, vla_ok=True):
    return BaselinePorts(base=FakeBase(), arm=arm,
                         perception=ScriptedPerception(), host=FakeHostLink(),
                         lidar=FakeLidar(), estop=None, vla=_SpyVla(ok=vla_ok),
                         use_depth_gate=False)


def test_턱이_비어도_CARRY_로_간다():
    """사용자: "실패하게 되면 어차피 그 상태에서 다시 시작하게 될텐데 왜 굳이
    실패 성공을 만들어놓은건지 이해가 안돼."

    판정은 틀릴 기회만 만들었고 실제로 세 번 틀려서 성공한 파지를 버렸다.
    못 집었으면 빈 그리퍼로 한 바퀴 돌고 다시 온다."""
    arm = FakeArm()
    arm.jaw_blocked_raw = None
    arm.gripper_position_raw_value = FakeArm.EMPTY_RAW
    ports = _grasp_ports(arm)

    nxt = BaselineGraspState("queen").execute(ports)

    assert Report.GRASP_DONE in ports.host.reported_kinds
    assert Report.GRASP_FAILED not in ports.host.reported_kinds
    assert nxt.name == MissionState.CARRY


def test_위치를_못_읽어도_CARRY_로_간다():
    """읽기 실패(-1)로 실패를 만들던 것이 2026-09-07 밤 사고였다."""
    class UnreadableArm(FakeArm):
        def gripper_position_raw(self) -> int:
            return -1

    ports = _grasp_ports(UnreadableArm())

    nxt = BaselineGraspState("queen").execute(ports)

    assert Report.GRASP_FAILED not in ports.host.reported_kinds
    assert nxt.name == MissionState.CARRY


def test_정책_루프가_실패해도_안_놓는다():
    """놓기는 되돌릴 수 없다 — 루프 미완료는 "못 잡았다"가 아니다."""
    arm = FakeArm()
    ports = _grasp_ports(arm, vla_ok=False)

    nxt = BaselineGraspState("queen").execute(ports)

    assert _Profile.release_width_mm not in arm.gripper_widths
    assert nxt.name == MissionState.CARRY


def test_팔을_못_움직이면_그때만_실패다():
    """유일하게 남긴 실패다. 팔이 안 접히면 주행이 거부되므로(arm_parked)
    그대로 CARRY 로 보내면 안 된다."""
    class StuckArm(FakeArm):
        def fold_to_cradle(self) -> bool:
            return False

    ports = _grasp_ports(StuckArm())

    BaselineGraspState("queen").execute(ports)

    assert Report.GRASP_FAILED in ports.host.reported_kinds
    detail = " ".join(d for _k, _s, d, _f in ports.host.reports)
    assert "팔을 움직이지 못했다" in detail


# ── servo 6 이 죽으면 상자로 간다 (2026-09-08 사용자 지시) ────────────────


def test_servo6이_죽으면_제자리_재시도_대신_상자로_간다():
    """사용자: "오히려 6번서보모터가 오류가 났을때 상자로 가게끔 하는 것이
    좋을 거 같아."

    servo 6 이 죽은 채 제자리에서 재시도해 봐야 파지가 될 리 없고, 그 사이
    물체를 문 채일 수도 있다. 주행하는 몇 초가 곧 회복 시간이기도 하다
    (2026-09-07 실측: 버스가 약 4초 뒤 돌아왔다)."""
    class DeadGripperArm(FakeArm):
        def fold_to_cradle(self) -> bool:
            return False          # servo 6 읽기 실패로 정렬이 죽는다

        def gripper_position_raw(self) -> int:
            return -1

    ports = _grasp_ports(DeadGripperArm())

    nxt = BaselineGraspState("queen").execute(ports)

    assert nxt.name == MissionState.CARRY
    assert Report.GRASP_FAILED not in ports.host.reported_kinds
    detail = " ".join(d for _k, _s, d, _f in ports.host.reports)
    assert "servo 6" in detail


def test_팔이_통째로_갇히면_그때는_실패다():
    """CARRY 전환마저 안 되면 팔이 알려진 자세에 없다 — 그대로 주행시키면
    안 된다."""
    class StuckArm(FakeArm):
        def fold_to_cradle(self) -> bool:
            return False

        def move_to_floor_pose(self, profile, stage) -> bool:
            self.floor_pose_calls.append((profile, stage))
            return False

        def gripper_position_raw(self) -> int:
            return -1

    ports = _grasp_ports(StuckArm())

    BaselineGraspState("queen").execute(ports)

    assert Report.GRASP_FAILED in ports.host.reported_kinds
