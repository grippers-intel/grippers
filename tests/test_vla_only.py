"""정책만 돌려 본다 — 정책이 안 시킨 팔 동작을 전부 걷어낸다 (2026-09-07).

## 지시

"파지 시퀀스에 팀원의 하드코딩 부분이 계속 들어가는 거 같은데 그 부분 우선은
잠시 배제해줄래? 그냥 vla 로만 동작하는 것을 확인하고 싶어."

## 무엇이 남아 있었나

정책을 켜도 정책이 안 시킨 팔 동작이 셋 있었다.

    creep_m 관문     팀원 정렬 코드가 낸 전진 거리를 모르면 파지를 **시작도**
                     안 한다. 그런데 _grasp_vla 는 creep_forward 를 아예 안
                     한다 — 정책과 무관한 값이 정책을 막고 있었다.
    remember_target  뎁스 관측. use_depth_gate=false 에서는 판정에 안 쓰이고,
                     실기에서 매번 3초 타임아웃으로 실패했다(09-07 로그 2회).
    CARRY 전환       정책이 끝낸 자세에서 손목만 올리는 별도 동작. 사용자가
                     "종료방식이 동료의 하드코딩과 똑같다"고 본 자리다.

## 무엇을 남겼나

**시작 자세(fold_to_cradle)는 안 건드린다.** 학습 회차 118개의 첫 관측이
전부 IDLE 크래들이었다 — 그것까지 빼면 정책이 분포 밖에서 시작한다. 이건
팀원 하드코딩이 아니라 정책을 돌리기 위한 조건이다.

## 대가

⚠️ **운반·투하가 깨진다.** 물체를 문 채 IDLE 에 있으면 그리퍼가 라이다
정면을 79% 가려 바구니를 못 본다(2026-08-26 실측). 진단 전용 스위치다.
"""

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.fake_host_link import FakeHostLink
from domain.adapters.fake.scripted_perception import ScriptedPerception
from domain.ports.baseline_ports import Report
from domain.task.baseline_mission import BaselineGraspState, BaselinePorts


class _SpyVla:
    def __init__(self, ok=True):
        self.ok, self.calls = ok, []

    def run_grasp(self, label, pan_bias_deg=0.0):
        self.calls.append((label, pan_bias_deg))
        return self.ok


def _ports(vla_only, ok=True, arm=None):
    return BaselinePorts(base=FakeBase(), arm=arm or FakeArm(), host=FakeHostLink(),
                         perception=ScriptedPerception(), lidar=None, estop=None, vla=_SpyVla(ok=ok),
                         use_depth_gate=False, vla_only=vla_only)


def _stages(ports):
    return [stage for _profile, stage in ports.arm.floor_pose_calls]


# ── creep 관문 ─────────────────────────────────────────────────────────────


def test_전진거리를_몰라도_정책은_돈다():
    """⚠️ 이 관문이 정책을 시작도 못 하게 막고 있었다. _grasp_vla 는
    creep_forward 를 아예 안 하므로 이 값과 무관하다."""
    ports = _ports(vla_only=True)

    BaselineGraspState("queen").execute(ports)

    assert ports.vla.calls, "정책이 안 불렸다"


def test_뎁스_관측을_안_한다():
    """실기에서 매번 3초 타임아웃으로 실패하던 호출이다."""
    ports = _ports(vla_only=True)

    BaselineGraspState("queen").execute(ports)

    assert ports.perception.remember_target_calls == 0


def test_CARRY_로_안_옮긴다():
    """정책이 끝낸 자세 그대로 둔다 — 사용자가 "동료의 하드코딩 같다"고 본
    그 동작이다."""
    ports = _ports(vla_only=True)

    BaselineGraspState("queen").execute(ports)

    assert "carry" not in _stages(ports)


def test_건너뛴_사실을_보고한다():
    """조용히 건너뛰면 왜 운반이 안 되는지 못 찾는다."""
    ports = _ports(vla_only=True)

    BaselineGraspState("queen").execute(ports)

    assert "vla_only" in " ".join(d for _r, _s, d, _f in ports.host.reports)


def test_평소에는_CARRY_로_옮긴다():
    ports = _ports(vla_only=False)

    BaselineGraspState("queen").execute(ports)

    assert "carry" in _stages(ports)


# ── 남겨 둔 것 ─────────────────────────────────────────────────────────────


def test_시작_자세는_그대로_맞춘다():
    """⚠️ 이것까지 빼면 정책이 분포 밖에서 시작한다 — 학습 회차 118개의 첫
    관측이 전부 IDLE 크래들이었다."""
    ports = _ports(vla_only=True)

    BaselineGraspState("queen").execute(ports)

    assert ports.arm.fold_calls >= 1


def test_실패_판정은_그대로다():
    """정책이 실패로 끝나면 vla_only 여도 실패다 — 판정까지 끄는 게 아니다.

    ⚠️ 턱을 비워 둔다. 2026-09-07 부터 "루프는 실패했는데 턱은 물고 있다"는
    실패로 안 친다(_grasp_vla 의 그 주석) — 여기서 보려는 것은 그 경우가
    아니라 **정말 못 잡은** 경우다."""
    arm = FakeArm()
    arm.jaw_blocked_raw = None
    arm.gripper_position_raw_value = FakeArm.EMPTY_RAW
    ports = _ports(vla_only=True, ok=False, arm=arm)

    BaselineGraspState("queen").execute(ports)

    assert Report.GRASP_FAILED in ports.host.reported_kinds


# ── 조준 바이어스 ──────────────────────────────────────────────────────────


def test_바이어스도_안_넣는다():
    """⚠️ 이것도 정책이 아니라 우리가 얹은 보정이다.

    실기 녹화(2026-09-06)에서 정책의 shoulder_pan 출력은 한 판 내내
    -3.50 ~ -4.18 도, 폭 0.7도였다. 거기 ±8도를 더하니 바이어스가 정책
    출력의 10배다 — 구조는 상대지만 효과는 사실상 절대 조준이다.

    부호 규약이 GRASP 경로에서 아직 실기 검증이 안 됐으므로, 빼고 돌려
    비교할 수 있어야 한다."""
    from domain.ports.baseline_ports import HostCommand, MissionState
    host = FakeHostLink(script=[HostCommand(
        state=MissionState.GRASP, yaw_correction_deg=+5.0)])
    ports = BaselinePorts(base=FakeBase(), arm=FakeArm(), host=host,
                          perception=ScriptedPerception(), lidar=None, estop=None, vla=_SpyVla(),
                          use_depth_gate=False, vla_only=True)

    BaselineGraspState("queen").execute(ports)

    assert ports.vla.calls[0][1] == 0.0


def test_평소에는_바이어스를_넣는다():
    """기본 동작은 안 바뀌어야 한다."""
    from domain.ports.baseline_ports import HostCommand, MissionState
    host = FakeHostLink(script=[HostCommand(
        state=MissionState.GRASP, yaw_correction_deg=+5.0)])
    ports = BaselinePorts(base=FakeBase(), arm=FakeArm(), host=host,
                          perception=ScriptedPerception(), lidar=None, estop=None, vla=_SpyVla(),
                          use_depth_gate=False, vla_only=False)

    BaselineGraspState("queen").execute(ports)

    assert ports.vla.calls[0][1] == -5.0
