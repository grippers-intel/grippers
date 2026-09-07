"""파지 성공을 부하가 아니라 그리퍼 위치로 판정한다 (2026-09-06).

## 부하가 왜 못 쓰는가 — 실측

    퀸을 실제로 물었을 때   0.0391 = 10/256
    빈손 닫힘               0.0352 =  9/256
    빈 턱 기계정지          0.0274 =  7/256
    LOAD_THRESHOLD          0.0469 = 12/256

**물었을 때가 문턱보다 낮다.** 부하가 1/256 단위로 양자화되는데 물체 유무의
차이가 딱 그 한 단위라, 판정이 사실상 동전 던지기다.

같은 날 실기에서 한 판은 0.0000 으로 실패, 다음 판은 0.0469 로 성공이
나왔다 — **둘 다 틀린 판정**이었다. 후자를 사용자가 이렇게 보고했다:
"파지에 실패했는데 성공했다고 하고 그냥 못 집었는데 물체 놓으러 가고 있거든".

## 위치는 왜 되는가

턱이 물체에 막히면 닫힘 목표까지 못 가고, 그 잔차가 곧 물체 두께다.

    빈 턱 기계정지    1112        (2026-09-07 실측)
    닫힘 명령 목표    1150
    퀸(17mm)         1189 근처

77 raw 차이다. 부하의 1/256 과 달리 헷갈릴 수가 없다.

⚠️ 빈 턱 값은 2026-09-07 에 1147 -> 1112 로 내려갔다. servo 6 의
Min_Angle_Limit 을 1140 -> 1090 으로 내렸기 때문이다 — 마모가 아니라 설정이다.

⚠️ 이 문턱이 답하는 것은 "턱 사이에 무엇이 있는가"뿐이다. **제대로
물었는가는 못 답한다** — 한쪽 턱에 걸린 물체도 턱을 그만큼 벌린다. 같은
날 1181 을 성공으로 읽고 헛투하를 하러 간 것이 그 경우다. 그 뒤를 받는
것이 투하 직전 재확인이다(test_insert_regrip_check.py).
"""

import pytest

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.fake_host_link import FakeHostLink
from domain.task import baseline_constants as bc
from domain.ports.baseline_ports import Report
from domain.task.baseline_mission import BaselinePorts

#: 실측값. 이 시험의 근거이자, 문턱이 이 둘 사이에 있어야 한다는 제약이다.
EMPTY_JAW_RAW = bc.GRIPPER_EMPTY_POSITION_RAW   # 1112, 2026-09-07 실측
QUEEN_HELD_RAW = 1189


def _ports(position_raw, load):
    arm = FakeArm()
    arm.gripper_position_raw_value = position_raw
    arm.load = load
    return BaselinePorts(base=FakeBase(), arm=arm, perception=None,
                         host=FakeHostLink(), lidar=None, estop=None,
                         use_depth_gate=False)


# ── 문턱 자체 ──────────────────────────────────────────────────────────────


def test_문턱이_빈턱과_물린것_사이에_있다():
    """이 시험이 깨지면 판정 자체가 무의미해진다."""
    assert EMPTY_JAW_RAW < bc.GRIPPER_HELD_POSITION_RAW < QUEEN_HELD_RAW


def test_문턱이_서보_데드밴드보다_충분히_떨어져_있다():
    """위치 데드밴드가 약 5 raw 로 실측됐다(2026-09-06). 그 서너 배는 떨어져야
    잡음으로 판정이 뒤집히지 않는다."""
    assert bc.GRIPPER_HELD_POSITION_RAW - EMPTY_JAW_RAW >= 15


def test_부하_문턱으로는_퀸을_못_잡아낸다():
    """왜 부하를 버렸는지를 숫자로 남긴다 — 실측 0.0391 이 문턱 아래다."""
    queen_load = 10.0 / 256.0
    assert queen_load < bc.LOAD_THRESHOLD, (
        "부하 문턱이 실측 파지값보다 높다 — 이 조건이 깨지면 부하 판정을 "
        "다시 검토할 수 있다")


# ── 판정 ───────────────────────────────────────────────────────────────────


def test_빈_턱이면_실패다():
    """⚠️ 이것이 2026-09-06 사고다 — 못 집었는데 놓으러 갔다."""
    arm = _ports(EMPTY_JAW_RAW, load=bc.LOAD_THRESHOLD).arm
    assert arm.gripper_position_raw() < bc.GRIPPER_HELD_POSITION_RAW


def test_물었으면_성공이다():
    arm = _ports(QUEEN_HELD_RAW, load=0.0).arm
    assert arm.gripper_position_raw() >= bc.GRIPPER_HELD_POSITION_RAW


def test_부하가_높아도_턱이_비었으면_실패다():
    """부하는 이제 판정에 안 쓴다 — 그게 이 변경의 전부다.

    실기에서 빈 턱이 12/256(문턱과 같은 값)을 낸 적이 있다."""
    ports = _ports(EMPTY_JAW_RAW, load=bc.LOAD_THRESHOLD + 0.01)
    assert ports.arm.gripper_position_raw() < bc.GRIPPER_HELD_POSITION_RAW


def test_부하가_0이어도_물었으면_성공이다():
    """반대 방향 — 실기에서 물었는데 0.0000 이 나온 적이 있다."""
    ports = _ports(QUEEN_HELD_RAW, load=0.0)
    assert ports.arm.gripper_position_raw() >= bc.GRIPPER_HELD_POSITION_RAW


def test_위치를_못_읽으면_음수로_알린다():
    """조용히 0 이나 기본값을 주면 '빈 턱'으로 오판한다 — 모르는 것과
    비어 있는 것은 다르다."""
    arm = FakeArm()
    arm.gripper_position_raw_value = -1
    assert arm.gripper_position_raw() < 0


# ── 얇은 물체 한계 ─────────────────────────────────────────────────────────


def test_문턱이_뜻하는_최소_물체_두께를_적어_둔다():
    """이 문턱을 넘으려면 물체가 얼마나 두꺼워야 하는가.

    지금 기물(퀸 17mm, 나이트 22mm)은 넉넉히 넘지만, 더 얇은 것을 넣으려면
    여기가 한계가 된다."""
    raw_per_mm = (1578 - 1150) / (96.0 - 9.0)          # 보정표 첫 구간
    min_mm = 9.0 + (bc.GRIPPER_HELD_POSITION_RAW - 1150) / raw_per_mm
    assert min_mm < 17.0, f"퀸(17mm)도 못 넘는다 — 문턱 {min_mm:.1f}mm"
    assert min_mm > 9.0


# ── 문턱은 닫기 명령에 따라 달라진다 (2026-09-07) ─────────────────────────
#
# 절대 문턱 하나로는 안 된다는 것이 뒤늦게 드러났다. box·star 프로파일은
# 20mm 까지만 닫아서, **빈 턱이어도** 거기서 멈춘다 — 옛 절대 문턱 1165 로는
# 그 1216 이 "물었음"으로 읽혔다. 팀원이 지목한 두 물체가 정확히 이것이다.


def test_0mm로_닫으면_옛_절대_문턱과_사실상_같다():
    """기존에 맞던 경우를 그대로 재현해야 한다 — 안 그러면 queen 판정이
    조용히 바뀐다."""
    assert abs(bc.held_threshold_raw(0.0) - bc.GRIPPER_HELD_POSITION_RAW) <= 5


def test_20mm로_닫는_프로파일은_문턱이_올라간다():
    """⚠️ 이것이 2026-09-07 에 찾은 구멍이다."""
    empty_at_20 = bc.empty_stop_raw(20.0)
    assert empty_at_20 > bc.GRIPPER_HELD_POSITION_RAW, (
        "20mm 빈 턱이 옛 절대 문턱보다 낮으면 이 시험의 전제가 사라진다")
    assert bc.held_threshold_raw(20.0) > empty_at_20


def test_box와_star가_실제로_20mm로_닫는다():
    """이 값이 바뀌면 위 두 시험의 근거가 사라진다 — 프로파일과 같이 본다."""
    from domain.task.baseline_mission import plan_for_label
    for label in ("box", "star"):
        assert plan_for_label(label).close_width_mm == 20.0


def test_빈_턱은_어떤_폭에서도_문턱을_못_넘는다():
    """문턱의 정의 자체 — 명령한 자리까지 갔으면 아무것도 안 물었다는 뜻이다."""
    for w in (0.0, 5.0, 20.0, 40.0, 96.0):
        assert bc.empty_stop_raw(w) < bc.held_threshold_raw(w)


# ── 읽기 전에 확실히 닫는다 (2026-09-07 실기) ─────────────────────────────
#
# 퀸 파지가 실패했는데 그리퍼 1190 을 읽고 성공으로 판정해 물체를 놓으러 갔다.
# 1190 은 퀸(17mm)을 제대로 문 값과 같아서 위치만으로는 못 가렸다.
#
# 원인은 문턱이 아니라 **전제**였다. held_threshold_raw 는 "그 폭으로 닫으라고
# 명령했을 때"를 기준으로 삼는데, VLA 경로는 정책이 그리퍼를 직접 몬다 —
# 턱이 1190 에서 멈춘 것이 "물체가 막아서"인지 "정책이 거기까지만 닫으라고
# 해서"인지 알 수가 없었다.


class _OkVla:
    def run_grasp(self, label, pan_bias_deg=0.0):
        return True


def _run_vla_grasp(position_raw, blocked_raw):
    """정책이 그 위치에 턱을 두고 끝낸 상황을 만든다."""
    from domain.adapters.fake.scripted_perception import ScriptedPerception
    from domain.task.baseline_mission import BaselineGraspState

    arm = FakeArm(load_ratio=0.05)
    arm.gripper_position_raw_value = position_raw
    arm.jaw_blocked_raw = blocked_raw
    ports = BaselinePorts(base=FakeBase(), arm=arm,
                          perception=ScriptedPerception(), host=FakeHostLink(),
                          lidar=None, estop=None, use_depth_gate=False, vla=_OkVla(), vla_only=True)
    BaselineGraspState("queen").execute(ports)
    return arm, ports


def test_판정_전에_닫기를_명령한다():
    """⚠️ 이 명령이 없으면 문턱의 전제가 깨진 채 비교하게 된다."""
    arm, _ = _run_vla_grasp(1190, 1190)
    assert 0.0 in arm.gripper_widths, "닫기(0mm)를 명령해야 한다"


def test_정책이_덜_닫아_놓은_빈_턱은_닫으면_드러난다():
    """⚠️ 2026-09-07 실기 그대로 — 1190 을 읽었지만 실제로는 빈 손이었다.

    확실히 닫으라고 하면 막는 것이 없으니 빈 턱까지 내려간다."""
    arm, ports = _run_vla_grasp(1190, None)

    assert arm.gripper_position_raw() < bc.held_threshold_raw(0.0)
    assert Report.GRASP_FAILED in ports.host.reported_kinds


def test_진짜_물었으면_닫아도_그대로다():
    """물체가 턱을 막으므로 더 안 닫힌다 — 정상 경로가 안 깨져야 한다."""
    arm, ports = _run_vla_grasp(1190, 1190)

    assert arm.gripper_position_raw() == 1190
    assert Report.GRASP_DONE in ports.host.reported_kinds
