"""투하 **직전**에 아직 물고 있는지 다시 확인한다 (2026-09-07).

## 왜 CARRY 판정만으로는 부족한가

파지 성공 판정은 CARRY 로 접은 직후 한 번 한다. 그런데 그 뒤로 투하까지는
차가 바구니 앞까지 **주행**한다 — 그 사이에 흘리면 아무도 안 본다. 그날
사용자 보고가 이것이다:

    "실제로는 잡지 못했는데 잡았다고 판단하여 물체를 놓으러 갔고"

## 문턱이 틀린 게 아니었다

그 회차의 CARRY 판정값은 1181 이었다. 빈 턱은 같은 날 실측으로 1112 이니
69 raw = 약 14mm 위다 — **판정 시점에는 턱 사이에 정말 무언가 있었다.**
물체가 한쪽 턱에 걸려 있어도 턱은 그만큼 벌어진 채 멈추고, 위치만으로는
"제대로 물었다"와 "걸쳐져 있다"를 못 가른다.

걸쳐져 있던 것은 운반 중에 떨어진다. 그러니 가르는 자리는 판정 문턱이
아니라 **시간**이다 — 바구니 앞에서 한 번 더 읽으면 걸린다.

## 못 읽으면 진행한다

-1(읽기 실패)은 "비었다"가 아니라 "모른다"다. 모르는 것을 실패로 단정해
물건을 든 채 서 있는 것이 헛투하보다 나쁘다 — 다른 판정들과 같은 원칙이다.
"""

import pytest

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.fake_host_link import FakeHostLink, FakeLidar
from domain.ports.baseline_ports import Report
from domain.task import baseline_constants as bc
from domain.task.baseline_mission import (
    BaselineIdleState, BaselineInsertState, BaselinePorts)


#: plan_for_label("queen") 이 고르는 프로파일 이름이다 — 라벨과 다르다.
PROFILE_DROP = ("chess_queen", "drop")
PROFILE_IDLE = ("chess_queen", "idle")


def _ports(gripper_raw):
    # 투하 후 부하가 줄어드는 정상 팔 — 그래야 이 파일의 관문만 남는다
    # (부하 기반 놓기 판정은 test_baseline_mission.py 쪽 책임이다).
    arm = FakeArm(load_ratio=[0.0626, 0.0313])
    arm.gripper_position_raw_value = gripper_raw
    return BaselinePorts(base=FakeBase(), arm=arm, perception=None,
                         host=FakeHostLink(), lidar=FakeLidar(), estop=None,
                         use_depth_gate=False)


def _details(ports):
    return " ".join(detail for _r, _s, detail, _f in ports.host.reports)


# ── 빈 채로 왔을 때 ────────────────────────────────────────────────────────


def test_실패해도_팔은_접는다():
    """팔을 전개한 채 두는 편이 더 위험하다 — 다른 실패 경로와 같은 원칙."""
    ports = _ports(bc.GRIPPER_EMPTY_POSITION_RAW)

    BaselineInsertState("queen").execute(ports)

    assert PROFILE_IDLE in ports.arm.floor_pose_calls
    assert Report.IDLE_DONE in ports.host.reported_kinds


# ── 물고 있을 때 ───────────────────────────────────────────────────────────


def test_물고_있으면_평소대로_투하한다():
    """이 관문이 정상 경로를 막으면 안 된다."""
    ports = _ports(1189)                       # 퀸을 문 실측값

    BaselineInsertState("queen").execute(ports)

    assert PROFILE_DROP in ports.arm.floor_pose_calls
    assert "비었다" not in _details(ports)


def test_문턱_경계는_투하한다():
    """>= 다 — 경계에서 조용히 포기하면 원인을 못 찾는다."""
    ports = _ports(bc.GRIPPER_HELD_POSITION_RAW)

    BaselineInsertState("queen").execute(ports)

    assert PROFILE_DROP in ports.arm.floor_pose_calls


# ── 못 읽을 때 ─────────────────────────────────────────────────────────────


def test_위치를_못_읽으면_투하를_진행한다():
    """-1 은 '비었다'가 아니라 '모른다'다. 물건을 든 채 서 있는 것이 더 나쁘다."""
    ports = _ports(-1)

    BaselineInsertState("queen").execute(ports)

    assert PROFILE_DROP in ports.arm.floor_pose_calls
    assert "비었다" not in _details(ports)


def test_투하_직전에_턱을_다시_안_읽는다():
    """⚠️ 2026-09-08 사용자 지시로 이 파일의 계약이 통째로 사라졌다:

        "진짜 쥐었는지 확인하는 단계와 로봇과 노트북이 집은 것과 못 집은
         것에 대해 양방향 소통하는 것이 필요없는 거 같아"

    막으려던 것은 "빈 그리퍼로 투하 동작을 하는 것"인데 그건 무해하고,
    물체가 아직 바닥에 있는지는 탑뷰가 이미 안다. Pi 가 서보 위치로 추측하면
    틀릴 기회만 는다 — 2026-09-07 하루에 세 번 틀려 성공한 파지를 버렸다."""
    import inspect

    from domain.task.baseline_mission import BaselineInsertState

    code = chr(10).join(line for line in inspect.getsource(
        BaselineInsertState.execute).splitlines()
        if not line.strip().startswith("#"))

    # 투하 직전 "물었나" 판정에 쓰던 것들이 사라졌는지만 본다.
    # INSERT_FAILED 자체는 남아 있다 — 투하 자세 실패·놓기 실패처럼
    # **동작이 실패한** 경우에는 여전히 알려야 한다.
    assert "held_threshold_raw" not in code
    assert "empty_stop_raw" not in code
    assert "투하 직전 그리퍼가 비었다" not in code
