"""baseline 미션 FSM 테스트 — 하드웨어·ROS2·네트워크 없이 Fake만으로 돈다.

사용자가 2026-08-25에 정리한 7단계 흐름을 그대로 검증한다. 기존
`domain/task/states.py`의 루프 FSM과는 별개 경로다.

⚠️ 실기 미검증. 여기서 검증하는 것은 전이 그래프와 "모르면 실패" 기본값뿐이고,
실측이 안 된 수치가 필요한 자리는 판정을 포기하는지를 확인한다."""

import threading

import pytest

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_host_link import FakeBaselineBase, FakeHostLink, FakeLidar
from domain.adapters.fake.scripted_perception import ScriptedPerception
from domain.ports.baseline_ports import BasketFace, HostPlan, Status
from domain.task import baseline_constants as bc
from domain.task import baseline_mission as bm
from domain.values import Pose2D

MAX_STEPS = 60


def _ports(host=None, lidar=None, arm=None, perception=None, base=None):
    return bm.BaselinePorts(
        base=base or FakeBaselineBase(),
        arm=arm or FakeArm(),
        perception=perception or ScriptedPerception(),
        host=host or FakeHostLink(),
        lidar=lidar or FakeLidar(),
        estop=threading.Event(),
    )


def _plan(**kw):
    kw.setdefault("target_label", "knight")
    kw.setdefault("destination", "chess")
    kw.setdefault("waypoints", (Pose2D(x=1.0, y=0.5, theta=0.0),))
    return HostPlan(**kw)


def _clear_perception(**kw):
    """정면이 깨끗하다고 보고하는 perception."""
    kw.setdefault("contact_risk", False)
    return ScriptedPerception(**kw)


# ── 1~3. Host 지시 -> IDLE에서 곧장 APPROACH ────────────────────────────────

def test_지시가_없으면_IDLE에_머문다():
    ports = _ports(host=FakeHostLink([None]))
    assert bm.BaselineIdleState().execute(ports).name == "IDLE"


def test_지시가_오면_SCAN_SELECT_없이_APPROACH로_간다():
    """목표는 Host가 이미 골랐다 — Pi가 다시 찾지 않는다."""
    host = FakeHostLink([_plan()])
    nxt = bm.BaselineIdleState().execute(_ports(host=host))
    assert nxt.name == "APPROACH"
    assert Status.APPROACHING in host.reported_statuses


def test_모르는_라벨은_시작하지_않는다():
    host = FakeHostLink([_plan(target_label="바나나")])
    nxt = bm.BaselineIdleState().execute(_ports(host=host))
    assert nxt.name == "DONE"


# ── 3~4. 경로 추종과 미세 회피 ──────────────────────────────────────────────

def test_정면이_깨끗하면_경로를_따라간다():
    base = FakeBaselineBase()
    ports = _ports(host=FakeHostLink([_plan()]), base=base,
                   perception=_clear_perception())
    nxt = bm.BaselineApproachState(_plan()).execute(ports)
    assert nxt.name == "APPROACH"
    assert base.drive_calls


def test_정면이_위험하면_멈추고_옆으로_비킨_뒤_보고한다(monkeypatch):
    monkeypatch.setattr(bc, "AVOID_LATERAL_STEP_M", 0.05)
    base = FakeBaselineBase()
    host = FakeHostLink([_plan()])
    ports = _ports(host=host, base=base,
                   perception=ScriptedPerception(contact_risk=True))

    nxt = bm.BaselineApproachState(_plan()).execute(ports)

    assert base.stop_calls == 1
    assert base.creep_lateral_calls == [0.05]
    assert Status.AVOIDING in host.reported_statuses
    assert nxt.avoided == 1
    assert not base.drive_calls, "위험한데 경로를 계속 따라가면 안 된다"


def test_회피폭이_미실측이면_비키지_않고_정지만_한다():
    """지어낸 거리로 옆걸음하느니 멈추고 Host에 맡긴다."""
    assert bc.AVOID_LATERAL_STEP_M is None
    base = FakeBaselineBase()
    host = FakeHostLink([_plan()])
    ports = _ports(host=host, base=base,
                   perception=ScriptedPerception(contact_risk=True))

    nxt = bm.BaselineApproachState(_plan()).execute(ports)

    assert base.creep_lateral_calls == []
    assert nxt.name == "IDLE"
    assert Status.AVOIDING in host.reported_statuses


def test_회피_예산을_다_쓰면_전면_재계획을_요청한다(monkeypatch):
    monkeypatch.setattr(bc, "AVOID_LATERAL_STEP_M", 0.05)
    host = FakeHostLink([_plan()])
    ports = _ports(host=host, perception=ScriptedPerception(contact_risk=True))
    state = bm.BaselineApproachState(_plan(), avoided=bc.MAX_AVOID_STEPS)

    assert state.execute(ports).name == "IDLE"
    assert "예산 소진" in host.reports[-1][1]


# ── 5. GRASP 전환은 Host가 정한다 ───────────────────────────────────────────

def test_GRASP_전환은_Host의_grasp_ready로만_일어난다():
    host = FakeHostLink([_plan(grasp_ready=True)])
    ports = _ports(host=host, perception=_clear_perception())
    assert bm.BaselineApproachState(_plan()).execute(ports).name == "GRASP"


def test_Pi는_스스로_19cm를_판정하지_않는다():
    """grasp_ready가 False면 아무리 가까워도 APPROACH를 유지한다."""
    ports = _ports(host=FakeHostLink([_plan(grasp_ready=False)]),
                   perception=_clear_perception())
    assert bm.BaselineApproachState(_plan()).execute(ports).name == "APPROACH"


# ── 6. GRASP ────────────────────────────────────────────────────────────────

def test_파지_성공하면_미세_전진을_거쳐_바구니_접근으로_넘어간다():
    base = FakeBaselineBase()
    host = FakeHostLink([_plan(grasp_ready=True)])
    ports = _ports(host=host, base=base, arm=FakeArm(load_ratio=[0.07]))

    nxt = bm.BaselineGraspState(_plan()).execute(ports)

    assert nxt.name == "APPROACH_BOX"
    assert base.creep_forward_calls == [bc.GRASP_CREEP_FORWARD_MM / 1000.0]
    assert Status.GRASP_DONE in host.reported_statuses


def test_파지_자세_순서가_교시_경로를_따른다():
    arm = FakeArm(load_ratio=[0.07])
    ports = _ports(arm=arm, host=FakeHostLink([_plan(grasp_ready=True)]))

    bm.BaselineGraspState(_plan()).execute(ports)

    stages = [stage for _profile, stage in arm.floor_pose_calls]
    assert stages == ["safe", "grasp", "midpoint", "safe", "idle"]


def test_CARRY_IDLE_빈손이면_물체가_보이는지로_두_갈래를_가른다():
    for confirmed, expected in ((False, Status.GRASP_FAILED_RETRY),
                                (True, Status.GRASP_FAILED_RETARGET)):
        host = FakeHostLink([_plan(grasp_ready=True)])
        ports = _ports(host=host, arm=FakeArm(load_ratio=[0.07, 0.07, 0.03]),
                       perception=ScriptedPerception(grasp_confirmed=confirmed))

        nxt = bm.BaselineGraspState(_plan()).execute(ports)

        assert nxt.name == "IDLE"
        assert expected in host.reported_statuses


def test_파지_실패하면_바구니로_출발하지_않는다():
    arm = FakeArm(load_ratio=[0.02])
    ports = _ports(arm=arm, host=FakeHostLink([_plan(grasp_ready=True)]))

    nxt = bm.BaselineGraspState(_plan()).execute(ports)

    assert nxt.name == "IDLE"
    assert "drop" not in [stage for _p, stage in arm.floor_pose_calls]


def test_얇은_체스말도_파지_하한_아래로_조이지_않는다():
    arm = FakeArm(load_ratio=[0.07])
    ports = _ports(arm=arm, host=FakeHostLink([_plan(grasp_ready=True)]))
    bm.BaselineGraspState(_plan(target_label="queen")).execute(ports)
    assert min(arm.gripper_widths) >= bm.GRASP_MIN_MM


def test_라벨로_프로필을_고른다_폭_휴리스틱을_쓰지_않는다():
    """Host가 라벨을 주므로 star/soccer도 갈린다 — 폭으로는 못 가르던 쌍이다."""
    assert bm.plan_for_label("star").profile == "soccer_polyhedron"
    assert bm.plan_for_label("box").profile == "cube"
    assert bm.plan_for_label("queen").profile == "chess_queen"
    assert bm.plan_for_label("없는라벨") is None


# ── 7. 바구니 접근과 INSERT ─────────────────────────────────────────────────

def test_라이다_오프셋이_미실측이면_정지_판정을_포기한다():
    """지어낸 거리에서 팔을 전개하느니 멈추고 보고한다."""
    assert bc.LIDAR_TO_CHASSIS_FRONT_M is None
    host = FakeHostLink([_plan()])
    base = FakeBaselineBase()
    nxt = bm.BaselineApproachBoxState(_plan()).execute(_ports(host=host, base=base))
    assert nxt.name == "IDLE"
    assert base.stop_calls == 1
    assert "미실측" in host.reports[-1][1]


def test_라이다가_정지거리를_보면_INSERT로_넘어간다(monkeypatch):
    monkeypatch.setattr(bc, "LIDAR_TO_CHASSIS_FRONT_M", 0.15)
    stop_at = bm.basket_stop_distance_m()
    lidar = FakeLidar([BasketFace(True, stop_at - 0.01, 0.0)])
    base = FakeBaselineBase()
    nxt = bm.BaselineApproachBoxState(_plan()).execute(
        _ports(lidar=lidar, base=base, host=FakeHostLink([_plan()])))
    assert nxt.name == "INSERT"
    assert base.stop_calls == 1


def test_라이다_판정이_실패하면_INSERT로_안_넘어간다(monkeypatch):
    monkeypatch.setattr(bc, "LIDAR_TO_CHASSIS_FRONT_M", 0.15)
    lidar = FakeLidar([BasketFace(False, 0.01, 0.0, "점 부족")])
    nxt = bm.BaselineApproachBoxState(_plan()).execute(
        _ports(lidar=lidar, host=FakeHostLink([_plan()])))
    assert nxt.name == "APPROACH_BOX"


def test_아직_멀면_경로를_계속_따라간다(monkeypatch):
    monkeypatch.setattr(bc, "LIDAR_TO_CHASSIS_FRONT_M", 0.15)
    base = FakeBaselineBase()
    lidar = FakeLidar([BasketFace(True, 1.0, 0.0)])
    bm.BaselineApproachBoxState(_plan()).execute(
        _ports(lidar=lidar, base=base, host=FakeHostLink([_plan()])))
    assert base.drive_calls


def test_INSERT는_열고_닫은_뒤_접는다():
    """접기 전에 닫는다 — 닫힌 그리퍼가 접기에 알맞은 형상이다."""
    arm = FakeArm(load_ratio=[0.07])
    host = FakeHostLink([_plan()])
    nxt = bm.BaselineInsertState(_plan()).execute(_ports(arm=arm, host=host))

    assert nxt.name == "DONE"
    assert arm.gripper_widths[-2:] == [
        bm.plan_for_label("knight").release_width_mm, bm.CLOSED_MM]
    assert arm.floor_pose_calls[-1][1] == "idle"
    assert Status.MISSION_DONE in host.reported_statuses


def test_INSERT는_활짝_열지_않는다():
    arm = FakeArm(load_ratio=[0.07])
    bm.BaselineInsertState(_plan()).execute(_ports(arm=arm))
    assert max(arm.gripper_widths) < bm.GRIPPER_MAX_SAFE_OPEN_MM


# ── 전체 흐름 ──────────────────────────────────────────────────────────────

def test_한_바퀴가_DONE으로_끝난다(monkeypatch):
    monkeypatch.setattr(bc, "LIDAR_TO_CHASSIS_FRONT_M", 0.15)
    stop_at = bm.basket_stop_distance_m()
    host = FakeHostLink([
        _plan(),
        _plan(grasp_ready=True),
        _plan(),
    ])
    ports = _ports(host=host, arm=FakeArm(load_ratio=[0.07]),
                   perception=_clear_perception(),
                   lidar=FakeLidar([BasketFace(True, stop_at - 0.01, 0.0)]))

    names = []
    for i, state in enumerate(bm.BaselineMission(ports).run()):
        names.append(state.name)
        if i > MAX_STEPS:
            pytest.fail(f"상한 안에 못 끝났다: {names}")

    assert names[0] == "IDLE"
    assert names[-1] == "DONE"
    assert "GRASP" in names and "INSERT" in names
    assert Status.MISSION_DONE in host.reported_statuses


def test_ESTOP은_전이_그래프가_아니라_인터럽트다():
    ports = _ports(host=FakeHostLink([_plan()]))
    ports.estop.set()
    states = list(bm.BaselineMission(ports).run())
    assert [s.name for s in states] == ["ESTOP"]


# ── 실측 TODO가 남아 있다는 사실 자체를 못 박는다 ──────────────────────────

def test_미실측_상수_목록이_비어_있지_않다():
    """실기 투입 전 이 목록이 비어야 한다. 비면 이 테스트를 지운다."""
    assert set(bc.unresolved()) == {
        "MARKER_TO_CHASSIS_FRONT_M",
        "LIDAR_TO_CHASSIS_FRONT_M",
        "LIDAR_MIN_RANGE_M",
        "BASKET_RIM_HEIGHT_M",
        "AVOID_LATERAL_STEP_M",
    }


def test_턱이_닫히는_지점이_실측_대상으로_남아_있다():
    """19cm 정렬 -> 10cm 전진이면 턱은 차체 전면 90mm 앞에서 닫힌다.

    이건 결함이 아니라 설계다 — 팔이 열린 채 내려온 뒤 차체가 전진해서
    물체를 턱 사이로 밀어 넣는다(사용자 설명 2026-08-26). 다만 전진 거리가
    매우 예민해 여러 번 실측해야 하고 50mm로 바뀔 수 있다. 값이 바뀌면 이
    테스트가 실패하니 BASELINE_MISSION_TODO.md도 같이 갱신하게 된다."""
    assert bc.jaw_close_forward_mm() == 90.0
    assert bc.GRASP_CREEP_FORWARD_MM in (100.0, 50.0), "실측으로 확정되면 후보를 좁혀라"
