"""장난감 정리 루프 FSM 통합 테스트. docs/design/state_machine.md 가 전이 그래프의
단일 소스이고, 특히 §4(재진입 방지)가 이 파일의 핵심이다.

전부 MissionTask.run() 을 끝까지 구동해서 검증한다 — 하드웨어·ROS2 없이
domain/adapters/fake/* 만 쓴다."""

import threading

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.scripted_perception import ScriptedPerception
from domain.task.mission_task import MissionTask
from domain.task.states import PosePlanState
from domain.values import Detection, ObjectClass, Point3


def _detection(track_id, cls=ObjectClass.GABE, x=0.2):
    return Detection(
        track_id=track_id,
        cls=cls,
        pose_m=Point3(x=x, y=0.0, z=0.0),
        dims_m=Point3(x=0.05, y=0.05, z=0.05),
        yaw_rad=0.0,
        confidence=0.9,
    )


# ── 1. 정상 완주 ──────────────────────────────────────────────────────────


def test_full_mission_completes_multiple_objects(make_ports, run_to_completion):
    """물체 N(=2)개를 순회해서 전부 처리하고 DONE으로 끝난다.

    상자에 넣은 물체는 바닥에서 사라지는 게 실제 하드웨어 거동이므로,
    ScriptedPerception.script로 사이클마다 바닥이 바뀌는 걸 흉내낸다 —
    상수 목록 하나(detections=)를 계속 반환하면 det_a가 처리된 뒤에도
    스캔에 여전히 잡혀 SCAN 무변화 감지가 det_b를 처리하기 전에 먼저
    발동해 버린다 (§4 재진입 방지의 두 메커니즘이 상호작용하는 지점)."""
    det_a = _detection(track_id=1, cls=ObjectClass.GABE, x=0.2)
    det_b = _detection(track_id=2, cls=ObjectClass.CHESS_PIECE, x=0.4)
    ports = make_ports(perception=ScriptedPerception(script=[[det_a, det_b], [det_b]]))

    states = run_to_completion(ports)

    names = [s.name for s in states]
    assert names[0] == "IDLE"
    assert names[-1] == "DONE"
    assert names.count("INSERT") == 2
    assert states[-1].ctx.done_ids == {1, 2}
    assert "ESTOP" not in names


# ── 2. 무한 루프 방지 ★ ──────────────────────────────────────────────────


def test_repeated_scan_results_terminate_in_finite_steps(make_ports, run_to_completion):
    """docs/design/state_machine.md §4: 'ScriptedPerception이 같은 목록을 계속
    반환해도 유한 스텝 안에 종료되는지'는 도메인 테스트 필수 항목이다.

    가장 나쁜 경우를 만든다 — 매 스캔이 같은 대상을 계속 재검출하고(script가
    소진되면 마지막 원소를 반복), 그 대상은 절대 파지 접근에 성공하지 못한다
    (base.drive_to 항상 실패). 그래도 run_to_completion의 스텝 상한(200) 안에
    끝나야 한다 — 상한 도달은 conftest.py에서 자동으로 실패 처리된다."""
    ports = make_ports(
        base=FakeBase(arrive=False),
        perception=ScriptedPerception(detections=[_detection(track_id=1)]),
    )

    states = run_to_completion(ports)

    assert states[-1].name == "DONE"
    assert len(states) < 20, "무한 루프 방지가 걸리긴 했지만 예상보다 훨씬 오래 걸렸다"


# ── 3. SCAN 무변화 감지 ──────────────────────────────────────────────────


def test_scan_no_change_detection_ends_mission(make_ports, run_to_completion):
    """연속 2회 스캔 결과(비어있지 않은)가 동일하면 DONE. done_ids/held_ids
    필터링과는 별도의 2차 방어선이므로, 그 메커니즘 자체가 즉시(SCAN을 딱
    2번만 방문하고) 발동하는지를 정확히 짚는다."""
    ports = make_ports(
        base=FakeBase(arrive=False),
        perception=ScriptedPerception(detections=[_detection(track_id=1)]),
    )

    states = run_to_completion(ports)
    names = [s.name for s in states]

    assert names.count("SCAN") == 2, (
        "1회차 SCAN은 대상을 발견해 SELECT로, APPROACH 실패 후 되돌아온 "
        "2회차 SCAN은 동일한 스캔 결과를 보고 즉시 DONE으로 가야 한다"
    )
    assert names[-1] == "DONE"


# ── 4. done_ids 재선택 방지 ──────────────────────────────────────────────


def test_done_object_is_never_reselected(make_ports, run_to_completion):
    """INSERT에 성공한 물체는 done_ids에 등록되고, 그 뒤로는 같은 스캔 결과에
    계속 나타나도 다시 APPROACH되지 않는다."""
    ports = make_ports(perception=ScriptedPerception(detections=[_detection(track_id=1)]))

    states = run_to_completion(ports)
    names = [s.name for s in states]

    assert names.count("INSERT") == 1
    assert names.count("APPROACH") == 1, "완료된 물체가 다시 선택되면 APPROACH가 2번 이상 나온다"
    assert states[-1].ctx.done_ids == {1}


# ── 5. held_ids 재선택 방지 ──────────────────────────────────────────────


def test_held_object_is_never_reselected(make_ports, run_to_completion):
    """GRASP에 실패해 보류된 물체는 held_ids에 등록되고, 그 뒤로는 같은 스캔
    결과에 계속 나타나도 다시 APPROACH되지 않는다."""
    ports = make_ports(
        arm=FakeArm(load_ratio=0.0),  # 항상 파지 실패
        perception=ScriptedPerception(detections=[_detection(track_id=1)]),
    )

    states = run_to_completion(ports)
    names = [s.name for s in states]

    assert names.count("APPROACH") == 1, "보류된 물체가 다시 선택되면 APPROACH가 2번 이상 나온다"
    assert states[-1].ctx.held_ids == {1}
    assert states[-1].name == "DONE"


# ── 6. GRASP 재시도 소진 ─────────────────────────────────────────────────


def test_grasp_retry_exhaustion_holds_and_returns_to_scan(make_ports, run_to_completion):
    """부하 미달이 MAX_GRASP_RETRY(3)회 반복되면 재시도를 그만두고 SCAN으로
    복귀 + 보류 등록한다 — 미션은 끝나지 않고 계속된다."""
    ports = make_ports(
        arm=FakeArm(load_ratio=0.0),
        perception=ScriptedPerception(detections=[_detection(track_id=7)]),
    )

    states = run_to_completion(ports)
    names = [s.name for s in states]

    assert names.count("GRASP") == 4  # 최초 시도 + 재시도 3회 (grasp_attempts 0,1,2,3)
    assert 7 in states[-1].ctx.held_ids
    assert names[-1] == "DONE"


# ── 7. POSE_PLAN 해 없음 → REJECT ───────────────────────────────────────


def test_pose_plan_no_solution_rejects_and_holds(make_ports, run_to_completion, monkeypatch):
    """POSE_PLAN은 현재 ⏸ 보류 상태라 _solve_phi()가 항상 φ=0(해 있음)을
    반환한다 — REJECT 분기는 구조만 있고 아직 실제로 도달하지 않는다
    (docs/design/state_machine.md §2). 그 구조가 살아있는지 확인하려면
    해가 없는 경우를 직접 주입해야 한다."""
    monkeypatch.setattr(PosePlanState, "_solve_phi", lambda self, dims_m, opening_mm: None)

    ports = make_ports(perception=ScriptedPerception(detections=[_detection(track_id=3)]))

    states = run_to_completion(ports)
    names = [s.name for s in states]

    assert "REJECT" in names
    assert "INSERT" not in names
    assert 3 in states[-1].ctx.held_ids
    assert names[-1] == "DONE"


# ── 8. E-STOP ────────────────────────────────────────────────────────────


def test_estop_interrupts_mission_immediately(make_ports):
    """미션 도중 E-STOP이 걸리면 다음 execute() 전에 EstopState로 갈아치워진다
    — 정상 전이가 아니라 인터럽트다 (state_machine.md §2)."""
    estop = threading.Event()
    ports = make_ports(estop=estop, perception=ScriptedPerception(detections=[_detection(1)]))

    gen = MissionTask(ports).run("장난감 정리해줘")
    first = next(gen)
    assert first.name == "IDLE"
    second = next(gen)
    assert second.name == "SCAN"

    estop.set()
    remaining = [s.name for s in gen]

    assert remaining[0] == "ESTOP"
    assert remaining[-1] == "ESTOP"  # ESTOP 이후로는 더 진행 안 됨


def test_estop_set_before_start_interrupts_immediately(make_ports, run_to_completion):
    estop = threading.Event()
    estop.set()
    ports = make_ports(estop=estop)

    states = run_to_completion(ports)

    assert [s.name for s in states] == ["ESTOP"]


# ── 9. FETCH 모드 ────────────────────────────────────────────────────────


def test_fetch_mode_routes_through_deliver_and_handover(make_ports, run_to_completion):
    """FETCH는 GRASP까지 TIDY와 완전히 동일한 코드를 타고, 그 다음부터
    DELIVER → HANDOVER로 갈라진다 — TRANSPORT/POSE_PLAN/INSERT는 아예
    거치지 않는다 (docs/design/sequences.md §4)."""
    target = _detection(track_id=5, cls=ObjectClass.GABE)
    ports = make_ports(
        # get_load()는 GRASP(1회차, 높아야 성공)과 HANDOVER(2회차, 낮아야
        # '사람이 받아감')가 반대 의미로 같이 쓴다 — 순서대로 반환.
        arm=FakeArm(load_ratio=[1.0, 0.0]),
        perception=ScriptedPerception(detections=[target]),
    )

    states = run_to_completion(ports, raw_text="가베 가져와")
    names = [s.name for s in states]

    assert "GRASP" in names
    assert "DELIVER" in names
    assert "HANDOVER" in names
    assert "TRANSPORT" not in names
    assert "POSE_PLAN" not in names
    assert "INSERT" not in names
    assert names[-1] == "DONE"
    assert states[-1].ctx.done_ids == {5}


def test_fetch_mode_select_ignores_non_target_class(make_ports, run_to_completion):
    """FETCH는 SELECT에서 spec.target_cls와 일치하는 것만 고른다
    (state_machine.md §3 SELECT 3번 조건) — GABE만 있으면 CHESS_PIECE를
    요청해도 고를 게 없어 곧바로 DONE이다."""
    ports = make_ports(
        arm=FakeArm(load_ratio=[1.0, 0.0]),
        perception=ScriptedPerception(detections=[_detection(track_id=9, cls=ObjectClass.GABE)]),
    )

    states = run_to_completion(ports, raw_text="체스말 가져와")
    names = [s.name for s in states]

    assert "APPROACH" not in names
    assert names[-1] == "DONE"
    assert states[-1].ctx.done_ids == frozenset()
