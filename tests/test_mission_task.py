"""장난감 정리 루프 FSM 통합 테스트. docs/design/state_machine.md 가 전이 그래프의
단일 소스이고, 특히 §4(재진입 방지)가 이 파일의 핵심이다.

전부 MissionTask.run() 을 끝까지 구동해서 검증한다 — 하드웨어·ROS2 없이
domain/adapters/fake/* 만 쓴다."""

import threading
from collections import Counter

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.scripted_interpreter import ScriptedInterpreter
from domain.adapters.fake.scripted_perception import ScriptedPerception
from domain.task import states as states_module
from domain.task.mission_task import MissionTask
from domain.task.states import SCAN_NO_CHANGE_LIMIT, PosePlanState
from domain.values import (
    BoxColor,
    Detection,
    MissionContext,
    MissionMode,
    MissionSpec,
    ObjectClass,
    Point3,
)


def _detection(track_id, cls=ObjectClass.GABE, x=0.2, confidence=0.9):
    return Detection(
        track_id=track_id,
        cls=cls,
        pose_m=Point3(x=x, y=0.0, z=0.0),
        dims_m=Point3(x=0.05, y=0.05, z=0.05),
        yaw_rad=0.0,
        confidence=confidence,
    )


def _attempts_by_target(states, state_name):
    """상태 시퀀스에서 **대상별 시도 횟수**를 센다. state_machine.md §4:
    "'끝났다'만 보면 부족하다 — 몇 개가 실제로 시도됐는지까지 세야 한다."
    조기 종료 결함은 미션을 정상 종료시키므로 종료 여부만 보는 검증은 전부 통과한다."""
    return Counter(s.target.track_id for s in states if s.name == state_name)


# ── 1. 정상 완주 ──────────────────────────────────────────────────────────


def test_full_mission_completes_multiple_objects(make_ports, run_to_completion):
    """물체 N(=2)개를 순회해서 전부 처리하고 DONE으로 끝난다.

    상자에 넣은 물체는 바닥에서 사라지는 게 실제 하드웨어 거동이므로,
    ScriptedPerception.script로 사이클마다 바닥이 바뀌는 걸 흉내낸다.

    정적 장면(detections=)에서도 통과해야 한다 — 그건 아래 §3의
    test_static_scene_first_grasp_failure_does_not_block_the_rest 가 맡는다.
    이슈 #131 이전에는 정적 장면이면 SCAN 무변화 감지가 det_b를 처리하기 전에
    먼저 발동해 버려서, 이 테스트가 script= 를 쓰는 것 말고는 방법이 없었다."""
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


# ── 3. SCAN 무변화 감지 (이슈 #131) ★ ───────────────────────────────────


def test_static_scene_first_grasp_failure_does_not_block_the_rest(make_ports, run_to_completion):
    """이슈 #131 완료 조건 1 — 물체 3개 중 첫 물체 파지가 실패해도 나머지 2개가
    각각 선택·시도된다. **정적 장면(detections=) 그대로** 검증한다.

    script= 로 사이클마다 바닥을 바꿔 주면 SCAN 무변화 감지를 우회하게 되어
    버그가 남은 채로도 초록불이 난다. 보류된 물체는 실기에서도 바닥에 그대로
    남아 있으므로 scan_floor() 결과가 사이클마다 동일한 게 정상이고, 진전은
    '관측 목록이 줄었는가'가 아니라 'SELECT 후보가 줄었는가'로 봐야 한다.

    수정 전 거동: IDLE SCAN SELECT APPROACH GRASP×4 SCAN DONE — 사이클 2의
    scan_floor() 결과가 사이클 1과 같아 물체 2·3이 한 번도 선택되지 않았다."""
    detections = [
        _detection(track_id=1, x=0.2),
        _detection(track_id=2, x=0.4),
        _detection(track_id=3, x=0.6),
    ]
    ports = make_ports(
        # 1번 물체의 GRASP만 4번(최초 + 재시도 3회) 전부 실패하고, 그 뒤로는 성공.
        arm=FakeArm(load_ratio=[0.0, 0.0, 0.0, 0.0, 1.0]),
        perception=ScriptedPerception(detections=detections),
    )

    states = run_to_completion(ports)
    names = [s.name for s in states]

    assert _attempts_by_target(states, "APPROACH") == {1: 1, 2: 1, 3: 1}, (
        "물체 3개가 각각 한 번씩 선정·접근돼야 한다 — 1번이 보류된 뒤 후보 집합이 "
        "줄어드는 것이 '진전'이다"
    )
    assert _attempts_by_target(states, "GRASP") == {1: 4, 2: 1, 3: 1}
    assert names.count("INSERT") == 2
    assert states[-1].ctx.held_ids == {1}
    assert states[-1].ctx.done_ids == {2, 3}
    assert names[-1] == "DONE"


def test_static_scene_all_grasps_failing_still_tries_every_object(make_ports, run_to_completion):
    """이슈 #131 재현 시나리오 그대로 — 물체 3개, 파지는 항상 실패. 수정 전에는
    대상별 GRASP 시도가 {1: 4} 였다(2·3번은 선택조차 되지 않음).

    대상별 시도 '횟수'는 검증하지 않는다 — 재시도 예산이 대상별이 아니라 미션
    누적이라 2·3번은 1회씩만 시도되는데, 그건 별개 원인의 이슈 #132 몫이다.
    여기서 고정할 계약은 '세 물체가 모두 시도된다'까지다."""
    detections = [_detection(track_id=i, x=0.2 * i) for i in (1, 2, 3)]
    ports = make_ports(
        arm=FakeArm(load_ratio=0.0),
        perception=ScriptedPerception(detections=detections),
    )

    states = run_to_completion(ports)
    grasps = _attempts_by_target(states, "GRASP")

    assert set(grasps) == {1, 2, 3}, f"시도되지 않은 물체가 있다 — 대상별 GRASP: {dict(grasps)}"
    assert all(count >= 1 for count in grasps.values())
    assert states[-1].ctx.held_ids == {1, 2, 3}
    assert states[-1].name == "DONE"


def test_same_candidate_set_for_k_cycles_ends_mission(make_ports, run_to_completion, monkeypatch):
    """이슈 #131 완료 조건 3 — 후보 집합이 계속 동일한 진짜 무한 루프는 여전히
    유한 스텝 안에 DONE.

    무변화 감지는 done_ids/held_ids 필터링(1차 방어선)이 깨졌을 때의 2차 방어선이므로,
    그 상황을 직접 만든다 — hold()를 무력화하면 APPROACH 실패가 후보를 줄이지 못해
    SCAN → SELECT → APPROACH → SCAN 이 영원히 돌 수 있다."""
    monkeypatch.setattr(MissionContext, "hold", lambda self, track_id: self)

    ports = make_ports(
        base=FakeBase(arrive=False),
        perception=ScriptedPerception(detections=[_detection(1), _detection(2, x=0.4)]),
    )

    states = run_to_completion(ports)
    names = [s.name for s in states]

    assert (
        names.count("SCAN") == SCAN_NO_CHANGE_LIMIT
    ), "후보 집합이 SCAN_NO_CHANGE_LIMIT 사이클 연속 동일하면 그 사이클에서 바로 DONE"
    assert names[-1] == "DONE"


def test_no_change_limit_is_configurable(make_ports, run_to_completion, monkeypatch):
    """K가 상수로 분리돼 실제로 발동 시점을 정한다 — 하드코딩된 2가 아니다."""
    monkeypatch.setattr(MissionContext, "hold", lambda self, track_id: self)
    monkeypatch.setattr(states_module, "SCAN_NO_CHANGE_LIMIT", 3)

    ports = make_ports(
        base=FakeBase(arrive=False),
        perception=ScriptedPerception(detections=[_detection(1)]),
    )

    states = run_to_completion(ports)
    names = [s.name for s in states]

    assert names.count("SCAN") == 3
    assert names[-1] == "DONE"


def test_no_change_detection_survives_float_jitter(make_ports, run_to_completion, monkeypatch):
    """이슈 #131 완료 조건 2 — 무변화 감지가 float 값 비교에 의존하지 않는다.

    실기에서는 같은 물체라도 pose_m·confidence가 프레임마다 미세하게 흔들린다.
    Detection을 값 비교하던 수정 전 구현은 두 프레임이 완전히 일치할 수 없어
    2차 방어선이 사실상 존재하지 않았다. 여기서 script=는 사이클마다 바닥이
    '바뀌는' 걸 흉내내는 게 아니라 **같은 물체에 카메라 노이즈만 얹는다** —
    track_id는 그대로다."""
    monkeypatch.setattr(MissionContext, "hold", lambda self, track_id: self)
    jittered = [
        [_detection(track_id=1, x=0.2 + i * 1e-6, confidence=0.9 - i * 1e-6)] for i in range(10)
    ]

    ports = make_ports(base=FakeBase(arrive=False), perception=ScriptedPerception(script=jittered))

    states = run_to_completion(ports)
    names = [s.name for s in states]

    assert names.count("SCAN") == SCAN_NO_CHANGE_LIMIT
    assert names[-1] == "DONE"


def test_zero_candidates_ends_at_select(make_ports, run_to_completion):
    """1차 방어선이 여전히 동작한다 — 후보가 0개인 경우는 SCAN이 아니라 SELECT가
    DONE으로 보낸다 (state_machine.md §3 SELECT 실패 시 전이).

    검출은 있지만 배치 규칙에 목적지가 없는 클래스뿐인 장면을 만든다 — SCAN은
    첫 사이클이라 무변화 감지가 발동할 수 없고, 판정은 SELECT가 한다."""
    spec = MissionSpec(
        mode=MissionMode.TIDY,
        target_cls=None,
        placement_rule={ObjectClass.CHESS_PIECE: BoxColor.BLUE},
        raw_text="장난감 정리해줘",
    )
    ports = make_ports(
        interpreter=ScriptedInterpreter(table={"장난감 정리해줘": spec}),
        perception=ScriptedPerception(detections=[_detection(track_id=1, cls=ObjectClass.GABE)]),
    )

    states = run_to_completion(ports)

    assert [s.name for s in states] == ["IDLE", "SCAN", "SELECT", "DONE"]
    assert states[-1].ctx.done_ids == frozenset()
    assert states[-1].ctx.held_ids == frozenset()


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
