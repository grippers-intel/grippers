"""장난감 정리 루프 FSM 통합 테스트. docs/design/state_machine.md 가 전이 그래프의
단일 소스이고, 특히 §4(재진입 방지)가 이 파일의 핵심이다.

전부 MissionTask.run() 을 끝까지 구동해서 검증한다 — 하드웨어·ROS2 없이
domain/adapters/fake/* 만 쓴다."""

import threading
from collections import Counter

from domain.adapters.fake.fake_arm import LOAD_EMPTY, LOAD_HOLDING, FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.scripted_interpreter import ScriptedInterpreter
from domain.adapters.fake.scripted_perception import ScriptedPerception
from domain.task import states as states_module
from domain.task.mission_task import MissionTask
from domain.task.states import (
    ALIGN_TOLERANCE_RAD,
    SCAN_NO_CHANGE_LIMIT,
    GraspState,
    InsertState,
    PosePlanState,
)
from domain.values import (
    BoxObservation,
    Destination,
    Detection,
    MissionContext,
    MissionMode,
    MissionSpec,
    ObjectClass,
    Point3,
    Pose2D,
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


def _box_observation(dest=Destination.LEFT, opening_mm=400.0):
    return BoxObservation(
        dest=dest,
        pose_m=Pose2D(x=0.5, y=0.0, theta=0.0),
        opening_mm=opening_mm,
        long_axis_rad=0.0,
    )


def _attempts_by_target(states, state_name):
    """상태 시퀀스에서 **대상별 시도 횟수**를 센다. state_machine.md §4:
    "'끝났다'만 보면 부족하다 — 몇 개가 실제로 시도됐는지까지 세야 한다."
    조기 종료 결함은 미션을 정상 종료시키므로 종료 여부만 보는 검증은 전부 통과한다."""
    return Counter(s.target.track_id for s in states if s.name == state_name)


# ── 1. 정상 완주 ──────────────────────────────────────────────────────────


def test_mission_ends_after_the_first_successful_insert(make_ports, run_to_completion):
    """2026-08-24 시연 범위 결정 — 투입에 **한 번** 성공하면 바닥에 물체가
    남아 있어도 미션을 끝낸다(states.py InsertState 주석 참고).

    바닥에 물체 2개를 놓지만 처리되는 건 가장 가까운 1번 하나뿐이고,
    2번은 손도 대지 않은 채 DONE으로 끝나야 한다. 예전 계약(둘 다 처리)으로
    되돌아가면 INSERT가 2회가 되어 여기서 걸린다."""
    det_a = _detection(track_id=1, cls=ObjectClass.GABE, x=0.2)
    det_b = _detection(track_id=2, cls=ObjectClass.CHESS_PIECE, x=0.4)
    ports = make_ports(perception=ScriptedPerception(detections=[det_a, det_b]))

    states = run_to_completion(ports)

    names = [s.name for s in states]
    assert names[0] == "IDLE"
    assert names[-1] == "DONE"
    assert names.count("INSERT") == 1
    assert states[-1].ctx.done_ids == {1}, "가장 가까운 물체 하나만 처리돼야 한다"
    assert 2 not in states[-1].ctx.held_ids, "2번은 실패한 게 아니라 아예 시도되지 않은 것이다"
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
    scan_floor() 결과가 사이클 1과 같아 물체 2·3이 한 번도 선택되지 않았다.

    ⚠️ 2026-08-24 시연 범위 결정(InsertState 주석)으로 **성공 경로는 한 번만**
    돈다 — 2번이 투입에 성공한 시점에 미션이 끝나므로 3번은 도달하지 않는다.
    여기서 고정하는 계약은 '1번이 보류된 뒤 2번이 선택된다'까지다. 물체
    3개 전부가 시도되는 것은 아래
    test_static_scene_all_grasps_failing_still_tries_every_object가 맡는다
    (전부 실패해 INSERT가 없으므로 루프가 끝까지 돈다)."""
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

    assert _attempts_by_target(states, "APPROACH") == {1: 1, 2: 1}, (
        "1번이 보류된 뒤 2번이 선정·접근돼야 한다 — 후보 집합이 줄어드는 것이 '진전'이다"
    )
    assert _attempts_by_target(states, "GRASP") == {1: 4, 2: 1}
    assert names.count("INSERT") == 1
    assert states[-1].ctx.held_ids == {1}
    assert states[-1].ctx.done_ids == {2}
    assert names[-1] == "DONE"


def test_static_scene_all_grasps_failing_still_tries_every_object(make_ports, run_to_completion):
    """이슈 #131 재현 시나리오 그대로 — 물체 3개, 파지는 항상 실패. 수정 전에는
    대상별 GRASP 시도가 {1: 4} 였다(2·3번은 선택조차 되지 않음).

    여기서 고정할 계약은 '세 물체가 모두 시도된다'까지다 — 대상별 시도 '횟수'는
    재시도 예산의 스코프 문제(이슈 #139)라 원인이 다르고, 아래 §6-2가 맡는다.
    두 결함은 증상이 겹쳐 보이지만 갈라 두는 편이 회귀 시 원인을 가려낸다."""
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
        placement_rule={ObjectClass.CHESS_PIECE: Destination.RIGHT},
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
        arm=FakeArm(load_ratio=LOAD_EMPTY),  # 항상 파지 실패(빈 채 실측값)
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
        arm=FakeArm(load_ratio=LOAD_EMPTY),
        perception=ScriptedPerception(detections=[_detection(track_id=7)]),
    )

    states = run_to_completion(ports)
    names = [s.name for s in states]

    assert names.count("GRASP") == 4  # 최초 시도 + 재시도 3회 (grasp_attempts 0,1,2,3)
    assert 7 in states[-1].ctx.held_ids
    assert names[-1] == "DONE"


# ── 6-2. 재시도 예산의 스코프 — 대상 1개 (이슈 #139) ★ ──────────────────


def test_grasp_budget_is_per_target_not_cumulative(make_ports, run_to_completion):
    """이슈 #139 — `grasp_attempts` 의 스코프는 **대상 1개**다. 첫 물체가 예산을
    전부 쓰고 보류돼도, 다음 물체는 다시 최초 시도 + MAX_GRASP_RETRY 회를 받는다.

    **정적 장면(detections=)으로 검증한다.** script= 로 사이클마다 바닥을 바꿔
    주면 보류된 물체가 관측에서 사라져 버려, 미션 누적 예산이 남아 있는 상태와
    구분이 안 된다 — 예산이 되돌아왔는지를 보려면 첫 물체가 예산을 실제로
    소진하고 바닥에 그대로 남아 있어야 한다 (state_machine.md §4).

    수정 전 거동: 대상별 GRASP가 {1: 4, 2: 1} — 1번이 미션 전체 예산을 소진해
    2번은 첫 시도 실패가 곧바로 영구 보류였다.

    리셋 자리가 SELECT가 아니라 GRASP로 잘못 들어가면 카운터가 0에 머물러 무한
    재시도가 되는데, 그건 위 §6 test_grasp_retry_exhaustion_holds_and_returns_to_scan
    이 대상 1개의 GRASP 횟수를 4로 고정해 잡는다."""
    detections = [_detection(track_id=1, x=0.2), _detection(track_id=2, x=0.4)]
    ports = make_ports(
        arm=FakeArm(load_ratio=LOAD_EMPTY),  # 두 물체 모두 파지 실패
        perception=ScriptedPerception(detections=detections),
    )

    states = run_to_completion(ports)
    names = [s.name for s in states]

    per_target = 1 + GraspState.MAX_GRASP_RETRY
    assert _attempts_by_target(states, "GRASP") == {1: per_target, 2: per_target}, (
        "두 물체가 각각 최초 시도 + MAX_GRASP_RETRY 회를 받아야 한다 — "
        "예산이 미션 누적이면 2번은 1회로 끝난다"
    )
    assert _attempts_by_target(states, "APPROACH") == {1: 1, 2: 1}
    assert states[-1].ctx.held_ids == {1, 2}
    assert names[-1] == "DONE"


def test_second_target_can_still_succeed_after_the_first_exhausts_the_budget(
    make_ports, run_to_completion
):
    """예산이 되돌아왔다는 것의 실제 의미 — 첫 물체가 예산을 소진한 뒤에도 두 번째
    물체는 재시도 끝에 **성공**할 수 있다.

    부하 시퀀스: 1번의 4회(최초+재시도 3회)가 전부 미달 → 보류. 2번은 3회 미달 뒤
    4회차에 파지 성공한다 — 미션 누적 예산이었다면 2번의 첫 실패에서 이미 보류돼
    이 성공에 도달하지 못한다. `failure_definition.md` §3의 "재시도 후 성공은
    실패가 아니다"가 두 번째 물체부터도 성립해야 M4 측정이 유효하다."""
    loads = [LOAD_EMPTY] * 4 + [LOAD_EMPTY] * 3 + [LOAD_HOLDING]
    ports = make_ports(
        arm=FakeArm(load_ratio=loads),
        perception=ScriptedPerception(
            detections=[_detection(track_id=1, x=0.2), _detection(track_id=2, x=0.4)]
        ),
    )

    states = run_to_completion(ports)
    names = [s.name for s in states]

    assert _attempts_by_target(states, "GRASP") == {1: 4, 2: 4}
    assert names.count("INSERT") == 1
    assert states[-1].ctx.held_ids == {1}
    assert states[-1].ctx.done_ids == {2}
    assert names[-1] == "DONE"


def test_reset_attempts_returns_a_new_context():
    """`MissionContext` 는 불변이다 — `reset_attempts()` 도 제자리 변경이 아니라
    새 인스턴스를 반환하고, 다른 필드는 건드리지 않는다
    (docs/design/class_diagram.md §1)."""
    spec = MissionSpec(
        mode=MissionMode.TIDY, target_cls=None, placement_rule={}, raw_text="정리해줘"
    )
    ctx = MissionContext(spec=spec, done_ids=frozenset({1}), held_ids=frozenset({2}), last_scan=())
    spent = ctx.retry().retry()

    reset = spent.reset_attempts()

    assert spent.grasp_attempts == 2, "원본이 제자리에서 바뀌면 안 된다"
    assert reset is not spent
    assert reset.grasp_attempts == 0
    assert (reset.spec, reset.done_ids, reset.held_ids, reset.last_scan) == (
        spec,
        frozenset({1}),
        frozenset({2}),
        (),
    )


# ── 6-1. 부하 임계값 — 실측 경계 ★ ───────────────────────────────────────
#
# 실측 (2026-08-18, n=25, 정착 2초 후, 절대값 / 1023):
#   빈 채 / 파지 실패(놓침)  raw 28~32    → 0.027 ~ 0.031
#   체스말(나이트·룩)        raw 48~124   → 0.047 ~ 0.121
#   가베(정육면체)           raw 140      → 0.137 (5/5 일관)
# 임계값은 이 두 분포 사이(0.031 < LOAD_THRESHOLD < 0.047)에 있어야 한다.


def test_empty_gripper_load_fails_grasp(make_ports, run_to_completion):
    """빈 채 실측 최대값(0.031)은 파지 실패로 판정돼야 한다.

    이전 임계값 0.15는 이 경계를 '통과시키지 못한' 게 아니라 아예 모든 값을
    막았다 — 가베(0.137)조차 미달이라 실기에서는 파지가 성공할 수 없었다."""
    ports = make_ports(
        arm=FakeArm(load_ratio=0.031),
        perception=ScriptedPerception(detections=[_detection(track_id=1)]),
    )

    states = run_to_completion(ports)
    names = [s.name for s in states]

    assert names.count("GRASP") == 4, "부하 미달이므로 최초 시도 + 재시도 3회를 다 쓴다"
    assert "TRANSPORT" not in names
    assert "INSERT" not in names
    assert states[-1].ctx.held_ids == {1}
    assert names[-1] == "DONE"


def test_minimum_holding_load_passes_grasp(make_ports, run_to_completion):
    """파지 성공 실측 최소값(0.047 — 체스말 나이트)은 통과해야 한다.
    ⚠️ 빈 채 최대 0.031과의 여유가 raw 기준 16틱뿐이다."""
    ports = make_ports(
        arm=FakeArm(load_ratio=0.047),
        perception=ScriptedPerception(detections=[_detection(track_id=1)]),
    )

    states = run_to_completion(ports)
    names = [s.name for s in states]

    assert names.count("GRASP") == 1, "첫 시도에서 부하 임계를 넘어야 한다"
    assert "TRANSPORT" in names
    assert names.count("INSERT") == 1
    assert states[-1].ctx.done_ids == {1}


def test_grasp_rechecks_at_145mm_then_folds_to_carry_idle_before_transport(make_ports):
    """수평 파지는 145 mm 재검증과 CARRY_IDLE을 마쳐야 운반으로 넘어간다."""
    target = _detection(track_id=1)
    arm = FakeArm(load_ratio=0.047)
    perception = ScriptedPerception(detections=[target])
    ports = make_ports(arm=arm, perception=perception)
    ctx = MissionContext(
        spec=MissionSpec(
            mode=MissionMode.TIDY,
            target_cls=ObjectClass.GABE,
            placement_rule={ObjectClass.GABE: Destination.LEFT},
            raw_text="",
        )
    )

    next_state = GraspState(ctx, target).execute(ports)

    assert arm.floor_pose_calls == [
        ("soccer_polyhedron", "safe"),
        ("soccer_polyhedron", "grasp"),
        ("soccer_polyhedron", "midpoint"),
        ("soccer_polyhedron", "safe"),
        ("soccer_polyhedron", "idle"),
    ]
    assert arm.gripper_widths == [168.0, 35.0]
    assert next_state.name == "TRANSPORT"
    # 로깅 전용: confirm_grasp가 실제로 호출되는지만 검증한다 — 판정에는
    # 아직 안 쓰이므로 next_state는 confirm_grasp 결과와 무관하게 TRANSPORT다.
    assert perception.confirm_grasp_calls == 1
    # 이 대상은 soccer_polyhedron 프로필이라 raw 클래스를 가릴 수 없다
    # (star/soccer 가 폭으로 겹친다 — _PROFILE_TO_RAW_CLASS 참고). 그래서
    # 기억 단계를 건너뛴다. 기준 없이 확인만 하면 어댑터가 False 를 내므로
    # 잘못된 성공 판정으로 이어지지 않는다.
    assert perception.remember_target_calls == 0
    assert perception.remembered_cls is None


def test_failed_lift_releases_object_and_blocks_transport(make_ports):
    """파지는 됐어도 140 mm 안전 높이 상승 실패를 운반 성공으로 보지 않는다."""

    class LiftFailingArm(FakeArm):
        def move_to_floor_pose(self, profile, stage):
            self.floor_pose_calls.append((profile, stage))
            return len(self.floor_pose_calls) < 4

    target = _detection(track_id=1)
    arm = LiftFailingArm(load_ratio=0.047)
    ports = make_ports(
        arm=arm,
        perception=ScriptedPerception(detections=[target]),
    )
    ctx = MissionContext(
        spec=MissionSpec(
            mode=MissionMode.TIDY,
            target_cls=ObjectClass.GABE,
            placement_rule={ObjectClass.GABE: Destination.LEFT},
            raw_text="",
        )
    )

    next_state = GraspState(ctx, target).execute(ports)

    assert next_state.name == "GRASP"
    assert arm.gripper_widths[-1] == states_module.OPEN_MM


def test_horizontal_mid_lift_load_drop_blocks_safe_lift(make_ports):
    target = _detection(track_id=1)
    arm = FakeArm(load_ratio=[0.07, 0.03])
    ports = make_ports(arm=arm, perception=ScriptedPerception(detections=[target]))
    ctx = MissionContext(
        spec=MissionSpec(
            mode=MissionMode.TIDY,
            target_cls=ObjectClass.GABE,
            placement_rule={ObjectClass.GABE: Destination.LEFT},
            raw_text="",
        )
    )

    next_state = GraspState(ctx, target).execute(ports)

    assert next_state.name == "GRASP"
    assert arm.floor_pose_calls[-1] == ("soccer_polyhedron", "midpoint")
    assert arm.floor_pose_calls.count(("soccer_polyhedron", "safe")) == 1
    assert arm.gripper_widths[-1] == states_module.OPEN_MM


def test_carry_idle_load_drop_hard_stops_before_base_transport(make_ports):
    target = _detection(track_id=1)
    arm = FakeArm(load_ratio=[0.07, 0.07, 0.03])
    ports = make_ports(arm=arm)
    ctx = MissionContext(
        spec=MissionSpec(
            mode=MissionMode.TIDY,
            target_cls=ObjectClass.GABE,
            placement_rule={ObjectClass.GABE: Destination.LEFT},
            raw_text="",
        )
    )

    next_state = GraspState(ctx, target).execute(ports)

    assert next_state.name == "ESTOP"
    assert arm.floor_pose_calls[-1] == ("soccer_polyhedron", "idle")


def test_vertical_fallback_is_used_only_when_horizontal_safe_pose_is_unavailable(make_ports):
    class NoHorizontalArm(FakeArm):
        def move_to_floor_pose(self, profile, stage):
            self.floor_pose_calls.append((profile, stage))
            return False

    target = _detection(track_id=1)
    arm = NoHorizontalArm(load_ratio=0.07)
    ports = make_ports(arm=arm)
    ctx = MissionContext(
        spec=MissionSpec(
            mode=MissionMode.TIDY,
            target_cls=ObjectClass.GABE,
            placement_rule={ObjectClass.GABE: Destination.LEFT},
            raw_text="",
        )
    )

    next_state = GraspState(ctx, target).execute(ports)

    assert arm.floor_pose_calls == [("soccer_polyhedron", "safe")]
    assert [down for _, down in arm.move_calls] == [False, True, False]
    assert next_state.name == "TRANSPORT"


def test_load_threshold_sits_between_measured_distributions():
    """임계값이 상수로 분리돼 있고, 실측 두 분포 사이에 있다.
    물체 종류가 추가돼 재측정할 때 이 테스트가 경계를 다시 확인해 준다."""
    assert 0.031 < GraspState.LOAD_THRESHOLD < 0.047


def test_load_threshold_is_tunable(make_ports, run_to_completion, monkeypatch):
    """판정이 하드코딩이 아니라 LOAD_THRESHOLD 상수를 실제로 참조한다 —
    임계값을 올리면 같은 부하가 실패로 뒤집힌다."""
    monkeypatch.setattr(GraspState, "LOAD_THRESHOLD", 0.2)

    ports = make_ports(
        arm=FakeArm(load_ratio=LOAD_HOLDING),
        perception=ScriptedPerception(detections=[_detection(track_id=1)]),
    )

    states = run_to_completion(ports)
    names = [s.name for s in states]

    assert "INSERT" not in names
    assert states[-1].ctx.held_ids == {1}


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
        arm=FakeArm(load_ratio=[LOAD_HOLDING, LOAD_HOLDING, LOAD_HOLDING, LOAD_EMPTY]),
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
        arm=FakeArm(load_ratio=[LOAD_HOLDING, LOAD_EMPTY]),
        perception=ScriptedPerception(detections=[_detection(track_id=9, cls=ObjectClass.GABE)]),
    )

    states = run_to_completion(ports, raw_text="체스말 가져와")
    names = [s.name for s in states]

    assert "APPROACH" not in names
    assert names[-1] == "DONE"
    assert states[-1].ctx.done_ids == frozenset()


# ── 10. TRANSPORT 정렬 오차 판정 (hld.md §6.4 #10) ★ ─────────────────────
#
# align_to_box()의 실패값(무한대)을 흡수하는지는 tests/test_fake_failure_contracts.py
# 가 본다. 여기서 고정하는 건 그 위의 계약 — 판정이 '무한대인가'가 아니라
# **허용 오차와의 비교**라는 것이다. inf만 검사하면 '정렬은 됐지만 오차가 큰'
# 경우가 그대로 INSERT로 흘러가는데, 그건 포트가 두 상황을 굳이 다른 값으로
# 구분해 돌려주는 이유(domain/ports/base_driver.py)를 버리는 셈이다.


def test_align_error_within_tolerance_proceeds_to_insert(make_ports, run_to_completion):
    """허용 오차 안의 정렬 오차는 성공이다 — 0.0(완벽 정렬)만 통과시키면 실기에서
    어떤 물체도 상자에 들어가지 못한다."""
    ports = make_ports(
        base=FakeBase(align_error_rad=ALIGN_TOLERANCE_RAD * 0.5),
        perception=ScriptedPerception(detections=[_detection(track_id=1)]),
    )

    states = run_to_completion(ports)
    names = [s.name for s in states]

    assert names.count("INSERT") == 1
    assert states[-1].ctx.done_ids == {1}


def test_basket_insert_opens_at_drop_195_without_lowering_to_floor(make_ports, run_to_completion):
    arm = FakeArm()
    ports = make_ports(
        arm=arm,
        perception=ScriptedPerception(detections=[_detection(track_id=1)]),
    )

    states = run_to_completion(ports)

    assert "INSERT" in [state.name for state in states]
    assert arm.floor_pose_calls[-2:] == [
        ("soccer_polyhedron", "drop"),
        ("soccer_polyhedron", "idle"),
    ]
    assert arm.gripper_widths[-1] == states_module.OPEN_MM
    assert all(not down for _, down in arm.move_calls)


def test_align_error_beyond_tolerance_holds_the_target(make_ports, run_to_completion):
    """허용 오차를 넘으면 정렬 실패로 보고 보류 등록 + SCAN 복귀한다. 값은 유한한데
    임계를 넘는 경우 — 판정이 `math.isinf()` 가 아니라 임계값 비교여야 잡힌다."""
    ports = make_ports(
        base=FakeBase(align_error_rad=ALIGN_TOLERANCE_RAD * 2),
        perception=ScriptedPerception(detections=[_detection(track_id=1)]),
    )

    states = run_to_completion(ports)
    names = [s.name for s in states]

    assert "POSE_PLAN" not in names
    assert "INSERT" not in names
    assert states[-1].ctx.held_ids == {1}
    assert names[-1] == "DONE"


def test_align_error_sign_does_not_change_the_verdict(make_ports, run_to_completion):
    """yaw 오차의 부호는 정렬해야 할 **방향**일 뿐이라 판정은 크기만 본다 —
    부호를 그대로 비교하면 왼쪽으로 틀어진 경우가 전부 통과해 버린다."""
    ports = make_ports(
        base=FakeBase(align_error_rad=-ALIGN_TOLERANCE_RAD * 2),
        perception=ScriptedPerception(detections=[_detection(track_id=1)]),
    )

    states = run_to_completion(ports)

    assert "INSERT" not in [s.name for s in states]
    assert states[-1].ctx.held_ids == {1}


def test_visual_grasp_check_remembers_the_target_before_descending(make_ports, run_to_completion):
    """시각 파지 확인은 두 관측의 짝이다.

    내려가기 **전에** 기억해야 한다 — grasp 자세로 내려가면 팔이 depth
    카메라를 가려서 그때는 정면을 볼 수 없다(2026-08-25 실기 확인). 그리고
    확인은 CARRY_IDLE 에서 한다. 순서가 어긋나면 비교 기준이 없어 판정이
    늘 실패한다.

    raw 클래스를 가릴 수 있는 체스말로 확인한다 — GABE(star/soccer)는 폭이
    겹쳐 매핑이 없다(_PROFILE_TO_RAW_CLASS 참고).
    """
    # 폭 24.5mm -> chess_rook 프로필 -> raw 클래스 "rook"
    rook = Detection(
        track_id=1,
        cls=ObjectClass.CHESS_PIECE,
        pose_m=Point3(x=0.2, y=0.0, z=0.0),
        dims_m=Point3(x=0.0245, y=0.0245, z=0.045),
        yaw_rad=0.0,
        confidence=0.95,
    )
    perception = ScriptedPerception(detections=[rook])
    ports = make_ports(perception=perception)
    run_to_completion(ports)

    assert perception.remembered_cls == "rook"
    assert perception.remember_target_calls >= 1
    assert perception.confirm_grasp_calls >= 1
