"""장난감 정리 주제의 루프 FSM. docs/design/state_machine.md 가 전이 그래프의 단일 소스이고,
docs/design/sequences.md 가 상태 내부 포트 호출 순서의 단일 소스다.

이전 주제(암실 반출)는 **선형** FSM이었다 — 한 번 끝까지 가면 종료였고, 어느 단계에서
실패하든 미션이 끝났다. 이 FSM은 **`SCAN` 으로 되돌아오는 루프**다:

- `SCAN` 이 루프의 유일한 재진입점. 모든 사이클이 여기로 되돌아온다
- 종료 조건은 "마지막 단계 도달"이 아니라 "관측 결과 남은 대상 없음" → `DONE`
- `*FailedState` 4종은 전부 삭제됐다. 실패는 미션 종료가 아니라 `SCAN` 복귀 +
  보류 목록(`held_ids`) 등록이다 (state_machine.md §5)

E-STOP은 이 전이 그래프에 없다 — `MissionTask.run()` 이 다음 `execute()` 호출 전에
`EstopState` 로 갈아치우는 인터럽트이지 정상 전이가 아니다 (state_machine.md §2)."""

from dataclasses import replace

from domain.task.floor_grasp_policy import (approach_target_key,
                                            select_horizontal_grasp_plan)
from domain.task.state import State
from domain.values import MissionContext, MissionMode, Point3, Pose2D

# ── 실측/미실측 상수 ─────────────────────────────────────────────────────
# 그리퍼 개구 폭은 2026-08-20 실측 완료. 나머지 좌표/임계값은 TODO이며,
# 하드코딩된 값을 이 블록 한 곳에 모아 실측 후 교체한다.
OPEN_MM = 168.0
CLOSED_MM = 9.0
GRASP_APPROACH_HEIGHT_M = 0.10  # TODO: 실측 — 파지 하강 전 안전 여유 높이
MIN_GRIPPER_CLEARANCE_M = 0.14  # 실측 요구 — 베이스 이동 전 바닥부터 파지 중심 높이
HANDOVER_POINT_M = Point3(x=0.30, y=0.0, z=0.35)  # TODO: 실측 — 사용자 인계 손끝 위치
HANDOVER_LOAD_THRESHOLD = 0.05  # TODO: 실측 — '사람이 받아감' 판정 부하 임계값
PUT_DOWN_POINT_M = Point3(x=0.30, y=0.0, z=0.0)  # TODO: 실측 — REJECT 시 안전한 내려놓기 위치
DELIVERY_POINT_M = Pose2D(x=0.0, y=0.0, theta=0.0)  # TODO: 실측 — FETCH 인계 접근 위치
# TODO: 실측 — 상자 앞 정렬을 '성공'으로 볼 yaw 오차 상한(rad). 0.05 rad ≈ 2.9°는
# 자리 표시자이며 검증되지 않았다. 실제 값은 상자 입구 여유폭과 투입 지점 IK 오차로
# 결정되므로, 미결 #4(엔드이펙터 개구 폭)·#7(margin) 실측과 함께 다시 잡아야 한다.
# 임계를 넘으면 실패로 보고 대상을 보류한다 — 실패 표현은 포트가 정한
# ALIGN_FAILED_YAW_ERROR_RAD(math.inf)이라 어떤 상한과 비교해도 실패로 남는다.
ALIGN_TOLERANCE_RAD = 0.05

# ── SCAN 무변화 감지 ─────────────────────────────────────────────────────
# 같은 후보 집합이 연속 몇 사이클 관측되면 "진전이 없다"고 볼 것인가
# (state_machine.md §4). 2 = 직전 사이클과 같으면 즉시 발동. 실기 관측이
# 흔들려 후보 집합이 한 사이클씩 깜빡이면 늘리는 쪽으로 조정한다.
SCAN_NO_CHANGE_LIMIT = 2


def select_candidates(ctx, detections):
    """`SELECT` 가 실제로 고를 수 있는 검출만 남긴다 — 선정 기준(state_machine.md §3)의
    1~3번(보류/완료 목록에 없을 것 · 목적지가 정의돼 있을 것 · FETCH 모드는 클래스 일치)이다.
    4번(최단 거리)은 후보 중 하나를 고르는 단계라 `SelectState` 쪽에 남는다.

    `SCAN` 무변화 감지가 "`SELECT` 가 고를 수 있는 것이 줄었는가"를 판단 기준으로 쓰므로
    두 상태가 반드시 같은 필터를 봐야 한다 — 복사본이 갈라지면 `SCAN` 이 `SELECT` 와 다른
    세계를 보게 되고, 그게 이슈 #131 이 재발하는 경로다."""
    spec = ctx.spec
    return [
        d
        for d in detections
        if d.track_id not in ctx.done_ids
        and d.track_id not in ctx.held_ids
        and d.cls in spec.placement_rule
        and (spec.mode is not MissionMode.FETCH or d.cls == spec.target_cls)
    ]


class IdleState(State):
    """`ctx.spec.raw_text` 를 `interpreter.parse()` 로 (재)해석해 `SCAN` 으로 넘어간다.
    `ctx` 가 없거나 `raw_text` 가 비어 있으면 대기(자기 자신 반환)한다.

    **해석하지 못한 명령도 대기다.** `parse()` 는 등록되지 않은 문형에 `None` 을
    돌려주는 것이 포트 계약이고(domain/ports/command_interpreter.py), real
    구현(`Ros2CommandInterpreter`)도 `understood=False` 면 `None` 이다. 이때
    `MissionContext(spec=None)` 을 만들어 `SCAN` 으로 넘기면 `SELECT` 가
    `spec.placement_rule` 을 읽는 순간 `AttributeError` 로 FSM 스레드가 죽는다.
    "미션이 시작되지 않았다"는 상태를 따로 만들지 않고 **IDLE 유지**로 표현한다 —
    state_machine.md §3의 `IDLE` 계약이 실패 시 전이를 "대기 유지"로 정의한다.
    `raw_text` 가 비어 있을 때와 같은 자리로 수렴하므로 분기도 하나로 남는다.

    `Ports.interpreter` 는 mission_task.py 마이그레이션(#7)에서 추가됐다 —
    `MissionTask.run(raw_text)` 가 이 State에 넘길 초기 `MissionContext` 를 만든다."""

    name = "IDLE"

    def __init__(self, ctx=None):
        self.ctx = ctx

    def execute(self, ports):
        if self.ctx is None or not self.ctx.spec.raw_text:
            return self
        spec = ports.interpreter.parse(self.ctx.spec.raw_text)
        if spec is None:
            return self
        return ScanState(MissionContext(spec=spec))


class ScanState(State):
    """루프의 유일한 재진입점. 무한 루프 방지 2종을 모두 구현한다
    (state_machine.md §4):

    1. `MAX_RESCAN` — 빈 관측이면 바로 `DONE` 이 아니라 재스캔을 먼저 시도한다
       (일시적 오검출 대비). 재시도가 소진되면 `DONE`.
    2. **SCAN 무변화 감지** — `SELECT` 후보의 `track_id` 집합이
       `SCAN_NO_CHANGE_LIMIT` 사이클 연속 동일하면 진전이 없다고 보고 `DONE`.
       `done_ids`/`held_ids` 필터링이 이미 `SELECT` 에서 무한 루프를 막지만,
       이건 그 필터링 자체가 깨졌을 때의 2차 방어선이다.

    비교 대상이 `scan_floor()` 결과 전체가 아니라 **후보의 `track_id` 집합**인 것이
    핵심이다 (이슈 #131). 관측 목록 전체를 비교하면 두 방향으로 어긋난다 — 보류된
    물체도 바닥에는 그대로 남아 있으므로 Fake 에서는 진전이 가능한데도 목록이 같아
    **과잉 발동**하고, 실기에서는 `pose_m`·`confidence` 가 float 이라 카메라 노이즈만
    있어도 두 프레임이 완전히 일치할 수 없어 **절대 발동하지 않는다**. 후보 집합은
    물체가 보류·완료될 때마다 줄어들고, 정수 집합 비교라 노이즈에 흔들리지 않는다.

    후보가 0개인 경우는 여기서 처리하지 않는다 — `SELECT` 가 `DONE` 으로 보내는 1차
    방어선이 이미 담당하고, 2차 방어선이 잡아야 하는 건 "후보는 있는데 진전이 없다"다.

    비교 기준은 `ctx.last_scan` 에 둔다 (`self` 의 인스턴스 필드가 아니다) —
    `APPROACH`/`GRASP`/... 를 거쳐 `SCAN` 으로 돌아오는 경로는 전부
    `ScanState(ctx)` 1인자 생성자를 쓰므로, 인스턴스 필드에 두면 그 경로들을
    거칠 때마다 초기화돼 사이클을 건너는 비교가 성립하지 않는다."""

    name = "SCAN"
    MAX_RESCAN = 3

    def __init__(self, ctx, rescans=0):
        self.ctx = ctx
        self.rescans = rescans

    def execute(self, ports):
        detections = ports.perception.scan_floor()

        if not detections:
            if self.rescans < self.MAX_RESCAN:
                return ScanState(self.ctx, self.rescans + 1)
            return DoneState(self.ctx)

        # last_scan 은 "연속으로 같게 관측된 후보 집합"의 이력이다 — 집합이 바뀌면
        # 길이 1로 되돌아가고, SCAN_NO_CHANGE_LIMIT 에 닿으면 진전 없음으로 본다.
        candidate_ids = frozenset(d.track_id for d in select_candidates(self.ctx, detections))
        previous = self.ctx.last_scan
        if previous and previous[-1] == candidate_ids:
            streak = previous + (candidate_ids,)
        else:
            streak = (candidate_ids,)

        ctx = replace(self.ctx, last_scan=streak)
        if len(streak) >= SCAN_NO_CHANGE_LIMIT:
            return DoneState(ctx)

        return SelectState(ctx, detections)


class SelectState(State):
    """순수 판단 상태 — 포트를 호출하지 않는다. 선정 기준(state_machine.md §3):

    1. 보류/완료 목록에 없을 것
    2. `placement_rule` 에 목적지가 정의되어 있을 것
    3. (FETCH 모드) `spec.target_cls` 와 일치할 것
    4. 위 조건을 만족하는 것 중 base_link 로부터 최단 거리

    사전 필터(치수만으로 φ 해 없음을 미리 거르는 것)는 의도적으로 넣지 않는다 —
    그러면 유즈케이스 2(투입 불가 판정)가 축소된다.

    **선정과 동시에 `grasp_attempts` 를 0으로 되돌린다** (state_machine.md §3·§4,
    이슈 #139). 재시도 예산의 스코프는 대상 1개이고, 대상이 바뀌는 지점은 이곳
    하나뿐이라 리셋 자리도 여기 하나다. 후보가 없어 `DONE` 으로 가는 경로는 잡을
    대상이 없으므로 되돌리지 않는다."""

    name = "SELECT"

    def __init__(self, ctx, detections):
        self.ctx = ctx
        self.detections = detections

    def execute(self, ports):
        target = self._pick(self.detections)
        if target is None:
            return DoneState(self.ctx)
        return ApproachState(self.ctx.reset_attempts(), target)

    def _pick(self, detections):
        # 1~3번 조건은 select_candidates() 한 곳에 있다 — SCAN 무변화 감지가 같은
        # 필터를 봐야 하므로 여기서 복제하지 않는다. 여기 남는 건 4번(최단 거리)뿐이다.
        candidates = select_candidates(self.ctx, detections)
        if not candidates:
            return None
        return min(candidates, key=lambda d: (d.pose_m.x**2 + d.pose_m.y**2) ** 0.5)


class ApproachState(State):
    """⚠️ 2026-08-23: `base.approach()`는 시각 서보 폐루프다 — pose 1회 추정치로
    목표점을 계산해 오도메트리로 주행하는 게 아니라, real 어댑터가 라이브
    카메라 관측에 반복 수렴시킨다(domain/ports/base_driver.py의 `approach`
    docstring, HANDOFF.md 참고). 그래서 여기서 목표 pose를 따로 계산하지
    않는다 — `target`을 그대로 넘기면 real 구현이 필요한 것만 뽑아 쓴다."""

    name = "APPROACH"

    def __init__(self, ctx, target):
        self.ctx = ctx
        self.target = target

    def execute(self, ports):
        arrived = ports.base.approach(self.target)
        if not arrived:
            return ScanState(self.ctx.hold(self.target.track_id))
        return GraspState(self.ctx, self.target)


class GraspState(State):
    """부하 기반 파지 검증 — 힘/토크 센서 없이 서보 부하값으로 판정한다
    (sequences.md §2). 재시도는 상태 변경이 아니라 새 인스턴스 반환으로
    표현한다 (기존 코드 관례 유지).

    `grasp_attempts` 는 **대상 1개 기준**의 남은 예산이고, 되돌리는 자리는
    `SelectState` 다 (이슈 #139). 여기서 `reset_attempts()` 를 부르면 안 된다 —
    `GRASP` 는 재시도마다 자기 자신을 새로 만들므로 카운터가 영원히 0에 머물러
    무한 재시도가 된다."""

    name = "GRASP"
    MAX_GRASP_RETRY = 3

    # 실측 근거 (2026-08-18, n=25, 정착 후 · 절대값 / 1023):
    #   빈 채·놓침      0.027 ~ 0.031
    #   체스말          0.047 ~ 0.121
    #   가베(정육면체)  0.137 (5/5 일관)
    #   체스말(퀸)      0.031(놓침) 또는 0.156 ~ 0.164 — 유선형이라 양극단
    # 즉 빈 채 최대 0.031 < 임계 < 파지 성공 최소 0.047.
    # ⚠️ 여유가 크지 않다 — raw 기준 32 대 48로 16틱 차이다. 파지 대상
    #    클래스가 추가되면 재측정이 필요하다.
    # 이전 값 0.15 는 어떤 물체도 넘지 못했다(가베조차 0.137) — 실기에서
    # 파지 판정이 항상 실패했다.
    LOAD_THRESHOLD = 0.04
    RETRY_ELIGIBLE_LOAD = 0.031
    RETRY_TIGHTEN_MM = 5.0

    def __init__(self, ctx, target):
        self.ctx = ctx
        self.target = target

    def execute(self, ports):
        plan = select_horizontal_grasp_plan(self.target)

        # 기본 경로는 실기 검증된 수평 파지다. 80 mm로 충분히 벌린 뒤 내려가
        # 체스말의 돌출된 머리에 손가락이 걸리지 않게 한다.
        if not ports.arm.move_to_floor_pose(plan.profile, "safe"):
            return self._execute_vertical_fallback(ports)
        ports.arm.set_gripper(plan.preopen_width_mm)
        # 내려가기 직전이 정면을 볼 수 있는 마지막 순간이다 — grasp 자세로
        # 내려가면 팔이 depth 카메라를 가린다. 여기서 목표를 기억해 두면
        # CARRY_IDLE 에서 "그 자리에 있던 것이 사라졌는가"를 물을 수 있다.
        # star/soccer 처럼 폭이 겹쳐 raw 클래스를 못 가르는 경우는 None 이다 —
        # 그때는 기억하지 않고, confirm_grasp 도 기준이 없어 False 를 낸다.
        raw_cls = approach_target_key(self.target)
        if raw_cls is not None:
            ports.perception.remember_target(raw_cls)
        if not ports.arm.move_to_floor_pose(plan.profile, "grasp"):
            return self._retry_after_release(ports)

        ports.arm.set_gripper(plan.close_width_mm)
        load = ports.arm.get_load()
        if self.RETRY_ELIGIBLE_LOAD < load < self.LOAD_THRESHOLD:
            retry_width = max(CLOSED_MM, plan.close_width_mm - self.RETRY_TIGHTEN_MM)
            ports.arm.set_gripper(retry_width)
            load = ports.arm.get_load()

        # 바닥에서 곧장 140 mm로 들지 않는다. 실측처럼 중간 높이에서 미끄러짐을
        # 다시 확인한 뒤에만 베이스가 움직일 수 있는 안전 자세로 간다.
        lifted = load >= self.LOAD_THRESHOLD and ports.arm.move_to_floor_pose(
            plan.profile, "midpoint"
        )
        held = lifted and ports.arm.get_load() >= self.LOAD_THRESHOLD
        cleared = held and ports.arm.move_to_floor_pose(plan.profile, "safe")
        if cleared:
            # 물체를 든 채 SAFE_145 → IDLE 관절 자세로 직접 접는 경로를 큐브로
            # 실측했다. servo 2 load가 196 → 64로 줄고 gripper load는 0.1056으로
            # 유지됐다. CARRY_IDLE 도달과 파지 재검증 전에는 베이스가 움직이면 안 된다.
            if not ports.arm.move_to_floor_pose(plan.profile, "idle"):
                return EstopState()
            if ports.arm.get_load() < self.LOAD_THRESHOLD:
                return EstopState()
            # CARRY_IDLE 에서는 팔이 프레임 밖이라 정면 바닥이 그대로 보인다
            # (2026-08-25 실기 확인). load 와 **독립적인 두 번째 근거**로,
            # 목표가 있던 자리에서 사라졌는지 확인한다 — load 는 "무언가를
            # 쥐었다"를, 이쪽은 "목표가 없어졌다"를 말한다.
            #
            # ⚠️ 아직 판정에 쓰지 않는다(로깅 전용). 내려오는 그리퍼가 물체를
            # 쳐서 시야 밖으로 밀어낸 경우에도 "사라짐"으로 보이므로, load
            # 임계값을 n=25 실측으로 잡았던 것과 같은 절차를 거친 뒤에만
            # 판정에 편입한다.
            ports.perception.confirm_grasp()
            if self.ctx.spec.mode is MissionMode.TIDY:
                return TransportState(self.ctx, self.target)
            return DeliverState(self.ctx, self.target)

        return self._retry_after_release(ports)

    def _execute_vertical_fallback(self, ports):
        """수평 전용 자세를 쓸 수 없을 때만 기존 수직 IK를 한 번 시도한다."""
        floor_z = self.target.pose_m.z - self.target.dims_m.z / 2.0
        safe_z = max(
            self.target.pose_m.z + GRASP_APPROACH_HEIGHT_M,
            floor_z + MIN_GRIPPER_CLEARANCE_M,
        )
        approach_point = Point3(
            x=self.target.pose_m.x,
            y=self.target.pose_m.y,
            z=safe_z,
        )
        if not ports.arm.move_to_cartesian(approach_point):
            return self._retry_after_release(ports)
        ports.arm.set_gripper(OPEN_MM)
        if not ports.arm.move_to_cartesian(self.target.pose_m, down=True):
            return self._retry_after_release(ports)
        ports.arm.set_gripper(CLOSED_MM)

        if ports.arm.get_load() >= self.LOAD_THRESHOLD and ports.arm.move_to_cartesian(
            approach_point
        ):
            if self.ctx.spec.mode is MissionMode.TIDY:
                return TransportState(self.ctx, self.target)
            return DeliverState(self.ctx, self.target)

        # 빈손이거나 안전 운반 높이까지 들지 못했다. 낮은 위치에서 물체를 놓고
        # 이전 pose를 재사용하지 않는다. 들어 올리지 못한 상태로 베이스가 움직이는
        # 것보다 대상을 보류하고 재스캔하는 편이 안전하다.
        return self._retry_after_release(ports)

    def _retry_after_release(self, ports):
        ports.arm.set_gripper(OPEN_MM)
        if self.ctx.grasp_attempts >= self.MAX_GRASP_RETRY:
            return ScanState(self.ctx.hold(self.target.track_id))

        # 실패한 파지가 물체를 밀었을 수 있다 — 이전 pose 재사용은 같은 실패를
        # 반복하므로 재스캔해서 같은 track_id의 갱신된 pose로 재시도한다.
        refreshed = ports.perception.scan_floor()
        updated = next((d for d in refreshed if d.track_id == self.target.track_id), self.target)
        return GraspState(self.ctx.retry(), updated)


class TransportState(State):
    """상자 앞까지 이송하고 정렬한다. 정렬 결과는 **판정 대상**이다 —
    `align_to_box()` 가 돌려주는 yaw 오차(rad)가 `ALIGN_TOLERANCE_RAD` 를 넘으면
    상자 앞에 서지 못한 것이므로 투입으로 진행하지 않고 대상을 보류한 뒤 `SCAN`
    으로 복귀한다 (hld.md §6.4 #10, state_machine.md §3 `TRANSPORT` 실패 시 전이).

    반환값을 버리면 정렬 실패가 성공으로 흘러간다 — 포트가 실패를
    `ALIGN_FAILED_YAW_ERROR_RAD`(무한대)로 표현하는 것도 임계값과 비교되는 것을
    전제로 한 설계다. 실패를 `0.0`(완벽 정렬)으로 표현하면 안 되는 이유와 같다
    (domain/ports/base_driver.py)."""

    name = "TRANSPORT"

    def __init__(self, ctx, target):
        self.ctx = ctx
        self.target = target

    def execute(self, ports):
        dest = self.ctx.spec.placement_rule[self.target.cls]
        box = ports.perception.find_box(dest)
        if box is None:
            return ScanState(self.ctx.hold(self.target.track_id))

        # TODO: 실측 — 상자 "앞" 지점은 box.pose_m 그대로가 아니라 접근 오프셋이
        # 필요하다. 실측 전까지는 상자 관측 pose를 그대로 목표로 쓴다.
        #
        # ⚠️ 2026-08-23: HANDOFF.md "바구니 접근 — 인식기만 갈아끼운다"는
        # 바구니도 물체와 같은 approach.py 시각 서보 루프(ArUco 관측기로
        # 교체)로 접근해야 한다고 본다 — 즉 여기도 궁극적으로 drive_to가
        # 아니라 base.approach()가 될 후보다. 다만 어느 ArUco 마커 ID가
        # 어느 Destination(LEFT/RIGHT)인지가 아직 실기로 확인 안 됐다
        # (HANDOFF.md는 "빨강=4, 파랑=5" 예시만 남겼고 지금은 색이 아니라
        # 좌우 모델이다) — 확정 전까지 drive_to를 그대로 둔다.
        arrived = ports.base.drive_to(box.pose_m)
        if not arrived:
            return ScanState(self.ctx.hold(self.target.track_id))

        # 부호는 정렬 방향일 뿐이라 크기만 본다. 무한대(정렬 실패)는 어떤 상한과
        # 비교해도 여기서 걸린다.
        yaw_error_rad = ports.base.align_to_box(box)
        if abs(yaw_error_rad) > ALIGN_TOLERANCE_RAD:
            return ScanState(self.ctx.hold(self.target.track_id))

        return PosePlanState(self.ctx, self.target, box)


class PosePlanState(State):
    """⏸ 보류 — 대상 클래스 미정 (긴 물체 제외로 실행할 물체가 없음).
    구조는 유지하되 현재 전 클래스가 φ=0으로 통과한다 (state_machine.md §2).

    class_diagram.md §3은 이 상태의 필드로 `ctx` 하나만 나열하지만, 실제로는
    `measure_opening(box)` 호출과 `dims_m` 참조에 `target`/`box` 가 필요하다 —
    다이어그램이 이 보류 구간을 축약해 둔 것으로 보고 두 필드를 추가했다."""

    name = "POSE_PLAN"

    def __init__(self, ctx, target, box):
        self.ctx = ctx
        self.target = target
        self.box = box

    def execute(self, ports):
        opening_mm = ports.perception.measure_opening(self.box)
        if opening_mm is None:
            return RejectState(self.ctx, self.target, "입구 폭 실측 실패")

        phi_rad = self._solve_phi(self.target.dims_m, opening_mm)
        if phi_rad is None:
            return RejectState(self.ctx, self.target, "φ 해 구간 없음")
        return InsertState(self.ctx, self.target, self.box, phi_rad)

    def _solve_phi(self, dims_m, opening_mm):
        """⏸ 보류. 실제 판정식(sequences.md §3):

            H_proj(φ) = L·|cos φ| + w·|sin φ| ≤ W_open − margin
            (φ = 장축과 수평면 사이 각도, rad. L=dims_m 장축, w=단축,
             W_open=opening_mm, margin=미결 #7)

        재도입 전까지는 전 클래스가 φ=0(눕힌 채)으로 통과한다 — 해 없음(None)
        경로는 구조만 살아 있고 아직 도달하지 않는다."""
        return 0.0


class InsertState(State):
    """상자 투입. 자세 전환은 반드시 정지 상태에서 한다 — 주행 중 전환 시
    무게중심 이탈로 전복 위험이 있다 (sequences.md §3).

    `monitor_clearance()` 를 시퀀스 다이어그램처럼 삽입 동작 중 반복 폴링하지
    않고 동작 직전 1회만 확인한다 — 도메인 계층은 동기 상태 전이 모델이라
    모션 중 폴링 루프는 표현할 수 없고, 그건 실제 어댑터의 액션 실행 내부
    (모션 중 안전 감시)가 담당할 몫으로 본다."""

    name = "INSERT"

    def __init__(self, ctx, target, box, phi_rad):
        self.ctx = ctx
        self.target = target
        self.box = box
        self.phi_rad = phi_rad

    def execute(self, ports):
        ports.base.stop()
        clearance = ports.perception.monitor_clearance()
        if clearance.contact_risk:
            return RejectState(self.ctx, self.target, "투입 중 접촉 위험")

        # 바구니에서는 바닥 파지 높이로 내려가지 않는다. CARRY_IDLE에서 검증된
        # 베이스가 정지·정렬된 CARRY_IDLE에서 실측 DROP_195로 직접 전개한 뒤
        # 그리퍼를 열어 낙하시킨다.
        # 가장 낮은 파지점(나이트)도 120 mm 테두리 위로 통과한다.
        plan = select_horizontal_grasp_plan(self.target)
        if not ports.arm.move_to_floor_pose(plan.profile, "drop"):
            return EstopState()
        ports.arm.set_gripper(OPEN_MM)
        if not ports.arm.move_to_floor_pose(plan.profile, "idle"):
            return EstopState()
        # 2026-08-24 사용자 결정(시연 범위 축소): 투입에 성공하면 SCAN으로
        # 돌아가지 않고 **물체 하나로 시연을 끝낸다**. 원래는 여기가
        # `ScanState(...)`라 바닥이 빌 때까지 반복하는 루프 FSM이었다.
        #
        # `IdleState`를 돌려주지 않는 이유: IdleState.execute()는 절대 `None`을
        # 반환하지 않는다 — ctx가 비면 자기 자신을, raw_text가 남아 있으면
        # 다시 파싱해 ScanState를 돌려준다. 전자는 MissionTask.run()의
        # `while state is not None` 루프가 IDLE만 무한히 yield하고, 후자는
        # 미션이 통째로 재시작된다. `DoneState`가 `None`을 반환해 미션
        # 제너레이터를 끝내면, mission_orchestrator_node의 FSM 스레드가
        # `self._command_queue.get()`으로 돌아가 다음 명령을 기다린다 —
        # 그게 실제로 원하는 "시연 종료 후 대기" 상태다.
        #
        # 여러 물체를 이어서 처리하는 원래 동작으로 되돌리려면 이 한 줄을
        # `ScanState(...)`로 바꾸면 된다(다른 상태는 손댈 필요 없다).
        return DoneState(self.ctx.complete(self.target.track_id))


class DeliverState(State):
    name = "DELIVER"

    def __init__(self, ctx, target):
        self.ctx = ctx
        self.target = target

    def execute(self, ports):
        arrived = ports.base.drive_to(DELIVERY_POINT_M)
        if not arrived:
            return ScanState(self.ctx.hold(self.target.track_id))
        return HandoverState(self.ctx, self.target)


class HandoverState(State):
    """class_diagram.md §3은 `ctx` 만 나열하지만, `done_ids` 등록에 `target.track_id`
    가 필요해 `target` 을 추가했다 (PosePlanState와 같은 사유)."""

    name = "HANDOVER"

    def __init__(self, ctx, target):
        self.ctx = ctx
        self.target = target

    def execute(self, ports):
        ports.arm.move_to_cartesian(HANDOVER_POINT_M)
        ports.arm.set_gripper(OPEN_MM)
        load_ratio = ports.arm.get_load()
        if load_ratio > HANDOVER_LOAD_THRESHOLD:
            # 아직 사람이 받아가지 않았다 — 재시도가 아니라 대기.
            return self
        return ScanState(self.ctx.complete(self.target.track_id))


class RejectState(State):
    """투입 불가 판정 — 유즈케이스 2 그 자체. 물체를 든 채 미션을 끝내지
    않는다: 내려놓고 보류 등록 후 SCAN 복귀한다 (sequences.md §3)."""

    name = "REJECT"

    def __init__(self, ctx, target, reason):
        self.ctx = ctx
        self.target = target
        self.reason = reason

    def execute(self, ports):
        ports.arm.move_to_cartesian(PUT_DOWN_POINT_M, down=True)
        ports.arm.set_gripper(OPEN_MM)
        return ScanState(self.ctx.hold(self.target.track_id))


class DoneState(State):
    name = "DONE"

    def __init__(self, ctx):
        self.ctx = ctx

    def execute(self, ports):
        return None


class EstopState(State):
    """정상 전이가 아니라 인터럽트 — state_machine.md §2, MissionTask.run() 참고.
    팔 자세를 래치해 낙하를 막는다 (hld.md §6.4 #6 갭이 여기서 해소된다)."""

    name = "ESTOP"

    def execute(self, ports):
        ports.base.stop()
        ports.arm.hold_position()
        return None
