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

from domain.task.state import State
from domain.values import MissionContext, MissionMode, Point3, Pose2D

# ── 미실측 상수 ──────────────────────────────────────────────────────────
# 전부 TODO. 하드코딩된 좌표/임계값은 여기 한 곳에 모아 두고, 실측이 끝나면
# 이 블록만 교체하면 되게 한다. 값 자체는 자리 표시자이며 검증되지 않았다.
OPEN_MM = 90.0  # TODO: 미결 #4 (엔드이펙터 개구 폭 실측) 결과로 교체
CLOSED_MM = 0.0  # TODO: 미결 #4 결과로 교체
GRASP_APPROACH_HEIGHT_M = 0.10  # TODO: 실측 — 파지 하강 전 안전 여유 높이
INSERT_DROP_HEIGHT_M = 0.05  # TODO: 실측 — 상자 입구 상단에서 투입 낙차 높이
HANDOVER_POINT_M = Point3(x=0.30, y=0.0, z=0.35)  # TODO: 실측 — 사용자 인계 손끝 위치
HANDOVER_LOAD_THRESHOLD = 0.05  # TODO: 실측 — '사람이 받아감' 판정 부하 임계값
PUT_DOWN_POINT_M = Point3(x=0.30, y=0.0, z=0.0)  # TODO: 실측 — REJECT 시 안전한 내려놓기 위치
DELIVERY_POINT_M = Pose2D(x=0.0, y=0.0, theta=0.0)  # TODO: 실측 — FETCH 인계 접근 위치


class IdleState(State):
    """`ctx.spec.raw_text` 를 `interpreter.parse()` 로 (재)해석해 `SCAN` 으로 넘어간다.
    `ctx` 가 없거나 `raw_text` 가 비어 있으면 대기(자기 자신 반환)한다.

    TODO(#7): `Ports` 에 아직 `interpreter` 필드가 없다 — 마이그레이션 순서
    (class_diagram.md §5: 1→2·3·4·5→8→6→7→9→10)상 `mission_task.py` 의
    `Ports` 갱신(#7)이 이 재작성(#6) 다음이라, `raw_text` 가 있는 `ctx` 로
    진입하면 `#7` 이 끝나기 전까지 `ports.interpreter` 에서 AttributeError가 난다."""

    name = "IDLE"

    def __init__(self, ctx=None):
        self.ctx = ctx

    def execute(self, ports):
        if self.ctx is None or not self.ctx.spec.raw_text:
            return self
        spec = ports.interpreter.parse(self.ctx.spec.raw_text)
        return ScanState(MissionContext(spec=spec))


class ScanState(State):
    """루프의 유일한 재진입점. 무한 루프 방지 2종을 모두 구현한다
    (state_machine.md §4):

    1. `MAX_RESCAN` — 빈 관측이면 바로 `DONE` 이 아니라 재스캔을 먼저 시도한다
       (일시적 오검출 대비). 재시도가 소진되면 `DONE`.
    2. **SCAN 무변화 감지** — 연속 2회 스캔 결과(비어있지 않은)가 동일하면
       진전이 없다고 보고 `DONE`. `done_ids`/`held_ids` 필터링이 이미
       `SELECT` 에서 무한 루프를 막지만, 이건 그 필터링 자체가 깨졌을 때의
       2차 방어선이다."""

    name = "SCAN"
    MAX_RESCAN = 3

    def __init__(self, ctx, rescans=0, last_scan=None):
        self.ctx = ctx
        self.rescans = rescans
        self.last_scan = last_scan

    def execute(self, ports):
        detections = ports.perception.scan_floor()

        if not detections:
            if self.rescans < self.MAX_RESCAN:
                return ScanState(self.ctx, self.rescans + 1, detections)
            return DoneState(self.ctx)

        if detections == self.last_scan:
            return DoneState(self.ctx)

        return SelectState(self.ctx, detections)


class SelectState(State):
    """순수 판단 상태 — 포트를 호출하지 않는다. 선정 기준(state_machine.md §3):

    1. 보류/완료 목록에 없을 것
    2. `placement_rule` 에 목적지가 정의되어 있을 것
    3. (FETCH 모드) `spec.target_cls` 와 일치할 것
    4. 위 조건을 만족하는 것 중 base_link 로부터 최단 거리

    사전 필터(치수만으로 φ 해 없음을 미리 거르는 것)는 의도적으로 넣지 않는다 —
    그러면 유즈케이스 2(투입 불가 판정)가 축소된다."""

    name = "SELECT"

    def __init__(self, ctx, detections):
        self.ctx = ctx
        self.detections = detections

    def execute(self, ports):
        target = self._pick(self.detections)
        if target is None:
            return DoneState(self.ctx)
        return ApproachState(self.ctx, target)

    def _pick(self, detections):
        spec = self.ctx.spec
        candidates = [
            d
            for d in detections
            if d.track_id not in self.ctx.done_ids
            and d.track_id not in self.ctx.held_ids
            and d.cls in spec.placement_rule
            and (spec.mode is not MissionMode.FETCH or d.cls == spec.target_cls)
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda d: (d.pose_m.x**2 + d.pose_m.y**2) ** 0.5)


class ApproachState(State):
    name = "APPROACH"

    def __init__(self, ctx, target):
        self.ctx = ctx
        self.target = target

    def execute(self, ports):
        # TODO: 실측 — 접근 지점은 target 위치 그대로가 아니라 그리퍼 스탠드오프를
        # 반영해야 한다. 실측 전까지는 검출 pose를 그대로 목표로 쓴다.
        approach = Pose2D(x=self.target.pose_m.x, y=self.target.pose_m.y, theta=self.target.yaw_rad)
        arrived = ports.base.drive_to(approach)
        if not arrived:
            return ScanState(self.ctx.hold(self.target.track_id))
        return GraspState(self.ctx, self.target)


class GraspState(State):
    """부하 기반 파지 검증 — 힘/토크 센서 없이 서보 부하값으로 판정한다
    (sequences.md §2). 재시도는 상태 변경이 아니라 새 인스턴스 반환으로
    표현한다 (기존 코드 관례 유지).

    NOTE: `grasp_attempts` 는 state_machine.md §3 예시 코드를 그대로 따라
    여기서 리셋하지 않는다 — 즉 미션 전체 누적 실패 횟수다. 대상별 재시도
    예산이 의도라면(sequences.md §2 "loop attempt ≤ MAX_GRASP_RETRY"는
    대상 1개 기준으로 읽힌다) `MissionContext` 에 리셋 메서드가 필요한데,
    현재 `complete()`/`hold()`/`retry()` 3개뿐이라 이 PR 범위에서는 임의로
    추가하지 않았다 — 사용자 확인 필요 (커밋 메시지 참고)."""

    name = "GRASP"
    MAX_GRASP_RETRY = 3
    LOAD_THRESHOLD = 0.15

    def __init__(self, ctx, target):
        self.ctx = ctx
        self.target = target

    def execute(self, ports):
        approach_point = Point3(
            x=self.target.pose_m.x,
            y=self.target.pose_m.y,
            z=self.target.pose_m.z + GRASP_APPROACH_HEIGHT_M,
        )
        ports.arm.move_to_cartesian(approach_point)
        ports.arm.move_to_cartesian(self.target.pose_m, down=True)
        ports.arm.set_gripper(CLOSED_MM)

        if ports.arm.get_load() >= self.LOAD_THRESHOLD:
            if self.ctx.spec.mode is MissionMode.TIDY:
                return TransportState(self.ctx, self.target)
            return DeliverState(self.ctx, self.target)

        # 빈손 — 그리퍼가 끝까지 닫힘.
        ports.arm.set_gripper(OPEN_MM)
        if self.ctx.grasp_attempts >= self.MAX_GRASP_RETRY:
            return ScanState(self.ctx.hold(self.target.track_id))

        # 실패한 파지가 물체를 밀었을 수 있다 — 이전 pose 재사용은 같은 실패를
        # 반복하므로 재스캔해서 같은 track_id의 갱신된 pose로 재시도한다.
        refreshed = ports.perception.scan_floor()
        updated = next(
            (d for d in refreshed if d.track_id == self.target.track_id), self.target
        )
        return GraspState(self.ctx.retry(), updated)


class TransportState(State):
    name = "TRANSPORT"

    def __init__(self, ctx, target):
        self.ctx = ctx
        self.target = target

    def execute(self, ports):
        color = self.ctx.spec.placement_rule[self.target.cls]
        box = ports.perception.find_box(color)
        if box is None:
            return ScanState(self.ctx.hold(self.target.track_id))

        # TODO: 실측 — 상자 "앞" 지점은 box.pose_m 그대로가 아니라 접근 오프셋이
        # 필요하다. 실측 전까지는 상자 관측 pose를 그대로 목표로 쓴다.
        arrived = ports.base.drive_to(box.pose_m)
        if not arrived:
            return ScanState(self.ctx.hold(self.target.track_id))

        ports.base.align_to_box(box)
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
        settled = ports.arm.reorient(self.phi_rad)
        if not settled:
            return RejectState(self.ctx, self.target, "자세 정착 실패")

        clearance = ports.perception.monitor_clearance()
        if clearance.contact_risk:
            return RejectState(self.ctx, self.target, "투입 중 접촉 위험")

        # TODO: 실측 — 실제 삽입 지점은 box.pose_m + 낙차 오프셋을 IK로 풀어야
        # 한다. 실측 전까지는 상자 위치 + 고정 낙차 높이를 그대로 쓴다.
        insert_point = Point3(
            x=self.box.pose_m.x, y=self.box.pose_m.y, z=INSERT_DROP_HEIGHT_M
        )
        ports.arm.move_to_cartesian(insert_point, down=True)
        ports.arm.set_gripper(OPEN_MM)
        ports.arm.fold_to_cradle()
        return ScanState(self.ctx.complete(self.target.track_id))


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
