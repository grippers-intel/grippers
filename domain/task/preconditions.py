"""Pi가 스스로 판단할 수 있는 전제 조건들 (팀 확정 임무 2번·4번, 2026-08-26).

Host가 "GRASP로 가라", "INSERT로 가라"고 지시하면 Pi는 곧장 실행하지 않고
**조건이 충족됐는지 먼저 판단해 보고한다.** 미충족이면 그 사실을 알리고
수정된 명령을 기다린다 — 스스로 고쳐서 진행하지 않는다.

## Pi가 판단할 수 있는 것과 없는 것

Pi에는 좌표가 없다. 그러므로 "물체 앞에 제대로 섰는가", "바구니 정면에
있는가" 같은 아레나 수준의 정렬은 **판단하지 않는다** — 그건 오버헤드로
차량과 물체를 동시에 보는 Host의 일이다.

Pi가 판단하는 것은 자기 센서로만 알 수 있는 것뿐이다:

  그리퍼 부하   — 지금 무언가를 물고 있는가, 비어 있는가
  자기 뎁스캠   — 내려가서 집을 물체가 정말 앞에 있는가, 무엇인가
  라이다        — 바구니 정면이 팔을 펼쳐도 되는 거리·각도에 있는가
  E-STOP·정지   — 지금 움직이고 있지는 않은가

이 구분이 흐려지면 같은 판정을 Host와 Pi가 각각 하게 되고, 둘이 어긋날 때
어느 쪽을 믿을지 정할 방법이 없어진다.

## 왜 이유를 목록으로 돌려주는가

`ok` 하나만 돌려주면 Host가 무엇을 고쳐야 할지 모른다. "수정된 명령을
기다린다"는 약속은 **무엇을 수정해야 하는지 알려줄 때만** 지킬 수 있다.
"""

from dataclasses import dataclass, field

from domain.task import baseline_constants as bc


@dataclass(frozen=True)
class PreconditionReport:
    """판정 결과. `ok=False`면 `reasons`에 미충족 항목이 사람이 읽을 수 있는
    문장으로 들어간다 — 그대로 Host 보고의 detail이 된다."""

    ok: bool
    reasons: tuple = ()
    detected_label: str | None = None

    @property
    def detail(self) -> str:
        return " / ".join(self.reasons)


@dataclass
class GraspInputs:
    """GRASP 판정에 필요한 관측값 묶음.

    포트를 직접 받지 않고 값으로 받는다 — 이 판정을 포트 더블 없이 순수
    단위 테스트로 고정할 수 있어야 하기 때문이다.

    ⚠️ 2026-09-01 사용자 지시로 원래 있던 다섯 항목 중 셋(estop_set·
    gripper_load·profile_known)을 뺐다 — 근거는 check_grasp() 문서 참고."""

    base_stopped: bool
    detected_label: str | None


@dataclass
class InsertInputs:
    """INSERT 판정에 필요한 관측값 묶음."""

    estop_set: bool
    base_stopped: bool
    gripper_load: float
    face_ok: bool
    face_distance_m: float
    face_yaw_error_rad: float
    face_reason: str = ""
    profile: str | None = None
    face_point_count: int = 0
    face_lateral_offset_m: float = 0.0
    face_lateral_known: bool = False
    # 직전 사이클과 비교한 값들. None이면 비교할 이전 표본이 없다는 뜻이다.
    distance_change_m: float | None = None
    load_change: float | None = None


def check_grasp(inputs: GraspInputs) -> PreconditionReport:
    """APPROACH -> GRASP 전환 조건 (임무 2번).

    ⚠️ 2026-09-01 사용자 지시로 넷에서 둘로 줄였다. 근거:
      - E-STOP: `BaselineMission.run()`이 사이클마다 최상위에서 먼저
        검사해 `ESTOP` 상태로 갈아치운다(baseline_mission.py 참고) — 이
        상태의 execute()가 도는 시점엔 이미 E-STOP이 아니라는 뜻이라,
        여기서 또 보는 것은 중복이었다. 게다가 하드웨어 배선이 아직
        안 돼 있어 이 필드는 사실상 값을 낼 방법이 없었다.
      - 그리퍼 부하(비어 있는가): 뭔가를 문 채 이 상태로 돌아오는
        경로가 없다는 전제로 뺐다 — CARRY가 아닌 한 그리퍼는 항상 비어
        있다.
      - 교시 자세 존재: `identify_target()`이 답하는 라벨은 여섯 개
        (`plan_for_label`이 아는 전부)뿐이라, 라벨이 잡히면 자세도 항상
        있다 — 이 조건은 한 번도 걸린 적이 없었다.

    남은 둘은 다르다. 차체 정지는 팔이 내려가는 동안 교시 자세의 전제가
    깨지는 걸 막고, 라벨 인식은 Pi 자기 눈으로 확인 못 한 채 내려가는
    것 자체를 막는다 — 둘 다 이 상태만 아는 것들이라 여기가 아니면
    아무 데서도 못 본다."""
    reasons = []

    if not inputs.base_stopped:
        # 팔이 내려가는 동안 차체가 움직이면 교시 자세의 전제가 깨진다.
        reasons.append("차체가 아직 정지하지 않았다")

    if inputs.detected_label is None:
        # 자기 뎁스캠이 목표를 못 봤다. Host는 오버헤드로 봤겠지만, 내려가는
        # 것은 이 팔이다 — 자기 눈으로 확인하지 못하면 내려가지 않는다.
        reasons.append("뎁스 카메라가 정면에서 목표를 찾지 못했다")

    return PreconditionReport(not reasons, tuple(reasons), inputs.detected_label)


def check_insert(inputs: InsertInputs) -> PreconditionReport:
    """INSERT 전환 조건 (임무 4번).

    INSERT는 팔을 크게 전개하는 동작이라 틀렸을 때의 비용이 이 미션에서
    가장 크다 — 물체가 바구니 밖으로 떨어지거나 테두리에 걸린다. 그래서
    **모르면 실패**를 가장 엄격하게 적용한다."""
    reasons = []

    if inputs.estop_set:
        reasons.append("E-STOP이 걸려 있다")

    if not inputs.base_stopped:
        reasons.append("차체가 아직 정지하지 않았다")

    if inputs.gripper_load < bc.LOAD_THRESHOLD:
        # 빈손으로 투하 자세를 펼쳐 봐야 얻을 것이 없고, 팔만 위험하게 뻗는다.
        reasons.append(
            f"그리퍼가 비어 있다 (부하 {inputs.gripper_load:.4f} < "
            f"{bc.LOAD_THRESHOLD:.4f})")

    if inputs.profile is None:
        reasons.append("무엇을 들고 있는지 모른다 — 놓기 폭을 정할 수 없다")

    if not inputs.face_ok:
        # 라이다가 바구니 정면을 못 잡았다. 거리 숫자는 의미가 없다.
        reasons.append(f"바구니 정면을 잡지 못했다 ({inputs.face_reason})")
        return PreconditionReport(False, tuple(reasons))

    upper = bc.BASKET_STOP_LIDAR_M + bc.BASKET_STOP_TOLERANCE_M
    if inputs.face_distance_m > upper:
        reasons.append(
            f"바구니가 멀다 (라이다 {inputs.face_distance_m:.3f}m > {upper:.3f}m)")
    elif inputs.face_distance_m < bc.BASKET_MIN_LIDAR_M:
        # 절벽 아래를 읽고 있다 — 우리가 교정한 그 면이 아닐 수 있다.
        reasons.append(
            f"라이다 판독이 하한보다 가깝다 ({inputs.face_distance_m:.3f}m < "
            f"{bc.BASKET_MIN_LIDAR_M:.3f}m) — 테두리를 넘겨보고 있을 수 있다")

    if abs(inputs.face_yaw_error_rad) > bc.BASKET_YAW_TOLERANCE_RAD:
        reasons.append(
            f"정렬이 틀어졌다 (yaw {inputs.face_yaw_error_rad:+.3f}rad > "
            f"{bc.BASKET_YAW_TOLERANCE_RAD:.3f}rad)")

    # ① 좌우 오프셋 — 거리와 yaw만으로는 안 보이는 오차다. 바구니와 나란한
    # 채로 옆으로 밀려 있으면 둘 다 정상인데 물체는 바깥에 떨어진다.
    # 모르는 경우(창을 양쪽 다 채움)는 통과시킨다 — 그 상황 자체가 "가장자리가
    # 창 밖에 있을 만큼 가운데"라는 뜻이다.
    if inputs.face_lateral_known and (
            abs(inputs.face_lateral_offset_m) > bc.BASKET_LATERAL_TOLERANCE_M):
        reasons.append(
            f"좌우로 밀려 있다 ({inputs.face_lateral_offset_m * 1000:+.0f}mm > "
            f"±{bc.BASKET_LATERAL_TOLERANCE_M * 1000:.0f}mm)")

    # ③ 점 개수 — 빔이 테두리를 스치기 시작하면 완전히 놓치기 전에 여기가
    # 먼저 준다.
    if inputs.face_point_count < bc.BASKET_MIN_FACE_POINTS:
        reasons.append(
            f"정면 점이 부족하다 ({inputs.face_point_count}개 < "
            f"{bc.BASKET_MIN_FACE_POINTS}개) — 테두리를 스치고 있을 수 있다")

    # ② 연속 판독 일치 — 한 프레임 튐과 "차체가 아직 안 멈춤"을 함께 잡는다.
    if inputs.distance_change_m is None:
        reasons.append("직전 판독이 없다 — 한 사이클 더 확인해야 한다")
    elif abs(inputs.distance_change_m) > bc.BASKET_STABILITY_TOLERANCE_M:
        reasons.append(
            f"판독이 흔들린다 ({inputs.distance_change_m * 1000:+.0f}mm) — "
            "아직 움직이는 중이거나 관측이 불안정하다")

    # ④ 부하 안정성 — 팔을 펼치기 전에 미끄러짐을 잡는다.
    if inputs.load_change is not None and inputs.load_change < -bc.GRIPPER_SLIP_LOAD_DROP:
        reasons.append(
            f"그리퍼 부하가 떨어지고 있다 ({inputs.load_change:+.4f}) — "
            "물체가 미끄러지는 중일 수 있다")

    return PreconditionReport(not reasons, tuple(reasons))
