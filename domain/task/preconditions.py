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
  E-STOP·정지   — 지금 움직이고 있지는 않은가

⚠️ 2026-09-04까지는 라이다(바구니 정면 거리·각도)도 이 목록에 있었다.
사용자 지시로 뺐다 — Host가 좌표를 전부 소유한다는 원칙을 INSERT까지
끝까지 적용해, "정말 도착했는가"를 Pi가 라이다로 다시 확인하는 이중
판정을 없앴다. 이제 INSERT 전환은 Pi 자기 상태(정지·파지 확인·부하
안정성)만 본다 — 위치가 맞는지는 Host의 몫이고 Host의 ArUco 하드스톱
(mission_config.BASKET_HARD_STOP_MARGIN_M)이 근접 안전판으로 남는다.

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
    """INSERT 판정에 필요한 관측값 묶음.

    ⚠️ 2026-09-04 사용자 지시로 라이다 기반 항목(바구니 정면 거리·yaw·좌우
    오프셋·점 개수·판독 안정성)을 전부 뺐다. Host가 좌표·경로·정지 위치를
    전부 소유한다는 원칙(baseline_ports.py)을 INSERT까지 끝까지 적용한
    것이다 — Pi가 라이다로 "정말 도착했는가"를 다시 확인하던 이중 판정을
    없애고, Host가 INSERT를 보내면 Pi는 자기 상태(정지·파지 확인·부하
    안정성)만 보고 그대로 실행한다. 위치가 틀렸을 때의 안전판은 이제
    Host 쪽 ArUco 하드스톱(mission_config.BASKET_HARD_STOP_MARGIN_M)뿐이다
    — Pi는 더 이상 걸러내지 않는다."""

    estop_set: bool
    base_stopped: bool
    gripper_load: float
    # GRASP가 CARRY 도달 시점에 이미 끝낸 "정말 물었는가" 판정(부하 OR
    # 뎁스 "사라짐")을 그대로 넘겨받은 값. 2026-09-03 실기(box)까지는 여기서
    # gripper_load를 다시 문턱과 비교해 "비어 있다"를 판정했는데, box는
    # 파지에 성공해도 부하가 계속 0에 가깝게 읽혀서(그리퍼가 정착하면
    # 실제로 물고 있어도 능동 토크를 안 낸다 — baseline_mission.py의
    # BaselineGraspState 주석 참고) 이 게이트가 영원히 막혔다. 이제는 그때
    # 끝난 판정을 다시 재지 않고 그대로 믿는다 — gripper_load는 여기 아래
    # 부하 안정성(직전 사이클 대비 변화량)에만 쓴다.
    grasp_confirmed: bool
    profile: str | None = None
    # 직전 사이클과 비교한 부하 변화. None이면 비교할 이전 표본이 없다는
    # 뜻이다 — 팔을 펼치기 전 미끄러짐만 잡는, 라이다와 무관한 독립 신호다.
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

    ⚠️ 2026-09-04 사용자 지시로 바구니 위치 판정(라이다 거리·yaw·좌우
    오프셋·점 개수·판독 안정성)을 전부 뺐다 — "여기서 넣어라"는 전적으로
    Host의 판단이고, Pi는 자기 상태만 보고 그대로 따른다. 남은 조건은
    전부 "지금 팔을 펼쳐도 안전한가"이지 "바구니가 정말 거기 있는가"가
    아니다 — 그 판단은 Host가 하고, 틀렸을 때의 안전판도 이제 Host 쪽
    (ArUco 하드스톱)에만 있다. 라이다 기반 판정 이력은 이 파일의 git
    이력 및 `error_budget.md`/`sequences.md` 구판에 남아 있다."""
    reasons = []

    if inputs.estop_set:
        reasons.append("E-STOP이 걸려 있다")

    if not inputs.base_stopped:
        reasons.append("차체가 아직 정지하지 않았다")

    if not inputs.grasp_confirmed:
        # 빈손으로 투하 자세를 펼쳐 봐야 얻을 것이 없고, 팔만 위험하게 뻗는다.
        # 2026-09-03부터: 여기서 raw 부하를 문턱과 다시 비교하지 않는다 —
        # GRASP가 CARRY 진입 때 이미 내린 판정(grasp_confirmed 필드 설명
        # 참고)을 믿는다. 현재 부하는 참고용으로만 같이 보고한다.
        reasons.append(
            f"그리퍼가 비어 있다 (파지 판정 때 확인되지 않았다 — "
            f"현재 부하 {inputs.gripper_load:.4f})")

    if inputs.profile is None:
        reasons.append("무엇을 들고 있는지 모른다 — 놓기 폭을 정할 수 없다")

    # 부하 안정성 — 팔을 펼치기 전에 미끄러짐을 잡는다. 라이다와 무관한
    # 독립 신호라 위치 판정을 빼도 그대로 남는다.
    if inputs.load_change is not None and inputs.load_change < -bc.GRIPPER_SLIP_LOAD_DROP:
        reasons.append(
            f"그리퍼 부하가 떨어지고 있다 ({inputs.load_change:+.4f}) — "
            "물체가 미끄러지는 중일 수 있다")

    return PreconditionReport(not reasons, tuple(reasons))
