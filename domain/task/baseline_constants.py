"""시연 baseline 미션의 수치 상수 — 실측 완료분과 TODO를 한곳에 모은다.

사용자가 2026-08-25에 정리한 전체 흐름을 그대로 구현하기 위한 값들이다.
`domain/task/states.py`의 상수와 겹치는 것이 있지만 **일부러 따로 둔다** —
기존 FSM과 로직이 충돌하는 부분이 있어 baseline을 독립적으로 굴려 보기
위해서다(사용자 지시).

값의 출처를 세 등급으로 구분한다. 등급을 섞어 쓰면 무엇이 검증된 값이고
무엇이 지어낸 값인지 알 수 없게 된다.

  실측  — 하드웨어로 재서 확인한 값. 그대로 믿어도 된다.
  지시  — 사용자가 정한 설계값. 실측은 아니지만 결정된 값이다.
  TODO  — **아직 모른다.** `None`으로 두어 쓰는 쪽이 반드시 걸리게 한다.

TODO를 0이나 그럴듯한 숫자로 채우지 않는 이유는 "모르면 실패"라는 이 저장소의
관례와 같다 — 지어낸 값으로 도는 것처럼 보이게 만들면 실기에서 원인을 못 찾는다.
"""

# ── 파지 정렬 (사용자 지시 5·6) ────────────────────────────────────────────
# APPROACH -> GRASP 전환 시점: 차체 전면 기준 물체까지 이 거리에 정렬됐을 때.
# Host가 오버헤드 이미지로 판정한다.
APPROACH_HANDOFF_FORWARD_MM = 190.0  # 지시

# GRASP 진입 후 팔을 내리기 전에 베이스가 곧장 전진하는 거리.
GRASP_CREEP_FORWARD_MM = 100.0  # 지시

# 교시 파지 자세가 전제하는 물체 중심 위치(차체 전면 기준 전방).
# floor_grasp_profiles.GRASP_OBJECT_CENTER_FORWARD_MM과 같은 값이다.
GRASP_OBJECT_CENTER_FORWARD_MM = 190.0  # 실측 2026-08-20


def creep_end_forward_mm() -> float:
    """미세 전진이 끝난 뒤 물체가 차체 전면에서 얼마나 앞에 있게 되는가."""
    return APPROACH_HANDOFF_FORWARD_MM - GRASP_CREEP_FORWARD_MM


def grasp_alignment_conflict_mm() -> float:
    """미세 전진 종료 지점과 교시 자세 전제의 차이. 0이어야 맞다.

    ⚠️ 지금 100mm 어긋나 있다. 190mm에 정렬한 뒤 100mm를 더 전진하면 물체는
    차체 전면 90mm 앞에 오는데, 교시 파지 자세는 190mm를 전제로 실측된 것이다
    (floor_grasp_profiles.py 2026-08-20). 셋 중 하나여야 한다:

      (a) 핸드오프를 290mm로 올린다  — 전진 후 190mm가 되어 교시 자세와 맞는다
      (b) 미세 전진을 0으로 한다     — 190mm에서 곧장 파지
      (c) 90mm용 교시 자세를 새로 뜬다

    BASELINE_MISSION_TODO.md 1번 항목이다. 결정 전까지는 이 함수가 0이 아닌
    값을 돌려주고, 테스트가 그 사실을 못 박는다."""
    return creep_end_forward_mm() - GRASP_OBJECT_CENTER_FORWARD_MM


# ── 차체 기하 ──────────────────────────────────────────────────────────────
# ArUco 마커(= Host가 보는 차량 위치) 중심에서 차체 전면까지.
# ⚠️ Host의 mission_config.py는 "실측 0.15"라 적고 Pi 인수인계서는 "미실측"이라
# 적는다. 두 문서가 상충하므로 실측 전까지 TODO로 둔다.
MARKER_TO_CHASSIS_FRONT_M = None  # TODO

# 라이다 원점에서 차체 전면까지. 라이다가 재는 거리는 차체 전면 기준이 아니라
# 라이다 기준이므로, 바구니 정지 판정에 이 값이 반드시 필요하다.
LIDAR_TO_CHASSIS_FRONT_M = None  # TODO

# 라이다 부착 높이(바닥 기준).
LIDAR_HEIGHT_M = 0.091  # 실측 2026-08-25 (사용자)

# 라이다 최소 측정 거리. 이보다 가까운 표면은 안 잡힌다.
# ⚠️ RPLidar A1급은 대략 0.15m다. 바구니 정지 판정이 이 하한 안쪽으로
# 들어가면 판정 자체가 불가능해지므로 실기로 확인해야 한다.
LIDAR_MIN_RANGE_M = None  # TODO

# ── 바구니 (사용자 지시 7) ─────────────────────────────────────────────────
# 바구니 입구 정면에서 차체 전면까지의 정지 거리.
BASKET_APPROACH_STANDOFF_M = 0.05  # 지시

# 바구니 테두리 높이.
# ⚠️ Pi의 floor_grasp_profiles.py(2026-08-20)는 약 0.115, Host의 config.py는
# BOX_H=0.220으로 적는다. 어느 쪽이든 라이다 평면(0.091)보다 높아 정면을
# 볼 수는 있지만, 0.115가 맞다면 여유가 24mm뿐이라 확인이 필요하다.
BASKET_RIM_HEIGHT_M = None  # TODO

# 투하 자세에서 그리퍼 중심이 차체 전면 앞으로 뻗는 거리.
BASKET_DROP_REACH_FORWARD_MM = 200.0  # 실측 2026-08-20

# ── 근접 감시 (사용자 지시 3) ──────────────────────────────────────────────
# 정면에 이보다 가까운 것이 잡히면 멈추고 미세 회피로 넘어간다.
# ⚠️ CPU YOLO 추론 지연을 재서 (지연 x 주행속도)만큼 여유를 더해야 한다.
PROXIMITY_STOP_DISTANCE_M = 0.25  # 지시(잠정)

# 미세 회피에서 옆으로 비키는 거리. 메카넘휠이라 옆걸음이 된다.
AVOID_LATERAL_STEP_M = None  # TODO

# 한 번의 APPROACH에서 미세 회피를 이만큼 넘게 하면 Host에 재계획을 맡긴다.
MAX_AVOID_STEPS = 3  # 지시

# ── 파지 판정 ──────────────────────────────────────────────────────────────
# states.GraspState.LOAD_THRESHOLD와 같은 값.
# ⚠️ 근거 실측이 2026-08-18(n=25, 빈 최대 0.031)인데 2026-08-25 재실측에서
# 자세별 빈 부하가 0.0235~0.0430으로 흔들렸다. 재실측 대상이다.
LOAD_THRESHOLD = 0.04  # 실측(낡음)

# 미션 한 번에 허용하는 파지 재시도 횟수.
MAX_GRASP_RETRY = 3  # 지시


def unresolved() -> dict:
    """아직 TODO인 상수 이름과 사유. 실기 투입 전 이 목록이 비어야 한다."""
    return {
        "MARKER_TO_CHASSIS_FRONT_M": "Host/Pi 문서가 상충 — 줄자 실측 필요",
        "LIDAR_TO_CHASSIS_FRONT_M": "미측정 — 바구니 정지 판정에 필수",
        "LIDAR_MIN_RANGE_M": "데이터시트 확인 + 실기 확인 필요",
        "BASKET_RIM_HEIGHT_M": "0.115 대 0.220 상충 — 줄자 실측 필요",
        "AVOID_LATERAL_STEP_M": "미세 회피 폭 미정 — 실기 조정",
    }
