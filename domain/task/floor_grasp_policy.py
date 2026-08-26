"""실측 수평 파지 프로필 선택 정책.

YOLO subtype이 아직 없으므로 현재는 검출 bounding-box의 바닥면 폭으로
검증된 물체 프로필을 고른다. 분류가 추가되면 이 휴리스틱을 명시 subtype으로
교체하되 ArmDriver의 profile 계약은 유지한다.
"""

from dataclasses import dataclass

from domain.values import Detection, ObjectClass

# ros2_ws/src/grippers_arm/grippers_arm/gripper_calibration.py의 GRIPPER_OPEN_MM
# 실측값과 같은 수다. domain 계층은 그 ROS 패키지를 import하지 않으므로(계층
# 분리) 값만 그대로 복제해 둔다 — 그리퍼 기구가 바뀌어 GRIPPER_OPEN_MM이
# 바뀌면 여기도 같이 바꿔야 한다. 2026-08-24: 사용자 지시로 80.0(임의로 절반쯤
# 열기)에서 이 안전 최대치로 올림.
GRIPPER_MAX_SAFE_OPEN_MM = 168.0

# 투하 시 물체 폭에 더할 여유 (mm). ros2_ws 쪽 GRIPPER_RELEASE_MM과 같은 수를
# 계층 분리 때문에 복제해 둔다 — 한쪽을 바꾸면 다른 쪽도 바꿔야 한다.
#
# 2026-08-25 사용자 지시: "물체를 놓을 때 완전히 벌리지 말고 물체가 그리퍼
# 사이에서 나올 정도로만 벌려." GRIPPER_MAX_SAFE_OPEN_MM(168)까지 열면
# 손가락 판이 바구니 위로 넓게 쓸릴 뿐 얻는 것이 없다.
GRIPPER_RELEASE_MM = 15.0


# ros2_ws 쪽 GRIPPER_SQUEEZE_MM · GRIPPER_GRASP_MIN_MM와 같은 수를 계층 분리
# 때문에 복제해 둔다 — 한쪽을 바꾸면 다른 쪽도 바꿔야 한다.
#
# ⚠️ 2026-08-26: 실제로 그 동기화가 깨져 있었다. 이 파일이 파지 폭을
# 13.0/13.0/15.0/30.0/35.0으로 **하드코딩**하고 있었는데, ros2 쪽은
# _close_width(물체폭 - 15.0, 하한 7.0)로 7.0/7.0/9.5/25.0/31.0을 낸다.
# 미션이 실제로 쓰는 것은 이 파일 값이라, 08-25에 하한을 9.0 -> 7.0으로
# 내리며 "최대한 세게 잡자"고 한 사용자 지시가 도메인 경로에는 반영되지
# 않은 채였다. 계산식을 옮겨 와 다시는 어긋나지 않게 한다.
GRIPPER_SQUEEZE_MM = 15.0
GRIPPER_GRASP_MIN_MM = 7.0


def _close_width(object_width_mm: float) -> float:
    """물체 폭에서 GRIPPER_SQUEEZE_MM만큼 더 좁힌 목표 폭.

    빈 닫힘 폭이 아니라 **파지 전용** 하한으로 clamp한다 — 물체가 턱을 멈춰
    주므로 파지 때는 더 좁게 명령해 위치 오차(=힘)를 키울 수 있다."""
    return max(GRIPPER_GRASP_MIN_MM, round(object_width_mm - GRIPPER_SQUEEZE_MM, 1))


def _release_width(object_width_mm: float) -> float:
    """물체가 턱 사이에서 빠져나올 만큼만 벌린 목표 폭."""
    return min(GRIPPER_MAX_SAFE_OPEN_MM, round(object_width_mm + GRIPPER_RELEASE_MM, 1))


@dataclass(frozen=True)
class HorizontalGraspPlan:
    profile: str
    preopen_width_mm: float
    close_width_mm: float
    release_width_mm: float


def select_horizontal_grasp_plan(target: Detection) -> HorizontalGraspPlan:
    widths_mm = sorted((target.dims_m.x * 1000.0, target.dims_m.y * 1000.0))
    narrow_mm, wide_mm = widths_mm

    if target.cls is ObjectClass.GABE:
        if wide_mm <= 42.0:
            return HorizontalGraspPlan(
                "cube", GRIPPER_MAX_SAFE_OPEN_MM, _close_width(40.0), _release_width(40.0))
        # 별기둥과 축구공은 같은 20 mm 자세와 35 mm 닫힘값으로 검증됐다.
        return HorizontalGraspPlan(
            "soccer_polyhedron", GRIPPER_MAX_SAFE_OPEN_MM, _close_width(46.0),
            _release_width(46.0))

    # 체스말 subtype이 없는 동안 실측 폭에 가장 가까운 프로필을 쓴다.
    chess = (
        (17.0, "chess_queen", _close_width(17.0)),
        (22.0, "chess_knight", _close_width(22.0)),
        (24.5, "chess_rook", _close_width(24.5)),
    )
    width_mm, profile, close_mm = min(chess, key=lambda item: abs(narrow_mm - item[0]))
    return HorizontalGraspPlan(
        profile, GRIPPER_MAX_SAFE_OPEN_MM, close_mm, _release_width(width_mm))


# HANDOFF.md(2026-08-23)의 시각 서보 접근 루프(tools/perception/approach.py)는
# 교시값을 raw YOLO 클래스 이름별로 저장한다(approach_target_<raw class>.json).
# 그런데 domain.values.Detection에는 YOLO subtype이 없다 — select_horizontal_
# grasp_plan()이 폭 휴리스틱을 쓰는 것과 같은 이유(위 주석 참고)다. 새 wire
# 필드를 추가하는 대신 이미 검증된 같은 휴리스틱을 재사용한다.
_PROFILE_TO_RAW_CLASS = {
    "chess_rook": "rook",
    "chess_knight": "knight",
    "chess_queen": "queen",
    "cube": "box",
    # "soccer_polyhedron"은 star/soccer 둘 다를 가리켜 폭만으로는 못 가른다 —
    # 모르면 실패 관례대로 매핑하지 않는다(아래 approach_target_key 참고).
}


def approach_target_key(target: Detection) -> str | None:
    """target을 tools/perception/approach.py의 --cls(=교시 파일 키)로 바꾼다.

    체스 기물 3종은 select_horizontal_grasp_plan과 같은 폭 휴리스틱으로
    raw 클래스 이름과 정확히 대응된다. GABE는 cube(≈box)만 갈리고
    star/soccer는 폭이 겹쳐 구분할 수 없다 — 이 경우 **`None`**을 돌려준다.
    호출자(real adapter)는 이걸 "정밀 접근 불가"로 다뤄야 한다."""
    plan = select_horizontal_grasp_plan(target)
    return _PROFILE_TO_RAW_CLASS.get(plan.profile)
