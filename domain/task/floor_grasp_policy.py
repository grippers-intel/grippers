"""실측 수평 파지 프로필 선택 정책.

YOLO subtype이 아직 없으므로 현재는 검출 bounding-box의 바닥면 폭으로
검증된 물체 프로필을 고른다. 분류가 추가되면 이 휴리스틱을 명시 subtype으로
교체하되 ArmDriver의 profile 계약은 유지한다.
"""

from dataclasses import dataclass

from domain.values import Detection, ObjectClass


@dataclass(frozen=True)
class HorizontalGraspPlan:
    profile: str
    preopen_width_mm: float
    close_width_mm: float


def select_horizontal_grasp_plan(target: Detection) -> HorizontalGraspPlan:
    widths_mm = sorted((target.dims_m.x * 1000.0, target.dims_m.y * 1000.0))
    narrow_mm, wide_mm = widths_mm

    if target.cls is ObjectClass.GABE:
        if wide_mm <= 42.0:
            return HorizontalGraspPlan("cube", 80.0, 30.0)
        # 별기둥과 축구공은 같은 20 mm 자세와 35 mm 닫힘값으로 검증됐다.
        return HorizontalGraspPlan("soccer_polyhedron", 80.0, 35.0)

    # 체스말 subtype이 없는 동안 실측 폭에 가장 가까운 프로필을 쓴다.
    chess = (
        (17.0, "chess_queen", 13.0),
        (22.0, "chess_knight", 13.0),
        (24.5, "chess_rook", 15.0),
    )
    _, profile, close_mm = min(chess, key=lambda item: abs(narrow_mm - item[0]))
    return HorizontalGraspPlan(profile, 80.0, close_mm)
