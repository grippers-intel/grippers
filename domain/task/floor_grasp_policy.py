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
        # 임계 42 mm 는 실측 폭(정육면체 40 · 오각별 45 · 축구공 46)의 사이값이다.
        # 여유가 각각 2 · 3 · 4 mm 뿐이고 입력 dims_m 는 호모그래피 추정(A2 판정
        # 기준 ≤ 20 mm)이라, 추정이 몇 mm 만 흔들려도 프로필이 바뀐다. 뒤의 로드
        # 재검증이 잡아주지만 subtype 분류가 붙으면 이 휴리스틱을 걷어낸다.
        if wide_mm <= 42.0:
            return HorizontalGraspPlan("cube", 80.0, 30.0)
        # 오각별 기둥(45)과 축구공 다면체(46)는 같은 20 mm 자세·80/35 로 검증됐고,
        # 1 mm 차이는 위 추정 오차 안이라 폭으로 가를 수 없다. 하나로 묶는다 —
        # `star_column` 프로필은 형상 자료로만 남기고 정책은 선택하지 않는다.
        return HorizontalGraspPlan("soccer_polyhedron", 80.0, 35.0)

    # 체스말 subtype이 없는 동안 실측 폭에 가장 가까운 프로필을 쓴다.
    chess = (
        (17.0, "chess_queen", 13.0),
        (22.0, "chess_knight", 13.0),
        (24.5, "chess_rook", 15.0),
    )
    _, profile, close_mm = min(chess, key=lambda item: abs(narrow_mm - item[0]))
    return HorizontalGraspPlan(profile, 80.0, close_mm)
