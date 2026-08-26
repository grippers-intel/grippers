from domain.task.floor_grasp_policy import approach_target_key, select_horizontal_grasp_plan
from domain.values import Detection, ObjectClass, Point3


def detection(cls, width_mm):
    return Detection(
        track_id=1,
        cls=cls,
        pose_m=Point3(0.24, 0.0, 0.02),
        dims_m=Point3(width_mm / 1000.0, width_mm / 1000.0, 0.04),
        yaw_rad=0.0,
        confidence=0.9,
    )


def test_gabe_policy_uses_measured_cube_and_wide_object_commands():
    cube = select_horizontal_grasp_plan(detection(ObjectClass.GABE, 40.0))
    wide = select_horizontal_grasp_plan(detection(ObjectClass.GABE, 46.0))

    assert (cube.profile, cube.preopen_width_mm, cube.close_width_mm) == (
        "cube",
        168.0,
        25.0,
    )
    assert (wide.profile, wide.preopen_width_mm, wide.close_width_mm) == (
        "soccer_polyhedron",
        168.0,
        31.0,
    )


def test_chess_policy_chooses_nearest_measured_grasp_width():
    assert select_horizontal_grasp_plan(detection(ObjectClass.CHESS_PIECE, 17.0)).profile == (
        "chess_queen"
    )
    assert select_horizontal_grasp_plan(detection(ObjectClass.CHESS_PIECE, 22.0)).profile == (
        "chess_knight"
    )
    rook = select_horizontal_grasp_plan(detection(ObjectClass.CHESS_PIECE, 24.5))
    assert (rook.profile, rook.close_width_mm) == ("chess_rook", 9.5)


def test_approach_target_key_resolves_chess_pieces_by_measured_width():
    """체스 3종은 select_horizontal_grasp_plan과 정확히 같은 폭 휴리스틱으로
    tools/perception/approach.py의 교시 파일 키(raw YOLO 클래스 이름)와
    1:1 대응된다 — HANDOFF.md 검증 케이스(chess_rook)가 이 경로다."""
    assert approach_target_key(detection(ObjectClass.CHESS_PIECE, 17.0)) == "queen"
    assert approach_target_key(detection(ObjectClass.CHESS_PIECE, 22.0)) == "knight"
    assert approach_target_key(detection(ObjectClass.CHESS_PIECE, 24.5)) == "rook"


def test_approach_target_key_resolves_cube_but_not_ambiguous_gabe():
    """GABE는 cube(≈box)만 폭으로 갈리고, star/soccer는 폭이 겹쳐(둘 다
    soccer_polyhedron 프로필) raw 클래스를 특정할 수 없다 — 모르면 실패
    관례대로 None."""
    assert approach_target_key(detection(ObjectClass.GABE, 40.0)) == "box"
    assert approach_target_key(detection(ObjectClass.GABE, 46.0)) is None
