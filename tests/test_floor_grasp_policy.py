from domain.task.floor_grasp_policy import select_horizontal_grasp_plan
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
        80.0,
        30.0,
    )
    assert (wide.profile, wide.preopen_width_mm, wide.close_width_mm) == (
        "soccer_polyhedron",
        80.0,
        35.0,
    )


def test_chess_policy_chooses_nearest_measured_grasp_width():
    assert select_horizontal_grasp_plan(detection(ObjectClass.CHESS_PIECE, 17.0)).profile == (
        "chess_queen"
    )
    assert select_horizontal_grasp_plan(detection(ObjectClass.CHESS_PIECE, 22.0)).profile == (
        "chess_knight"
    )
    rook = select_horizontal_grasp_plan(detection(ObjectClass.CHESS_PIECE, 24.5))
    assert (rook.profile, rook.close_width_mm) == ("chess_rook", 15.0)


def test_star_column_and_soccer_share_one_profile():
    """폭 45 와 46 은 호모그래피 추정 오차 안이라 갈 수 없다 — 같은 프로필로 간다.

    `star_column` 프로필이 존재한다는 것만 검사하면 정책이 그걸 고르는 것처럼
    읽힌다. 실제로는 둘 다 `soccer_polyhedron` 으로 가고, 값이 같아 동작한다.
    """
    star = detection(ObjectClass.GABE, 45.0)
    soccer = detection(ObjectClass.GABE, 46.0)

    assert select_horizontal_grasp_plan(star) == select_horizontal_grasp_plan(soccer)
    assert select_horizontal_grasp_plan(star).profile == "soccer_polyhedron"
