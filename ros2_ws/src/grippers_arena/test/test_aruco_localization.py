"""aruco_localization.py 순수 수학 테스트. rclpy/카메라 없이도 돈다 — 마커
검출 자체가 아니라 "검출된 코너로 좌표를 구하는" 수학만 본다
(grippers_perception의 multi_frame_consensus.py와 같은 이유).

`fit_homography_dlt`의 실제 피팅 성공 경로는 cv2가 있어야 검증된다 —
`pytest.importorskip("cv2")`로 그 한 테스트만 건너뛴다(cv2 없는 이 환경에선
스킵되고, cv2 있는 실기/개발 환경에서는 그대로 돈다)."""

import math

import pytest
from grippers_arena.aruco_localization import (
    MIN_CORRESPONDENCES,
    apply_homography,
    axis_aligned_marker_world_corners,
    build_correspondences,
    fit_homography_dlt,
    robot_pose_from_marker_corners,
)


def test_apply_homography_scales_and_translates():
    # X = 2u + 10, Y = 3v + 20 (동차 성분 w는 항상 1)
    h = (2, 0, 10, 0, 3, 20, 0, 0, 1)

    assert apply_homography(h, 5, 5) == (20, 35)


def test_apply_homography_normalizes_by_w():
    # w = u + 1 이라 (u, v) = (1, 0)이면 w=2 — 정규화를 빼먹으면 틀린다.
    h = (1, 0, 0, 0, 1, 0, 1, 0, 1)

    x, y = apply_homography(h, 1, 4)
    assert x == 0.5  # (1*1 + 0*4 + 0) / (1*1 + 0*4 + 1) = 1/2
    assert y == 2.0  # (0*1 + 1*4 + 0) / 2


def test_axis_aligned_marker_world_corners_order_matches_convention():
    corners = axis_aligned_marker_world_corners(center_mm=(100, 100), size_mm=20)

    assert corners == (
        (90, 110),  # top_left
        (110, 110),  # top_right
        (110, 90),  # bottom_right
        (90, 90),  # bottom_left
    )


def test_build_correspondences_skips_markers_missing_from_layout():
    layout = {1: axis_aligned_marker_world_corners((0, 0), 40)}
    detections = {
        1: ((10, 10), (50, 10), (50, 50), (10, 50)),
        99: ((200, 200), (240, 200), (240, 240), (200, 240)),  # layout에 없음
    }

    image_points, world_points = build_correspondences(detections, layout)

    assert image_points == [(10, 10), (50, 10), (50, 50), (10, 50)]
    assert world_points == list(layout[1])


def test_build_correspondences_returns_empty_when_nothing_matches():
    assert build_correspondences({1: ()}, {2: ()}) == ([], [])


def test_fit_homography_dlt_returns_none_below_minimum_points():
    few_points = [(0, 0)] * (MIN_CORRESPONDENCES - 1)

    assert fit_homography_dlt(few_points, few_points) is None


def test_fit_homography_dlt_fits_a_known_affine_map():
    cv2 = pytest.importorskip("cv2")
    del cv2  # 설치 여부만 확인하면 된다 — 실제 호출은 모듈 안에서 한다.

    # 세계 좌표 = 이미지 좌표 * 2 (스케일만 있는 단순 매핑)인 정사각형 4점.
    image_points = [(0, 0), (10, 0), (10, 10), (0, 10)]
    world_points = [(0, 0), (20, 0), (20, 20), (0, 20)]

    h = fit_homography_dlt(image_points, world_points)

    assert h is not None
    for (u, v), (expected_x, expected_y) in zip(image_points, world_points, strict=True):
        x, y = apply_homography(h, u, v)
        assert x == pytest.approx(expected_x, abs=1e-6)
        assert y == pytest.approx(expected_y, abs=1e-6)


def test_robot_pose_from_marker_corners_returns_none_without_homography():
    assert robot_pose_from_marker_corners(None, [(0, 0)] * 4) is None


def test_robot_pose_from_marker_corners_returns_none_for_wrong_corner_count():
    identity = (1, 0, 0, 0, 1, 0, 0, 0, 1)

    assert robot_pose_from_marker_corners(identity, [(0, 0)] * 3) is None


def test_robot_pose_from_marker_corners_default_front_edge():
    """항등 호모그래피(세계 좌표 = 이미지 좌표)로 기하만 검증한다.
    top_left=(0,10), top_right=(10,10), bottom_right=(10,0), bottom_left=(0,0)
    — 위쪽(+y)이 정면인 마커."""
    identity = (1, 0, 0, 0, 1, 0, 0, 0, 1)
    corners = [(0, 10), (10, 10), (10, 0), (0, 0)]

    x, y, theta = robot_pose_from_marker_corners(identity, corners)

    assert (x, y) == (5, 5)  # 4코너 중심
    assert theta == pytest.approx(math.pi / 2)


def test_robot_pose_from_marker_corners_custom_front_edge():
    """front_edge를 바꾸면(top_right→bottom_right, 오른쪽 모서리) 정면 방향도
    그만큼 바뀐다 — 실제 마커 부착 방향에 맞춰 이 인자 하나만 바꾸면 되는
    설계를 검증한다."""
    identity = (1, 0, 0, 0, 1, 0, 0, 0, 1)
    corners = [(0, 10), (10, 10), (10, 0), (0, 0)]

    x, y, theta = robot_pose_from_marker_corners(identity, corners, front_edge=(1, 2))

    assert (x, y) == (5, 5)
    assert theta == pytest.approx(0.0, abs=1e-9)  # +x 방향(오른쪽)을 정면으로 봄
