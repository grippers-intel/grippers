"""넓은 기물: 바닥 접점(카메라에 가까운 가장자리) -> 중심 (2026-10-05, star)."""
import math

import numpy as np

from perception.piece_tracker import away_from_camera, observations_from_detections


class _Det:
    def __init__(self, label, px):
        self.label, self.confidence, self.bottom_center = label, 0.9, px


class _Cam:
    """pixels_to_plane 이 픽셀을 그대로 지도 좌표로 돌려주는 가짜 카메라."""
    def __init__(self, name, center):
        self.name, self.center, self.ready = name, np.array(center, dtype=float), True

    def pixels_to_plane(self, px, z):
        return np.asarray(px, dtype=float)


def test_edge_moves_away_from_the_camera_by_the_radius():
    x, y = away_from_camera((1.0, 0.0), (1.0, 0.9), 0.04)
    assert math.isclose(x, 1.0) and math.isclose(y, 0.94)
    x, y = away_from_camera((1.0, 1.83), (1.0, 0.98), 0.04)
    assert math.isclose(y, 0.94)


def test_two_cameras_facing_each_other_agree_after_correction():
    """장판 가운데 별: 앞 카메라는 y 를 작게, 뒤 카메라는 크게 본다(각자 가까운 가장자리)."""
    front, back = _Cam("A", (0.99, -0.30, 1.6)), _Cam("B", (0.99, 2.13, 1.6))
    a = observations_from_detections(front, [_Det("star", (0.99, 0.86))], 0.5, {"star": 0.045})[0]
    b = observations_from_detections(back, [_Det("star", (0.99, 0.95))], 0.5, {"star": 0.045})[0]
    assert abs(a.y - b.y) < 1e-6 and math.isclose(a.y, 0.905)


def test_labels_without_a_radius_are_unchanged():
    cam = _Cam("A", (0.99, -0.30, 1.6))
    o = observations_from_detections(cam, [_Det("queen", (0.5, 0.7))], 0.5, {"star": 0.045})[0]
    assert (o.x, o.y) == (0.5, 0.7)
    o = observations_from_detections(cam, [_Det("star", (0.5, 0.7))], 0.5)[0]
    assert (o.x, o.y) == (0.5, 0.7)
