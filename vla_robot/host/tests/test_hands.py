"""손 검출 후처리: 가장자리 거르기 · 두 카메라 교차/높이 평면 좌표 · 지속 시간 확인 · 설정 검증.
MediaPipe 자체는 부르지 않는다(검출 결과 HandSighting 을 직접 만든다)."""
from dataclasses import replace

import numpy as np
import pytest

import host_config
from localization.aruco_localizer import Camera, approx_camera_matrix, floor_object_points
from perception.hands import (HAND_LABEL, HandDetector, HandSighting, hand_observations,
                              in_hand_zone, locate_hands, make_hand_tracker, nearest_spot)
from tests.test_localizer_synthetic import _camera_pose, _project


@pytest.mark.parametrize("xy, ok", [
    ((0.99, 0.12), True),     # F2 — 앞 가장자리
    ((0.075, 0.915), True),   # L2
    ((1.905, 1.56), True),    # R3 — 뒤쪽 모서리지만 오른쪽 가장자리
    ((0.99, -0.20), True),    # 장판 앞으로 내민 손
    ((0.99, 0.90), False),    # 작업 구역 가운데 — 기물 자리
    ((0.99, 1.70), False),    # 뒤 가장자리는 뺐다(상자 쪽)
    ((0.99, -0.40), False),   # outside_m 보다 멀다
])
def test_hand_zone(cfg, xy, ok):
    assert in_hand_zone(cfg.hands, cfg.arena, *xy) is ok


def test_back_edge_can_be_enabled(cfg):
    h = replace(cfg.hands, edges=("back",))
    assert in_hand_zone(h, cfg.arena, 0.99, 1.70)
    assert not in_hand_zone(h, cfg.arena, 0.99, 0.12)


def test_nearest_spot():
    assert nearest_spot((0.10, 0.90)) == "L2"
    assert nearest_spot((0.99, 0.90)) == "?"
    assert nearest_spot(None) == "-"


FRONT = ((1.02, -0.03, 1.59), (0.99, 1.10, 0.0))     # 실측 외부파라미터와 비슷한 자리
BACK = ((0.96, 1.91, 1.58), (0.99, 0.70, 0.0))


def _synthetic_cam(cfg, where=BACK, name="cam1"):
    """바닥 마커로 외부파라미터를 푼 합성 카메라와 (K, R, t)."""
    K = approx_camera_matrix(1280, 720, 70.4)
    R, t = _camera_pose(*where)
    cam = Camera(name, K, np.zeros((5, 1)), cfg.aruco, calibrated=True)
    det = {mid: _project(K, R, t, pts) for mid, pts in floor_object_points(cfg.aruco).items()}
    assert cam.solve_extrinsics(det)
    return cam, K, R, t


def _sighting_at(K, R, t, x, y, z, score=0.9):
    """손바닥 중심이 (x, y, z) 인 손 — 21점을 모두 그 점 둘레에 둔다."""
    ring = [(x + 0.04 * np.cos(a), y + 0.04 * np.sin(a), z) for a in np.linspace(0, 2 * np.pi, 21)]
    pts = _project(K, R, t, ring)
    pts += _project(K, R, t, [(x, y, z)])[0] - pts[[0, 5, 9, 13, 17]].mean(axis=0)
    return HandSighting(pts, score)


def _two_cams(cfg):
    return _synthetic_cam(cfg, FRONT, "cam0"), _synthetic_cam(cfg, BACK, "cam1")


def test_single_camera_uses_hand_height(cfg):
    cam, K, R, t = _synthetic_cam(cfg)
    z = cfg.hands.hand_z_m
    obs = hand_observations([cam], [[_sighting_at(K, R, t, 0.99, 0.12, z)]], cfg.hands, cfg.arena)
    assert len(obs) == 1 and obs[0].label == HAND_LABEL and obs[0].cam_name == "cam1"
    assert abs(obs[0].x - 0.99) < 0.005 and abs(obs[0].y - 0.12) < 0.005
    # 같은 픽셀을 바닥으로 풀면 카메라에서 먼 쪽으로 크게 밀린다 — 높이가 필요한 이유
    floor = cam.pixels_to_plane(_sighting_at(K, R, t, 0.99, 0.12, z).palm.reshape(1, 2), 0.0)
    assert floor[0, 1] < 0.12 - 0.15


@pytest.mark.parametrize("z", [0.20, 0.32, 0.45, 0.60])
def test_two_cameras_triangulate_any_height(cfg, z):
    """10-01 실기: 45 cm 로 든 손 하나가 높이 고정 풀이로 y 0.76 / 1.00 두 개가 됐다."""
    (c0, K0, R0, t0), (c1, K1, R1, t1) = _two_cams(cfg)
    s0, s1 = _sighting_at(K0, R0, t0, 0.075, 0.915, z), _sighting_at(K1, R1, t1, 0.075, 0.915, z)
    pts = locate_hands([c0, c1], [[s0], [s1]], cfg.hands)
    assert len(pts) == 1 and pts[0].cams == "cam0+cam1"
    assert abs(pts[0].x - 0.075) < 0.005 and abs(pts[0].y - 0.915) < 0.005 and abs(pts[0].z - z) < 0.005
    obs = hand_observations([c0, c1], [[s0], [s1]], cfg.hands, cfg.arena)
    assert len(obs) == 1


def test_fixed_height_would_split_a_raised_hand(cfg):
    """교차를 안 하면 생기는 일: 든 손을 낮은 평면으로 풀면 각 카메라가 자기에게서 먼 쪽으로
    민다 — 앞 카메라는 뒤로, 뒤 카메라는 앞으로. 실측(0.76 / 1.00)과 같은 크기로 갈라진다."""
    (c0, K0, R0, t0), (c1, K1, R1, t1) = _two_cams(cfg)
    z = cfg.hands.hand_z_m
    y0 = c0.pixels_to_plane(_sighting_at(K0, R0, t0, 0.075, 0.915, 0.45).palm.reshape(1, 2), z)[0, 1]
    y1 = c1.pixels_to_plane(_sighting_at(K1, R1, t1, 0.075, 0.915, 0.45).palm.reshape(1, 2), z)[0, 1]
    assert y0 - y1 > cfg.hands.merge_dist_m


def test_two_hands_stay_two(cfg):
    (c0, K0, R0, t0), (c1, K1, R1, t1) = _two_cams(cfg)
    z = 0.35
    left = (0.075, 0.915, z)
    right = (1.905, 0.915, z)
    pts = locate_hands([c0, c1],
                       [[_sighting_at(K0, R0, t0, *left), _sighting_at(K0, R0, t0, *right)],
                        [_sighting_at(K1, R1, t1, *right), _sighting_at(K1, R1, t1, *left)]],
                       cfg.hands)
    got = sorted((round(p.x, 2), round(p.y, 2)) for p in pts)
    assert got == [(0.07, 0.92), (1.9, 0.92)] or got == [(0.08, 0.92), (1.9, 0.92)]
    assert all(p.cams == "cam0+cam1" for p in pts)


def test_hand_seen_by_one_camera_only(cfg):
    (c0, K0, R0, t0), (c1, K1, R1, t1) = _two_cams(cfg)
    s1 = _sighting_at(K1, R1, t1, 0.99, 0.12, cfg.hands.hand_z_m)
    pts = locate_hands([c0, c1], [[], [s1]], cfg.hands)
    assert len(pts) == 1 and pts[0].cams == "cam1" and pts[0].z == cfg.hands.hand_z_m


def test_hand_observation_filters(cfg):
    cam, K, R, t = _synthetic_cam(cfg)
    z = cfg.hands.hand_z_m
    middle = _sighting_at(K, R, t, 0.99, 0.90, z)              # 작업 구역 가운데
    weak = _sighting_at(K, R, t, 0.075, 0.915, z, score=0.2)   # 확신도 낮음
    assert hand_observations([cam], [[middle, weak]], cfg.hands, cfg.arena) == []
    unready = Camera("x", K, np.zeros((5, 1)), cfg.aruco)
    assert hand_observations([unready], [[_sighting_at(K, R, t, 0.99, 0.12, z)]], cfg.hands,
                             cfg.arena) == []
    assert hand_observations([cam], [None], cfg.hands, cfg.arena) == []


def test_hand_needs_confirm_time(cfg):
    cam, K, R, t = _synthetic_cam(cfg)
    tracker = make_hand_tracker(cfg.hands, cfg.tracker)
    s = _sighting_at(K, R, t, 0.075, 0.915, cfg.hands.hand_z_m)
    obs = [hand_observations([cam], [[s]], cfg.hands, cfg.arena)]
    assert tracker.update(obs, 0.0).get(HAND_LABEL) is None
    assert tracker.update(obs, cfg.hands.confirm_s * 0.5).get(HAND_LABEL) is None
    hands = tracker.update(obs, cfg.hands.confirm_s + 0.05)[HAND_LABEL]
    assert len(hands) == 1 and nearest_spot(hands[0]) == "L2"
    # 사라지면 hold_s 뒤에 지도에서 빠진다
    assert tracker.update([[]], cfg.hands.confirm_s + cfg.hands.hold_s + 0.2).get(HAND_LABEL) is None


def test_disabled_detector_is_inert(cfg):
    det = HandDetector(replace(cfg.hands, enabled=False), [0, 1])
    assert not det.ok
    det.submit(0, np.zeros((4, 4, 3), np.uint8))
    assert det.latest(0) is None
    det.close()


def test_missing_model_is_inert(cfg, tmp_path):
    det = HandDetector(replace(cfg.hands, model_path=str(tmp_path / "none.task")), [0])
    assert not det.ok


def test_bad_edge_rejected(tmp_path):
    text = host_config.DEFAULT_CONFIG.read_text(encoding="utf-8")
    bad = tmp_path / "host.yaml"
    bad.write_text(text.replace("edges: [front, left, right]", "edges: [front, top]"), encoding="utf-8")
    with pytest.raises(host_config.ConfigError):
        host_config.load_host_config(bad)


def test_one_hand_unpaired_in_both_cameras_is_one_hand(cfg):
    """10-08: 손 하나가 15~20 cm 떨어진 둘로 보였다 — 두 카메라 광선이 짝(8 cm)을 못 지으면 카메라마다 높이 평면으로
    풀어 따로 나왔다. 짝 없는 두 카메라 손이 same_hand_m 안이면 하나 — 더 곧게 내려다보는 앞 카메라 것을 쓴다."""
    (c0, K0, R0, t0), (c1, K1, R1, t1) = _two_cams(cfg)
    z = cfg.hands.hand_z_m
    s0 = _sighting_at(K0, R0, t0, 0.99, 0.12, z)                      # 앞 카메라: 손 바로 위에 가깝다
    s1 = _sighting_at(K1, R1, t1, 1.12, 0.12, 0.45)                   # 뒤 카메라: 손바닥 중심이 다르게 잡혀 광선이 어긋남
    split = locate_hands([c0, c1], [[s0], [s1]], replace(cfg.hands, same_hand_m=0.0))
    assert len(split) == 2 and all("+" not in p.cams for p in split)  # 고치기 전: 둘
    pts = locate_hands([c0, c1], [[s0], [s1]], cfg.hands)
    assert len(pts) == 1 and pts[0].cams == "cam0"
    assert abs(pts[0].x - 0.99) < 0.01 and abs(pts[0].y - 0.12) < 0.01


def test_two_real_hands_far_apart_stay_two_when_unpaired(cfg):
    (c0, K0, R0, t0), (c1, K1, R1, t1) = _two_cams(cfg)
    z = cfg.hands.hand_z_m
    pts = locate_hands([c0, c1], [[_sighting_at(K0, R0, t0, 0.33, 0.12, z)],
                                  [_sighting_at(K1, R1, t1, 1.905, 0.915, z)]], cfg.hands)
    assert len(pts) == 2
