"""맵 도구의 순수 계산 부분 — 카메라 없이 돌아가는 것만 본다.

화면·카메라·주행은 실기에서 본다. 여기서 지키는 것은 기하와 판정 기준이다.
"""
import numpy as np
import pytest

from tools import calib_yaw, check_coverage, make_layout


# ---------------------------------------------------------------------------
# make_layout — 줄자 값 -> 좌표
# ---------------------------------------------------------------------------
def test_rectangle_reproduces_the_current_layout(cfg):
    """지금 host.yaml 의 네 점은 (7, 52.5) cm 에서 가로 191 · 세로 78.5 로 나온다."""
    got = make_layout.rectangle(0.070, 0.525, 1.910, 0.785)
    for mid, (x, y) in got.items():
        want = cfg.aruco.floor_markers[mid]
        assert (x, y) == pytest.approx(want, abs=1e-9)


def test_check_flags_markers_outside_the_walls(cfg):
    points = make_layout.rectangle(0.10, 0.40, 2.00, 1.00)      # 오른쪽 끝 2.10 = 장판 밖
    warn = make_layout.check(points, cfg.arena.wall_x, cfg.arena.wall_y)
    assert any("장판 밖" in w for w in warn)


def test_check_flags_clustered_markers(cfg):
    points = make_layout.rectangle(0.80, 0.80, 0.10, 0.10)
    warn = make_layout.check(points, cfg.arena.wall_x, cfg.arena.wall_y)
    assert any("모여 있다" in w for w in warn)


def test_check_flags_a_skewed_rectangle(cfg):
    points = make_layout.rectangle(0.10, 0.40, 1.60, 1.00)
    points[4] = (points[4][0] + 0.05, points[4][1])              # 한 점만 5 cm 밀기
    warn = make_layout.check(points, cfg.arena.wall_x, cfg.arena.wall_y)
    assert any("대각선" in w for w in warn)


def test_good_layout_has_no_warning(cfg):
    points = {mid: tuple(v) for mid, v in cfg.aruco.floor_markers.items()}
    assert make_layout.check(points, cfg.arena.wall_x, cfg.arena.wall_y) == []


# ---------------------------------------------------------------------------
# check_coverage — 가림 기하
# ---------------------------------------------------------------------------
def test_wall_blocks_the_near_floor_when_the_camera_is_set_back(cfg):
    """뒤로 물러나 세우면 가벽이 가까운 바닥을 가린다 — 근거리 띠가 죽는 이유다.
    값은 팀 배치도 Rev.II 안(후퇴 0.95 m · 높이 1.65 m)."""
    C, _R, _t = check_coverage.camera_pose("A", 0.9, 0.95, 1.65, 44.1, cfg.arena.wall_y)
    # 벽 기준 상대 좌표로 둔다 — 경계(wall_y)는 장판을 바꿀 때마다 옮겨진다
    near = np.array([0.9, cfg.arena.wall_y[0] + 0.05, 0.0])
    far = np.array([0.9, 1.20, 0.0])
    assert check_coverage.wall_blocks(C, near, cfg.arena.wall_y, 0.25)
    assert not check_coverage.wall_blocks(C, far, cfg.arena.wall_y, 0.25)


def test_no_wall_occlusion_when_the_camera_sits_on_the_wall(cfg):
    """지금 배치는 후퇴 0 이다 — 렌즈가 가벽 바로 위라 **자기 가벽은 아무것도 못 가린다.**
    이 배치에서 근거리 한계를 정하는 것은 가벽이 아니라 화각이다."""
    C, _R, _t = check_coverage.camera_pose("A", 0.9, 0.0, 1.30, 42.8, cfg.arena.wall_y)
    assert not check_coverage.wall_blocks(C, np.array([0.9, 0.05, 0.0]), cfg.arena.wall_y, 0.25)


def test_wall_does_not_block_what_is_above_it(cfg):
    """로봇 마커는 0.27 m 에 떠 있어 같은 자리라도 가벽 위로 보인다."""
    C, _R, _t = check_coverage.camera_pose("A", 0.9, 0.0, 1.30, 42.8, cfg.arena.wall_y)
    target = np.array([0.9, 0.35, cfg.aruco.robot_marker_height_m])
    assert not check_coverage.wall_blocks(C, target, cfg.arena.wall_y, 0.25)


def test_box_blocks_what_is_behind_it(cfg):
    """상자 바로 앞 바닥은 뒤쪽 카메라(상자 너머)에서 상자에 가린다."""
    C, _R, _t = check_coverage.camera_pose("B", 1.025, 0.0, 1.30, 42.8, cfg.arena.wall_y)
    bx, by, _yaw = cfg.arena.boxes["basket"]
    behind = np.array([bx, by - cfg.arena.box_size[1] / 2.0 - 0.05, 0.0])
    assert check_coverage.box_blocks(C, behind, cfg.arena.boxes, cfg.arena.box_size)
    # 상자에서 멀리 떨어진 앞쪽은 막히지 않는다
    assert not check_coverage.box_blocks(C, np.array([0.9, 0.9, 0.0]),
                                         cfg.arena.boxes, cfg.arena.box_size)


def test_current_placement_covers_the_workspace(cfg):
    """배치도 REV.2(앞뒤 긴 변 가운데 · 높이 1.60 · 가벽 없음)에서 카메라마다 마커 4장을
    보고, 작업 구역을 거의 전부 두 대가 함께 봐야 한다."""
    K, _real = check_coverage.camera_matrix(cfg, cfg.cameras.indices[0])
    conf = {"K": K, "w": cfg.cameras.width, "h": cfg.cameras.height,
            "wall_y": cfg.arena.wall_y, "boxes": cfg.arena.boxes,
            "box_size": cfg.arena.box_size, "workspace_x": cfg.arena.workspace_x,
            "workspace_y": cfg.arena.workspace_y}
    rep = check_coverage.evaluate(cfg, conf, check_coverage.centre_x(cfg), 0.0, 1.60, 0.0, grid=12)
    assert rep["marker_seen"] == [4, 4]
    assert rep["any"] == pytest.approx(100.0)
    assert rep["both"] >= 95.0                      # 성긴 격자라 구석 한두 칸이 한 대일 수 있다
    assert 0 < rep["worst_mm_px"] < 3.0             # 40 mm 물체가 최악점에서도 13 px 이상


def test_a_camera_too_low_loses_coverage(cfg):
    """높이를 0.5 m 로 낮추면 가벽이 시야를 먹어 커버리지가 떨어진다 — 도구가 그걸 잡아야 한다."""
    hc = cfg
    from localization.aruco_localizer import approx_camera_matrix
    conf = {"K": approx_camera_matrix(hc.cameras.width, hc.cameras.height, hc.cameras.hfov_deg),
            "w": hc.cameras.width, "h": hc.cameras.height, "wall_y": hc.arena.wall_y,
            "boxes": hc.arena.boxes, "box_size": hc.arena.box_size,
            "workspace_x": hc.arena.workspace_x, "workspace_y": hc.arena.workspace_y}
    low = [check_coverage.camera_pose(s, 0.9, 0.0, 0.50, 42.8, hc.arena.wall_y)
           for s in ("A", "B")]
    rep = check_coverage.coverage(low, conf, hc.aruco.robot_marker_height_m,
                                  hc.aruco.robot_marker_size_m, 0.25, n=12)
    assert rep["any"] < 100.0


# ---------------------------------------------------------------------------
# calib_yaw — 각도 계산
# ---------------------------------------------------------------------------
def test_circular_mean_handles_the_wrap():
    """±180 근처에서 산술평균은 0 이 된다. 원형 평균은 180 을 낸다."""
    assert calib_yaw.circular_mean_deg([179.0, -179.0]) == pytest.approx(180.0, abs=1e-6)
    assert calib_yaw.circular_mean_deg([10.0, 20.0, 30.0]) == pytest.approx(20.0, abs=0.01)


def test_wrap180():
    assert calib_yaw.wrap180(190.0) == pytest.approx(-170.0)
    assert calib_yaw.wrap180(-190.0) == pytest.approx(170.0)
    assert calib_yaw.wrap180(0.0) == 0.0


def test_static_drift_math_matches_the_2026_09_06_measurement(cfg):
    """실측 예: +x 를 향했는데 yaw 가 +5.06 로 읽혔다 -> 보정값은 그만큼 줄어든다."""
    expect = calib_yaw.EXPECTED["x"]
    drift = calib_yaw.wrap180(5.06 - expect)
    new = calib_yaw.wrap180(90.76 - drift)          # 그때의 기존 보정값
    assert new == pytest.approx(85.7, abs=0.01)


def test_aim_tilt_points_at_the_workspace_centre(cfg):
    """하향각 = 작업 구역 중심을 겨누는 각. 카메라는 작업 경계(장판 가장자리)에 선다.

    배치도 REV.2: 앞뒤가 1.835 m 라 작업 구역 중심(0.9175)이 장판 중심과 같고, 1.60 m 에서 60.2° 다.
    """
    import math
    h = 1.60
    cy = (cfg.arena.workspace_y[0] + cfg.arena.workspace_y[1]) / 2.0
    want = math.degrees(math.atan2(h, cy - cfg.arena.wall_y[0]))
    tilt = check_coverage.aim_tilt_deg("A", 0.9, 0.0, h, cfg)
    assert tilt == pytest.approx(want)
    assert tilt == pytest.approx(60.2, abs=0.2)
    # 낮게 세우면 완만해진다
    assert check_coverage.aim_tilt_deg("A", 0.9, 0.0, 1.30, cfg) < tilt


def test_camera_matrix_uses_the_real_calibration(cfg):
    """근사 화각이 아니라 cam0.npz 의 K 를 써야 커버리지 숫자가 실물과 맞는다."""
    K, real = check_coverage.camera_matrix(cfg, cfg.cameras.indices[0])
    assert real, "host/calib/cam0.npz 가 있어야 한다"
    import math
    hfov = 2 * math.degrees(math.atan(cfg.cameras.width / 2 / K[0, 0]))
    assert 65.0 < hfov < 72.0      # C920 실측 68.6도


def test_removing_the_walls_improves_dual_coverage(cfg):
    """가벽이 없으면 근거리 가림이 사라져 두 대가 보는 영역이 넓어진다 (2026-09-23 결정 근거)."""
    K, _real = check_coverage.camera_matrix(cfg, cfg.cameras.indices[0])
    conf = {"K": K, "w": cfg.cameras.width, "h": cfg.cameras.height,
            "wall_y": cfg.arena.wall_y, "boxes": cfg.arena.boxes,
            "box_size": cfg.arena.box_size, "workspace_x": cfg.arena.workspace_x,
            "workspace_y": cfg.arena.workspace_y}
    # 후퇴 0.95 m 배치에서만 가벽이 시야를 먹는다(후퇴 0 이면 렌즈가 벽 위라 무관)
    no_wall = check_coverage.evaluate(cfg, conf, 0.9, 0.95, 1.65, 0.0, grid=12)
    with_wall = check_coverage.evaluate(cfg, conf, 0.9, 0.95, 1.65, 0.25, grid=12)
    assert no_wall["both"] >= with_wall["both"]


def test_sweep_returns_usable_placements_first(cfg):
    """스윕은 두 대 관측이 넓고 해상도가 좋은 순으로 돌려준다."""
    K, _real = check_coverage.camera_matrix(cfg, cfg.cameras.indices[0])
    conf = {"K": K, "w": cfg.cameras.width, "h": cfg.cameras.height,
            "wall_y": cfg.arena.wall_y, "boxes": cfg.arena.boxes,
            "box_size": cfg.arena.box_size, "workspace_x": cfg.arena.workspace_x,
            "workspace_y": cfg.arena.workspace_y}
    found = check_coverage.sweep(cfg, conf, 0.0, 12, cfg.aruco.min_floor_markers)
    assert found, "가벽 없는 1.8 m 정사각에서는 세울 자리가 있어야 한다"
    assert all(r["any"] == pytest.approx(100.0) for r in found)
    assert found[0]["both"] >= found[-1]["both"]
    assert 1.2 <= found[0]["height"] <= 2.0


def test_mat_layout_for_the_measured_mat(cfg):
    """실측 장판(2.050 × 1.835 m, 긴 변이 좌우)에서 배치도 REV.2 — host.yaml 과 같아야 한다."""
    lay = make_layout.mat_layout(2.050, 1.835, cfg.arena.box_size)
    assert lay["wall_x"] == pytest.approx(cfg.arena.wall_x)
    assert lay["wall_y"] == pytest.approx(cfg.arena.wall_y)
    for mid, xy in lay["floor_markers"].items():
        assert xy == pytest.approx(cfg.aruco.floor_markers[mid])
    assert lay["workspace_x"] == pytest.approx(cfg.arena.workspace_x)
    assert lay["workspace_y"] == pytest.approx(cfg.arena.workspace_y)
    assert lay["box"] == pytest.approx(cfg.arena.boxes["basket"])
    # 현장 숫자: 상자 좌우 92 cm · 종이 앞뒤 변 45.5 cm
    assert lay["box_side_gap"] == pytest.approx(0.920)
    assert lay["paper_edge_y"] == pytest.approx(0.455)
    assert make_layout.check(lay["floor_markers"], lay["wall_x"], lay["wall_y"]) == []


def test_mat_layout_follows_a_slightly_different_mat(cfg):
    """장판을 다시 재서 조금 달라도 그 값대로 — 마커는 끝에 붙고 상자는 가운데·뒤끝이다."""
    lay = make_layout.mat_layout(2.040, 1.830, cfg.arena.box_size)
    assert lay["floor_markers"][2] == pytest.approx((1.970, 0.525))
    assert lay["floor_markers"][4] == pytest.approx((1.970, 1.305))
    bx, by, _yaw = lay["box"]
    assert (bx, by) == pytest.approx((1.020, 1.830 - 0.175))
