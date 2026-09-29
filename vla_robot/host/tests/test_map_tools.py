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
    """지금 host.yaml 의 네 점은 (10, 40) cm 에서 가로 160 · 세로 100 으로 나온다."""
    got = make_layout.rectangle(0.10, 0.40, 1.60, 1.00)
    for mid, (x, y) in got.items():
        want = cfg.aruco.floor_markers[mid]
        assert (x, y) == pytest.approx(want, abs=1e-9)


def test_check_flags_markers_outside_the_walls(cfg):
    points = make_layout.rectangle(0.10, 0.40, 2.00, 1.00)      # 2 m 폭 = 가벽 밖
    warn = make_layout.check(points, cfg.arena.wall_x, cfg.arena.wall_y)
    assert any("가벽 밖" in w for w in warn)


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
    """chess 상자 뒤(더 먼 쪽) 바닥은 뒤쪽 카메라에서 상자에 가린다."""
    C, _R, _t = check_coverage.camera_pose("B", 0.9, 0.0, 1.30, 42.8, cfg.arena.wall_y)
    bx, by, _yaw = cfg.arena.boxes["chess"]
    behind = np.array([bx, by - cfg.arena.box_size[1] / 2.0 - 0.05, 0.0])
    assert check_coverage.box_blocks(C, behind, cfg.arena.boxes, cfg.arena.box_size)
    # 상자에서 멀리 떨어진 앞쪽은 막히지 않는다
    assert not check_coverage.box_blocks(C, np.array([0.9, 0.9, 0.0]),
                                         cfg.arena.boxes, cfg.arena.box_size)


def test_current_placement_covers_the_workspace(cfg):
    """도면 배치(높이 1.30 · 후퇴 0 · 하향 42.8°)에서 작업 구역이 전부 덮여야 한다."""
    hc = cfg
    from localization.aruco_localizer import approx_camera_matrix
    conf = {"K": approx_camera_matrix(hc.cameras.width, hc.cameras.height, hc.cameras.hfov_deg),
            "w": hc.cameras.width, "h": hc.cameras.height, "wall_y": hc.arena.wall_y,
            "boxes": hc.arena.boxes, "box_size": hc.arena.box_size,
            "workspace_x": hc.arena.workspace_x, "workspace_y": hc.arena.workspace_y}
    cams = [check_coverage.camera_pose(s, 0.9, 0.0, 1.30, 42.8, hc.arena.wall_y)
            for s in ("A", "B")]
    rep = check_coverage.coverage(cams, conf, hc.aruco.robot_marker_height_m,
                                  hc.aruco.robot_marker_size_m, 0.25, n=12)
    assert rep["any"] == pytest.approx(100.0)       # 한 대 이상은 전 구역
    assert rep["both"] > 40.0                       # 가운데 띠는 두 대가 본다
    assert 0 < rep["worst_mm_px"] < 5.0             # 80 mm 마커가 최악점에서도 16 px 이상


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

    2026-09-29 부터 경계가 장판 가장자리(실측 y -0.125)라, 권장 배치 1.60 m 에서 57.4° 다.
    """
    import math
    h = 1.60
    cy = (cfg.arena.workspace_y[0] + cfg.arena.workspace_y[1]) / 2.0
    want = math.degrees(math.atan2(h, cy - cfg.arena.wall_y[0]))
    tilt = check_coverage.aim_tilt_deg("A", 0.9, 0.0, h, cfg)
    assert tilt == pytest.approx(want)
    assert tilt == pytest.approx(57.4, abs=0.2)
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
    """실측 장판(1.835 × 2.050 m, 2026-09-29)에 설계 사각형을 가운데 두면 — 현장에서 잴 숫자들.

    배송 규격은 2.000 이었지만 실측은 2.050 이었다. 앞뒤 여백이 10 -> 12.5 cm 로 바뀐다.
    """
    lay = make_layout.mat_layout(1.835, 2.050, cfg.aruco.floor_markers,
                                 cfg.arena.boxes, cfg.arena.box_size)
    assert lay["margin"] == pytest.approx((0.0175, 0.125))
    # 네 마커 모두 옆 가장자리 11.75 cm · 앞/뒤 가장자리 52.5 cm — 대칭이라 외우기 쉽다
    for mid, d in lay["markers"].items():
        assert min(d["left"], d["right"]) == pytest.approx(0.1175)
        assert min(d["front"], d["back"]) == pytest.approx(0.525)
    # 상자 뒷면은 장판 뒤끝에서 12.5 cm
    for b in lay["boxes"].values():
        assert b["back_edge_from_back"] == pytest.approx(0.125)
    # 작업 경계가 곧 host.yaml 값이다
    assert lay["wall_x"] == pytest.approx(cfg.arena.wall_x)
    assert lay["wall_y"] == pytest.approx(cfg.arena.wall_y)


def test_mat_layout_follows_a_slightly_different_mat(cfg):
    """실측이 배송 규격과 조금 달라도 그 값을 넣으면 그대로 반영된다."""
    lay = make_layout.mat_layout(1.830, 1.990, cfg.aruco.floor_markers,
                                 cfg.arena.boxes, cfg.arena.box_size)
    assert lay["margin"] == pytest.approx((0.015, 0.095))
    assert lay["markers"][1]["left"] == pytest.approx(0.115)
    assert lay["markers"][1]["front"] == pytest.approx(0.495)
