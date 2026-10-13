"""카메라 없이 ArUco 수학을 검증한다: 알려진 카메라 자세로 마커 코너를 투영해 넣고
로봇 pose 가 되돌아오는지 본다(외부파라미터 + z 구속 + yaw_offset)."""
import math

import cv2
import numpy as np

from localization.aruco_localizer import Camera, RobotLocalizer, approx_camera_matrix, floor_object_points


def _camera_pose(cam_xyz, look_at):
    """map -> camera 회전/이동 (카메라 +z 가 look_at 을 향한다)."""
    c = np.asarray(cam_xyz, float)
    z = np.asarray(look_at, float) - c
    z /= np.linalg.norm(z)
    x = np.cross(z, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.vstack([x, y, z])
    t = -R @ c
    return R, t.reshape(3, 1)


def _project(K, R, t, pts):
    rvec, _ = cv2.Rodrigues(R)
    img, _ = cv2.projectPoints(np.asarray(pts, float), rvec, t, K, np.zeros(5))
    return img.reshape(-1, 2)


def _robot_corners_map(cfg, x, y, heading_deg):
    """로봇 전진 방향 heading 일 때 마커 코너(map). 마커 로컬 +x 는 heading - yaw_offset."""
    a = math.radians(heading_deg - cfg.aruco.yaw_offset_deg)
    ux, uy = math.cos(a), math.sin(a)          # 마커 로컬 +x (코너0 -> 코너1)
    vx, vy = -uy, ux                           # 마커 로컬 +y (종이 위쪽)
    h = cfg.aruco.robot_marker_size_m / 2.0
    z = cfg.aruco.robot_marker_height_m
    local = [(-h, +h), (+h, +h), (+h, -h), (-h, -h)]
    return [(x + lx * ux + ly * vx, y + lx * uy + ly * vy, z) for lx, ly in local]


def test_synthetic_two_camera_pose(cfg):
    K = approx_camera_matrix(1280, 720, 70.4)
    placements = [((0.9, -0.05, 1.30), (0.9, 0.9, 0.0)), ((0.9, 1.85, 1.30), (0.9, 0.9, 0.0))]
    floor = floor_object_points(cfg.aruco)
    cams, dets_for = [], []
    truth = (0.62, 0.83, 37.0)
    for i, (c, look) in enumerate(placements):
        R, t = _camera_pose(c, look)
        cam = Camera(f"cam{i}", K, np.zeros((5, 1)), cfg.aruco, calibrated=True)
        det = {mid: _project(K, R, t, pts) for mid, pts in floor.items()}
        det[cfg.aruco.robot_marker_id] = _project(K, R, t, _robot_corners_map(cfg, *truth))
        cams.append(cam)
        dets_for.append(det)

    loc = RobotLocalizer(cfg.aruco)
    pose = loc.update(cams, dets_for)
    assert pose.ok and pose.n_cams == 2
    assert abs(pose.x - truth[0]) < 0.003 and abs(pose.y - truth[1]) < 0.003
    assert abs((pose.yaw_deg - truth[2] + 180) % 360 - 180) < 0.5
    assert all(c.reproj_px < 0.5 for c in cams)


def test_pose_hold_then_lost(cfg):
    t = [0.0]
    loc = RobotLocalizer(cfg.aruco, clock=lambda: t[0])
    loc._last = type(loc._last)(1.0, 1.0, 0.0, ok=True)
    loc._last_t = 0.0
    t[0] = cfg.aruco.pose_hold_s * 0.5
    held = loc.update([], [])
    assert held.ok and not held.fresh
    t[0] = cfg.aruco.pose_hold_s * 1.5
    assert not loc.update([], []).ok
