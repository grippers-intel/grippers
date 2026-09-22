"""ArUco -> 카메라 외부파라미터 -> z 구속 로봇 pose.

핵심 (기존 localizer.py 와 같은 원리)
1) 바닥 마커 네 장은 map 좌표를 아는 z=0 평면 위 점이다. solvePnP 로 카메라의 map
   상 자세를 푼다. 두 카메라가 같은 기준점을 보므로 좌표계가 저절로 통일된다.
2) 로봇 마커는 z = robot_marker_height 에 떠 있다. 지면 호모그래피로 풀면
   H/tan(고도각)만큼 밀린다(최악 250 mm). 그래서 코너 픽셀의 광선을 그 높이 평면과
   교차시킨다 — 깊이를 추정하지 않고 "높이를 안다"로 점을 확정한다.

OpenCV 4.7+/5.x 전용: ArucoDetector + solvePnPGeneric (옛 detectMarkers/
estimatePoseSingleMarkers 는 5.0 에서 제거됐다).
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np

from localization.pose import Pose


# ---------------------------------------------------------------------------
# 검출기
# ---------------------------------------------------------------------------
def make_detector(aruco_cfg) -> aruco.ArucoDetector:
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, aruco_cfg.dictionary))
    params = aruco.DetectorParameters()
    # 서브픽셀 코너 보정 — mm 단위 목표에서는 사실상 필수다.
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 50
    params.cornerRefinementMinAccuracy = 0.01
    # 원거리 작은 로봇 마커(화면 33~48 px)를 놓치지 않게 하한을 낮춘다.
    params.minMarkerPerimeterRate = 0.01
    return aruco.ArucoDetector(dictionary, params)


def detect(detector: aruco.ArucoDetector, gray: np.ndarray) -> dict[int, np.ndarray]:
    """{marker_id: (4,2) float64 코너}. 중복 ID 는 첫 검출만 쓴다."""
    corners, ids, _ = detector.detectMarkers(gray)
    out: dict[int, np.ndarray] = {}
    if ids is None:
        return out
    for c, i in zip(corners, ids.ravel().tolist()):
        if i not in out:
            out[int(i)] = c.reshape(4, 2).astype(np.float64)
    return out


def floor_object_points(aruco_cfg) -> dict[int, np.ndarray]:
    """바닥 마커 코너 4점(z=0). 순서는 ArUco 검출 순서: 좌상, 우상, 우하, 좌하
    (인쇄물 윗변이 +y 를 향하게 붙였다는 전제 — 한 장이라도 돌아가 있으면
    reproj 가 수십 px 로 뜬다)."""
    h = aruco_cfg.floor_marker_size_m / 2.0
    return {
        mid: np.array([[cx - h, cy + h, 0.0], [cx + h, cy + h, 0.0],
                       [cx + h, cy - h, 0.0], [cx - h, cy - h, 0.0]], dtype=np.float64)
        for mid, (cx, cy) in aruco_cfg.floor_markers.items()
    }


def approx_camera_matrix(w: int, h: int, hfov_deg: float) -> np.ndarray:
    fx = (w / 2.0) / np.tan(np.radians(hfov_deg) / 2.0)
    return np.array([[fx, 0.0, w / 2.0], [0.0, fx, h / 2.0], [0.0, 0.0, 1.0]])


# ---------------------------------------------------------------------------
# 카메라
# ---------------------------------------------------------------------------
@dataclass
class Camera:
    name: str
    K: np.ndarray
    dist: np.ndarray
    aruco_cfg: object
    calibrated: bool = False

    # 외부파라미터 (map -> camera):  X_cam = R @ X_map + t
    R: np.ndarray | None = None
    t: np.ndarray | None = None
    center: np.ndarray | None = None
    reproj_px: float = float("inf")
    n_floor: int = 0

    locked: bool = False
    acc_n: int = 0
    _acc: dict = field(default_factory=dict)
    _bad_frames: int = 0

    @classmethod
    def load(cls, index: int, cameras_cfg, aruco_cfg) -> "Camera":
        path = Path(cameras_cfg.calib_dir) / f"cam{index}.npz"
        name = f"cam{index}"
        if path.exists():
            d = np.load(path)
            return cls(name, d["K"].astype(np.float64), d["dist"].astype(np.float64),
                       aruco_cfg, calibrated=True)
        # ⚠️ 조용히 근사로 넘어가면 캘리브레이션이 빠진 줄 모르고 돈다. 크게 알린다.
        print(f"[aruco] ⚠️⚠️ {path} 없음 — {name} 은 근사 내부파라미터(HFOV "
              f"{cameras_cfg.hfov_deg}°)로 돈다. 위치 오차가 수 cm 까지 커진다. "
              f"tools/calibrate_camera.py 로 캘리브레이션할 것")
        return cls(name, approx_camera_matrix(cameras_cfg.width, cameras_cfg.height,
                                              cameras_cfg.hfov_deg),
                   np.zeros((5, 1)), aruco_cfg, calibrated=False)

    # -- 외부파라미터 -------------------------------------------------------
    def solve_extrinsics(self, det: dict[int, np.ndarray]) -> bool:
        """카메라는 삼각대에 고정돼 있으므로 여러 프레임 코너를 평균내 한 번 풀고 고정한다.
            매 프레임 재계산: 오차 평균 0.77 mm · 최대 2.14 mm
            고정 사용       : 오차 평균 0.29 mm · 최대 0.70 mm
        고정 후에는 바닥 마커가 전부 가려져도 로봇 추적이 이어진다."""
        cfg = self.aruco_cfg
        per_id = floor_object_points(cfg)
        self.n_floor = sum(1 for m in per_id if m in det)
        if self.locked:
            return self._check_lock(det, per_id)
        # "4장 다 보이는 프레임만" 모으면 후퇴 0 m 배치(한 대가 항상 2장만 봄)에서
        # 영영 고정이 안 걸린다. 그래서 보이는 것만 누적한다.
        if cfg.extrinsic_lock_frames and self.n_floor >= cfg.min_floor_markers:
            for mid in per_id:
                if mid in det:
                    self._acc.setdefault(mid, []).append(det[mid])
            self.acc_n += 1
            if self.acc_n >= cfg.extrinsic_lock_frames:
                avg = {m: np.mean(np.stack(v), axis=0) for m, v in self._acc.items()}
                if self._solve_from(avg, per_id):
                    self.locked = True
                    self._acc.clear()
                    return True
        return self._solve_from(det, per_id)

    def _check_lock(self, det, per_id) -> bool:
        obj = [per_id[m] for m in per_id if m in det]
        img = [det[m] for m in per_id if m in det]
        if not obj:
            return True
        rvec, _ = cv2.Rodrigues(self.R)
        proj, _ = cv2.projectPoints(np.vstack(obj), rvec, self.t, self.K, self.dist)
        err = float(np.sqrt(((proj.reshape(-1, 2) - np.vstack(img)) ** 2).sum(axis=1).mean()))
        self.reproj_px = err
        if err > self.aruco_cfg.extrinsic_relock_px:
            self._bad_frames += 1
            if self._bad_frames >= self.aruco_cfg.extrinsic_relock_frames:
                print(f"[aruco] {self.name}: 고정값 재투영 {err:.1f}px 지속 — 카메라가 움직였다고 보고 다시 푼다")
                self.locked = False
                self._acc.clear()
                self.acc_n = 0
                self._bad_frames = 0
        else:
            self._bad_frames = 0
        return True

    def _solve_from(self, det, per_id) -> bool:
        cfg = self.aruco_cfg
        obj = [per_id[m] for m in per_id if m in det]
        img = [det[m] for m in per_id if m in det]
        if len(obj) < cfg.min_floor_markers:
            self.reproj_px = float("inf")
            return False
        obj = np.vstack(obj)
        img = np.vstack(img)
        # ⚠️ IPPE 단독 금지. 평면 PnP 는 해가 둘인데 마주보는 배치에서 IPPE 가 두 해 모두
        #    빗나갔다(합성 검증 IPPE 64.7 px vs SQPNP 0.000 px). SQPNP 를 주력, IPPE 는 후보.
        cand = []
        for flag in (cv2.SOLVEPNP_SQPNP, cv2.SOLVEPNP_IPPE):
            try:
                n, rvs, tvs, _ = cv2.solvePnPGeneric(obj, img, self.K, self.dist, flags=flag)
            except cv2.error:
                continue
            if n:
                cand.extend(zip(rvs, tvs))
        best = None
        for rv, tv in cand:
            rv, tv = cv2.solvePnPRefineLM(obj, img, self.K, self.dist, rv, tv)
            proj, _ = cv2.projectPoints(obj, rv, tv, self.K, self.dist)
            err = float(np.sqrt(((proj.reshape(-1, 2) - img) ** 2).sum(axis=1).mean()))
            R, _ = cv2.Rodrigues(rv)
            center = (-R.T @ tv.reshape(3, 1)).ravel()
            if center[2] <= 0:          # 카메라는 바닥 위에 있다 — 뒤집힌 해 제거
                continue
            if best is None or err < best[0]:
                best = (err, R, tv.reshape(3, 1), center)
        if best is None:
            self.reproj_px = float("inf")
            return False
        self.reproj_px, self.R, self.t, self.center = best
        return self.reproj_px <= cfg.max_extrinsic_reproj_px

    @property
    def ready(self) -> bool:
        if self.R is None:
            return False
        return self.locked or self.reproj_px <= self.aruco_cfg.max_extrinsic_reproj_px

    # -- 광선-평면 교차 ------------------------------------------------------
    def pixels_to_plane(self, px, z: float) -> np.ndarray | None:
        """픽셀 (N,2) -> map z=const 평면 위 (N,3)."""
        if self.R is None:
            return None
        src = np.asarray(px, dtype=np.float64).reshape(-1, 1, 2)
        norm = cv2.undistortPoints(src, self.K, self.dist).reshape(-1, 2)
        d_cam = np.hstack([norm, np.ones((len(norm), 1))])
        d_map = (self.R.T @ d_cam.T).T
        dz = d_map[:, 2]
        if np.any(np.abs(dz) < 1e-6):
            return None
        s = (z - self.center[2]) / dz
        if np.any(s <= 0):
            return None
        return self.center[None, :] + s[:, None] * d_map


# ---------------------------------------------------------------------------
# 로봇 pose
# ---------------------------------------------------------------------------
def _pose_from_corners(cam: Camera, corners: np.ndarray, height: float):
    pts = cam.pixels_to_plane(corners, height)
    if pts is None:
        return None
    cx, cy = pts[:, 0].mean(), pts[:, 1].mean()
    # 마커 로컬 +x = (코너0->코너1) 과 (코너3->코너2) 의 평균
    e1, e2 = pts[1] - pts[0], pts[2] - pts[3]
    vx, vy = (e1[0] + e2[0]) / 2.0, (e1[1] + e2[1]) / 2.0
    if abs(vx) < 1e-9 and abs(vy) < 1e-9:
        return None
    return float(cx), float(cy), float(np.arctan2(vy, vx))


class RobotLocalizer:
    """카메라들의 관측을 재투영 오차 역수로 가중 평균한다. 놓치면 pose_hold_s 동안 유지."""

    def __init__(self, aruco_cfg, clock=time.monotonic) -> None:
        self.cfg = aruco_cfg
        self._clock = clock
        self._last = Pose()
        self._last_t = 0.0

    def update(self, cams: list[Camera], dets: list[dict[int, np.ndarray]]) -> Pose:
        now = self._clock()
        xs, ys, cs, ss, ws, per_cam = [], [], [], [], [], {}
        for cam, det in zip(cams, dets):
            cam.solve_extrinsics(det)
            if not cam.ready or self.cfg.robot_marker_id not in det:
                continue
            got = _pose_from_corners(cam, det[self.cfg.robot_marker_id], self.cfg.robot_marker_height_m)
            if got is None:
                continue
            x, y, yaw = got
            w = 1.0 / max(cam.reproj_px, 0.05)
            xs.append(x * w)
            ys.append(y * w)
            cs.append(np.cos(yaw) * w)
            ss.append(np.sin(yaw) * w)
            ws.append(w)
            per_cam[cam.name] = (x, y, float(np.degrees(yaw)))

        if ws:
            wsum = float(np.sum(ws))
            yaw = float(np.arctan2(np.sum(ss) / wsum, np.sum(cs) / wsum))
            pose = Pose(x=float(np.sum(xs) / wsum), y=float(np.sum(ys) / wsum),
                        yaw_deg=(np.degrees(yaw) + self.cfg.yaw_offset_deg + 180.0) % 360.0 - 180.0,
                        ok=True, n_cams=len(ws), age_s=0.0, fresh=True, per_cam=per_cam)
            self._last, self._last_t = pose, now
            return pose

        if self._last.ok:
            age = now - self._last_t
            held = Pose(self._last.x, self._last.y, self._last.yaw_deg,
                        ok=age <= self.cfg.pose_hold_s, n_cams=0, age_s=age, fresh=False,
                        per_cam=self._last.per_cam)
            if not held.ok:
                self._last = Pose()
            return held
        return Pose()


def draw_overlay(frame, cam: Camera, det, pose: Pose, robot_id: int):
    """디버그용 카메라 오버레이."""
    for mid, c in det.items():
        pts = c.astype(np.int32)
        color = (0, 165, 255) if mid == robot_id else (0, 255, 0)
        cv2.polylines(frame, [pts], True, color, 2)
        cv2.putText(frame, str(mid), tuple(int(v) for v in pts[0]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    lock = "LOCKED" if cam.locked else f"locking {cam.acc_n}"
    text = (f"{cam.name} calib={'OK' if cam.calibrated else 'APPROX'} floor={cam.n_floor} "
            f"reproj={cam.reproj_px:.2f}px [{lock}]")
    cv2.putText(frame, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, str(pose), (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return frame
