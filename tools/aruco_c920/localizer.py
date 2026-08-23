"""ArUco 인식 → 외부파라미터 → z 구속 실좌표 변환.

OpenCV 5.0.0 기준으로 작성했다. 5.0 에서는 4.6 이하의
aruco.Dictionary_get / aruco.detectMarkers 뿐 아니라
aruco.estimatePoseSingleMarkers 까지 제거되었으므로,
ArucoDetector + cv2.solvePnP 를 직접 조합한다.

핵심 아이디어
-------------
1) 바닥 마커 4점은 map 좌표를 아는 z = 0 평면 위의 점이다.
   → solvePnP 로 "카메라가 map 어디에 어떤 자세로 있는지"(외부파라미터)를 푼다.
   두 카메라가 같은 4점을 보므로 두 좌표계가 자동으로 통일된다.

2) 로봇 상판 마커는 지면이 아니라 z = ROBOT_MARKER_HEIGHT 에 떠 있다.
   지면 호모그래피에 그냥 넣으면 H/tan(고도각) 만큼 밀린다(최악 250 mm).
   → 코너 픽셀에서 광선을 쏴 z = 상판높이 평면과 교차시킨다.
   상판이 수평이면 네 코너가 전부 그 평면 위에 있으므로 z 가 정확히 구속되고,
   교점 4개의 평균이 곧 마커 중심이다.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np

import config as cfg


# ---------------------------------------------------------------------------
# 검출기
# ---------------------------------------------------------------------------
def make_detector() -> aruco.ArucoDetector:
    """OpenCV 4.7+ / 5.x 전용 ArucoDetector 생성."""
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, cfg.ARUCO_DICT))
    params = aruco.DetectorParameters()
    # 서브픽셀 코너 보정 — 30 mm 목표에서는 사실상 필수다.
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 50
    params.cornerRefinementMinAccuracy = 0.01
    # 원거리 작은 마커를 놓치지 않도록 하한을 조금 낮춘다.
    params.minMarkerPerimeterRate = 0.01
    return aruco.ArucoDetector(dictionary, params)


def detect(detector: aruco.ArucoDetector, gray: np.ndarray) -> dict[int, np.ndarray]:
    """{marker_id: (4,2) float64 코너} 로 반환. 중복 ID 는 첫 검출만 쓴다."""
    corners, ids, _ = detector.detectMarkers(gray)
    out: dict[int, np.ndarray] = {}
    if ids is None:
        return out
    for c, i in zip(corners, ids.ravel().tolist()):
        if i not in out:
            out[i] = c.reshape(4, 2).astype(np.float64)
    return out


# ---------------------------------------------------------------------------
# 카메라
# ---------------------------------------------------------------------------
@dataclass
class Camera:
    name: str
    K: np.ndarray                      # 3x3 내부파라미터
    dist: np.ndarray                   # 왜곡계수
    calibrated: bool = False

    # 외부파라미터 (map -> camera):  X_cam = R @ X_map + t
    R: np.ndarray | None = None
    t: np.ndarray | None = None
    center: np.ndarray | None = None   # map 좌표계에서의 카메라 위치
    reproj_px: float = float("inf")
    n_floor: int = 0

    # 외부파라미터 고정 상태
    locked: bool = False
    _acc: dict = field(default_factory=dict)   # {mid: [코너 (4,2) ...]}
    _acc_n: int = 0
    _bad_frames: int = 0

    @classmethod
    def load(cls, name: str, index: int) -> "Camera":
        path = Path(cfg.CALIB_DIR) / f"cam{index}.npz"
        if path.exists():
            d = np.load(path)
            return cls(name, d["K"].astype(np.float64),
                       d["dist"].astype(np.float64), calibrated=True)
        return cls(name, cfg.approx_camera_matrix(),
                   np.zeros((5, 1), dtype=np.float64), calibrated=False)

    # -- 외부파라미터 -------------------------------------------------------
    def solve_extrinsics(self, det: dict[int, np.ndarray]) -> bool:
        """바닥 마커들로 카메라의 map 상 자세를 푼다. 성공 여부 반환.

        카메라는 고정되어 있으므로, 4장이 모두 보이는 프레임을 모아 코너를
        평균낸 뒤 한 번 풀고 고정한다(config.EXTRINSIC_LOCK_FRAMES).
        고정 후에는 바닥 마커가 가려져도 로봇 추적이 흔들리지 않는다.
        """
        _, per_id = cfg.floor_object_points()
        self.n_floor = sum(1 for m in cfg.FLOOR_MARKER_IDS if m in det)

        if self.locked:
            return self._check_lock(det, per_id)

        # 고정 전: 마커가 최소 개수 이상 보이는 프레임을 모은다.
        #
        # 원래는 "4장이 다 보이는 프레임만" 모았는데, 카메라 배치에 따라
        # (예: 후퇴 0m — 카메라 한 대가 4점을 동시에 못 보고 항상 2점만
        # 보는 배치) 그 조건이 영영 안 채워질 수 있다. 그러면 고정이
        # 절대 안 걸려서, 바닥 마커가 전부 가려지는 순간 추적이 바로
        # 끊긴다 — 고정 기능이 있으나 마나 해진다. 그래서 "보이는 마커만"
        # 누적하고, 프레임당 최소 조건은 MIN_FLOOR_MARKERS 로 낮춘다.
        if cfg.EXTRINSIC_LOCK_FRAMES and self.n_floor >= cfg.MIN_FLOOR_MARKERS:
            for mid in cfg.FLOOR_MARKER_IDS:
                if mid in det:
                    self._acc.setdefault(mid, []).append(det[mid])
            self._acc_n += 1
            if self._acc_n >= cfg.EXTRINSIC_LOCK_FRAMES:
                avg = {m: np.mean(np.stack(v), axis=0) for m, v in self._acc.items()}
                if self._solve_from(avg, per_id):
                    self.locked = True
                    self._acc.clear()
                    return True

        return self._solve_from(det, per_id)

    def _check_lock(self, det, per_id) -> bool:
        """고정된 외부파라미터가 아직 맞는지 검산. 틀어졌으면 고정을 푼다."""
        obj, img = [], []
        for mid in cfg.FLOOR_MARKER_IDS:
            if mid in det:
                obj.append(per_id[mid])
                img.append(det[mid])
        if not obj:
            return True          # 바닥 마커가 하나도 안 보여도 고정값을 그대로 쓴다

        rvec, _ = cv2.Rodrigues(self.R)
        proj, _ = cv2.projectPoints(np.vstack(obj), rvec, self.t, self.K, self.dist)
        err = float(np.sqrt(((proj.reshape(-1, 2) - np.vstack(img)) ** 2)
                            .sum(axis=1).mean()))
        self.reproj_px = err
        if err > cfg.EXTRINSIC_RELOCK_PX:
            self._bad_frames += 1
            if self._bad_frames >= cfg.EXTRINSIC_RELOCK_FRAMES:
                # 카메라가 움직였다고 보고 처음부터 다시
                self.locked = False
                self._acc.clear()
                self._acc_n = 0
                self._bad_frames = 0
        else:
            self._bad_frames = 0
        return True

    def _solve_from(self, det: dict[int, np.ndarray], per_id) -> bool:
        obj, img = [], []
        for mid in cfg.FLOOR_MARKER_IDS:
            if mid in det:
                obj.append(per_id[mid])
                img.append(det[mid])
        if len(obj) < cfg.MIN_FLOOR_MARKERS:
            self.reproj_px = float("inf")
            return False

        obj = np.vstack(obj)
        img = np.vstack(img)

        # 바닥 마커는 전부 z = 0 평면 위(평면 PnP)다.
        #
        # ⚠️ SOLVEPNP_IPPE 를 단독으로 쓰지 말 것. 평면 PnP 는 원리상 해가 2개인데,
        #    IPPE 는 카메라가 어느 쪽에서 보느냐에 따라 두 해 모두 빗나가는 경우가
        #    있다. 실측 배치(마주보는 두 변)에서 반대편 카메라가 정확히 그 경우로,
        #    합성 검증 시 IPPE 는 재투영 64.7 px, SQPNP 는 0.000 px 였다.
        #    → 전역해를 주는 SQPNP 를 주력으로 쓰고, IPPE 는 후보로만 더한다.
        cand_r: list[np.ndarray] = []
        cand_t: list[np.ndarray] = []
        for flag in (cv2.SOLVEPNP_SQPNP, cv2.SOLVEPNP_IPPE):
            try:
                n, rvs, tvs, _ = cv2.solvePnPGeneric(obj, img, self.K, self.dist,
                                                     flags=flag)
            except cv2.error:
                continue
            if n:
                cand_r.extend(rvs)
                cand_t.extend(tvs)
        if not cand_r:
            self.reproj_px = float("inf")
            return False

        best = None
        for rv, tv in zip(cand_r, cand_t):
            rv, tv = cv2.solvePnPRefineLM(obj, img, self.K, self.dist, rv, tv)
            proj, _ = cv2.projectPoints(obj, rv, tv, self.K, self.dist)
            err = float(np.sqrt(((proj.reshape(-1, 2) - img) ** 2).sum(axis=1).mean()))
            R, _ = cv2.Rodrigues(rv)
            center = (-R.T @ tv.reshape(3, 1)).ravel()
            # 카메라는 바닥 위에 있다 — 뒤집힌 해를 걸러내는 물리 조건
            if center[2] <= 0:
                continue
            if best is None or err < best[0]:
                best = (err, R, tv.reshape(3, 1), center)

        if best is None:
            self.reproj_px = float("inf")
            return False

        self.reproj_px, self.R, self.t, self.center = best
        return self.reproj_px <= cfg.MAX_EXTRINSIC_REPROJ_PX

    @property
    def ready(self) -> bool:
        if self.R is None:
            return False
        if self.locked:
            # 고정된 값은 바닥 마커가 하나도 안 보여도 그대로 쓴다.
            # 카메라가 움직였다면 _check_lock 이 고정을 풀어 준다.
            return True
        return self.reproj_px <= cfg.MAX_EXTRINSIC_REPROJ_PX

    # -- 광선 - 평면 교차 ---------------------------------------------------
    def pixels_to_plane(self, px: np.ndarray, z: float) -> np.ndarray | None:
        """픽셀 (N,2) 를 map 의 z = const 평면 위 (N,3) 점으로 변환.

        이것이 "z 구속"의 실체다. 깊이를 추정하지 않고, 높이를 안다는 사실로
        광선 위의 한 점을 확정한다.
        """
        if self.R is None:
            return None
        src = np.asarray(px, dtype=np.float64).reshape(-1, 1, 2)
        norm = cv2.undistortPoints(src, self.K, self.dist).reshape(-1, 2)

        # 카메라 좌표계 광선 방향 → map 좌표계로 회전
        d_cam = np.hstack([norm, np.ones((len(norm), 1))])       # (N,3)
        d_map = (self.R.T @ d_cam.T).T                            # (N,3)

        cz, dz = self.center[2], d_map[:, 2]
        # 내려다보는 카메라이므로 dz < 0 이어야 한다. 0 에 가까우면 지평선 근처.
        if np.any(np.abs(dz) < 1e-6):
            return None
        s = (z - cz) / dz
        if np.any(s <= 0):          # 카메라 뒤쪽으로 나가면 무효
            return None
        return self.center[None, :] + s[:, None] * d_map


# ---------------------------------------------------------------------------
# 포즈
# ---------------------------------------------------------------------------
@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw_deg: float = 0.0
    ok: bool = False
    n_cams: int = 0
    age_s: float = 0.0          # 마지막 실제 관측으로부터 경과 시간
    fresh: bool = False         # 이번 프레임에 실제로 봤는가
    per_cam: dict = field(default_factory=dict)

    def __str__(self) -> str:
        if not self.ok:
            return "pose: LOST"
        tag = "" if self.fresh else f" (HOLD {self.age_s:.2f}s)"
        return (f"x={self.x * 1000:7.1f}mm  y={self.y * 1000:7.1f}mm  "
                f"yaw={self.yaw_deg:6.1f}deg  cams={self.n_cams}{tag}")


def _pose_from_corners(cam: Camera, corners: np.ndarray) -> tuple[float, float, float] | None:
    """한 카메라의 로봇 마커 코너에서 (x, y, yaw_rad) 를 뽑는다."""
    pts = cam.pixels_to_plane(corners, cfg.ROBOT_MARKER_HEIGHT)
    if pts is None:
        return None
    cx, cy = pts[:, 0].mean(), pts[:, 1].mean()

    # 마커 로컬 +x 축 = (코너0→코너1) 과 (코너3→코너2) 의 평균
    e1 = pts[1] - pts[0]
    e2 = pts[2] - pts[3]
    vx, vy = (e1[0] + e2[0]) / 2.0, (e1[1] + e2[1]) / 2.0
    if abs(vx) < 1e-9 and abs(vy) < 1e-9:
        return None
    return float(cx), float(cy), float(np.arctan2(vy, vx))


class RobotLocalizer:
    """두 카메라의 관측을 합쳐 로봇 (x, y, yaw) 를 낸다. 폴백 포함."""

    def __init__(self) -> None:
        self._last = Pose()
        self._last_t = 0.0

    def update(self, cams: list[Camera], dets: list[dict[int, np.ndarray]]) -> Pose:
        now = time.monotonic()
        xs, ys, cs, ss, ws, per_cam = [], [], [], [], [], {}

        for cam, det in zip(cams, dets):
            cam.solve_extrinsics(det)
            if not cam.ready or cfg.ROBOT_MARKER_ID not in det:
                continue
            got = _pose_from_corners(cam, det[cfg.ROBOT_MARKER_ID])
            if got is None:
                continue
            x, y, yaw = got
            # 재투영 오차가 작은 카메라에 더 무게를 준다.
            w = 1.0 / max(cam.reproj_px, 0.05)
            xs.append(x * w); ys.append(y * w); ws.append(w)
            cs.append(np.cos(yaw) * w); ss.append(np.sin(yaw) * w)
            per_cam[cam.name] = (x, y, np.degrees(yaw))

        if ws:
            wsum = float(np.sum(ws))
            yaw = float(np.arctan2(np.sum(ss) / wsum, np.sum(cs) / wsum))
            pose = Pose(
                x=float(np.sum(xs) / wsum),
                y=float(np.sum(ys) / wsum),
                yaw_deg=(np.degrees(yaw) + cfg.YAW_OFFSET_DEG + 180.0) % 360.0 - 180.0,
                ok=True, n_cams=len(ws), age_s=0.0, fresh=True, per_cam=per_cam,
            )
            self._last, self._last_t = pose, now
            return pose

        # --- 폴백: 마지막 값 유지 -----------------------------------------
        if self._last.ok:
            age = now - self._last_t
            held = Pose(self._last.x, self._last.y, self._last.yaw_deg,
                        ok=age <= cfg.POSE_HOLD_SEC, n_cams=0,
                        age_s=age, fresh=False, per_cam=self._last.per_cam)
            if not held.ok:
                self._last = Pose()
            return held
        return Pose()


# ---------------------------------------------------------------------------
# 미션 코드가 쓰는 API
#
# 여기까지가 "로봇이 어디 있나"를 내는 부분이고, 아래 둘은 그 결과를 주행
# 명령으로 옮기기 위한 도우미다. 예: 장난감 상자 앞으로 가려면
#     fwd, left, bearing = relative_to_robot(pose, box_pose("toy"))
# ---------------------------------------------------------------------------
def box_pose(name: str) -> tuple[float, float, float]:
    """상자의 (x, y, yaw_deg). config.BOXES 에 적어 둔 값을 그대로 돌려준다."""
    if name not in cfg.BOXES:
        raise KeyError(f"알 수 없는 박스: {name} (가능: {list(cfg.BOXES)})")
    return cfg.BOXES[name]


def relative_to_robot(pose: Pose, target: tuple[float, float, float]
                      ) -> tuple[float, float, float]:
    """로봇 기준 상대좌표 (전방 거리 m, 좌측 거리 m, 상대 방위각 deg).

    bearing 이 0 이면 목표가 정면, +90 이면 왼쪽, -90 이면 오른쪽이다.
    """
    dx, dy = target[0] - pose.x, target[1] - pose.y
    th = np.radians(pose.yaw_deg)
    fwd = dx * np.cos(th) + dy * np.sin(th)
    left = -dx * np.sin(th) + dy * np.cos(th)
    bearing = (np.degrees(np.arctan2(left, fwd)) + 180.0) % 360.0 - 180.0
    return float(fwd), float(left), float(bearing)
