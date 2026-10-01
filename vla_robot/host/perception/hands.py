"""탑뷰 프레임에서 사람 손을 찾는다 — MediaPipe HandLandmarker(학습 없이 쓰는 기성 모델).

손은 **가져다줄 곳**이다. 기물 목록·장애물에 넣지 않고 따로 낸다.

- 넌블로킹: Geti 검출기처럼 카메라마다 스레드 하나가 최근 프레임만 본다. 손 랜드마커도
  카메라마다 따로 만든다(한 인스턴스를 두 스레드가 같이 부르지 않게).
- 좌표: 손바닥 중심(손목 + 손가락 뿌리 4개 평균)의 광선. 두 카메라가 같은 손을 보면 두 광선의
  교차점(높이까지), 한 대만 보면 `hand_z_m` 평면(받는 자세 30~35 cm)과의 교차점 — 바닥으로 풀면
  수십 cm 밀린다.
- 거르기: 가장자리 띠(`edges`, `edge_band_m`) 밖은 버린다. 장판 안쪽은 기물 자리다.
  지속 시간(`confirm_s`)은 PieceTracker 를 그대로 쓴다(라벨 "hand" 하나).
- mediapipe 는 **선택 의존성**이다. 없거나 모델 파일이 없으면 경고 후 손 없이 돈다.

2026-10-01 실측: full 프레임(1280×720) ~30 ms/카메라, 앞·양옆 9곳 모두 검출.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Optional

import numpy as np

from perception.piece_tracker import PieceObs, PieceTracker

HAND_LABEL = "hand"
PALM_IDX = (0, 5, 9, 13, 17)          # 손목 + 검지~새끼 뿌리
# 손 촬영 계획(2026-10-01)의 손 위치 9곳, 손 중심 (m). 화면 표시용 이름일 뿐이다.
HAND_SPOTS = {
    "F1": (0.33, 0.12), "F2": (0.99, 0.12), "F3": (1.65, 0.12),
    "L1": (0.075, 0.26), "L2": (0.075, 0.915), "L3": (0.075, 1.56),
    "R1": (1.905, 0.26), "R2": (1.905, 0.915), "R3": (1.905, 1.56),
}
SPOT_RADIUS_M = 0.25


@dataclass(frozen=True)
class HandSighting:
    pts: np.ndarray        # (21, 2) px
    score: float

    @property
    def palm(self) -> np.ndarray:
        return self.pts[list(PALM_IDX)].mean(axis=0)


def make_landmarker(model_path, num_hands: int, min_conf: float):
    from mediapipe.tasks.python import BaseOptions, vision
    opts = vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(model_path)),
        num_hands=num_hands, min_hand_detection_confidence=min_conf,
        min_hand_presence_confidence=min_conf)
    return vision.HandLandmarker.create_from_options(opts)


def run_landmarker(landmarker, bgr: np.ndarray, ox: int = 0, oy: int = 0) -> list[HandSighting]:
    """BGR 프레임(또는 조각) 한 장. (ox, oy) 는 조각의 원본 좌표 오프셋."""
    import cv2
    import mediapipe as mp
    h, w = bgr.shape[:2]
    rgb = np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    res = landmarker.detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb))
    out = []
    for lms, hd in zip(res.hand_landmarks, res.handedness):
        pts = np.array([[p.x * w + ox, p.y * h + oy] for p in lms])
        out.append(HandSighting(pts, float(hd[0].score) if hd else 0.0))
    return out


def nearest_spot(xy) -> str:
    if xy is None:
        return "-"
    name, (sx, sy) = min(HAND_SPOTS.items(),
                         key=lambda kv: np.hypot(kv[1][0] - xy[0], kv[1][1] - xy[1]))
    return name if np.hypot(sx - xy[0], sy - xy[1]) <= SPOT_RADIUS_M else "?"


def in_hand_zone(hcfg, arena, x: float, y: float) -> bool:
    """장판(+outside_m) 안이면서, 고른 가장자리 중 하나에서 edge_band_m 안쪽."""
    (x0, x1), (y0, y1) = arena.wall_x, arena.wall_y
    o = hcfg.outside_m
    if not (x0 - o <= x <= x1 + o and y0 - o <= y <= y1 + o):
        return False
    dist = {"front": y - y0, "back": y1 - y, "left": x - x0, "right": x1 - x}
    return any(dist[e] <= hcfg.edge_band_m for e in hcfg.edges)


@dataclass(frozen=True)
class HandPoint:
    x: float
    y: float
    z: float
    score: float
    cams: str              # "cam0+cam1" = 두 카메라 교차, "cam1" = 한 대(높이 가정)


def _ray(cam, px) -> tuple[np.ndarray, np.ndarray]:
    """픽셀 -> map 광선 (원점 = 카메라 중심, 방향 단위벡터)."""
    import cv2
    norm = cv2.undistortPoints(np.asarray(px, np.float64).reshape(1, 1, 2), cam.K, cam.dist).reshape(2)
    d = cam.R.T @ np.array([norm[0], norm[1], 1.0])
    return np.asarray(cam.center, float).reshape(3), d / np.linalg.norm(d)


def _closest(p1, d1, p2, d2):
    """두 광선의 가장 가까운 두 점의 중점과 그 사이 거리. 평행이거나 카메라 뒤면 None."""
    w0 = p1 - p2
    b, d, e = d1 @ d2, d1 @ w0, d2 @ w0
    den = 1.0 - b * b
    if den < 1e-9:
        return None
    s, t = (b * e - d) / den, (e - b * d) / den
    if s <= 0 or t <= 0:
        return None
    q1, q2 = p1 + s * d1, p2 + t * d2
    return (q1 + q2) / 2.0, float(np.linalg.norm(q1 - q2))


def locate_hands(cams, sightings_per_cam, hcfg) -> list[HandPoint]:
    """두 카메라가 같은 손을 보면 **광선 교차로 높이까지** 푼다(2026-10-01: 손을 45 cm 로 들자
    고정 높이 0.32 로 푼 두 카메라 위치가 y 로 24 cm 갈라져 손이 둘로 보였다 — 각 카메라가
    자기에게서 먼 쪽으로 민다). 짝이 없는 손만 `hand_z_m` 평면으로 푼다."""
    rays = []                                   # (카메라 번호, 광선, 손)
    for ci, (cam, sightings) in enumerate(zip(cams, sightings_per_cam)):
        if not sightings or cam is None or not cam.ready:
            continue
        for s in sightings:
            if s.score >= hcfg.min_conf:
                rays.append((ci, _ray(cam, s.palm), s, cam))
    pairs = []
    for i, (ca, ra, sa, _) in enumerate(rays):
        for j in range(i + 1, len(rays)):
            cb, rb, sb, _ = rays[j]
            if ca == cb:
                continue
            hit = _closest(*ra, *rb)
            if hit is not None and hit[1] <= hcfg.pair_max_gap_m and 0.0 < hit[0][2] <= hcfg.max_z_m:
                pairs.append((hit[1], i, j, hit[0]))
    out, used = [], set()
    for _gap, i, j, p in sorted(pairs, key=lambda q: q[0]):
        if i in used or j in used:
            continue
        used |= {i, j}
        out.append(HandPoint(float(p[0]), float(p[1]), float(p[2]),
                             max(rays[i][2].score, rays[j][2].score),
                             f"{rays[i][3].name}+{rays[j][3].name}"))
    for k, (_ci, _r, s, cam) in enumerate(rays):
        if k in used:
            continue
        pt = cam.pixels_to_plane(s.palm.reshape(1, 2), z=hcfg.hand_z_m)
        if pt is not None:
            out.append(HandPoint(float(pt[0, 0]), float(pt[0, 1]), hcfg.hand_z_m, s.score, cam.name))
    return out


def hand_observations(cams, sightings_per_cam, hcfg, arena) -> list[PieceObs]:
    """카메라별 손 검출 -> 가장자리 띠 안의 손 관측. 외부파라미터가 안 풀린 카메라는 건너뛴다."""
    return [PieceObs(HAND_LABEL, h.x, h.y, h.score, h.cams)
            for h in locate_hands(cams, sightings_per_cam, hcfg)
            if in_hand_zone(hcfg, arena, h.x, h.y)]


def make_hand_tracker(hcfg, tracker_cfg) -> PieceTracker:
    """기물 추적기를 손 값으로 쓴다 — 위치로 묶고, confirm_s 동안 계속 보여야 내보낸다."""
    return PieceTracker(replace(tracker_cfg, merge_dist_m=hcfg.merge_dist_m, hold_s=hcfg.hold_s,
                                confirm_s=hcfg.confirm_s, max_per_label=hcfg.max_hands))


class _HandWorker:
    def __init__(self, landmarker, name: str, interval_s: float) -> None:
        self._lm = landmarker
        self.name = name
        self._interval = interval_s
        self._frame: Optional[np.ndarray] = None
        self._result: Optional[list[HandSighting]] = None
        self._lock = threading.Lock()
        self._new = threading.Event()
        self._stop = False
        self._last = 0.0
        self._thread = threading.Thread(target=self._run, name=f"hands-{name}", daemon=True)
        self._thread.start()

    def submit(self, frame_bgr: np.ndarray) -> None:
        with self._lock:
            self._frame = frame_bgr
        self._new.set()

    def latest(self) -> Optional[list[HandSighting]]:
        with self._lock:
            return self._result

    def stop(self) -> None:
        self._stop = True
        self._new.set()
        self._thread.join(timeout=2.0)
        try:
            self._lm.close()
        except Exception:  # noqa: BLE001 — 종료 경로
            pass

    def _run(self) -> None:
        while not self._stop:
            if not self._new.wait(timeout=0.5):
                continue
            self._new.clear()
            wait_left = self._interval - (time.monotonic() - self._last)
            while wait_left > 0 and not self._stop:
                time.sleep(min(wait_left, 0.05))
                wait_left -= 0.05
            with self._lock:
                frame = self._frame
            if frame is None or self._stop:
                continue
            try:
                result = run_landmarker(self._lm, frame)
            except Exception as exc:  # noqa: BLE001 — 스레드가 죽으면 손이 영영 안 보인다
                print(f"[hands] ⚠️ {self.name}: 손 검출 오류 — {exc}")
                continue
            finally:
                self._last = time.monotonic()
            with self._lock:
                self._result = result


class HandDetector:
    """카메라별 손 검출 스레드. `ok` 가 False 면 아무것도 하지 않는다."""

    def __init__(self, hcfg, cam_indices) -> None:
        self._workers: dict[int, _HandWorker] = {}
        self.ok = False
        if not hcfg.enabled:
            return
        try:
            import mediapipe  # noqa: F401
        except ImportError:
            print("[hands] ⚠️ mediapipe 없음 — 손 검출 없이 돕니다 (pip install mediapipe)")
            return
        if not Path(hcfg.model_path).exists():
            print(f"[hands] ⚠️ {hcfg.model_path} 없음 — 손 검출 없이 돕니다 "
                  "(tools/hand_probe.py 설명의 주소에서 받기)")
            return
        for idx in cam_indices:
            lm = make_landmarker(hcfg.model_path, hcfg.num_hands, hcfg.min_conf)
            self._workers[int(idx)] = _HandWorker(lm, f"cam{idx}", hcfg.min_infer_interval_s)
        self.ok = True

    def submit(self, cam_index: int, frame_bgr: np.ndarray) -> None:
        w = self._workers.get(int(cam_index))
        if w is not None:
            w.submit(frame_bgr.copy())

    def latest(self, cam_index: int) -> Optional[list[HandSighting]]:
        w = self._workers.get(int(cam_index))
        return None if w is None else w.latest()

    def close(self) -> None:
        for w in self._workers.values():
            w.stop()
