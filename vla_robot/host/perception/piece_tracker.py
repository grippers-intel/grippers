"""검출(bbox) -> 지도 좌표 기물, 프레임 간 추적.

바닥 접점 근사: bbox **아래쪽 중앙** 픽셀을 z=0 평면으로 쏜다. 카메라가 35~59° 로
내려다보므로 물체가 바닥에 닿는 점이 대략 거기다. bbox 중심을 쓰면 물체 높이만큼
카메라 쪽으로 밀려 보인다.

추적을 하는 이유 (기존 PieceTracker 와 같다)
  1) 한 카메라만 놓쳐도 그 프레임에 사라지는 깜빡임
  2) 같은 물체가 프레임마다 다른 라벨로 튀어 "두 개"로 보이는 문제
  3) 한 프레임짜리 오검출 유령
-> 트랙은 라벨이 아니라 **위치**로 잡고, 라벨은 감쇠 가중 다수결로 확정한다.
   생긴 지 confirm_s 가 안 된 트랙은 내보내지 않는다. "관측 횟수"가 아니라 "시간"으로
   재는 이유: 검출 워커가 느려서 메인 루프는 같은 캐시 결과를 수십 번 읽는다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

XY = tuple[float, float]


@dataclass(frozen=True)
class PieceObs:
    label: str
    x: float
    y: float
    confidence: float
    cam_name: str


def away_from_camera(cam_xy, pt_xy, r: float) -> XY:
    """바닥 접점(카메라에 가장 가까운 가장자리)을 기물 중심으로 — 카메라에서 멀어지는 쪽으로 r 만큼.

    bbox 아래쪽 중앙은 물체가 바닥에 닿는 점 중 **카메라에 가장 가까운** 점이다. 작은 기물(체스 말
    밑면 ~2 cm)은 상관없지만 넓은 기물은 중심이 그만큼 카메라 반대쪽에 있다. 2026-10-05: 두 카메라
    사이(장판 가운데)의 별은 보는 카메라에 따라 위치가 서로 반대로 쏠려 파지 거리가 계속 짧았다.
    """
    dx, dy = pt_xy[0] - cam_xy[0], pt_xy[1] - cam_xy[1]
    n = math.hypot(dx, dy)
    if r <= 0 or n < 1e-6:
        return float(pt_xy[0]), float(pt_xy[1])
    return float(pt_xy[0] + r * dx / n), float(pt_xy[1] + r * dy / n)


def observations_from_detections(cam, detections, conf_threshold: float,
                                 radius_by_label: dict | None = None) -> list[PieceObs]:
    """카메라 외부파라미터가 아직 안 풀렸으면 빈 목록 — 위치를 알 수 없다.

    radius_by_label: 넓은 기물의 바닥 반경(m). 바닥 접점을 그만큼 카메라 반대쪽으로 옮겨 중심으로 쓴다.
    """
    if detections is None or cam is None or not cam.ready:
        return []
    radius_by_label = radius_by_label or {}
    out = []
    for d in detections:
        if d.confidence < conf_threshold:
            continue
        pt = cam.pixels_to_plane(np.array([d.bottom_center]), z=0.0)
        if pt is None:
            continue
        x, y = float(pt[0, 0]), float(pt[0, 1])
        r = float(radius_by_label.get(d.label, 0.0))
        center = getattr(cam, "center", None)
        if r > 0 and center is not None:
            x, y = away_from_camera((float(center[0]), float(center[1])), (x, y), r)
        out.append(PieceObs(d.label, x, y, float(d.confidence), cam.name))
    return out


@dataclass
class _Track:
    x: float
    y: float
    first_seen: float
    last_seen: float = 0.0
    n_obs: int = 0
    label_scores: dict = field(default_factory=dict)

    @property
    def label(self) -> str:
        return max(self.label_scores, key=self.label_scores.get)

    @property
    def score(self) -> float:
        return sum(self.label_scores.values())


class PieceTracker:
    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self._tracks: list[_Track] = []

    def reset(self) -> None:
        self._tracks = []

    def update(self, obs_lists, now: float) -> dict[str, list[XY]]:
        cfg = self.cfg
        for t in self._tracks:
            for k in t.label_scores:
                t.label_scores[k] *= cfg.label_decay

        for obs in (o for lst in obs_lists for o in lst):
            best, best_d = None, cfg.merge_dist_m
            for t in self._tracks:
                d = math.hypot(obs.x - t.x, obs.y - t.y)
                if d <= best_d:
                    best, best_d = t, d
            if best is None:
                best = _Track(obs.x, obs.y, first_seen=now)
                self._tracks.append(best)
            # 관측이 쌓일수록 덜 흔들리는 지수 이동평균
            alpha = 1.0 / (best.n_obs + 1) if best.n_obs < 5 else 0.2
            best.x += (obs.x - best.x) * alpha
            best.y += (obs.y - best.y) * alpha
            best.label_scores[obs.label] = best.label_scores.get(obs.label, 0.0) + obs.confidence
            best.last_seen = now
            best.n_obs += 1

        self._tracks = [t for t in self._tracks if now - t.last_seen <= cfg.hold_s]

        by_label: dict[str, list[_Track]] = {}
        for t in self._tracks:
            if now - t.first_seen < cfg.confirm_s or not t.label_scores:
                continue
            by_label.setdefault(t.label, []).append(t)
        result: dict[str, list[XY]] = {}
        for label, tracks in by_label.items():
            tracks.sort(key=lambda t: t.score, reverse=True)
            result[label] = [(t.x, t.y) for t in tracks[:cfg.max_per_label]]
        return result
