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


def observations_from_detections(cam, detections, conf_threshold: float) -> list[PieceObs]:
    """카메라 외부파라미터가 아직 안 풀렸으면 빈 목록 — 위치를 알 수 없다."""
    if detections is None or cam is None or not cam.ready:
        return []
    out = []
    for d in detections:
        if d.confidence < conf_threshold:
            continue
        pt = cam.pixels_to_plane(np.array([d.bottom_center]), z=0.0)
        if pt is None:
            continue
        out.append(PieceObs(d.label, float(pt[0, 0]), float(pt[0, 1]), float(d.confidence), cam.name))
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
