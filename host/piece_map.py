"""geti 객체 인식 결과(Prediction)를 지도(map) 좌표로 바꾼다.

카메라 외부파라미터는 ArUco 가 바닥 마커로 이미 풀어놓은 것(localizer.Camera)을
그대로 재사용한다 — 기물 인식을 위해 따로 캘리브레이션할 게 없다.

바닥 접점 근사: 바운딩박스 "아래쪽 중앙" 픽셀을 z=0 평면으로 쏜다. 카메라가
비스듬히 내려다보므로(35~59도) 물체가 바닥에 닿는 지점은 대략 그 박스의
아래쪽 가장자리 중앙이다 — 감시카메라에서 사람 위치를 "발 위치"로 잡는 것과
같은 근사다. 박스 중심을 쓰면 기물 높이만큼 카메라 쪽으로 밀려 보인다.
기물별로 정확한 높이를 잰 값이 있으면(config.ROBOT_MARKER_HEIGHT 처럼) 나중에
라벨별 z 오프셋으로 보정할 수 있다 — 지금은 근사만 한다.

이 모듈은 추론을 직접 하지 않는다. geti_detector.GetiWorker 가 백그라운드에서
이미 계산해둔 Prediction 을 받아서 좌표만 바꾼다(중복 추론 방지).
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

import mission_config as mcfg

# config.py/localizer.py 는 aruco/ 하위폴더에 있다(팀원이 동기화하는 파일이라
# 건드리지 않는다) — 그 폴더를 경로에 추가해서 기존처럼 bare import 로 쓴다.
sys.path.insert(0, str(Path(__file__).parent / "aruco"))

from localizer import Camera

EMPTY_LABELS = {"No object", "Empty"}


@dataclass
class PieceObs:
    label: str
    x: float
    y: float
    confidence: float
    cam_name: str


def pieces_from_prediction(cam: Camera, prediction) -> list[PieceObs]:
    """한 카메라의 geti Prediction 을 지도 좌표 관측 목록으로 바꾼다.

    바닥 접점 근사: 바운딩박스 "아래쪽 중앙" 픽셀을 z=0 평면으로 쏜다. 카메라가
    비스듬히 내려다보므로(35~59도) 물체가 바닥에 닿는 지점은 대략 그 박스의
    아래쪽 가장자리 중앙이다. 박스 중심을 쓰면 기물 높이만큼 카메라 쪽으로
    밀려 보인다.

    cam.ready 가 아니면(외부파라미터를 아직 못 풀었으면) 빈 리스트를 낸다 —
    ArUco 가 바닥 기준점을 잡기 전에는 기물 위치도 알 수 없다.
    """
    if not cam.ready or prediction is None:
        return []

    out: list[PieceObs] = []
    for ann in prediction.annotations:
        label = max(ann.labels, key=lambda l: l.probability, default=None)
        if label is None or label.name in EMPTY_LABELS:
            continue
        if label.probability < mcfg.PIECE_CONF_THRESHOLD:
            continue

        shape = ann.shape
        px = np.array([[shape.x + shape.width / 2.0, shape.y + shape.height]])
        pt = cam.pixels_to_plane(px, z=0.0)
        if pt is None:
            continue
        out.append(PieceObs(label.name, float(pt[0, 0]), float(pt[0, 1]),
                             float(label.probability), cam.name))
    return out


def merge_observations(
    obs_lists: list[list[PieceObs]],
    cluster_dist: float = mcfg.PIECE_MERGE_DIST_M,
) -> dict[str, list[tuple[float, float]]]:
    """여러 카메라의 관측을 라벨별로 뭉친다.

    같은 라벨의 관측이 cluster_dist 안에 모여 있으면 같은 기물로 보고
    신뢰도 가중 평균으로 좌표 하나를 낸다. 기물은 로봇이 옮기기 전까지는
    스스로 움직이지 않으므로 프레임 간 추적(tracking)은 하지 않고, 매
    사이클 새로 검출한 값만으로 계산한다.
    """
    by_label: dict[str, list[PieceObs]] = {}
    for obs in (o for lst in obs_lists for o in lst):
        by_label.setdefault(obs.label, []).append(obs)

    result: dict[str, list[tuple[float, float]]] = {}
    for label, items in by_label.items():
        clusters: list[list[PieceObs]] = []
        for o in items:
            for c in clusters:
                cx = sum(m.x for m in c) / len(c)
                cy = sum(m.y for m in c) / len(c)
                if math.hypot(o.x - cx, o.y - cy) <= cluster_dist:
                    c.append(o)
                    break
            else:
                clusters.append([o])

        merged = []
        for c in clusters:
            w = sum(m.confidence for m in c)
            x = sum(m.x * m.confidence for m in c) / w
            y = sum(m.y * m.confidence for m in c) / w
            merged.append((x, y))
        result[label] = merged
    return result


@dataclass
class _Track:
    """PieceTracker 내부용 — 위치로 식별되는 기물 하나의 누적 상태."""
    x: float
    y: float
    label_scores: dict[str, float] = field(default_factory=dict)  # 라벨별 누적(감쇠) confidence
    last_seen: float = 0.0
    first_seen: float = 0.0
    n_obs: int = 0

    def confirmed(self, now: float) -> bool:
        return now - self.first_seen >= mcfg.PIECE_CONFIRM_SEC

    @property
    def label(self) -> str:
        return max(self.label_scores, key=self.label_scores.get)

    @property
    def score(self) -> float:
        return sum(self.label_scores.values())


class PieceTracker:
    """여러 프레임에 걸쳐 기물을 "위치"로 추적하며 라벨을 다수결로 확정한다.

    merge_observations() 는 매 프레임을 독립적으로 계산해서 세 가지 문제가
    생긴다: (1) 특정 위치/각도에서 한쪽 카메라만 놓쳐도 그 프레임엔 안 보임,
    (2) 어쩌다 한 프레임 놓치면 지도에서 깜빡임, (3) 한 기물이 프레임마다
    다른 라벨로 잘못 잡히면 서로 다른 기물처럼 따로 잡힘.

    여기서는 트랙을 라벨이 아니라 "위치"로 매칭한다 — 그래서 같은 물리적
    기물이 프레임마다 다른 라벨로 튀어도 트랙 하나로 유지되고(③ 해결),
    트랙의 확정 라벨은 최근 관측들의 confidence 가중 다수결로 정해져서
    한두 프레임의 오분류에 흔들리지 않는다. 카메라 한쪽만 봐도(또는 아예
    못 봐도) PIECE_HOLD_SEC 동안은 트랙을 유지한다(①②완화).

    라벨당 실제 개수는 PIECE_MAX_PER_LABEL 로 제한되어 있으므로, 같은
    라벨의 트랙이 그보다 많아지면 점수(누적 confidence) 가 낮은 트랙은
    노이즈로 보고 지도 출력에서 뺀다 — 트랙 자체는 지우지 않으므로 다시
    강해지면(진짜 그 라벨이 맞으면) 순위가 올라와 복귀할 수 있다.

    막 생긴 트랙은 PIECE_CONFIRM_SEC 가 지나기 전엔 출력에서 아예 뺀다(추적
    자체는 계속함) — 한 프레임짜리 오검출이 유령 기물로 잠깐이라도 지도에
    찍히는 것을 막는다. 기물은 안 움직이므로 실수할 이유가 없다: 진짜
    기물이면 그 사이 새 관측이 더 들어와 확정되고, 노이즈면 다음 관측 없이
    PIECE_HOLD_SEC 안에 사라진다.
    """

    def __init__(self) -> None:
        self._tracks: list[_Track] = []

    def reset(self) -> None:
        """모든 트랙을 지운다 — 화면/상태를 강제로 초기화할 때 쓴다(LiveMap 리셋 버튼)."""
        self._tracks = []

    def update(self, obs_lists: list[list[PieceObs]]) -> dict[str, list[tuple[float, float]]]:
        now = time.monotonic()

        # 과거 라벨 표를 서서히 잊는다 — 진짜로 라벨이 바뀔 일은 없지만(기물은
        # 로봇이 옮기기 전엔 안 바뀜), 초기 오분류가 영원히 남지 않게 한다.
        for t in self._tracks:
            for k in t.label_scores:
                t.label_scores[k] *= mcfg.PIECE_LABEL_DECAY

        for obs in (o for lst in obs_lists for o in lst):
            best, best_d = None, mcfg.PIECE_MERGE_DIST_M
            for t in self._tracks:
                d = math.hypot(obs.x - t.x, obs.y - t.y)
                if d <= best_d:
                    best, best_d = t, d
            if best is None:
                best = _Track(obs.x, obs.y, first_seen=now)
                self._tracks.append(best)

            # 위치는 지수 이동평균으로 부드럽게 — 관측이 쌓일수록 덜 흔들리게.
            alpha = 1.0 / (best.n_obs + 1) if best.n_obs < 5 else 0.2
            best.x += (obs.x - best.x) * alpha
            best.y += (obs.y - best.y) * alpha
            best.label_scores[obs.label] = best.label_scores.get(obs.label, 0.0) + obs.confidence
            best.last_seen = now
            best.n_obs += 1

        self._tracks = [t for t in self._tracks if now - t.last_seen <= mcfg.PIECE_HOLD_SEC]

        by_label: dict[str, list[_Track]] = {}
        for t in self._tracks:
            if not t.confirmed(now):
                continue   # 아직 확정 안 된 트랙 — 내부적으로는 계속 추적하되 출력엔 안 냄
            by_label.setdefault(t.label, []).append(t)

        result: dict[str, list[tuple[float, float]]] = {}
        for label, tracks in by_label.items():
            tracks.sort(key=lambda t: t.score, reverse=True)
            result[label] = [(t.x, t.y) for t in tracks[:mcfg.PIECE_MAX_PER_LABEL]]
        return result
