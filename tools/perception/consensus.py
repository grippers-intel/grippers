# -*- coding: utf-8 -*-
"""다중 프레임 합의 필터 — 한 장의 검출을 믿지 않고 여러 장을 모아 확정한다.

한 프레임의 검출은 흔들린다. 박스가 몇~수십 픽셀씩 움직이고, 없던 물체가
나타났다 사라진다. 파지에 필요한 정밀도는 그 상태로 안 나온다.

**전제: 베이스가 정지해 있다.** 움직이면 장면이 바뀌어 프레임 간 대응이 깨진다.
그래서 미션 FSM은 집기 직전에 로봇을 세우고 이 필터를 돌린다.

원리:
  1. 각 검출을 **박스 아래변 중점**으로 대표시킨다 — 물체가 바닥에 닿는 지점이고,
     중심을 쓰면 키 큰 기물일수록 뒤로 밀린다
  2. 프레임을 가로질러 가까운 검출끼리 묶는다(클러스터)
  3. n장 중 k장 이상에서 보인 클러스터만 인정한다 — 오탐은 산발적이고
     진짜 물체는 계속 보이므로 여기서 갈린다
  4. 위치는 **중앙값**을 쓴다. 평균은 한 프레임의 이상치에 끌려간다
  5. 클래스는 다수결, 산포는 신뢰도 지표로 함께 낸다
"""
from __future__ import annotations

import collections
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Track:
    """여러 프레임에 걸쳐 같은 물체로 묶인 검출들."""
    pts: list = field(default_factory=list)     # 바닥 접점 (x, y)
    sizes: list = field(default_factory=list)   # (박스 폭, 박스 높이)
    classes: list = field(default_factory=list)
    confs: list = field(default_factory=list)
    frames: set = field(default_factory=set)    # 등장한 프레임 번호(중복 제외)

    @property
    def center(self) -> tuple:
        a = np.asarray(self.pts)
        return float(np.median(a[:, 0])), float(np.median(a[:, 1]))

    @property
    def size(self) -> tuple:
        """박스 폭·높이의 중앙값. **거리 신호로 쓴다** — 가까워질수록 커지고,
        좌우로 움직여도 변하지 않는다. y 가 화면 끝에서 포화돼도 이건 살아 있다."""
        a = np.asarray(self.sizes)
        return float(np.median(a[:, 0])), float(np.median(a[:, 1]))

    @property
    def spread(self) -> float:
        """중앙값에서의 거리의 중앙값. 작을수록 위치가 일관된 것."""
        a = np.asarray(self.pts)
        c = np.median(a, axis=0)
        return float(np.median(np.linalg.norm(a - c, axis=1)))

    @property
    def label(self) -> str:
        return collections.Counter(self.classes).most_common(1)[0][0]

    @property
    def purity(self) -> float:
        """클래스 다수결이 얼마나 압도적이었나. 낮으면 헷갈리는 물체."""
        c = collections.Counter(self.classes)
        return c.most_common(1)[0][1] / len(self.classes)


def bottom_center(box) -> tuple:
    x1, y1, x2, y2 = box
    return ((x1 + x2) / 2.0, y2)


def consensus(detections: list, n_frames: int,
              radius: float = 45.0, size_frac: float = 0.35,
              min_ratio: float = 0.5) -> list:
    """detections: 프레임별 [(cls, conf, (x1,y1,x2,y2)), ...] 의 리스트.

    radius    같은 물체로 볼 바닥 접점 거리의 하한(픽셀)
    size_frac 반경을 박스 폭에 비례시키는 계수. 화면을 가득 채우는 가까운 물체는
              검출 박스 자체가 크게 흔들려서 고정 반경으로는 한 물체가 두 트랙으로
              쪼개진다. 실제로 가까운 룩이 69픽셀 떨어진 두 트랙이 됐다.
              먼 작은 물체는 폭이 작아 반경도 작게 유지되므로 서로 안 섞인다.
    min_ratio 전체 프레임 중 이 비율 이상 보여야 인정 (k-of-n 의 k/n)
    """
    tracks: list[Track] = []
    for fi, dets in enumerate(detections):
        for cls, conf, box in dets:
            p = bottom_center(box)
            r = max(radius, (box[2] - box[0]) * size_frac)
            best, bd = None, r
            for t in tracks:                      # 가장 가까운 기존 트랙에 붙인다
                # **같은 클래스끼리만 묶는다.** 박스 크기에 비례하는 반경 탓에
                # 바짝 붙은 서로 다른 물체가 한 트랙으로 삼켜졌다(실측: box와 star가
                # 합쳐져 순도 0.50, 산포 16.5). 클래스로 갈라놓으면 애초에 안 섞인다.
                # 같은 클래스가 붙어 있는 경우는 남지만, 그건 순도/산포가 표시해 준다.
                if t.classes[0] != cls:
                    continue
                c = t.center
                d = ((p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2) ** 0.5
                if d < bd:
                    best, bd = t, d
            if best is None:
                best = Track(); tracks.append(best)
            best.pts.append(p); best.classes.append(cls)
            best.sizes.append((box[2] - box[0], box[3] - box[1]))
            best.confs.append(conf); best.frames.add(fi)

    need = max(1, int(round(n_frames * min_ratio)))
    return [t for t in tracks if len(t.frames) >= need]
