"""파지 확인 — 그리퍼캠 근접 영역이 얼마나 달라졌는가. 순수 numpy.

TPU 턱에서는 개구율도 부하도 빈손과 겹친다(2026-09-22 실측: 빈손 6.6~12.2% / 부하 0.13~0.18,
룩을 쥔 상태 10.9% / 0.17). 반면 근접 영역은 빈손끼리 0.0% 대 파지 27~42% 로 갈린다.

⚠️ **같은 자세끼리만 의미가 있다.** 팔이 시작 자세로 돌아온 뒤에 비교해야 한다 —
자세가 다르면 바닥이 통째로 달라 보여 물체가 없어도 50% 가 나온다(2026-09-22 실기).
"""
from __future__ import annotations

import numpy as np


def roi_changed_percent(before: np.ndarray, after: np.ndarray,
                        roi_fractions, pixel_threshold: float) -> float:
    """두 프레임의 근접 ROI 에서 달라진 픽셀 비율(%).

    물체를 쥐면 턱 바로 앞이 물체로 덮이므로 이 값이 크게 뛴다.
    2026-09-22 실측: 같은 자세의 빈손끼리 0.0% · 룩을 쥔 상태 30~33%.
    개구율·부하가 TPU 때문에 빈손과 겹치는 것과 달리 여기서는 10배 갈린다.

    ⚠️ **같은 자세끼리만 의미가 있다.** 호출부가 자세를 확인하고 부른다.
    """
    if before is None or after is None or before.shape != after.shape:
        return -1.0
    h, w = before.shape[:2]
    y0, y1, x0, x1 = roi_fractions
    a = before[int(h * y0):int(h * y1), int(w * x0):int(w * x1)].astype(np.int16)
    b = after[int(h * y0):int(h * y1), int(w * x0):int(w * x1)].astype(np.int16)
    if a.size == 0:
        return -1.0
    return float(np.mean(np.max(np.abs(b - a), axis=2) > pixel_threshold) * 100.0)
