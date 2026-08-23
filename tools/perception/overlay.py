#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""합의 필터를 한 장으로 설명하는 그림을 만든다 — 발표·제출 자료용.

왼쪽은 N 프레임의 **원시 검출을 전부 겹친 것**이고, 오른쪽은 **합의 후 확정된 물체**다.
왼쪽의 흩어진 박스가 오른쪽에서 몇 개로 수렴하는 게 이 필터가 하는 일 전부다.
말로 설명하면 길지만 그림 한 장이면 끝난다.

그림 안 글자는 영문이다 — OpenCV 기본 폰트가 한글을 못 그리고, 제출처가
글로벌 해커톤이라 어차피 영문이 맞다.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy

sys.path.insert(0, "/grippers/tools/perception")
from floor_observer import FloorObserver   # noqa: E402

OUT_DIR = "/grippers/recordings"
# BGR. 클래스마다 다른 색이어야 겹쳐 그렸을 때 구분된다.
COLORS = {
    "rook":   (80, 200, 255),
    "knight": (80, 255, 140),
    "queen":  (255, 170, 80),
    "soccer": (255, 120, 220),
    "star":   (120, 120, 255),
    "box":    (200, 200, 100),
}
GREY = (150, 150, 150)


def color_of(cls):
    return COLORS.get(cls, GREY)


def banner(img, text, sub=""):
    """패널 위에 제목 띠를 얹는다."""
    h = 64 if sub else 44
    strip = np.full((h, img.shape[1], 3), 32, np.uint8)
    cv2.putText(strip, text, (14, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.75, (255, 255, 255), 2, cv2.LINE_AA)
    if sub:
        cv2.putText(strip, sub, (14, 54), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (170, 170, 170), 1, cv2.LINE_AA)
    return np.vstack([strip, img])


def draw_raw(base, per_frame):
    """모든 프레임의 검출을 반투명하게 겹친다. 흔들림이 그대로 보인다."""
    layer = base.copy()
    n = 0
    for dets in per_frame:
        for cls, conf, (x1, y1, x2, y2) in dets:
            cv2.rectangle(layer, (int(x1), int(y1)), (int(x2), int(y2)),
                          color_of(cls), 1, cv2.LINE_AA)
            n += 1
    return cv2.addWeighted(layer, 0.55, base, 0.45, 0), n


def draw_confirmed(base, obs, frames):
    out = base.copy()
    for o in obs:
        c = color_of(o.cls)
        x1, x2 = int(o.x - o.w / 2), int(o.x + o.w / 2)
        y1, y2 = int(o.y - o.h), int(o.y)
        cv2.rectangle(out, (x1, y1), (x2, y2), c, 2, cv2.LINE_AA)
        # 바닥 접점 — 접근 루프가 실제로 쓰는 좌표다.
        cv2.drawMarker(out, (int(o.x), int(o.y)), c, cv2.MARKER_CROSS, 14, 2)
        tag = f"{o.cls} {o.conf:.2f}  {o.support}/{frames}"
        (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
        ty = max(th + 4, y1 - 4)
        cv2.rectangle(out, (x1, ty - th - 4), (x1 + tw + 6, ty + 2), c, -1)
        cv2.putText(out, tag, (x1 + 3, ty - 1), cv2.FONT_HERSHEY_SIMPLEX,
                    0.48, (20, 20, 20), 1, cv2.LINE_AA)
    return out


def main():
    ap = argparse.ArgumentParser(description="합의 필터 시각화")
    ap.add_argument("--label", default="consensus")
    ap.add_argument("--frames", type=int, default=12)
    ap.add_argument("--conf", type=float, default=0.45)
    ap.add_argument("--min-y", type=float, default=0.0,
                    help="거리 게이트. 그림에서는 기본으로 끈다(전부 보이게)")
    args = ap.parse_args()

    rclpy.init()
    node = FloorObserver(n_frames=args.frames, conf=args.conf,
                         min_ratio=0.6, min_purity=0.8,
                         min_y=args.min_y, allowed=None, warmup=1.0)
    print(f"[오버레이] {args.frames}프레임 수집 중…", flush=True)
    t0 = time.monotonic()
    while not node.ready and time.monotonic() - t0 < 25:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.ready:
        print("[오버레이] 프레임 부족 — 카메라 드라이버를 확인하세요", flush=True)
        rclpy.shutdown(); sys.exit(1)

    obs = node.analyse()
    base = node._frames[len(node._frames) // 2]

    left, n_raw = draw_raw(base, node.last_per_frame)
    right = draw_confirmed(base, obs, args.frames)

    left = banner(left, "1. Raw detections",
                  f"{args.frames} frames overlaid  -  {n_raw} boxes")
    right = banner(right, "2. After multi-frame consensus",
                   f"{len(obs)} objects confirmed"
                   + (f"  -  spread {np.mean([o.spread for o in obs]):.1f}px"
                      if obs else ""))
    gap = np.full((left.shape[0], 8, 3), 32, np.uint8)
    sheet = np.hstack([left, gap, right])

    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f"overlay_{args.label}.png")
    cv2.imwrite(path, sheet)

    print(f"[오버레이] 원시 {n_raw}개 → 확정 {len(obs)}개", flush=True)
    for o in obs:
        print(f"    {o.cls:<8} x={o.x:>5.0f} y={o.y:>5.0f} "
              f"box {o.w:>4.0f}x{o.h:<4.0f} support {o.support}/{args.frames} "
              f"spread {o.spread:.1f}px", flush=True)
    print(f"[오버레이] 저장 — {path}", flush=True)
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
