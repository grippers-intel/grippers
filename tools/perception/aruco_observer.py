#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ArUco 마커 관측기 — 바구니처럼 **학습시킬 수 없는 목표**를 위한 인식기.

`floor_observer.FloorObserver` 와 **같은 것을 내놓는다**(`Observation` 목록). 그래서
`approach.py` 의 접근 루프가 한 줄도 안 바뀐 채 그대로 돈다. 목표가 룩이든
바구니든 루프 입장에서는 "화면상의 박스" 하나일 뿐이다.

    YOLO   → Observation ┐
                         ├→ approach.py → 목표 앞 정렬
    ArUco  → Observation ┘

**왜 YOLO 가 아니라 ArUco 인가.** 바구니를 YOLO 로 잡으려면 렌더·라벨링·재학습이
필요하다. 마커는 고전 CV 라 학습이 전혀 없고, 오탐이 사실상 없으며, ID 로 바구니를
구분할 수 있다(빨강 바구니 = 4번, 파랑 = 5번 식). 대신 **마커가 보여야만** 한다.

**거리·좌우 신호는 YOLO 경로와 완전히 같다** — 거리는 박스 높이, 좌우는 아래변
중점 x. 교시값도 같은 파일 형식을 쓴다.

**요는 마커가 안 보이면 아무것도 못 한다는 것이다.** 파지 직후 로봇은 물체 쪽을
보고 있으므로, 바구니를 찾으려면 먼저 제자리에서 돌아야 한다 —
`approach.py --search` 가 그 일을 한다.
"""
from __future__ import annotations

import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

sys.path.insert(0, "/grippers/tools/perception")
from consensus import consensus          # noqa: E402
from floor_observer import Observation, to_bgr, TOPIC   # noqa: E402

DEFAULT_DICT = "DICT_4X4_50"


def label_of(marker_id: int) -> str:
    """접근 루프의 --cls 로 그대로 쓰는 이름."""
    return f"aruco{int(marker_id)}"


class ArucoObserver(Node):
    """FloorObserver 와 같은 계약: .ready / .analyse() / ._frames / ._t_open."""

    def __init__(self, *, topic=TOPIC, n_frames=10, dictionary=DEFAULT_DICT,
                 rotate=180, warmup=0.6, min_ratio=0.5, max_spread=40.0,
                 ids=None, **_ignored):
        # _ignored: FloorObserver 와 호출부를 공유하려고 남긴다(conf, min_y 등).
        super().__init__("aruco_observer")
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        self.det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
        self.p = dict(n_frames=n_frames, rotate=rotate, warmup=warmup,
                      min_ratio=min_ratio, max_spread=max_spread,
                      ids=set(int(i) for i in ids) if ids else None)
        self._frames: list = []
        self._t_open = time.monotonic()
        self.create_subscription(Image, topic, self._on_image, 10)

    def _on_image(self, msg: Image):
        if time.monotonic() - self._t_open < self.p["warmup"]:
            return
        if len(self._frames) >= self.p["n_frames"]:
            return
        img = to_bgr(msg)
        if self.p["rotate"]:                     # 카메라가 거꾸로 달려 있다
            img = cv2.rotate(img, {90: cv2.ROTATE_90_CLOCKWISE,
                                   180: cv2.ROTATE_180,
                                   270: cv2.ROTATE_90_COUNTERCLOCKWISE}[self.p["rotate"]])
        self._frames.append(img)

    @property
    def ready(self) -> bool:
        return len(self._frames) >= self.p["n_frames"]

    def _detect(self, img):
        """한 장에서 (라벨, 신뢰, xyxy) 목록. 합의 필터가 먹는 형식 그대로."""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.det.detectMarkers(gray)
        out = []
        if ids is None:
            return out
        for c, i in zip(corners, ids.flatten()):
            if self.p["ids"] and int(i) not in self.p["ids"]:
                continue
            pts = c.reshape(-1, 2)
            x1, y1 = pts.min(axis=0)
            x2, y2 = pts.max(axis=0)
            # 마커는 있거나 없거나다. 신뢰도 개념이 없으므로 1.0 으로 채운다.
            out.append((label_of(i), 1.0, [float(x1), float(y1), float(x2), float(y2)]))
        return out

    def analyse(self) -> list[Observation]:
        per_frame = [self._detect(img) for img in self._frames]
        out = []
        for t in consensus(per_frame, len(self._frames), min_ratio=self.p["min_ratio"]):
            if t.spread > self.p["max_spread"]:   # 위치가 안 잡히면 접근하지 않는다
                continue
            cx, cy = t.center
            bw, bh = t.size
            # 순도는 항상 1.0 이다 — 마커 ID 는 오분류가 없다.
            out.append(Observation(t.label, cx, cy, len(t.frames),
                                   1.0, 1.0, t.spread, bw, bh))
        out.sort(key=lambda o: -o.h)              # 큰 것(가까운 것)부터
        return out


def main():
    ap = argparse.ArgumentParser(description="ArUco 관측기 — 단독 실행 시험")
    ap.add_argument("--frames", type=int, default=10)
    ap.add_argument("--dict", default=DEFAULT_DICT)
    ap.add_argument("--ids", type=int, nargs="*", default=None,
                    help="이 ID 만 본다. 생략하면 전부")
    ap.add_argument("--ratio", type=float, default=0.5)
    args = ap.parse_args()

    rclpy.init()
    node = ArucoObserver(n_frames=args.frames, dictionary=args.dict,
                         ids=args.ids, min_ratio=args.ratio)
    print(f"[ArUco] {args.frames}프레임 수집 중…", flush=True)
    t0 = time.monotonic()
    while not node.ready and time.monotonic() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.ready:
        print("[ArUco] 프레임 부족 — 카메라 드라이버가 도는지 확인하세요", flush=True)
        rclpy.shutdown(); sys.exit(1)

    obs = node.analyse()
    print(f"[ArUco] 수집 {time.monotonic()-t0:.1f}s\n", flush=True)
    if not obs:
        print("  마커 없음 — 시야·조명·마커 크기를 확인하세요")
    else:
        print(f"  {'마커':<10} {'화면위치':>14} {'박스 폭×높이':>13} {'지지':>7} {'산포':>6}")
        for o in obs:
            print(f"  {o.cls:<10} {o.x:>7.0f},{o.y:>6.0f} "
                  f"{o.w:>6.0f}×{o.h:<6.0f} {o.support:>3}/{args.frames:<3} {o.spread:>6.1f}")
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
