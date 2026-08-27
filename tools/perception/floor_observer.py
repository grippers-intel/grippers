#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""바닥 관측기 — 로봇이 멈춘 상태에서 여러 프레임을 모아 물체 목록을 확정한다.

미션 FSM 이 집기 직전에 호출하는 부품이다. 한 장의 검출은 믿지 않는다.

실측으로 정한 동작점 (60프레임 a_near 촬영본 기준):
  - conf 0.45 / k-of-n 0.6 에서 프레임당 7.27개 검출이 물체 6개로 수렴
  - 확정된 물체의 위치 산포가 **1픽셀 이내** — 파지에 필요한 정밀도가 여기서 나온다

**순도 게이트(purity)가 핵심 안전장치다.** 합의 필터는 "여러 프레임에서 일관된 것"을
믿으므로 *일관되게 틀린* 오분류는 못 걸러낸다. 대신 클래스 다수결이 갈리면 순도가
떨어지므로, 순도가 낮은 관측은 **아예 집으러 가지 않는다.** 헛집는 것보다 건너뛰는
쪽이 낫다.

**클래스 허용목록** — 2026-08-23 train-8 실측에서 knight/queen/rook/soccer 는
안정적이었으나 box 는 60장 중 0회 검출, star 는 신뢰도 0.31 로 불안정했다.
2026-08-27 train-9로 재검증하니(--frames 60) box 60/60·순도 1.00·신뢰 0.93,
star 60/60·순도 1.00·신뢰 0.95 로 나머지 넷보다도 깨끗했다 — train-8의
검출력 한계였을 뿐이었다. 이제 여섯 클래스 전부 믿는다.
"""
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

sys.path.insert(0, "/grippers/tools/perception")
from consensus import consensus  # noqa: E402

TOPIC = "/ascamera/camera_publisher/rgb0/image"
# 2026-08-27: 예전 기본값(best_ncnn_model)이 Pi에 없어 이 도구 자체가 못
# 떴었다. perception_node.CPU_YOLO_MODEL_PATH_DEFAULT와 같은 경로로 맞춘다 —
# 이름도 사용자 지시로 best_cpu.pt에서 best.pt로 통일했다.
MODEL = "/grippers/models/best.pt"
RELIABLE = ("knight", "queen", "rook", "soccer", "box", "star")  # 2026-08-27 train-9로 box·star 추가


@dataclass
class Observation:
    """확정된 물체 하나. 좌표는 화면상 바닥 접점(픽셀)."""
    cls: str
    x: float
    y: float
    support: int      # 몇 프레임에서 보였나
    purity: float     # 클래스 다수결이 얼마나 압도적이었나
    conf: float
    spread: float     # 위치 일관성(픽셀). 클수록 못 믿는다
    w: float = 0.0    # 검출 박스 폭  — 거리 신호
    h: float = 0.0    # 검출 박스 높이 — 거리 신호


def to_bgr(msg: Image) -> np.ndarray:
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1] if enc == "rgb8" else img
    if enc == "mono8":
        return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    raise ValueError(f"지원하지 않는 인코딩: {msg.encoding}")


class FloorObserver(Node):
    def __init__(self, *, model_path=MODEL, topic=TOPIC, n_frames=15,
                 conf=0.45, min_ratio=0.6, min_purity=0.8, min_support_conf=0.35,
                 allowed=None, rotate=180, warmup=1.0, min_y=290.0,
                 max_spread=40.0):
        super().__init__("floor_observer")
        from ultralytics import YOLO
        self.model = YOLO(model_path, task="detect")
        self.p = dict(n_frames=n_frames, conf=conf, min_ratio=min_ratio,
                      min_purity=min_purity, min_support_conf=min_support_conf,
                      allowed=set(allowed) if allowed else None,
                      rotate=rotate, warmup=warmup, min_y=min_y,
                      max_spread=max_spread)
        self._frames: list = []
        self._t_open = time.monotonic()
        # 시각화용. analyse() 가 돌 때마다 프레임별 원시 검출을 남긴다 —
        # 합의 전/후를 겹쳐 그리면 필터가 무슨 일을 했는지 한 장으로 보인다.
        self.last_per_frame: list = []
        self.create_subscription(Image, topic, self._on_image, 10)

    def _on_image(self, msg: Image):
        if time.monotonic() - self._t_open < self.p["warmup"]:
            return                                   # 자동 노출 안정화 구간
        if len(self._frames) >= self.p["n_frames"]:
            return
        img = to_bgr(msg)
        if self.p["rotate"]:                         # 카메라가 거꾸로 달려 있다
            img = cv2.rotate(img, {90: cv2.ROTATE_90_CLOCKWISE,
                                   180: cv2.ROTATE_180,
                                   270: cv2.ROTATE_90_COUNTERCLOCKWISE}[self.p["rotate"]])
        self._frames.append(img)

    @property
    def ready(self) -> bool:
        return len(self._frames) >= self.p["n_frames"]

    def analyse(self) -> list[Observation]:
        """모은 프레임에 검출을 돌리고 합의 → 게이트를 거쳐 물체 목록을 낸다."""
        per_frame = []
        for img in self._frames:
            r = self.model.predict(img, imgsz=640, conf=self.p["conf"], verbose=False)[0]
            per_frame.append([(self.model.names[int(c)], float(cf),
                               [float(v) for v in b])
                              for c, cf, b in zip(r.boxes.cls, r.boxes.conf, r.boxes.xyxy)])

        self.last_per_frame = per_frame

        out = []
        for t in consensus(per_frame, len(self._frames), min_ratio=self.p["min_ratio"]):
            mean_conf = sum(t.confs) / len(t.confs)
            # ── 게이트 ────────────────────────────────────────────────────
            if t.purity < self.p["min_purity"]:      # 클래스가 갈린다 → 못 믿는다
                continue
            if mean_conf < self.p["min_support_conf"]:
                continue
            if self.p["allowed"] and t.label not in self.p["allowed"]:
                continue                              # 검증 안 된 클래스는 무시
            cx, cy = t.center
            # ── 거리 게이트 ───────────────────────────────────────────────
            # **빈 바닥 대조군으로 실측해서 정한 값이다.** 물체가 하나도 없는 아레나를
            # 60프레임 찍었더니 오탐 4개가 전부 y=259~277 에 나왔다(벽 밑동·바구니
            # 주변). 반면 세 촬영본의 진짜 물체는 최소 y=293 이었다. 그 사이를 자른다.
            #
            # 카메라 높이나 아레나가 바뀌면 이 값은 다시 재야 한다. 방법은 같다 —
            # 빈 바닥을 찍고, 오탐의 최대 y 에 여유를 더한다.
            #
            # 이 아래(먼 곳)는 어차피 손이 안 닿는다. 탐색용으로 넓게 보려면
            # min_y 를 낮춰 부르면 된다.
            if cy < self.p["min_y"]:
                continue
            if t.spread > self.p["max_spread"]:       # 위치가 안 잡히면 접근하지 않는다
                continue
            bw, bh = t.size
            out.append(Observation(t.label, cx, cy, len(t.frames),
                                   t.purity, mean_conf, t.spread, bw, bh))
        out.sort(key=lambda o: -o.y)                  # 가까운 것(화면 아래)부터
        return out


def main():
    ap = argparse.ArgumentParser(description="바닥 관측기 — 단독 실행 시험")
    ap.add_argument("--frames", type=int, default=15)
    ap.add_argument("--conf", type=float, default=0.45)
    ap.add_argument("--ratio", type=float, default=0.6)
    ap.add_argument("--purity", type=float, default=0.8)
    # 2026-08-27: 290은 floor_consensus.MIN_BOTTOM_Y_PX가 2026-08-23에 이미
    # 200으로 낮춘 그 낡은 값이었다 — SCAN 거리대(0.66~1.13m)에서 bbox 하단
    # y가 227~271이라 290 기준으로는 다 "너무 멀다"로 걸러진다. 이 도구가
    # floor_consensus.py와 다른 값을 쓰면 진단 결과를 못 믿는다.
    ap.add_argument("--min-y", type=float, default=200.0,
                    help="이 높이보다 위(먼 곳)의 검출은 무시. floor_consensus."
                         "MIN_BOTTOM_Y_PX와 같은 기본값(200)")
    ap.add_argument("--all-classes", action="store_true",
                    help="허용목록을 풀고 6클래스 전부 본다(진단용, 항상 켜져 있음)")
    ap.add_argument("--model", default=MODEL,
                    help=f"YOLO 가중치 경로 (기본 {MODEL})")
    args = ap.parse_args()

    rclpy.init()
    node = FloorObserver(model_path=args.model,
                         n_frames=args.frames, conf=args.conf,
                         min_ratio=args.ratio, min_purity=args.purity,
                         min_y=args.min_y, allowed=None)
    print(f"[관측] {args.frames}프레임 수집 중…", flush=True)
    t0 = time.monotonic()
    while not node.ready and time.monotonic() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.ready:
        print("[관측] 프레임 부족 — 카메라 드라이버가 도는지 확인하세요", flush=True)
        rclpy.shutdown(); sys.exit(1)

    t1 = time.monotonic()
    obs = node.analyse()
    print(f"[관측] 수집 {t1-t0:.1f}s + 분석 {time.monotonic()-t1:.1f}s\n", flush=True)
    if not obs:
        print("  확정된 물체 없음")
    else:
        print(f"  {'클래스':<9} {'화면위치':>14} {'박스 폭×높이':>13} "
              f"{'지지':>7} {'순도':>6} {'신뢰':>6} {'산포':>6}")
        for o in obs:
            print(f"  {o.cls:<9} {o.x:>7.0f},{o.y:>6.0f} "
                  f"{o.w:>6.0f}×{o.h:<6.0f} "
                  f"{o.support:>3}/{args.frames:<3} {o.purity:>6.2f} "
                  f"{o.conf:>6.2f} {o.spread:>6.1f}")
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
