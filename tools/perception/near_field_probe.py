#!/usr/bin/env python3
"""근거리에서 무엇이 왜 안 보이는지 가르는 도구.

2026-08-25, pose_verify_cycle이 파지 위치의 물체를 못 보는 원인을 좁히려고
만들었다. "안 보임"에는 서로 다른 원인이 섞여 있는데 observe_target의
found=False 하나로는 구분이 안 된다:

    (a) 프레임 밖 — 너무 가까워 카메라 시야 아래로 빠졌다
    (b) 모델이 못 봤다 — 그 스케일·원근에서 검출 자체가 안 된다
    (c) 게이트에 막혔다 — 모델은 봤는데 CONF_THRESHOLD(0.45)를 못 넘었다

그래서 같은 프레임에 대해 **세 가지를 나란히** 낸다: 낮은 conf로 돌린 YOLO
원시 검출 전부, perception이 실제로 돌려주는 observe_target 응답, 그리고
주석이 찍힌 이미지. (c)는 원시 검출에는 있는데 observe_target에는 없는
클래스로 즉시 드러나고, (a)는 주석 이미지에서 물체가 프레임 밖인지로
드러난다.

⚠️ --label을 반드시 붙일 것. 파일 이름이 겹치면 앞 회차가 조용히 지워진다
(2026-08-25에 18cm·25cm 캡처를 그렇게 잃었다).

사용:
  python3 near_field_probe.py --label 18cm --cls box
  python3 near_field_probe.py --label 35cm --cls box --conf 0.10
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from grippers_interfaces.srv import ObserveTarget
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

# --- 2026-08-25 이 도구로 얻은 카메라 기하 ---------------------------------
#
# 큐브(폭 40mm)를 네 거리에 놓고 찍은 프레임에서, bbox 폭으로 거리를 역산하고
# bbox 바닥 행(= 물체가 바닥에 닿는 지점)을 짝지어 카메라 높이와 피치를 함께
# 최소제곱으로 풀었다. 네 점 RMS 7.2px로 맞는다:
#
#     접촉행 261 / 폭 68px -> 34.6cm      접촉행 303 / 폭 91px  -> 25.9cm
#     접촉행 371 / 폭 118px -> 20.0cm     접촉행 367 / 폭 126px -> 18.7cm
#
#     => 카메라 높이 8.8cm, 아래로 12.75도
#
# 여기서 나오는 결론이 이 파일의 존재 이유였다: **프레임 하단(480행)이 보는
# 바닥은 12.8cm**다. 즉 파지 위치(전방 18~19cm)의 물체는 시야에 넉넉히 들어와
# 있고, 거기서 안 보이는 것은 시야 문제가 아니라 인식 문제다. 실제로 그 거리의
# 큐브는 rook으로 오분류됐고 box는 conf 0.10까지 낮춰도 나오지 않았다.
#
# ⚠️ 카메라를 다시 장착하면 이 수치는 전부 무효다. 그때는 같은 방법으로 다시
# 재면 된다 — 물체 하나를 서너 거리에 놓고 이 도구로 찍는 것이 전부다.
CAMERA_HEIGHT_M = 0.088
CAMERA_PITCH_DEG = 12.75
NEAR_FIELD_CUTOFF_M = 0.128

RGB_TOPIC = "depth_cam/rgb/image_rotated"
MODEL_PATH = "/grippers/models/best_cpu.pt"
OUT_DIR = "/grippers/recordings/near_field"
# perception_node가 admission에 쓰는 값(floor_consensus.CONF_THRESHOLD).
# 여기에 베껴 적는 이유는 하나뿐이다 — 원시 검출 표에서 "이건 게이트에
# 막혔다"를 눈으로 바로 표시하기 위해서다. 판정에는 쓰지 않는다.
PERCEPTION_CONF_GATE = 0.45
FRAME_WAIT_S = 8.0


class ProbeNode(Node):
    def __init__(self):
        super().__init__("near_field_probe")
        self._frame = None
        self.create_subscription(Image, RGB_TOPIC, self._on_rgb, 10)
        self._observe = self.create_client(ObserveTarget, "perception/observe_target")

    def _on_rgb(self, msg):
        self._frame = msg

    def wait_frame(self, timeout_s=FRAME_WAIT_S):
        deadline = time.monotonic() + timeout_s
        while self._frame is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self._frame

    def observe(self, raw_cls, timeout_sec=3.0):
        if not self._observe.wait_for_service(timeout_sec=timeout_sec):
            return None
        future = self._observe.call_async(ObserveTarget.Request(raw_cls=raw_cls))
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result() if future.done() else None


def bgr_from_image_msg(msg) -> np.ndarray:
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)
    return buf.copy()


def raw_detections(model, bgr, conf):
    """(클래스, 신뢰도, (x1,y1,x2,y2)) 목록 — 게이트 이전의 날것."""
    result = model(bgr, conf=conf, verbose=False)[0]
    names = result.names
    out = []
    for box in result.boxes:
        x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
        out.append((names[int(box.cls[0])], float(box.conf[0]), (x1, y1, x2, y2)))
    return sorted(out, key=lambda d: -d[1])


def annotate(bgr, detections, gate=PERCEPTION_CONF_GATE):
    """게이트를 넘은 것은 초록, 못 넘은 것은 빨강 — 한눈에 (c)가 보이게."""
    image = bgr.copy()
    for cls, conf, (x1, y1, x2, y2) in detections:
        colour = (0, 200, 0) if conf >= gate else (0, 0, 255)
        cv2.rectangle(image, (x1, y1), (x2, y2), colour, 2)
        cv2.putText(image, f"{cls} {conf:.2f} h{y2 - y1} w{x2 - x1}",
                    (x1, max(14, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1)
    cv2.line(image, (0, image.shape[0] - 1), (image.shape[1], image.shape[0] - 1),
             (255, 255, 0), 2)
    return image


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", required=True,
                    help="회차 이름(예: 18cm). 파일 이름에 들어간다 — 겹치면 덮어쓴다")
    ap.add_argument("--cls", default="box", help="observe_target으로 물어볼 클래스")
    ap.add_argument("--conf", type=float, default=0.10,
                    help=f"YOLO 원시 검출 하한. perception 게이트는 {PERCEPTION_CONF_GATE}")
    ap.add_argument("--out-dir", default=OUT_DIR)
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    stem = os.path.join(args.out_dir, f"{args.label}")
    if os.path.exists(f"{stem}_yolo.png"):
        print(f"[경고] {stem}_yolo.png가 이미 있습니다 — 덮어씁니다. "
              "--label을 다르게 주세요", file=sys.stderr)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ProbeNode()
    try:
        msg = node.wait_frame()
        if msg is None:
            print(f"[실패] {FRAME_WAIT_S}s 안에 {RGB_TOPIC} 프레임이 없습니다 — "
                  "depth_camera / depth_cam_rotate_node 확인", file=sys.stderr)
            return 1
        bgr = bgr_from_image_msg(msg)
        cv2.imwrite(f"{stem}_raw.png", bgr)

        # perception이 실제로 무엇을 돌려주는지 — 게이트 **이후**.
        response = node.observe(args.cls)

        from ultralytics import YOLO  # import가 느리다 — 프레임을 받은 뒤에 한다
        detections = raw_detections(YOLO(MODEL_PATH), bgr, args.conf)
        cv2.imwrite(f"{stem}_yolo.png", annotate(bgr, detections))

        print(f"\n=== {args.label} — 원시 검출 (conf >= {args.conf}) ===")
        if not detections:
            print("  (없음) — 모델이 이 프레임에서 아무것도 못 봤습니다")
        for cls, conf, (x1, y1, x2, y2) in detections:
            passed = conf >= PERCEPTION_CONF_GATE
            gated = "" if passed else f"  ← 게이트 {PERCEPTION_CONF_GATE} 미만"
            print(f"  {cls:<8} conf={conf:.2f}  bbox=({x1},{y1})-({x2},{y2})  "
                  f"h={y2 - y1} w={x2 - x1}{gated}")

        print(f"\n=== observe_target('{args.cls}') — 게이트 이후 ===")
        if response is None:
            print("  응답 없음 — perception_node를 확인하세요")
        elif response.found:
            print(f"  found=True  x={response.x:.1f} h={response.h:.1f} w={response.w:.1f}")
        else:
            print("  found=False")

        mine = [d for d in detections if d[0] == args.cls]
        print("\n=== 해석 ===")
        if response is not None and response.found:
            print(f"  '{args.cls}' 정상 검출 — 이 거리에서는 문제 없습니다.")
        elif not mine:
            print(f"  모델이 conf {args.conf}까지 낮춰도 '{args.cls}'를 못 봤습니다.")
            print("  주석 이미지에서 물체가 프레임 안에 있는지 확인하세요 —")
            print("  안에 있는데 못 봤다면 이 거리·스케일이 학습 분포 밖입니다.")
        else:
            best = max(mine, key=lambda d: d[1])
            print(f"  모델은 봤습니다(conf={best[1]:.2f})만 "
                  f"게이트 {PERCEPTION_CONF_GATE}를 못 넘어 버려졌습니다.")
            print("  검출 실패가 아니라 admission 문제입니다.")

        print(f"\n저장: {stem}_raw.png  {stem}_yolo.png")
        print("주석: 초록=게이트 통과, 빨강=게이트 미만, 하늘색 선=프레임 하단")
        print(f"참고: 이 카메라의 근거리 컷오프는 {NEAR_FIELD_CUTOFF_M * 100:.0f}cm다 "
              "— 그보다 먼 물체가 안 보이면 시야가 아니라 인식 문제다")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
