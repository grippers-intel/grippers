#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""뎁스카메라 RGB 토픽에서 프레임을 받아 JPEG로 저장한다.

**이 카메라는 OpenCV로 직접 열면 안 된다.** /dev/depth_cam(video0)을 그냥 열면
OpenCV가 YUYV 1280x1040 을 잡는데, 그건 RGB와 뎁스가 한 프레임에 쌓인 원시
결합 스트림이라 초록/보라 띠만 찍힌다. MJPG 모드들도 1280x720은 프레임이 안
나오고 640x642 같은 것들은 검은 화면이다.

제대로 된 RGB는 Angstrong 드라이버(ascamera)를 통해서만 나온다:

    source /opt/ros/humble/setup.bash
    source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash   # ← ascamera 가 여기 있다
    source /ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=21 need_compile=False DEPTH_CAMERA_TYPE=ascamera
    ros2 launch peripherals depth_camera.launch.py

드라이버는 약 15Hz로 발행한다.
"""
from __future__ import annotations

import argparse
import json
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC = "/ascamera/camera_publisher/rgb0/image"


def to_bgr(msg: Image) -> np.ndarray:
    """sensor_msgs/Image → OpenCV BGR. cv_bridge 없이 직접 변환한다."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1] if enc == "rgb8" else img
    if enc == "mono8":
        return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    raise ValueError(f"지원하지 않는 인코딩: {msg.encoding}")


class Grabber(Node):
    def __init__(self, args):
        super().__init__("frame_grabber")
        self.args = args
        stamp = time.strftime("%Y%m%d_%H%M%S")
        tail = f"_{args.label}" if args.label else ""
        self.out = os.path.join(args.out, f"frames_{stamp}{tail}")
        os.makedirs(self.out, exist_ok=True)
        self.n = 0
        self.t_start = None
        self.next_t = 0.0
        self.enc = [int(cv2.IMWRITE_JPEG_QUALITY), args.quality]
        self.shape = None
        self.create_subscription(Image, args.topic, self.on_image, 10)
        print(f"[캡처] {args.topic} → {self.out}", flush=True)
        print(f"[캡처] 워밍업 {args.warmup}s 후 {args.duration:.0f}초 촬영", flush=True)
        self.t_open = time.monotonic()

    def on_image(self, msg: Image):
        now = time.monotonic()
        if now - self.t_open < self.args.warmup:   # 자동 노출 안정화 구간은 버린다
            return
        if self.t_start is None:
            self.t_start, self.next_t = now, now
            print("[캡처] 촬영 시작", flush=True)
        if now - self.t_start >= self.args.duration:
            raise KeyboardInterrupt
        if now < self.next_t:                      # 15Hz 원본을 목표 주기로 솎아낸다
            return
        try:
            img = to_bgr(msg)
        except ValueError as exc:
            print(f"[캡처] {exc}", flush=True); raise KeyboardInterrupt
        if self.args.rotate:      # 카메라가 거꾸로 달려 있다. 학습 데이터는 정립이므로 여기서 맞춘다
            img = cv2.rotate(img, {90: cv2.ROTATE_90_CLOCKWISE,
                                   180: cv2.ROTATE_180,
                                   270: cv2.ROTATE_90_COUNTERCLOCKWISE}[self.args.rotate])
        self.shape = img.shape
        cv2.imwrite(os.path.join(self.out, f"{self.n:05d}.jpg"), img, self.enc)
        self.n += 1
        self.next_t += 1.0 / self.args.fps
        if self.n % 25 == 0:
            print(f"[캡처] {self.n}장 · {now - self.t_start:.0f}/{self.args.duration:.0f}초", flush=True)

    def finish(self):
        el = (time.monotonic() - self.t_start) if self.t_start else 0.0
        meta = {"topic": self.args.topic, "label": self.args.label,
                "rotate": self.args.rotate,
                "shape": list(self.shape) if self.shape else None,
                "target_fps": self.args.fps, "frames": self.n,
                "elapsed_sec": round(el, 1),
                "actual_fps": round(self.n / el, 2) if el else 0}
        with open(os.path.join(self.out, "manifest.json"), "w") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        mb = sum(os.path.getsize(os.path.join(self.out, x))
                 for x in os.listdir(self.out)) / 1e6
        print(f"[캡처] 완료 — {self.n}장, {el:.0f}초, {mb:.0f}MB", flush=True)
        print(f"[캡처] 저장 위치: {self.out}", flush=True)


def main():
    ap = argparse.ArgumentParser(description="뎁스카메라 RGB 프레임 캡처")
    ap.add_argument("--topic", default=TOPIC)
    ap.add_argument("--out", default="/grippers/recordings")
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--quality", type=int, default=92)
    ap.add_argument("--warmup", type=float, default=2.0)
    ap.add_argument("--label", default="")
    ap.add_argument("--rotate", type=int, default=180, choices=[0, 90, 180, 270],
                    help="저장 전 회전. 이 뎁스캠은 거꾸로 달려 있어 180이 기본")
    args = ap.parse_args()

    rclpy.init()
    node = Grabber(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
