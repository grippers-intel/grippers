#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""뎁스카메라 RGB 화면을 Enter 키로 한 장씩 캡처하는 대화형 콘솔.

capture_ros.py(고정 시간 자동 연사)와 달리 이건 사람이 화면을 보며 원하는
순간에 이름을 붙여 한 장씩 뽑는 용도다. ROS 구독은 백그라운드 스레드에서
계속 돌며 최신 프레임만 들고 있고, 메인 스레드는 터미널 입력만 기다린다.

기본 구독 토픽은 depth_cam_rotate_node가 180도 보정해 내보내는
`depth_cam/rgb/image_rotated`다 — 원본(`/ascamera/camera_publisher/rgb0/image`)
그대로 쓰면 카메라가 거꾸로 달려 있어 상하좌우가 뒤집힌다(§ depth_cam_rotate_node
docstring 참고). rotate 노드가 안 떠 있으면 --topic으로 원본을 지정하고
--rotate 180을 같이 줄 것.

사전 준비(표준 기동 순서, HANDOFF.md §4-1):
    source /opt/ros/humble/setup.zsh
    source /ros2_ws/install/setup.zsh
    export ROS_DOMAIN_ID=21 DEPTH_CAMERA_TYPE=ascamera need_compile=False
    ros2 launch peripherals depth_camera.launch.py &
    sleep 8
    ros2 run grippers_perception depth_cam_rotate_node &

사용:
    python3 capture_on_enter.py
    (파일명 입력 후 Enter — 비워두면 자동 이름) capture> rook_side1
    [저장] /grippers/recordings/enter_captures/rook_side1_20260824_111530.jpg
    (파일명 입력 후 Enter — 비워두면 자동 이름) capture> q      # 종료
"""
from __future__ import annotations

import argparse
import os
import re
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC_DEFAULT = "depth_cam/rgb/image_rotated"
OUT_DIR_DEFAULT = "/grippers/recordings/enter_captures"

_UNSAFE_CHARS = re.compile(r"[^A-Za-z0-9._가-힣-]+")


def to_bgr(msg: Image) -> np.ndarray:
    """sensor_msgs/Image → OpenCV BGR. cv_bridge 없이 직접 변환한다 (numpy 2.x
    ABI 비호환으로 cv_bridge를 못 쓰는 이유는 depth_cam_rotate_node.py 참고)."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1] if enc == "rgb8" else img
    if enc == "mono8":
        return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    raise ValueError(f"지원하지 않는 인코딩: {msg.encoding}")


def safe_label(raw: str) -> str:
    """파일명에 쓸 수 없는 문자를 제거/치환한다. 빈 문자열이면 그대로 빈 채 반환."""
    raw = raw.strip()
    if not raw:
        return ""
    return _UNSAFE_CHARS.sub("_", raw).strip("_") or "capture"


class LatestFrameGrabber(Node):
    """구독만 하고 아무것도 저장하지 않는다 — 최신 프레임 한 장만 락으로 보호해
    들고 있다가, 메인 스레드가 필요할 때 꺼내 간다."""

    def __init__(self, topic: str, rotate: int):
        super().__init__("capture_on_enter")
        self.rotate = rotate
        self._lock = threading.Lock()
        self._latest: np.ndarray | None = None
        self._count = 0
        self.create_subscription(Image, topic, self._on_image, 10)
        self.get_logger().info(f"구독 시작: {topic}")

    def _on_image(self, msg: Image):
        try:
            img = to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return
        if self.rotate:
            img = cv2.rotate(img, {90: cv2.ROTATE_90_CLOCKWISE,
                                   180: cv2.ROTATE_180,
                                   270: cv2.ROTATE_90_COUNTERCLOCKWISE}[self.rotate])
        with self._lock:
            self._latest = img
            self._count += 1

    def snapshot(self) -> np.ndarray | None:
        with self._lock:
            return None if self._latest is None else self._latest.copy()

    @property
    def frame_count(self) -> int:
        with self._lock:
            return self._count


def run(topic: str, out_dir: str, rotate: int, quality: int):
    os.makedirs(out_dir, exist_ok=True)

    rclpy.init()
    node = LatestFrameGrabber(topic, rotate)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f"[캡처] 대상 토픽: {topic}")
    print(f"[캡처] 저장 위치: {out_dir}")
    print("[캡처] 프레임 수신 대기 중...")
    t0 = time.monotonic()
    while node.frame_count == 0 and time.monotonic() - t0 < 10.0:
        time.sleep(0.2)
    if node.frame_count == 0:
        print("[캡처] 10초 동안 프레임이 안 들어옵니다 — 카메라/rotate 노드가 떠 있는지, "
              "토픽 이름이 맞는지 확인하세요. 그래도 입력은 계속 받습니다.")
    else:
        print("[캡처] 프레임 수신 확인됨 — 준비 완료")

    saved = []
    try:
        while True:
            try:
                raw = input("(파일명 입력 후 Enter — 비워두면 자동 이름, q로 종료) capture> ")
            except EOFError:
                break
            if raw.strip().lower() in ("q", "quit", "exit"):
                break

            frame = node.snapshot()
            if frame is None:
                print("[캡처] 아직 받은 프레임이 없습니다 — 카메라 파이프라인을 확인하세요")
                continue

            label = safe_label(raw)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"{label}_{stamp}.jpg" if label else f"capture_{stamp}.jpg"
            path = os.path.join(out_dir, filename)
            cv2.imwrite(path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
            saved.append(path)
            print(f"[캡처] 저장: {path}")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)

    print(f"\n[캡처] 종료 — 총 {len(saved)}장")
    for path in saved:
        print(f"  {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default=TOPIC_DEFAULT)
    ap.add_argument("--out", default=OUT_DIR_DEFAULT)
    ap.add_argument("--rotate", type=int, default=0, choices=[0, 90, 180, 270],
                     help="추가 회전. 기본 토픽은 이미 보정돼 있으므로 기본값 0. "
                          "원본(rgb0/image)을 직접 구독할 땐 180을 줄 것")
    ap.add_argument("--quality", type=int, default=95)
    args = ap.parse_args()
    run(args.topic, args.out, args.rotate, args.quality)


if __name__ == "__main__":
    main()
