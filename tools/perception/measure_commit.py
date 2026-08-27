#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""커밋 라인 측정 — 로봇을 뒤로 물리면서 물체의 화면 y 와 이동 거리를 함께 본다.

파지 지점은 화면 y=480 으로 **포화**돼 있어 거리를 못 잰다. 그래서 접근을
두 단계로 나눈다:

    시각 서보로 커밋 라인까지 → 거기서부터는 정해진 거리를 개루프로 전진

이 도구는 그 "커밋 라인 y" 와 "전진 거리" 를 한 번에 재기 위한 것이다.
사용법: 텔레옵을 띄운 채 이걸 돌리고, 물체를 그대로 둔 상태에서 **로봇만**
천천히 뒤로 물린다. y 가 480 에서 내려오는 것을 보며 400~440 쯤에서 멈추고,
그때 표시된 이동 거리를 읽으면 된다.

빠른 갱신을 위해 **한 프레임씩만** 검출한다(합의 필터 없음). 대략의 위치를
보며 운전하는 용도이고, 최종 확정값은 floor_observer 로 다시 재는 게 맞다.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image

sys.path.insert(0, "/grippers/tools/perception")
from floor_observer import to_bgr  # noqa: E402

TOPIC = "/ascamera/camera_publisher/rgb0/image"
MODEL = "/grippers/models/best.pt"  # 2026-08-27: best_ncnn_model은 Pi에 없던 경로였다


class CommitMeter(Node):
    def __init__(self, args):
        super().__init__("commit_meter")
        from ultralytics import YOLO
        self.model = YOLO(MODEL, task="detect")
        self.args = args
        self.frame = None
        self.origin = None       # 시작 위치 (x, y)
        self.pos = None
        self.create_subscription(Image, args.topic, self._on_img, 10)
        # **/odom 이 아니라 /odom_raw 다.** odom_publisher_node 는 바퀴 오도메트리를
        # odom_raw 로 내보내고, /odom 은 EKF(robot_localization)가 IMU 와 융합해
        # 만드는 토픽이다. 그 EKF 는 controller.launch.py 에 들어 있는데 이 컨테이너에
        # imu_calib 패키지가 없어 못 띄운다. 그래서 /odom 은 아예 존재하지 않는다.
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, 10)

    def _on_img(self, msg: Image):
        img = to_bgr(msg)
        self.frame = cv2.rotate(img, cv2.ROTATE_180)   # 카메라가 거꾸로 달려 있다

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y)
        if self.origin is None:
            self.origin = self.pos

    @property
    def travelled(self) -> float:
        if self.origin is None or self.pos is None:
            return float("nan")
        return math.hypot(self.pos[0] - self.origin[0], self.pos[1] - self.origin[1])

    def look(self):
        if self.frame is None:
            return None
        r = self.model.predict(self.frame, imgsz=640, conf=self.args.conf, verbose=False)[0]
        best = None
        for c, cf, b in zip(r.boxes.cls, r.boxes.conf, r.boxes.xyxy):
            name = self.model.names[int(c)]
            if self.args.cls and name != self.args.cls:
                continue
            x1, _, x2, y2 = [float(v) for v in b]
            cand = (name, (x1 + x2) / 2.0, y2, float(cf))
            if best is None or cand[2] > best[2]:      # 가장 가까운(아래) 것
                best = cand
        return best


def main():
    ap = argparse.ArgumentParser(description="커밋 라인 측정")
    ap.add_argument("--cls", default="rook", help="추적할 클래스. 빈 문자열이면 전부")
    ap.add_argument("--conf", type=float, default=0.45)
    ap.add_argument("--topic", default=TOPIC)
    ap.add_argument("--period", type=float, default=1.2, help="갱신 주기(초)")
    ap.add_argument("--odom-topic", default="/odom_raw",
                    help="바퀴 오도메트리 토픽. EKF 가 없어 /odom 이 아니라 /odom_raw 다")
    args = ap.parse_args()

    rclpy.init()
    node = CommitMeter(args)
    print("텔레옵으로 로봇을 **천천히 뒤로** 물리세요. 물체는 그대로 두고 로봇만 움직입니다.")
    print("y 가 480 에서 내려옵니다. 400~440 쯤에서 멈추고 그때의 이동 거리를 쓰세요.")
    print("Ctrl-C 로 종료\n")
    print(f"  {'물체':<8} {'화면 x':>7} {'화면 y':>7} {'신뢰':>6} {'이동거리':>9}")

    t_next = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() < t_next:
                continue
            t_next = time.monotonic() + args.period
            hit = node.look()
            d = node.travelled
            if node.origin is None:
                warned = getattr(node, "_odom_warned", False)
                if not warned:
                    print(f"  ⚠ {args.odom_topic} 에서 오도메트리가 안 옵니다 — "
                          f"베이스 스택이 떠 있는지 확인하세요 (./teleop.sh --base-only)")
                    node._odom_warned = True
            if hit is None:
                print(f"  {'(검출 없음)':<8} {'—':>7} {'—':>7} {'—':>6} {d:>8.3f}m")
            else:
                name, x, y, cf = hit
                flag = "  ← 포화" if y >= 478 else ""
                dist = "  —  " if node.origin is None else f"{d:>8.3f}m"
                print(f"  {name:<8} {x:>7.0f} {y:>7.0f} {cf:>6.2f} {dist}{flag}")
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n최종 이동 거리: {node.travelled:.3f} m")
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
