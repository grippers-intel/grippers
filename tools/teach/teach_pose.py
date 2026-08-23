#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""파지 자세 교시 — 리드암으로 만든 자세를 이름 붙여 저장한다.

IK 를 풀지 않는다. 물체는 전부 바닥에 있고 위에서 집으므로, 사람이 리드암으로
한 번 보여준 자세를 그대로 재생하면 된다. 베이스가 위치를 맞춰주므로 팔은
거의 같은 동작만 반복한다.

**시리얼 포트를 직접 열지 않는다.** 텔레옵의 팔로워 노드가 /dev/soarm 을 점유하고
있어 두 프로세스가 같이 못 연다. 대신 그 노드가 발행하는 /teleop/follower_counts
토픽에서 현재 관절값을 받는다. 그래서 **텔레옵이 돌고 있어야** 한다.

사용법 (텔레옵을 띄워둔 채, 다른 터미널에서):
    리드암으로 자세를 만든 뒤 → ./teach approach
    다시 자세를 만든 뒤       → ./teach grasp
    목록 보기                 → ./teach --list

저장 위치: /grippers/config/grasp_poses.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

TOPIC = "/teleop/follower_counts"
STORE = "/grippers/config/grasp_poses.json"
JOINTS = ["Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "Gripper"]


class PoseGrabber(Node):
    def __init__(self, topic: str):
        super().__init__("pose_grabber")
        self.counts = None
        self.create_subscription(Int32MultiArray, topic, self._on_msg, 10)

    def _on_msg(self, msg: Int32MultiArray):
        if len(msg.data) == 6 and all(v >= 0 for v in msg.data):
            self.counts = list(msg.data)


def load() -> dict:
    if os.path.exists(STORE):
        with open(STORE, encoding="utf-8") as f:
            return json.load(f)
    return {}


def save(d: dict) -> None:
    os.makedirs(os.path.dirname(STORE), exist_ok=True)
    with open(STORE, "w", encoding="utf-8") as f:
        json.dump(d, f, ensure_ascii=False, indent=2)


def show(d: dict) -> None:
    if not d:
        print("저장된 자세가 없습니다."); return
    print(f"저장된 자세 {len(d)}개 — {STORE}\n")
    print(f"  {'이름':<12} " + " ".join(f"{j[:5]:>6}" for j in JOINTS))
    for name, rec in d.items():
        print(f"  {name:<12} " + " ".join(f"{c:>6}" for c in rec["counts"]))


def main():
    ap = argparse.ArgumentParser(description="파지 자세 교시")
    ap.add_argument("label", nargs="?", help="자세 이름 (approach, grasp, lift, drop, home …)")
    ap.add_argument("--topic", default=TOPIC)
    ap.add_argument("--list", action="store_true", help="저장된 자세 목록")
    ap.add_argument("--delete", metavar="이름", help="자세 삭제")
    ap.add_argument("--timeout", type=float, default=8.0)
    args = ap.parse_args()

    if args.list:
        show(load()); return
    if args.delete:
        d = load()
        if args.delete in d:
            del d[args.delete]; save(d); print(f"삭제됨: {args.delete}")
        else:
            print(f"그런 자세가 없습니다: {args.delete}")
        return
    if not args.label:
        ap.error("자세 이름이 필요합니다. 목록은 --list")

    rclpy.init()
    node = PoseGrabber(args.topic)
    t0 = time.monotonic()
    while node.counts is None and time.monotonic() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    counts = node.counts
    node.destroy_node(); rclpy.shutdown()

    if counts is None:
        print(f"관절값을 못 받았습니다 ({args.topic}).\n"
              f"텔레옵이 돌고 있는지 확인하세요 — 팔로워 노드가 이 토픽을 발행합니다.",
              file=sys.stderr)
        sys.exit(1)

    d = load()
    existed = args.label in d
    d[args.label] = {"counts": counts, "saved_at": time.strftime("%Y-%m-%d %H:%M:%S")}
    save(d)
    print(f"{'덮어씀' if existed else '저장'}: {args.label}")
    print("  " + " ".join(f"{j[:5]}={c}" for j, c in zip(JOINTS, counts)))


if __name__ == "__main__":
    main()
