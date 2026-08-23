#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MentorPi 메카넘 베이스 키보드 텔레옵 — /cmd_vel 발행.

peripherals/teleop_key_control.py 를 그대로 안 쓰는 이유: 그 노드는 전후·회전만
낸다(linear.x, angular.z). 이 로봇은 메카넘이라 linear.y 로 **옆으로 평행이동**이
되고, odom_publisher_node 의 cmd_vel_callback 도 메카넘 분기에서 linear.y 를
실제로 쓴다. 자율주행 영상처럼 보이려면 이 축이 필요하다 — 상자 앞에서 몸을
돌리지 않고 옆으로 붙는 움직임이 나온다.

속도 상한은 odom_publisher_node.app_cmd_vel_callback 의 안전 클램프와 같은
값으로 맞췄다(선 0.2m/s, 각 0.5rad/s). 더 크게 보내도 어차피 거기서 잘린다.

키:
  w/s   전진/후진        a/d   좌/우 평행이동(스트레이프)
  q/e   좌/우 제자리회전  SPACE 즉시 정지
  z/x   속도 -/+         Ctrl-C 종료
"""
from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

MAX_LIN = 0.2   # m/s  — 컨트롤러 클램프와 동일
MAX_ANG = 0.5   # rad/s

BINDINGS = {           # 키 → (x, y, θ) 방향
    "w": (1, 0, 0), "s": (-1, 0, 0),
    "a": (0, 1, 0), "d": (0, -1, 0),
    "q": (0, 0, 1), "e": (0, 0, -1),
}


class BaseTeleop(Node):
    def __init__(self, args):
        super().__init__("base_teleop_mecanum")
        self.pub = self.create_publisher(Twist, args.topic, 10)
        self.args = args
        self.scale = args.scale
        self.vec = (0.0, 0.0, 0.0)
        self.last_key = 0.0

    def publish(self):
        x, y, t = self.vec
        msg = Twist()
        msg.linear.x = x * MAX_LIN * self.scale
        msg.linear.y = y * MAX_LIN * self.scale
        msg.angular.z = t * MAX_ANG * self.scale
        self.pub.publish(msg)

    def stop(self):
        self.vec = (0.0, 0.0, 0.0)
        self.pub.publish(Twist())


def main():
    ap = argparse.ArgumentParser(description="MentorPi 메카넘 키보드 텔레옵")
    ap.add_argument("--topic", default="cmd_vel")
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--scale", type=float, default=0.6, help="최대 속도 대비 배율")
    ap.add_argument("--no-strafe", action="store_true",
                    help="게걸음(a/d)을 막는다. **좌표 교시 주행용** — 이 베이스의 "
                         "오도메트리는 차동구동 모델이라 횡이동을 아예 안 센다. "
                         "게걸음으로 간 거리는 없던 일이 되어 교시 좌표가 조용히 "
                         "틀어진다(2026-08-23 실측)")
    ap.add_argument("--key-timeout", type=float, default=0.35,
                    help="이 시간 동안 키 입력이 없으면 정지한다")
    args = ap.parse_args()

    rclpy.init()
    node = BaseTeleop(args)
    print(__doc__.split("키:")[1])
    if args.no_strafe:
        print("  ** 게걸음(a/d) 금지 모드 — 좌표 교시용. w/s/q/e 만 쓰세요 **")
    print(f"토픽 {args.topic}  배율 {args.scale:.1f}  "
          f"(최대 {MAX_LIN*args.scale:.2f}m/s, {MAX_ANG*args.scale:.2f}rad/s)\n")

    old = termios.tcgetattr(sys.stdin)
    period = 1.0 / args.rate
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            r, _, _ = select.select([sys.stdin], [], [], period)
            if r:
                k = sys.stdin.read(1)
                if k == "\x03":
                    break
                if k == " ":
                    node.stop()
                elif k == "z":
                    node.scale = max(0.1, node.scale - 0.1)
                elif k == "x":
                    node.scale = min(1.0, node.scale + 0.1)
                elif args.no_strafe and k in ("a", "d"):
                    node.stop()
                    print("\r  게걸음 금지 모드입니다 — 오도메트리가 횡이동을 "
                          "못 재서 좌표가 틀어집니다. q/e 로 돌고 w 로 가세요.   ",
                          flush=True)
                elif k in BINDINGS:
                    node.vec = BINDINGS[k]
                    node.last_key = time.monotonic()
            # 키를 떼면(=반복 입력이 끊기면) 스스로 멈춘다. 이게 없으면
            # 마지막 방향으로 계속 굴러간다.
            elif time.monotonic() - node.last_key > args.key_timeout:
                node.vec = (0.0, 0.0, 0.0)

            node.publish()
            x, y, t = node.vec
            print(f"\r x{x:+.0f} y{y:+.0f} θ{t:+.0f}  배율 {node.scale:.1f}  ",
                  end="", flush=True)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        print("\n정지 명령 발행 후 종료")


if __name__ == "__main__":
    main()
