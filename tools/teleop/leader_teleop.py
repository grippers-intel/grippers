#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""노트북 쪽 조종기 — 리더 암(팔) + 키보드(베이스)를 한 터미널에서 다룬다.

한 손은 리더 암을 잡고, 다른 손은 키보드에 둔다. 팔과 베이스를 각각 다른
터미널로 나누면 시연 도중에 창을 옮겨 다녀야 해서 그 자체가 실수를 부른다.

리더 암은 토크를 꺼서 자유롭게 움직이는 상태로 두고 위치만 읽는다.
"""
from __future__ import annotations

import argparse
import select
import socket
import sys
import termios
import time
import tty

from driver_sdk import JOINT_IDS, JOINT_NAMES, STS3215Driver
from teleop_protocol import DEFAULT_PORT, encode

# 파이의 IPv6 링크로컬. 핫스팟의 글로벌 프리픽스는 세션마다 바뀌지만
# 이 주소는 인터페이스에 고정이라 기본값으로 쓰기에 더 안전하다.
DEFAULT_HOST = "fe80::4617:1d27:ddf2:de43%en0"

BASE_KEYS = {          # 키 → 베이스 (x, y, θ) 방향
    "w": (1, 0, 0), "s": (-1, 0, 0),
    "a": (0, 1, 0), "d": (0, -1, 0),
    "q": (0, 0, 1), "e": (0, 0, -1),
}

HELP = """
┌─ 조종 ────────────────────────────────────────────────┐
│  팔    리더 암을 손으로 움직이면 팔로워가 따라옵니다   │
│        f      추종 켜기/끄기 (켜는 순간이 기준점)      │
│                                                        │
│  베이스 w/s    전진/후진     a/d    좌/우 평행이동     │
│         q/e    좌/우 회전    SPACE  즉시 정지          │
│         z/x    속도 -/+                                │
│                                                        │
│  종료  Ctrl-C                                          │
└────────────────────────────────────────────────────────┘"""


def main():
    ap = argparse.ArgumentParser(description="노트북 쪽 텔레옵 조종기")
    ap.add_argument("--leader-port", default="/dev/cu.usbmodem5B3D0450831")
    ap.add_argument("--host", default=DEFAULT_HOST, help="파이 주소")
    ap.add_argument("--udp-port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--rate", type=float, default=50.0, help="송신 주파수 Hz")
    ap.add_argument("--scale", type=float, default=0.6, help="베이스 최대속도 대비 배율")
    ap.add_argument("--key-timeout", type=float, default=0.35,
                    help="이 시간 동안 방향키 입력이 없으면 베이스를 세운다")
    ap.add_argument("--keep-leader-torque", action="store_true",
                    help="리더 토크를 끄지 않는다(기본은 꺼서 손으로 움직이게 함)")
    ap.add_argument("--headless", action="store_true",
                    help="키 입력 없이 즉시 팔 추종(터미널 없이 실행할 때)")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="이 시간(초) 뒤 자동 종료. 0이면 무제한")
    args = ap.parse_args()

    # 주소 해석 — 링크로컬의 %en0 스코프까지 여기서 처리된다.
    try:
        info = socket.getaddrinfo(args.host, args.udp_port,
                                  socket.AF_UNSPEC, socket.SOCK_DGRAM)[0]
    except socket.gaierror as exc:
        sys.exit(f"주소 해석 실패 {args.host}: {exc}")
    family, _, _, _, sockaddr = info
    sock = socket.socket(family, socket.SOCK_DGRAM)

    drv = STS3215Driver(port=args.leader_port)
    if not drv.connect():
        sys.exit(f"리더 암 열기 실패: {args.leader_port}\n"
                 f"포트 확인: ls /dev/cu.usbmodem*")
    dead = [s for s in JOINT_IDS if not drv.ping(s)]
    if dead:
        drv.disconnect()
        sys.exit(f"리더 서보 응답 없음 id={dead} — 전원/케이블을 확인하세요")

    if not args.keep_leader_torque:
        drv.set_all_torque(False)   # 사람이 들고 움직여야 하므로 토크는 꺼둔다

    interactive = not args.headless
    print(f"리더 {args.leader_port} → 파이 {args.host}:{args.udp_port} @ {args.rate:.0f}Hz")
    if interactive:
        print(HELP)

    period = 1.0 / args.rate
    seq, epoch, engaged = 0, 0, False
    scale = args.scale
    base = (0.0, 0.0, 0.0)
    last_dir_key = 0.0
    if args.headless:
        epoch, engaged = 1, True

    old_tty = termios.tcgetattr(sys.stdin) if interactive else None
    t_end = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        if interactive:
            tty.setcbreak(sys.stdin.fileno())
        next_t = time.monotonic()
        while True:
            if t_end and time.monotonic() >= t_end:
                break

            if interactive and select.select([sys.stdin], [], [], 0.0)[0]:
                k = sys.stdin.read(1)
                if k == "\x03":                    # Ctrl-C
                    break
                if k == "f":
                    engaged = not engaged
                    if engaged:
                        epoch += 1   # 파이가 이 변화를 보고 기준점을 다시 잡는다
                elif k == " ":
                    base, last_dir_key = (0.0, 0.0, 0.0), 0.0
                elif k == "z":
                    scale = max(0.1, round(scale - 0.1, 1))
                elif k == "x":
                    scale = min(1.0, round(scale + 0.1, 1))
                elif k in BASE_KEYS:
                    base, last_dir_key = BASE_KEYS[k], time.monotonic()

            # 키를 떼면(=반복 입력이 끊기면) 베이스는 스스로 선다.
            # 이게 없으면 마지막 방향으로 계속 굴러간다.
            if base != (0.0, 0.0, 0.0) and \
                    time.monotonic() - last_dir_key > args.key_timeout:
                base = (0.0, 0.0, 0.0)

            pos = [drv.get_all_positions().get(sid) for sid in JOINT_IDS]
            seq += 1
            sock.sendto(encode(seq, epoch, engaged, pos, base, scale), sockaddr)

            if interactive or seq % 25 == 0:
                arm = "\033[32m추종중\033[0m" if engaged else "\033[33m대기  \033[0m"
                bx, by, bt = base
                mv = "정지" if base == (0.0, 0.0, 0.0) else f"x{bx:+.0f} y{by:+.0f} θ{bt:+.0f}"
                joints = " ".join(f"{n[:4]}:{p if p is not None else '----':>4}"
                                  for n, p in zip(JOINT_NAMES, pos))
                print(f"\r 팔[{arm}] 베이스[{mv:>14}] 배율{scale:.1f}  {joints} ",
                      end="" if interactive else "\n", flush=True)

            next_t += period
            time.sleep(max(0.0, next_t - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        if old_tty is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty)
        # 종료 알림 — 파이가 데드맨 0.4초를 기다리지 않고 즉시 멈추게 한다.
        for _ in range(5):
            seq += 1
            sock.sendto(encode(seq, epoch, False, [None] * 6, (0, 0, 0), 0.0), sockaddr)
            time.sleep(0.02)
        drv.disconnect()
        print("\n정지 명령 전송 후 종료")


if __name__ == "__main__":
    main()
