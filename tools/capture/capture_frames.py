#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""로봇 카메라 프레임을 JPEG로 저장한다 — 인식 개발용 데이터 수집.

네트워크 없이 로컬에서 돌도록 만들었다. SSH로 띄워놓고 랜선을 뽑은 뒤
로봇을 손으로 밀며 촬영하고, 끝나면 다시 연결해 회수하면 된다.

rosbag 대신 JPEG를 쓰는 이유:
  - 원본 이미지 토픽 rosbag은 640x480 30fps 기준 분당 1.6GB. 디스크가 못 버틴다
  - 팀원들이 ROS 없이도 바로 열어볼 수 있다
  - 다중 프레임 합의 필터 개발에는 프레임 시퀀스면 충분하다

**카메라는 반드시 심볼릭 링크로 연다** — 재부팅하면 /dev/video 번호가 바뀐다.
"""
from __future__ import annotations

import argparse
import json
import os
import time

import cv2


def main():
    ap = argparse.ArgumentParser(description="카메라 프레임 캡처")
    ap.add_argument("--device", default="/dev/depth_cam",
                    help="카메라. 번호(video0)가 아니라 심볼릭 링크를 쓸 것")
    ap.add_argument("--out", default="/grippers/recordings",
                    help="저장 위치(호스트의 ~/docker/shared/grippers/recordings)")
    ap.add_argument("--fps", type=float, default=5.0, help="저장 주기")
    ap.add_argument("--duration", type=float, default=180.0, help="촬영 시간(초)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fourcc", default="MJPG",
                    help="픽셀 포맷. 뎁스캠은 MJPG여야 순수 RGB가 나온다")
    ap.add_argument("--quality", type=int, default=92)
    ap.add_argument("--label", default="",
                    help="폴더 이름에 붙일 꼬리표. 예: pos1, empty, layoutB")
    ap.add_argument("--warmup", type=float, default=2.0,
                    help="자동 노출이 안정될 때까지 버리는 시간")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        raise SystemExit(f"카메라 열기 실패: {args.device}")
    # 포맷을 먼저 정하고 해상도를 정해야 한다. 순서가 바뀌면 무시된다.
    #
    # 이 뎁스 카메라는 YUYV 1280x1040 으로 RGB와 뎁스를 한 프레임에 쌓아서 내보낸다.
    # OpenCV 기본값이 그쪽이라 그대로 두면 초록/보라 띠만 찍힌다. MJPG 1280x720 이
    # 순수 RGB 스트림이다.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    got = "".join(chr((fcc >> (8 * i)) & 0xFF) for i in range(4))
    if got != args.fourcc or (w, h) != (args.width, args.height):
        print(f"[캡처] 경고 — 요청 {args.fourcc} {args.width}x{args.height}, "
              f"실제 {got} {w}x{h}", flush=True)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    tail = f"_{args.label}" if args.label else ""
    out = os.path.join(args.out, f"frames_{stamp}{tail}")
    os.makedirs(out, exist_ok=True)
    print(f"[캡처] {args.device} {got} {w}x{h} → {out}", flush=True)

    # 자동 노출 안정화 — 이 프레임들은 버린다
    t_warm = time.monotonic()
    while time.monotonic() - t_warm < args.warmup:
        cap.read()
    print(f"[캡처] 워밍업 {args.warmup}s 완료, {args.duration:.0f}초 촬영 시작", flush=True)

    period = 1.0 / args.fps
    n, t0, next_t = 0, time.monotonic(), time.monotonic()
    enc = [int(cv2.IMWRITE_JPEG_QUALITY), args.quality]
    try:
        while time.monotonic() - t0 < args.duration:
            ok, frame = cap.read()
            if not ok:
                print("[캡처] 프레임 읽기 실패 — 계속", flush=True)
                continue
            cv2.imwrite(os.path.join(out, f"{n:05d}.jpg"), frame, enc)
            n += 1
            if n % 25 == 0:
                el = time.monotonic() - t0
                print(f"[캡처] {n}장 · {el:.0f}/{args.duration:.0f}초", flush=True)
            next_t += period
            time.sleep(max(0.0, next_t - time.monotonic()))
    except KeyboardInterrupt:
        print("[캡처] 중단됨", flush=True)
    finally:
        cap.release()
        el = time.monotonic() - t0
        meta = {"device": args.device, "label": args.label,
                "fourcc": got, "width": w, "height": h,
                "target_fps": args.fps, "frames": n,
                "elapsed_sec": round(el, 1),
                "actual_fps": round(n / el, 2) if el else 0}
        with open(os.path.join(out, "manifest.json"), "w") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        mb = sum(os.path.getsize(os.path.join(out, x))
                 for x in os.listdir(out)) / 1e6
        print(f"[캡처] 완료 — {n}장, {el:.0f}초, {mb:.0f}MB", flush=True)
        print(f"[캡처] 저장 위치: {out}", flush=True)


if __name__ == "__main__":
    main()
