"""탑뷰 카메라 열기/읽기.

⚠️ 속성 설정 순서가 fps 를 가른다(hardware/grippers/tools/arm/rollout_policy.py 실측):
    해상도 -> FOURCC   MJPG 29.9 fps
    FOURCC -> 해상도   YUY2 10.0 fps
그래서 해상도를 먼저 넣고 FOURCC 를 넣는다.

⚠️ C920 오토포커스를 끈다. 캘리브레이션 때 고정한 초점과 달라지면 내부파라미터가 틀어진다.
"""
from __future__ import annotations

import sys

import cv2
import numpy as np


def open_cams(indices, width: int, height: int) -> list[cv2.VideoCapture]:
    backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
    caps = []
    for i in indices:
        cap = cv2.VideoCapture(int(i), backend)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        if not cap.isOpened():
            print(f"[cameras] ⚠️ 카메라 {i} 를 열 수 없습니다")
        else:
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if (w, h) != (width, height):
                # 캘리브레이션(npz)이 1280x720 기준이라 해상도가 다르면 좌표가 통째로 틀린다.
                print(f"[cameras] ⚠️ 카메라 {i}: 요청 {width}x{height}, 실제 {w}x{h}")
        caps.append(cap)
    return caps


def read_frames(caps) -> list[np.ndarray | None]:
    frames = []
    for cap in caps:
        ok, frame = cap.read() if cap.isOpened() else (False, None)
        frames.append(frame if ok else None)
    return frames


def release_all(caps) -> None:
    for cap in caps:
        try:
            cap.release()
        except Exception:  # noqa: BLE001 — 종료 경로
            pass
