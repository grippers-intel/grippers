"""탑뷰 카메라 열기/읽기.

⚠️ 속성 설정 순서가 fps 를 가른다(hardware/grippers/tools/arm/rollout_policy.py 실측):
    해상도 -> FOURCC   MJPG 29.9 fps
    FOURCC -> 해상도   YUY2 10.0 fps
그래서 해상도를 먼저 넣고 FOURCC 를 넣는다.

## 초점은 끄는 것만으로는 고정되지 않는다 (C920)

C920 은 초점이 움직이면 `fx`·`fy` 가 같이 변한다 — `calib/*.npz` 는 특정 초점에서 잰 값이다.
그런데 Windows DSHOW 에서 `AUTOFOCUS=0` 만 걸면 초점이 그 자리(흔히 최근접 250)에 멈춰
**바닥 마커가 한 장도 안 잡힌 적이 있다**(ArUco_C920/run_localize.py 기록). 그래서
오토포커스를 **먼저 끄고 → 초점값을 넣는다**(순서를 바꾸면 다음 재초점 때 덮어써진다).
스트림이 시작되기 전의 설정 쓰기를 흘리는 백엔드가 있어 **첫 프레임 뒤에 한 번 더** 건다.
값은 `cameras.focus`(0 = 먼 곳). 1.6 m 높이에서 1~2 m 앞 바닥을 보므로 먼 쪽이 맞다.
"""
from __future__ import annotations

import sys
from typing import Optional

import cv2
import numpy as np


def _lock_focus(cap, focus: Optional[int]) -> None:
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    if focus is not None and focus >= 0:
        cap.set(cv2.CAP_PROP_FOCUS, focus)


def open_cams(indices, width: int, height: int,
              focus: Optional[int] = None) -> list[cv2.VideoCapture]:
    """카메라를 연다. focus 가 0 이상이면 그 값으로 초점을 고정한다(None/음수 = 끄기만)."""
    backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
    caps = []
    for i in indices:
        cap = cv2.VideoCapture(int(i), backend)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        _lock_focus(cap, focus)
        if not cap.isOpened():
            print(f"[cameras] ⚠️ 카메라 {i} 를 열 수 없습니다")
        else:
            cap.read()                  # 스트림을 띄운 뒤
            _lock_focus(cap, focus)     # 한 번 더 — 시작 전 설정을 흘리는 백엔드가 있다
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if (w, h) != (width, height):
                # 캘리브레이션(npz)이 1280x720 기준이라 해상도가 다르면 좌표가 통째로 틀린다.
                print(f"[cameras] ⚠️ 카메라 {i}: 요청 {width}x{height}, 실제 {w}x{h}")
            if focus is not None and focus >= 0:
                got = cap.get(cv2.CAP_PROP_FOCUS)
                # 되읽은 FOCUS 는 DSHOW 에서 믿을 만하다(쓴 값 그대로 나온다, 옵시디언 실측).
                # AUTOFOCUS 되읽기는 백엔드마다 뜻이 달라 판정에 쓰지 않는다.
                if int(round(got)) != int(focus):
                    print(f"[cameras] ⚠️ 카메라 {i}: 초점 {focus} 을 넣었는데 {got:.0f} 로 읽힌다")
                else:
                    print(f"[cameras] 카메라 {i}: 초점 고정 FOCUS={focus}")
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
