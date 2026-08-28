#!/usr/bin/env python3
"""이식한 경로로 C920 두 대를 동시에 열어 ArUco + geti 검출을 실시간으로 본다.

## 왜 이 도구가 있나

macOS 이식에서 **가장 불확실한 지점이 카메라 두 대**였다. Windows 는
DirectShow 열거 순서가 곧 `cv2` 인덱스라 `CAM_INDICES = (0, 1)` 이 그냥
맞았지만, macOS 에는 그런 보장이 없다.

실제로 이 맥에서는 이렇게 잡힌다.

    [0] FaceTime HD 카메라      <- 내장. 이것이 0번이다
    [1] HD Pro Webcam C920
    [2] HD Pro Webcam C920

`CAM_INDICES = (0, 1)` 을 그대로 쓰면 **내장 카메라를 cam0 으로 잡는다.**
그래서 이 도구는 인덱스를 하드코딩하지 않고 `camera_devices.resolve_indices()`
로 **이름(C920)으로 고른다** — 이식한 경로가 실제로 맞는지 눈으로 확인하는 것이
목적이다.

## 확장 디스플레이

`--display 1` 이면 확장 화면에 띄운다. macOS 의 전역 데스크톱 좌표를 Quartz 로
읽어서 그 화면의 origin 으로 창을 옮긴다 — 화면 배치가 바뀌어도 맞는다.
Quartz 가 없으면 주 화면에 뜬다.

## 실행

    python3 host/mac_live_view.py --display 1 --seconds 60
"""
from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "aruco"))

import cv2
import numpy as np

import camera_backend as backend
import camera_devices as devices
import config as cfg

WIN = "grippers host — macOS live"


def screen_origin(index: int) -> tuple[int, int]:
    """전역 데스크톱 좌표에서 해당 화면의 좌상단. 못 읽으면 (0, 0)."""
    try:
        import Quartz
    except ImportError:
        return (0, 0)
    _n, ids, _ = Quartz.CGGetActiveDisplayList(8, None, None)
    ordered = sorted(ids, key=lambda d: Quartz.CGDisplayBounds(d).origin.x)
    if index >= len(ordered):
        return (0, 0)
    b = Quartz.CGDisplayBounds(ordered[index])
    return (int(b.origin.x), int(b.origin.y))


def load_detector():
    """geti 배포본을 연다. 없으면 None — ArUco 만으로도 도구는 쓸모가 있다."""
    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "geti_sdk-deployment", "deployment")
    weights = os.path.join(folder, "Detection", "model", "model.bin")
    if not os.path.exists(weights):
        print("[geti] model.bin 이 없어 ArUco 만 표시한다", file=sys.stderr)
        return None
    from geti_sdk.deployment import Deployment
    dep = Deployment.from_folder(folder)
    dep.load_inference_models(device="CPU")
    return dep


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--display", type=int, default=1,
                    help="0=주 화면, 1=오른쪽 확장 화면")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--infer-interval", type=float, default=0.3,
                    help="geti 추론 주기. 원본 GETI_INFER_INTERVAL_S 와 같은 값")
    args = ap.parse_args()

    print("[backend]", backend.platform_note())
    for i, name, _uid in devices.list_video_devices():
        print(f"  [{i}] {name}")

    indices, names = devices.resolve_indices()
    print(f"[camera] 이름으로 고른 인덱스: {indices}  ({', '.join(names)})")
    if len(indices) < 2:
        print("⛔ C920 두 대를 못 찾았다", file=sys.stderr)
        print(backend.diagnose(), file=sys.stderr)
        return 1

    caps = [devices.open_camera(i) for i in indices[:2]]
    for i, cap in zip(indices, caps):
        if not cap.isOpened():
            print(f"⛔ 인덱스 {i} 를 못 열었다", file=sys.stderr)
            print(backend.diagnose(), file=sys.stderr)
            return 1

    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, cfg.ARUCO_DICT))
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    dep = load_detector()

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    ox, oy = screen_origin(args.display)
    cv2.moveWindow(WIN, ox + 20, oy + 20)
    cv2.resizeWindow(WIN, 1600, 480)
    print(f"[display] 화면 {args.display} origin=({ox}, {oy}) 에 창을 띄웠다")

    last_infer = 0.0
    boxes: list[list] = [[], []]
    marker_hits = [0, 0]
    frames = 0
    t_start = time.time()
    fps_t, fps_n, fps = time.time(), 0, 0.0

    while time.time() - t_start < args.seconds:
        panes = []
        do_infer = (time.time() - last_infer) >= args.infer_interval
        for n, cap in enumerate(caps):
            ok, frame = cap.read()
            if not ok:
                frame = np.zeros((cfg.IMG_H, cfg.IMG_W, 3), np.uint8)
            corners, ids, _ = detector.detectMarkers(frame)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                marker_hits[n] = len(ids)
            else:
                marker_hits[n] = 0

            if dep is not None and do_infer:
                pred = dep.infer(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                found = []
                for a in pred.annotations:
                    for l in a.labels:
                        if l.name != "No object" and l.probability >= 0.6:
                            s = a.shape
                            found.append((l.name, l.probability,
                                          int(s.x), int(s.y), int(s.width), int(s.height)))
                boxes[n] = found

            for label, p, x, y, w, h in boxes[n]:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 220, 0), 2)
                cv2.putText(frame, f"{label} {p:.2f}", (x, max(18, y - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)

            cv2.putText(frame, f"cam{n} idx={indices[n]}  aruco={marker_hits[n]}"
                               f"  geti={len(boxes[n])}",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            panes.append(cv2.resize(frame, (800, 450)))

        if do_infer:
            last_infer = time.time()

        view = np.hstack(panes)
        fps_n += 1
        if time.time() - fps_t >= 1.0:
            fps = fps_n / (time.time() - fps_t)
            fps_t, fps_n = time.time(), 0
        left = args.seconds - (time.time() - t_start)
        cv2.putText(view, f"{fps:4.1f} FPS   q=quit   {left:4.0f}s",
                    (10, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow(WIN, view)
        frames += 1
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

    for cap in caps:
        cap.release()
    cv2.destroyAllWindows()
    print(f"[결과] {frames} 프레임, 평균 {frames / (time.time() - t_start):.1f} FPS")
    print(f"[결과] 마지막 ArUco 검출: cam0={marker_hits[0]}개  cam1={marker_hits[1]}개")
    print(f"[결과] 마지막 geti 검출 : cam0={[b[0] for b in boxes[0]]}  "
          f"cam1={[b[0] for b in boxes[1]]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
