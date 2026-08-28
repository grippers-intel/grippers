"""마커를 붙이면서 두 카메라가 잘 보는지 실시간으로 확인하는 도우미.

사용법
    python place_markers.py
    python place_markers.py --cams 0 2

화면 세 개가 뜬다.
    STATUS      : 마커별로 어느 카메라가 보는지 (이것만 보면 된다)
    cam0 / cam1 : 실제 카메라 영상

끝낼 때는 아무 창이나 클릭하고 q.

이 프로그램은 아무것도 저장하지 않는다. 마음껏 붙였다 떼면서 보면 된다.
"""

import argparse
import sys

import cv2
import numpy as np

import config as cfg
from localizer import Camera, RobotLocalizer, detect, make_detector

W, H = 760, 470
FONT = cv2.FONT_HERSHEY_SIMPLEX

GREEN = (90, 220, 90)
RED = (70, 70, 240)
YELLOW = (60, 210, 250)
GREY = (150, 150, 150)
WHITE = (240, 240, 240)


def marker_px(corners: np.ndarray) -> float:
    """마커 한 변이 화면에서 몇 px 인지."""
    return float(np.mean([np.linalg.norm(corners[(k + 1) % 4] - corners[k])
                          for k in range(4)]))


def quality(reproj: float) -> tuple[str, tuple]:
    """재투영오차로 '잘 쟀는지'를 판정. 측정으로 얻은 기준이다."""
    if reproj == float("inf"):
        return "--", GREY
    if reproj < 1.0:
        return "GOOD", GREEN
    if reproj < 2.0:
        return "OK", YELLOW
    return "REMEASURE!", RED


def status_panel(cams, dets, pose) -> np.ndarray:
    img = np.full((H, W, 3), 28, np.uint8)

    def put(txt, x, y, col=WHITE, sc=0.6, th=1):
        cv2.putText(img, txt, (x, y), FONT, sc, col, th, cv2.LINE_AA)

    put("MARKER PLACEMENT HELPER", 20, 34, WHITE, 0.85, 2)
    put("q = quit", W - 110, 34, GREY, 0.55)
    cv2.line(img, (20, 48), (W - 20, 48), (70, 70, 70), 1)

    put("MARKER", 24, 82, GREY, 0.55)
    for k, c in enumerate(cams):
        put(c.name.upper(), 165 + k * 175, 82, GREY, 0.55)
    put("RESULT", 515, 82, GREY, 0.55)

    rows = [(mid, f"ID {mid}" + ("  (robot)" if mid == cfg.ROBOT_MARKER_ID else ""))
            for mid in (list(cfg.FLOOR_MARKER_IDS) + [cfg.ROBOT_MARKER_ID])]

    y = 116
    for mid, label in rows:
        seen = []
        for k, det in enumerate(dets):
            if mid in det:
                seen.append(True)
                put(f"OK {marker_px(det[mid]):3.0f}px", 165 + k * 175, y, GREEN, 0.6)
            else:
                seen.append(False)
                put("-- not seen", 165 + k * 175, y, RED, 0.6)
        n = sum(seen)
        if mid == cfg.ROBOT_MARKER_ID:
            txt, col = ("BOTH", GREEN) if n == 2 else \
                       ("ONE ONLY", YELLOW) if n == 1 else ("LOST", RED)
        else:
            txt, col = ("BOTH OK", GREEN) if n == 2 else \
                       ("NEED BOTH", YELLOW) if n == 1 else ("NOT SEEN", RED)
        put(label, 24, y, WHITE, 0.6)
        put(txt, 515, y, col, 0.6)
        y += 34

    cv2.line(img, (20, y - 4), (W - 20, y - 4), (70, 70, 70), 1)
    y += 26

    # 바닥 마커 개수와 측정 품질
    for k, c in enumerate(cams):
        n = c.n_floor
        col = GREEN if n == 4 else (YELLOW if n >= 2 else RED)
        put(f"{c.name}: floor {n}/4", 24 + k * 240, y, col, 0.62)
        q, qc = quality(c.reproj_px)
        rp = "--" if c.reproj_px == float("inf") else f"{c.reproj_px:.2f}px"
        put(f"reproj {rp} {q}", 24 + k * 240, y + 28, qc, 0.62)
        if c.locked:
            put("extrinsics LOCKED", 24 + k * 240, y + 54, GREEN, 0.55)
        elif cfg.EXTRINSIC_LOCK_FRAMES:
            put(f"locking {c._acc_n}/{cfg.EXTRINSIC_LOCK_FRAMES}",
                24 + k * 240, y + 54, GREY, 0.55)
    y += 92

    if not cams[0].calibrated:
        put("WARNING: not calibrated - run calibrate_camera.py first",
            24, y, YELLOW, 0.55)
    y += 26

    cv2.line(img, (20, y - 8), (W - 20, y - 8), (70, 70, 70), 1)
    y += 24
    if pose.ok:
        inside = cfg.in_workspace(pose.x, pose.y)
        put(f"ROBOT  x={pose.x*1000:6.0f}mm  y={pose.y*1000:6.0f}mm  "
            f"yaw={pose.yaw_deg:6.1f}deg", 24, y, WHITE, 0.62)
        put("IN WORKSPACE" if inside else "OUT OF WORKSPACE",
            24, y + 28, GREEN if inside else YELLOW, 0.62)
    else:
        put("ROBOT  not located yet", 24, y, GREY, 0.62)
        put(f"(needs {cfg.MIN_FLOOR_MARKERS}+ floor markers and the robot marker)",
            24, y + 28, GREY, 0.5)
    return img


def draw_view(frame, cam, det):
    for mid, c in det.items():
        pts = c.astype(np.int32)
        col = (0, 165, 255) if mid == cfg.ROBOT_MARKER_ID else (90, 220, 90)
        cv2.polylines(frame, [pts], True, col, 2)
        ctr = pts.mean(axis=0).astype(int)
        cv2.putText(frame, str(mid), (ctr[0] - 10, ctr[1] + 8),
                    FONT, 0.9, col, 2, cv2.LINE_AA)
    cv2.putText(frame, f"{cam.name}  floor {cam.n_floor}/4", (12, 30),
                FONT, 0.7, WHITE, 2, cv2.LINE_AA)
    return frame


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cams", type=int, nargs="+", default=list(cfg.CAM_INDICES))
    ap.add_argument("--view-scale", type=float, default=0.5,
                    help="미리보기 창 크기 배율 (기본 0.5). 1080p 를 원본으로 띄우면 "
                         "화면을 넘는다. 창 모서리를 끌어 조절해도 비율은 유지된다.")
    args = ap.parse_args()

    detector = make_detector()
    cams = [Camera.load(f"cam{i}", i) for i in args.cams]
    caps = []
    for i in args.cams:
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.IMG_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.IMG_H)
        # ⚠️ FOURCC 는 반드시 해상도 '뒤에' — 먼저 걸면 DSHOW 가 무시하고
        #    YUY2 로 남아 1080p 에서 5 fps 가 된다.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)   # C920 오토포커스 끄기 — 캘리브레이션 때 고정한 초점과 어긋나면 안 됨
        caps.append(cap)
    if not any(c.isOpened() for c in caps):
        print("열린 카메라가 없습니다. --cams 로 번호를 지정해 보세요.")
        for c in caps:
            c.release()
        return 1

    # 1080p 프레임을 원본 크기로 띄우면 화면을 넘는다. WINDOW_NORMAL 로 열어야
    # 크기 조절이 되고, WINDOW_KEEPRATIO 가 있어야 끌어도 가로세로 비율이 유지된다.
    for cam in cams:
        cv2.namedWindow(cam.name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(cam.name,
                         int(cfg.IMG_W * args.view_scale),
                         int(cfg.IMG_H * args.view_scale))

    print(__doc__)
    loc = RobotLocalizer()
    try:
        while True:
            frames, dets = [], []
            for cap in caps:
                ok, f = cap.read()
                frames.append(f if ok else None)
                dets.append({} if not ok else
                            detect(detector, cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)))
            pose = loc.update(cams, dets)

            cv2.imshow("STATUS", status_panel(cams, dets, pose))
            for cam, f, d in zip(cams, frames, dets):
                if f is not None:
                    cv2.imshow(cam.name, draw_view(f, cam, d))
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    finally:
        for c in caps:
            c.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
