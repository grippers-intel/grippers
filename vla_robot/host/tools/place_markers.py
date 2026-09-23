"""바닥 마커를 붙이면서 두 카메라가 제대로 보는지 실시간으로 확인한다.

    python tools/place_markers.py
    python tools/place_markers.py --cams 0 1

창 세 개가 뜬다. STATUS 만 보면 된다.
    STATUS      마커별로 어느 카메라가 보는지 · 재투영오차 · 로봇 pose
    cam0/cam1   실제 영상 (검출된 마커에 테두리)

아무 창이나 클릭하고 q 로 끝낸다. **아무것도 저장하지 않는다** — 마음껏 붙였다 떼면서 본다.

## 무엇을 보고 판단하나

- 바닥 마커 4장은 **둘 다(BOTH OK)** 가 이상적이지만, 대향 배치에서는 각 카메라가
  가까운 2장만 보는 것이 정상이다. 최소 조건은 카메라마다 `aruco.min_floor_markers`
  (기본 2) 이상이다.
- 재투영오차가 실측 품질이다. **1 px 미만 GOOD · 2 px 미만 OK · 그 이상이면 다시 잴 것.**
  마커를 붙인 위치와 `host.yaml` 의 `floor_markers` 좌표가 어긋나면 여기서 바로 뜬다.
- 인쇄물은 **윗변이 +y(상자 쪽)를 향하게** 붙인다. 한 장이라도 돌아가 있으면 오차가
  수십 px 로 뛴다(코너 순서가 뒤집히기 때문이다).

⚠️ 캘리브레이션(`host/calib/cam{n}.npz`)이 없으면 근사 내부파라미터로 돌아 오차가
   수 cm 까지 커진다. 먼저 `tools/calibrate_camera.py` 를 돌릴 것.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

HOST_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST_ROOT))

import host_config  # noqa: E402
from localization.aruco_localizer import (Camera, RobotLocalizer, detect,  # noqa: E402
                                          make_detector)
from localization.cameras import open_cams, read_frames, release_all  # noqa: E402

W, H = 760, 500
FONT = cv2.FONT_HERSHEY_SIMPLEX
GREEN, RED, YELLOW, GREY, WHITE = ((90, 220, 90), (70, 70, 240), (60, 210, 250),
                                   (150, 150, 150), (240, 240, 240))


def marker_px(corners: np.ndarray) -> float:
    """마커 한 변이 화면에서 몇 px 인지."""
    return float(np.mean([np.linalg.norm(corners[(k + 1) % 4] - corners[k]) for k in range(4)]))


def quality(reproj: float) -> tuple[str, tuple]:
    """재투영오차로 '잘 쟀는지'를 가른다. 실측으로 얻은 기준이다."""
    if reproj == float("inf"):
        return "--", GREY
    if reproj < 1.0:
        return "GOOD", GREEN
    if reproj < 2.0:
        return "OK", YELLOW
    return "REMEASURE!", RED


def status_panel(cfg, cams, dets, pose) -> np.ndarray:
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

    robot_id = cfg.aruco.robot_marker_id
    rows = [(mid, f"ID {mid}") for mid in sorted(cfg.aruco.floor_markers)]
    rows.append((robot_id, f"ID {robot_id}  (robot)"))

    y = 116
    for mid, label in rows:
        seen = 0
        for k, det in enumerate(dets):
            if mid in det:
                seen += 1
                put(f"OK {marker_px(det[mid]):3.0f}px", 165 + k * 175, y, GREEN, 0.6)
            else:
                put("-- not seen", 165 + k * 175, y, RED, 0.6)
        if mid == robot_id:
            txt, col = (("BOTH", GREEN) if seen >= 2 else
                        ("ONE ONLY", YELLOW) if seen == 1 else ("LOST", RED))
        else:
            txt, col = (("BOTH OK", GREEN) if seen >= 2 else
                        ("ONE ONLY", YELLOW) if seen == 1 else ("NOT SEEN", RED))
        put(label, 24, y, WHITE, 0.6)
        put(txt, 515, y, col, 0.6)
        y += 32

    cv2.line(img, (20, y - 4), (W - 20, y - 4), (70, 70, 70), 1)
    y += 26

    need = cfg.aruco.min_floor_markers
    total = len(cfg.aruco.floor_markers)
    for k, c in enumerate(cams):
        col = GREEN if c.n_floor >= total else (YELLOW if c.n_floor >= need else RED)
        put(f"{c.name}: floor {c.n_floor}/{total}", 24 + k * 240, y, col, 0.62)
        q, qc = quality(c.reproj_px)
        rp = "--" if c.reproj_px == float("inf") else f"{c.reproj_px:.2f}px"
        put(f"reproj {rp} {q}", 24 + k * 240, y + 28, qc, 0.62)
        if c.locked:
            put("extrinsics LOCKED", 24 + k * 240, y + 54, GREEN, 0.55)
        else:
            put(f"locking {c.acc_n}/{cfg.aruco.extrinsic_lock_frames}",
                24 + k * 240, y + 54, GREY, 0.55)
        if not c.calibrated:
            put("NOT CALIBRATED", 24 + k * 240, y + 78, YELLOW, 0.55)
    y += 110

    cv2.line(img, (20, y - 8), (W - 20, y - 8), (70, 70, 70), 1)
    y += 24
    if pose.ok:
        a = cfg.arena
        inside = (a.workspace_x[0] <= pose.x <= a.workspace_x[1]
                  and a.workspace_y[0] <= pose.y <= a.workspace_y[1])
        put(f"ROBOT  x={pose.x * 1000:6.0f}mm  y={pose.y * 1000:6.0f}mm  "
            f"yaw={pose.yaw_deg:6.1f}deg  cams={pose.n_cams}", 24, y, WHITE, 0.62)
        put("IN WORKSPACE" if inside else "OUT OF WORKSPACE", 24, y + 28,
            GREEN if inside else YELLOW, 0.62)
    else:
        put("ROBOT  not located yet", 24, y, GREY, 0.62)
        put(f"(needs {need}+ floor markers and the robot marker)", 24, y + 28, GREY, 0.5)
    return img


def draw_view(frame, cam, det, robot_id: int):
    for mid, c in det.items():
        pts = c.astype(np.int32)
        col = (0, 165, 255) if mid == robot_id else (90, 220, 90)
        cv2.polylines(frame, [pts], True, col, 2)
        ctr = pts.mean(axis=0).astype(int)
        cv2.putText(frame, str(mid), (ctr[0] - 10, ctr[1] + 8), FONT, 0.9, col, 2, cv2.LINE_AA)
    cv2.putText(frame, f"{cam.name}  floor {cam.n_floor}", (12, 30), FONT, 0.7, WHITE, 2,
                cv2.LINE_AA)
    return frame


def main() -> int:
    host_config.configure_console()
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--cams", type=int, nargs="+", default=None)
    args = ap.parse_args()

    cfg = host_config.load_host_config(args.config)
    indices = args.cams if args.cams is not None else list(cfg.cameras.indices)
    detector = make_detector(cfg.aruco)
    cams = [Camera.load(i, cfg.cameras, cfg.aruco) for i in indices]
    caps = open_cams(indices, cfg.cameras.width, cfg.cameras.height)
    if not any(c.isOpened() for c in caps):
        print("열린 카메라가 없습니다. --cams 로 번호를 지정해 보십시오")
        release_all(caps)
        return 1

    print(__doc__)
    loc = RobotLocalizer(cfg.aruco)
    try:
        while True:
            frames = read_frames(caps)
            dets = [{} if f is None else detect(detector, cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
                    for f in frames]
            # 외부파라미터는 update() 안에서 푼다. 여기서 또 부르면 lock 프레임이 두 배로 센다.
            pose = loc.update(cams, dets)

            cv2.imshow("STATUS", status_panel(cfg, cams, dets, pose))
            for cam, f, d in zip(cams, frames, dets):
                if f is not None:
                    cv2.imshow(cam.name, draw_view(f, cam, d, cfg.aruco.robot_marker_id))
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    finally:
        release_all(caps)
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
