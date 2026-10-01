"""탑뷰 카메라에서 MediaPipe 손 검출이 되는지 시험한다 — 미션·Geti 없이 손만 본다.

    python tools/hand_probe.py --live                       # 두 카메라 실시간 (q 종료)
    python tools/hand_probe.py --images ../../ArUco_C920/dataset_hand --out hand_probe_out

사전 준비: `pip install mediapipe` (host venv, numpy·opencv·protobuf 는 그대로 둔다) +
`models/hand_landmarker.task`
(https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task).

## 왜 조각으로 나눠 보나

MediaPipe 손바닥 검출기는 입력을 192×192 로 줄여 본다. 1.6 m 위 카메라에서 30~35 cm 높이의
손은 1280×720 화면에서 ~110 px 이라, 화면 전체를 줄이면 ~17 px 로 작아져 놓치기 쉽다.
`--tile` 크기의 정사각 조각(겹침 25 %)마다 따로 돌리면 손이 조각 안에서 커진다.
`--mode` 로 `full`(전체 한 번) · `tile`(조각) · `both`(둘 다, 비교용) 를 고른다.

## 손 좌표

손바닥 중심(손목 + 손가락 뿌리 4개 평균)의 광선을 `--hand-z`(기본 0.32 m) 높이 평면과
교차시킨다 — 로봇 마커를 마커 높이로 푸는 것과 같다. 바닥(z=0)으로 풀면 수십 cm 밀린다.
가장 가까운 손 위치(F1~R3, 0.25 m 이내)를 이름으로 붙인다.
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HOST_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST_ROOT))

import host_config  # noqa: E402
from localization.aruco_localizer import (Camera, detect, floor_object_points,  # noqa: E402
                                          make_detector)
from localization.cameras import open_cams, read_frames, release_all  # noqa: E402

# 손 촬영 계획(2026-10-01)의 손 위치 9곳, 손 중심 (m)
HAND_SPOTS = {
    "F1": (0.33, 0.12), "F2": (0.99, 0.12), "F3": (1.65, 0.12),
    "L1": (0.075, 0.26), "L2": (0.075, 0.915), "L3": (0.075, 1.56),
    "R1": (1.905, 0.26), "R2": (1.905, 0.915), "R3": (1.905, 1.56),
}
SPOT_RADIUS_M = 0.25
PALM_IDX = (0, 5, 9, 13, 17)          # 손목 + 검지~새끼 뿌리
MODEL_PATH = HOST_ROOT / "models" / "hand_landmarker.task"
CONNECTIONS = ((0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10),
               (10, 11), (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17),
               (17, 18), (18, 19), (19, 20))


def make_landmarker(num_hands: int, min_conf: float):
    from mediapipe.tasks.python import BaseOptions, vision
    if not MODEL_PATH.exists():
        raise SystemExit(f"{MODEL_PATH} 없음 — 모듈 설명의 주소에서 받아 두십시오")
    opts = vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(MODEL_PATH)),
        num_hands=num_hands, min_hand_detection_confidence=min_conf,
        min_hand_presence_confidence=min_conf)
    return vision.HandLandmarker.create_from_options(opts)


def _run(landmarker, bgr: np.ndarray, ox: int, oy: int) -> list[dict]:
    import mediapipe as mp
    h, w = bgr.shape[:2]
    rgb = np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    res = landmarker.detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb))
    out = []
    for lms, hd in zip(res.hand_landmarks, res.handedness):
        pts = np.array([[p.x * w + ox, p.y * h + oy] for p in lms])
        out.append({"pts": pts, "score": float(hd[0].score) if hd else 0.0,
                    "side": hd[0].category_name if hd else "?"})
    return out


def tiles(w: int, h: int, size: int) -> list[tuple[int, int, int]]:
    """겹침 25 % 정사각 조각 (x, y, size). 화면보다 크면 화면 짧은 변으로 줄인다."""
    size = min(size, w, h)
    step = max(1, int(size * 0.75))

    def starts(n):
        s = list(range(0, max(n - size, 0) + 1, step))
        if s[-1] != n - size:
            s.append(n - size)
        return s
    return [(x, y, size) for y in starts(h) for x in starts(w)]


def palm_px(hand: dict) -> np.ndarray:
    return hand["pts"][list(PALM_IDX)].mean(axis=0)


def merge(hands: list[dict]) -> list[dict]:
    """조각 겹침으로 같은 손이 두 번 잡히면 점수 높은 쪽만 남긴다."""
    kept: list[dict] = []
    for hd in sorted(hands, key=lambda d: -d["score"]):
        size = np.ptp(hd["pts"], axis=0).max()
        if all(np.linalg.norm(palm_px(hd) - palm_px(k)) > 0.5 * size for k in kept):
            kept.append(hd)
    return kept


def detect_hands(landmarker, frame: np.ndarray, mode: str, tile: int) -> list[dict]:
    if mode == "full":
        return _run(landmarker, frame, 0, 0)
    found = []
    for x, y, s in tiles(frame.shape[1], frame.shape[0], tile):
        found += _run(landmarker, frame[y:y + s, x:x + s], x, y)
    return merge(found)


def scaled_camera(cam: Camera, w: int, h: int, cfg) -> Camera:
    """캘리브레이션은 cameras.width×height 기준. 촬영 스크립트(1920×1080)처럼 크기가 다르면
    같은 화각이므로 K 만 비례로 늘린다(왜곡 계수는 정규 좌표라 그대로)."""
    sx, sy = w / cfg.cameras.width, h / cfg.cameras.height
    if abs(sx - 1) < 1e-6 and abs(sy - 1) < 1e-6:
        return cam
    out = Camera(cam.name, cam.K.copy(), cam.dist, cam.aruco_cfg, cam.calibrated)
    out.K[0] *= sx
    out.K[1] *= sy
    return out


def to_map(cam: Camera, px: np.ndarray, z: float):
    if not cam.ready:
        return None
    p = cam.pixels_to_plane(px.reshape(1, 2), z)
    return None if p is None else (float(p[0, 0]), float(p[0, 1]))


def nearest_spot(xy) -> str:
    if xy is None:
        return "-"
    name, (sx, sy) = min(HAND_SPOTS.items(), key=lambda kv: np.hypot(kv[1][0] - xy[0], kv[1][1] - xy[1]))
    return name if np.hypot(sx - xy[0], sy - xy[1]) <= SPOT_RADIUS_M else "?"


def draw(frame: np.ndarray, hands: list[dict], labels: list[str], color) -> None:
    for hd, label in zip(hands, labels):
        pts = hd["pts"].astype(int)
        for a, b in CONNECTIONS:
            cv2.line(frame, tuple(pts[a]), tuple(pts[b]), color, 2)
        c = palm_px(hd).astype(int)
        cv2.circle(frame, tuple(c), 6, color, -1)
        cv2.putText(frame, label, (c[0] + 8, c[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)


def describe(cam: Camera, hands: list[dict], z: float) -> tuple[list[str], list[dict]]:
    labels, rows = [], []
    for hd in hands:
        xy = to_map(cam, palm_px(hd), z)
        spot = nearest_spot(xy)
        txt = f"{spot} {hd['score']:.2f}" + ("" if xy is None else f" ({xy[0]:.2f},{xy[1]:.2f})")
        labels.append(txt)
        rows.append({"spot": spot, "score": round(hd["score"], 3),
                     "x": None if xy is None else round(xy[0], 3),
                     "y": None if xy is None else round(xy[1], 3)})
    return labels, rows


# ---------------------------------------------------------------------------
def run_images(args, cfg, landmarker) -> int:
    root = Path(args.images)
    files = sorted(p for p in root.rglob("*.jpg"))
    if not files:
        print(f"{root} 에 jpg 가 없습니다")
        return 1
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    detector = make_detector(cfg.aruco)
    floor_pts = floor_object_points(cfg.aruco)
    base = {}
    modes = ["full", "tile"] if args.mode == "both" else [args.mode]
    colors = {"full": (0, 165, 255), "tile": (0, 255, 0)}
    rows_out = []
    hit = {m: 0 for m in modes}
    per_spot: dict[str, dict[str, int]] = {}
    for f in files:
        img = cv2.imread(str(f))
        if img is None:
            continue
        cam_name = next((p for p in f.parts[::-1] if p.startswith("cam")), "cam0")
        idx = int(cam_name[3:]) if cam_name[3:].isdigit() else 0
        if idx not in base:
            base[idx] = Camera.load(idx, cfg.cameras, cfg.aruco)
        cam = scaled_camera(base[idx], img.shape[1], img.shape[0], cfg)
        # 사진마다 한 장으로 푼다(카메라는 고정이지만 사진 크기·순서가 섞여 있을 수 있다).
        cam._solve_from(detect(detector, cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)), floor_pts)
        vis = img.copy()
        for m in modes:
            t0 = time.perf_counter()
            hands = detect_hands(landmarker, img, m, int(args.tile * img.shape[1] / cfg.cameras.width))
            ms = (time.perf_counter() - t0) * 1000
            labels, rows = describe(cam, hands, args.hand_z)
            draw(vis, hands, [f"{m}:{t}" for t in labels], colors[m])
            hit[m] += bool(hands)
            for r in rows or [{"spot": "", "score": "", "x": "", "y": ""}]:
                rows_out.append({"file": str(f.relative_to(root)), "mode": m, "hands": len(hands),
                                 "ms": round(ms), **r})
            for r in rows:
                per_spot.setdefault(r["spot"], {k: 0 for k in modes})[m] += 1
        cv2.imwrite(str(out / f"{cam_name}_{f.name}"), vis)
    with open(out / "hand_probe.csv", "w", newline="", encoding="utf-8-sig") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows_out[0].keys()))
        w.writeheader()
        w.writerows(rows_out)
    print(f"\n사진 {len(files)}장 · 결과 {out}")
    for m in modes:
        print(f"  {m:4s}: 손 1개 이상 잡힌 사진 {hit[m]}/{len(files)}")
    print("  위치별 검출 수: " + ", ".join(f"{s}={v}" for s, v in sorted(per_spot.items())))
    return 0


def run_live(args, cfg, landmarker) -> int:
    indices = list(cfg.cameras.indices)
    caps = open_cams(indices, cfg.cameras.width, cfg.cameras.height, cfg.cameras.focus)
    if not any(c.isOpened() for c in caps):
        print("열린 카메라가 없습니다 — run_host 가 떠 있으면 끄십시오")
        return 1
    detector = make_detector(cfg.aruco)
    cams = [Camera.load(i, cfg.cameras, cfg.aruco) for i in indices]
    mode = "tile" if args.mode == "both" else args.mode
    print(f"모드 {mode} · 조각 {args.tile}px · 손 높이 {args.hand_z} m — q 종료, m 으로 full/tile 전환")
    try:
        while True:
            for cam, frame in zip(cams, read_frames(caps)):
                if frame is None:
                    continue
                if not cam.locked:
                    cam.solve_extrinsics(detect(detector, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)))
                t0 = time.perf_counter()
                hands = detect_hands(landmarker, frame, mode, args.tile)
                ms = (time.perf_counter() - t0) * 1000
                labels, _ = describe(cam, hands, args.hand_z)
                draw(frame, hands, labels, (0, 255, 0))
                status = f"{mode} {ms:.0f} ms  hands {len(hands)}  " + \
                         ("extrinsic OK" if cam.ready else "extrinsic ...")
                cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                small = cv2.resize(frame, None, fx=args.view_scale, fy=args.view_scale)
                cv2.imshow(cam.name, small)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("m"):
                mode = "full" if mode == "tile" else "tile"
    finally:
        release_all(caps)
        cv2.destroyAllWindows()
    return 0


def main() -> int:
    host_config.configure_console()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=None)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--live", action="store_true", help="두 카메라 실시간")
    src.add_argument("--images", help="사진 폴더(하위 cam0/cam1 폴더 이름으로 카메라를 고른다)")
    ap.add_argument("--out", default="hand_probe_out", help="--images 결과 폴더")
    ap.add_argument("--mode", choices=("full", "tile", "both"), default="both")
    ap.add_argument("--tile", type=int, default=384, help="조각 한 변(px, 1280×720 기준)")
    ap.add_argument("--hand-z", type=float, default=0.32, help="손 높이(m)")
    ap.add_argument("--num-hands", type=int, default=2)
    ap.add_argument("--min-conf", type=float, default=0.5)
    ap.add_argument("--view-scale", type=float, default=0.6)
    args = ap.parse_args()
    cfg = host_config.load_host_config(args.config)
    landmarker = make_landmarker(args.num_hands, args.min_conf)
    return run_live(args, cfg, landmarker) if args.live else run_images(args, cfg, landmarker)


if __name__ == "__main__":
    raise SystemExit(main())
