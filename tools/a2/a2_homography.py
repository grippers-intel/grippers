#!/usr/bin/env python3
"""A2 · 지면 호모그래피 실측 도구 (grippers / issue #91).

판정 기준
    기지 위치 물체 좌표 오차 <= 20 mm  (issue #91 DoD)
    가림 케이스(로봇 0.3 m / 상자 0.4 m 뒤) 검출률 포함

좌표 규약
    월드 : 바닥 평면. 원점 = 작업 공간의 한 모서리.
           X, Y 는 작업 공간 변을 따라 mm. Z = 0 (바닥).
           **두 카메라가 같은 월드 원점을 쓴다.** 그래야 병합이 성립한다.
    이미지: 물체 지시점은 바운딩박스 아래쪽 모서리 중점(바닥 접지선).
           호모그래피는 바닥 평면만 사상하므로 물체 윗면을 찍으면 높이만큼 밀린다.

카메라 2대 구성
    모든 하위 명령이 ``--cam-id A|B`` 를 받는다. 산출물이 카메라별로 분리되고
    ``report`` 가 양쪽을 합쳐 하나의 이슈 코멘트를 만든다.
    1대만 쓰던 시절의 파일명(a2_homography.json)은 ``--cam-id A`` 로 대체되었다.

사용 순서
    python a2_homography.py devices
    python a2_homography.py lock    --cam 0 --cam-id A
    python a2_homography.py shoot   --cam 0 --cam-id A
    python a2_homography.py calib   --cam-id A --make-template
    python a2_homography.py calib   --cam-id A
    python a2_homography.py verify  --cam-id A --make-template
    python a2_homography.py verify  --cam-id A
    (B 카메라에 대해 lock~verify 반복)
    python a2_homography.py occlude --make-template
    python a2_homography.py occlude
    python a2_homography.py report

CSV 형식 (헤더 필수, 단위 mm)
    base_points_A.csv   : name,world_x,world_y
    verify_points_A.csv : name,world_x,world_y
    occlusion.csv       : case,occluder,distance_mm,trials,detected

의존성: opencv-python, numpy
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import sys
import time
from datetime import datetime

import cv2
import numpy as np

# ---------------------------------------------------------------- 확정 기하
#
# docs/subsystems/perception.md 를 단일 기준으로 삼는다.
# 값이 바뀌면 이 블록만 고치면 report 표까지 따라온다.

WORKSPACE_MM = 1800.0  # 가벽 35 x 45 cm x 16장으로 두른 내부 (8/19 실측)
CAM_HEIGHT_MM = 2050.0  # 보유 스탠드 최대 2.1 m 에서 흔들림 여유를 뺀 값 (#149 D3)
CAM_SETBACK_MM = 600.0  # 마주보는 두 "변 중앙" 에서 바깥으로 후퇴 (#149 D3 정정 8/19)
FOCAL_PX = 1411.0  # f = (1280/2)/tan(24.4°) — C270 720p
CAM_PITCH_DEG = 63.8  # 아래로 내려다보는 각. 스탠드 헤드가 이만큼 꺾이는지 먼저 확인할 것
WORST_SLANT_MM = 2695.0  # 최악점(담당 절반의 경계 모서리) 슬랜트
WORST_ELEV_DEG = 49.5  # 최악점 고도각
MIN_OBJECT_MM = 40.0  # 최악점에서 20 px 을 보장하는 최소 물체 폭 (#149 D1)
GEOMETRY_NOTE = "C270 x2 · 720p · HFOV 48.8° · 마주보는 두 변 중앙 · 높이 2.05 m · 후퇴 0.60 m"

# 담당 구역: A 는 y in [0, 900], B 는 y in [900, 1800]. 경계는 두 대가 함께 본다.
HALF_MM = WORKSPACE_MM / 2

PASS_MM = 20.0  # issue #91 DoD
CAM_IDS = ("A", "B")

# ---------------------------------------------------------------- 산출물 경로


def calib_json(cam_id: str) -> str:
    return f"a2_homography_{cam_id}.json"


def verify_json(cam_id: str) -> str:
    return f"a2_verify_{cam_id}.json"


def lock_json(cam_id: str) -> str:
    return f"a1c_lock_{cam_id}.json"


OCCLUDE_JSON = "a2_occlusion.json"
REPORT_MD = "a2_report.md"


# ---------------------------------------------------------------- 백엔드
#
# OpenCV 5 부터 videoio 백엔드가 플러그인으로 분리되어, 상수(CAP_DSHOW 등)가
# 존재해도 실제 플러그인이 wheel 에 없을 수 있다. 그 경우 나오는 메시지가
#   "backend is generally available but can't be used to capture by index"
# 이다. 그래서 상수를 하드코딩하지 않고 레지스트리를 보고 고른다.


def _backend_candidates() -> list[tuple[str, int]]:
    """이 빌드에서 실제로 쓸 수 있는 videoio 백엔드 목록."""
    out: list[tuple[str, int]] = []
    try:
        for be in cv2.videoio_registry.getCameraBackends():
            out.append((cv2.videoio_registry.getBackendName(be), int(be)))
    except AttributeError:
        # 아주 오래된 빌드 대비 — 플랫폼 기본값만 넣는다.
        if sys.platform.startswith("win"):
            out = [("DSHOW", cv2.CAP_DSHOW), ("MSMF", cv2.CAP_MSMF)]
        else:
            out = [("V4L2", cv2.CAP_V4L2)]
    out.append(("ANY", int(cv2.CAP_ANY)))
    return out


def _resolve_backend(name: str | None) -> int:
    if not name:
        return int(cv2.CAP_ANY)
    for bname, bid in _backend_candidates():
        if bname.upper() == name.upper():
            return bid
    raise SystemExit(f"[!] 백엔드 {name} 를 이 빌드에서 찾을 수 없습니다. devices 를 먼저 보세요.")


def _open(cam: int, width: int, height: int, backend: str | None = None) -> cv2.VideoCapture:
    """카메라를 열고 프레임이 실제로 나오는지까지 확인한다."""
    cap = cv2.VideoCapture(cam, _resolve_backend(backend))
    if not cap.isOpened():
        raise SystemExit(
            f"[!] 카메라 {cam} 를 열 수 없습니다. "
            "`python a2_homography.py devices` 로 쓸 수 있는 조합을 확인하세요."
        )
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    for _ in range(5):
        ok, _frame = cap.read()
        if ok:
            return cap
    cap.release()
    raise SystemExit(
        f"[!] 카메라 {cam} 가 열리기는 했으나 프레임이 나오지 않습니다. "
        "다른 앱이 점유 중이거나 OS 카메라 권한이 막혀 있을 수 있습니다."
    )


def _frame_stats(frame: np.ndarray) -> tuple[float, float]:
    """평균 밝기와 라플라시안 분산.

    속성값만 보면 카메라가 조용히 재노출·재초점하는 것을 못 잡는다.
    값은 고정돼 보이는데 화면이 움직이면 A1-c 는 실패다.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    return float(gray.mean()), float(cv2.Laplacian(gray, cv2.CV_64F).var())


def _read_points_csv(path: str) -> list[dict]:
    if not os.path.exists(path):
        raise SystemExit(f"[!] {path} 가 없습니다. --make-template 로 뼈대를 만드세요.")
    rows: list[dict] = []
    with open(path, newline="", encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            if not r.get("name"):
                continue
            rows.append(
                {
                    "name": r["name"].strip(),
                    "world": (float(r["world_x"]), float(r["world_y"])),
                }
            )
    if not rows:
        raise SystemExit(f"[!] {path} 에 유효한 행이 없습니다.")
    return rows


def _load_json(path: str) -> dict | None:
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as f:
        return json.load(f)


# ---------------------------------------------------------------- 명령: devices


def cmd_devices(a: argparse.Namespace) -> None:
    """이 PC 에서 실제로 프레임이 나오는 (백엔드, 인덱스) 조합을 찾는다."""
    print(f"플랫폼   : {platform.platform()}")
    print(f"Python   : {sys.version.split()[0]}")
    print(f"OpenCV   : {cv2.__version__}\n")

    backends = _backend_candidates()
    print("이 빌드에 등록된 카메라 백엔드")
    for name, bid in backends:
        print(f"  {name} ({bid})")

    # 탐색 중 OpenCV 가 뱉는 경고를 줄인다 (없는 빌드도 있으므로 방어).
    try:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
    except AttributeError:
        pass

    print(f"\n인덱스 0~{a.max_index} 탐색 중 …\n")
    found: list[dict] = []
    for name, bid in backends:
        for idx in range(a.max_index + 1):
            cap = cv2.VideoCapture(idx, bid)
            if not cap.isOpened():
                cap.release()
                continue
            ok, frame = cap.read()
            if ok and frame is not None:
                h, w = frame.shape[:2]
                found.append({"backend": name, "index": idx, "width": w, "height": h})
                print(f"  ✅ {name:<8} index {idx}  →  {w}x{h}")
            cap.release()

    try:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_WARNING)
    except AttributeError:
        pass

    if not found:
        print("  프레임을 주는 조합이 하나도 없습니다.\n")
        print("  1. 웹캠이 실제로 꽂혀 있는지, 다른 앱이 점유 중인지 확인")
        print("  2. Windows: 설정 → 개인 정보 → 카메라 에서 '데스크톱 앱' 항목까지 허용")
        print("  3. Linux:   `ls /dev/video*` 와 video 그룹 권한 확인")
        raise SystemExit(1)

    print(f"\n찾은 조합 {len(found)}개.")
    if len(found) < 2:
        print("  ! A1-d 는 2대 동시 스트리밍이 판정 기준입니다. 아직 1대만 보입니다.")
    best = found[0]
    print("\n권장 실행:")
    print(
        f"  python a2_homography.py lock --cam {best['index']} "
        f"--backend {best['backend']} --cam-id A"
    )


# ---------------------------------------------------------------- 명령: lock

# 자동 노출을 끄는 값은 백엔드마다 다르다. 후보를 차례로 시도해 실제로 먹는 값을 찾는다.
AUTO_EXPOSURE_MANUAL = (0.25, 0.0, 1.0, 3.0)

LOCK_PROPS = [
    ("AUTOFOCUS", cv2.CAP_PROP_AUTOFOCUS, 0.0),
    ("FOCUS", cv2.CAP_PROP_FOCUS, None),
    ("AUTO_WB", cv2.CAP_PROP_AUTO_WB, 0.0),
    ("WB_TEMPERATURE", cv2.CAP_PROP_WB_TEMPERATURE, None),
    ("AUTO_EXPOSURE", cv2.CAP_PROP_AUTO_EXPOSURE, "manual"),
    ("EXPOSURE", cv2.CAP_PROP_EXPOSURE, None),
    ("BRIGHTNESS", cv2.CAP_PROP_BRIGHTNESS, None),
    ("GAIN", cv2.CAP_PROP_GAIN, None),
]

UNSUPPORTED = -1.0  # OpenCV 5 는 미지원 속성에 -1 을 돌려준다 (4.x 는 0)


def _try_lock_prop(cap: cv2.VideoCapture, prop: int, target: float) -> tuple[float, float, bool]:
    before = cap.get(prop)
    cap.set(prop, target)
    time.sleep(0.2)
    for _ in range(5):
        cap.read()
    after = cap.get(prop)
    return before, after, abs(after - target) < 1e-6


def cmd_lock(a: argparse.Namespace) -> None:
    """A1-c 겸용 — AF/AWB/AE 고정이 실제로 먹는지 확인하고 증적을 남긴다."""
    cap = _open(a.cam, a.width, a.height, a.backend)
    for _ in range(10):
        cap.read()

    result: list[dict] = []
    print(f"\n{'속성':<18}{'설정전':>12}{'목표':>12}{'설정후':>12}  판정")
    print("-" * 70)
    for name, prop, target in LOCK_PROPS:
        before = cap.get(prop)

        # 미지원 속성을 '고정 성공' 으로 세지 않는다. 이게 A1-c 거짓 PASS 의 원인이었다.
        if before == UNSUPPORTED:
            print(f"{name:<18}{before:>12.3f}{'-':>12}{'-':>12}  미지원")
            result.append({"prop": name, "supported": False, "locked": False})
            continue

        if target == "manual":
            chosen, ok = None, False
            for cand in AUTO_EXPOSURE_MANUAL:
                _b, after, ok = _try_lock_prop(cap, prop, cand)
                if ok:
                    chosen = cand
                    break
            target_val = chosen if chosen is not None else float("nan")
            after_val = cap.get(prop)
        else:
            target_val = before if target is None else float(target)
            _b, after_val, ok = _try_lock_prop(cap, prop, target_val)

        verdict = "OK" if ok else "FAIL"
        print(f"{name:<18}{before:>12.3f}{target_val:>12.3f}{after_val:>12.3f}  {verdict}")
        result.append(
            {
                "prop": name,
                "supported": True,
                "before": before,
                "target": target_val,
                "after": after_val,
                "locked": bool(ok),
            }
        )

    # ------------------------------------------------------------ 드리프트
    #
    # 속성값이 그대로여도 화면이 밝아지거나 초점이 흔들리면 고정이 아니다.
    # 그래서 값과 영상 통계를 함께 본다.
    print(f"\n[{a.seconds:.0f}초 드리프트 확인] 화면을 건드리지 말고 기다리세요.")
    t0 = time.time()
    samples: list[dict] = []
    while time.time() - t0 < a.seconds:
        ok, frame = cap.read()
        if not ok:
            continue
        bright, lapvar = _frame_stats(frame)
        samples.append(
            {
                "t": round(time.time() - t0, 1),
                "brightness": bright,
                "lapvar": lapvar,
                "props": {p[0]: cap.get(p[1]) for p in LOCK_PROPS},
            }
        )
        time.sleep(1.0)
    cap.release()

    if not samples:
        raise SystemExit("[!] 드리프트 구간에서 프레임을 하나도 못 받았습니다.")

    prop_stable = True
    for name, _prop, _t in LOCK_PROPS:
        vals = {round(s["props"][name], 4) for s in samples}
        if len(vals) > 1:
            prop_stable = False
            print(f"  ! {name} 값이 흔들립니다: {sorted(vals)}")

    bs = [s["brightness"] for s in samples]
    ls = [s["lapvar"] for s in samples]
    bright_range = max(bs) - min(bs)
    lap_rel = (max(ls) - min(ls)) / max(ls) if max(ls) > 0 else 0.0
    image_stable = bright_range <= a.brightness_tol and lap_rel <= a.focus_tol

    print(f"\n  평균 밝기 변동  {bright_range:6.2f} / 255   (허용 {a.brightness_tol})")
    print(f"  선명도 변동     {lap_rel * 100:6.1f} %         (허용 {a.focus_tol * 100:.0f} %)")
    if not image_stable:
        print("  ! 속성값과 무관하게 영상이 변했습니다 — 카메라가 자동 보정 중입니다.")

    stable = prop_stable and image_stable
    print(f"\nA1-c 판정: {'PASS' if stable else 'FAIL'}")
    if not stable:
        print("  고정초점·수동노출을 지원하는 모델로 교체하거나, OS 카메라 설정에서")
        print("  자동 보정을 끄고 다시 시도하세요. Linux 는 v4l2-ctl 로 직접 끌 수 있습니다.")

    path = lock_json(a.cam_id)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(
            {
                "cam_id": a.cam_id,
                "camera_index": a.cam,
                "backend": a.backend or "ANY",
                "at": datetime.now().isoformat(timespec="seconds"),
                "props": result,
                "seconds": a.seconds,
                "brightness_range": bright_range,
                "focus_rel_range": lap_rel,
                "prop_stable": prop_stable,
                "image_stable": image_stable,
                "stable": stable,
            },
            f,
            ensure_ascii=False,
            indent=2,
        )
    print(f"\n[+] {path} 저장 (A1-c 증적)")


# ---------------------------------------------------------------- 클릭 수집


class Picker:
    """이미지에서 점을 순서대로 클릭받는다. 확대 미리보기 포함."""

    def __init__(self, image: np.ndarray, labels: list[str], zoom: int = 6):
        self.img = image
        self.labels = labels
        self.zoom = zoom
        self.picked: list[tuple[float, float]] = []
        self.cursor = (0, 0)
        self.win = "A2 point picker"

    def _draw(self) -> np.ndarray:
        vis = self.img.copy()
        for i, (x, y) in enumerate(self.picked):
            p = (int(round(x)), int(round(y)))
            cv2.drawMarker(vis, p, (0, 255, 0), cv2.MARKER_CROSS, 22, 2)
            cv2.putText(
                vis,
                self.labels[i],
                (p[0] + 10, p[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

        idx = len(self.picked)
        msg = (
            f"[{idx + 1}/{len(self.labels)}] click: {self.labels[idx]}"
            if idx < len(self.labels)
            else "done - press ENTER"
        )
        cv2.rectangle(vis, (0, 0), (vis.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(
            vis,
            msg + "   (u=undo, ESC=abort)",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

        cx, cy = self.cursor
        h, w = self.img.shape[:2]
        r = 30
        x0, y0 = max(0, cx - r), max(0, cy - r)
        x1, y1 = min(w, cx + r), min(h, cy + r)
        if x1 > x0 and y1 > y0:
            crop = cv2.resize(
                self.img[y0:y1, x0:x1],
                None,
                fx=self.zoom,
                fy=self.zoom,
                interpolation=cv2.INTER_NEAREST,
            )
            ch, cw = crop.shape[:2]
            cv2.drawMarker(crop, (cw // 2, ch // 2), (0, 0, 255), cv2.MARKER_CROSS, 40, 1)
            ph, pw = min(ch, h), min(cw, w)
            vis[h - ph : h, w - pw : w] = crop[:ph, :pw]
            cv2.rectangle(vis, (w - pw, h - ph), (w - 1, h - 1), (0, 0, 255), 2)
        return vis

    def _on_mouse(self, event, x, y, flags, _):  # noqa: ARG002
        self.cursor = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN and len(self.picked) < len(self.labels):
            self.picked.append((float(x), float(y)))

    def run(self) -> list[tuple[float, float]]:
        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.win, 1280, 720)
        cv2.setMouseCallback(self.win, self._on_mouse)
        while True:
            cv2.imshow(self.win, self._draw())
            k = cv2.waitKey(20) & 0xFF
            if k == 27:
                cv2.destroyWindow(self.win)
                raise SystemExit("[!] 사용자가 중단했습니다.")
            if k in (ord("u"), ord("U")) and self.picked:
                self.picked.pop()
            if k in (13, 10) and len(self.picked) == len(self.labels):
                break
        cv2.destroyWindow(self.win)
        return self.picked


# ---------------------------------------------------------------- 명령: shoot


def cmd_shoot(a: argparse.Namespace) -> None:
    cap = _open(a.cam, a.width, a.height, a.backend)
    out = a.out or f"frame_base_{a.cam_id}.png"
    print(f"[ SPACE = 저장 / ESC = 취소 ]  카메라 {a.cam_id} 가 고정되어 있는지 확인하세요.")
    win = f"A2 shoot [{a.cam_id}]"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)
    saved = False
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        cv2.imshow(win, frame)
        k = cv2.waitKey(20) & 0xFF
        if k == 27:
            break
        if k == 32:
            cv2.imwrite(out, frame)
            h, w = frame.shape[:2]
            print(f"[+] {out} 저장 ({w}x{h})")
            saved = True
            break
    cap.release()
    cv2.destroyAllWindows()
    if not saved:
        raise SystemExit("[!] 저장하지 않고 종료했습니다.")


# ---------------------------------------------------------------- 명령: calib


def cmd_calib(a: argparse.Namespace) -> None:
    points = a.points or f"base_points_{a.cam_id}.csv"
    image = a.image or f"frame_base_{a.cam_id}.png"

    if a.make_template:
        # 기지점은 "자기 담당 절반의 네 귀퉁이" 다 — 가까운 변 두 점 + 경계 두 점.
        # 네 모서리를 다 쓰려고 하지 마라. 변 중앙 배치에서 반대편 절반은 프레임 밖이다.
        # 두 시야를 묶는 것은 점을 공유하는 게 아니라 같은 줄자 원점·같은 X 축을 쓰는 것이다.
        near_y = 0 if a.cam_id == "A" else WORKSPACE_MM
        with open(points, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["name", "world_x", "world_y"])
            w.writerow(["near_x0", 0, near_y])
            w.writerow(["near_x1", WORKSPACE_MM, near_y])
            w.writerow(["boundary_x1", WORKSPACE_MM, HALF_MM])
            w.writerow(["boundary_x0", 0, HALF_MM])
        print(f"[+] {points} 템플릿 생성. 실측값(mm)으로 고쳐서 다시 실행하세요.")
        print(f"    {a.cam_id} 담당 절반의 네 귀퉁이입니다 (가까운 변 2점 + 경계 2점).")
        print("    A · B 카메라가 같은 원점·같은 축을 써야 병합이 됩니다.")
        return

    img = cv2.imread(image)
    if img is None:
        raise SystemExit(f"[!] {image} 를 읽을 수 없습니다.")
    pts = _read_points_csv(points)
    if len(pts) < 4:
        raise SystemExit("[!] 기지점이 4개 미만입니다.")

    print(f"\n[{a.cam_id}] 각 점의 바닥 접지 위치를 클릭하세요.")
    img_pts = Picker(img, [p["name"] for p in pts]).run()

    src = np.array(img_pts, dtype=np.float64)
    dst = np.array([p["world"] for p in pts], dtype=np.float64)
    if len(pts) > 4:
        homography, _mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
    else:
        homography = cv2.getPerspectiveTransform(src.astype(np.float32), dst.astype(np.float32))
    if homography is None:
        raise SystemExit("[!] 호모그래피를 구하지 못했습니다. 네 점이 공선에 가깝지 않은지 보세요.")

    proj = cv2.perspectiveTransform(src.reshape(-1, 1, 2), homography).reshape(-1, 2)
    res = np.linalg.norm(proj - dst, axis=1)
    print("\n기지점 잔차 (mm)")
    for p, r in zip(pts, res, strict=True):
        print(f"  {p['name']:<14}{r:8.2f}")
    rms = float(np.sqrt((res**2).mean()))
    print(f"  {'RMS':<14}{rms:8.2f}")

    path = calib_json(a.cam_id)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(
            {
                "cam_id": a.cam_id,
                "at": datetime.now().isoformat(timespec="seconds"),
                "image": os.path.abspath(image),
                "image_size": [img.shape[1], img.shape[0]],
                "camera_height_mm": CAM_HEIGHT_MM,
                "workspace_mm": WORKSPACE_MM,
                "H": homography.tolist(),
                "base_points": [
                    {
                        "name": p["name"],
                        "image": list(ip),
                        "world": list(p["world"]),
                        "residual_mm": float(r),
                    }
                    for p, ip, r in zip(pts, img_pts, res, strict=True)
                ],
                "base_rms_mm": rms,
            },
            f,
            ensure_ascii=False,
            indent=2,
        )
    print(f"\n[+] {path} 저장")


# ---------------------------------------------------------------- 명령: verify

# 카메라별 담당 절반 + 경계. 최악점은 담당 절반의 경계 모서리다.
# 검증점은 담당 절반 안에서 고르되 경계 모서리를 반드시 포함한다 — 거기가 최악점이다.
# A 는 y in [0, 900], B 는 y in [900, 1800]. B 는 A 를 y 축으로 뒤집은 것이다.
VERIFY_TEMPLATE = {
    "A": [
        ("boundary_x0", 0, 900),  # 최악점
        ("boundary_x1", 1800, 900),  # 최악점
        ("boundary_mid", 900, 900),
        ("near_x0", 0, 0),
        ("near_x1", 1800, 0),
        ("near_mid", 900, 0),
        ("q_left", 300, 450),
        ("q_right", 1500, 450),
        ("center", 900, 450),
        ("off_axis", 1500, 750),
    ],
    "B": [
        ("boundary_x0", 0, 900),  # 최악점
        ("boundary_x1", 1800, 900),  # 최악점
        ("boundary_mid", 900, 900),
        ("near_x0", 0, 1800),
        ("near_x1", 1800, 1800),
        ("near_mid", 900, 1800),
        ("q_left", 300, 1350),
        ("q_right", 1500, 1350),
        ("center", 900, 1350),
        ("off_axis", 1500, 1050),
    ],
}


def cmd_verify(a: argparse.Namespace) -> None:
    points = a.points or f"verify_points_{a.cam_id}.csv"
    image = a.image or f"frame_base_{a.cam_id}.png"

    if a.make_template:
        with open(points, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["name", "world_x", "world_y"])
            for n, x, y in VERIFY_TEMPLATE[a.cam_id]:
                w.writerow([n, x, y])
        print(f"[+] {points} 템플릿 생성 ({len(VERIFY_TEMPLATE[a.cam_id])}점).")
        print(f"    {a.cam_id} 담당 절반 + 경계입니다. 실제 배치 좌표(mm)로 고치세요.")
        print(f"    경계 모서리를 반드시 포함하세요 — 고도각 {WORST_ELEV_DEG}° 로 최악 조건입니다.")
        return

    calib = _load_json(calib_json(a.cam_id))
    if calib is None:
        raise SystemExit(f"[!] {calib_json(a.cam_id)} 이 없습니다. 먼저 calib 을 실행하세요.")
    homography = np.array(calib["H"], dtype=np.float64)

    img = cv2.imread(image)
    if img is None:
        raise SystemExit(f"[!] {image} 를 읽을 수 없습니다.")
    pts = _read_points_csv(points)

    print(f"\n[{a.cam_id}] 검증점의 바닥 접지 위치를 클릭하세요.")
    img_pts = Picker(img, [p["name"] for p in pts]).run()

    src = np.array(img_pts, dtype=np.float64).reshape(-1, 1, 2)
    est = cv2.perspectiveTransform(src, homography).reshape(-1, 2)
    truth = np.array([p["world"] for p in pts], dtype=np.float64)
    dist = np.linalg.norm(est - truth, axis=1)

    rows = []
    print(f"\n{'점':<16}{'실측 X':>9}{'실측 Y':>9}{'추정 X':>9}{'추정 Y':>9}{'오차':>9}   판정")
    print("-" * 74)
    for p, t, e, d in zip(pts, truth, est, dist, strict=True):
        verdict = "PASS" if d <= PASS_MM else "FAIL"
        print(f"{p['name']:<16}{t[0]:9.0f}{t[1]:9.0f}{e[0]:9.1f}{e[1]:9.1f}{d:9.2f}   {verdict}")
        rows.append(
            {
                "name": p["name"],
                "truth": t.tolist(),
                "estimate": e.tolist(),
                "error_mm": float(d),
                "pass": bool(d <= PASS_MM),
            }
        )

    mean_mm = float(dist.mean())
    max_mm = float(dist.max())
    p95_mm = float(np.percentile(dist, 95))
    n_pass = int((dist <= PASS_MM).sum())
    gate = max_mm <= PASS_MM

    print("-" * 74)
    print(f"평균 {mean_mm:.2f} mm · p95 {p95_mm:.2f} mm · 최대 {max_mm:.2f} mm")
    print(f"통과 {n_pass}/{len(dist)} 점")
    print(
        f"\n[{a.cam_id}] 판정: {'PASS' if gate else 'FAIL'}  (기준 최대 오차 <= {PASS_MM:.0f} mm)"
    )

    vis = img.copy()
    for ip, d in zip(img_pts, dist, strict=True):
        c = (0, 200, 0) if d <= PASS_MM else (0, 0, 255)
        p = (int(round(ip[0])), int(round(ip[1])))
        cv2.drawMarker(vis, p, c, cv2.MARKER_CROSS, 22, 2)
        cv2.putText(vis, f"{d:.0f}", (p[0] + 10, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)
    overlay = f"a2_verify_overlay_{a.cam_id}.png"
    cv2.imwrite(overlay, vis)

    path = verify_json(a.cam_id)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(
            {
                "cam_id": a.cam_id,
                "at": datetime.now().isoformat(timespec="seconds"),
                "image": os.path.abspath(image),
                "threshold_mm": PASS_MM,
                "points": rows,
                "mean_mm": mean_mm,
                "p95_mm": p95_mm,
                "max_mm": max_mm,
                "passed": n_pass,
                "total": len(rows),
                "gate_pass": gate,
            },
            f,
            ensure_ascii=False,
            indent=2,
        )
    print(f"\n[+] {path} · {overlay} 저장")


# ---------------------------------------------------------------- 명령: occlude


def cmd_occlude(a: argparse.Namespace) -> None:
    """#91 DoD 의 가림 케이스 집계. 검출 시행 결과를 CSV 로 받아 검출률만 낸다."""
    if a.make_template:
        with open(a.trials, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["case", "occluder", "distance_mm", "trials", "detected"])
            w.writerow(["baseline", "none", 0, 20, ""])
            w.writerow(["robot_behind", "robot", 300, 20, ""])
            w.writerow(["box_behind", "box", 400, 20, ""])
            w.writerow(["both_cams", "robot", 300, 20, ""])
        print(f"[+] {a.trials} 템플릿 생성. detected 열에 검출 성공 횟수를 채우세요.")
        print("    both_cams 는 A·B 중 한쪽만 가려도 다른 쪽이 잡는지 보는 항목입니다.")
        return

    if not os.path.exists(a.trials):
        raise SystemExit(f"[!] {a.trials} 가 없습니다. --make-template 를 먼저 쓰세요.")
    rows = []
    with open(a.trials, newline="", encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            if not r.get("case") or not r.get("detected"):
                continue
            trials, detected = int(r["trials"]), int(r["detected"])
            rows.append(
                {
                    "case": r["case"],
                    "occluder": r["occluder"],
                    "distance_mm": float(r["distance_mm"]),
                    "trials": trials,
                    "detected": detected,
                    "rate": detected / trials if trials else 0.0,
                }
            )
    if not rows:
        raise SystemExit("[!] detected 열이 비어 있습니다.")

    print(f"\n{'케이스':<16}{'가림물':<10}{'거리(mm)':>10}{'시행':>7}{'검출':>7}{'검출률':>9}")
    print("-" * 60)
    for r in rows:
        print(
            f"{r['case']:<16}{r['occluder']:<10}{r['distance_mm']:>10.0f}"
            f"{r['trials']:>7}{r['detected']:>7}{r['rate'] * 100:>8.1f}%"
        )

    with open(OCCLUDE_JSON, "w", encoding="utf-8") as f:
        json.dump(
            {"at": datetime.now().isoformat(timespec="seconds"), "rows": rows},
            f,
            ensure_ascii=False,
            indent=2,
        )
    print(f"\n[+] {OCCLUDE_JSON} 저장")


# ---------------------------------------------------------------- 명령: report


def _report_camera_section(add, cam_id: str) -> bool | None:
    """카메라 한 대분 섹션. 측정 결과가 없으면 None 을 돌려준다."""
    v = _load_json(verify_json(cam_id))
    if v is None:
        return None
    calib = _load_json(calib_json(cam_id)) or {}

    add(f"### 카메라 {cam_id} — {'PASS' if v['gate_pass'] else 'FAIL'}")
    add("")
    if calib.get("image_size"):
        add(f"해상도 {calib['image_size'][0]} × {calib['image_size'][1]} · 측정일 `{v['at']}`")
    if calib.get("base_rms_mm") is not None:
        add(f"기지점 RMS 잔차 **{calib['base_rms_mm']:.2f} mm**")
    add("")
    add("| 점 | 실측 X (mm) | 실측 Y (mm) | 추정 X | 추정 Y | 오차 (mm) | 판정 |")
    add("|---|---:|---:|---:|---:|---:|---|")
    for p in v["points"]:
        add(
            f"| `{p['name']}` | {p['truth'][0]:.0f} | {p['truth'][1]:.0f} | "
            f"{p['estimate'][0]:.1f} | {p['estimate'][1]:.1f} | "
            f"**{p['error_mm']:.2f}** | {'✅' if p['pass'] else '❌'} |"
        )
    add("")
    add(
        f"평균 **{v['mean_mm']:.2f} mm** · p95 **{v['p95_mm']:.2f} mm** · "
        f"최대 **{v['max_mm']:.2f} mm** · 통과 {v['passed']}/{v['total']} 점"
    )
    add("")
    return bool(v["gate_pass"])


def cmd_report(_: argparse.Namespace) -> None:
    results = {cid: _load_json(verify_json(cid)) for cid in CAM_IDS}
    if not any(results.values()):
        raise SystemExit("[!] verify 결과가 하나도 없습니다. verify 를 먼저 실행하세요.")

    measured = [cid for cid in CAM_IDS if results[cid]]
    gate = all(results[cid]["gate_pass"] for cid in measured) and len(measured) == len(CAM_IDS)

    lines: list[str] = []
    add = lines.append

    add(f"## A2 · 지면 호모그래피 실측 결과 — **{'PASS' if gate else 'FAIL'}**")
    add("")
    add(f"판정 기준 최대 오차 ≤ {PASS_MM:.0f} mm · 측정 카메라 {', '.join(measured)}")
    if len(measured) < len(CAM_IDS):
        missing = [c for c in CAM_IDS if c not in measured]
        add("")
        add(f"> ⚠️ 카메라 {', '.join(missing)} 는 아직 측정하지 않았습니다. **A2 미완료**입니다.")
    add("")
    add("### 조건")
    add("")
    add("| 항목 | 값 |")
    add("|---|---|")
    add(f"| 작업 공간 | {WORKSPACE_MM / 1000:.1f} × {WORKSPACE_MM / 1000:.1f} m |")
    add(f"| 카메라 | 환경 고정 · 높이 {CAM_HEIGHT_MM / 1000:.2f} m |")
    add(f"| 배치 | {GEOMETRY_NOTE} |")
    add(f"| 변 중앙 후퇴 | {CAM_SETBACK_MM / 1000:.2f} m · 하향 {CAM_PITCH_DEG}° |")
    add(f"| 핀홀 초점거리 | {FOCAL_PX:.0f} px |")
    add(f"| 최악점 | 슬랜트 {WORST_SLANT_MM / 1000:.2f} m · 고도각 {WORST_ELEV_DEG}° |")
    add(f"| 최소 물체 폭 | {MIN_OBJECT_MM:.0f} mm (최악점 20 px 보장) |")
    add("| 지시점 규약 | 바운딩박스 아래쪽 모서리(바닥 접지선) |")
    add("")

    for cid in CAM_IDS:
        _report_camera_section(add, cid)

    occ = _load_json(OCCLUDE_JSON)
    if occ:
        add("### 가림 케이스 검출률")
        add("")
        add("| 케이스 | 가림물 | 거리 (mm) | 시행 | 검출 | 검출률 |")
        add("|---|---|---:|---:|---:|---:|")
        for r in occ["rows"]:
            add(
                f"| {r['case']} | {r['occluder']} | {r['distance_mm']:.0f} | "
                f"{r['trials']} | {r['detected']} | **{r['rate'] * 100:.1f}%** |"
            )
        add("")
        add(
            f"> 최악점 고도각 {WORST_ELEV_DEG}° 로 경계 모서리가 최악 조건입니다. "
            "두 카메라가 마주 보므로 한쪽이 가려도 반대쪽이 잡는 것이 이 배치의 이점입니다."
        )
        add("")

    locks = {cid: _load_json(lock_json(cid)) for cid in CAM_IDS}
    if any(locks.values()):
        add("### A1-c · 웹캠 파라미터 고정 (동시 확인)")
        add("")
        add("| 카메라 | 속성 고정 | 영상 안정 | 종합 |")
        add("|---|---|---|---|")
        for cid in CAM_IDS:
            lk = locks[cid]
            if not lk:
                continue
            add(
                f"| {cid} | {'✅' if lk['prop_stable'] else '❌'} | "
                f"{'✅' if lk['image_stable'] else '❌'} | "
                f"{'✅ PASS' if lk['stable'] else '❌ FAIL'} |"
            )
        add("")
        add(
            "> 속성값뿐 아니라 평균 밝기·선명도 변동까지 함께 봅니다. "
            "미지원 속성은 고정 성공으로 세지 않습니다."
        )
        add("")

    add("### 판정")
    add("")
    if gate:
        worst = max(results[cid]["max_mm"] for cid in measured)
        add(
            f"두 카메라 모두 최대 오차 {worst:.2f} mm 로 기준 {PASS_MM:.0f} mm 를 만족합니다. "
            "**깊이 센서 없이 웹캠 2대 + 지면 호모그래피만으로 위치 추정이 성립**함이 "
            "실측으로 확인되었습니다. M2 의 위치 추정 경로를 이 구성으로 확정합니다."
        )
    else:
        add("기준을 만족하지 못했습니다. 대체안 논의가 필요합니다.")
        add("")
        add("- 기지점을 작업 공간 바깥쪽으로 더 벌려 조건수 개선")
        add("- 렌즈 왜곡 보정(`cv2.undistort`)을 호모그래피 앞단에 삽입")
        add("- 접지선 클릭/검출 정밀도 — 바운딩박스 하단이 실제 접지선과 어긋나는지 확인")
        add("- 카메라 고정부 흔들림 — 고정이 깨지면 호모그래피 전제 자체가 무너짐")
    add("")

    with open(REPORT_MD, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"[+] {REPORT_MD} 저장")
    print(f'    gh issue comment 91 --body-file "{os.path.abspath(REPORT_MD)}"')


# ---------------------------------------------------------------- CLI


def _add_camera_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--cam", type=int, default=0, help="카메라 인덱스 (devices 로 확인)")
    p.add_argument("--backend", default=None, help="videoio 백엔드 이름 (devices 로 확인)")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)


def _add_cam_id(p: argparse.ArgumentParser) -> None:
    p.add_argument("--cam-id", choices=CAM_IDS, default="A", help="산출물 구분자")


def main() -> None:
    ap = argparse.ArgumentParser(description="A2 지면 호모그래피 실측 (#91)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("devices", help="쓸 수 있는 백엔드·인덱스 탐색")
    p.add_argument("--max-index", type=int, default=5)
    p.set_defaults(func=cmd_devices)

    p = sub.add_parser("lock", help="AF/AWB/AE 고정 확인 (A1-c 겸용)")
    _add_camera_args(p)
    _add_cam_id(p)
    p.add_argument("--seconds", type=float, default=30.0)
    p.add_argument("--brightness-tol", type=float, default=3.0, help="허용 평균 밝기 변동")
    p.add_argument("--focus-tol", type=float, default=0.15, help="허용 선명도 상대 변동")
    p.set_defaults(func=cmd_lock)

    p = sub.add_parser("shoot", help="기준 프레임 촬영")
    _add_camera_args(p)
    _add_cam_id(p)
    p.add_argument("--out", default=None)
    p.set_defaults(func=cmd_shoot)

    p = sub.add_parser("calib", help="기지점 4개로 호모그래피 계산")
    _add_cam_id(p)
    p.add_argument("--image", default=None)
    p.add_argument("--points", default=None)
    p.add_argument("--make-template", action="store_true")
    p.set_defaults(func=cmd_calib)

    p = sub.add_parser("verify", help="검증점으로 오차 판정")
    _add_cam_id(p)
    p.add_argument("--image", default=None)
    p.add_argument("--points", default=None)
    p.add_argument("--make-template", action="store_true")
    p.set_defaults(func=cmd_verify)

    p = sub.add_parser("occlude", help="가림 케이스 검출률 집계")
    p.add_argument("--trials", default="occlusion.csv")
    p.add_argument("--make-template", action="store_true")
    p.set_defaults(func=cmd_occlude)

    p = sub.add_parser("report", help="#91 에 붙일 마크다운 생성 (A·B 병합)")
    p.set_defaults(func=cmd_report)

    a = ap.parse_args()
    a.func(a)


if __name__ == "__main__":
    main()
