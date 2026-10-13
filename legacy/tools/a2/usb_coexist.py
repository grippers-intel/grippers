#!/usr/bin/env python3
"""
USB 카메라 공존 부하 시험 — issue #90 (A1-d) · #87 (depth cam 복귀 판정)

무엇을 판별하는가
    웹캠 2대(모서리 1.6 m + 로봇 탑재)와 depth cam 을 Pi 5 한 대에 동시에 물렸을 때
    끊김 없이 스트리밍이 유지되는지. **대역폭이 아니라 전력에서 먼저 무너지는 경우가 많습니다.**

왜 MJPG 를 강제하는가
    1080p30 을 YUYV(무압축)로 열면 카메라 한 대가 약 1.5 Gbps 를 먹습니다. 두 대면 3 Gbps 라
    USB 3.0 한 포트를 사실상 다 씁니다. MJPG 로 열면 10분의 1 수준으로 떨어집니다.
    **드라이버가 조용히 YUYV 로 되돌리는 일이 흔하므로 실제 적용 여부를 매번 확인합니다.**

Pi 5 전력 주의
    공식 5 V/5 A (27 W) 어댑터일 때만 USB 주변장치에 1.6 A 를 줍니다.
    그 외 어댑터에서는 **600 mA 로 제한**됩니다. 웹캠 2대 + depth cam 이면 넘길 수 있고,
    이때 증상은 "대역폭 부족"이 아니라 **장치가 재열거되며 스트림이 끊기는 것**입니다.

사용법
    python3 usb_coexist.py probe
    python3 usb_coexist.py run --cams 0,2 --seconds 60
    python3 usb_coexist.py run --cams 0,2,4 --seconds 60 --label "webcam2 + depth"
    python3 usb_coexist.py report

의존성: opencv-python, numpy  (probe 는 v4l2-ctl 이 있으면 더 자세히 나옵니다)
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import threading
import time
from datetime import datetime

import cv2
import numpy as np

RESULT_JSON = "a1d_coexist.json"
REPORT_MD = "a1d_report.md"

# 판정 기준 (#90 DoD: 30초 무중단)
MIN_SECONDS = 30.0
MAX_GAP_MS = 500.0  # 프레임 간격이 이보다 벌어지면 "끊김" 1회
FPS_TOLERANCE = 0.80  # 목표 fps 의 80% 미만이면 실패


# ---------------------------------------------------------------- 백엔드 호환
#
# Linux 는 V4L2, Windows 는 DSHOW/MSMF 다. OpenCV 5 부터는 백엔드가 플러그인이라
# 상수가 있어도 실제로 못 쓸 수 있으므로 레지스트리를 보고 고른다.


def _resolve_backend(name: str | None) -> int:
    if name:
        try:
            for be in cv2.videoio_registry.getCameraBackends():
                if cv2.videoio_registry.getBackendName(be).upper() == name.upper():
                    return int(be)
        except AttributeError:
            pass
        raise SystemExit(f"[!] 백엔드 {name} 를 이 빌드에서 찾을 수 없습니다.")
    if os.name == "nt":
        return int(cv2.CAP_ANY)
    return int(cv2.CAP_V4L2)


def _fourcc(code: str) -> int:
    """OpenCV 5 는 VideoWriter_fourcc 를 VideoWriter.fourcc 로 옮겼다."""
    fn = getattr(cv2, "VideoWriter_fourcc", None) or cv2.VideoWriter.fourcc
    return int(fn(*code))


# ---------------------------------------------------------------- 환경 점검


def _sh(cmd: list[str]) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=10).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def check_power() -> dict:
    """Pi 5 전력·스로틀 상태. 여기서 걸리면 스트리밍 시험은 의미가 없습니다."""
    info: dict = {}

    throttled = _sh(["vcgencmd", "get_throttled"])
    if throttled:
        info["throttled_raw"] = throttled
        try:
            bits = int(throttled.split("=")[1], 16)
            info["undervoltage_now"] = bool(bits & 0x1)
            info["undervoltage_since_boot"] = bool(bits & 0x10000)
        except (IndexError, ValueError):
            pass

    # 1.6A 로 풀렸는지 (공식 5A 어댑터일 때만 1)
    for path in (
        "/sys/firmware/devicetree/base/chosen/power/max_current_enable",
        "/proc/device-tree/chosen/power/usb_max_current_enable",
    ):
        if os.path.exists(path):
            with open(path, "rb") as f:
                info["usb_max_current_enable"] = int.from_bytes(f.read(4), "big")
            break

    return info


def cmd_probe(_: argparse.Namespace) -> None:
    print("=== 전력 / 스로틀 ===")
    p = check_power()
    if not p:
        print("  (vcgencmd 없음 — Pi 가 아니거나 경로가 다릅니다)")
    for k, v in p.items():
        print(f"  {k}: {v}")
    if p.get("usb_max_current_enable") == 0:
        print("  ! USB 주변장치가 600 mA 로 제한된 상태입니다.")
        print(
            "    공식 5 V/5 A (27 W) 어댑터로 바꾸세요. 안 그러면 3대 동시는 전력에서 떨어집니다."
        )
    if p.get("undervoltage_since_boot"):
        print("  ! 부팅 이후 저전압이 한 번이라도 있었습니다. 어댑터를 먼저 해결하세요.")

    print("\n=== USB 장치 ===")
    print(_sh(["lsusb"]) or "  (lsusb 없음)")

    print("\n=== V4L2 장치 ===")
    if shutil.which("v4l2-ctl"):
        print(_sh(["v4l2-ctl", "--list-devices"]))
        for idx in range(10):
            dev = f"/dev/video{idx}"
            if not os.path.exists(dev):
                continue
            out = _sh(["v4l2-ctl", "-d", dev, "--list-formats-ext"])
            if "MJPG" in out or "YUYV" in out:
                has_mjpg = "MJPG" in out
                print(f"  {dev}: MJPG {'있음' if has_mjpg else '없음 🔴'}")
    else:
        print("  v4l2-ctl 없음 → sudo apt install v4l-utils 권장")
        for idx in range(10):
            if os.path.exists(f"/dev/video{idx}"):
                print(f"  /dev/video{idx} 존재")


# ---------------------------------------------------------------- 스트리밍 시험


class CamWorker(threading.Thread):
    def __init__(self, idx: int, a: argparse.Namespace, stop: threading.Event):
        super().__init__(daemon=True)
        self.idx = idx
        self.a = a
        self.stop = stop
        self.stamps: list[float] = []
        self.read_fail = 0
        self.opened = False
        self.actual: dict = {}
        self.error = ""

    def run(self) -> None:
        cap = cv2.VideoCapture(self.idx, _resolve_backend(self.a.backend))
        if not cap.isOpened():
            self.error = "열기 실패"
            return
        # 순서 중요: FOURCC 를 먼저 잡아야 해상도가 MJPG 기준으로 협상됩니다.
        cap.set(cv2.CAP_PROP_FOURCC, _fourcc(self.a.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.a.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.a.height)
        cap.set(cv2.CAP_PROP_FPS, self.a.fps)

        raw = int(cap.get(cv2.CAP_PROP_FOURCC))
        self.actual = {
            "fourcc": "".join(chr((raw >> (8 * i)) & 0xFF) for i in range(4)),
            "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": cap.get(cv2.CAP_PROP_FPS),
        }
        self.opened = True

        for _ in range(5):  # 워밍업 프레임은 통계에서 뺍니다
            cap.read()
        while not self.stop.is_set():
            ok, frame = cap.read()
            if ok and frame is not None:
                self.stamps.append(time.perf_counter())
            else:
                self.read_fail += 1
                time.sleep(0.005)
        cap.release()

    def stats(self) -> dict:
        s = {
            "camera": self.idx,
            "opened": self.opened,
            "error": self.error,
            "requested": {
                "fourcc": self.a.fourcc,
                "width": self.a.width,
                "height": self.a.height,
                "fps": self.a.fps,
            },
            "actual": self.actual,
            "frames": len(self.stamps),
            "read_fail": self.read_fail,
        }
        if len(self.stamps) < 2:
            s.update({"fps_measured": 0.0, "gap_p95_ms": None, "gap_max_ms": None, "stalls": None})
            return s
        gaps = np.diff(np.array(self.stamps)) * 1000.0
        dur = self.stamps[-1] - self.stamps[0]
        s.update(
            {
                "duration_s": dur,
                "fps_measured": len(self.stamps) / dur if dur > 0 else 0.0,
                "gap_mean_ms": float(gaps.mean()),
                "gap_p95_ms": float(np.percentile(gaps, 95)),
                "gap_max_ms": float(gaps.max()),
                "stalls": int((gaps > MAX_GAP_MS).sum()),
            }
        )
        return s


def cmd_run(a: argparse.Namespace) -> None:
    cams = [int(x) for x in a.cams.split(",") if x.strip() != ""]
    if not cams:
        raise SystemExit("[!] --cams 0,2 형태로 지정하세요.")
    if a.seconds < MIN_SECONDS:
        print(f"[!] #90 기준이 {MIN_SECONDS:.0f}초입니다. --seconds 를 늘리세요.")

    power_before = check_power()
    print(f"\n카메라 {cams} · {a.width}x{a.height} @ {a.fps} · {a.fourcc} · {a.seconds:.0f}초")
    if power_before.get("usb_max_current_enable") == 0:
        print("! USB 600 mA 제한 상태입니다. 결과가 전력 문제로 오염될 수 있습니다.")

    stop = threading.Event()
    workers = [CamWorker(c, a, stop) for c in cams]
    for w in workers:
        w.start()
        time.sleep(0.4)  # 동시 열기 시 열거 충돌을 피합니다

    t0 = time.time()
    while time.time() - t0 < a.seconds:
        time.sleep(1.0)
        el = time.time() - t0
        print(
            f"\r  {el:5.1f}s  " + "  ".join(f"cam{w.idx}:{len(w.stamps):5d}f" for w in workers),
            end="",
            flush=True,
        )
    stop.set()
    for w in workers:
        w.join(timeout=5.0)
    print()

    rows = [w.stats() for w in workers]
    power_after = check_power()

    print(
        f"\n{'카메라':<8}{'포맷':>8}{'해상도':>12}{'측정 fps':>10}{'p95 간격':>10}"
        f"{'최대 간격':>10}{'끊김':>7}{'실패':>7}  판정"
    )
    print("-" * 82)
    all_pass = True
    for r in rows:
        if not r["opened"]:
            print(
                f"{r['camera']:<8}{'-':>8}{'-':>12}{'-':>10}{'-':>10}{'-':>10}"
                f"{'-':>7}{'-':>7}  FAIL ({r['error']})"
            )
            all_pass = False
            continue
        act = r["actual"]
        fps_ok = r["fps_measured"] >= a.fps * FPS_TOLERANCE
        gap_ok = (r["stalls"] or 0) == 0
        fmt_ok = act["fourcc"].strip("\x00") == a.fourcc
        ok = fps_ok and gap_ok and fmt_ok
        all_pass &= ok
        print(
            f"{r['camera']:<8}{act['fourcc']:>8}{act['width']}x{act['height']:>7}"
            f"{r['fps_measured']:>10.2f}{r['gap_p95_ms']:>10.1f}{r['gap_max_ms']:>10.1f}"
            f"{r['stalls']:>7}{r['read_fail']:>7}  {'PASS' if ok else 'FAIL'}"
        )
        if not fmt_ok:
            print(
                f"         ! MJPG 요청이 {act['fourcc']} 로 되돌아갔습니다 — 대역폭이 폭증합니다."
            )

    print("-" * 82)
    print(f"종합: {'PASS' if all_pass else 'FAIL'}")
    if power_after.get("undervoltage_now") or power_after.get("undervoltage_since_boot"):
        print("! 시험 중 저전압이 감지되었습니다. 어댑터부터 해결해야 결과가 유효합니다.")
        all_pass = False

    entry = {
        "at": datetime.now().isoformat(timespec="seconds"),
        "label": a.label,
        "cameras": cams,
        "seconds": a.seconds,
        "power_before": power_before,
        "power_after": power_after,
        "results": rows,
        "pass": bool(all_pass),
    }
    history = []
    if os.path.exists(RESULT_JSON):
        with open(RESULT_JSON, encoding="utf-8") as f:
            history = json.load(f).get("runs", [])
    history.append(entry)
    with open(RESULT_JSON, "w", encoding="utf-8") as f:
        json.dump({"runs": history}, f, ensure_ascii=False, indent=2)
    print(f"\n[+] {RESULT_JSON} 저장 ({len(history)}번째 시행)")


# ---------------------------------------------------------------- 리포트


def cmd_report(_: argparse.Namespace) -> None:
    if not os.path.exists(RESULT_JSON):
        raise SystemExit(f"[!] {RESULT_JSON} 이 없습니다. run 을 먼저 실행하세요.")
    with open(RESULT_JSON, encoding="utf-8") as f:
        runs = json.load(f)["runs"]

    L: list[str] = []
    add = L.append
    add("## A1-d · USB 카메라 동시 스트리밍 검증")
    add("")
    add(
        f"판정 기준 — {MIN_SECONDS:.0f}초 무중단 · 프레임 간격 {MAX_GAP_MS:.0f} ms 초과 0회 · "
        f"측정 fps ≥ 목표의 {FPS_TOLERANCE * 100:.0f}%"
    )
    add("")
    add("| # | 구성 | 카메라 | 시간 | 결과 |")
    add("|---|---|---|---:|---|")
    for i, r in enumerate(runs, 1):
        add(
            f"| {i} | {r['label'] or '—'} | `{r['cameras']}` | {r['seconds']:.0f}s | "
            f"{'✅ PASS' if r['pass'] else '❌ FAIL'} |"
        )
    add("")

    last = runs[-1]
    add(f"### 최종 시행 상세 — {last['label'] or '구성 미기재'}")
    add("")
    add("| 카메라 | 실제 포맷 | 해상도 | 측정 fps | p95 간격 | 최대 간격 | 끊김 | 읽기 실패 |")
    add("|---|---|---|---:|---:|---:|---:|---:|")
    for r in last["results"]:
        if not r["opened"]:
            add(f"| {r['camera']} | — | — | — | — | — | — | 열기 실패 |")
            continue
        a_ = r["actual"]
        add(
            f"| {r['camera']} | `{a_['fourcc']}` | {a_['width']}×{a_['height']} | "
            f"{r['fps_measured']:.2f} | {r['gap_p95_ms']:.1f} ms | {r['gap_max_ms']:.1f} ms | "
            f"**{r['stalls']}** | {r['read_fail']} |"
        )
    add("")

    pw = last.get("power_after") or {}
    if pw:
        add("### 전력 상태")
        add("")
        limit = pw.get("usb_max_current_enable")
        if limit is not None:
            add(
                f"- USB 주변장치 한도: **{'1.6 A' if limit else '600 mA 🔴'}**"
                f"{'' if limit else ' — 공식 5 V/5 A 어댑터가 아닙니다'}"
            )
        if pw.get("undervoltage_since_boot") is not None:
            add(f"- 부팅 이후 저전압: {'있음 🔴' if pw['undervoltage_since_boot'] else '없음 ✅'}")
        add("")

    with open(REPORT_MD, "w", encoding="utf-8") as f:
        f.write("\n".join(L))
    print(f"[+] {REPORT_MD} 저장")
    print(f'    gh issue comment 90 --body-file "{os.path.abspath(REPORT_MD)}"')


# ---------------------------------------------------------------- CLI


def main() -> None:
    ap = argparse.ArgumentParser(description="USB 카메라 공존 부하 시험 (#90 / #87)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("probe", help="전력·USB·V4L2 상태 점검")
    p.set_defaults(func=cmd_probe)

    p = sub.add_parser("run", help="동시 스트리밍 부하 시험")
    p.add_argument("--cams", default="0,2", help="예: 0,2 또는 0,2,4")
    p.add_argument("--width", type=int, default=1920)
    p.add_argument("--height", type=int, default=1080)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--fourcc", default="MJPG")
    p.add_argument("--backend", default=None, help="videoio 백엔드 이름 (미지정 시 자동)")
    p.add_argument("--seconds", type=float, default=60.0)
    p.add_argument("--label", default="")
    p.set_defaults(func=cmd_run)

    p = sub.add_parser("report", help="#90 에 붙일 마크다운 생성")
    p.set_defaults(func=cmd_report)

    a = ap.parse_args()
    a.func(a)


if __name__ == "__main__":
    main()
