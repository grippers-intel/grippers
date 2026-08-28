"""이동 중 추적 성능을 측정한다.

사용법
    python motion_test.py                      # q 누를 때까지 기록
    python motion_test.py --seconds 20         # 20초만
    python motion_test.py --tag 0.3ms          # 파일명에 회차 표시

조작
    SPACE : 구간 표시(마커를 찍어 둠 — 나중에 회차를 구분할 때)
    q     : 종료 후 분석

로봇을 사람이 밀거나 끈으로 당겨도 된다. 속도 기준(실측 노출 7.8 ms · 1.33 mm/px):
    0.34 m/s 이하  블러 2 px 이하 — 안전
    0.85 m/s       블러 반 칸 — 경계
    1.71 m/s       블러 한 칸 — 해독 실패
작업 공간 세로 1.6 m 를 8초에 통과하면 0.2 m/s, 5초면 0.3 m/s 다.

⚠️ 사람이 작업 공간 안에 서면 화면에 잡혀 바닥 마커 검출률이 떨어진다.
   격벽 밖에서 끈으로 당기거나 막대로 미는 쪽이 깨끗하다.

바닥 마커는 정지해 있어 블러가 없다. 외부파라미터 lock 은 이동과 무관하므로,
문제가 생기면 순수하게 로봇 마커 쪽이다.
"""

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import sys

# config.py/localizer.py 는 aruco/ 하위폴더에 있다(팀원이 동기화하는 파일이라
# 건드리지 않는다) — 그 폴더를 경로에 추가해서 기존처럼 bare import 로 쓴다.
# 예전엔 이 폴더 최상위에도 같은 이름의 사본이 있어서 이 줄 없이도 돌았지만,
# 그건 캘리브레이션 이전 실측 기록본이었고 지금은 _legacy/ 로 옮겼다.
sys.path.insert(0, str(Path(__file__).parent / "aruco"))

import config as cfg
from localizer import Camera, RobotLocalizer, detect, make_detector

FOCUS = {0: 5, 1: 0}
MOVING_MM_S = 30.0        # 이보다 빠르면 '이동 중' 으로 분류


def open_cam(idx: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if not cap.isOpened():
        return cap
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.IMG_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.IMG_H)
    # FOURCC 는 반드시 해상도 뒤에 — 먼저 걸면 DSHOW 가 무시해 5 fps 가 된다
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    if idx in FOCUS:
        cap.set(cv2.CAP_PROP_FOCUS, FOCUS[idx])
    return cap


def draw_track(rows, path: Path) -> None:
    """궤적을 그림으로 저장한다. 튀는 지점은 그림에서 바로 보인다."""
    S, PAD = 460, 40                       # 1.8 m -> 460 px
    W = int(1.8 * S / 1.8) + PAD * 2
    img = np.full((W, W, 3), 26, np.uint8)

    def to_px(x, y):
        return (int(PAD + x * S / 1.8), int(PAD + (1.8 - y) * S / 1.8))

    cv2.rectangle(img, to_px(0, 1.8), to_px(1.8, 0), (70, 70, 70), 1)
    x0, x1 = cfg.WORKSPACE_X
    y0, y1 = cfg.WORKSPACE_Y
    cv2.rectangle(img, to_px(x0, y1), to_px(x1, y0), (55, 75, 55), 1)
    for mid, (mx, my) in cfg.FLOOR_MARKER_WORLD.items():
        h = cfg.FLOOR_MARKER_SIZE / 2
        cv2.rectangle(img, to_px(mx - h, my + h), to_px(mx + h, my - h),
                      (200, 200, 200), -1)
        cv2.putText(img, str(mid), to_px(mx + h + 0.02, my),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
    for name, (bx, by, _) in cfg.BOXES.items():
        cv2.rectangle(img, to_px(bx - cfg.BOX_W / 2, by + cfg.BOX_L / 2),
                      to_px(bx + cfg.BOX_W / 2, by - cfg.BOX_L / 2),
                      (90, 140, 190), 1)
        cv2.putText(img, name, to_px(bx - 0.06, by),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (90, 140, 190), 1)

    pts = [(r["x"], r["y"], r["v"], r["fresh"]) for r in rows if r["ok"]]
    for k in range(1, len(pts)):
        x1_, y1_, v, fr = pts[k]
        x0_, y0_, _, _ = pts[k - 1]
        # 속도로 색을 준다: 느림 초록 -> 빠름 빨강
        t = min(1.0, v / 800.0)
        col = (int(60 + 40 * t), int(220 * (1 - t) + 40), int(60 + 195 * t))
        if not fr:
            col = (0, 165, 255)            # 폴백(HOLD) 구간은 주황
        cv2.line(img, to_px(x0_, y0_), to_px(x1_, y1_), col, 2)
    if pts:
        cv2.circle(img, to_px(pts[0][0], pts[0][1]), 5, (255, 255, 255), -1)
        cv2.circle(img, to_px(pts[-1][0], pts[-1][1]), 5, (0, 0, 255), 2)

    cv2.putText(img, "start(white) end(red) / HOLD=orange / speed green->red",
                (PAD, W - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 150, 150), 1)
    cv2.imwrite(str(path), img)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cams", type=int, nargs="+", default=list(cfg.CAM_INDICES))
    ap.add_argument("--seconds", type=float, default=0.0, help="0 이면 q 누를 때까지")
    ap.add_argument("--out", type=str, default="motion")
    ap.add_argument("--tag", type=str, default="", help="파일명에 붙일 회차 표시")
    ap.add_argument("--view-scale", type=float, default=0.35)
    args = ap.parse_args()

    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    stem = datetime.now().strftime("%Y%m%d_%H%M%S") + (f"_{args.tag}" if args.tag else "")

    detector = make_detector()
    cams = [Camera.load(f"cam{i}", i) for i in args.cams]
    caps = {i: open_cam(i) for i in args.cams}
    if not all(c.isOpened() for c in caps.values()):
        print("카메라를 열 수 없습니다.")
        return 1
    time.sleep(1.0)
    for c in caps.values():
        for _ in range(10):
            c.read()
    for i in args.cams:
        w = f"cam{i}"
        cv2.namedWindow(w, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(w, int(cfg.IMG_W * args.view_scale),
                         int(cfg.IMG_H * args.view_scale))

    print(__doc__)
    print("기록 시작 — 로봇을 움직이세요. q 로 종료.\n")

    loc = RobotLocalizer()
    rows, marks = [], []
    prev = None
    t0 = time.monotonic()
    try:
        while True:
            t = time.monotonic() - t0
            if args.seconds and t >= args.seconds:
                break
            frames, dets, seen = [], [], {}
            for i in args.cams:
                ok, f = caps[i].read()
                d = {} if not ok else detect(detector, cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
                frames.append(f if ok else None)
                dets.append(d)
                seen[i] = dict(
                    robot=cfg.ROBOT_MARKER_ID in d,
                    floor=len([m for m in cfg.FLOOR_MARKER_IDS if m in d]))
            p = loc.update(cams, dets)

            v = 0.0
            if p.ok and prev is not None and t > prev[0]:
                v = math.hypot(p.x - prev[1], p.y - prev[2]) * 1000 / (t - prev[0])
            if p.ok:
                prev = (t, p.x, p.y)

            gap = float("nan")
            if len(p.per_cam) == 2:
                (ax, ay, _), (bx, by, _) = list(p.per_cam.values())
                gap = math.hypot(ax - bx, ay - by) * 1000

            rows.append(dict(t=t, ok=p.ok, fresh=p.fresh, n_cams=p.n_cams,
                             x=p.x, y=p.y, yaw=p.yaw_deg, v=v, gap=gap,
                             age=p.age_s,
                             **{f"r{i}": seen[i]["robot"] for i in args.cams},
                             **{f"f{i}": seen[i]["floor"] for i in args.cams}))

            for i, f in zip(args.cams, frames):
                if f is None:
                    continue
                s = (f"cam{i}  {v:6.0f} mm/s  gap {gap:5.1f}  "
                     f"{'FRESH' if p.fresh else 'HOLD'}  n={p.n_cams}")
                cv2.putText(f, s, (14, 44), cv2.FONT_HERSHEY_SIMPLEX, 1.1,
                            (0, 0, 0), 6, cv2.LINE_AA)
                cv2.putText(f, s, (14, 44), cv2.FONT_HERSHEY_SIMPLEX, 1.1,
                            (0, 255, 0) if p.fresh else (0, 165, 255), 2, cv2.LINE_AA)
                cv2.imshow(f"cam{i}", f)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord(" "):
                marks.append(t)
                print(f"  구간 표시 @ {t:.1f}s")
    finally:
        for c in caps.values():
            c.release()
        cv2.destroyAllWindows()

    if len(rows) < 20:
        print("표본이 너무 적습니다.")
        return 1

    # ---- 저장 -------------------------------------------------------------
    csv_path = out / f"{stem}.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    png_path = out / f"{stem}.png"
    draw_track(rows, png_path)

    # ---- 분석 -------------------------------------------------------------
    n = len(rows)
    dur = rows[-1]["t"]
    okr = [r for r in rows if r["ok"]]
    fresh = [r for r in okr if r["fresh"]]
    mov = [r for r in fresh if r["v"] >= MOVING_MM_S]
    sta = [r for r in fresh if r["v"] < MOVING_MM_S]

    print(f"\n{'='*70}")
    print(f"이동 중 성능  ({n} 프레임 / {dur:.1f} 초 = {n/dur:.1f} fps)")
    print("=" * 70)
    if marks:
        print(f"  구간 표시: {', '.join(f'{m:.1f}s' for m in marks)}")

    print(f"\n  {'':16}{'정지':>12}{'이동 중':>12}")
    print(f"  {'-'*40}")
    print(f"  {'프레임':16}{len(sta):>12}{len(mov):>12}")
    for i in args.cams:
        for key, lbl in ((f"r{i}", f"cam{i} 로봇마커"), (f"f{i}", f"cam{i} 바닥 4/4")):
            def rate(g):
                if not g:
                    return float("nan")
                if key.startswith("r"):
                    return 100 * np.mean([r[key] for r in g])
                return 100 * np.mean([r[key] == 4 for r in g])
            print(f"  {lbl:16}{rate(sta):>11.1f}%{rate(mov):>11.1f}%")
    for lbl, g in (("두 대 사용", None),):
        s = 100 * np.mean([r["n_cams"] == 2 for r in sta]) if sta else float("nan")
        m = 100 * np.mean([r["n_cams"] == 2 for r in mov]) if mov else float("nan")
        print(f"  {lbl:16}{s:>11.1f}%{m:>11.1f}%")
    for lbl, key in (("두 카메라 차이", "gap"),):
        s = np.nanmean([r[key] for r in sta]) if sta else float("nan")
        m = np.nanmean([r[key] for r in mov]) if mov else float("nan")
        print(f"  {lbl:16}{s:>10.1f}mm{m:>10.1f}mm")

    if mov:
        v = np.array([r["v"] for r in mov])
        print(f"\n  이동 속도: 평균 {v.mean():.0f} · 중앙 {np.median(v):.0f} · "
              f"최대 {v.max():.0f} mm/s  ({v.mean()/1000:.2f} m/s)")
        for lo, hi, lbl in ((0, 340, "안전(<0.34)"), (340, 850, "경계(0.34~0.85)"),
                            (850, 1e9, "위험(>0.85)")):
            k = np.mean((v >= lo) & (v < hi)) * 100
            if k > 0:
                print(f"     {lbl:18}: {k:5.1f}% 의 프레임")

    # 폴백(HOLD) 구간
    holds, cur = [], 0
    for r in rows:
        if r["ok"] and not r["fresh"]:
            cur += 1
        elif cur:
            holds.append(cur)
            cur = 0
    if cur:
        holds.append(cur)
    lost = sum(1 for r in rows if not r["ok"])
    print(f"\n  폴백(HOLD) 발생 {len(holds)} 회"
          + (f" · 최장 {max(holds)} 프레임 ({max(holds)/(n/dur):.2f} 초)" if holds else "")
          + f" · 완전 LOST {lost} 프레임")

    # 궤적 튐
    if len(fresh) > 5:
        steps = [math.hypot(fresh[k]["x"] - fresh[k-1]["x"],
                            fresh[k]["y"] - fresh[k-1]["y"]) * 1000
                 for k in range(1, len(fresh))]
        s = np.array(steps)
        med = np.median(s)
        big = s[s > max(med * 4, 20)]
        print(f"  프레임간 이동: 중앙 {med:.1f} mm · 최대 {s.max():.1f} mm"
              f" · 튐(중앙 4배 초과) {len(big)} 회")

    print(f"\n  저장: {csv_path.name} · {png_path.name}  ({out})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
