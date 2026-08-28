"""geti 추론을 CPU 와 iGPU 에서 나란히 돌려 **같은 것을 검출하는지** 확인한다.

## 왜 이 스크립트가 필요한가

2026-08-27 실측으로 iGPU 추론이 CPU 보다 1.6배 빠르다는 것은 확인됐다
(468.8 -> 298.7ms). 그런데 **명령 주기는 1.60 -> 1.63Hz 로 변하지 않았다** —
병목이 CPU 부족이 아니라 live_map 의 렌더였기 때문이다. 그래서 지금은 디바이스를
바꿀 이유가 없고, 기본값은 CPU 다.

바꿀 이유가 생기는 것은 두 가지가 같이 만족될 때다:

1. **검출이 동등한가** — GPU 는 다른 커널을 쓰므로 confidence 가 미세하게
   다를 수 있다. `mission_config.PIECE_CONF_THRESHOLD`(0.6) 문턱 근처에 걸린
   기물이 한쪽에서만 잡히면, 크래시가 아니라 **"기물 하나가 가끔 안 잡힘"**
   으로 나타난다 — 원인 찾기가 가장 어려운 종류다.
2. **주기가 실제로 빨라지는가** — live_map 을 고친 뒤에야 측정이 의미를 갖는다.
   **실제 시연 때 띄워 둘 것만 띄운 상태**에서 재야 한다.

   > 2026-08-28: 2번은 답이 나왔다. live_map 을 고친 뒤 CPU 로 **~7.0 Hz**
   > (워치독 3.33 Hz 의 2.1배). 주기를 더 벌 이유가 없어 **CPU 유지로
   > 확정**했다. 그래서 이 스크립트의 목적은 "GPU 로 갈까"가 아니라
   > **"당일 문제가 생기면 GPU 로 도망칠 수 있나"** 를 미리 아는 보험이다.

## 쓰는 법

    # 카메라로 라이브 비교 (기물을 아레나에 놓고)
    python tools_geti_ab.py

    # 저장된 이미지로 (카메라 없이도 됨)
    python tools_geti_ab.py --images dataset/cam0

    # 시연 조건 재현 — 당일 띄워 둘 것만 띄운 상태로 돌릴 것
    python tools_geti_ab.py --frames 30

같은 프레임을 두 디바이스에 넣어야 비교가 성립하므로, 프레임을 먼저 모아 두고
디바이스별로 순차 추론한다.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
import warnings
from pathlib import Path

import cv2
import numpy as np

warnings.filterwarnings("ignore")

sys.path.insert(0, str(Path(__file__).parent / "aruco"))

import config as cfg
import geti_detector
import mission_config as mcfg

# 같은 물체로 볼 최대 중심 거리(픽셀). 두 디바이스가 같은 물체를 조금 다른
# bbox 로 낼 수 있어서, 좌표가 정확히 같기를 요구하지 않는다.
MATCH_PX = 40.0


def _boxes(prediction):
    """(라벨, confidence, 중심x, 중심y) 목록. piece_map 과 같은 방식으로 읽는다."""
    out = []
    if prediction is None:
        return out
    for obj in prediction.annotations:
        if not obj.labels:
            continue
        lab = obj.labels[0]
        s = obj.shape
        cx = s.x + s.width / 2.0
        cy = s.y + s.height / 2.0
        out.append((lab.name, float(lab.probability), cx, cy))
    return out


def _match(a, b):
    """두 목록을 중심 거리로 짝짓는다. (짝, a에만, b에만)"""
    pairs, used = [], set()
    for i, (la, pa, xa, ya) in enumerate(a):
        best, best_d = None, MATCH_PX
        for j, (lb, pb, xb, yb) in enumerate(b):
            if j in used:
                continue
            d = float(np.hypot(xa - xb, ya - yb))
            if d < best_d:
                best, best_d = j, d
        if best is None:
            continue
        used.add(best)
        pairs.append((i, best, best_d))
    only_a = [i for i in range(len(a)) if i not in {p[0] for p in pairs}]
    only_b = [j for j in range(len(b)) if j not in used]
    return pairs, only_a, only_b


def collect_frames(args) -> list[np.ndarray]:
    if args.images:
        paths = sorted(Path(args.images).rglob("*.jpg"))[: args.frames]
        if not paths:
            print(f"이미지가 없습니다: {args.images}")
            sys.exit(1)
        print(f"이미지 {len(paths)}장 사용: {args.images}")
        return [cv2.cvtColor(cv2.imread(str(p)), cv2.COLOR_BGR2RGB) for p in paths]

    caps = []
    for i in cfg.CAM_INDICES:
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.IMG_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.IMG_H)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        if not cap.isOpened():
            print(f"⚠️ 카메라 {i} 를 열 수 없습니다.")
        caps.append(cap)
    for _ in range(10):
        for c in caps:
            c.read()

    frames = []
    print(f"카메라 {list(cfg.CAM_INDICES)} 에서 {args.frames} 프레임 수집 중...")
    while len(frames) < args.frames:
        for cap in caps:
            ok, fr = cap.read()
            if ok:
                frames.append(cv2.cvtColor(fr, cv2.COLOR_BGR2RGB))
        time.sleep(0.05)
    for c in caps:
        c.release()
    return frames[: args.frames]


def infer_all(device: str, frames: list[np.ndarray]):
    t = time.perf_counter()
    dep = geti_detector.load_deployment(device=device)
    load = time.perf_counter() - t
    dep.infer(frames[0])                     # 워밍업 (첫 회는 커널 준비 포함)
    results, times = [], []
    for f in frames:
        t = time.perf_counter()
        results.append(_boxes(dep.infer(f)))
        times.append(time.perf_counter() - t)
    return results, load, statistics.median(times)


def main() -> int:
    ap = argparse.ArgumentParser(description="geti CPU vs iGPU 검출 동등성 비교")
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--images", type=str, default=None,
                    help="카메라 대신 이 폴더의 jpg 를 쓴다")
    ap.add_argument("--devices", nargs=2, default=["CPU", "GPU.0"])
    args = ap.parse_args()

    frames = collect_frames(args)
    thr = mcfg.PIECE_CONF_THRESHOLD
    print(f"프레임 {len(frames)}장 · 문턱 PIECE_CONF_THRESHOLD = {thr}\n")

    out = {}
    for dev in args.devices:
        try:
            res, load, med = infer_all(dev, frames)
        except Exception as exc:
            print(f"{dev:7s}  실패 — {type(exc).__name__}: {str(exc)[:100]}")
            return 1
        out[dev] = res
        kept = sum(1 for r in res for (_l, p, _x, _y) in r if p >= thr)
        print(f"{dev:7s}  로드 {load:5.1f}s  추론 중앙값 {med*1000:6.1f}ms  "
              f"문턱 통과 검출 {kept}개")

    a_dev, b_dev = args.devices
    A, B = out[a_dev], out[b_dev]

    same_label = diff_label = 0
    only_a = only_b = 0
    flips = []          # 문턱을 사이에 두고 갈린 것
    dprob = []
    # mission 은 문턱 미만 검출을 버린다(piece_map). 그래서 0.59 짜리가 한쪽에만
    # 있는 것은 차이가 아니다 — 판정이 노이즈 하나에 뒤집히지 않도록
    # 문턱 이상만 따로 센다.
    lost_a, lost_b, mislabel = [], [], []
    for k, (ra, rb) in enumerate(zip(A, B)):
        pairs, oa, ob = _match(ra, rb)
        only_a += len(oa)
        only_b += len(ob)
        for i in oa:
            if ra[i][1] >= thr:
                lost_a.append((k, ra[i]))
        for j in ob:
            if rb[j][1] >= thr:
                lost_b.append((k, rb[j]))
        for i, j, _d in pairs:
            la, pa, _, _ = ra[i]
            lb, pb, _, _ = rb[j]
            if la == lb:
                same_label += 1
            else:
                diff_label += 1
                if max(pa, pb) >= thr:
                    mislabel.append((k, la, lb, pa, pb))
            dprob.append(pb - pa)
            if (pa >= thr) != (pb >= thr):
                flips.append((k, la, lb, pa, pb))

    print(f"\n=== 비교: {a_dev} vs {b_dev} ===")
    print(f"  짝지어진 검출        {same_label + diff_label}개 "
          f"(라벨 일치 {same_label} · 불일치 {diff_label})")
    print(f"  {a_dev} 에만 있음      {only_a}개")
    print(f"  {b_dev} 에만 있음      {only_b}개")
    if dprob:
        print(f"  confidence 차이      평균 {statistics.mean(dprob):+.4f} · "
              f"최대 {max(dprob, key=abs):+.4f}")
    print(f"  ★ 문턱({thr}) 을 사이에 두고 갈린 검출: {len(flips)}개")
    for k, la, lb, pa, pb in flips[:10]:
        print(f"      프레임 {k}: {la}/{lb}  {a_dev} {pa:.3f} vs {b_dev} {pb:.3f}")

    print("  (문턱 미만 검출도 포함한 원자료다 — mission 은 그것들을 버리므로"
          " 판정 근거가 아니다.)")

    # --- 미션에 영향이 있는 것만 ---
    print(f"\n  --- 문턱({thr}) 이상만 본 차이 ---")
    print(f"  ★ {a_dev} 에서만 문턱 통과(={b_dev} 로 가면 놓침): {len(lost_a)}개")
    for k, (lab, pp, cx, cy) in lost_a[:10]:
        print(f"      프레임 {k}: {lab} {pp:.3f} 중심({cx:.0f},{cy:.0f})")
    print(f"  ★ {b_dev} 에서만 문턱 통과(={a_dev} 가 놓치는 것): {len(lost_b)}개")
    for k, (lab, pp, cx, cy) in lost_b[:10]:
        print(f"      프레임 {k}: {lab} {pp:.3f} 중심({cx:.0f},{cy:.0f})")
    print(f"  ★ 문턱 이상인데 라벨이 갈린 검출: {len(mislabel)}개")
    for k, la, lb, pa, pb in mislabel[:10]:
        print(f"      프레임 {k}: {a_dev} {la} {pa:.3f} vs {b_dev} {lb} {pb:.3f}")

    ok = not (flips or lost_a or lost_b or mislabel)
    print("\n" + ("  판정: 동등 — 디바이스를 바꿔도 **mission 이 보는 검출**은 같습니다."
                  if ok else
                  "  판정: ★ 차이 있음 — 위 ★ 항목을 확인하고, 애매하면 CPU 를 유지하세요."))
    print("  (주기 비교는 live_map 수정 후 run_mission.py --geti-device 로 따로 재세요.)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
