"""합의 필터를 실제 촬영본에 돌려 오탐이 얼마나 걸러지는지 측정한다."""
import sys, glob
sys.path.insert(0, "/grippers/tools/perception")
from ultralytics import YOLO
from consensus import consensus

MODEL = "/grippers/models/best.pt"  # 2026-08-27: best_ncnn_model은 Pi에 없던 경로였다
folder, conf = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
ratio = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5

m = YOLO(MODEL, task="detect")
files = sorted(glob.glob(folder + "/*.jpg"))
per_frame = []
for f in files:
    r = m.predict(f, imgsz=640, conf=conf, verbose=False)[0]
    per_frame.append([(m.names[int(c)], float(cf), [float(v) for v in b])
                      for c, cf, b in zip(r.boxes.cls, r.boxes.conf, r.boxes.xyxy)])

raw = sum(len(d) for d in per_frame)
print(f"프레임 {len(files)}장 · conf≥{conf} · k/n={ratio}")
print(f"필터 전 : 총 {raw}개 검출 (프레임당 {raw/len(files):.2f})\n")

tracks = consensus(per_frame, len(files), min_ratio=ratio)
tracks.sort(key=lambda t: -len(t.frames))
print(f"필터 후 : 물체 {len(tracks)}개 확정\n")
print(f"  {'클래스':<9} {'지지':>7} {'위치(x,y)':>16} {'산포':>6} {'순도':>6} {'평균신뢰':>8}")
for t in tracks:
    cx, cy = t.center
    print(f"  {t.label:<9} {len(t.frames):>3}/{len(files):<3} "
          f"{cx:>7.0f},{cy:>7.0f} {t.spread:>6.1f} {t.purity:>6.2f} "
          f"{sum(t.confs)/len(t.confs):>8.3f}")
