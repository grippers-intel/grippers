#!/usr/bin/env python3
"""기물이 왜 "작업 영역에 없다"고 나오는지 단계별로 찍어 본다.

`run_mission.py` 는 최종 결과(작업 영역 안 라벨 목록)만 보여 준다. 그게 비면
어느 단계에서 빠졌는지 알 수가 없다 — geti 가 못 봤는지, 좌표 변환이 실패했는지,
추적기가 확정을 못 했는지, 작업 영역 밖으로 계산됐는지.

이 도구는 그 네 단계를 매 사이클 한 줄로 찍는다.

    geti 검출     라벨과 확률 (문턱 적용 전)
    좌표 변환     지도 좌표 (m). cam.ready 가 아니면 여기서 빈다
    추적기        트랙 수와 확정 여부 (PIECE_CONFIRM_SEC = 1.2초 필요)
    작업 영역     WORKSPACE_X/Y 안인가

⚠️ run_mission.py 와 **같이 돌리지 마십시오.** 카메라를 두 프로세스가 열 수
없습니다. 미션을 멈추고 이것만 돌린 뒤, 확인이 끝나면 미션을 다시 띄우십시오.

사용법
    python diag_pieces.py
    python diag_pieces.py --cams 0 1 --seconds 30
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).parent / "aruco"))

import config as cfg
from localizer import Camera, RobotLocalizer, detect, make_detector

import geti_detector
import mission_config as mcfg
import piece_map
from mission import _in_workspace
from run_localize import open_cams


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cams", type=int, nargs="+", default=list(cfg.CAM_INDICES))
    ap.add_argument("--geti-device", type=str, default="CPU")
    ap.add_argument("--seconds", type=float, default=30.0)
    args = ap.parse_args()

    detector = make_detector()
    cams = [Camera.load(f"cam{i}", i) for i in args.cams]
    for c in cams:
        print(f"{c.name}: calibrated={c.calibrated}"
              + ("" if c.calibrated else "   ⚠️ npz 없음 — 위치가 몇 cm 틀립니다"))
    caps = open_cams(args.cams)
    if not any(c.isOpened() for c in caps):
        print("열린 카메라가 없습니다")
        return 1

    print(f"geti 불러오는 중 ({args.geti_device}) ...")
    workers = [geti_detector.GetiWorker(
        geti_detector.load_deployment(device=args.geti_device), c.name) for c in cams]
    loc = RobotLocalizer()
    tracker = piece_map.PieceTracker()

    print(f"\n작업 영역  X {cfg.WORKSPACE_X}  Y {cfg.WORKSPACE_Y}")
    print(f"문턱 conf {mcfg.PIECE_CONF_THRESHOLD}  병합거리 {mcfg.PIECE_MERGE_DIST_M}m  "
          f"확정 {mcfg.PIECE_CONFIRM_SEC}s\n")

    end = time.time() + args.seconds
    n = 0
    try:
        while time.time() < end:
            grabbed, dets = [], []
            for cap in caps:
                ok, frame = cap.read()
                grabbed.append(frame if ok else None)
                dets.append({} if not ok else
                            detect(detector, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)))
            pose = loc.update(cams, dets)

            preds = []
            for frame, worker in zip(grabbed, workers):
                if frame is None:
                    preds.append(None)
                    continue
                worker.submit(frame.copy())
                preds.append(worker.latest())

            n += 1
            if n % 10:                       # 1초에 한 번쯤만 찍는다
                time.sleep(0.02)
                continue

            print(f"--- 사이클 {n}   로봇 pose ok={pose.ok}")
            for cam, pred in zip(cams, preds):
                if pred is None:
                    print(f"  {cam.name}: geti 결과 아직 없음")
                    continue
                raw = []
                for ann in pred.annotations:
                    lab = max(ann.labels, key=lambda l: l.probability, default=None)
                    if lab is not None:
                        raw.append(f"{lab.name} {lab.probability:.2f}")
                obs = piece_map.pieces_from_prediction(cam, pred)
                print(f"  {cam.name}: ready={cam.ready}  검출[{', '.join(raw) or '없음'}]"
                      f"  -> 좌표 {[f'{o.label} ({o.x:.3f}, {o.y:.3f})' for o in obs] or '없음'}")

            obs_lists = [piece_map.pieces_from_prediction(c, p) for c, p in zip(cams, preds)]
            pmap = tracker.update(obs_lists)
            tracks = getattr(tracker, "_tracks", [])
            now = time.monotonic()
            print(f"  추적기: 트랙 {len(tracks)}개 "
                  + " ".join(f"[{t.label} ({t.x:.3f},{t.y:.3f}) n={t.n_obs} "
                             f"age={now - t.first_seen:.1f}s "
                             f"{'확정' if t.confirmed(now) else '대기'}]" for t in tracks))
            if pmap:
                for label, pts in pmap.items():
                    for p in pts:
                        print(f"  결과: {label} ({p[0]:.3f}, {p[1]:.3f})  "
                              f"작업영역 {'안' if _in_workspace(p) else '★밖★'}")
            else:
                print("  결과: 확정된 기물 없음")
            print()
    finally:
        for w in workers:
            w.stop()
        for c in caps:
            c.release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
