"""C920 내부파라미터 캘리브레이션 (체스보드).

사용법
    python calibrate_camera.py --cam 0
    python calibrate_camera.py --cam 1

조작
    SPACE : 현재 프레임 캡처 (코너가 잡힌 상태에서만)
    c     : 캘리브레이션 실행 후 calib/cam{N}.npz 저장
    q     : 종료

권장 — 체스보드를 화면 구석·기울기·거리를 바꿔가며 15~25 장.
정면 사진만 모으면 왜곡계수가 안 풀린다.

⚠️ 이 단계를 건너뛰면 30 mm 목표는 못 맞춘다. C920 도 개체차가 있어서
   HFOV 로 만든 근사값으로는 화면 가장자리에서 수십 mm 씩 틀어진다.

⚠️ C920 은 오토포커스가 있다. 촬영 중 초점이 움직이면 fx/fy 가 흔들려
   캘리브레이션이 무효화된다 — 이 스크립트가 시작할 때 오토포커스를
   꺼서(CAP_PROP_AUTOFOCUS=0) 고정 초점으로 찍는다. 필요하면 카메라
   설정 프로그램(로지텍 캡처 등)에서 초점을 원하는 거리에 수동으로
   맞춘 뒤 실행할 것.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np

import config as cfg

# 체스보드 내부 코너 개수 (칸 수 - 1). 9x6 은 10x7 칸 보드.
BOARD = (9, 6)
SQUARE_M = 0.025          # 한 칸의 실제 한 변 길이(m) — 인쇄물을 자로 잴 것


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", type=int, required=True, help="cv2.VideoCapture 인덱스")
    ap.add_argument("--board", default="9x6")
    ap.add_argument("--square", type=float, default=SQUARE_M)
    args = ap.parse_args()

    bw, bh = (int(v) for v in args.board.lower().split("x"))
    board = (bw, bh)

    objp = np.zeros((bw * bh, 3), np.float32)
    objp[:, :2] = np.mgrid[0:bw, 0:bh].T.reshape(-1, 2) * args.square

    cap = cv2.VideoCapture(args.cam, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.IMG_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.IMG_H)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)   # C920 오토포커스 끄기 — 초점이 흔들리면 캘리브레이션이 무효화된다
    if not cap.isOpened():
        print(f"카메라 {args.cam} 를 열 수 없습니다.")
        return 1

    obj_pts: list[np.ndarray] = []
    img_pts: list[np.ndarray] = []
    size = None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    print(__doc__)
    while True:
        ok, frame = cap.read()
        if not ok:
            print("프레임 읽기 실패")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]

        found, corners = cv2.findChessboardCorners(
            gray, board,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK)
        view = frame.copy()
        if found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
            cv2.drawChessboardCorners(view, board, corners, found)

        cv2.putText(view, f"captured: {len(obj_pts)}   SPACE=capture  c=calibrate  q=quit",
                    (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 0) if found else (0, 0, 255), 2)
        cv2.imshow(f"calib cam{args.cam}", view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" ") and found:
            obj_pts.append(objp.copy())
            img_pts.append(corners)
            print(f"  캡처 {len(obj_pts)}")
        if key == ord("c"):
            if len(obj_pts) < 8:
                print(f"  표본이 부족합니다 ({len(obj_pts)}장). 최소 8장, 15장 이상 권장.")
                continue
            rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
                obj_pts, img_pts, size, None, None)

            errs = []
            for i in range(len(obj_pts)):
                proj, _ = cv2.projectPoints(obj_pts[i], rvecs[i], tvecs[i], K, dist)
                errs.append(np.sqrt(((proj.reshape(-1, 2) - img_pts[i].reshape(-1, 2)) ** 2)
                                    .sum(axis=1).mean()))
            out = Path(cfg.CALIB_DIR)
            out.mkdir(exist_ok=True)
            path = out / f"cam{args.cam}.npz"
            np.savez(path, K=K, dist=dist, rms=rms, image_size=np.array(size))

            print(f"\n저장: {path}")
            print(f"  RMS            : {rms:.4f} px")
            print(f"  평균 재투영오차 : {np.mean(errs):.4f} px  (최대 {np.max(errs):.4f})")
            print(f"  fx, fy         : {K[0,0]:.1f}, {K[1,1]:.1f}")
            print(f"  cx, cy         : {K[0,2]:.1f}, {K[1,2]:.1f}")
            print(f"  dist           : {dist.ravel()}")
            if rms > 0.6:
                print("  ⚠️ RMS 가 큽니다. 보드를 더 다양한 각도/거리로 다시 찍으세요.")
            break

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
