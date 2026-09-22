"""체스보드로 탑뷰 카메라 내부파라미터를 잰다 -> host/calib/cam{index}.npz (K, dist).

사용법 (host/ 에서):
    python tools/calibrate_camera.py --cam 0 --cols 9 --rows 6 --square 0.025

- cols/rows 는 **내부 코너** 개수다(칸 수가 아니다).
- 체스보드를 화면 구석구석, 여러 기울기로 20장 이상 모은다.
- 촬영 해상도는 실제 운용과 같아야 한다(1280x720). 다르면 K 가 맞지 않는다.
- 오토포커스를 끈 상태에서 잰다. 초점이 바뀌면 다시 잴 것.

키: space = 코너가 잡힌 프레임 저장 · c = 계산하고 저장 · q = 종료
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

HOST_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST_ROOT))

from localization.cameras import open_cams  # noqa: E402


def main() -> int:
    import host_config
    host_config.configure_console()
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", type=int, required=True)
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square", type=float, default=0.025, help="칸 한 변(m)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    out = Path(args.out) if args.out else HOST_ROOT / "calib" / f"cam{args.cam}.npz"
    pattern = (args.cols, args.rows)
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    cap = open_cams([args.cam], args.width, args.height)[0]
    if not cap.isOpened():
        return 1
    obj_pts, img_pts, size = [], [], None
    win = f"calibrate cam{args.cam}"
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK)
        disp = frame.copy()
        if found:
            cv2.drawChessboardCorners(disp, pattern, corners, found)
        cv2.putText(disp, f"samples={len(obj_pts)} found={found}  space=save c=calibrate q=quit",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imshow(win, disp)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" ") and found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            obj_pts.append(objp)
            img_pts.append(corners)
            print(f"sample {len(obj_pts)}")
        if key == ord("c"):
            if len(obj_pts) < 10:
                print("최소 10장(권장 20장 이상) 필요합니다")
                continue
            rms, K, dist, _r, _t = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
            out.parent.mkdir(parents=True, exist_ok=True)
            np.savez(out, K=K, dist=dist, image_size=np.array(size), rms=rms)
            print(f"저장: {out}  RMS={rms:.3f}px  (1.0 미만 권장)\nK=\n{K}\ndist={dist.ravel()}")
            break
    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
