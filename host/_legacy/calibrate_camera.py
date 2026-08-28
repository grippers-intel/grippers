"""C920 내부파라미터 캘리브레이션 (체스보드).

사용법
    python calibrate_camera.py --cam 0
    python calibrate_camera.py --cam 1
    python calibrate_camera.py --cam 0 --focus 30     # 초점 수동 고정

조작
    SPACE : 현재 프레임 캡처 (코너가 잡힌 상태에서만)
    c     : 캘리브레이션 실행 후 calib/cam{N}.npz 저장
    q     : 종료

보드 — 기본값은 7x7 내부코너(= 8x8 칸, 일반 체스판). 다른 보드는 --board 로.

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

# 체스보드 내부 코너 개수 (칸 수 - 1). 7x7 은 8x8 칸 = 일반 체스판.
BOARD = (7, 7)

# 한 칸의 실제 한 변 길이(m) — 자로 잰 값을 적는다.
# 참고: 이 값은 K / dist 에 영향을 주지 않는다. 물체 좌표를 균일하게
# 스케일하면 tvec 만 같이 스케일되고 내부파라미터는 그대로다.
# 내부파라미터만 저장하는 이 스크립트에서는 대충 맞아도 결과가 같다.
SQUARE_M = 0.025


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", type=int, required=True, help="cv2.VideoCapture 인덱스")
    ap.add_argument("--board", default="7x7")
    ap.add_argument("--square", type=float, default=SQUARE_M)
    ap.add_argument("--focus", type=int, default=None,
                    help="수동 초점값 고정 (C920/DSHOW: 0=먼 거리 ~ 255=가까움, 5 단위). "
                         "생략하면 초점을 건드리지 않는다.")
    ap.add_argument("--view-scale", type=float, default=0.5,
                    help="미리보기 창 크기 배율 (기본 0.5). 1080p 원본은 화면을 넘는다. "
                         "창 모서리를 끌어 조절해도 비율은 유지된다.")
    args = ap.parse_args()

    bw, bh = (int(v) for v in args.board.lower().split("x"))
    board = (bw, bh)

    objp = np.zeros((bw * bh, 3), np.float32)
    objp[:, :2] = np.mgrid[0:bw, 0:bh].T.reshape(-1, 2) * args.square

    cap = cv2.VideoCapture(args.cam, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"카메라 {args.cam} 를 열 수 없습니다.")
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.IMG_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.IMG_H)
    # ⚠️ FOURCC 는 반드시 해상도 '뒤에'. 순서를 바꾸면 DSHOW 가 조용히 무시한다
    #    (실측: 먼저 5.0 fps · 나중 27 fps). 1080p YUY2 는 USB2 대역폭을 넘어
    #    카메라가 5 fps 로 협상해 버려서, 촬영이 견디기 힘들 만큼 느려진다.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    # C920 오토포커스 끄기 — 초점이 흔들리면 캘리브레이션이 무효화된다.
    # 주의: DSHOW 백엔드에서 이 set 은 True 를 반환하면서도 실제로는 먹지 않는다.
    #       되읽어서 확인하고, 안 꺼졌으면 --focus 로 고정하도록 안내한다.
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    af = cap.get(cv2.CAP_PROP_AUTOFOCUS)
    if args.focus is not None:
        cap.set(cv2.CAP_PROP_FOCUS, args.focus)
        print(f"초점 고정: FOCUS={cap.get(cv2.CAP_PROP_FOCUS):.0f}")
    elif af != 0:
        print(f"⚠️ 오토포커스가 꺼지지 않았습니다 (AUTOFOCUS={af:g}). "
              f"촬영 중 초점이 움직이면 캘리브레이션이 무효가 됩니다.")
        print("   --focus 30 처럼 초점값을 직접 지정하는 것을 권장합니다.")

    obj_pts: list[np.ndarray] = []
    img_pts: list[np.ndarray] = []
    size = None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # 1080p 를 원본 크기로 띄우면 화면을 넘어 코너 표시를 확인할 수 없다.
    # WINDOW_NORMAL 이라야 조절되고 WINDOW_KEEPRATIO 가 비율을 지킨다.
    win = f"calib cam{args.cam}"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow(win, int(cfg.IMG_W * args.view_scale),
                     int(cfg.IMG_H * args.view_scale))

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
        cv2.imshow(win, view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" "):
            if found:
                obj_pts.append(objp.copy())
                img_pts.append(corners)
                print(f"  캡처 {len(obj_pts)}")
            else:
                print("  코너 미검출 — 캡처되지 않음. 화면 글자가 초록일 때 누르세요.")
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
