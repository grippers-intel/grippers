"""C920 내부파라미터 캘리브레이션 (체스보드).

사용법 (저장소 루트에서 실행)
    python aruco\calibrate_camera.py --cam 0     # 첫 번째 C920
    python aruco\calibrate_camera.py --cam 1     # 두 번째 C920
    python aruco\calibrate_camera.py --list      # 이 PC 의 카메라 목록만 본다

--cam 은 장치 번호가 아니라 'C920 중 몇 번째냐'다. 카메라는 이름으로 찾으므로
휴대폰 연결 카메라가 켜져 있어도 그쪽을 잡지 않는다. 저장 파일 이름은 실제
장치 번호를 따른다 (calib/cam0.npz, calib/cam2.npz ...).

저장 위치
    <저장소 루트>/calib/cam{장치번호}.npz — 실행 위치와 무관하게 항상 여기다.
    localizer.Camera.load() 가 읽는 자리와 같다.

    ⚠️ 파일 이름이 "장치 번호"에 묶여 있다. 캘리브레이션 뒤에 USB 포트를 바꿔
       번호가 뒤바뀌면 두 카메라의 내부파라미터가 서로 뒤집혀 적용된다 —
       조용히 틀린다. 포트를 바꿨으면 --list 로 번호를 다시 확인할 것.

조작
    SPACE : 현재 프레임 캡처 (코너가 잡힌 상태에서만)
    c     : 캘리브레이션 실행 후 calib/cam{N}.npz 저장
    q     : 종료

    창은 가장자리를 끌어 크기를 바꿀 수 있다. 화면 구석까지 체스판을 가져갈 때
    창을 키워 놓으면 코너가 잡혔는지 눈으로 확인하기 쉽다.

체스판 규격
    기본값은 이 프로젝트가 쓰는 실물 체스판이다 — 놀이칸 8x8 (내부 코너 7x7),
    놀이 영역 280 mm 이므로 한 칸 35 mm. 다른 판을 쓰면 --board 로 바꾼다.
    --board 는 "칸 수"가 아니라 "내부 코너 수"다. 8칸 판이면 7x7 이다.

    ※ --square 는 결과에 영향이 없다. 이 프로그램이 저장하는 값은 내부파라미터
      K 와 왜곡계수뿐인데 둘 다 스케일과 무관하기 때문이다. 실측으로 확인했다 —
      같은 사진에 0.025 / 0.035 / 0.100 을 넣어도 fx·주점·왜곡이 소수점까지 같다.
      (달라지는 건 "체스판이 몇 m 앞에 있었나" 뿐이고 그건 쓰지 않는다)
      반대로 --board 가 틀리면 한 장도 검출되지 않는다. 조용히 틀리지는 않는다.

권장 — 체스보드를 화면 구석·기울기·거리를 바꿔가며 15~25 장.
정면 사진만 모으면 왜곡계수가 안 풀린다.
바닥 마커가 찍히는 영역(화면 가장자리와 아래쪽)에 판을 꼭 가져갈 것 —
가운데서만 찍으면 정작 필요한 곳의 왜곡이 안 풀린다.

⚠️ 이 단계를 건너뛰면 30 mm 목표는 못 맞춘다. C920 은 개체차가 있어서
   HFOV 로 만든 근사값으로는 화면 가장자리에서 수십 mm 씩 틀어진다.
"""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

import camera_devices as devices
import config as cfg

# 체스보드 내부 코너 개수 (칸 수 - 1). 7x7 은 8x8 칸 보드 = 보통의 체스판.
BOARD = (7, 7)
SQUARE_M = 0.035          # 한 칸의 실제 한 변 길이(m). 280 mm / 8 칸.
                          # 결과에는 영향이 없다 (맨 위 설명 참고)

# 저장 위치는 저장소 루트의 calib/ 로 고정한다.
#
# config.CALIB_DIR 은 "calib" 이라는 상대경로이고, localizer.Camera.load() 가
# 그걸 현재 작업 디렉터리 기준으로 읽는다. run_mission.py / run_localize.py 는
# 저장소 루트에서 실행하므로 읽는 쪽은 항상 <루트>/calib/ 이다. 그런데 이
# 스크립트는 aruco/ 안에 있어서, aruco/ 에서 실행하면 aruco/calib/ 에 저장돼
# 읽는 쪽과 어긋난다 — 어디서 실행하든 같은 곳에 쓰도록 루트로 고정한다.
CALIB_DIR = Path(__file__).resolve().parent.parent / cfg.CALIB_DIR


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", type=int, default=0,
                    help="C920 중 몇 번째 (0 부터). --index 로 장치 번호 직접 지정 가능")
    ap.add_argument("--index", type=int, default=None,
                    help="cv2.VideoCapture 장치 번호를 직접 지정")
    ap.add_argument("--list", action="store_true", help="카메라 목록만 보고 끝낸다")
    ap.add_argument("--board", default="7x7",
                    help="내부 코너 개수 (칸 수 - 1). 8x8 칸 체스판이면 7x7")
    ap.add_argument("--square", type=float, default=SQUARE_M)
    args = ap.parse_args()

    if args.list:
        print("이 PC 의 카메라 (번호 = cv2.VideoCapture 인덱스)")
        print(devices.describe())
        return 0

    if args.index is not None:          # 직접 지정
        index = args.index
    else:                               # 기본 - 이름으로 C920 을 찾아 --cam 번째를 쓴다
        try:
            found, _ = devices.resolve_indices(want=args.cam + 1)
        except devices.CameraNotFound as e:
            print(e)
            return 1
        index = found[args.cam]
    devices.report([index], devices.names_of([index]))
    print(f"체스판 설정: 내부 코너 {args.board} · 한 칸 {args.square*1000:.0f} mm")

    bw, bh = (int(v) for v in args.board.lower().split("x"))
    board = (bw, bh)

    objp = np.zeros((bw * bh, 3), np.float32)
    objp[:, :2] = np.mgrid[0:bw, 0:bh].T.reshape(-1, 2) * args.square

    # devices.open_camera() 를 쓴다 — 해상도뿐 아니라 오토포커스까지 꺼 준다.
    # 촬영 중 초점이 움직이면 초점거리가 같이 변해서 캘리브레이션이 무효가 되고,
    # 실행 때(run_localize.open_cams)와 같은 방식으로 열어야 초점 조건이 맞는다.
    cap = devices.open_camera(index)
    if not cap.isOpened():
        print(f"카메라 {index} 를 열 수 없습니다. 다른 프로그램이 쓰고 있지 않은지 확인하세요.")
        return 1

    obj_pts: list[np.ndarray] = []
    img_pts: list[np.ndarray] = []
    size = None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # 창을 미리 만들어 둔다. imshow 가 알아서 만들게 두면 WINDOW_AUTOSIZE 라
    # 가장자리를 끌어도 크기가 안 바뀐다. WINDOW_NORMAL 로 만들어야 조절되고,
    # KEEPRATIO 를 같이 줘야 늘려도 화면이 찌그러지지 않는다(찌그러지면 코너가
    # 제대로 잡혔는지 눈으로 판단하기 어렵다).
    win = f"calib cam{index}"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow(win, cfg.IMG_W * 3 // 4, cfg.IMG_H * 3 // 4)

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
            CALIB_DIR.mkdir(parents=True, exist_ok=True)
            path = CALIB_DIR / f"cam{index}.npz"
            np.savez(path, K=K, dist=dist, rms=rms, image_size=np.array(size))

            print(f"\n저장: {path}")
            print(f"  RMS            : {rms:.4f} px")
            print(f"  평균 재투영오차 : {np.mean(errs):.4f} px  (최대 {np.max(errs):.4f})")
            print(f"  fx, fy         : {K[0,0]:.1f}, {K[1,1]:.1f}")
            print(f"  cx, cy         : {K[0,2]:.1f}, {K[1,2]:.1f}")
            print(f"  dist           : {dist.ravel()}")

            # 화각 검산 — fx 로 역산한 수평화각이 config.HFOV_DEG 와 크게 다르면
            # 다른 카메라(또는 다른 해상도/크롭 모드)로 잡은 것이다. RMS 가 좋아도
            # 이 값이 틀리면 위치가 통째로 어긋나므로, 조용히 넘어가면 안 된다.
            hfov = math.degrees(2 * math.atan((size[0] / 2) / K[0, 0]))
            print(f"  함의 HFOV      : {hfov:.1f}°  (config.HFOV_DEG = {cfg.HFOV_DEG})")
            if abs(hfov - cfg.HFOV_DEG) > 8.0:
                print(f"  ⚠️ 화각이 {abs(hfov - cfg.HFOV_DEG):.0f}° 나 어긋납니다 — "
                      "다른 카메라를 찍었거나 해상도가 다릅니다.")
                print(f"     찍은 해상도 {size[0]}x{size[1]} 가 "
                      f"config.IMG_W/H({cfg.IMG_W}x{cfg.IMG_H}) 와 같은지 확인하세요.")

            if rms > 0.6:
                print("  ⚠️ RMS 가 큽니다. 보드를 더 다양한 각도/거리로 다시 찍으세요.")
            break

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
