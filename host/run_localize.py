"""웹캠 2대로 로봇 절대 위치·자세를 실시간 추정한다.

사용법
    python run_localize.py                    # 카메라 0, 1 사용
    python run_localize.py --cams 0 2
    python run_localize.py --log run.csv      # CSV 기록 + 종료 시 통계
    python run_localize.py --no-view          # 창 없이 콘솔만
    python run_localize.py --no-geti          # geti 객체 인식 끄고 ArUco만

카메라 창에는 ArUco 로봇 추적과 geti_sdk-deployment 객체 인식(별,
축구공, 상자, 체스말 등)이 함께 그려진다. geti 추론은 CPU 로 돌리면
프레임 하나에 ~0.8초 걸리므로(RTDetr, 1280x720 실측), ArUco 추적이
느려지지 않도록 카메라별 백그라운드 스레드에서 돌고, 화면에는 그
시점까지 나온 가장 최근 결과가 그려진다(geti_detector.py 참고).

종료: 창에서 q, 또는 Ctrl+C

--log 로 정지 상태를 30초쯤 기록하면 종료할 때 표준편차와 최대편차가 찍힌다.
#175 완료 조건(정지 오차 <= 30 mm, yaw <= 3 deg) 확인에 그대로 쓰면 된다.
"""

import argparse
import csv
import signal
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# config.py/localizer.py 는 aruco/ 하위폴더에 있다(팀원이 동기화하는 파일이라
# 건드리지 않는다) — 그 폴더를 경로에 추가해서 기존처럼 bare import 로 쓴다.
sys.path.insert(0, str(Path(__file__).parent / "aruco"))

import config as cfg
from localizer import Camera, Pose, RobotLocalizer, detect, make_detector

try:
    import geti_detector
    _GETI_IMPORT_ERROR = None
except Exception as _exc:      # geti_sdk 미설치 등 — --no-geti 없이도 그냥 꺼지게
    geti_detector = None
    _GETI_IMPORT_ERROR = _exc

_stop = False


def _on_sigint(signum, frame):
    global _stop
    _stop = True


def open_cams(indices: list[int]) -> list[cv2.VideoCapture]:
    caps = []
    for i in indices:
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.IMG_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.IMG_H)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)   # C920 오토포커스 끄기 — 캘리브레이션 때 고정한 초점과 어긋나면 안 됨
        if not cap.isOpened():
            print(f"⚠️ 카메라 {i} 를 열 수 없습니다.")
        caps.append(cap)
    return caps


def draw(frame, cam: Camera, det, pose: Pose):
    """검출 상태를 화면에 그린다."""
    for mid, c in det.items():
        pts = c.astype(np.int32)
        color = (0, 165, 255) if mid == cfg.ROBOT_MARKER_ID else (0, 255, 0)
        cv2.polylines(frame, [pts], True, color, 2)
        cv2.putText(frame, str(mid), tuple(pts[0]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    lock = "LOCKED" if cam.locked else (
        f"locking {cam._acc_n}/{cfg.EXTRINSIC_LOCK_FRAMES}"
        if cfg.EXTRINSIC_LOCK_FRAMES else "live")
    lines = [
        f"{cam.name}  calib={'OK' if cam.calibrated else 'APPROX'}"
        f"  floor={cam.n_floor}/4  reproj={cam.reproj_px:.2f}px  [{lock}]",
    ]
    color = (255, 255, 255)
    if pose.ok:
        lines.append(f"x={pose.x*1000:.0f}mm y={pose.y*1000:.0f}mm "
                     f"yaw={pose.yaw_deg:.1f}deg cams={pose.n_cams}"
                     + ("" if pose.fresh else f" HOLD {pose.age_s:.2f}s"))
        if not cfg.in_workspace(pose.x, pose.y):
            # cv2.putText 는 한글을 못 그린다 — 화면 글자는 반드시 영문만
            lines.append("OUT OF WORKSPACE (blind strip near wall)")
            color = (0, 200, 255)
    else:
        lines.append("ROBOT LOST")
        color = (0, 0, 255)
    for k, s in enumerate(lines):
        cv2.putText(frame, s, (12, 28 + 26 * k), cv2.FONT_HERSHEY_SIMPLEX,
                    0.62, color if k else (255, 255, 255), 2)
    return frame


def summarize(rows: list[tuple]) -> None:
    """정지 상태 기록에서 산포를 계산해 완료 조건과 비교한다."""
    fresh = [r for r in rows if r[5]]     # fresh 인 샘플만
    if len(fresh) < 10:
        print("\n통계를 내기에 표본이 부족합니다.")
        return
    x = np.array([r[1] for r in fresh]) * 1000.0
    y = np.array([r[2] for r in fresh]) * 1000.0
    yaw = np.radians([r[3] for r in fresh])
    yaw_mean = np.arctan2(np.sin(yaw).mean(), np.cos(yaw).mean())
    dyaw = np.degrees((yaw - yaw_mean + np.pi) % (2 * np.pi) - np.pi)
    r = np.hypot(x - x.mean(), y - y.mean())

    print(f"\n{'='*58}\n정지 산포 (표본 {len(fresh)})\n{'='*58}")
    print(f"  평균 위치     : ({x.mean():.1f}, {y.mean():.1f}) mm")
    print(f"  위치 표준편차 : {r.std():.1f} mm   최대편차 {r.max():.1f} mm")
    print(f"  yaw 표준편차  : {dyaw.std():.2f} deg  최대편차 {np.abs(dyaw).max():.2f} deg")
    print(f"\n  완료조건 위치 <= 30 mm : "
          f"{'PASS' if r.max() <= 30 else 'FAIL'} (최대편차 {r.max():.1f} mm)")
    print(f"  완료조건 yaw  <=  3 deg: "
          f"{'PASS' if np.abs(dyaw).max() <= 3 else 'FAIL'} "
          f"(최대편차 {np.abs(dyaw).max():.2f} deg)")
    print("\n  ※ 이건 '흔들림(반복도)'이지 '정확도'가 아니다. 절대 오차는 로봇을")
    print("    바닥에 표시한 기지 좌표에 세우고 평균값과 비교해서 따로 확인할 것.")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cams", type=int, nargs="+", default=list(cfg.CAM_INDICES))
    ap.add_argument("--log", type=str, default=None)
    ap.add_argument("--no-view", action="store_true")
    ap.add_argument("--no-geti", action="store_true",
                     help="geti 객체 인식을 끄고 ArUco 추적만 한다")
    ap.add_argument("--geti-device", type=str, default="CPU",
                     help="geti 추론 장치 (CPU/GPU 등, 기본 CPU)")
    args = ap.parse_args()

    signal.signal(signal.SIGINT, _on_sigint)

    detector = make_detector()
    cams = [Camera.load(f"cam{i}", i) for i in args.cams]
    for c in cams:
        if not c.calibrated:
            print(f"⚠️ {c.name}: calib/cam*.npz 가 없어 HFOV {cfg.HFOV_DEG}° 근사값을 씁니다. "
                  f"정확도 목표를 맞추려면 calibrate_camera.py 를 먼저 돌리세요.")

    caps = open_cams(args.cams)
    if not any(c.isOpened() for c in caps):
        print("\n열린 카메라가 하나도 없습니다. --cams 로 인덱스를 바꿔 보세요.")
        print("연결만 먼저 확인하려면: python -c \"import cv2;"
              " print([i for i in range(6)"
              " if cv2.VideoCapture(i, cv2.CAP_DSHOW).isOpened()])\"")
        for c in caps:
            c.release()
        return 1

    if not args.no_view:
        for cam in cams:
            cv2.namedWindow(cam.name, cv2.WINDOW_NORMAL)

    loc = RobotLocalizer()

    geti_workers: list = [None] * len(cams)
    if not args.no_geti and not args.no_view:
        if geti_detector is None:
            print(f"⚠️ geti_sdk 를 불러올 수 없어 객체 인식을 끕니다: {_GETI_IMPORT_ERROR}")
        else:
            print(f"geti 모델 불러오는 중 ({args.geti_device}, 카메라당 1개)...")
            # 카메라마다 별도 Deployment 인스턴스를 준다 — 하나를 공유하면 두 배경
            # 스레드가 동시에 infer() 를 불러서 "Infer Request is busy" 오류가 난다.
            geti_workers = [geti_detector.GetiWorker(
                geti_detector.load_deployment(device=args.geti_device), c.name) for c in cams]
            print("geti 모델 준비 완료 — 카메라별로 백그라운드에서 추론합니다.")

    rows: list[tuple] = []
    writer = fh = None
    if args.log:
        fh = open(args.log, "w", newline="", encoding="utf-8")
        writer = csv.writer(fh)
        writer.writerow(["t", "x_m", "y_m", "yaw_deg", "n_cams", "fresh", "age_s"])

    t0 = time.monotonic()
    frames_seen = 0
    print("시작 — q 또는 Ctrl+C 로 종료\n")

    try:
        while not _stop:
            grabbed, dets = [], []
            for cap in caps:
                ok, frame = cap.read()
                grabbed.append(frame if ok else None)
                dets.append({} if not ok else
                            detect(detector, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)))

            pose = loc.update(cams, dets)
            frames_seen += 1
            t = time.monotonic() - t0
            row = (t, pose.x, pose.y, pose.yaw_deg, pose.n_cams, pose.fresh, pose.age_s)
            rows.append(row)
            if writer:
                writer.writerow([f"{t:.4f}", f"{pose.x:.5f}", f"{pose.y:.5f}",
                                 f"{pose.yaw_deg:.3f}", pose.n_cams,
                                 int(pose.fresh), f"{pose.age_s:.3f}"])

            if frames_seen % 10 == 0:
                print(f"\r{pose}   ", end="", flush=True)

            if not args.no_view:
                for cam, frame, det, worker in zip(cams, grabbed, dets, geti_workers):
                    if frame is None:
                        continue
                    disp = frame
                    if worker is not None:
                        # 복사본을 넘긴다 — 아래에서 draw() 가 frame 을 제자리에서
                        # 칠하는 동안 백그라운드 스레드가 같은 배열을 읽으면 안 됨
                        worker.submit(frame.copy())
                        pred = worker.latest()
                        if pred is not None:
                            disp = geti_detector.draw(disp, pred)
                    cv2.imshow(cam.name, draw(disp, cam, det, pose))
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
    finally:
        for worker in geti_workers:
            if worker is not None:
                worker.stop()
        for cap in caps:
            cap.release()
        cv2.destroyAllWindows()
        if fh:
            fh.close()
            print(f"\n로그 저장: {args.log}")
        summarize(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
