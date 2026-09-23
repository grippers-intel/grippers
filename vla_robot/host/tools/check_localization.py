"""로봇 pose 가 실제로 잡히는지, 얼마나 흔들리는지 잰다 — 미션 없이 측위만 돌린다.

    python tools/check_localization.py                 # 10초 재고 통계
    python tools/check_localization.py --seconds 30    # 더 길게
    python tools/check_localization.py --show          # 카메라 창도 띄운다

로봇을 **세워 둔 채로** 돌린다. 정지한 로봇의 위치·각도가 얼마나 떨리는지가 곧
측위 품질이고, 그 값이 주행 제어의 바닥 잡음이 된다.

## 나오는 값과 판정

| 값 | 좋음 | 뜻 |
|---|---|---|
| 재투영오차 | < 1 px | 마커 좌표와 실제 배치가 맞는다 |
| 위치 흔들림 | < 5 mm | 이보다 크면 마커가 작게 잡히거나 초점이 안 맞는다 |
| yaw 흔들림 | < 1.0° | 정지 로봇 기준. 크면 상판 마커가 흔들리거나 반사가 있다 |
| 관측 카메라 수 | 2 | 1 이면 한쪽 시야를 벗어났다는 뜻이다 |
| 놓침 비율 | 0 % | 놓치면 Host 는 stop 을 보낸다(`pose_hold_s` 만큼만 버틴다) |

⚠️ 이 도구는 로봇을 움직이지 않는다. yaw **보정값**은 `tools/calib_yaw.py` 로 잰다.
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from pathlib import Path

import cv2

HOST_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST_ROOT))

import host_config  # noqa: E402
from localization.aruco_localizer import (Camera, RobotLocalizer, detect,  # noqa: E402
                                          draw_overlay, make_detector)
from localization.cameras import open_cams, read_frames, release_all  # noqa: E402


def circular_mean_deg(values) -> float:
    """±180 근처에서 산술평균은 엉뚱한 값이 된다 — 각도는 원형 평균으로 낸다."""
    x = sum(math.cos(math.radians(v)) for v in values)
    y = sum(math.sin(math.radians(v)) for v in values)
    return math.degrees(math.atan2(y, x))


def circular_spread_deg(values, mean: float) -> float:
    return statistics.pstdev([(v - mean + 180.0) % 360.0 - 180.0 for v in values]) if len(values) > 1 else 0.0


def main() -> int:
    host_config.configure_console()
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--cams", type=int, nargs="+", default=None)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--show", action="store_true", help="카메라 오버레이 창을 띄운다")
    args = ap.parse_args()

    cfg = host_config.load_host_config(args.config)
    indices = args.cams if args.cams is not None else list(cfg.cameras.indices)
    detector = make_detector(cfg.aruco)
    cams = [Camera.load(i, cfg.cameras, cfg.aruco) for i in indices]
    caps = open_cams(indices, cfg.cameras.width, cfg.cameras.height)
    if not any(c.isOpened() for c in caps):
        print("열린 카메라가 없습니다")
        release_all(caps)
        return 1

    loc = RobotLocalizer(cfg.aruco)
    xs, ys, yaws, cams_seen = [], [], [], []
    frames_total = 0
    print(f"{args.seconds:.0f}초 동안 읽습니다. 로봇을 움직이지 마십시오 "
          f"(q 로 중단)")
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            frames = read_frames(caps)
            dets = [{} if f is None else detect(detector, cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
                    for f in frames]
            pose = loc.update(cams, dets)
            frames_total += 1
            if pose.ok and pose.fresh:
                xs.append(pose.x)
                ys.append(pose.y)
                yaws.append(pose.yaw_deg)
                cams_seen.append(pose.n_cams)
            if args.show:
                for cam, f, d in zip(cams, frames, dets):
                    if f is not None:
                        cv2.imshow(cam.name, draw_overlay(f, cam, d, pose,
                                                          cfg.aruco.robot_marker_id))
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
    finally:
        release_all(caps)
        if args.show:
            cv2.destroyAllWindows()

    print()
    for cam in cams:
        lock = "LOCKED" if cam.locked else f"locking {cam.acc_n}/{cfg.aruco.extrinsic_lock_frames}"
        rp = "--" if cam.reproj_px == float("inf") else f"{cam.reproj_px:.2f} px"
        print(f"{cam.name}: 바닥 마커 {cam.n_floor}/{len(cfg.aruco.floor_markers)} · "
              f"재투영 {rp} · {lock} · "
              f"{'캘리브레이션 있음' if cam.calibrated else '⚠️ 근사 내부파라미터'}")

    if not xs:
        print("\n로봇 pose 를 한 번도 못 잡았습니다 — tools/place_markers.py 로 마커부터 확인하십시오")
        return 1

    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    myaw = circular_mean_deg(yaws)
    jx = statistics.pstdev(xs) * 1000 if len(xs) > 1 else 0.0
    jy = statistics.pstdev(ys) * 1000 if len(ys) > 1 else 0.0
    jyaw = circular_spread_deg(yaws, myaw)
    miss = 100.0 * (1.0 - len(xs) / max(frames_total, 1))

    print(f"\n표본 {len(xs)} / 프레임 {frames_total} (놓침 {miss:.1f} %)")
    print(f"위치   x {mx * 1000:7.1f} mm ± {jx:.1f}   y {my * 1000:7.1f} mm ± {jy:.1f}")
    print(f"각도   yaw {myaw:+7.2f}° ± {jyaw:.2f}   (yaw_offset_deg {cfg.aruco.yaw_offset_deg} 적용 후)")
    print(f"관측   카메라 {statistics.fmean(cams_seen):.2f} 대 평균")

    print()
    if max(jx, jy) > 5.0:
        print(f"⚠️ 위치 흔들림 {max(jx, jy):.1f} mm — 마커가 작게 잡히거나 초점이 안 맞습니다")
    if jyaw > 1.0:
        print(f"⚠️ yaw 흔들림 {jyaw:.2f}° — 상판 마커 반사나 떨림을 보십시오")
    if miss > 1.0:
        print(f"⚠️ 놓침 {miss:.1f} % — 주행 중에는 Host 가 그때마다 정지합니다")
    if statistics.fmean(cams_seen) < 1.5:
        print("⚠️ 한 대만 보고 있습니다 — 두 대가 겹쳐 보는 영역에서 재야 값이 안정됩니다")
    if max(jx, jy) <= 5.0 and jyaw <= 1.0 and miss <= 1.0:
        print("측위 품질 이상 없음")
    return 0


if __name__ == "__main__":
    sys.exit(main())
