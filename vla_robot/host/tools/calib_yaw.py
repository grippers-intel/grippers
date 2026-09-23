"""로봇 상판 마커의 축 보정값(`aruco.yaw_offset_deg`)을 잰다.

이 값은 **상판 마커의 로컬 +x 가 차체 정면과 이루는 각**이다. 틀리면 로봇은 "정면을
봤다"고 판단하고 직진하는데 실제로는 비스듬히 간다. 그러면 방위 오차가 계속 새로 생겨
**좌우로 흔들리며 앞으로 간다** — 2026-09-05 실기에서 본 증상이 그것이다.

## 두 가지 방법

**주행 측정(추천)** — 차를 직진시켜 *바퀴가 만든 실제 이동 방향*과 보고된 yaw 를 비교한다.
사람 판단이 안 들어간다. 차가 앞으로 움직이므로 **앞이 트인 곳에서** 실행할 것.

    python tools/calib_yaw.py --drive --pi 192.168.0.7
    python tools/calib_yaw.py --drive --pi 192.168.0.7 --runs 6 --distance 0.30

**정지 측정** — 로봇을 알려진 축(+x 또는 +y)에 맞춰 놓고 읽는다. 차가 못 움직일 때의
대체 경로다(2026-09-06 에는 배터리가 낮아 이쪽으로 잡아 85.7° 가 나왔다).

    python tools/calib_yaw.py --facing x
    python tools/calib_yaw.py --facing y

> 정지 측정은 **두 축을 다 재라.** 한 축만 재면 놓기 오차와 실제 축 어긋남을 구분할 수
> 없다. 두 축에서 같은 방향·같은 크기로 치우치면 실제 어긋남이고, 두 값의 차이가 곧
> 손으로 놓은 정확도다(2026-09-06: +x 에서 +5.06 · +y 에서 +3.52 → 차이 1.5°).

## 안전

주행 모드는 어떤 경로로 끝나든 `finally` 에서 정지를 연발한다(UDP 는 한 발이 빠진다).
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

host_config.ensure_vla_common()

from link.vehicle_link import UdpVehicleLink  # noqa: E402
from localization.aruco_localizer import (Camera, RobotLocalizer, detect,  # noqa: E402
                                          make_detector)
from localization.cameras import open_cams, read_frames, release_all  # noqa: E402
from vla_common.protocol import HostCommand, State  # noqa: E402

#: 표본이 이보다 적으면 값을 안 낸다. 10초면 80장쯤 들어오므로 20 은 "거의 안 보였다"는 뜻.
MIN_SAMPLES = 20
#: 정지한 로봇의 yaw 잡음은 1° 아래다 — 그보다 크면 마커가 잘 안 보이거나 실제로 움직인다.
MAX_JITTER_DEG = 1.5
#: 손으로 놓는 정확도가 이 수준이라, 이보다 작은 치우침은 재도 의미가 없다.
NEGLIGIBLE_DEG = 2.0
#: 이동 방향을 믿을 수 있는 최소 거리. 위치 잡음이 수 mm 라 10 cm 면 방향 오차가 1° 아래다.
MIN_TRAVEL_M = 0.10
#: 한 회차에서 yaw 가 이보다 변하면 순수 직진으로 못 본다. 2026-09-06 에 회전 관성이
#: 남은 채 전진해 155° 어긋난 회차가 나왔다. 정상 회차의 변화는 1° 아래였다.
MAX_TURN_DURING_RUN_DEG = 5.0

EXPECTED = {"x": 0.0, "y": 90.0}


def circular_mean_deg(values) -> float:
    x = sum(math.cos(math.radians(v)) for v in values)
    y = sum(math.sin(math.radians(v)) for v in values)
    return math.degrees(math.atan2(y, x))


def wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


class Eyes:
    """카메라 + 측위를 한 덩어리로 묶는다. 두 모드가 같은 경로를 쓴다."""

    def __init__(self, cfg, indices):
        self.cfg = cfg
        self.detector = make_detector(cfg.aruco)
        self.cams = [Camera.load(i, cfg.cameras, cfg.aruco) for i in indices]
        self.caps = open_cams(indices, cfg.cameras.width, cfg.cameras.height)
        self.loc = RobotLocalizer(cfg.aruco)

    def opened(self) -> bool:
        return any(c.isOpened() for c in self.caps)

    def pose(self):
        frames = read_frames(self.caps)
        dets = [{} if f is None else detect(self.detector, cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
                for f in frames]
        return self.loc.update(self.cams, dets)

    def fresh_pose(self, tries: int = 30):
        for _ in range(tries):
            p = self.pose()
            if p.ok and p.fresh:
                return p
        return None

    def close(self):
        release_all(self.caps)


# ---------------------------------------------------------------------------
# 정지 측정
# ---------------------------------------------------------------------------
def run_static(cfg, eyes: Eyes, facing: str, seconds: float) -> int:
    expect = EXPECTED[facing]
    print(f"로봇이 +{facing} 를 향한 상태로 {seconds:.0f}초 읽습니다. 차는 움직이지 않습니다.")
    yaws, xs, ys = [], [], []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        p = eyes.pose()
        if p.ok and p.fresh:
            yaws.append(p.yaw_deg)
            xs.append(p.x)
            ys.append(p.y)

    if len(yaws) < MIN_SAMPLES:
        print(f"표본 {len(yaws)}개 — 부족합니다. 로봇이 카메라에 보이는지 확인하십시오")
        return 1

    mean = circular_mean_deg(yaws)
    jitter = statistics.pstdev([wrap180(v - mean) for v in yaws])
    drift = wrap180(mean - expect)
    new = wrap180(cfg.aruco.yaw_offset_deg - drift)

    print(f"\n표본 {len(yaws)}개   위치 ({statistics.fmean(xs) * 1000:.0f}, "
          f"{statistics.fmean(ys) * 1000:.0f}) mm")
    print(f"읽힌 yaw {mean:+.2f}°   기대 {expect:+.0f}°   치우침 {drift:+.2f}°   "
          f"(흔들림 {jitter:.2f}°)")
    print(f"지금 yaw_offset_deg = {cfg.aruco.yaw_offset_deg}")
    if jitter > MAX_JITTER_DEG:
        print(f"\n⚠️ 흔들림 {jitter:.2f}° — 마커가 잘 안 보이거나 로봇이 움직입니다")
    print()
    if abs(drift) < NEGLIGIBLE_DEG:
        print("치우침이 손으로 놓는 정확도 수준입니다 — 고칠 필요 없습니다.")
    else:
        print("다른 축으로도 재서 같은 방향·같은 크기인지 확인하십시오.")
        print("한 축만으로는 놓기 오차와 실제 어긋남을 구분할 수 없습니다.")
        print(f"\n두 축이 일치하면 host.yaml 의 aruco: 블록을 이렇게 고치십시오")
        print(f"    yaw_offset_deg: {new:.1f}      # 지금 {cfg.aruco.yaw_offset_deg}")
    return 0


# ---------------------------------------------------------------------------
# 주행 측정
# ---------------------------------------------------------------------------
def _drive(link, eyes: Eyes, seconds: float, linear: float = 0.0, angular: float = 0.0) -> None:
    """속도를 그 시간 동안 계속 보낸다. Pi 워치독이 0.5 s 라 한 번만 보내면 선다."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        link.send(HostCommand(State.APPROACH, linear_x=linear, angular_z=angular))
        eyes.pose()               # 카메라를 계속 비워야 프레임이 밀리지 않는다
        time.sleep(0.05)
    link.send_stop_burst()


def run_drive(cfg, eyes: Eyes, pi_ip: str, runs: int, distance: float, speed: float) -> int:
    link = UdpVehicleLink(pi_ip, cfg.link.command_port, cfg.link.status_port,
                          bind_ip=cfg.link.bind_ip, stop_burst=cfg.link.stop_burst)
    seconds = max(distance / max(speed, 0.01), 1.0)
    print(f"{runs}회 × 약 {distance * 100:.0f} cm 직진합니다. 로봇 앞을 비우십시오.")
    print("각 회차 사이에 제자리에서 조금 돌려 여러 방향에서 잽니다.\n")
    offsets = []
    try:
        for k in range(1, runs + 1):
            before = eyes.fresh_pose()
            if before is None:
                print(f"[{k}/{runs}] pose 를 못 잡았습니다 — 건너뜁니다")
                continue
            _drive(link, eyes, seconds, linear=speed)
            time.sleep(0.5)                      # 관성이 멎기를 기다린다
            after = eyes.fresh_pose()
            if after is None:
                print(f"[{k}/{runs}] 이동 후 pose 를 못 잡았습니다 — 건너뜁니다")
                continue
            dx, dy = after.x - before.x, after.y - before.y
            travel = math.hypot(dx, dy)
            turned = abs(wrap180(after.yaw_deg - before.yaw_deg))
            if travel < MIN_TRAVEL_M:
                print(f"[{k}/{runs}] {travel * 100:.1f} cm 밖에 안 갔습니다 — 버립니다 "
                      f"(배터리·바닥 확인)")
                continue
            if turned > MAX_TURN_DURING_RUN_DEG:
                print(f"[{k}/{runs}] 도중에 yaw 가 {turned:.1f}° 변했습니다 — 버립니다")
                continue
            moved_deg = math.degrees(math.atan2(dy, dx))
            off = wrap180(moved_deg - before.yaw_deg)
            offsets.append(off)
            print(f"[{k}/{runs}] {travel * 100:5.1f} cm  이동방향 {moved_deg:+7.2f}°  "
                  f"보고 yaw {before.yaw_deg:+7.2f}°  차이 {off:+6.2f}°")
            if k < runs:
                _drive(link, eyes, 2.0, angular=cfg.drive.rotation_rad_s)
                time.sleep(0.5)
    finally:
        link.send_stop_burst()
        link.close()

    if len(offsets) < 2:
        print("\n쓸 만한 회차가 2번을 못 넘었습니다 — 값을 내지 않습니다")
        return 1
    mean = circular_mean_deg(offsets)
    spread = statistics.pstdev([wrap180(v - mean) for v in offsets])
    new = wrap180(cfg.aruco.yaw_offset_deg + mean)
    print(f"\n회차 {len(offsets)}개   평균 차이 {mean:+.2f}°   회차별 편차 {spread:.2f}°")
    print(f"지금 yaw_offset_deg = {cfg.aruco.yaw_offset_deg}")
    if spread > 3.0:
        print(f"\n⚠️ 편차 {spread:.2f}° — 축 보정이 아니라 다른 문제일 수 있습니다"
              " (한쪽 바퀴 미끄러짐·바닥 기울기)")
    if abs(mean) < NEGLIGIBLE_DEG:
        print("\n치우침이 작습니다 — 고칠 필요 없습니다.")
    else:
        print(f"\nhost.yaml 의 aruco: 블록을 이렇게 고치십시오")
        print(f"    yaw_offset_deg: {new:.1f}      # 지금 {cfg.aruco.yaw_offset_deg}")
    return 0


def main() -> int:
    host_config.configure_console()
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--cams", type=int, nargs="+", default=None)
    ap.add_argument("--facing", choices=sorted(EXPECTED), help="정지 측정: 로봇 정면이 향한 축")
    ap.add_argument("--seconds", type=float, default=10.0, help="정지 측정 시간")
    ap.add_argument("--drive", action="store_true", help="주행 측정(추천)")
    ap.add_argument("--pi", default="192.168.0.7", help="주행 측정: Pi 주소")
    ap.add_argument("--runs", type=int, default=4)
    ap.add_argument("--distance", type=float, default=0.25, help="회차당 이동 거리(m)")
    args = ap.parse_args()

    if args.drive == bool(args.facing):
        print("--drive 또는 --facing x|y 중 하나를 주십시오")
        return 2

    cfg = host_config.load_host_config(args.config)
    indices = args.cams if args.cams is not None else list(cfg.cameras.indices)
    eyes = Eyes(cfg, indices)
    if not eyes.opened():
        print("열린 카메라가 없습니다")
        eyes.close()
        return 1
    try:
        if args.drive:
            return run_drive(cfg, eyes, args.pi, args.runs, args.distance,
                             cfg.drive.linear_mps)
        return run_static(cfg, eyes, args.facing, args.seconds)
    finally:
        eyes.close()


if __name__ == "__main__":
    sys.exit(main())
