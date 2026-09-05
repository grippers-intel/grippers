#!/usr/bin/env python3
"""정지 상태로 마커 축 보정값(YAW_OFFSET_DEG)을 잰다 — 차를 안 움직인다.

## 언제 쓰나

`calib_yaw_axis.py` 가 더 정확하다. 그쪽은 로봇을 직진시켜 **바퀴가 만든 실제
이동 방향**과 보고된 yaw 를 비교하므로 사람 판단이 안 들어간다. 그걸 못 쓸
때의 대체 경로가 이 파일이다 — 2026-09-06 에는 배터리가 낮아 차가 아예 못
움직여서 이쪽으로 잡았다.

## 원리

로봇을 알려진 방향(+x 또는 +y)으로 놓으면 yaw 가 그 방향의 각도로 읽혀야
한다. 어긋난 만큼이 보정량이다.

    +x 를 향함  ->  yaw 는 0 이어야 한다
    +y 를 향함  ->  yaw 는 +90 이어야 한다

    새 YAW_OFFSET_DEG = 지금 값 - (읽힌 yaw - 기대 각도)

## 두 축을 다 재라

한 축만 재면 **놓기 오차와 실제 축 어긋남을 구분할 수 없다.** 두 축에서
같은 방향으로 같은 크기만큼 치우치면 그건 실제 어긋남이고, 두 값이 다르면
그 차이가 놓기 오차의 크기다.

2026-09-06 실측이 그 예다. +x 에서 +5.06, +y 에서 +3.52 — 둘 다 양수라
실제 어긋남으로 판정했고, 차이 1.5도가 손으로 놓은 정확도였다.

## 정확도

사람이 로봇을 축에 맞춰 놓은 것이 기준이다. 눈으로 맞추면 3~5도는 쉽게
틀어지므로, 곧은 기준(벽·테이블 모서리·바닥 테이프)에 차체를 대고 맞출 것.

사용법
    python calib_yaw_static.py --facing x      # 로봇이 +x 를 향한 상태
    python calib_yaw_static.py --facing y      # 로봇이 +y 를 향한 상태
    python calib_yaw_static.py --facing x --seconds 20
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).parent / "aruco"))

import config as cfg
from localizer import Camera, RobotLocalizer, detect, make_detector

from run_localize import open_cams

#: 표본이 이보다 적으면 값을 안 낸다. 10초에 80장쯤 들어오므로 20 은
#: "로봇이 거의 안 보였다"는 뜻이다.
MIN_SAMPLES = 20

#: 흔들림이 이보다 크면 경고한다(도). 정지한 로봇의 yaw 잡음은 1도 아래다 —
#: 그보다 크면 마커가 잘 안 보이거나 로봇이 실제로 움직이고 있다.
MAX_JITTER_DEG = 1.5

#: 보정량이 이보다 작으면 고치지 않는다(도). 손으로 놓는 정확도가 이 수준이라,
#: 더 작은 차이는 재도 의미가 없다.
NEGLIGIBLE_DEG = 2.0

EXPECTED = {"x": 0.0, "y": 90.0}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--facing", choices=sorted(EXPECTED), required=True,
                    help="로봇 정면이 향한 축. +x 면 x, +y 면 y")
    ap.add_argument("--cams", type=int, nargs="+", default=list(cfg.CAM_INDICES))
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()

    expect = EXPECTED[args.facing]
    detector = make_detector()
    cams = [Camera.load(f"cam{i}", i) for i in args.cams]
    caps = open_cams(args.cams)
    if not any(c.isOpened() for c in caps):
        print("열린 카메라가 없습니다")
        return 1
    loc = RobotLocalizer()

    print(f"로봇이 +{args.facing} 를 향한 상태로 {args.seconds:.0f}초 읽습니다. "
          "차는 움직이지 않습니다.")
    yaws, xs, ys = [], [], []
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            dets = []
            for cap in caps:
                ok, frame = cap.read()
                dets.append({} if not ok else
                            detect(detector, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)))
            pose = loc.update(cams, dets)
            if pose.ok:
                yaws.append(pose.yaw_deg)
                xs.append(pose.x)
                ys.append(pose.y)
    finally:
        for c in caps:
            c.release()

    if len(yaws) < MIN_SAMPLES:
        print(f"표본 {len(yaws)}개 — 부족합니다. 로봇이 두 카메라에 보이는지 보십시오")
        return 1

    # 각도는 원형 평균으로 낸다 — ±180 근처에서 산술평균은 엉뚱한 값이 된다.
    sx = sum(math.cos(math.radians(v)) for v in yaws)
    sy = sum(math.sin(math.radians(v)) for v in yaws)
    mean = math.degrees(math.atan2(sy, sx))
    jitter = statistics.pstdev([(v - mean + 180.0) % 360.0 - 180.0 for v in yaws])
    drift = (mean - expect + 180.0) % 360.0 - 180.0
    new = (cfg.YAW_OFFSET_DEG - drift + 180.0) % 360.0 - 180.0

    print(f"\n표본 {len(yaws)}개   위치 ({statistics.fmean(xs)*1000:.0f}, "
          f"{statistics.fmean(ys)*1000:.0f})mm")
    print(f"읽힌 yaw {mean:+.2f}도   기대 {expect:+.0f}도   "
          f"치우침 {drift:+.2f}도   (흔들림 {jitter:.2f}도)")
    print(f"지금 YAW_OFFSET_DEG = {cfg.YAW_OFFSET_DEG}")
    if jitter > MAX_JITTER_DEG:
        print(f"\n⚠️ 흔들림 {jitter:.2f}도 — 마커가 잘 안 보이거나 로봇이 움직입니다")
    print()
    if abs(drift) < NEGLIGIBLE_DEG:
        print("치우침이 손으로 놓는 정확도 수준입니다 — 고칠 필요 없습니다.")
    else:
        print("다른 축으로도 재서 같은 방향·같은 크기인지 확인하십시오.")
        print("한 축만으로는 놓기 오차와 실제 어긋남을 구분할 수 없습니다.")
        print(f"\n두 축이 일치하면 config.py 를 이렇게 고치십시오:")
        print(f"    YAW_OFFSET_DEG = {new:.1f}      # 지금 {cfg.YAW_OFFSET_DEG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
