#!/usr/bin/env python3
"""로봇 마커의 축 보정값(YAW_OFFSET_DEG)을 실제 주행으로 잰다.

## 왜 필요한가

`config.YAW_OFFSET_DEG` 는 **로봇 상판 마커가 차체 정면과 이루는 각**이다.
이 값이 틀리면 로봇은 "정면을 봤다"고 판단하고 직진하는데 실제로는 비스듬히
간다. 그러면 방위 오차가 계속 새로 생겨 **좌우로 흔들리며 앞으로 간다** —
2026-09-05 실기에서 본 증상이 그것이다.

## 기존 절차와 무엇이 다른가

사용법.txt 9단계는 이렇게 시킨다.

    (가) 로봇을 오른쪽(x가 커지는 쪽)을 향하게 똑바로 놓는다
    (나) 화면의 yaw 를 읽는다 (= A)
    (다) YAW_OFFSET_DEG 를 90 - A 로 고친다

**사람이 로봇을 축에 맞춰 놓는 것**이 기준이라, 눈으로 맞춘 각도 오차가 그대로
상수에 들어간다. 몇 도는 쉽게 틀어지고, 그 몇 도가 주행 내내 따라다닌다.

이 도구는 사람이 각도를 맞추지 않는다. **로봇을 직진시키고 위치가 실제로 어느
방향으로 움직였는지**를 본다. 바퀴가 만든 방향이 곧 차체 정면이므로, 그것과
보고된 yaw 의 차이가 찾는 값이다. 사람은 "앞이 트인 곳에 두는 것"만 하면 된다.

    새 YAW_OFFSET_DEG = 지금 값 + (실제 이동 방향 - 보고된 yaw)

## 왜 여러 번 재는가

한 번의 이동 방향은 ArUco 위치 잡음(수 mm)과 바닥 미끄러짐에 흔들린다. 로봇을
여러 방향으로 돌려가며 재고 원형 평균을 낸다. 회차별 편차가 크면 그건 축
보정이 아니라 다른 문제라는 뜻이므로, 편차도 같이 보여 준다.

## 안전

로봇이 **앞으로 움직인다**(기본 25cm x 회차). 앞이 트인 곳에서 실행할 것.
어떤 경로로 끝나든 finally 에서 정지를 여러 번 보낸다.

사용법
    python calib_yaw_axis.py --vehicle-ip 192.168.0.7
    python calib_yaw_axis.py --vehicle-ip 192.168.0.7 --runs 6 --distance 0.3
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
from vehicle_link import MissionCommand, UdpVehicleLink

#: 각 회차 사이에 로봇을 이만큼 돌린다. 여러 방향에서 재야 한 방향에만 있는
#: 편향(바닥 기울기, 한쪽 바퀴 미끄러짐)이 평균에서 드러난다.
TURN_BETWEEN_RUNS_S = 2.0

#: 이동 방향을 믿을 수 있는 최소 거리. 위치 잡음이 수 mm 이므로 10cm 만
#: 움직여도 방향 오차는 1도 아래다. 그보다 짧으면 잡음이 방향을 지배한다.
MIN_TRAVEL_M = 0.10

#: 한 회차 안에서 yaw 가 이보다 많이 변하면 순수 직진으로 못 본다(도).
#: 2026-09-06 에 회전 관성이 남은 채 전진해 155도 어긋난 회차가 나왔다.
#: 정상 회차의 yaw 변화는 1도 아래였으므로 5도면 넉넉하다.
MAX_TURN_DURING_RUN_DEG = 5.0


def _pose(loc, cams, caps, detector, tries: int = 30):
    """신선한 pose 하나. 못 얻으면 None."""
    for _ in range(tries):
        dets = []
        for cap in caps:
            ok, frame = cap.read()
            dets.append({} if not ok else
                        detect(detector, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)))
        pose = loc.update(cams, dets)
        if pose.ok:
            return pose
        time.sleep(0.05)
    return None


def _drive(link, loc, cams, caps, detector, cmd: str, seconds: float):
    """cmd 를 seconds 동안 보내면서 pose 를 계속 갱신한다.

    ⚠️ 상태 문자열은 **Host 내부 이름**(mission.State)이어야 한다. Pi 쪽 이름
    ("APPROACH")을 넘기면 vehicle_link._STATE_TO_PI 에 없어서 안전값
    IDLE + stop=True 로 떨어진다 — 전진 명령이 정지로 바뀌어 나가고, 밖에서
    보면 "명령은 보내는데 차가 안 움직인다"로만 보인다(2026-09-06 실기에서
    이걸로 한참 헤맸다). 여기서 쓰는 것은 APPROACH_PIECE 다.

    ⚠️ **pose 를 얻었는지와 무관하게 매 사이클 보낸다.** 처음에는 pose 가 있는
    사이클에만 보냈는데, 그러면 마커를 놓칠 때마다 명령이 끊기고 Pi 워치독이
    0.3초 만에 세운다 — 2026-09-05 첫 실행에서 4회 중 3회가 0.5cm 도 못
    움직였다. 좌표는 화면 표시용 참고 필드라(VEHICLE_LINK_PROTOCOL) 조금
    낡아도 되고, 정지하지 않는 것이 훨씬 중요하다.

    (관측 사이클 수, pose 를 얻은 사이클 수) 를 돌려준다 — 이 비율이 낮으면
    측정을 믿을 수 없고, 그 자체가 진단이다.
    """
    end = time.monotonic() + seconds
    last = None
    cycles = hits = 0
    tick = 0
    while time.monotonic() < end:
        # ⚠️ 송신을 카메라 읽기와 붙여 두면 안 된다. 카메라 2대를 읽는 데
        # 60~100ms 가 들어 실제 송신이 ~10Hz 로 떨어지는데, Pi 오케스트레이터의
        # 주기도 10Hz 라 지터로 세 번에 한 번은 새 명령을 못 찾는다. 그러면
        # 워치독이 정지를 내고, 모터가 초당 여러 번 끊겨 정지마찰을 못 뜯는다
        # — 2026-09-06 실측: /cmd_vel 의 34%, set_motor 의 43% 가 0 이었고
        # 차는 6초 동안 0mm 를 갔다.
        #
        # 그래서 송신은 50Hz 로 돌리고, pose 는 다섯 번에 한 번만 읽는다.
        # 좌표는 화면 표시용 참고 필드라 조금 낡아도 된다.
        tick += 1
        if tick % 5 == 1:
            cycles += 1
            pose = _pose(loc, cams, caps, detector, tries=1)
            if pose is not None:
                last, hits = pose, hits + 1
        link.send(MissionCommand(cmd, "APPROACH_PIECE",
                                 last.x if last else 0.0,
                                 last.y if last else 0.0,
                                 last.yaw_deg if last else 0.0))
        time.sleep(0.02)
    for _ in range(8):
        link.send(MissionCommand("stop", "APPROACH_PIECE",
                                 last.x if last else 0.0,
                                 last.y if last else 0.0,
                                 last.yaw_deg if last else 0.0))
        time.sleep(0.05)
    return cycles, hits


def _wait_still(loc, cams, caps, detector, max_s: float = 4.0,
                still_deg: float = 0.8) -> None:
    """yaw 가 멈출 때까지 기다린다.

    ⚠️ 회전 직후 고정 시간만 쉬면 부족하다. 2026-09-06 실측에서 도는 도중에
    전진이 시작돼 이동 방향이 yaw 와 155도까지 어긋난 회차가 나왔다 — 이
    도구는 "차가 자기 정면으로 갔다"를 전제하므로 그 회차는 통째로 못 쓴다.

    팀원 주행 코드의 ESCAPE 모드도 같은 이유로 위험하다(정렬을 무시한 전진 —
    navigator.DriveMode.ESCAPE 주석). 여기서는 우리가 직접 명령을 내므로
    ESCAPE 는 안 걸리지만, 회전 관성은 그대로 남는다.
    """
    end = time.monotonic() + max_s
    prev = None
    while time.monotonic() < end:
        pose = _pose(loc, cams, caps, detector, tries=1)
        if pose is not None:
            if prev is not None:
                d = abs((pose.yaw_deg - prev + 180.0) % 360.0 - 180.0)
                if d < still_deg:
                    return
            prev = pose.yaw_deg
        time.sleep(0.15)


def _circular_mean(degs: list[float]) -> float:
    x = sum(math.cos(math.radians(d)) for d in degs)
    y = sum(math.sin(math.radians(d)) for d in degs)
    return math.degrees(math.atan2(y, x))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle-ip", required=True)
    ap.add_argument("--cams", type=int, nargs="+", default=list(cfg.CAM_INDICES))
    ap.add_argument("--runs", type=int, default=4)
    ap.add_argument("--distance", type=float, default=0.25,
                    help="회차당 전진 거리(m). 합의 속도 0.1m/s 로 환산해 시간을 정한다")
    args = ap.parse_args()

    detector = make_detector()
    cams = [Camera.load(f"cam{i}", i) for i in args.cams]
    caps = open_cams(args.cams)
    if not any(c.isOpened() for c in caps):
        print("열린 카메라가 없습니다")
        return 1
    loc = RobotLocalizer()
    link = UdpVehicleLink(args.vehicle_ip)

    print(f"지금 config.YAW_OFFSET_DEG = {cfg.YAW_OFFSET_DEG}")
    print(f"{args.runs}회 x {args.distance*100:.0f}cm 전진합니다. 앞을 비워 두십시오.\n")

    seconds = args.distance / 0.1     # AGREED_LINEAR_MPS
    deltas = []
    try:
        for run in range(args.runs):
            before = _pose(loc, cams, caps, detector)
            if before is None:
                print(f"[{run+1}] 로봇을 못 봅니다 — 건너뜁니다")
                continue
            cycles, hits = _drive(link, loc, cams, caps, detector, "go", seconds)
            time.sleep(0.6)           # 관성으로 미끄러지는 동안 기다린다
            after = _pose(loc, cams, caps, detector)
            if after is None:
                print(f"[{run+1}] 이동 뒤 로봇을 못 봅니다 — 건너뜁니다")
                continue

            dx, dy = after.x - before.x, after.y - before.y
            travel = math.hypot(dx, dy)
            rate = hits / cycles if cycles else 0.0
            if travel < MIN_TRAVEL_M:
                print(f"[{run+1}] {travel*100:.1f}cm 밖에 안 움직였습니다 — 버립니다 "
                      f"(주행 중 pose 획득률 {rate*100:.0f}%)")
                continue
            # 명령한 것보다 훨씬 많이 움직였으면 측위가 튄 것이다. 그 값으로
            # 방향을 내면 엉뚱한 보정값이 나온다.
            if travel > args.distance * 2.5:
                print(f"[{run+1}] {travel*100:.1f}cm — 명령({args.distance*100:.0f}cm)보다 "
                      f"너무 멉니다. 측위가 튀었을 수 있어 버립니다")
                continue
            # 이동 중에 yaw 가 크게 변했으면 순수 직진이 아니다 — 회전이
            # 섞인 궤적의 시작·끝만 이으면 방향이 엉뚱하게 나온다.
            turned = abs((after.yaw_deg - before.yaw_deg + 180.0) % 360.0 - 180.0)
            if turned > MAX_TURN_DURING_RUN_DEG:
                print(f"[{run+1}] 이동 중 yaw 가 {turned:.1f}도 변했습니다 — "
                      "직진이 아니라 버립니다")
                continue
            moved_deg = math.degrees(math.atan2(dy, dx))
            # 보고된 yaw 는 이동 전후가 같아야 정상이다(직진이므로).
            reported = _circular_mean([before.yaw_deg, after.yaw_deg])
            delta = (moved_deg - reported + 180.0) % 360.0 - 180.0
            deltas.append(delta)
            print(f"[{run+1}] 이동 {travel*100:5.1f}cm  실제방향 {moved_deg:+7.1f}도  "
                  f"보고 yaw {reported:+7.1f}도  차이 {delta:+6.1f}도  "
                  f"pose {rate*100:.0f}%")

            if run < args.runs - 1:
                _drive(link, loc, cams, caps, detector, "yaw+", TURN_BETWEEN_RUNS_S)
                _wait_still(loc, cams, caps, detector)
    finally:
        # 어떤 경로로 끝나든 세운다. 예외로 전진 명령이 마지막에 남으면
        # Pi 워치독이 잡을 때까지 계속 간다.
        for _ in range(8):
            link.send(MissionCommand("stop", "SEARCH_TARGET", 0.0, 0.0, 0.0))
            time.sleep(0.05)
        for c in caps:
            c.release()

    print()
    if len(deltas) < 2:
        print("쓸 만한 회차가 2개 미만입니다 — 값을 못 냅니다.")
        print("바퀴가 실제로 돌았는지, 로봇이 두 카메라에 보이는지 보십시오.")
        return 1

    mean = _circular_mean(deltas)
    spread = statistics.pstdev([(d - mean + 180.0) % 360.0 - 180.0 for d in deltas])
    new_offset = (cfg.YAW_OFFSET_DEG + mean + 180.0) % 360.0 - 180.0
    print(f"차이 평균 {mean:+.1f}도   회차 편차 {spread:.1f}도   표본 {len(deltas)}회")
    if spread > 5.0:
        print("\n⚠️ 회차별 편차가 큽니다. 축 보정이 아니라 다른 문제일 수 있습니다 —")
        print("   바퀴 미끄러짐, 바닥 기울기, 한쪽 모터 약함, 마커 흔들림을 보십시오.")
    if abs(mean) < 2.0:
        print("\n지금 값이 맞습니다 — 고칠 필요가 없습니다.")
        return 0
    print(f"\nconfig.py 를 이렇게 고치십시오:")
    print(f"    YAW_OFFSET_DEG = {new_offset:.1f}      # 지금 {cfg.YAW_OFFSET_DEG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
