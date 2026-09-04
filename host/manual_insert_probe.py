#!/usr/bin/env python3
"""WASD 로 직접 몰면서 INSERT 경계를 손으로 확인하는 도구 (2026-09-05).

## 왜 필요한가

CARRY_TO_DEST 를 사선 접근 부채꼴 + 동적 정렬로 바꾼 뒤(mission.py,
basket_target.check_approach_sector), ArUco 마커와 그리퍼 사이의 물리
오프셋을 이 저장소가 몰라서 실기 검증 전에는 안전하다고 말할 수 없는
상태다(basket_target.py의 SOUTH_APPROACH_SECTOR_RADIUS_M 주석 참고).

전체 미션(SEARCH→APPROACH→GRASP→CARRY→...)을 자동으로 돌리면 그 경계에
도달하는 자세를 마음대로 못 고른다 — 이 도구는 그 대신 사람이 WASD 로
직접(여러 각도로 사선 포함) 몰면서, **Host 소프트웨어가 "여기서부터
INSERT를 시도해도 된다"고 판단하는 그 순간**을 신호로 알려준다. 그 자리에서
차가 멈추면, 이미 그리퍼에 쥐고 있는 기물(예: 룩)을 손으로 상자 쪽에
대보거나 눈으로 각도를 보고 "이 자세에서 내려놓으면 걸리는지/바깥으로
떨어지는지"를 사람이 직접 판단한다.

## 신호 두 가지

1. **NUDGE_LINE 진입** — `basket_target.check_basket_insert_gate()` 가
   거짓→참으로 바뀌는 순간(=NUDGE_BOX 가 "다 왔다"고 볼 바로 그 경계).
   차를 자동으로 멈추고, 요란한 배너 + 터미널 벨을 울리고, **Enter 를 칠
   때까지 그 자리에서 기다린다.** 그동안 손으로 확인하면 된다.
2. **센터라인 통과** — 로봇 x 가 목표영역 중심 x
   (`basket_target.target_center()[0]`) 를 지나는 순간. 정지하지 않고
   벨+로그 한 줄만 남긴다(운전을 방해하지 않는 보조 신호).

⚠️ **부저는 못 쓴다.** Host→Pi 전선 규격(`domain/ports/baseline_ports.py`
의 `HostCommand`)에 부저 필드가 없다 — 이 프로젝트의 실제 STM32 부저는
Pi 쪽에서만 울릴 수 있고, 여기서 원격으로 트리거할 방법이 없다. 그래서
신호는 터미널 배너 + 벨 문자(`\\a`)뿐이다. 스피커/헤드폰 볼륨을 켜 두면
벨 소리가 들린다.

**Enter 는 실제 INSERT 명령을 보낸다** (사용자 확인, 2026-09-05 — Pi 의
`arm_driver`가 이미 알고 있는 `drop`(300mm, 옛 195mm) 자세다). mission.py
의 PLACE 상태와 정확히 같은 방식으로 `MissionCommand("stop", "PLACE", ...)`
를 계속 보낸다(`status="PLACE"` → `vehicle_link._STATE_TO_PI`가
`MissionState.INSERT`로 옮긴다) — Pi 가 팔을 drop(300mm)으로 옮겨 그리퍼를
열고, 성공/실패를 판정한 뒤 IDLE 로 복귀할 때까지 기다린다. 그동안 새 WASD
입력은 받지 않는다(팔이 움직이는 도중에 차를 몰면 안 된다) — 결과가 오면
화면에 찍고 다시 평소 운전으로 돌아간다.

## 조작

    w        전진(go)
    s        후진(back)
    a        제자리 좌회전(yaw+, CCW)
    d        제자리 우회전(yaw-, CW)
    space/x  즉시 정지
    Enter    (NUDGE_LINE 에 멈춰 대기 중일 때만) INSERT 실행 — drop(300mm) → IDLE
    q        정지하고 종료(팔이 INSERT 도중이면 물리적으로는 안 멈출 수 있음)

**키를 놓으면(0.5초 안에 다시 안 누르면) 자동으로 멈춘다** — 눌러야
움직이는 방식이지 토글이 아니다. 터미널의 키 반복(길게 누르면 계속
들어오는 그 반복)이 곧 "누르고 있음"이 된다.

## 안전

이 도구도 결국 `UdpVehicleLink` 로 진짜 명령을 보낸다 — mission.py 의
전체 안전장치(hard_stop, 재정렬 예산 등) 는 하나도 안 거친다. 순수하게
사람이 보면서 세우는 것이 유일한 안전장치다. 처음 몇 번은 저속으로,
바구니에 최대한 가까워지기 **전에** 손을 뗄 준비를 하고 시작할 것.

포즈를 잃으면(ArUco 마커를 카메라가 놓치면) 매 사이클 "stop" 을 보낸다
(mission.py `step()` 의 pose.ok 분기와 같은 원칙).

Enter 로 INSERT 가 시작되면 실제로 팔이 움직이고 그리퍼가 열린다 — 이
시험의 목적이 "이 자세에서 실제로 투하하면 어떻게 되는가"이니 당연하지만,
Enter 를 누르기 전에 그리퍼 아래·주변에 손이나 다른 물체가 없는지 다시
한번 볼 것.

## 쓰는 법

    python3 manual_insert_probe.py --box chess --vehicle-ip 192.168.0.7
    python3 manual_insert_probe.py --box toy --cams 0 1   # 차량 없이 시험(콘솔 출력만)
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty
from pathlib import Path
from typing import Optional

import cv2

sys.path.insert(0, str(Path(__file__).parent / "aruco"))

import config as cfg                                    # noqa: E402
from localizer import Camera, RobotLocalizer, detect, make_detector  # noqa: E402

import basket_target                                     # noqa: E402
import mission_log                                        # noqa: E402
from run_localize import open_cams                        # noqa: E402
from vehicle_link import ConsoleVehicleLink, MissionCommand, UdpVehicleLink  # noqa: E402

LOOP_HZ = 14.0
LOOP_PERIOD_S = 1.0 / LOOP_HZ
# 이 시간 안에 새 키가 안 들어오면 자동으로 stop — "누르고 있어야 움직인다".
AUTO_STOP_IDLE_SEC = 0.5

_KEY_TO_CMD = {"w": "go", "s": "back", "a": "yaw+", "d": "yaw-",
               " ": "stop", "x": "stop"}


class _RawKeys:
    """터미널을 cbreak 모드로 바꿔 Enter 없이 한 글자씩 즉시 읽는다.

    with 블록을 벗어나면 원래 설정으로 반드시 되돌린다 — 안 그러면
    프롬프트가 이상해진 채로 남는다."""

    def __init__(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError(
                "stdin 이 터미널이 아닙니다 — 이 도구는 실제 키보드 입력이 "
                "필요합니다(백그라운드/파이프 실행 불가).")
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)

    def __enter__(self) -> "_RawKeys":
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, *_exc) -> None:
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def poll(self) -> Optional[str]:
        """지금 대기 중인 키 하나. 없으면 None(안 기다림)."""
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if not r:
            return None
        return sys.stdin.read(1)


def _bell(n: int = 3) -> None:
    sys.stdout.write("\a" * n)
    sys.stdout.flush()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("## 조작")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--box", choices=sorted(cfg.BOXES.keys()), default="chess",
                    help="INSERT 판정 대상 상자 (기본 chess)")
    ap.add_argument("--cams", type=int, nargs="+", default=list(cfg.CAM_INDICES))
    ap.add_argument("--vehicle-ip", type=str, default=None,
                    help="주면 실제 UDP 로 차량에 보낸다. 안 주면 콘솔에만 찍는다"
                         "(차량 없이 이 도구 자체를 시험할 때).")
    ap.add_argument("--vehicle-cmd-port", type=int, default=5005)
    ap.add_argument("--vehicle-status-port", type=int, default=5006)
    args = ap.parse_args()

    detector = make_detector()
    cams = [Camera.load(f"cam{i}", i) for i in args.cams]
    for c in cams:
        if not c.calibrated:
            print(f"⚠️ {c.name}: calib/cam*.npz 가 없어 HFOV 근사값을 씁니다 — "
                  f"이 시험은 위치 정밀도가 핵심이라 근사값이면 결과를 믿기 "
                  f"어렵습니다. calibrate_camera.py 를 먼저 돌리세요.")
    caps = open_cams(args.cams)
    if not any(c.isOpened() for c in caps):
        print("\n열린 카메라가 하나도 없습니다. --cams 로 인덱스를 바꿔 보세요.")
        for c in caps:
            c.release()
        return 1

    loc = RobotLocalizer()
    if args.vehicle_ip:
        link = UdpVehicleLink(args.vehicle_ip, cmd_port=args.vehicle_cmd_port,
                              status_port=args.vehicle_status_port)
        print(f"차량 연결: UDP -> {args.vehicle_ip}:{args.vehicle_cmd_port} "
              f"(상태 수신: :{args.vehicle_status_port})")
    else:
        link = ConsoleVehicleLink(auto_complete=False)
        print("--vehicle-ip 없음 — 콘솔에만 찍습니다(차량에 실제로 안 나갑니다).")

    box_lo, box_hi, box_ylo, box_yhi = basket_target.target_rect(args.box)
    center_x, _center_y = basket_target.target_center(args.box)

    log_path = mission_log.default_log_path("insert_probe")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_f = open(log_path, "a", encoding="utf-8", buffering=1)

    def _log(line: str) -> None:
        stamped = f"{time.strftime('%H:%M:%S')} {line}"
        log_f.write(stamped + "\n")

    print(__doc__.split("## 안전")[0].split("## 조작")[1])
    print(f"대상 상자: {args.box}  목표영역 x=[{box_lo:.3f},{box_hi:.3f}] "
          f"y=[{box_ylo:.3f},{box_yhi:.3f}]  center_x={center_x:.3f}")
    print(f"로그: {log_path}")
    print("-" * 70, flush=True)

    current_cmd = "stop"
    last_key_at = 0.0
    # "drive"   — 평소 WASD 운전.
    # "confirm" — NUDGE_LINE 에 서서 사람 확인을 기다리는 중(정지 유지).
    # "insert"  — Enter 를 눌러 INSERT(drop 300mm → IDLE)를 보내고 결과를 기다리는 중.
    mode = "drive"
    insert_started_at = 0.0
    insert_last_warn_at = 0.0
    prev_gate_ok = False
    prev_x: Optional[float] = None
    quit_requested = False
    pose_lost_warned = False
    pose = None   # finally 블록에서 참조 — 첫 사이클 전에 죽어도 NameError 안 나게

    def _send(cmd: str, status: str, pose) -> None:
        link.send(MissionCommand(cmd, status, pose.x if pose else 0.0,
                                 pose.y if pose else 0.0,
                                 pose.yaw_deg if pose else 0.0))

    with _RawKeys() as keys:
        try:
            while not quit_requested:
                tick_start = time.perf_counter()
                key = keys.poll()
                now = time.monotonic()

                if key:
                    if key == "q":
                        quit_requested = True
                        break
                    if mode == "confirm" and key in ("\r", "\n"):
                        mode = "insert"
                        insert_started_at = now
                        insert_last_warn_at = now
                        print("\n[INSERT 시작] drop(300mm) → IDLE 대기 중 — "
                              "끝날 때까지 새 주행 입력은 안 받습니다.\n", flush=True)
                        _log("INSERT_START")
                    elif mode == "drive" and key in _KEY_TO_CMD:
                        current_cmd = _KEY_TO_CMD[key]
                        last_key_at = now

                if mode == "drive" and current_cmd != "stop" and now - last_key_at > AUTO_STOP_IDLE_SEC:
                    current_cmd = "stop"

                grabbed = []
                dets = []
                for cap in caps:
                    ok, frame = cap.read()
                    grabbed.append(frame if ok else None)
                    dets.append({} if not ok else
                                detect(detector, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)))
                pose = loc.update(cams, dets)
                # 한 사이클에 한 번만 부른다 — 두 번째 호출은 큐가 이미 비어
                # IDLE만 돌려준다(poll_status() 자체 docstring 참고).
                report_status = link.poll_status()

                if not pose.ok and mode != "insert":
                    _send("stop", "NUDGE_BOX", None)
                    if not pose_lost_warned:
                        pose_lost_warned = True
                        print("\n⚠️ ArUco 포즈를 놓쳤습니다 — 정지 명령만 보냅니다"
                              "(마커가 다시 보이면 자동으로 이어집니다)\n", flush=True)
                        _log("POSE_LOST")
                    time.sleep(max(0.0, LOOP_PERIOD_S - (time.perf_counter() - tick_start)))
                    continue
                if pose_lost_warned:
                    pose_lost_warned = False
                    print("\n[복구] ArUco 포즈를 다시 찾았습니다.\n", flush=True)
                    _log("POSE_RECOVERED")

                if mode == "insert":
                    # mission.py PLACE 상태와 정확히 같은 명령을 계속 보낸다.
                    _send("stop", "PLACE", pose)
                    if report_status in ("PLACE_DONE", "FAILED"):
                        elapsed = now - insert_started_at
                        mark = "✅ PLACE_DONE" if report_status == "PLACE_DONE" else "❌ FAILED"
                        print(f"\n{mark} — {elapsed:.1f}초 걸림. IDLE로 돌아갔습니다. "
                              f"평소 운전으로 복귀합니다.\n", flush=True)
                        _log(f"INSERT_RESULT {report_status} elapsed_s={elapsed:.1f}")
                        mode = "drive"
                        current_cmd = "stop"
                        last_key_at = now
                    else:
                        if now - insert_last_warn_at > 3.0:
                            insert_last_warn_at = now
                            _log(f"INSERT_WAITING status={report_status} "
                                 f"elapsed_s={now - insert_started_at:.1f}")
                        sys.stdout.write(
                            f"\r[insert 대기] 상태={report_status:6s} 경과 "
                            f"{now - insert_started_at:5.1f}s          ")
                        sys.stdout.flush()
                    time.sleep(max(0.0, LOOP_PERIOD_S - (time.perf_counter() - tick_start)))
                    continue

                robot_xy = (pose.x, pose.y)
                gate = basket_target.check_basket_insert_gate(robot_xy, pose.yaw_deg, args.box)

                if not prev_gate_ok and gate.ok and mode == "drive":
                    current_cmd = "stop"
                    mode = "confirm"
                    _bell(5)
                    print("\n" + "=" * 70)
                    print("🔔 NUDGE_LINE 진입 — Host 판정: 여기서부터 INSERT 시도해 볼 만함")
                    print(f"   pose=({pose.x:.3f},{pose.y:.3f}) yaw={pose.yaw_deg:.1f}°")
                    print(f"   목표영역까지 {gate.distance_m*1000:.0f}mm, "
                          f"지향오차 {gate.facing_error_deg:+.1f}°")
                    print("   차를 정지시켰습니다. 지금 자세에서 그리퍼의 기물을 "
                          "확인하세요.")
                    print("   Enter 를 치면 INSERT(drop 300mm → IDLE) 를 실행합니다.")
                    print("=" * 70 + "\n", flush=True)
                    _log(f"NUDGE_LINE_ENTER pose=({pose.x:.3f},{pose.y:.3f},"
                         f"{pose.yaw_deg:.1f}) dist_mm={gate.distance_m*1000:.0f} "
                         f"facing_deg={gate.facing_error_deg:+.1f}")
                prev_gate_ok = gate.ok

                if prev_x is not None and (prev_x - center_x) * (pose.x - center_x) < 0:
                    _bell(1)
                    print(f"\n🔸 센터라인 통과 — x={pose.x:.3f} (center_x={center_x:.3f})\n",
                          flush=True)
                    _log(f"CENTERLINE_CROSS x={pose.x:.3f} center_x={center_x:.3f}")
                prev_x = pose.x

                send_cmd = "stop" if mode == "confirm" else current_cmd
                _send(send_cmd, "NUDGE_BOX", pose)

                sys.stdout.write(
                    f"\r[{send_cmd:5s}] pose=({pose.x:6.3f},{pose.y:6.3f},"
                    f"{pose.yaw_deg:6.1f}°) 목표까지 {gate.distance_m*1000:5.0f}mm "
                    f"지향 {gate.facing_error_deg:+5.1f}° "
                    f"{'GATE_OK' if gate.ok else '       '}"
                    f"{'  [NUDGE_LINE — Enter로 INSERT]' if mode == 'confirm' else '           '}   ")
                sys.stdout.flush()

                time.sleep(max(0.0, LOOP_PERIOD_S - (time.perf_counter() - tick_start)))
        finally:
            _send("stop", "NUDGE_BOX", pose)
            print("\n\n[종료] 정지 명령을 보냈습니다.", flush=True)
            _log("STOP_AND_EXIT")
            log_f.close()
            for c in caps:
                c.release()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
