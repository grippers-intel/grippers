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
`arm_driver`가 이미 알고 있는 `drop`(300mm, 옛 195mm) 자세다). 정식
`MissionState.INSERT`("PLACE")가 아니라 **`DEBUG_FORCE_INSERT`**(테스트
전용 우회로, `baseline_ports.py` 참고)를 보낸다 — Pi 가 팔을 drop(300mm)
으로 옮겨 그리퍼를 열고, 성공/실패를 판정한 뒤 IDLE 로 복귀할 때까지
기다린다. 그동안 새 WASD 입력은 받지 않는다(팔이 움직이는 도중에 차를
몰면 안 된다) — 결과가 오면 화면에 찍고 다시 평소 운전으로 돌아간다.

⚠️ **정식 "PLACE"를 보내면 안 먹는다** — 2026-09-05 실기에서 확인: Pi의
`BaselineCarryState._judge_insert()`가 라이다 기반 `check_insert` 게이트
(요·좌우·거리 판정, 2026-09-04 밤 바구니 놓침 사고 이후 재활성화된 최종
안전판)를 거치는데, 이 도구는 그 게이트를 만족시킬 만큼 정밀하게 세우는
용도가 아니라서 계속 `INSERT_BLOCKED`만 받고 CARRY에 눌러앉아 팔이 아예
안 움직인다(사용자 지시 — "라이다 명령이 왜 들어가 있어? 빼고"). 이
도구는 그 게이트를 시험하는 게 아니라 **safe_300(servo 1 요 보정) 자체**를
확인하는 것이므로 `DEBUG_FORCE_INSERT`로 그 게이트를 건너뛴다. 정식
run_mission.py 경로의 그 안전판은 전혀 안 건드렸다 — 이 우회로는
manual_insert_probe.py만 쓴다.

## safe_300 실기 확인(2026-09-05)

NUDGE_LINE에 진입하는 순간의 `gate.facing_error_deg`(배너에 찍히는 바로 그
지향오차)를 그대로 `MissionCommand.yaw_correction_deg`에 실어 INSERT와
함께 보낸다. Pi의 `BaselineInsertState`는 drop(300mm) 자세에 도달했지만
그리퍼를 열기 **전에**, 이 값이 0이 아니면 `offset_base_yaw()`로 servo
1(팔 베이스 요)을 그만큼 돌려 흡수하고, 놓은 뒤 idle로 접기 전에 원래
각도로 되돌린다 — 차량을 다시 회전시키지 않고 팔로 잔여 오차를 흡수하는
경로(safe_300)를 이 도구 하나로 실기 검증할 수 있다.

⚠️ 아직 자동화되지 않은 부분: 이 도구는 여전히 "차가 정면에 가깝게 들어와
NUDGE_LINE을 넘는 순간"을 사람이 WASD로 만들어 준다 — host/mission.py의
CARRY_TO_DEST/FACE_BOX/NUDGE_BOX는 손대지 않았다. "차량이 방향에 상관없이
경계선에서 바로 서고, 그 잔여 오차 전부를 팔로 흡수" 하는 자동 경로는
아직 없다 — 지금 있는 것은 그 잔여 오차를 servo 1로 흡수하는 **뒷단
메커니즘 자체가 실기에서 먹는지**를 사람이 대신 서서 확인하는 수단이다.
⚠️ servo 1 회전 부호는 2026-09-05 첫 실기에서 반대로 확인됐다(사용자
보고 — "servo1이 돌았는데, 반대방향으로 돌았어") — `facing_error_deg`를
그대로 넘기면 오차를 줄이는 대신 키운다. `baseline_mission.py`의
`BaselineInsertState.execute()`에서 `correct_drop_yaw`를 부를 때 부호를
뒤집도록 고쳤다(`correction_rad = -math.radians(yaw_correction_deg)`).

⚠️ **2026-09-05 두 번째 실기 — 두 번째 Enter에서 팔이 아예 안 움직였다**
(사용자 보고 — "두번째 시도는 팔이 안 움직이네"). `BaselineInsertState.execute()`는
성공/실패 상관없이 항상 진짜 `BaselineIdleState`로 돌아가는데, `BaselineIdleState`는
APPROACH/DEBUG_FORCE_CARRY/DONE만 받는다 — 그 뒤 이 도구가 계속 보내는
"CARRY_TO_DEST"(=MissionState.CARRY)는 IDLE에서 조용히 무시된다(단
`_drive`는 IDLE에서도 먹으므로 차량 자체는 정상 주행처럼 보인다). 그
상태로 두 번째 Enter를 쳐도 `DEBUG_FORCE_INSERT`가 IDLE의 인식 목록에
없어 아무 일도 안 난다. 고침: `DEBUG_FORCE_CARRY` 핸드셰이크를
`_force_carry()` 함수로 빼서 시작할 때뿐 아니라 INSERT 결과(PLACE_DONE/
FAILED)가 나올 때마다 매번 다시 부른다.

⚠️ **2026-09-05 첫 실기에서 이게 안 먹었다** — Pi의 진짜 FSM(baseline_mission.py)
은 상태 객체가 자기가 인식하는 다음 상태로만 넘어가서, IDLE에서 곧장
INSERT로 점프하는 경로가 아예 없다(GRASP_FORCE도 결국 팔이 실제 파지를
수행한다). 그래서 이 스크립트는 이제 **시작하자마자 한 번**
`MissionState.DEBUG_FORCE_CARRY`(테스트 전용 우회로, `baseline_ports.py`
참고)를 보내 실제 파지 없이 CARRY로 먼저 들어간 뒤에 WASD 운전을
시작한다 — 그래야 나중에 Enter의 `DEBUG_FORCE_INSERT`가 유효한 명령이
된다. 이 우회로는 Pi가 **막 부팅/bringup 직후 IDLE일 때만** 먹는다 —
run_mission.py를 방금 돌렸거나 중간에 멈췄다면 Pi가 이미 다른 상태에
있어 안 먹을 수 있다(그럴 땐 Pi를 재기동하거나 IDLE로 되돌린 뒤 다시
실행할 것). 드랍 자세/개방폭은 `baseline_mission.DEBUG_FORCE_CARRY_LABEL`
(기본값 "rook")의 교시 계획을 쓴다 — 실제로 무엇을 손에 쥐여줬든 이
라벨 기준으로 팔이 움직인다.

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

    # 2026-09-05 — Pi FSM은 IDLE에서 INSERT로 곧장 못 간다(모듈 docstring의
    # "2026-09-05 첫 실기" 항목 참고). 그래서 WASD를 받기 전에 먼저
    # DEBUG_FORCE_CARRY로 실제 파지 없이 CARRY에 들어가 둔다. 최대 1초간
    # 반복 전송하고(UDP 유실 대비), Pi가 IDLE을 벗어났는지(poll_status()
    # != "IDLE")로 성공 여부를 확인한다 — 실패해도 진행은 시키되, Enter를
    # 눌러도 이번에도 안 될 수 있다고 분명히 경고한다.
    #
    # ⚠️ 2026-09-05 두 번째 실기에서 확인: INSERT(DEBUG_FORCE_INSERT)가
    # 끝나면(성공이든 실패든) BaselineInsertState.execute()는 항상
    # BaselineIdleState로 돌아간다(위 execute() 참고) — CARRY에 안
    # 남는다. BaselineIdleState는 APPROACH/DEBUG_FORCE_CARRY/DONE 만
    # 받으므로, 그 뒤 계속 "CARRY_TO_DEST"(=MissionState.CARRY)로
    # 운전해도 Pi는 IDLE에 그대로 머문다(단 _drive는 IDLE에서도 먹으므로
    # 차량 자체는 정상으로 움직여 사람 눈에는 안 티가 난다) — 그 상태로
    # 두 번째 Enter를 쳐도 DEBUG_FORCE_INSERT가 IDLE의 인식 목록에 없어
    # 조용히 무시되고 팔이 전혀 안 움직인다(사용자 보고 — "두번째 시도는
    # 팔이 안 움직이네"). 그래서 이 핸드셰이크를 함수로 빼서 시작할 때
    # 한 번, 그리고 INSERT 결과가 나올 때마다(성공/실패 상관없이) 매번
    # 다시 부른다.
    def _force_carry() -> bool:
        print("DEBUG_FORCE_CARRY 전송 중 — 실제 파지 없이 CARRY로 먼저 들어갑니다...",
              flush=True)
        ok = False
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            link.send(MissionCommand("stop", "DEBUG_FORCE_CARRY", 0.0, 0.0, 0.0))
            if link.poll_status() != "IDLE":
                ok = True
                break
            time.sleep(LOOP_PERIOD_S)
        if ok:
            print("CARRY 진입 확인 — Enter로 INSERT를 보낼 수 있습니다.\n", flush=True)
        else:
            print("⚠️ CARRY 진입을 확인 못 했습니다 — Pi가 막 IDLE이 아니었을 수 있습니다"
                  "(run_mission.py를 방금 돌렸다면 Pi를 재기동하세요). 계속 진행하지만"
                  " Enter를 눌러도 이번에도 팔이 안 움직일 수 있습니다.\n", flush=True)
        _log(f"DEBUG_FORCE_CARRY_SENT ok={ok}")
        return ok

    force_carry_ok = _force_carry()
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

    def _send(cmd: str, status: str, pose, yaw_correction_deg: float = 0.0) -> None:
        link.send(MissionCommand(cmd, status, pose.x if pose else 0.0,
                                 pose.y if pose else 0.0,
                                 pose.yaw_deg if pose else 0.0,
                                 yaw_correction_deg=yaw_correction_deg))

    # safe_300 실기 확인용(2026-09-05) — NUDGE_LINE 진입 순간 이미 계산돼
    # 있는 gate.facing_error_deg(아래 배너에도 찍히는 바로 그 값)를 그대로
    # INSERT 명령에 실어 보낸다. 자동 계산 경로(host/mission.py 쪽 "차량이
    # 방향 무관하게 경계선에서 서고 남은 오차를 팔로 흡수") 전체를 아직
    # 안 만들었어도, 이 도구로 지금 당장 servo 1 보정 자체가 실기에서
    # 먹는지만 따로 검증할 수 있다.
    pending_yaw_correction_deg = 0.0

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
                    if mode == "insert" and key in (" ", "x"):
                        # 2026-09-05 실기 — Pi의 lidar 기반 check_insert(요·좌우·
                        # 거리 허용치가 이 도구의 Host 게이트보다 훨씬 빡빡하다,
                        # BASKET_YAW_TOLERANCE_RAD=0.087·BASKET_LATERAL_TOLERANCE_M
                        # =0.070·BASKET_STOP_LIDAR_M±TOLERANCE)이 계속
                        # INSERT_BLOCKED를 내면 이 모드에서 나갈 방법이 전에는
                        # 없었다(q로 도구 전체를 끄는 것 말고는). CARRY는 그대로
                        # 유지한 채(Pi FSM은 계속 BaselineCarryState다 — 다시
                        # DEBUG_FORCE_CARRY를 보낼 필요 없다) 드라이브만 재개한다.
                        mode = "drive"
                        current_cmd = "stop"
                        last_key_at = now
                        prev_gate_ok = False   # 지금 위치가 아직 게이트 안이면
                                               # 바로 다음 사이클에 확인 배너를 다시 띄운다
                        print("\n[INSERT 취소] 드라이브로 복귀합니다 — 자세를 다시 "
                              "잡고 NUDGE_LINE에서 다시 Enter를 눌러 보세요.\n",
                              flush=True)
                        _log("INSERT_CANCELLED_BY_USER")
                    elif mode == "confirm" and key in ("\r", "\n"):
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
                    _send("stop", "CARRY_TO_DEST", None)
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
                    # 2026-09-05 실기: "PLACE"(정식 INSERT)를 보냈더니 Pi의
                    # 라이다 기반 check_insert 게이트(요·좌우·거리 판정)가
                    # 계속 INSERT_BLOCKED만 내고 CARRY에 눌러앉아 팔이 아예
                    # 안 움직였다 — 이 도구는 safe_300(servo 1 요 보정) 자체를
                    # 확인하려는 것이지 라이다 게이트를 시험하려는 게 아니다
                    # (사용자 지시 — "라이다 명령이 왜 들어가 있어? 빼고").
                    # DEBUG_FORCE_INSERT로 그 게이트를 건너뛰고 곧장
                    # BaselineInsertState로 들어간다.
                    _send("stop", "DEBUG_FORCE_INSERT", pose,
                          yaw_correction_deg=pending_yaw_correction_deg)
                    if report_status in ("PLACE_DONE", "FAILED"):
                        elapsed = now - insert_started_at
                        mark = "✅ PLACE_DONE" if report_status == "PLACE_DONE" else "❌ FAILED"
                        print(f"\n{mark} — {elapsed:.1f}초 걸림. IDLE로 돌아갔습니다. "
                              f"평소 운전으로 복귀합니다.\n", flush=True)
                        _log(f"INSERT_RESULT {report_status} elapsed_s={elapsed:.1f}")
                        # BaselineInsertState는 성공/실패 상관없이 항상 진짜
                        # BaselineIdleState로 돌아간다 — 다음 Enter가 먹으려면
                        # CARRY를 다시 만들어 둬야 한다(위 _force_carry 정의부
                        # 주석 참고, 2026-09-05 두 번째 실기에서 확인).
                        _force_carry()
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
                    # 이 순간의 잔여 지향오차를 그대로 servo 1 보정값으로 쓴다
                    # (safe_300 실기 확인용, 2026-09-05). Enter를 누르기까지
                    # 시간이 걸려도 다시 갱신하지 않는다 — "경계선에서 멈춘
                    # 순간"의 오차를 보는 것이 이 도구의 취지와 맞는다.
                    pending_yaw_correction_deg = gate.facing_error_deg
                    _bell(5)
                    print("\n" + "=" * 70)
                    print("🔔 NUDGE_LINE 진입 — Host 판정: 여기서부터 INSERT 시도해 볼 만함")
                    print(f"   pose=({pose.x:.3f},{pose.y:.3f}) yaw={pose.yaw_deg:.1f}°")
                    print(f"   목표영역까지 {gate.distance_m*1000:.0f}mm, "
                          f"지향오차 {gate.facing_error_deg:+.1f}°")
                    print(f"   Enter를 누르면 이 지향오차({pending_yaw_correction_deg:+.1f}°)를 "
                          "safe_300에서 servo 1로 보정합니다.")
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

                # 2026-09-05 실기: status를 "NUDGE_BOX"(→ MissionState.
                # APPROACH_BOX)로 보냈더니, Pi의 BaselineCarryState가 매
                # 사이클 라이다로 retreat_if_too_close를 판정해 0.128m
                # 안의 아무 물체(벽·기물 등, 바구니가 아니어도)에도
                # base.stop()을 걸고 INSERT_BLOCKED를 냈다 — 목표 근처가
                # 아니어도, Enter를 누르기 한참 전부터도 걸려서 WASD가
                # 안 먹혔다(사용자 지시 — "라이다가 왜 또 나와"). 이 도구는
                # 사람이 보면서 세우는 것이 유일한 안전장치라고 이미 문서화돼
                # 있으므로(위 "안전" 절 참고), 평소 운전에는 그 라이다 판정이
                # 아예 안 걸리는 "CARRY_TO_DEST"(→ MissionState.CARRY)를 쓴다
                # — NUDGE_LINE 진입·촉발 판정 자체는 여전히 Host의
                # check_basket_insert_gate(라이다 아님)가 한다.
                send_cmd = "stop" if mode == "confirm" else current_cmd
                _send(send_cmd, "CARRY_TO_DEST", pose)

                sys.stdout.write(
                    f"\r[{send_cmd:5s}] pose=({pose.x:6.3f},{pose.y:6.3f},"
                    f"{pose.yaw_deg:6.1f}°) 목표까지 {gate.distance_m*1000:5.0f}mm "
                    f"지향 {gate.facing_error_deg:+5.1f}° "
                    f"{'GATE_OK' if gate.ok else '       '}"
                    f"{'  [NUDGE_LINE — Enter로 INSERT]' if mode == 'confirm' else '           '}   ")
                sys.stdout.flush()

                time.sleep(max(0.0, LOOP_PERIOD_S - (time.perf_counter() - tick_start)))
        finally:
            _send("stop", "CARRY_TO_DEST", pose)
            print("\n\n[종료] 정지 명령을 보냈습니다.", flush=True)
            _log("STOP_AND_EXIT")
            log_f.close()
            for c in caps:
                c.release()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
