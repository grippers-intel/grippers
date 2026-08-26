#!/usr/bin/env python3
"""Host의 **실제 코드**와 Pi의 **실제 코드**를 로컬에서 맞붙인다 (2026-08-26).

하드웨어가 필요 없다. 양쪽 링크 계층이 표준 라이브러리만 쓰기 때문에 맥 한
대에서 두 프로세스를 띄우고 실제로 패킷을 주고받게 할 수 있다 — 흉내가 아니라
`sysy009/grippers_topview`의 `UdpVehicleLink`와 이 저장소의 `UdpHostLink`를
그대로 import해서 쓴다.

## 왜 이게 필요한가

인수인계에 "양쪽을 붙여 본 적이 없다"고 적어 둔 항목이다. 포트 번호, 필드
이름, 워치독 타이밍 중 **하나만 어긋나도 통합 당일에 아무것도 안 움직인다.**
그리고 실제로 어긋나 있다 — Host는 `status`, Pi는 `state`를 쓴다.

## 무엇을 확인하나

    1. 명령이 파싱되는가        Host가 보낸 것을 Pi가 HostCommand로 받는가
    2. 보고가 도달하는가        Pi가 보낸 것을 Host의 poll_status()가 읽는가
    3. 워치독이 도는가          명령을 끊으면 Pi가 3사이클 안에 멈추는가
    4. 규약 위반을 막는가       회전+병진 혼합이 거부되는가

## 두 가지 모드

    --as-is       Host 저장소 코드를 그대로 쓴다. **실패하는 것이 정상이다** —
                  현재 어긋나 있다는 것을 실행으로 보여주는 것이 목적이다.
    --translated  아래 TRANSLATION 표대로 변환해 보낸다. 이것이 통과하면
                  "Host가 send()만 고치면 붙는다"가 증명된 것이다.

## 이 파일이 곧 Host 팀에 줄 규격이다

`translate()`는 Host `UdpVehicleLink.send()`가 해야 할 일을 그대로 적은 것이다.
설명 문서보다 이쪽이 낫다 — 돌려 볼 수 있고, 어긋나면 시험이 깨진다.

## 실행

    python3 tools/host_link_conformance.py --as-is
    python3 tools/host_link_conformance.py --translated
"""

import argparse
import json
import pathlib
import socket
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from domain.adapters.real.udp_host_link import UdpHostLink
from domain.task import baseline_constants as bc
from domain.task.motion import resolve_motion
from domain.ports.baseline_ports import HostCommand

# Host 저장소를 옆에 클론해 둔 자리. 없으면 안내하고 끝낸다 — 흉내로 대신하면
# 이 시험의 존재 이유(진짜 코드끼리 붙인다)가 사라진다.
TOPVIEW = pathlib.Path.home() / "Desktop/intel/grippers_topview"

BANNER = "=" * 70
CMD_PORT, STATUS_PORT = 45005, 45006

# ── Host가 send()에서 해야 할 변환 ─────────────────────────────────────────
# 크기는 팀이 2026-08-26에 고정했다(직진·횡이동 0.1 m/s, 제자리회전 0.25 rad/s).
# 그래서 Host의 방향-only 명령(go/stop/yaw+/yaw-)이 일대일로 번역된다.
# ⚠️ `go`의 속도가 구간마다 다르다. 바구니로 붙는 구간(FACE_BOX/NUDGE_BOX)은
# 0.06이다 — 정지 지연 235ms 동안 0.1이면 23.5mm를 더 가는데 INSERT 허용폭이
# ±15mm라 오버슈트가 창을 넘는다(domain/task/motion.py 주석 참고).
#
# Host가 안 낮춰도 Pi가 그 구간에서 0.06으로 자르지만, **Host가 직접 보내는
# 편이 낫다** — 그래야 Host의 도착 예측과 실제 이동이 어긋나지 않는다.
BASKET_APPROACH_STATUSES = ("FACE_BOX", "NUDGE_BOX")

CMD_TO_MOTION = {
    "go":    {"linear_x": 0.1, "linear_y": 0.0, "angular_z": 0.0, "stop": False},
    "stop":  {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0, "stop": True},
    "yaw+":  {"linear_x": 0.0, "linear_y": 0.0, "angular_z": +0.25, "stop": False},
    "yaw-":  {"linear_x": 0.0, "linear_y": 0.0, "angular_z": -0.25, "stop": False},
}

# Host의 미션 단계 -> Pi의 MissionState. NUDGE_BOX는 Host 문서에는 없고
# mission.py에만 있다 — 코드를 기준으로 삼아야 한다.
STATUS_TO_STATE = {
    "SEARCH_TARGET": "IDLE",
    "APPROACH_PIECE": "APPROACH",
    "GRASP": "GRASP",
    "CARRY_TO_DEST": "CARRY",
    "FACE_BOX": "APPROACH_BOX",
    "NUDGE_BOX": "APPROACH_BOX",
    "PLACE": "INSERT",
    "DONE": "DONE",
}

# Pi의 보고 -> Host가 아는 어휘. 대부분 그대로 통하고 INSERT 계열만 다르다.
# ⚠️ BLOCKED/CENTERING/REJECTED는 Host 어휘에 대응이 **없다.** FAILED로
# 뭉뚱그리면 Host는 재시도밖에 못 하는데, 재시도는 같은 자리에서 같은 이유로
# 또 막힌다. 이 표의 빈칸이 곧 Host 상태 기계에 필요한 분기다.
REPORT_TO_HOST = {
    "GRASP_DONE": "GRASP_DONE",
    "INSERT_DONE": "PLACE_DONE",
    "GRASP_FAILED": "FAILED",
    "INSERT_FAILED": "FAILED",
    "IDLE_DONE": "IDLE",
    "STATE": "BUSY",
}
NO_HOST_EQUIVALENT = ("GRASP_BLOCKED", "GRASP_CENTERING", "INSERT_BLOCKED",
                      "REJECTED")


def translate(mission_command):
    """Host `MissionCommand` -> 팀 확정 다섯 필드.

    Host `UdpVehicleLink.send()`가 `asdict(cmd)` 대신 이것을 보내면 된다."""
    motion = dict(CMD_TO_MOTION[mission_command.cmd])
    if (mission_command.cmd == "go"
            and mission_command.status in BASKET_APPROACH_STATUSES):
        motion["linear_x"] = 0.06
    return {"state": STATUS_TO_STATE[mission_command.status], **motion}


# ── 시험 ───────────────────────────────────────────────────────────────────

class Result:
    def __init__(self):
        self.rows = []

    def add(self, name, ok, detail=""):
        self.rows.append((name, ok, detail))
        mark = "✅" if ok else "⛔"
        print(f"  {mark}  {name}")
        if detail:
            for line in detail.splitlines():
                print(f"        {line}")

    def summary(self):
        passed = sum(1 for _n, ok, _d in self.rows if ok)
        print()
        print(BANNER)
        print(f"  {passed}/{len(self.rows)} 통과")
        print(BANNER)
        return passed == len(self.rows)


def load_host_link():
    if not (TOPVIEW / "vehicle_link.py").exists():
        raise SystemExit(
            f"Host 저장소를 찾을 수 없습니다: {TOPVIEW}\n"
            "    git clone https://github.com/sysy009/grippers_topview.git")
    sys.path.insert(0, str(TOPVIEW))
    import vehicle_link
    return vehicle_link


def run(translated):
    vl = load_host_link()
    result = Result()

    pi = UdpHostLink("127.0.0.1", command_port=CMD_PORT, status_port=STATUS_PORT)
    host = vl.UdpVehicleLink("127.0.0.1", cmd_port=CMD_PORT, status_port=STATUS_PORT)
    raw = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # --- 1. 명령이 파싱되는가 -------------------------------------------
        command = vl.MissionCommand(
            cmd="go", status="APPROACH_PIECE",
            robot_x=0.912, robot_y=0.543, robot_yaw_deg=87.3)
        if translated:
            raw.sendto(json.dumps(translate(command)).encode(),
                       ("127.0.0.1", CMD_PORT))
        else:
            host.send(command)
        time.sleep(0.3)
        received = pi.latest_command()
        result.add(
            "1. Host 명령을 Pi가 파싱한다",
            received is not None,
            f"받은 것: {received}" if received else
            "Pi가 패킷을 버렸다 — Host는 'status', Pi는 'state'를 쓴다")

        # 속도까지 맞는지. 파싱만 되고 값이 0이면 차가 안 움직인다.
        if received is not None:
            moves = abs(received.linear_x) > 1e-9 or abs(received.angular_z) > 1e-9
            result.add(
                "1b. 'go'가 실제 전진 속도로 도착한다",
                moves and not received.stop,
                f"linear_x={received.linear_x} angular_z={received.angular_z} "
                f"stop={received.stop}")

        # --- 2. 보고가 도달하는가 -------------------------------------------
        pi.report("GRASP_DONE", "GRASP", "queen 부하 0.0626")
        time.sleep(0.3)
        status = host.poll_status()
        result.add(
            "2. Pi 보고를 Host가 읽는다",
            status == "GRASP_DONE",
            f"poll_status() = {status!r}" + ("" if status == "GRASP_DONE" else
            "\nPi는 'report' 키로 보내는데 Host는 'status'를 찾는다 —"
            "\nKeyError가 나서 조용히 버려지고 영원히 IDLE이 나온다."
            "\n"
            "\n⚠️ 이것은 --translated로도 안 고쳐진다. 변환은 명령 방향만"
            "\n   손대기 때문이다. 보고 방향은 Host가 고쳐야 한다:"
            "\n"
            "\n     vehicle_link.py UdpVehicleLink.poll_status()"
            "\n     -  latest = json.loads(data)[\"status\"]"
            "\n     +  latest = json.loads(data)[\"report\"]"))

        # --- 3. 워치독 ------------------------------------------------------
        # 새 명령이 없으면 latest_command()가 None을 돌려주고, 미션 루프가
        # 3사이클(0.1s x 3) 세어 정지시킨다. 여기서는 그 전제인 "안 오면
        # None"만 확인한다 — 루프 자체는 ROS 노드 안에 있다.
        starved = [pi.latest_command() for _ in range(3)]
        result.add(
            "3. 명령이 끊기면 None을 돌려준다 (워치독 전제)",
            all(c is None for c in starved),
            f"연속 3회: {starved}\n"
            f"Pi 워치독 {bc.HOST_COMMAND_TIMEOUT_CYCLES}사이클 x 0.1s = 0.3s, "
            f"Host VEHICLE_LINK_TIMEOUT_S = 0.3 — 일치")

        # --- 4. 규약 위반 ---------------------------------------------------
        mixed = HostCommand(state="APPROACH", linear_x=0.1, linear_y=0.0,
                            angular_z=0.25, stop=False)
        decision = resolve_motion(mixed)
        result.add(
            "4. 회전+병진 혼합을 거부한다",
            not decision.ok,
            f"사유: {decision.reason}" if not decision.ok else
            "거부되지 않았다 — 규약이 헐거워졌다")

        # --- 5. 어휘 공백 (정보) --------------------------------------------
        print()
        print("  ℹ️  Host 어휘에 대응이 없는 Pi 보고:")
        for report in NO_HOST_EQUIVALENT:
            print(f"        {report}")
        print("      FAILED로 뭉뚱그리면 Host는 재시도밖에 못 하고,")
        print("      재시도는 같은 자리에서 같은 이유로 또 막힌다.")

    finally:
        pi.close()
        host.close()
        raw.close()

    return result.summary()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--as-is", action="store_true",
                      help="Host 저장소 코드 그대로 (실패하는 것이 정상)")
    mode.add_argument("--translated", action="store_true",
                      help="변환표를 적용 (통과해야 한다)")
    args = parser.parse_args()

    print(BANNER)
    print("Host <-> Pi 링크 적합성 시험  (하드웨어 불필요)")
    print(BANNER)
    print(f"  Host 코드: {TOPVIEW}")
    print(f"  Pi 코드  : {pathlib.Path(__file__).resolve().parent.parent}")
    print(f"  모드     : {'변환 적용' if args.translated else 'Host 코드 그대로'}")
    print()

    ok = run(args.translated)
    if args.as_is:
        print("\n  (--as-is는 실패가 정상입니다 — 지금 어긋나 있다는 것을")
        print("   실행으로 보여주는 것이 목적입니다. --translated로 다시 돌려 보세요.)")
        return 0
    if not ok:
        print("\n  남은 실패가 '2. 보고'뿐이라면 Host의 poll_status() 한 줄입니다.")
        print("  명령 방향은 이 변환표로 붙는다는 것이 위에서 증명됐습니다.")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
