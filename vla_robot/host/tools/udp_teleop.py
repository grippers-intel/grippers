"""키보드로 Pi 에 HostCommand 를 직접 보낸다 — ArUco·검출기 없이 Pi 쪽만 시험.

사용법 (host/ 에서):
    python tools/udp_teleop.py --pi-ip 192.168.0.7

OpenCV 창에 포커스를 두고 조작한다(추가 의존성 없이 Windows/macOS/Linux 공통).

  w/s  앞/뒤        a/d  왼쪽/오른쪽 횡이동      q/e  반시계/시계 회전
  space 또는 x  정지          esc  종료
  1 IDLE  2 APPROACH  3 CARRY  4 APPROACH_BOX  5 GRASP  6 PLACE  0 ESTOP

⚠️ 키는 **래치**다 — 한 번 누르면 정지(space)를 누를 때까지 그 속도를 계속 보낸다.
   키 반복 지연(약 0.5 s) 때문에 "누르고 있는 동안만" 방식은 끊겨서 차가 덜컥거린다.
⚠️ 5(GRASP)/6(PLACE) 는 Pi 에서 **실제 팔 작업을 시작한다.**
명령은 10 Hz 로 계속 나간다. 창을 닫으면 멈추고, 멈추면 Pi 워치독이 세운다.
"""
from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HOST_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST_ROOT))

import host_config  # noqa: E402,F401
from link.vehicle_link import UdpVehicleLink  # noqa: E402
from vla_common.protocol import COMMAND_PORT, STATUS_PORT, HostCommand, State  # noqa: E402

STATE_KEYS = {ord("1"): State.IDLE, ord("2"): State.APPROACH, ord("3"): State.CARRY,
              ord("4"): State.APPROACH_BOX, ord("5"): State.GRASP, ord("6"): State.PLACE,
              ord("0"): State.ESTOP}


def main() -> int:
    host_config.configure_console()
    ap = argparse.ArgumentParser()
    ap.add_argument("--pi-ip", required=True)
    ap.add_argument("--command-port", type=int, default=COMMAND_PORT)
    ap.add_argument("--status-port", type=int, default=STATUS_PORT)
    ap.add_argument("--linear", type=float, default=0.15)
    ap.add_argument("--angular", type=float, default=0.5)
    ap.add_argument("--label", default="queen")
    args = ap.parse_args()

    link = UdpVehicleLink(args.pi_ip, args.command_port, args.status_port)
    state = State.IDLE
    vel = (0.0, 0.0, 0.0)
    win = "udp_teleop (focus here)"
    cv2.namedWindow(win)
    moves = {ord("w"): (args.linear, 0, 0), ord("s"): (-args.linear, 0, 0),
             ord("a"): (0, args.linear, 0), ord("d"): (0, -args.linear, 0),
             ord("q"): (0, 0, args.angular), ord("e"): (0, 0, -args.angular)}
    try:
        while True:
            t0 = time.monotonic()
            stop = vel == (0.0, 0.0, 0.0)
            link.send(HostCommand(state, linear_x=vel[0], linear_y=vel[1], angular_z=vel[2],
                                  stop=stop, label=args.label))
            st = link.latest_status()
            img = np.full((260, 640, 3), 30, np.uint8)
            lines = [f"send: {state} vx={vel[0]:+.2f} vy={vel[1]:+.2f} wz={vel[2]:+.2f} stop={stop}",
                     f"status age: {link.status_age_s():.2f}s"]
            if st is not None:
                r = st.result
                lines += [f"pi: state={st.state} busy={st.busy} job={st.job_id}",
                          f"pi: base_ok={st.base_ok} watchdog={st.watchdog}",
                          f"last: {'-' if r is None else f'#{r.job_id} {r.action} ok={r.ok}'}",
                          f"      {'' if r is None else r.detail[:70]}",
                          f"detail: {st.detail[:70]}"]
            lines.append("wasd/qe move  space stop  1-6 state  0 ESTOP  esc quit")
            for i, text in enumerate(lines):
                cv2.putText(img, text, (10, 28 + 28 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1)
            cv2.imshow(win, img)
            key = cv2.waitKey(max(1, int(100 - (time.monotonic() - t0) * 1000))) & 0xFF
            if key == 27:
                break
            if key in (ord(" "), ord("x")):
                vel = (0.0, 0.0, 0.0)
            elif key in moves:
                vel = tuple(float(v) for v in moves[key])
            elif key in STATE_KEYS:
                state = STATE_KEYS[key]
                vel = (0.0, 0.0, 0.0)
                print(f"[teleop] state -> {state}")
            try:
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
    finally:
        link.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
