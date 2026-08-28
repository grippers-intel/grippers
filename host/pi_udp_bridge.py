"""Host PC -> 차량(Pi) UDP 브릿지 뼈대.

PI_BRIDGE_TASK.md 의 규격을 그대로 구현한다. 실제 모터 제어/SmolVLA 호출은
_handle_cmd()/_do_grasp()/_do_place() 안에 채워 넣을 자리만 만들어뒀다 —
지금은 콘솔에 로그만 찍는다.

사용법
    python pi_udp_bridge.py
    python pi_udp_bridge.py --cmd-port 5005 --status-port 5006 --watchdog-sec 1.5
"""

from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from typing import Optional

_stop = False


def _send_status(sock: socket.socket, host_addr: tuple[str, int], status: str) -> None:
    payload = json.dumps({"status": status, "t": time.time()}).encode("utf-8")
    sock.sendto(payload, host_addr)
    print(f"[bridge] -> Host: {status}")


def _do_grasp(status_sock: socket.socket, host_addr: tuple[str, int], target_label: Optional[str]) -> None:
    """실제 SmolVLA 파지 호출 자리. 지금은 흉내만 낸다."""
    print(f"[bridge] GRASP 시작 (target={target_label}) — 여기에 실제 SmolVLA 호출을 넣을 것")
    # TODO: 실제 파지 로직으로 교체
    time.sleep(1.0)
    _send_status(status_sock, host_addr, "GRASP_DONE")


def _do_place(status_sock: socket.socket, host_addr: tuple[str, int]) -> None:
    """실제 SmolVLA 배치 호출 자리. 지금은 흉내만 낸다."""
    print("[bridge] PLACE 시작 — 여기에 실제 SmolVLA 호출을 넣을 것")
    # TODO: 실제 배치 로직으로 교체
    time.sleep(1.0)
    _send_status(status_sock, host_addr, "PLACE_DONE")


def _handle_cmd(msg: dict, status_sock: socket.socket, host_addr: tuple[str, int],
                busy: threading.Event) -> None:
    cmd = msg.get("cmd")
    status = msg.get("status")

    if status in ("GRASP", "PLACE") and not busy.is_set():
        # GRASP/PLACE 는 시간이 걸리는 동작이라 별도 스레드로 돌린다 — 그래야
        # 그 사이에도 계속 오는 UDP 명령(항상 "stop")을 안 놓치고 계속 받을
        # 수 있고, 워치독도 그동안 안 걸린다.
        busy.set()

        def _run():
            try:
                if status == "GRASP":
                    _do_grasp(status_sock, host_addr, msg.get("target_label"))
                else:
                    _do_place(status_sock, host_addr)
            finally:
                busy.clear()

        threading.Thread(target=_run, daemon=True).start()
        return

    if busy.is_set():
        return   # 파지/배치 중엔 이동 명령 무시(어차피 항상 "stop" 로 옴)

    # TODO: 실제 모터 제어로 교체
    if cmd == "go":
        pass   # 전진
    elif cmd == "stop":
        pass   # 정지
    elif cmd == "yaw+":
        pass   # 반시계 회전
    elif cmd == "yaw-":
        pass   # 시계 회전
    else:
        print(f"⚠️ 알 수 없는 cmd: {cmd!r}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cmd-port", type=int, default=5005)
    ap.add_argument("--status-port", type=int, default=5006)
    ap.add_argument("--watchdog-sec", type=float, default=1.5,
                     help="이 시간 안에 새 명령이 안 오면 정지 처리(Host 쪽과 실측해서 맞출 것)")
    args = ap.parse_args()

    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.bind(("0.0.0.0", args.cmd_port))
    cmd_sock.settimeout(0.5)   # 워치독 검사를 위해 주기적으로 깨어나게

    status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"[bridge] 명령 수신 대기 :{args.cmd_port}  상태 응답 포트 :{args.status_port}  "
          f"워치독 {args.watchdog_sec}초")

    busy = threading.Event()
    host_addr: Optional[tuple[str, int]] = None
    last_recv_t = 0.0

    try:
        while not _stop:
            try:
                data, addr = cmd_sock.recvfrom(4096)
            except socket.timeout:
                # 워치독: 명령을 한 번이라도 받은 적 있는데 그 뒤로 오래 끊겼으면 정지
                if host_addr is not None and time.monotonic() - last_recv_t > args.watchdog_sec:
                    print("\r[bridge] ⚠️ 워치독 — Host 신호 끊김, 정지   ", end="", flush=True)
                    # TODO: 실제 정지 명령으로 교체
                continue

            last_recv_t = time.monotonic()
            host_addr = (addr[0], args.status_port)   # 상태는 이 포트로 돌려보냄

            try:
                msg = json.loads(data)
            except json.JSONDecodeError:
                print(f"⚠️ 잘못된 패킷: {data!r}")
                continue

            _handle_cmd(msg, status_sock, host_addr, busy)
    except KeyboardInterrupt:
        pass
    finally:
        cmd_sock.close()
        status_sock.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
