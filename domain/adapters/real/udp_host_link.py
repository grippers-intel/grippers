"""UdpHostLink — HostLink 포트의 실기 구현. Host PC와 UDP+JSON으로 말한다.

프로토콜은 Host 쪽 `VEHICLE_LINK_PROTOCOL.md`가 단일 소스다. 여기서는 팀이
2026-08-26에 확정한 **다섯 필드**만 읽는다:

    state · linear_x · linear_y · angular_z · stop

Host가 다른 필드를 더 보내도 무시한다 — 좌표나 경로가 섞여 들어오더라도
Pi가 그것을 읽기 시작하는 순간 역할 분담이 무너지기 때문이다.

## 왜 최신 것만 보는가

UDP는 순서도 도착도 보장하지 않는다. 그런데 이 링크가 실어 나르는 것은
**그 순간의 속도 명령**이라 오래된 패킷은 쓸모가 없다 — 재전송을 기다리는
것보다 다음 것을 쓰는 쪽이 항상 낫다. 그래서 수신 스레드는 큐를 쌓지 않고
마지막 것만 덮어쓴다.

## None은 정지가 아니다

`latest_command()`는 **아직 못 읽은 새 명령이 없으면 None**을 돌려준다.
이것을 "정지"로 바꿔서 돌려주지 않는 이유는, 링크가 끊긴 것과 Host가
정지를 지시한 것이 전혀 다른 사건이기 때문이다. 앞의 것은 워치독이
처리해야 하고(`baseline_mission.LinkWatchdog`), 뒤의 것은 정상 명령이다.
"""

import json
import socket
import threading

from domain.ports.baseline_ports import HostCommand

COMMAND_PORT = 5005
STATUS_PORT = 5006
RECV_BUFFER = 4096


class UdpHostLink:
    def __init__(self, host_ip: str, command_port: int = COMMAND_PORT,
                 status_port: int = STATUS_PORT, logger=None):
        self._host = (host_ip, status_port)
        self._logger = logger
        self._lock = threading.Lock()
        self._latest = None
        self._fresh = False

        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.bind(("0.0.0.0", command_port))
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()

    # --- 수신 ---------------------------------------------------------

    def _receive_loop(self):
        while not self._stop.is_set():
            try:
                payload, _addr = self._rx.recvfrom(RECV_BUFFER)
            except OSError:
                if self._stop.is_set():
                    return
                continue
            command = self._parse(payload)
            if command is None:
                continue
            with self._lock:
                self._latest = command
                self._fresh = True

    def _parse(self, payload):
        """망가진 패킷은 **버린다.** 반쯤 읽어서 0으로 채우면 그 0이 곧
        속도 명령이 되는데, 그것은 "정지"라는 뜻이 아니라 "모른다"다."""
        try:
            data = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self._warn("Host 패킷 파싱 실패 — 버림")
            return None
        state = data.get("state")
        if not isinstance(state, str):
            self._warn("Host 패킷에 state가 없다 — 버림")
            return None
        try:
            return HostCommand(
                state=state,
                linear_x=float(data.get("linear_x", 0.0)),
                linear_y=float(data.get("linear_y", 0.0)),
                angular_z=float(data.get("angular_z", 0.0)),
                stop=bool(data.get("stop", False)),
            )
        except (TypeError, ValueError):
            self._warn("Host 패킷의 속도 필드가 수치가 아니다 — 버림")
            return None

    def latest_command(self):
        """마지막으로 받은 **아직 안 읽은** 명령. 새것이 없으면 None."""
        with self._lock:
            if not self._fresh:
                return None
            self._fresh = False
            return self._latest

    # --- 송신 ---------------------------------------------------------

    def report(self, report: str, state: str, detail: str = "") -> None:
        """보고는 fire-and-forget이다 — 안 닿으면 Host 워치독이 판단한다."""
        payload = json.dumps(
            {"report": report, "state": state, "detail": detail},
            ensure_ascii=False).encode("utf-8")
        try:
            self._tx.sendto(payload, self._host)
        except OSError as exc:
            self._warn(f"Host 보고 전송 실패: {exc}")

    def close(self):
        self._stop.set()
        self._rx.close()
        self._tx.close()

    def _warn(self, message):
        if self._logger is not None:
            self._logger.warn(message)
