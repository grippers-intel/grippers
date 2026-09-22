"""Pi 쪽 UDP 링크 — Host 명령 수신, 상태 송신. ROS 에 의존하지 않는다.

## 보고 대상은 "마지막으로 유효한 명령을 보낸 주소"

기존에는 host_ip 기본값이 한 사람의 개발 PC 주소라, 다른 사람이 Host 를 띄우면 명령은
오는데 보고가 남의 PC 로 갔다. 파싱에 **성공한** 명령의 출처만 따라가므로 아무 패킷이나
보낸다고 보고가 새지 않는다. 고정하려면 fixed_host_ip 를 준다.

## 최신 것만

속도 명령은 순간값이라 큐를 쌓지 않는다. 수신 스레드는 가장 최근 것 하나만 덮어쓴다.
"""
from __future__ import annotations

import socket
import threading
import time
from typing import Callable, Optional

from vla_common.protocol import MAX_PACKET_BYTES, HostCommand, PiStatus, ProtocolError


class UdpLink:
    def __init__(self, bind_ip: str, command_port: int, status_port: int,
                 fixed_host_ip: str = "", log: Callable[[str], None] = print) -> None:
        self._status_port = status_port
        self._fixed = fixed_host_ip or ""
        self._host: Optional[tuple[str, int]] = (self._fixed, status_port) if self._fixed else None
        self._log = log
        self._lock = threading.Lock()
        self._latest: Optional[tuple[HostCommand, float]] = None
        self._fresh = False
        self.bad_packets = 0
        self.last_bad_reason = ""

        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.bind((bind_ip, command_port))
        self._rx.settimeout(0.2)
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="udp_link_rx", daemon=True)
        self._thread.start()

    @property
    def host_address(self) -> str:
        with self._lock:
            return "" if self._host is None else f"{self._host[0]}:{self._host[1]}"

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, addr = self._rx.recvfrom(MAX_PACKET_BYTES + 1)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                continue
            try:
                cmd = HostCommand.from_bytes(data)
            except ProtocolError as exc:
                self.bad_packets += 1
                self.last_bad_reason = str(exc)
                continue
            with self._lock:
                self._latest = (cmd, time.monotonic())
                self._fresh = True
                if not self._fixed:
                    target = (addr[0], self._status_port)
                    if target != self._host:
                        self._log(f"보고 대상: {target[0]}:{target[1]} (명령을 보낸 쪽)")
                        self._host = target

    def take_new(self) -> Optional[tuple[HostCommand, float]]:
        """아직 안 읽은 최신 명령과 수신 시각(monotonic). 없으면 None."""
        with self._lock:
            if not self._fresh:
                return None
            self._fresh = False
            return self._latest

    def send_status(self, status: PiStatus) -> bool:
        with self._lock:
            host = self._host
        if host is None:
            return False
        try:
            self._tx.sendto(status.to_bytes(), host)
            return True
        except OSError as exc:
            self._log(f"상태 송신 실패: {exc}")
            return False

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._rx.close()
        self._tx.close()
