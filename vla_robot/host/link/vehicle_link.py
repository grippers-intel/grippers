"""Host <-> Pi UDP 링크. 규격은 `vla_common.protocol` 하나뿐이다.

## 최신 것만 본다
속도 명령은 그 순간의 값이라 오래된 패킷은 쓸모가 없다. 수신도 큐를 쌓지 않고 가장
최근 유효 PiStatus 만 남긴다 — 사건(작업 결과)을 잃을 걱정은 없다. Pi 가 결과를 매
패킷 반복하기 때문이다(protocol.JobTracker 참고).

## 안 닿아도 예외를 내지 않는다
Pi 가 아직 안 켜져 있어도 send() 는 조용히 나간다. 링크 단절 판정은 받는 쪽(Pi)의
워치독 몫이다 — Host 가 말을 멈추면 차도 멈춘다.

## 종료할 때 stop 을 연발한다
그냥 닫으면 Pi 워치독이 설 때까지(0.5 s) 바퀴가 돈다. UDP 는 한 발이 빠질 수 있다.
"""
from __future__ import annotations

import socket
import time
from dataclasses import replace
from typing import Optional

from vla_common.protocol import (MAX_PACKET_BYTES, HostCommand, PiStatus, ProtocolError,
                                 State)

_WARN_REPEAT_S = 5.0


class _RateLimitedWarn:
    """같은 문구는 5초마다 한 번, 그동안 몇 번 더 났는지와 함께. 눌러 버리지 않고
    세어서 보여주는 이유: 경고가 상시가 됐다는 사실 자체가 진단 정보다."""

    def __init__(self) -> None:
        self._seen: dict[str, tuple[float, int]] = {}

    def __call__(self, msg: str) -> None:
        now = time.monotonic()
        last, count = self._seen.get(msg, (-1e9, 0))
        if now - last < _WARN_REPEAT_S:
            self._seen[msg] = (last, count + 1)
            return
        suffix = f" (직전 {_WARN_REPEAT_S:.0f}초간 {count}회 더)" if count else ""
        print(f"[link] {msg}{suffix}")
        self._seen[msg] = (now, 0)


class UdpVehicleLink:
    def __init__(self, pi_ip: str, command_port: int, status_port: int,
                 bind_ip: str = "0.0.0.0", stop_burst: int = 8) -> None:
        self.pi_ip = pi_ip
        self.command_port = command_port
        self.stop_burst = stop_burst
        self._seq = 0
        self._last_state = State.IDLE
        self._status: Optional[PiStatus] = None
        self._status_t: Optional[float] = None
        self._warn = _RateLimitedWarn()
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.bind((bind_ip, status_port))
        self._rx.setblocking(False)
        self._closed = False

    # -- 송신 -------------------------------------------------------------
    def send(self, cmd: HostCommand) -> None:
        self._seq += 1
        self._last_state = cmd.state
        try:
            self._tx.sendto(replace(cmd, seq=self._seq).to_bytes(), (self.pi_ip, self.command_port))
        except OSError as exc:
            # 네트워크가 잠깐 끊겨도 미션 루프는 살아야 한다 — 다음 사이클에 다시 보낸다.
            self._warn(f"전송 실패: {exc}")

    # -- 수신 -------------------------------------------------------------
    def poll(self) -> Optional[PiStatus]:
        while True:
            try:
                data, addr = self._rx.recvfrom(MAX_PACKET_BYTES + 1)
            except BlockingIOError:
                break
            except OSError as exc:
                # Windows 는 ICMP port unreachable 을 ConnectionResetError(10054)로 올린다.
                if isinstance(exc, ConnectionResetError):
                    continue
                self._warn(f"수신 오류: {exc}")
                break
            if addr[0] != self.pi_ip:
                self._warn(f"Pi({self.pi_ip})가 아닌 {addr[0]} 에서 온 상태 패킷 — 버림")
                continue
            try:
                self._status = PiStatus.from_bytes(data)
                self._status_t = time.monotonic()
            except ProtocolError as exc:
                self._warn(f"Pi 상태 패킷 해석 실패 — 버림: {exc}")
        return self._status

    def latest_status(self) -> Optional[PiStatus]:
        return self.poll()

    def status_age_s(self) -> float:
        return float("inf") if self._status_t is None else time.monotonic() - self._status_t

    # -- 종료 -------------------------------------------------------------
    def send_stop_burst(self) -> None:
        # 작업 중이면 Pi 가 무시하므로 상태 이름은 마지막 것을 유지하되 stop 을 건다.
        state = self._last_state if self._last_state != State.ESTOP else State.IDLE
        for _ in range(self.stop_burst):
            self.send(HostCommand(state, stop=True))
            time.sleep(0.05)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.send_stop_burst()
            print(f"[link] 정지 명령 {self.stop_burst}회 송신")
        except Exception as exc:  # noqa: BLE001 — 종료 경로
            print(f"[link] 정지 명령 실패: {exc} — Pi 워치독이 세운다")
        self._tx.close()
        self._rx.close()


class ConsoleLink:
    """차량 없이 FSM 만 볼 때. 명령이 바뀔 때만 찍는다. 상태는 없으므로 GRASP 는
    host timeout 으로 끝난다 — 전체 흐름은 --sim 으로 볼 것."""

    def __init__(self) -> None:
        self._last = None

    def send(self, cmd: HostCommand) -> None:
        key = (cmd.state, round(cmd.linear_x, 3), round(cmd.linear_y, 3), round(cmd.angular_z, 3), cmd.stop)
        if key != self._last:
            print(f"[console-link] {cmd.state:12s} vx={cmd.linear_x:+.2f} vy={cmd.linear_y:+.2f} "
                  f"wz={cmd.angular_z:+.2f} stop={cmd.stop} label={cmd.label}")
            self._last = key

    def latest_status(self):
        return None

    def status_age_s(self) -> float:
        return float("inf")

    def close(self) -> None:
        pass
