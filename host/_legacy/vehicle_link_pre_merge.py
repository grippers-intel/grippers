"""Host PC -> 자율주행 차량으로 미션 명령을 보내는 자리.

실제 전송 방식(ROS2 토픽, 브리지 등)은 차량과 통신하는 컴퓨터에서 정한다.
여기서는 "보낼 내용"의 모양(MissionCommand)만 고정해두고, 전송은 인터페이스
뒤로 숨긴다 — 나중에 전송 방식이 정해지면 VehicleLink 를 상속하는 구현체
하나만 추가하면 되고, mission.py 쪽 로직은 손댈 필요가 없다.

명령은 두 필드로 나뉜다:
  - cmd    : 이번 순간 바퀴가 뭘 해야 하는지 ("go"/"stop"/"yaw+"/"yaw-") —
             Host 가 ArUco 로 매 사이클 계산해서 넘기므로, 차량 쪽은 각도
             계산을 전혀 안 해도 된다. "yaw+"/"yaw-" 는 그냥 그 방향으로
             제자리 회전하고 있다가 "stop" 이 오면 바로 멈추면 된다.
  - status : 지금 미션이 어느 단계인지(mission.State 이름과 동일:
             SEARCH_TARGET/APPROACH_PIECE/GRASP/CARRY_TO_DEST/FACE_BOX/
             PLACE) — cmd 만으로는 "지금 왜 멈춰 있는지"(정렬 중인지,
             파지 중인지, 그냥 도착 대기인지)를 구분 못 하므로 같이 보낸다.
             GRASP/PLACE 일 때는 cmd 는 항상 "stop" 이고, 그 상태에서
             SmolVLA 로 집기/내려놓기를 하는 건 차량 쪽 몫이다.

Host PC 는 파지·배치 동작 자체를 계산하지 않는다. status 를 "GRASP"/"PLACE"
로 보내는 것까지가 Host PC 의 역할이고, 그 다음은 차량의 SmolVLA 가 자기
카메라(그리퍼캠 + 차량 RGB캠)로 알아서 한다. poll_status() 는 차량이 그
동작을 끝냈는지만 알려주면 된다.
"""

from __future__ import annotations

import json
import socket
import time
from dataclasses import asdict, dataclass, field
from typing import Optional


@dataclass
class MissionCommand:
    cmd: str                           # "go" | "stop" | "yaw+" | "yaw-"
    status: str                        # 지금 미션 단계 (mission.State 이름)
    robot_x: float
    robot_y: float
    robot_yaw_deg: float
    target_label: Optional[str] = None     # status="GRASP" 일 때 무엇을 집을지
    fresh: bool = True                     # 워치독용 — 매 전송마다 True 로 보낸다
    t: float = field(default_factory=time.monotonic)


class VehicleLink:
    """전송 어댑터의 추상 인터페이스."""

    def send(self, cmd: MissionCommand) -> None:
        raise NotImplementedError

    def poll_status(self) -> str:
        """차량이 보고하는 상태.

        "IDLE" | "BUSY" | "GRASP_DONE" | "PLACE_DONE" | "FAILED" 중 하나를
        기대한다. 구체적인 보고 방식(ROS2 서비스/토픽 등)은 전송 어댑터가
        정한다.
        """
        raise NotImplementedError


class ConsoleVehicleLink(VehicleLink):
    """전송 방식이 정해지기 전, mission.py 로직만 로봇 없이 시험하기 위한 자리표시자.

    실제 차량 없이 돌리면 GRASP/PLACE 명령을 보내는 즉시 완료된 것으로 치고
    바로 다음 상태로 넘어가도록 흉내낸다. 진짜 전송 어댑터가 생기면 이 클래스
    대신 그걸 mission.py 에 넘기면 된다.
    """

    def __init__(self, auto_complete: bool = True) -> None:
        self._auto_complete = auto_complete
        self._pending_done: Optional[str] = None

    def send(self, cmd: MissionCommand) -> None:
        extra = f"target={cmd.target_label}" if cmd.target_label else ""
        print(f"\r[vehicle_link] {cmd.cmd:5s} [{cmd.status:14s}] "
              f"robot=({cmd.robot_x:6.3f},{cmd.robot_y:6.3f},{cmd.robot_yaw_deg:6.1f}°) "
              f"{extra}   ",
              end="", flush=True)
        if self._auto_complete and cmd.status in ("GRASP", "PLACE"):
            self._pending_done = f"{cmd.status}_DONE"

    def poll_status(self) -> str:
        if self._pending_done:
            status, self._pending_done = self._pending_done, None
            return status
        return "IDLE"


class UdpVehicleLink(VehicleLink):
    """VEHICLE_LINK_PROTOCOL.md 그대로: 명령은 UDP 로 Pi 에 쏘고(포트 5005),
    상태는 UDP 로 받는다(포트 5006). 둘 다 JSON.

    UDP 라 send() 는 상대가 없어도(Pi 가 아직 안 켜져 있어도) 그냥 조용히
    나가고 예외가 안 난다 — 워치독은 받는 쪽(Pi) 책임이다.

    poll_status() 는 논블로킹이다 — 그 사이 들어온 상태 패킷이 여러 개면
    가장 최근 것만 쓰고 나머지는 버린다(오래된 상태를 뒤늦게 처리하지
    않기 위함, 이 프로젝트 전체의 "최신 것만 믿는다" 철학과 동일).
    """

    def __init__(self, pi_ip: str, cmd_port: int = 5005, status_port: int = 5006,
                 bind_ip: str = "0.0.0.0") -> None:
        self.pi_ip = pi_ip
        self.cmd_port = cmd_port

        self._send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock.setblocking(False)
        self._recv_sock.bind((bind_ip, status_port))

    def send(self, cmd: MissionCommand) -> None:
        payload = json.dumps(asdict(cmd)).encode("utf-8")
        try:
            self._send_sock.sendto(payload, (self.pi_ip, self.cmd_port))
        except OSError as exc:
            # 네트워크가 잠깐 끊겨도 미션 루프 자체는 안 죽어야 한다 —
            # 다음 사이클에 다시 시도된다.
            print(f"⚠️ UdpVehicleLink: 전송 실패 — {exc}")

    def poll_status(self) -> str:
        latest = None
        while True:
            try:
                data, _addr = self._recv_sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as exc:
                print(f"⚠️ UdpVehicleLink: 수신 오류 — {exc}")
                break
            try:
                latest = json.loads(data)["status"]
            except (json.JSONDecodeError, KeyError):
                continue   # 잘못된 패킷은 무시하고 다음 것 확인
        return latest if latest is not None else "IDLE"

    def close(self) -> None:
        self._send_sock.close()
        self._recv_sock.close()
