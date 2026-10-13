import socket
import time

from vla_common.protocol import HostCommand, PiStatus, State
from vla_robot_mission.udp_link import UdpLink

CMD_PORT, STATUS_PORT = 25105, 25106


def wait(pred, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        v = pred()
        if v:
            return v
        time.sleep(0.01)
    return None


def test_receives_latest_and_reports_to_sender():
    link = UdpLink("127.0.0.1", CMD_PORT, STATUS_PORT, log=lambda m: None)
    host_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    host_rx.bind(("127.0.0.1", STATUS_PORT))
    host_rx.settimeout(2.0)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        tx.sendto(b"garbage", ("127.0.0.1", CMD_PORT))
        for i in range(3):
            tx.sendto(HostCommand(State.APPROACH, linear_x=0.1, seq=i).to_bytes(), ("127.0.0.1", CMD_PORT))
        time.sleep(0.2)
        got = wait(link.take_new)
        assert got is not None and got[0].seq == 2
        assert link.take_new() is None          # 한 번만 준다
        assert wait(lambda: link.bad_packets == 1)

        status = PiStatus("b", State.APPROACH, False, 0, None, True, False, ack_seq=2)
        assert link.send_status(status)
        data, _ = host_rx.recvfrom(4096)
        assert PiStatus.from_bytes(data) == status
    finally:
        link.close()
        host_rx.close()
        tx.close()
