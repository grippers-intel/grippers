import socket
import time

from link.vehicle_link import UdpVehicleLink
from vla_common.protocol import HostCommand, JobResult, PiStatus, State

CMD_PORT, STATUS_PORT = 15105, 15106


def _status(job_id):
    return PiStatus(boot_id="x", state=State.GRASP, busy=False, job_id=job_id,
                    result=JobResult(job_id, State.GRASP, True), base_ok=True, watchdog=False)


def test_udp_loopback():
    pi_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pi_rx.bind(("127.0.0.1", CMD_PORT))
    pi_rx.settimeout(1.0)
    pi_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    link = UdpVehicleLink("127.0.0.1", CMD_PORT, STATUS_PORT, bind_ip="127.0.0.1", stop_burst=3)
    try:
        link.send(HostCommand(State.APPROACH, linear_x=0.1))
        link.send(HostCommand(State.APPROACH, linear_x=0.1))
        first = HostCommand.from_bytes(pi_rx.recvfrom(4096)[0])
        second = HostCommand.from_bytes(pi_rx.recvfrom(4096)[0])
        assert (first.seq, second.seq) == (1, 2)
        assert first.state == State.APPROACH and first.linear_x == 0.1

        assert link.latest_status() is None
        pi_tx.sendto(b"garbage", ("127.0.0.1", STATUS_PORT))
        pi_tx.sendto(_status(1).to_bytes(), ("127.0.0.1", STATUS_PORT))
        pi_tx.sendto(_status(2).to_bytes(), ("127.0.0.1", STATUS_PORT))
        deadline = time.monotonic() + 1.0
        st = None
        while time.monotonic() < deadline:
            st = link.latest_status()
            if st is not None and st.job_id == 2:
                break
            time.sleep(0.01)
        assert st is not None and st.job_id == 2
        assert link.status_age_s() < 1.0
    finally:
        link.close()
    burst = [HostCommand.from_bytes(pi_rx.recvfrom(4096)[0]) for _ in range(3)]
    assert all(c.stop for c in burst)
    pi_rx.close()
    pi_tx.close()
