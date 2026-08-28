"""UdpHostLink가 **명령을 보낸 쪽**으로 보고하는지.

## 왜 이 테스트가 있나

`host_ip` 기본값이 작성자 개발 PC 주소(192.168.0.10)로 굳어 있었고,
`bringup.launch.py`가 그 인자를 노출하지 않았다. 그래서 다른 사람이 Host를
띄우면 **명령은 가는데 보고는 남의 PC로 갔다.**

명령이 단방향이라 증상이 고약하다 — 차는 정상적으로 움직이고, Host만 아무
보고도 못 받아 GRASP에서 영원히 멈춘다. 링크가 끊긴 것처럼 보이지만 실제로는
절반만 연결된 상태다.

"우리를 조종하는 쪽에 보고한다"로 바꾸면 설정할 것이 없어진다. 이 테스트는
그 규칙과, 그 규칙이 **아무 패킷에나 끌려가지 않는다**는 것을 같이 지킨다.
"""
import json
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from domain.adapters.real.udp_host_link import UdpHostLink

WRONG = "127.0.0.2"      # 다른 팀원 PC 역할
OURS = "127.0.0.1"
_PORT = [15005]          # 테스트마다 다른 포트를 쓴다


def _ports():
    _PORT[0] += 10
    return _PORT[0], _PORT[0] + 1


def _rx_socket(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((OURS, port))
    s.settimeout(0.6)
    return s


def _send_command(cmd_port):
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.bind((OURS, 0))
    tx.sendto(json.dumps({"state": "APPROACH", "linear_x": 0.1, "linear_y": 0.0,
                          "angular_z": 0.0, "stop": False}).encode(),
              (OURS, cmd_port))
    time.sleep(0.3)
    return tx


def test_reports_go_to_the_initial_host_before_any_command():
    """첫 명령 전에는 설정된 초기값으로 간다 — 조용히 사라지면 안 된다."""
    cmd_port, status_port = _ports()
    pi = UdpHostLink(WRONG, command_port=cmd_port, status_port=status_port)
    rx = _rx_socket(status_port)
    pi.report("STATE", "IDLE", "명령 전")
    try:
        rx.recvfrom(4096)
        raise AssertionError("초기값이 아니라 우리에게 왔다")
    except socket.timeout:
        pass
    finally:
        rx.close()


def test_reports_follow_whoever_sends_a_command():
    cmd_port, status_port = _ports()
    pi = UdpHostLink(WRONG, command_port=cmd_port, status_port=status_port)
    rx = _rx_socket(status_port)
    try:
        _send_command(cmd_port)
        assert pi.latest_command() is not None, "Pi 가 명령을 못 받았다"
        pi.report("GRASP_DONE", "GRASP", "명령 후")
        payload, _ = rx.recvfrom(4096)
        assert json.loads(payload)["report"] == "GRASP_DONE"
    finally:
        rx.close()


def test_unparseable_packets_do_not_redirect_reports():
    """아무 UDP 패킷이나 보낸다고 보고가 그쪽으로 따라가면 안 된다."""
    cmd_port, status_port = _ports()
    pi = UdpHostLink(WRONG, command_port=cmd_port, status_port=status_port)
    before = pi._host
    junk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    junk.bind(("127.0.0.3", 0))
    junk.sendto(b"not json at all", (OURS, cmd_port))
    time.sleep(0.3)
    assert pi._host == before, f"쓰레기 패킷에 {pi._host} 로 끌려갔다"


def test_following_can_be_turned_off():
    """고정하고 싶으면 끌 수 있어야 한다."""
    cmd_port, status_port = _ports()
    pi = UdpHostLink(WRONG, command_port=cmd_port, status_port=status_port,
                     follow_commander=False)
    _send_command(cmd_port)
    assert pi._host == (WRONG, status_port)


if __name__ == "__main__":
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in fns:
        try:
            fn()
            print(f"  PASS  {name}")
        except AssertionError as exc:
            failed += 1
            print(f"  FAIL  {name}: {exc}")
    print(f"\n{len(fns) - failed}/{len(fns)} 통과")
    sys.exit(1 if failed else 0)
