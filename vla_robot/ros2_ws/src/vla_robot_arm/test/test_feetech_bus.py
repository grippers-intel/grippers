"""feetech_bus 패킷 계층 테스트 — 실제 시리얼 없이 가짜 포트로 확인한다."""
import pytest

from vla_robot_arm import feetech_bus as fb


class FakeSerial:
    """쓴 패킷에 미리 정한 응답을 돌려준다."""

    def __init__(self, responder):
        self.responder = responder
        self.rx = bytearray()
        self.written = []
        self.is_open = True

    def reset_input_buffer(self):
        self.rx.clear()

    def reset_output_buffer(self):
        pass

    def write(self, data):
        self.written.append(bytes(data))
        self.rx.extend(self.responder(bytes(data)))

    def flush(self):
        pass

    def read(self, n):
        out = bytes(self.rx[:n])
        del self.rx[:n]
        return out

    def close(self):
        self.is_open = False


def status(servo_id, params=b"", err=0, noise=b""):
    body = [servo_id, len(params) + 2, err, *params]
    return noise + bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])


def make_bus(responder):
    bus = fb.FeetechBus("fake", retries=2, timeout_s=0.001)
    bus._ser = FakeSerial(responder)
    return bus


def test_checksum_matches_known_ping_packet():
    # 공식 예: ID 1 PING = FF FF 01 02 01 FB
    assert fb.FeetechBus.build_packet(1, fb.INST_PING) == bytes([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB])


def test_read_position_parses_with_leading_noise():
    bus = make_bus(lambda pkt: status(pkt[2], bytes([0xD0, 0x07]), noise=b"\x00\x13"))
    assert bus.read_position(3) == 2000


def test_error_bits_do_not_drop_data():
    # 과부하(0x20)가 켜져도 위치는 읽혀야 한다 — 물체를 쥐고 있을 때 정상 상황
    bus = make_bus(lambda pkt: status(pkt[2], bytes([0x00, 0x08]), err=0x20))
    assert bus.read_position(6) == 2048
    assert bus.stats.last_error[6] == 0x20


def test_bad_checksum_is_rejected_and_retried():
    calls = []

    def responder(pkt):
        calls.append(pkt)
        good = status(pkt[2], bytes([0x01, 0x00]))
        return good[:-1] + bytes([(good[-1] + 1) & 0xFF])

    bus = make_bus(responder)
    assert bus.read_position(1) is None
    assert len(calls) == 2  # retries=2


def test_wrong_id_is_rejected():
    bus = make_bus(lambda pkt: status(9, bytes([0x01, 0x00])))
    assert bus.read_position(1) is None


def test_negative_present_position_sign_magnitude():
    raw = fb.encode_sign_magnitude(-5, 15)
    bus = make_bus(lambda pkt: status(pkt[2], bytes([raw & 0xFF, raw >> 8])))
    assert bus.read_position(2) == -5


def test_homing_offset_sign_bit_11():
    assert fb.decode_sign_magnitude(fb.encode_sign_magnitude(-1917, 11), 11) == -1917
    with pytest.raises(ValueError):
        fb.encode_sign_magnitude(2048, 11)


def test_sync_write_packet_layout():
    bus = make_bus(lambda pkt: b"")
    bus.write_goal_positions({1: 2048, 2: 5000})   # 5000 은 4095 로 잘린다
    pkt = bus._ser.written[-1]
    assert pkt[:5] == bytes([0xFF, 0xFF, fb.BROADCAST_ID, 2 * 3 + 4, fb.INST_SYNC_WRITE])
    assert pkt[5:7] == bytes([fb.ADDR_GOAL_POSITION, 2])
    assert pkt[7:10] == bytes([1, 0x00, 0x08])
    assert pkt[10:13] == bytes([2, 0xFF, 0x0F])
    assert pkt[-1] == (~sum(pkt[2:-1])) & 0xFF
