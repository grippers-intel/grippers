"""Feetech STS3215 반이중 시리얼 버스 드라이버 (SCS 프로토콜 0).

기존 코드에는 같은 버스를 여는 방법이 네 가지(직접 구현, scservo_sdk,
lerobot FeetechMotorsBus, lerobot SOFollower) 있었다. 여기서는 이 파일 하나만 쓴다.

    TX: FF FF ID LEN INST PARAM... CHK      LEN = PARAM 수 + 2
    RX: FF FF ID LEN ERR  PARAM... CHK      CHK = ~(ID + LEN + ... ) & 0xFF

## 기존 드라이버와 다른 점 (의도적)

- **포트 독점.** POSIX 에서 `exclusive=True` 로 연다. 다른 프로세스가 이미 열었으면
  여기서 예외가 난다. 두 프로세스가 같은 버스에 쓰면 패킷이 섞여 팔 이동이 통째로
  깨지는데, 기존에는 그걸 막을 장치가 없었다.
- **응답을 헤더부터 찾아 체크섬까지 검증한다.** 기존 구현은 고정 길이만 읽고
  체크섬을 안 봤다.
- **ERR 바이트가 0 이 아니어도 데이터를 버리지 않는다.** 과부하 비트(0x20)는
  물체를 꽉 쥐었을 때 정상적으로 켜진다. 기존 구현은 그때 위치 읽기가 실패했다.
  오류 비트는 `last_error` 에 남긴다.
- **목표 위치는 SYNC_WRITE 한 패킷으로 6개를 동시에 쓴다.** 30Hz 재생에서 서보마다
  write+응답 대기를 하면 관절 사이에 시간차가 생긴다.
"""
from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field
from typing import Iterable, Mapping, Optional

import serial

BROADCAST_ID = 0xFE

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_WRITE = 0x83

# STS3215 컨트롤 테이블 (lerobot feetech tables 와 같은 주소)
ADDR_MIN_POSITION_LIMIT = 9      # 2 bytes, EEPROM
ADDR_MAX_POSITION_LIMIT = 11     # 2 bytes, EEPROM
ADDR_HOMING_OFFSET = 31          # 2 bytes, EEPROM, 부호 비트 11
ADDR_TORQUE_ENABLE = 40          # 1 byte
ADDR_ACCELERATION = 41           # 1 byte
ADDR_GOAL_POSITION = 42          # 2 bytes, 부호 비트 15
ADDR_GOAL_TIME = 44              # 2 bytes
ADDR_GOAL_VELOCITY = 46          # 2 bytes, 부호 비트 15
ADDR_LOCK = 55                   # 1 byte, 0 = EEPROM 쓰기 허용
ADDR_PRESENT_POSITION = 56       # 2 bytes, 부호 비트 15
ADDR_PRESENT_VELOCITY = 58       # 2 bytes, 부호 비트 15
ADDR_PRESENT_LOAD = 60           # 2 bytes, 크기 10비트 + 방향 비트 10
ADDR_PRESENT_VOLTAGE = 62        # 1 byte, 0.1V
ADDR_PRESENT_TEMPERATURE = 63    # 1 byte, °C
ADDR_MOVING = 66                 # 1 byte

POSITION_MIN, POSITION_MAX = 0, 4095
LOAD_MAX = 1023
HOMING_MAX_MAGNITUDE = 2047

ERROR_BITS = {
    0x01: "voltage", 0x02: "angle", 0x04: "overheat", 0x08: "overele",
    0x20: "overload",
}


class BusError(RuntimeError):
    pass


def decode_sign_magnitude(value: int, sign_bit: int) -> int:
    magnitude = value & ((1 << sign_bit) - 1)
    return -magnitude if (value >> sign_bit) & 1 else magnitude


def encode_sign_magnitude(value: int, sign_bit: int) -> int:
    limit = (1 << sign_bit) - 1
    if abs(value) > limit:
        raise ValueError(f"값 {value} 가 부호-크기 {sign_bit}비트 범위(±{limit}) 밖이다")
    return (1 << sign_bit) | abs(value) if value < 0 else value


def describe_error(err: int) -> str:
    names = [name for bit, name in ERROR_BITS.items() if err & bit]
    return ",".join(names) if names else ("0" if err == 0 else hex(err))


@dataclass
class BusStats:
    reads: int = 0
    read_failures: int = 0
    writes: int = 0
    write_failures: int = 0
    last_error: dict = field(default_factory=dict)   # servo id -> ERR 바이트


class FeetechBus:
    def __init__(self, port: str, baudrate: int = 1_000_000, timeout_s: float = 0.02,
                 retries: int = 3) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self.retries = max(1, int(retries))
        self.stats = BusStats()
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.RLock()

    # -- 연결 ---------------------------------------------------------------
    def open(self) -> "FeetechBus":
        kwargs = dict(port=self.port, baudrate=self.baudrate, timeout=self.timeout_s,
                      write_timeout=0.1, bytesize=serial.EIGHTBITS,
                      parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        if os.name == "posix":
            kwargs["exclusive"] = True
        try:
            self._ser = serial.Serial(**kwargs)
        except (serial.SerialException, OSError) as exc:
            raise BusError(f"{self.port} 를 열지 못했다 (다른 프로세스가 쓰는 중인지 확인): {exc}") from exc
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        return self

    def close(self) -> None:
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                finally:
                    self._ser = None

    def __enter__(self) -> "FeetechBus":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    # -- 패킷 ---------------------------------------------------------------
    @staticmethod
    def _checksum(body: Iterable[int]) -> int:
        return (~sum(body)) & 0xFF

    @classmethod
    def build_packet(cls, servo_id: int, instruction: int, params: Iterable[int] = ()) -> bytes:
        params = list(params)
        body = [servo_id & 0xFF, len(params) + 2, instruction, *[p & 0xFF for p in params]]
        return bytes([0xFF, 0xFF, *body, cls._checksum(body)])

    def _read_exact(self, n: int, deadline: float) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            if time.monotonic() > deadline:
                break
            chunk = self._ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
        return bytes(buf)

    def _read_status(self, servo_id: int, n_params: int) -> Optional[tuple[int, bytes]]:
        """응답 하나를 읽는다. (ERR, PARAMS) 또는 None."""
        deadline = time.monotonic() + self.timeout_s * 3
        # 헤더 FF FF 를 찾는다. 앞에 잡음 바이트가 있을 수 있다.
        prev = None
        while True:
            b = self._read_exact(1, deadline)
            if not b:
                return None
            if prev == 0xFF and b[0] == 0xFF:
                break
            prev = b[0]
        head = self._read_exact(2, deadline)             # ID, LEN
        if len(head) < 2:
            return None
        rid, length = head[0], head[1]
        if length < 2:
            return None
        rest = self._read_exact(length, deadline)        # ERR, PARAMS..., CHK
        if len(rest) < length:
            return None
        if self._checksum([rid, length, *rest[:-1]]) != rest[-1]:
            return None
        if rid != servo_id:
            return None
        err, params = rest[0], rest[1:-1]
        if len(params) < n_params:
            return None
        return err, bytes(params[:n_params])

    def _transact(self, servo_id: int, instruction: int, params: Iterable[int],
                  n_response_params: int) -> Optional[tuple[int, bytes]]:
        packet = self.build_packet(servo_id, instruction, params)
        with self._lock:
            if not self.is_open:
                raise BusError("버스가 열려 있지 않다")
            self._ser.reset_input_buffer()
            self._ser.write(packet)
            self._ser.flush()
            if servo_id == BROADCAST_ID:
                return None
            status = self._read_status(servo_id, n_response_params)
        if status is not None:
            err = status[0]
            if err:
                self.stats.last_error[servo_id] = err
            else:
                self.stats.last_error.pop(servo_id, None)
        return status

    # -- 레지스터 ------------------------------------------------------------
    def ping(self, servo_id: int) -> bool:
        for _ in range(self.retries):
            if self._transact(servo_id, INST_PING, (), 0) is not None:
                return True
        return False

    def read(self, servo_id: int, addr: int, size: int) -> Optional[int]:
        for _ in range(self.retries):
            self.stats.reads += 1
            status = self._transact(servo_id, INST_READ, (addr, size), size)
            if status is not None:
                data = status[1]
                return data[0] if size == 1 else data[0] | (data[1] << 8)
            self.stats.read_failures += 1
        return None

    def write(self, servo_id: int, addr: int, size: int, value: int) -> bool:
        value = int(value)
        data = [value & 0xFF] if size == 1 else [value & 0xFF, (value >> 8) & 0xFF]
        for _ in range(self.retries):
            self.stats.writes += 1
            if self._transact(servo_id, INST_WRITE, (addr, *data), 0) is not None:
                return True
            self.stats.write_failures += 1
        return False

    def sync_write(self, addr: int, size: int, values: Mapping[int, int]) -> None:
        """응답이 없는 브로드캐스트. 유실은 다음 스텝이 덮는다."""
        if not values:
            return
        params = [addr, size]
        for servo_id, value in values.items():
            value = int(value)
            params.append(servo_id)
            params.append(value & 0xFF)
            if size == 2:
                params.append((value >> 8) & 0xFF)
        self.stats.writes += 1
        self._transact(BROADCAST_ID, INST_SYNC_WRITE, params, 0)

    # -- 고수준 --------------------------------------------------------------
    def read_position(self, servo_id: int) -> Optional[int]:
        raw = self.read(servo_id, ADDR_PRESENT_POSITION, 2)
        return None if raw is None else decode_sign_magnitude(raw, 15)

    def read_positions(self, ids: Iterable[int]) -> Optional[list[int]]:
        """전부 읽혀야 값을 준다. 일부만 맞는 관측으로 정책을 돌리면 조용히 틀린다."""
        out = []
        for servo_id in ids:
            pos = self.read_position(servo_id)
            if pos is None:
                return None
            out.append(pos)
        return out

    def write_goal_positions(self, goals: Mapping[int, int]) -> None:
        clamped = {sid: max(POSITION_MIN, min(POSITION_MAX, int(v))) for sid, v in goals.items()}
        self.sync_write(ADDR_GOAL_POSITION, 2, clamped)

    def set_torque(self, ids: Iterable[int], enable: bool) -> bool:
        return all(self.write(sid, ADDR_TORQUE_ENABLE, 1, 1 if enable else 0) for sid in ids)

    def read_torque(self, servo_id: int) -> Optional[bool]:
        v = self.read(servo_id, ADDR_TORQUE_ENABLE, 1)
        return None if v is None else bool(v)

    def set_acceleration(self, ids: Iterable[int], value: int) -> None:
        self.sync_write(ADDR_ACCELERATION, 1, {sid: max(0, min(254, int(value))) for sid in ids})

    def set_goal_velocity(self, ids: Iterable[int], value: int) -> None:
        """0 = 무제한. ⚠️ 다른 도구가 남긴 값이 RAM 에 남아 정책 재생을 느리게 만든
        사고가 있었다(2026-09-02). 재생 시작마다 명시적으로 쓴다."""
        self.sync_write(ADDR_GOAL_VELOCITY, 2, {sid: max(0, min(32767, int(value))) for sid in ids})

    def read_load_ratio(self, servo_id: int) -> Optional[float]:
        raw = self.read(servo_id, ADDR_PRESENT_LOAD, 2)
        return None if raw is None else min(1.0, (raw & 0x3FF) / LOAD_MAX)

    def read_voltage(self, servo_id: int) -> Optional[float]:
        raw = self.read(servo_id, ADDR_PRESENT_VOLTAGE, 1)
        return None if raw is None else raw / 10.0

    def read_temperature(self, servo_id: int) -> Optional[int]:
        return self.read(servo_id, ADDR_PRESENT_TEMPERATURE, 1)

    def read_moving(self, servo_id: int) -> Optional[bool]:
        v = self.read(servo_id, ADDR_MOVING, 1)
        return None if v is None else bool(v)

    def read_homing_offset(self, servo_id: int) -> Optional[int]:
        raw = self.read(servo_id, ADDR_HOMING_OFFSET, 2)
        return None if raw is None else decode_sign_magnitude(raw, 11)

    def read_position_limits(self, servo_id: int) -> Optional[tuple[int, int]]:
        lo = self.read(servo_id, ADDR_MIN_POSITION_LIMIT, 2)
        hi = self.read(servo_id, ADDR_MAX_POSITION_LIMIT, 2)
        return None if lo is None or hi is None else (lo, hi)

    def write_position_limits(self, servo_id: int, low: int, high: int) -> bool:
        """EEPROM 쓰기(Min/Max_Position_Limit). lerobot write_calibration 이 range_min/max 를
        여기에 쓴다. 토크가 꺼져 있어야 한다."""
        if not (POSITION_MIN <= low < high <= POSITION_MAX):
            raise ValueError(f"위치 한계가 잘못됐다: {low}..{high}")
        if self.read_torque(servo_id):
            raise BusError(f"servo {servo_id}: 토크가 켜진 채로 EEPROM 을 쓰지 않는다")
        ok = self.write(servo_id, ADDR_LOCK, 1, 0)
        ok = ok and self.write(servo_id, ADDR_MIN_POSITION_LIMIT, 2, low)
        ok = ok and self.write(servo_id, ADDR_MAX_POSITION_LIMIT, 2, high)
        self.write(servo_id, ADDR_LOCK, 1, 1)
        time.sleep(0.02)
        return ok and self.read_position_limits(servo_id) == (low, high)

    def write_homing_offset(self, servo_id: int, offset: int) -> bool:
        """EEPROM 쓰기. 토크가 꺼져 있어야 한다. 쓰고 나서 다시 읽어 확인한다."""
        if abs(int(offset)) > HOMING_MAX_MAGNITUDE:
            raise ValueError(f"homing offset {offset} 가 ±{HOMING_MAX_MAGNITUDE} 밖이다")
        if self.read_torque(servo_id):
            raise BusError(f"servo {servo_id}: 토크가 켜진 채로 EEPROM 을 쓰지 않는다")
        encoded = encode_sign_magnitude(int(offset), 11)
        ok = self.write(servo_id, ADDR_LOCK, 1, 0)
        ok = ok and self.write(servo_id, ADDR_HOMING_OFFSET, 2, encoded)
        self.write(servo_id, ADDR_LOCK, 1, 1)
        time.sleep(0.02)
        return ok and self.read_homing_offset(servo_id) == int(offset)
