"""Host <-> Pi UDP 계약. **이 파일이 단일 원본이다.**

기존 코드에서는 같은 포트(5005/5006)에 서로 못 알아듣는 규격 두 벌이 돌았다.
여기서는 Host 와 Pi 가 이 모듈을 직접 import 하므로 규격이 갈라질 수 없다.

## 역할 분담 (기존 설계 유지)

- **Host** 가 공간을 소유한다: 로봇 pose, 물체 좌표, 경로, 속도 결정.
- **Pi** 는 속도를 실행하고, 팔 작업(GRASP / PLACE)을 수행하고, 결과를 보고한다.
- 그래서 명령에 좌표가 없다. 라벨은 정책 지시문("pick up the queen")에만 쓴다.

## 패킷

Host -> Pi (COMMAND_PORT, 10 Hz 이상):

    {"v":1, "seq":42, "state":"APPROACH",
     "linear_x":0.15, "linear_y":0.0, "angular_z":0.0, "stop":false, "label":"queen",
     "arm_yaw_deg":0.0}

Pi -> Host (STATUS_PORT, 10 Hz):

    {"v":1, "boot_id":"a1b2c3", "ack_seq":42, "state":"GRASP", "busy":true,
     "job_id":3, "result":{"job_id":2,"action":"GRASP","ok":true,"detail":"..."},
     "base_ok":true, "watchdog":false, "detail":""}

## 사건을 잃지 않는 법 — 결과를 매 패킷 반복한다

예전에는 GRASP_DONE 같은 완료 보고를 **한 번만** 보냈다. UDP 라 한 번 빠지거나
다른 보고에 덮이면 Host 가 GRASP 에서 영원히 기다렸다.

여기서는 Pi 가 마지막으로 끝난 작업의 결과(`result`)를 **매 상태 패킷에 계속
싣는다.** Host 는 GRASP 를 보내기 직전의 `result.job_id` 를 기억해 두었다가, 그보다
큰 job_id 의 결과가 보이면 완료로 본다. 패킷이 몇 개 빠져도 다음 패킷이 같은
결과를 다시 가져온다.

Pi 가 재시작하면 job_id 가 0 부터 다시 센다. 그래서 `boot_id` 를 같이 보내고,
Host 는 boot_id 가 바뀌면 기준 job_id 를 0 으로 되돌린다.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Optional

PROTOCOL_VERSION = 1
COMMAND_PORT = 5005
STATUS_PORT = 5006
MAX_PACKET_BYTES = 4096
MAX_LABEL_LEN = 32


class State:
    """Host 가 Pi 에 요구하는 상태. 문자열 그대로 전선에 실린다."""

    IDLE = "IDLE"                  # 할 일 없음. 속도 명령은 따른다(보통 stop)
    APPROACH = "APPROACH"          # 물체로 주행
    GRASP = "GRASP"                # 정지 + VLA 파지 작업
    CARRY = "CARRY"                # 물체를 든 채 주행
    APPROACH_BOX = "APPROACH_BOX"  # 상자 앞 미세 주행
    PLACE = "PLACE"                # 정지 + 내려놓기 작업
    ESTOP = "ESTOP"                # 즉시 정지, 진행 중인 팔 작업 취소

    DRIVE_STATES = (IDLE, APPROACH, CARRY, APPROACH_BOX)
    JOB_STATES = (GRASP, PLACE)
    ALL = (IDLE, APPROACH, GRASP, CARRY, APPROACH_BOX, PLACE, ESTOP)


class ProtocolError(ValueError):
    """패킷을 해석할 수 없다. **버린다** — 반쯤 읽어 0 으로 채우면 그 0 이 곧
    속도 명령이 되는데, 그건 "정지"가 아니라 "모른다"다."""


def _finite(name: str, value) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"{name} 가 수치가 아니다: {value!r}") from exc
    if not math.isfinite(v):
        raise ProtocolError(f"{name} 가 유한하지 않다: {value!r}")
    return v


def _decode_json(data: bytes) -> dict:
    if len(data) > MAX_PACKET_BYTES:
        raise ProtocolError(f"패킷이 너무 크다: {len(data)} bytes")
    try:
        obj = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"JSON 파싱 실패: {exc}") from exc
    if not isinstance(obj, dict):
        raise ProtocolError("JSON 최상위가 객체가 아니다")
    if obj.get("v") != PROTOCOL_VERSION:
        raise ProtocolError(f"프로토콜 버전 불일치: {obj.get('v')!r} != {PROTOCOL_VERSION}")
    return obj


@dataclass(frozen=True)
class HostCommand:
    """Host 가 한 사이클에 보내는 지시 전부. 좌표가 없다는 점이 핵심이다."""

    state: str
    linear_x: float = 0.0     # m/s, + 앞
    linear_y: float = 0.0     # m/s, + 왼쪽 (메카넘 횡이동)
    angular_z: float = 0.0    # rad/s, + 반시계
    stop: bool = False        # 참이면 속도 필드를 무시하고 정지
    label: str = ""           # 정책 지시문용 라벨. 좌표가 아니다
    seq: int = 0
    # PLACE 에서만 쓴다. 차를 상자 정면에 세운 뒤 남는 좌우 각도를 팔의 base(servo 1)로
    # 메운다 — 차체는 0.5 rad/s 에 데드밴드가 있어 몇 도짜리 회전을 못 낸다.
    # 좌표가 아니라 **각도 하나**다: "지금 네가 보는 방향에서 이만큼 더 틀어라".
    # Pi 가 place.max_base_yaw_deg 로 자른다. 옛 Pi 는 이 키를 모르면 0 으로 읽는다.
    arm_yaw_deg: float = 0.0

    def to_bytes(self) -> bytes:
        return json.dumps({
            "v": PROTOCOL_VERSION,
            "seq": int(self.seq),
            "state": self.state,
            "linear_x": float(self.linear_x),
            "linear_y": float(self.linear_y),
            "angular_z": float(self.angular_z),
            "stop": bool(self.stop),
            "label": self.label,
            "arm_yaw_deg": float(self.arm_yaw_deg),
        }, ensure_ascii=False).encode("utf-8")

    @classmethod
    def from_bytes(cls, data: bytes) -> "HostCommand":
        obj = _decode_json(data)
        state = obj.get("state")
        if state not in State.ALL:
            raise ProtocolError(f"모르는 state: {state!r}")
        label = obj.get("label", "")
        if not isinstance(label, str) or len(label) > MAX_LABEL_LEN:
            raise ProtocolError(f"label 이 올바르지 않다: {label!r}")
        stop = obj.get("stop", False)
        if not isinstance(stop, bool):
            raise ProtocolError(f"stop 은 bool 이어야 한다: {stop!r}")
        seq = obj.get("seq", 0)
        if not isinstance(seq, int) or isinstance(seq, bool):
            raise ProtocolError(f"seq 는 정수여야 한다: {seq!r}")
        return cls(
            state=state,
            linear_x=_finite("linear_x", obj.get("linear_x", 0.0)),
            linear_y=_finite("linear_y", obj.get("linear_y", 0.0)),
            angular_z=_finite("angular_z", obj.get("angular_z", 0.0)),
            stop=stop,
            label=label,
            seq=seq,
            arm_yaw_deg=_finite("arm_yaw_deg", obj.get("arm_yaw_deg", 0.0)),
        )


@dataclass(frozen=True)
class JobResult:
    """Pi 가 마친 팔 작업 하나의 결과. 다음 작업이 끝날 때까지 매 패킷 반복된다."""

    job_id: int
    action: str        # State.GRASP | State.PLACE
    ok: bool
    detail: str = ""

    def as_dict(self) -> dict:
        return {"job_id": self.job_id, "action": self.action,
                "ok": self.ok, "detail": self.detail}

    @classmethod
    def from_dict(cls, obj) -> "JobResult":
        if not isinstance(obj, dict):
            raise ProtocolError("result 가 객체가 아니다")
        action = obj.get("action")
        if action not in State.JOB_STATES:
            raise ProtocolError(f"result.action 이 올바르지 않다: {action!r}")
        job_id = obj.get("job_id")
        if not isinstance(job_id, int) or isinstance(job_id, bool):
            raise ProtocolError(f"result.job_id 가 정수가 아니다: {job_id!r}")
        return cls(job_id=job_id, action=action, ok=bool(obj.get("ok")),
                   detail=str(obj.get("detail", "")))


@dataclass(frozen=True)
class PiStatus:
    """Pi 가 주기적으로 보내는 상태."""

    boot_id: str
    state: str                        # Pi 가 지금 실행 중인 상태
    busy: bool                        # 팔 작업 진행 중
    job_id: int                       # 진행 중(또는 마지막) 작업 번호. 0 = 없음
    result: Optional[JobResult]       # 마지막으로 끝난 작업
    base_ok: bool                     # 구동계가 명령을 받아 가는가
    watchdog: bool                    # 명령이 끊겨 정지 중인가
    ack_seq: int = 0
    detail: str = ""

    def to_bytes(self) -> bytes:
        return json.dumps({
            "v": PROTOCOL_VERSION,
            "boot_id": self.boot_id,
            "ack_seq": int(self.ack_seq),
            "state": self.state,
            "busy": bool(self.busy),
            "job_id": int(self.job_id),
            "result": self.result.as_dict() if self.result else None,
            "base_ok": bool(self.base_ok),
            "watchdog": bool(self.watchdog),
            "detail": self.detail[:512],
        }, ensure_ascii=False).encode("utf-8")

    @classmethod
    def from_bytes(cls, data: bytes) -> "PiStatus":
        obj = _decode_json(data)
        state = obj.get("state")
        if state not in State.ALL:
            raise ProtocolError(f"모르는 state: {state!r}")
        raw_result = obj.get("result")
        result = None if raw_result is None else JobResult.from_dict(raw_result)
        job_id = obj.get("job_id", 0)
        if not isinstance(job_id, int) or isinstance(job_id, bool):
            raise ProtocolError(f"job_id 가 정수가 아니다: {job_id!r}")
        ack_seq = obj.get("ack_seq", 0)
        if not isinstance(ack_seq, int) or isinstance(ack_seq, bool):
            raise ProtocolError(f"ack_seq 가 정수가 아니다: {ack_seq!r}")
        return cls(
            boot_id=str(obj.get("boot_id", "")),
            state=state,
            busy=bool(obj.get("busy", False)),
            job_id=job_id,
            result=result,
            base_ok=bool(obj.get("base_ok", False)),
            watchdog=bool(obj.get("watchdog", False)),
            ack_seq=ack_seq,
            detail=str(obj.get("detail", "")),
        )


class JobTracker:
    """Host 쪽에서 "내가 시킨 작업이 끝났나"를 판정한다.

    사용법:
        tracker.arm(status)        # GRASP/PLACE 를 보내기 직전 한 번
        tracker.poll(status)       # 매 사이클 -> None(아직) | JobResult
    """

    def __init__(self) -> None:
        self._boot_id: Optional[str] = None
        self._baseline = 0
        self._action: Optional[str] = None

    def arm(self, action: str, status: Optional[PiStatus]) -> None:
        """작업 명령을 처음 보내기 **직전**에 부른다.

        status 가 None(아직 Pi 상태를 한 번도 못 받음)이면 기준을 0 으로 두지 않는다 —
        Pi 에 남아 있던 옛 결과가 방금 끝난 것으로 읽힌다. 대신 처음 도착하는 상태의
        **결과 번호**를 기준으로 삼는다. 그 사이에 우리 작업이 이미 끝났다면 그 결과를
        놓치게 되지만, 그 경우는 Host 타임아웃(실패)으로 끝난다. 거짓 성공보다 안전하다.
        """
        if action not in State.JOB_STATES:
            raise ValueError(f"작업 상태가 아니다: {action}")
        self._action = action
        if status is None:
            self._boot_id, self._baseline = None, None
        else:
            self._boot_id = status.boot_id
            self._baseline = max(status.job_id, status.result.job_id if status.result else 0)

    def poll(self, status: Optional[PiStatus]) -> Optional[JobResult]:
        if self._action is None or status is None:
            return None
        if self._baseline is None:
            self._boot_id = status.boot_id
            self._baseline = status.result.job_id if status.result else 0
            return None
        if self._boot_id is not None and status.boot_id != self._boot_id:
            # Pi 가 재시작했다. 번호가 처음부터 다시 센다.
            self._boot_id, self._baseline = status.boot_id, 0
        elif self._boot_id is None:
            self._boot_id = status.boot_id
        result = status.result
        if result is None or result.action != self._action or result.job_id <= self._baseline:
            return None
        self._action = None
        return result

    @property
    def armed(self) -> bool:
        return self._action is not None
