# -*- coding: utf-8 -*-
"""리더(노트북) → 파이 텔레옵 패킷 규약. 양쪽이 이 파일을 공유한다.

패킷 하나에 **팔과 베이스를 같이** 싣는다. 조작자 입장에서 둘은 한 손으로
하는 일이고(한 손은 리더 암, 다른 손은 키보드), 채널을 나누면 터미널이
두 개로 갈라져 시연 중에 창을 옮겨 다녀야 한다.

UDP를 쓰는 이유: 절대 위치를 50Hz로 계속 보내기 때문에 패킷 하나가 유실돼도
다음 패킷이 곧바로 진실을 덮어쓴다. TCP의 재전송은 여기서 오히려 해롭다 —
20ms 늦게 도착한 관절값은 이미 쓸모가 없는데 뒤 패킷을 막고 앉아있게 된다.

핸드셰이크가 없는 대신 epoch를 쓴다. 조작자가 팔 추종을 켤 때마다 리더가
epoch를 1 올리고, 파이는 처음 보는 epoch를 받으면 그 순간의 리더/팔로워
자세를 각각 기준점으로 latch한다. 이렇게 하면 "켜짐" 패킷이 유실돼도 다음
패킷에서 저절로 복구된다.
"""
from __future__ import annotations

import json

PROTOCOL_VERSION = 2
DEFAULT_PORT = 47800

# 서보 카운트는 0..4095에서 한 바퀴 돈다. 4090 → 5 는 +4091이 아니라 +11이다.
POS_RANGE = 4096
POS_HALF = POS_RANGE // 2


def wrap_delta(a: int, b: int) -> int:
    """a - b 를 -2048..2047 범위의 최단 회전으로 계산한다."""
    return (a - b + POS_HALF) % POS_RANGE - POS_HALF


def encode(seq: int, epoch: int, engaged: bool, pos: list,
           base: tuple = (0.0, 0.0, 0.0), scale: float = 0.0) -> bytes:
    """pos는 관절 6개 원시 카운트(읽기 실패는 None).
    base는 정규화된 (x, y, θ) 방향 -1..1, scale은 최대속도 대비 배율."""
    return json.dumps(
        {"v": PROTOCOL_VERSION, "seq": seq, "epoch": epoch,
         "en": bool(engaged), "pos": pos,
         "base": [round(float(v), 3) for v in base],
         "sc": round(float(scale), 3)},
        separators=(",", ":"),
    ).encode("utf-8")


def decode(raw: bytes) -> dict | None:
    """망가진 패킷은 조용히 버린다 — 다음 패킷이 20ms 뒤에 온다."""
    try:
        msg = json.loads(raw.decode("utf-8"))
    except Exception:
        return None
    if msg.get("v") != PROTOCOL_VERSION:
        return None
    pos = msg.get("pos")
    if not isinstance(pos, list) or len(pos) != 6:
        return None
    base = msg.get("base")
    if not isinstance(base, list) or len(base) != 3:
        msg["base"] = [0.0, 0.0, 0.0]
    msg.setdefault("sc", 0.0)
    return msg
