"""정책 추론 서버 HTTP 규격 — 서버와 클라이언트가 이 모듈 하나를 공유한다.

    GET  /health   -> JSON {"ok", "ckpt", "policy_type", "policy_hw", "chunk_size",
                            "n_action_steps", "image_color", "calls", "avg_ms"}
    POST /predict  -> 요청:  [4바이트 big-endian 헤더길이][헤더 JSON][raw uint8 픽셀 HxWx3]
                      헤더:  {"shape":[H,W,3], "dtype":"uint8", "color":"bgr",
                              "state":[6 floats], "task": str}
                      응답:  [4바이트 헤더길이][헤더 JSON {"shape":[T,6], "ms"}][float32 LE]
                      오류:  HTTP 500 + [4바이트][{"error": str}]

프레임은 **항상 BGR 로 보낸다**(ROS bgr8 / OpenCV 기본). 정책이 기대하는 색 순서로
바꾸는 것은 `PolicyRunner` 한 곳에서만 한다 — 변환 위치가 둘이면 한 쪽만 고치는
사고가 난다.

손실 압축(JPEG)을 쓰지 않는다. 180x320 raw 는 172KB 이고 청크당 한 번이라 LAN 에서
문제가 안 되며, 압축하면 정책 입력이 학습 때와 달라진다.
"""
from __future__ import annotations

import json
import struct

import numpy as np

DEFAULT_PORT = 8770
FRAME_COLOR = "bgr"


class WireError(ValueError):
    pass


def pack(header: dict, blob: bytes = b"") -> bytes:
    raw = json.dumps(header, ensure_ascii=False).encode("utf-8")
    return struct.pack(">I", len(raw)) + raw + blob


def unpack(body: bytes) -> tuple[dict, bytes]:
    if len(body) < 4:
        raise WireError(f"본문이 잘렸다: {len(body)} bytes")
    (n,) = struct.unpack(">I", body[:4])
    if 4 + n > len(body):
        raise WireError(f"헤더 길이({n})가 본문보다 길다({len(body)})")
    try:
        header = json.loads(body[4:4 + n].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise WireError(f"헤더 JSON 오류: {exc}") from exc
    if not isinstance(header, dict):
        raise WireError("헤더가 객체가 아니다")
    return header, body[4 + n:]


def encode_request(image_bgr: np.ndarray, state6, task: str) -> bytes:
    image = np.ascontiguousarray(image_bgr)
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise WireError(f"uint8 HxWx3 이미지만 보낸다: {image.dtype} {image.shape}")
    state = [float(v) for v in state6]
    if len(state) != 6:
        raise WireError(f"state 는 6개여야 한다: {len(state)}")
    header = {"shape": list(image.shape), "dtype": "uint8", "color": FRAME_COLOR,
              "state": state, "task": str(task)}
    return pack(header, image.tobytes())


def decode_request(body: bytes) -> tuple[np.ndarray, list[float], str]:
    header, blob = unpack(body)
    if header.get("dtype") != "uint8" or header.get("color") != FRAME_COLOR:
        raise WireError(f"uint8 BGR 프레임만 받는다: {header.get('dtype')} {header.get('color')}")
    shape = tuple(int(v) for v in header.get("shape", ()))
    if len(shape) != 3 or shape[2] != 3:
        raise WireError(f"shape 이 HxWx3 이 아니다: {shape}")
    want = shape[0] * shape[1] * shape[2]
    if len(blob) != want:
        raise WireError(f"픽셀 길이가 shape {shape} 와 다르다: {len(blob)} != {want}")
    state = header.get("state")
    if not isinstance(state, list) or len(state) != 6:
        raise WireError(f"state 는 6개 리스트여야 한다: {state!r}")
    image = np.frombuffer(blob, dtype=np.uint8).reshape(shape).copy()
    return image, [float(v) for v in state], str(header.get("task", ""))


def encode_response(chunk: np.ndarray, ms: float) -> bytes:
    arr = np.ascontiguousarray(chunk, dtype="<f4")
    if arr.ndim != 2 or arr.shape[1] != 6:
        raise WireError(f"청크는 [T,6] 이어야 한다: {arr.shape}")
    return pack({"shape": list(arr.shape), "ms": round(float(ms), 1)}, arr.tobytes())


def decode_response(body: bytes) -> tuple[np.ndarray, float]:
    header, blob = unpack(body)
    if "error" in header:
        raise WireError(f"서버 오류: {header['error']}")
    shape = tuple(int(v) for v in header.get("shape", ()))
    if len(shape) != 2 or shape[1] != 6:
        raise WireError(f"응답 shape 이 [T,6] 이 아니다: {shape}")
    if len(blob) != shape[0] * shape[1] * 4:
        raise WireError(f"응답 길이가 shape {shape} 와 다르다: {len(blob)}")
    chunk = np.frombuffer(blob, dtype="<f4").reshape(shape).astype(np.float32)
    return chunk, float(header.get("ms", 0.0))


def encode_error(message: str) -> bytes:
    return pack({"error": str(message)})
