"""원격 정책 클라이언트 — `PolicyRunner` 와 같은 `predict_chunk` 시그니처.

Pi 로컬 ACT 는 397~465ms, 원격(노트북 GPU) 왕복은 117ms 였다(2026-09-05). 둘 다
청크 3.33초 안에 들어가므로 기본은 local 이고, remote 는 Pi 에서 실시간이 안 되는
정책(Diffusion/SmolVLA)이나 체크포인트를 자주 바꿀 때 쓴다.

프레임은 서버의 입력 해상도로 **먼저 줄여서** raw 로 보낸다(720p 2.7MB -> 172KB).
리사이즈는 torch bilinear(align_corners=False)와 같은 좌표 규칙의 numpy 구현이다.
색 변환은 하지 않는다 — 서버의 PolicyRunner 가 image_color 설정대로 한다.
"""
from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from typing import Optional, Sequence

import numpy as np

from vla_common import policy_wire


def resize_bilinear_uint8(image: np.ndarray, hw: Sequence[int]) -> np.ndarray:
    h, w = int(hw[0]), int(hw[1])
    src_h, src_w = image.shape[:2]
    if (src_h, src_w) == (h, w):
        return np.ascontiguousarray(image)
    ys = (np.arange(h) + 0.5) * src_h / h - 0.5
    xs = (np.arange(w) + 0.5) * src_w / w - 0.5
    y0 = np.clip(np.floor(ys).astype(int), 0, src_h - 1)
    x0 = np.clip(np.floor(xs).astype(int), 0, src_w - 1)
    y1 = np.clip(y0 + 1, 0, src_h - 1)
    x1 = np.clip(x0 + 1, 0, src_w - 1)
    wy = np.clip(ys - np.floor(ys), 0, 1).reshape(-1, 1, 1)
    wx = np.clip(xs - np.floor(xs), 0, 1).reshape(1, -1, 1)
    img = image.astype(np.float32)
    top = img[y0][:, x0] * (1 - wx) + img[y0][:, x1] * wx
    bottom = img[y1][:, x0] * (1 - wx) + img[y1][:, x1] * wx
    out = top * (1 - wy) + bottom * wy
    return np.ascontiguousarray(np.rint(np.clip(out, 0, 255)).astype(np.uint8))


class RemotePolicyClient:
    def __init__(self, url: str, timeout_s: float = 5.0,
                 n_action_steps: Optional[int] = None) -> None:
        self.base_url = url.rstrip("/")
        self.timeout_s = float(timeout_s)
        self._n_override = int(n_action_steps) if n_action_steps else None
        self.policy_hw: Optional[tuple[int, int]] = None
        self.chunk_size = 0
        self.n_action_steps = 0
        self.last_latency_ms = 0.0
        self.info: dict = {}

    def health(self) -> dict:
        """기동 때 한 번 불러 **일찍 실패**한다 — 파지 도중에 서버가 없는 걸 알면
        팔이 어중간한 자세에 남는다."""
        url = self.base_url + "/health"
        try:
            with urllib.request.urlopen(url, timeout=self.timeout_s) as resp:
                info = json.loads(resp.read().decode("utf-8"))
        except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"추론 서버에 연결하지 못했다({url}): {exc}") from exc
        try:
            self.policy_hw = (int(info["policy_hw"][0]), int(info["policy_hw"][1]))
            self.chunk_size = int(info["chunk_size"])
            server_steps = int(info["n_action_steps"])
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            raise RuntimeError(f"추론 서버 health 응답 형식 오류: {info}") from exc
        self.n_action_steps = self._n_override or server_steps
        self.info = info
        return info

    def describe(self) -> dict:
        return {"remote": self.base_url, **self.info, "n_action_steps": self.n_action_steps}

    def predict_chunk(self, image_bgr: np.ndarray, state6: Sequence[float], task: str) -> np.ndarray:
        if self.policy_hw is None:
            self.health()
        small = resize_bilinear_uint8(np.asarray(image_bgr, dtype=np.uint8), self.policy_hw)
        body = policy_wire.encode_request(small, state6, task)
        req = urllib.request.Request(self.base_url + "/predict", data=body,
                                     headers={"Content-Type": "application/octet-stream"})
        started = time.monotonic()
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                payload = resp.read()
        except urllib.error.HTTPError as exc:
            # HTTPError 는 URLError 의 자식이라 먼저 잡아야 서버 오류 문구를 살린다.
            try:
                header, _ = policy_wire.unpack(exc.read())
                message = header.get("error") or f"HTTP {exc.code}"
            except Exception:  # noqa: BLE001
                message = f"HTTP {exc.code}"
            raise RuntimeError(f"추론 서버 오류: {message}") from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise RuntimeError(f"추론 서버 호출 실패: {exc}") from exc
        self.last_latency_ms = (time.monotonic() - started) * 1000.0
        chunk, _ms = policy_wire.decode_response(payload)
        return chunk[: self.n_action_steps]
