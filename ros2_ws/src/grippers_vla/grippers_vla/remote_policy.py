"""추론을 다른 기계에 맡기는 클라이언트 — `PolicyRunner` 와 같은 자리에 끼운다.

`predict_chunk(image_bgr, state6, task)` 시그니처가 `PolicyRunner` 와 같아서
`vla_inference_node` 는 둘 중 무엇을 들고 있는지 몰라도 된다.

## ⚠️ 기본 경로가 아니다 — ACT 는 Pi 에서 직접 돈다

2026-09-05 실측(act_v5_all 120k steps, 180x320):

    Pi 로컬  397~465ms (듀티 14%)  — perception·arm_driver·vla_inference 가
                                    같이 도는 조건에서 잰 값이다
    원격     117ms     (듀티 3.5%) — 리사이즈·전송·추론·회신 왕복 전부 포함

원격이 3.5배 빠르지만 **둘 다 청크 3.33초 안에 여유롭게 들어간다.** 여유가
있는데 시연 경로에 노트북과 네트워크 의존을 하나 더 만들 이유가 없어서
`policy_source` 기본값은 `local` 이다.

그래도 이 경로를 남겨 두는 이유는 둘이다.

1. **Pi 에서 실시간이 안 되는 정책.** SmolVLA 가 그렇다
   (`tools/arm/smolvla_check.py`). 그런 정책은 원격 말고 길이 없다.
2. **체크포인트를 자주 갈아끼울 때.** 200MB 를 Pi 로 옮기지 않아도 된다.

## ⚠️ 손실 압축을 쓰지 않는다

프레임을 **180x320 으로 먼저 줄여서 raw 로** 보낸다. 172KB 인데 청크 하나가
3.33초이므로 초당 52KB 다 — 유선 LAN 에서 의미 없는 양이다.

JPEG 로 줄이면 8~15KB 까지 내려가지만 **정책 입력이 학습 때와 달라진다.**
대역폭이 문제가 아닌 상황에서 입력을 건드릴 이유가 없다. 나중에 무선으로
옮겨 대역폭이 정말 문제가 되면 그때 PNG(무손실)를 먼저 검토할 것.

리사이즈를 **보내기 전에** 하는 이유도 같다. 720x1280 을 보내면 2.7MB 이고,
어차피 서버가 180x320 으로 줄인다 — 줄이는 위치만 옮기면 대역폭이 16배 준다.
리사이즈 방법은 `PolicyRunner._batch` 와 같은 bilinear 여야 한다.
"""
from __future__ import annotations

import json
import time
import urllib.error
import urllib.request

import numpy as np

#: 서버 응답을 이만큼 기다린다. 실측 왕복이 117ms(2026-09-05)라 한참 여유가
#: 있지만, 서버가 첫 호출에서 걸리거나 링크가 흔들릴 때를 보고 크게 잡았다.
#: 넘으면 실패로 접는다 — 청크가 3.33초라 여기서 오래 매달리면 팔이 끊긴다.
DEFAULT_TIMEOUT_S = 5.0


def _server_error_text(response) -> str:
    """서버가 본문에 담아 보낸 오류 문구.

    `HTTPError` 는 예외이면서 동시에 응답 객체라 본문을 여기서 읽을 수 있다.
    이걸 안 꺼내면 Pi 로그에 "HTTP Error 500" 만 남아서, 정작 무엇이
    터졌는지(체크포인트 경로? shape 불일치?) 서버 콘솔을 봐야만 안다.
    """
    try:
        body = response.read()
        n = int.from_bytes(body[:4], "big")
        text = json.loads(body[4:4 + n].decode("utf-8")).get("error", "")
        if text:
            return text
    except Exception:  # noqa: BLE001 — 오류 보고 중에 또 터지면 안 된다
        pass
    return f"HTTP {getattr(response, 'code', '?')}"


class RemotePolicyRunner:
    """추론 서버에 프레임과 관절값을 보내고 청크를 받아온다."""

    def __init__(self, url: str, policy_hw=(180, 320), n_action_steps: int | None = None,
                 chunk_size: int = 100, timeout_s: float = DEFAULT_TIMEOUT_S):
        self.url = url.rstrip("/") + "/predict"
        self.policy_hw = tuple(policy_hw)
        # None 이면 서버가 들고 있는 값을 따른다. 숫자면 **부른 쪽이 일부러
        # 줄인 것**이므로 health() 가 덮지 않는다(아래 참고).
        self._n_steps_override = None if n_action_steps is None else int(n_action_steps)
        self.chunk_size = int(chunk_size)
        self.n_action_steps = self._n_steps_override or self.chunk_size
        self.timeout_s = float(timeout_s)
        self.last_latency_ms = 0.0

    def describe(self) -> str:
        return f"remote {self.url} (입력 {self.policy_hw})"

    def health(self) -> dict:
        """서버가 살아 있고 어떤 체크포인트를 들고 있는지. 실패하면 예외.

        노드 기동 때 한 번 불러서 **일찍 실패하게** 한다 — 파지를 시작한 뒤에
        서버가 없는 것을 알면 팔이 어중간한 자세에 남는다."""
        url = self.url.replace("/predict", "/health")
        try:
            with urllib.request.urlopen(url, timeout=self.timeout_s) as response:
                info = json.loads(response.read().decode("utf-8"))
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            raise RuntimeError(f"추론 서버에 못 붙었습니다({url}): {e}") from e
        # 서버가 실제로 쓰는 해상도에 맞춘다. 여기서 안 맞추면 Pi 가 엉뚱한
        # 크기로 줄여 보내고, 서버가 다시 줄이면서 두 번 리샘플된다.
        if "policy_hw" in info:
            self.policy_hw = tuple(info["policy_hw"])
        if "chunk_size" in info:
            self.chunk_size = int(info["chunk_size"])
        # ⚠️ 부른 쪽이 n_action_steps 를 직접 준 경우에는 덮지 않는다. 덮으면
        # `-p n_action_steps:=50` 이 remote 에서만 조용히 무시된다 — 파라미터가
        # 먹은 줄 알고 실기에서 헤매기 딱 좋다.
        if self._n_steps_override is not None:
            self.n_action_steps = self._n_steps_override
        elif "n_action_steps" in info:
            self.n_action_steps = int(info["n_action_steps"])
        return info

    def _resize(self, image_bgr: np.ndarray) -> np.ndarray:
        """`PolicyRunner._batch` 와 같은 bilinear 로 줄인다.

        cv2 를 쓰지 않는다 — Pi 컨테이너의 cv2 는 있지만, 여기서 굳이
        의존을 늘리지 않으려는 것이다. torch 도 부르지 않는다(이 클라이언트가
        도는 쪽에는 정책이 없어야 한다).
        """
        h, w = self.policy_hw
        # 헤더에 dtype 을 uint8 로 적어 보내므로 여기서 못을 박는다. float 프레임이
        # 그대로 나가면 서버가 바이트를 uint8 로 다시 읽어 조용히 깨진 그림이 된다.
        if image_bgr.dtype != np.uint8:
            image_bgr = np.clip(image_bgr, 0, 255).astype(np.uint8)
        src_h, src_w = image_bgr.shape[:2]
        if (src_h, src_w) == (h, w):
            return np.ascontiguousarray(image_bgr)
        # align_corners=False 인 bilinear 과 같은 좌표 규칙.
        ys = (np.arange(h) + 0.5) * src_h / h - 0.5
        xs = (np.arange(w) + 0.5) * src_w / w - 0.5
        y0 = np.clip(np.floor(ys).astype(int), 0, src_h - 1)
        x0 = np.clip(np.floor(xs).astype(int), 0, src_w - 1)
        y1 = np.clip(y0 + 1, 0, src_h - 1)
        x1 = np.clip(x0 + 1, 0, src_w - 1)
        wy = np.clip(ys - y0, 0, 1).reshape(-1, 1, 1)
        wx = np.clip(xs - x0, 0, 1).reshape(1, -1, 1)
        img = image_bgr.astype(np.float32)
        top = img[y0][:, x0] * (1 - wx) + img[y0][:, x1] * wx
        bottom = img[y1][:, x0] * (1 - wx) + img[y1][:, x1] * wx
        out = top * (1 - wy) + bottom * wy
        return np.ascontiguousarray(np.rint(np.clip(out, 0, 255)).astype(np.uint8))

    def predict_chunk(self, image_bgr: np.ndarray, state6, task: str) -> np.ndarray:
        small = self._resize(np.asarray(image_bgr))
        header = json.dumps({
            "shape": list(small.shape),
            "dtype": "uint8",
            "state": [float(v) for v in state6],
            "task": task,
        }).encode("utf-8")
        # 헤더 길이(4바이트) + 헤더 + raw 픽셀. 형식을 단순하게 둔 이유는
        # 양쪽에 추가 의존성을 안 만들기 위해서다.
        body = len(header).to_bytes(4, "big") + header + small.tobytes()
        request = urllib.request.Request(
            self.url, data=body,
            headers={"Content-Type": "application/octet-stream"})
        started = time.monotonic()
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                payload = response.read()
        except urllib.error.HTTPError as e:
            # ⚠️ URLError 보다 먼저 잡아야 한다 — HTTPError 가 URLError 의
            # 자식이라 순서를 바꾸면 서버가 보낸 오류 문구를 통째로 잃는다.
            raise RuntimeError(f"추론 서버 오류: {_server_error_text(e)}") from e
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            raise RuntimeError(f"추론 서버 호출 실패: {e}") from e
        self.last_latency_ms = (time.monotonic() - started) * 1000.0

        if len(payload) < 4:
            raise RuntimeError(f"추론 서버 응답이 잘렸습니다: {len(payload)}바이트")
        n = int.from_bytes(payload[:4], "big")
        meta = json.loads(payload[4:4 + n].decode("utf-8"))
        chunk = np.frombuffer(payload[4 + n:], dtype=np.float32).reshape(meta["shape"])
        return chunk[: self.n_action_steps]
