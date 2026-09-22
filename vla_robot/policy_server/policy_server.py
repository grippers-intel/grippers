"""VLA 정책 추론 서버 — 노트북 GPU 에서 돌리고 Pi 가 policy.source: remote 로 부른다.

    python policy_server.py --ckpt D:/ckpt/act_v5_all_180_120k_120000 --device cuda
    python policy_server.py --ckpt D:/ckpt/dp_v5_all_180_60k_060000 --device cuda --n-action-steps 63

규격은 vla_common/policy_wire.py 한 곳에 있다.

⚠️ 인증이 없다. 같은 LAN 에서만 쓸 것 — 로봇 팔을 움직이는 값을 내주는 서버다.
   기본 바인드를 127.0.0.1 이 아니라 --host 로 명시하게 한 이유다.
⚠️ lerobot 버전은 체크포인트를 만든 버전과 맞춰야 한다(ACT v5: 0.4.x, DP v5: 0.6.x).
⚠️ DP 기본 n_action_steps=32(1.07초)는 짧아서 뻗은 자세를 못 빠져나왔다(2026-09-06).
   63(= horizon - n_obs_steps + 1) 을 권장한다.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

try:
    from vla_common import policy_wire
    from vla_common.policy_runner import PolicyRunner
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "vla_common"))
    from vla_common import policy_wire
    from vla_common.policy_runner import PolicyRunner

MAX_BODY_BYTES = 16 * 1024 * 1024


class Server:
    def __init__(self, runner: PolicyRunner) -> None:
        self.runner = runner
        # 정책·전처리기는 한 벌이라 동시에 들어가면 섞인다. 청크당 한 번이라 줄 서도 손해가 없다.
        self.lock = threading.Lock()
        self.calls = 0
        self.total_ms = 0.0


def make_handler(server: Server):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send(self, code: int, body: bytes, content_type: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.split("?")[0] != "/health":
                self._send(404, b"not found", "text/plain")
                return
            with server.lock:
                calls, total = server.calls, server.total_ms
            info = {"ok": True, **server.runner.describe(), "calls": calls,
                    "avg_ms": round(total / calls, 1) if calls else 0.0}
            self._send(200, json.dumps(info).encode("utf-8"), "application/json")

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0) or 0)
            if length <= 0 or length > MAX_BODY_BYTES:
                self._send(400, policy_wire.encode_error(f"본문 길이 오류: {length}"),
                           "application/octet-stream")
                return
            body = self.rfile.read(length)
            if self.path.split("?")[0] != "/predict":
                self._send(404, policy_wire.encode_error("not found"), "application/octet-stream")
                return
            try:
                image, state, task = policy_wire.decode_request(body)
                with server.lock:
                    t0 = time.monotonic()
                    chunk = server.runner.predict_chunk(image, state, task)
                    ms = (time.monotonic() - t0) * 1000.0
                    server.calls += 1
                    server.total_ms += ms
                    n = server.calls
                print(f"[{n:5d}] {task[:28]:28s} {image.shape[1]}x{image.shape[0]} -> "
                      f"{tuple(chunk.shape)} {ms:.0f} ms", flush=True)
                self._send(200, policy_wire.encode_response(chunk, ms), "application/octet-stream")
            except Exception as exc:  # noqa: BLE001 — 서버는 죽지 않는다
                print(f"!! {type(exc).__name__}: {exc}", flush=True)
                self._send(500, policy_wire.encode_error(f"{type(exc).__name__}: {exc}"),
                           "application/octet-stream")

    return Handler


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ckpt", required=True, help="pretrained_model 디렉터리")
    ap.add_argument("--host", required=True, help="바인드 주소 (예: 0.0.0.0 — LAN 전용)")
    ap.add_argument("--port", type=int, default=policy_wire.DEFAULT_PORT)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--image-color", choices=("rgb", "bgr"), default="rgb",
                    help="정책에 넣을 색 순서. LeRobot 녹화 기본값은 rgb")
    ap.add_argument("--image-size", type=int, nargs=2, metavar=("H", "W"), default=None,
                    help="생략하면 train_config.json 의 resize")
    ap.add_argument("--wrist-roll", type=float, default=0.067,
                    help="wrist_roll 관측을 덮을 값. --no-freeze-wrist-roll 로 끈다")
    ap.add_argument("--no-freeze-wrist-roll", action="store_true")
    ap.add_argument("--n-action-steps", type=int, default=None)
    ap.add_argument("--denoise", type=int, default=None, help="DP 전용 denoising 스텝")
    ap.add_argument("--scheduler", choices=("DDPM", "DDIM"), default=None, help="DP 전용")
    args = ap.parse_args()

    print(f"정책 적재: {args.ckpt} ({args.device})")
    t0 = time.monotonic()
    runner = PolicyRunner(
        args.ckpt, device=args.device, image_color=args.image_color, image_size=args.image_size,
        wrist_roll_value=None if args.no_freeze_wrist_roll else args.wrist_roll,
        n_action_steps=args.n_action_steps, num_inference_steps=args.denoise,
        noise_scheduler_type=args.scheduler)
    h, w = runner.policy_hw
    runner.predict_chunk(np.zeros((h, w, 3), np.uint8), [0.0] * 6, "warmup")
    print(f"준비 {time.monotonic() - t0:.1f}s — {runner.describe()}")
    print(f"청크 {runner.n_action_steps / 30.0:.2f}s @30fps — 추론 ms 가 이보다 크면 팔이 끊긴다")

    httpd = ThreadingHTTPServer((args.host, args.port), make_handler(Server(runner)))
    print(f"대기: http://{args.host}:{args.port}  (GET /health, POST /predict)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
        runner.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
