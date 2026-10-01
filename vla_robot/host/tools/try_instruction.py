"""Claude 지시 해석을 로봇·카메라 없이 한 번 불러 본다(키 확인용).

    python tools/try_instruction.py "체스 말만 전부 정리해줘"
    python tools/try_instruction.py "자유롭게 움직이는 말 가져와" --labels queen knight rook star

키는 ANTHROPIC_API_KEY 환경변수(사용자 환경변수로 등록한 뒤 터미널을 새로 열 것). 한 번 부를 때마다 요금이 든다.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

HOST_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST_ROOT))

import host_config  # noqa: E402
from mission.instruction import InstructionResolver  # noqa: E402


def main() -> int:
    host_config.configure_console()
    ap = argparse.ArgumentParser()
    ap.add_argument("text", nargs="+", help="지시 문장(여러 개면 차례로)")
    ap.add_argument("--labels", nargs="+", default=["box", "soccer", "star", "queen", "knight", "rook"],
                    help="지금 보인다고 칠 라벨")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = host_config.load_host_config(args.config)
    res = InstructionResolver(cfg.instruction)
    print(f"모델 {cfg.instruction.model} · effort {cfg.instruction.effort} · 보이는 라벨 {args.labels}")
    for text in args.text:
        t0 = time.monotonic()
        r = res.resolve(text, args.labels)
        dt = time.monotonic() - t0
        if r.error:
            print(f"\n✗ {text!r}  ({dt:.1f} s)\n  오류: {r.error}  request_id={r.request_id}")
            continue
        mark = "✓" if r.ok else "?"
        print(f"\n{mark} {text!r}  ({dt:.1f} s)\n  라벨 {list(r.labels)} · {r.quantity} · {r.intent}"
              f"\n  답: {r.reply}\n  이유: {r.reason}  request_id={r.request_id}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
