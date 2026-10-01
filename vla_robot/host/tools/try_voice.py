"""노트북 마이크 음성 인식을 로봇·화면 없이 시험한다.

    python tools/try_voice.py                 # 마이크로 한 문장(말이 끝나면 자동으로 멈춤)
    python tools/try_voice.py --repeat 3      # 세 번
    python tools/try_voice.py --wav a.wav b.wav   # 녹음 파일로
    python tools/try_voice.py --download      # 모델 받기(models/whisper-small, 464 MB, 한 번만)

HF_HOME 이 지금 없는 드라이브를 가리키면(이 노트북: F:\\ml-cache) 받기가 실패한다 — --download 는 이 프로세스에서만
캐시를 프로젝트 안 임시 폴더로 돌려 받고 지운다(사용자 환경변수는 건드리지 않는다).
"""
from __future__ import annotations

import argparse
import os
import shutil
import sys
import time
import wave
from pathlib import Path

import numpy as np

HOST_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST_ROOT))

import host_config  # noqa: E402
from voice.voice_input import VoiceInput, clean_text, resample  # noqa: E402


def download(cfg) -> int:
    target = Path(cfg.voice.model_dir)
    cache = HOST_ROOT / "models" / ".hf_cache"
    os.environ["HF_HOME"] = str(cache)          # 이 프로세스만
    from faster_whisper import download_model
    size = target.name.replace("whisper-", "") or "small"
    t0 = time.monotonic()
    path = download_model(size, output_dir=str(target))
    shutil.rmtree(cache, ignore_errors=True)
    print(f"받음: {path} ({time.monotonic() - t0:.0f} s)")
    return 0


def read_wav(path: str) -> np.ndarray:
    with wave.open(path, "rb") as w:
        rate, ch, width = w.getframerate(), w.getnchannels(), w.getsampwidth()
        raw = w.readframes(w.getnframes())
    x = np.frombuffer(raw, dtype={1: np.int8, 2: np.int16, 4: np.int32}[width]).astype(np.float32)
    x /= float(np.iinfo({1: np.int8, 2: np.int16, 4: np.int32}[width]).max)
    if ch > 1:
        x = x.reshape(-1, ch).mean(axis=1)
    return resample(x, rate)


def main() -> int:
    host_config.configure_console()
    ap = argparse.ArgumentParser()
    ap.add_argument("--wav", nargs="+")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--download", action="store_true")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = host_config.load_host_config(args.config)
    if args.download:
        return download(cfg)

    v = VoiceInput(cfg.voice)
    print(f"모델 {cfg.voice.model_dir} 읽는 중…")
    v._model_ready.wait()
    if v._model is None:
        print(f"✗ {v._model_error}")
        return 1
    if args.wav:
        for p in args.wav:
            t0 = time.monotonic()
            segs, _ = v._model.transcribe(read_wav(p), language=cfg.voice.language, beam_size=cfg.voice.beam_size,
                                          initial_prompt=cfg.voice.prompt or None,
                                          condition_on_previous_text=False, without_timestamps=True)
            text = clean_text("".join(s.text for s in segs))      # 실제 계산은 여기서 돈다(지연 생성)
            print(f"{time.monotonic() - t0:4.1f} s  {Path(p).name}: {text}")
        return 0
    for i in range(args.repeat):
        input(f"\n[{i + 1}/{args.repeat}] Enter 를 누르고 바로 말하세요 (말이 끝나면 자동으로 멈춤) ")
        why = v.toggle()
        if why:
            print(f"✗ {why}")
            return 1
        t0 = time.monotonic()
        while v.phase in ("listening", "transcribing"):
            print(f"\r  {'듣는 중' if v.phase == 'listening' else '인식 중'}  소리 {'#' * int(v.level * 20):20s}",
                  end="", flush=True)
            time.sleep(0.05)
        print()
        if v.phase == "error":
            print(f"✗ {v.take_error()}")
            continue
        print(f"✓ \"{v.take_final()}\"  (전체 {time.monotonic() - t0:.1f} s, 인식 {v.last_transcribe_s:.1f} s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
