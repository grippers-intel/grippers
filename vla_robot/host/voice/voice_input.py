"""노트북 내장 마이크 -> 한국어 문장 (faster-whisper, 오프라인). 시연 UI 마이크 버튼이 쓴다.

phase:  idle -> listening -> transcribing -> final (사람이 전송을 누름) | error -> idle
- listening: 마이크를 연다. 처음 0.3 s 로 주변 소음을 재고, 그보다 확실히 큰 소리가 나면 "말 시작",
  말이 끝나고 silence_s 동안 조용하면 자동으로 멈춘다. 다시 누르면 바로 멈춘다. 최대 max_s.
- transcribing: whisper(기본 small, int8, CPU). 노트북에서 한 문장 ~2.6 s(2026-10-01, 8스레드).
- final: 들은 문장을 화면에 보여 주고 **사람이 전송을 눌러야** 해석(Claude)으로 간다 — 팀원 화면 설계와 같다.
  whisper 는 비슷한 소리의 다른 낱말을 자신 있게 낸다(팀원 실측: "퀸"->"균", "박스로"->"박수로").

모델은 `voice.model_dir`(host/models/whisper-small, 464 MB, 저장소 밖)에서 읽는다 — 인터넷이 필요 없다.
받는 법: tools/try_voice.py --download (HF_HOME 이 없는 드라이브를 가리키면 프로젝트 안 임시 캐시로 받는다).
"""
from __future__ import annotations

import queue
import threading
import time
from pathlib import Path
from typing import Callable, Optional

import numpy as np

RATE = 16000                     # whisper 입력


def clean_text(text: str) -> str:
    t = " ".join(text.split()).strip()
    return t.rstrip(".。!?").strip()


def resample(x: np.ndarray, src: int, dst: int = RATE) -> np.ndarray:
    if src == dst or len(x) == 0:
        return x.astype(np.float32)
    n = int(round(len(x) * dst / src))
    return np.interp(np.linspace(0, len(x) - 1, n), np.arange(len(x)), x).astype(np.float32)


class Endpointer:
    """소리 크기(RMS)로 말의 시작·끝을 잡는다. 블록(약 30 ms)마다 feed()."""

    def __init__(self, vcfg, block_s: float) -> None:
        self.cfg = vcfg
        self.block_s = block_s
        self.floor: list[float] = []
        self.threshold: Optional[float] = None
        self.started_at: Optional[float] = None     # 말 시작(녹음 시작부터 s)
        self.quiet_s = 0.0
        self.t = 0.0

    def feed(self, rms: float) -> Optional[str]:
        """None = 계속, "end" = 말이 끝났다, "nospeech" = 너무 오래 조용, "max" = 최대 길이."""
        c = self.cfg
        self.t += self.block_s
        if self.threshold is None:
            self.floor.append(rms)
            if self.t >= c.calib_s:
                self.threshold = max(c.min_rms, c.speech_ratio * float(np.median(self.floor)))
            return None
        if self.t >= c.max_s:
            return "max"
        if self.started_at is None:
            if rms >= self.threshold:
                self.started_at = self.t
            elif self.t >= c.no_speech_s:
                return "nospeech"
            return None
        self.quiet_s = self.quiet_s + self.block_s if rms < self.threshold else 0.0
        if self.quiet_s >= c.silence_s and self.t - self.started_at >= 0.3:
            return "end"
        return None


class VoiceInput:
    def __init__(self, vcfg, model_loader: Optional[Callable] = None,
                 stream_factory: Optional[Callable] = None) -> None:
        self.cfg = vcfg
        self._lock = threading.Lock()
        self.phase = "idle"
        self.text = ""
        self.error: Optional[str] = None
        self.level = 0.0
        self._stop = threading.Event()
        self._model = None
        self._model_error: Optional[str] = None
        self._model_ready = threading.Event()
        self._loader = model_loader or self._load_model
        self._stream_factory = stream_factory or self._open_stream
        self.available = bool(vcfg.enabled)
        if self.available:
            threading.Thread(target=self._preload, name="whisper-load", daemon=True).start()

    # -- 모델 ---------------------------------------------------------------
    def _load_model(self):
        from faster_whisper import WhisperModel
        path = Path(self.cfg.model_dir)
        if not (path / "model.bin").exists():
            raise FileNotFoundError(f"음성 모델이 없습니다: {path} (tools/try_voice.py --download)")
        return WhisperModel(str(path), device="cpu", compute_type="int8", cpu_threads=self.cfg.threads)

    def _preload(self) -> None:
        try:
            self._model = self._loader()
        except ImportError:
            self._model_error = "faster-whisper 가 없습니다 (pip install -r requirements.txt)"
        except Exception as exc:  # noqa: BLE001 — 화면에 알리고 나머지는 계속 돈다
            self._model_error = str(exc)
        finally:
            self._model_ready.set()

    # -- 마이크 -------------------------------------------------------------
    def _open_stream(self, callback):
        """sounddevice 입력 스트림. 16 kHz 를 못 열면 장치 기본 rate 로 열고 나중에 줄인다."""
        import sounddevice as sd
        block = int(RATE * 0.03)
        try:
            return sd.InputStream(samplerate=RATE, channels=1, dtype="float32", blocksize=block,
                                  callback=callback), RATE
        except Exception:  # noqa: BLE001 — 장치가 16 kHz 를 거부하면
            rate = int(sd.query_devices(kind="input")["default_samplerate"])
            return sd.InputStream(samplerate=rate, channels=1, dtype="float32",
                                  blocksize=int(rate * 0.03), callback=callback), rate

    # -- 조작 ---------------------------------------------------------------
    def toggle(self) -> Optional[str]:
        """마이크 버튼. 시작/멈춤. 못 쓰면 이유(문자열)를 돌려준다."""
        with self._lock:
            phase = self.phase
        if not self.available:
            return "음성 입력이 꺼져 있습니다(voice.enabled)"
        if phase == "listening":
            self._stop.set()
            return None
        if phase == "transcribing":
            return "앞 문장을 인식하는 중입니다"
        if self._model_ready.is_set() and self._model is None:
            return f"음성 인식을 쓸 수 없습니다 — {self._model_error}"
        self._stop.clear()
        with self._lock:
            self.phase, self.text, self.error, self.level = "listening", "", None, 0.0
        threading.Thread(target=self._listen, name="voice", daemon=True).start()
        return None

    def take_final(self) -> Optional[str]:
        """전송 — final 문장을 넘기고 idle 로."""
        with self._lock:
            if self.phase != "final":
                return None
            t, self.phase, self.text = self.text, "idle", ""
            return t

    def cancel(self) -> None:
        self._stop.set()
        with self._lock:
            if self.phase in ("final", "error"):
                self.phase, self.text, self.error = "idle", "", None

    def take_error(self) -> Optional[str]:
        with self._lock:
            if self.phase != "error":
                return None
            e, self.phase, self.error = self.error, "idle", None
            return e

    # -- 스레드 -------------------------------------------------------------
    def _fail(self, msg: str) -> None:
        with self._lock:
            self.phase, self.error, self.level = "error", msg, 0.0

    def _listen(self) -> None:
        chunks: "queue.Queue[np.ndarray]" = queue.Queue()

        def cb(indata, _frames, _time, _status):
            chunks.put(indata[:, 0].copy())

        try:
            stream, rate = self._stream_factory(cb)
        except Exception as exc:  # noqa: BLE001
            self._fail(f"마이크를 열 수 없습니다: {exc}")
            return
        ep = Endpointer(self.cfg, 0.03)
        audio: list[np.ndarray] = []
        why = None
        try:
            with stream:
                while not self._stop.is_set():
                    try:
                        block = chunks.get(timeout=0.5)
                    except queue.Empty:
                        continue
                    audio.append(block)
                    rms = float(np.sqrt(np.mean(block ** 2))) if len(block) else 0.0
                    with self._lock:
                        self.level = min(1.0, rms / max(ep.threshold or self.cfg.min_rms, 1e-6) / 4)
                    why = ep.feed(rms)
                    if why:
                        break
        except Exception as exc:  # noqa: BLE001
            self._fail(f"녹음 중 오류: {exc}")
            return
        if why == "nospeech" or (ep.started_at is None and why != "max"):
            self._fail("아무 말도 들리지 않았습니다 — 마이크를 누르고 바로 말해 주세요")
            return
        with self._lock:
            self.phase, self.level = "transcribing", 0.0
        self._transcribe(resample(np.concatenate(audio) if audio else np.zeros(0, np.float32), rate))

    def _transcribe(self, pcm: np.ndarray) -> None:
        if not self._model_ready.wait(timeout=60) or self._model is None:
            self._fail(f"음성 인식을 쓸 수 없습니다 — {self._model_error or '모델 로딩 지연'}")
            return
        try:
            t0 = time.monotonic()
            segs, _info = self._model.transcribe(
                pcm, language=self.cfg.language, beam_size=self.cfg.beam_size,
                initial_prompt=self.cfg.prompt or None, condition_on_previous_text=False,
                without_timestamps=True)
            text = clean_text("".join(s.text for s in segs))
            self.last_transcribe_s = time.monotonic() - t0
        except Exception as exc:  # noqa: BLE001
            self._fail(f"인식 중 오류: {exc}")
            return
        if not text:
            self._fail("말을 알아듣지 못했습니다 — 다시 말해 주세요")
            return
        with self._lock:
            self.phase, self.text = "final", text
