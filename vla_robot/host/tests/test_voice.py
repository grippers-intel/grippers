"""음성 입력: 말 시작·끝 잡기, 녹음 -> 인식 -> 확인 -> 전송 흐름, 화면 상태.
마이크·whisper 는 가짜를 끼운다(실제 모델은 tools/try_voice.py 로 확인)."""
import threading
import time
from dataclasses import replace

import numpy as np

from mission.commands import CommandDesk
from mission.host_fsm import MissionFSM
from tests.test_instruction import FakeResolver
from view.ui_state import UiState
from voice.voice_input import RATE, Endpointer, VoiceInput, clean_text, resample


def _signal(quiet_s=0.5, speech_s=1.0, tail_s=1.5, amp=0.2, noise=0.002, rate=RATE):
    rng = np.random.default_rng(0)
    parts = [noise * rng.standard_normal(int(quiet_s * rate)),
             amp * np.sin(2 * np.pi * 220 * np.arange(int(speech_s * rate)) / rate),
             noise * rng.standard_normal(int(tail_s * rate))]
    return np.concatenate(parts).astype(np.float32)


class FakeStream:
    def __init__(self, callback, signal, rate, block_s=0.03):
        self.cb, self.signal, self.rate, self.block = callback, signal, rate, int(rate * block_s)
        self._t = None

    def __enter__(self):
        def run():
            for i in range(0, len(self.signal), self.block):
                self.cb(self.signal[i:i + self.block].reshape(-1, 1), 0, None, None)
        self._t = threading.Thread(target=run, daemon=True)
        self._t.start()
        return self

    def __exit__(self, *a):
        return False


class FakeModel:
    def __init__(self, text):
        self.text, self.seen = text, None

    def transcribe(self, pcm, **kw):
        self.seen = (len(pcm), kw)
        seg = type("S", (), {"text": self.text})()
        return [seg], None


def _voice(cfg, signal, text="퀸을 바구니에 넣어줘.", rate=RATE):
    model = FakeModel(text)
    v = VoiceInput(cfg.voice, model_loader=lambda: model,
                   stream_factory=lambda cb: (FakeStream(cb, signal, rate), rate))
    return v, model


def _wait(pred, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.01)
    return False


def test_endpointer_finds_end_of_speech(cfg):
    ep = Endpointer(cfg.voice, 0.03)
    sig = _signal()
    outs = []
    for i in range(0, len(sig), int(RATE * 0.03)):
        b = sig[i:i + int(RATE * 0.03)]
        outs.append(ep.feed(float(np.sqrt(np.mean(b ** 2)))))
        if outs[-1]:
            break
    assert outs[-1] == "end"
    assert 0.4 <= ep.started_at <= 0.6
    assert ep.t < 0.5 + 1.0 + cfg.voice.silence_s + 0.2      # 꼬리 1.5 s 를 다 기다리지 않는다


def test_silence_only_is_no_speech(cfg):
    ep = Endpointer(cfg.voice, 0.03)
    sig = _signal(quiet_s=7.0, speech_s=0.0, tail_s=0.0)
    out = None
    for i in range(0, len(sig), int(RATE * 0.03)):
        out = ep.feed(float(np.sqrt(np.mean(sig[i:i + int(RATE * 0.03)] ** 2)))) or out
        if out:
            break
    assert out == "nospeech"


def test_listen_transcribe_final_and_take(cfg):
    v, model = _voice(cfg, _signal())
    assert v.toggle() is None and v.phase == "listening"
    assert _wait(lambda: v.phase == "final"), v.phase
    assert v.text == "퀸을 바구니에 넣어줘"                    # 끝 마침표는 뗀다
    assert model.seen[1]["language"] == "ko" and "퀸" in model.seen[1]["initial_prompt"]
    assert v.take_final() == "퀸을 바구니에 넣어줘" and v.phase == "idle"


def test_device_rate_is_resampled(cfg):
    sig = _signal(rate=44100)
    v, model = _voice(cfg, sig, rate=44100)
    v.toggle()
    assert _wait(lambda: v.phase == "final")
    # 16 kHz 로 줄여서 넘긴다(말 끝에서 멈추므로 원래 길이보다 짧다)
    assert model.seen[0] < len(sig) * RATE / 44100 + 10
    assert abs(len(resample(np.zeros(44100, np.float32), 44100)) - RATE) <= 1


def test_no_speech_is_an_error_not_a_hang(cfg):
    c = replace(cfg, voice=replace(cfg.voice, no_speech_s=1.0))
    v, _ = _voice(c, _signal(quiet_s=2.0, speech_s=0.0, tail_s=0.0))
    v.toggle()
    assert _wait(lambda: v.phase == "error")
    assert "들리지 않았습니다" in v.take_error() and v.phase == "idle"


def test_missing_model_reports_reason(cfg):
    c = replace(cfg, voice=replace(cfg.voice, model_dir="Z:/no-model-here"))
    v = VoiceInput(c.voice, stream_factory=lambda cb: (FakeStream(cb, _signal(), RATE), RATE))
    assert v._model_ready.wait(5)
    why = v.toggle()
    assert why and "음성 모델이 없습니다" in why


def test_clean_text():
    assert clean_text("  퀸을  바구니에 넣어줘. ") == "퀸을 바구니에 넣어줘"


def test_ui_screens_follow_voice(cfg):
    v, _ = _voice(cfg, _signal())
    fsm = MissionFSM(cfg)
    ui = UiState(cfg)
    pose = type("P", (), {"ok": True, "x": 1.0, "y": 0.5, "yaw_deg": 90.0, "xy": (1.0, 0.5), "fresh": True})()
    pmap = {"queen": [(0.6, 0.9)]}
    v.toggle()
    s = ui.build(pose, pmap, fsm, None, 0.0, 10.0, voice=v)
    assert s["screen"] == "listen" and s["recording"] is True
    assert _wait(lambda: v.phase == "final")
    s = ui.build(pose, pmap, fsm, None, 0.0, 10.0, voice=v)
    assert s["screen"] == "command" and s["command"]["action"] == "send"
    assert s["command"]["echo"] == "퀸을 바구니에 넣어줘"
    # 전송 -> 해석 -> 지시
    fake = FakeResolver({"퀸을 바구니에 넣어줘": {"matched": True, "labels": ["queen"], "quantity": "one",
                                            "intent": "organize", "reply": "", "reason": ""}})
    desk = CommandDesk(cfg, resolver=fake)
    desk.submit(v.take_final(), pmap)
    desk.update(fsm)
    assert fsm.order is not None and fsm.order.labels == ("queen",)
