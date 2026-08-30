"""마이크 음성 지시를 로컬 Whisper(faster-whisper) 로 텍스트로 바꾼다.

geti_detector.GetiWorker / instruction_resolver.InstructionResolver 와 같은
비동기 패턴 — 녹음도 변환도 시간이 걸리므로(변환은 CPU 로 0.5~수 초) 메인
루프(카메라 캡처 + 차량 명령 전송)를 막으면 안 된다.

음성 버튼은 토글이다: 한 번 누르면 녹음 시작, 다시 누르면 녹음 종료 + 백그라운드
스레드에서 Whisper 변환 시작. 변환 결과는 지시를 자동으로 전송하지 않고
지시 입력창에 채워 넣기만 한다(live_map.LiveMap.set_instruction_text) —
음성인식이 틀렸을 때(예: "기물"을 "김을"로 잘못 들었을 때) 사용자가 눈으로
보고 고친 뒤 직접 전송 버튼을 누르게 하기 위한 안전장치다. 실측해보니
Whisper 가 이런 실수를 종종 하므로, 자동 전송은 위험하다고 판단해 이렇게
설계했다.
"""

from __future__ import annotations

import os
import threading
from dataclasses import dataclass
from typing import Optional, Union

import numpy as np
import sounddevice as sd

MODEL_SIZE = "base"   # small 로 바꿔도 정확도 차이가 크지 않았음(실측) — 속도 우선
SAMPLE_RATE = 16000


@dataclass
class VoiceResult:
    text: str
    error: Optional[str] = None


class VoiceRecorder:
    """toggle() 을 호출할 때마다 녹음 시작/종료가 번갈아 일어난다.

    device 를 안 주면 그 컴퓨터의 "시스템 기본 입력 장치"를 쓴다 — 대부분은
    이걸로 충분하지만, 마이크가 여러 개거나 기본값이 엉뚱하게 잡힌
    컴퓨터에서는 계속 "인식된 말이 없음"만 나올 수 있다. 그럴 땐
    `python -c "import sounddevice as sd; print(sd.query_devices())"` 로
    장치 번호를 확인해서 device 인자로 넘기거나, 코드를 안 건드리고
    WHISPER_MIC_DEVICE 환경변수(장치 번호)로 지정해도 된다.
    """

    def __init__(self, model_size: str = MODEL_SIZE,
                 device: Optional[Union[int, str]] = None) -> None:
        self._model_size = model_size
        env_device = os.environ.get("WHISPER_MIC_DEVICE")
        if device is None and env_device:
            device = int(env_device) if env_device.isdigit() else env_device
        self._device = device
        self._model = None   # 첫 녹음이 끝날 때 지연 로딩 — 시작 속도를 안 늦추려고
        self._lock = threading.Lock()
        self._recording = False
        self._busy = False   # 변환 중
        self._frames: list[np.ndarray] = []
        self._stream: Optional[sd.InputStream] = None
        self._result: Optional[VoiceResult] = None

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._busy

    def toggle(self) -> bool:
        """음성 버튼 콜백에서 부른다. 반환값: 실제로 지금 녹음 중이면 True
        (호출 쪽이 버튼 모양을 바꿀 수 있게) — 마이크 시작에 실패하면
        내부적으로 다시 False 로 되돌리므로, 미리 계산한 의도가 아니라
        시도 후의 실제 상태를 돌려준다."""
        with self._lock:
            if self._busy:
                return False   # 이전 녹음 변환 중이면 새로 시작 안 함
            starting = not self._recording
        if starting:
            self._start()
        else:
            self._stop_and_transcribe()
        with self._lock:
            return self._recording

    def _start(self) -> None:
        with self._lock:
            self._recording = True
            self._frames = []

        def _on_audio(indata, frames, time_info, status) -> None:
            with self._lock:
                if self._recording:
                    self._frames.append(indata.copy())

        try:
            self._stream = sd.InputStream(
                samplerate=SAMPLE_RATE, channels=1, dtype="float32", callback=_on_audio,
                device=self._device)
            self._stream.start()
        except Exception as exc:
            # 장치 번호가 잘못됐거나(WHISPER_MIC_DEVICE 오지정) 마이크 권한이
            # 없는 등 — 여기서 죽으면 GUI 콜백 안에서 죽는 거라 조용히 실패로
            # 보일 수 있으므로, 바로 에러 결과를 만들어 poll_result() 로
            # 넘긴다(run_mission.py 가 화면에 표시).
            with self._lock:
                self._recording = False
                self._result = VoiceResult("", error=f"마이크 시작 실패: {exc}")
            self._stream = None

    def _stop_and_transcribe(self) -> None:
        with self._lock:
            self._recording = False
            self._busy = True
            frames = list(self._frames)
        if self._stream is not None:
            self._stream.stop()
            self._stream.close()
            self._stream = None
        threading.Thread(target=self._run_transcribe, args=(frames,), daemon=True).start()

    def _run_transcribe(self, frames: list[np.ndarray]) -> None:
        try:
            if not frames:
                result = VoiceResult("", error="녹음된 소리가 없음")
            else:
                audio = np.concatenate(frames, axis=0).reshape(-1)
                if len(audio) < SAMPLE_RATE * 0.3:
                    result = VoiceResult("", error="녹음이 너무 짧음")
                else:
                    if self._model is None:
                        from faster_whisper import WhisperModel
                        self._model = WhisperModel(
                            self._model_size, device="cpu", compute_type="int8")
                    segments, _info = self._model.transcribe(audio, language="ko")
                    text = "".join(seg.text for seg in segments).strip()
                    result = VoiceResult(text) if text else VoiceResult("", error="인식된 말이 없음")
        except Exception as exc:
            result = VoiceResult("", error=str(exc))
        with self._lock:
            self._result = result
            self._busy = False

    def poll_result(self) -> Optional[VoiceResult]:
        """한 번 소비하면 다시 None(일회성 이벤트)."""
        with self._lock:
            result, self._result = self._result, None
            return result
