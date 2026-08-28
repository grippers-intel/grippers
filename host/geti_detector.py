"""Geti SDK 객체 인식을 탑뷰 카메라 프레임에 얹는다.

ArUco 위치 추정(localizer.py)과는 완전히 별개의 기능이다. run_localize.py 의
메인 루프는 ArUco 추적이 목적이라 카메라 프레임 속도로 돌아야 하는데,
CPU 로 돌리는 geti 추론은 프레임 한 장에 ~0.8초가 걸린다(RTDetr, 1280x720
실측). 메인 루프에서 그대로 부르면 ArUco 추적 자체가 초당 1프레임 수준으로
느려져 버린다. 그래서 카메라별로 백그라운드 스레드를 하나씩 두고, 스레드가
끝낸 가장 최근 결과를 메인 루프가 그때그때 가져다 그리는 방식으로 뗀다.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from geti_sdk.deployment import Deployment
from geti_sdk.data_models.predictions import Prediction
from geti_sdk.utils import show_image_with_annotation_scene

import mission_config as mcfg

DEPLOYMENT_DIR = Path(__file__).parent / "geti_sdk-deployment" / "deployment"

# 컴파일된 모델 커널을 캐시할 폴더. 레포에 넣지 않는다(.gitignore) —
# 기계마다 드라이버 버전마다 내용이 다르고, 지워도 다시 만들어진다.
CACHE_DIR = Path(__file__).parent / ".ov_cache"


def load_deployment(device: str = "CPU", cache: bool = True) -> Deployment:
    """geti_sdk-deployment/deployment 폴더에서 모델을 불러와 추론 준비까지 한다.

    ⚠️ 반환된 Deployment 는 스레드 세이프하지 않다 — 내부 InferRequest 가
    동시 호출을 못 받아서, GetiWorker 두 개가 같은 인스턴스를 공유하면
    "Infer Request is busy" 오류가 난다. 카메라(=GetiWorker)마다 이 함수를
    따로 불러서 각자 자기 Deployment 를 갖게 할 것 — 호출부(run_localize.py,
    run_mission.py) 참고.

    ## 캐시 (2026-08-27 실측)

    OpenVINO 는 모델을 디바이스용 커널로 컴파일해서 올린다. GPU 는 그 컴파일이
    비싼데, `CACHE_DIR` 을 주면 결과를 디스크에 두고 다음 실행에서 재사용한다:

        iGPU · 캐시 없음   로드 30.3초        CPU · (참고)  로드 5.6초
        iGPU · 캐시 1회차  로드  3.4초
        iGPU · 캐시 2회차  로드  3.1초

    캐시를 켜면 GPU 가 CPU 보다 **빨리** 뜬다. 캐시 없이 GPU 를 쓰면 매 실행
    30초를 버리므로 기본값을 켜 둔다. CPU 에도 켜 두는 것이 손해가 아니다.

    ⚠️ **디바이스 선택은 이 함수가 정하지 않는다** (호출부의 --geti-device).
    2026-08-27 실측으로 iGPU 가 추론은 1.6배 빠르지만(468.8 -> 298.7ms)
    **메인 루프 주기는 1.60 -> 1.63Hz 로 사실상 그대로였다** — 병목이 CPU
    부족이 아니라 live_map 의 단일 스레드 렌더(273~569ms)이기 때문이다.
    이 워커는 이미 백그라운드 스레드라 메인 루프를 막지 않는다(geti 를 켜도
    9.96 -> 9.93Hz). 그래서 지금은 GPU 로 옮겨도 얻는 것이 없다 — Host 쪽
    CPU 부하가 실제로 늘면 그때 재검토할 것.
    """
    ov_config = None
    if cache:
        CACHE_DIR.mkdir(parents=True, exist_ok=True)
        ov_config = {"CACHE_DIR": str(CACHE_DIR)}
    deployment = Deployment.from_folder(str(DEPLOYMENT_DIR))
    deployment.load_inference_models(device=device, openvino_configuration=ov_config)
    return deployment


def draw(frame_bgr: np.ndarray, prediction: Prediction) -> np.ndarray:
    """검출 결과를 프레임 위에 그려서 새 BGR 이미지로 반환한다."""
    return show_image_with_annotation_scene(
        frame_bgr, prediction, show_results=False, channel_order="bgr",
    )


class GetiWorker:
    """카메라 한 대분 프레임을 백그라운드에서 계속 추론하는 워커.

    메인 루프는 매 프레임 submit() 만 부르고(넌블로킹), 그릴 때는 latest() 로
    "그 시점까지 나온 가장 최근 결과"를 가져다 쓴다. 추론이 느려도 ArUco
    추적 루프는 카메라 프레임 속도 그대로 돈다.

    기물은 로봇이 옮기기 전엔 안 움직이므로, submit() 이 아무리 자주 와도
    실제 추론은 mission_config.GETI_INFER_INTERVAL_S 간격보다 자주 다시
    돌리지 않는다(실측: 추론 자체가 카메라 1대에 ~0.8초+ 걸려서, 쉬지 않고
    계속 돌리면 메인 루프가 그만큼 CPU 를 못 받아 전체가 느려진다 — 실측
    1.7Hz까지 떨어짐). 이 사이 새로 들어온 프레임은 latest 값만 남기고
    버려진다 — 어차피 기물이 그대로라 최신 프레임이나 그 전 프레임이나
    결과가 같을 것이기 때문.
    """

    def __init__(self, deployment: Deployment, name: str) -> None:
        self.deployment = deployment
        self.name = name
        self._frame: Optional[np.ndarray] = None
        self._result: Optional[Prediction] = None
        self._lock = threading.Lock()
        self._new_frame = threading.Event()
        self._stop = False
        self._last_infer_t = 0.0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, frame_bgr: np.ndarray) -> None:
        with self._lock:
            self._frame = frame_bgr
        self._new_frame.set()

    def latest(self) -> Optional[Prediction]:
        with self._lock:
            return self._result

    def stop(self) -> None:
        self._stop = True
        self._new_frame.set()
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self._stop:
            if not self._new_frame.wait(timeout=0.5):
                continue
            self._new_frame.clear()

            # 마지막 추론 이후 최소 간격이 안 지났으면 그만큼 쉬었다 간다 —
            # 그 사이 더 최신 프레임이 들어와도 상관없다(아래서 그때 시점의
            # self._frame 을 다시 읽으므로 가장 최근 걸 쓰게 된다). 잘게
            # 쪼개 재우는 이유는 stop() 이 불렸을 때 최대 간격만큼 안 밀리고
            # 바로 빠져나가게 하기 위함.
            wait_left = mcfg.GETI_INFER_INTERVAL_S - (time.monotonic() - self._last_infer_t)
            while wait_left > 0 and not self._stop:
                time.sleep(min(wait_left, 0.1))
                wait_left -= 0.1
            if self._stop:
                break

            with self._lock:
                frame = self._frame
            if frame is None:
                continue
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            try:
                result = self.deployment.infer(rgb)
            except Exception as exc:  # 모델 추론 실패로 스레드가 죽으면 안 됨
                print(f"⚠️ {self.name}: geti 추론 오류 — {exc}")
                continue
            finally:
                self._last_infer_t = time.monotonic()
            with self._lock:
                self._result = result
