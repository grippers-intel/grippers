"""탑뷰 프레임에서 기물을 찾는 검출기.

`Detector` 는 넌블로킹이다: 메인 루프는 매 프레임 `submit()` 만 부르고 `latest()` 로
그때까지 나온 가장 최근 결과를 가져간다. Geti RT-DETR 추론이 카메라당 ~0.8 s 라
메인 루프에서 직접 부르면 ArUco 추적이 1 Hz 로 떨어지기 때문이다.

geti-sdk 는 **선택 의존성**이다. `kind: none` 이면 import 하지 않는다.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Protocol

import numpy as np


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    x: float        # bbox 좌상단 (px)
    y: float
    w: float
    h: float

    @property
    def bottom_center(self) -> tuple[float, float]:
        return (self.x + self.w / 2.0, self.y + self.h)


class Detector(Protocol):
    def submit(self, cam_index: int, frame_bgr: np.ndarray) -> None: ...
    def latest(self, cam_index: int) -> Optional[list[Detection]]: ...
    def close(self) -> None: ...


class NoneDetector:
    """검출기 없이 돌 때(ArUco 만 확인하거나 사람이 기물 위치를 모를 때)."""

    def submit(self, cam_index, frame_bgr) -> None:
        pass

    def latest(self, cam_index):
        return []

    def close(self) -> None:
        pass


class _GetiWorker:
    """카메라 한 대분. Deployment 를 **카메라마다 따로** 가진다 — 하나를 공유하면
    두 스레드가 동시에 infer() 를 불러 "Infer Request is busy" 로 죽는다."""

    def __init__(self, deployment, name: str, cfg) -> None:
        self._dep = deployment
        self.name = name
        self._cfg = cfg
        self._frame: Optional[np.ndarray] = None
        self._result: Optional[list[Detection]] = None
        self._lock = threading.Lock()
        self._new = threading.Event()
        self._stop = False
        self._last_infer = 0.0
        self._thread = threading.Thread(target=self._run, name=f"geti-{name}", daemon=True)
        self._thread.start()

    def submit(self, frame_bgr: np.ndarray) -> None:
        with self._lock:
            self._frame = frame_bgr
        self._new.set()

    def latest(self) -> Optional[list[Detection]]:
        with self._lock:
            return self._result

    def stop(self) -> None:
        self._stop = True
        self._new.set()
        self._thread.join(timeout=2.0)

    def _convert(self, prediction) -> list[Detection]:
        out = []
        for ann in prediction.annotations:
            best = max(ann.labels, key=lambda lb: lb.probability, default=None)
            if best is None or best.name in self._cfg.empty_labels:
                continue
            if best.probability < self._cfg.conf_threshold:
                continue
            s = ann.shape
            out.append(Detection(best.name, float(best.probability),
                                 float(s.x), float(s.y), float(s.width), float(s.height)))
        return out

    def _run(self) -> None:
        import cv2
        while not self._stop:
            if not self._new.wait(timeout=0.5):
                continue
            self._new.clear()
            # 기물은 로봇이 옮기기 전엔 안 움직인다. 최소 간격만큼 쉬어 메인 루프에
            # CPU 를 돌려준다(0 s: 1.7 Hz, 0.3 s: 8.3 Hz 실측).
            wait_left = self._cfg.min_infer_interval_s - (time.monotonic() - self._last_infer)
            while wait_left > 0 and not self._stop:
                time.sleep(min(wait_left, 0.1))
                wait_left -= 0.1
            with self._lock:
                frame = self._frame
            if frame is None or self._stop:
                continue
            try:
                # Geti 는 RGB 를 기대한다. OpenCV 프레임은 BGR 이다.
                prediction = self._dep.infer(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                result = self._convert(prediction)
            except Exception as exc:  # noqa: BLE001 — 스레드가 죽으면 지도가 영영 멈춘다
                print(f"[detector] ⚠️ {self.name}: geti 추론 오류 — {exc}")
                continue
            finally:
                self._last_infer = time.monotonic()
            with self._lock:
                self._result = result


class GetiDetector:
    def __init__(self, cfg, cam_indices) -> None:
        try:
            from geti_sdk.deployment import Deployment
        except ImportError as exc:
            raise RuntimeError(
                "geti-sdk 가 없습니다 (pip install geti-sdk). 검출기 없이 돌리려면 "
                "--detector none") from exc
        path = Path(cfg.deployment_dir)
        if not path.exists():
            raise RuntimeError(
                f"Geti 배포 폴더가 없습니다: {path}\n"
                "hardware\\grippers_topview\\geti_sdk-deployment\\deployment 를 이 경로로 복사하세요")
        ov_config = None
        if cfg.cache_dir:
            # iGPU 캐시 없음 30 s -> 캐시 3 s. CPU 에서도 손해가 없어 항상 켠다.
            Path(cfg.cache_dir).mkdir(parents=True, exist_ok=True)
            ov_config = {"CACHE_DIR": str(cfg.cache_dir)}
        self._workers: dict[int, _GetiWorker] = {}
        for idx in cam_indices:
            dep = Deployment.from_folder(str(path))
            dep.load_inference_models(device=cfg.device, openvino_configuration=ov_config)
            self._workers[int(idx)] = _GetiWorker(dep, f"cam{idx}", cfg)

    def submit(self, cam_index: int, frame_bgr: np.ndarray) -> None:
        w = self._workers.get(int(cam_index))
        if w is not None:
            w.submit(frame_bgr.copy())

    def latest(self, cam_index: int):
        w = self._workers.get(int(cam_index))
        return None if w is None else w.latest()

    def close(self) -> None:
        for w in self._workers.values():
            w.stop()


def make_detector(cfg, cam_indices) -> Detector:
    if cfg.kind == "none":
        return NoneDetector()
    return GetiDetector(cfg, cam_indices)
