"""로컬 정책 추론 — ROS 에 의존하지 않는다(노트북·Pi·policy_server 가 같이 쓴다).

입력:  BGR uint8 프레임(임의 해상도), 관절값 6개(정책 단위), 지시문
출력:  액션 청크 [n_action_steps, 6] (정책 단위, 절대 관절 목표)

전처리는 실기로 물체를 집은 기준 롤아웃(hardware/grippers/tools/arm/rollout_policy.py)과
같게 맞춘다. 아래 함정은 전부 기존 실기에서 확인된 것이다.

1. **색 순서.** LeRobot OpenCVCamera 의 기본 color_mode 가 RGB 라 녹화 데이터와 기준
   롤아웃이 RGB 였다. 기존 ROS 경로는 BGR 을 그대로 넣었다. 여기서는
   `image_color` 설정으로 명시하고, 변환은 이 파일 한 곳에서만 한다.
2. **입력 해상도는 train_config.json 에 있다.** config.json 의 input_features 는 원본
   크기(3x720x1280)를 적어 둔다. 그걸 믿으면 정책이 본 적 없는 해상도가 오류 없이 들어간다.
   리사이즈는 torch bilinear, align_corners=False (기준 롤아웃과 동일).
3. **device 를 덮어야 한다.** 학습 장치(xpu/cuda)가 전처리기에 박혀 있다.
4. **wrist_roll 관측을 학습 평균으로 덮는다.** 캘리브레이션 범위가 14틱뿐이라 1틱만
   흔들려도 정규화값이 튀고, 정책이 이미지를 무시하고 같은 액션을 낸다(2026-09-02).
5. **DP 는 관측을 n_obs_steps 개 쌓아 받는다.** 하나를 복제해 채운다. DP 설정에는
   chunk_size 가 없고 horizon 이 있다.
6. **n_action_steps 를 줄이지 말 것.** ACT 는 시간 정보를 청크 안에 담는다. 30 으로
   줄였더니 같은 청크를 무한 반복하며 팔이 25초간 멈췄다.
7. **config.json 의 모르는 필드**(예: pretrained_revision)로 적재가 죽는 lerobot 버전이
   있다. 적재 실패 시 모르는 필드를 뺀 사본으로 한 번 더 시도한다.
"""
from __future__ import annotations

import dataclasses
import json
import shutil
import tempfile
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from vla_common.arm_units import NUM_JOINTS, WRIST_ROLL_INDEX


def read_train_resize(checkpoint: str | Path) -> Optional[tuple[int, int]]:
    """train_config.json 의 dataset.image_transforms resize [H, W]. 없으면 None."""
    path = Path(checkpoint) / "train_config.json"
    try:
        tf = json.loads(path.read_text(encoding="utf-8"))["dataset"]["image_transforms"]
    except (OSError, KeyError, TypeError, json.JSONDecodeError):
        return None
    if not tf.get("enable") or "resize" not in tf.get("tfs", {}):
        return None
    size = tf["tfs"]["resize"]["kwargs"]["size"]
    return int(size[0]), int(size[1])


def _load_config_tolerant(checkpoint: str):
    """PreTrainedConfig 적재. 모르는 필드로 실패하면 걸러낸 사본으로 재시도한다.

    반환: (cfg, 실제로 쓸 체크포인트 경로, 임시 디렉터리 또는 None)
    """
    from lerobot.configs.policies import PreTrainedConfig

    try:
        return PreTrainedConfig.from_pretrained(checkpoint), checkpoint, None
    except Exception as first_error:  # noqa: BLE001 — lerobot 버전마다 예외 종류가 다르다
        src = Path(checkpoint)
        raw = json.loads((src / "config.json").read_text(encoding="utf-8"))
        try:
            choice = PreTrainedConfig.get_choice_class(raw["type"])
        except Exception:  # noqa: BLE001
            raise first_error
        allowed = {f.name for f in dataclasses.fields(choice)} | {"type"}
        dropped = sorted(set(raw) - allowed)
        if not dropped:
            raise
        tmp = Path(tempfile.mkdtemp(prefix="vla_ckpt_"))
        for item in src.iterdir():
            if item.is_file() and item.name != "config.json":
                try:
                    (tmp / item.name).symlink_to(item.resolve())
                except OSError:
                    shutil.copy2(item, tmp / item.name)
        (tmp / "config.json").write_text(
            json.dumps({k: v for k, v in raw.items() if k in allowed}, indent=2), encoding="utf-8")
        print(f"[policy_runner] config.json 의 모르는 필드를 빼고 적재: {dropped}")
        return PreTrainedConfig.from_pretrained(str(tmp)), str(tmp), tmp


class PolicyRunner:
    """체크포인트 하나를 들고 (BGR 프레임, 관절값, 지시문) -> 액션 청크."""

    def __init__(self, checkpoint: str, device: str = "cpu", image_color: str = "rgb",
                 image_size: Optional[Sequence[int]] = None,
                 wrist_roll_value: Optional[float] = 0.067,
                 n_action_steps: Optional[int] = None,
                 num_inference_steps: Optional[int] = None,
                 noise_scheduler_type: Optional[str] = None) -> None:
        import torch
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors

        if image_color not in ("rgb", "bgr"):
            raise ValueError(f"image_color 는 rgb|bgr: {image_color}")
        self._torch = torch
        self.checkpoint = str(checkpoint)
        self.device = device
        self.image_color = image_color
        self.wrist_roll_value = wrist_roll_value

        cfg, load_path, self._tmpdir = _load_config_tolerant(self.checkpoint)
        cfg.device = device
        if n_action_steps:
            cfg.n_action_steps = int(n_action_steps)
        # 확산 정책 전용. ACT 에는 없는 필드라 줘도 무시된다.
        # ⚠️ 이 모델(학습 체인 100스텝)에서는 DDIM 이 같은 시간에 DDPM 보다 오차가 두 배였다.
        if noise_scheduler_type:
            cfg.noise_scheduler_type = noise_scheduler_type
        if num_inference_steps:
            cfg.num_inference_steps = int(num_inference_steps)
        self.cfg = cfg
        self.policy_type = str(getattr(cfg, "type", "?"))

        self.policy = get_policy_class(cfg.type).from_pretrained(load_path, config=cfg)
        self.policy.to(device).eval()
        self.pre, self.post = make_pre_post_processors(
            cfg, pretrained_path=load_path,
            preprocessor_overrides={"device_processor": {"device": device}},
        )

        size = tuple(int(v) for v in image_size) if image_size and all(image_size) else None
        self.policy_hw = size or read_train_resize(self.checkpoint)
        if self.policy_hw is None:
            raise RuntimeError(
                f"{self.checkpoint}/train_config.json 에서 학습 리사이즈를 찾지 못했다. "
                "policy.image_size 로 직접 줄 것 — 원본 크기로 넣으면 조용히 틀린다.")

        self.chunk_size = int(getattr(cfg, "chunk_size", None) or getattr(cfg, "horizon", None)
                              or cfg.n_action_steps)
        self.n_action_steps = int(cfg.n_action_steps)
        self.n_obs_steps = int(getattr(cfg, "n_obs_steps", 1) or 1)
        self.image_key = self._find_image_key(cfg)

    @staticmethod
    def _find_image_key(cfg) -> str:
        keys = [k for k in getattr(cfg, "input_features", {}) if k.startswith("observation.images.")]
        if len(keys) != 1:
            raise RuntimeError(f"카메라 입력이 정확히 1개여야 한다: {keys}")
        return keys[0]

    def describe(self) -> dict:
        return {
            "ckpt": self.checkpoint,
            "policy_type": self.policy_type,
            "policy_hw": list(self.policy_hw),
            "chunk_size": self.chunk_size,
            "n_action_steps": self.n_action_steps,
            "n_obs_steps": self.n_obs_steps,
            "image_color": self.image_color,
            "image_key": self.image_key,
        }

    def _prepare(self, image_bgr: np.ndarray, state6: Sequence[float]):
        torch = self._torch
        if len(state6) != NUM_JOINTS:
            raise ValueError(f"state 는 6개여야 한다: {len(state6)}")
        image = np.asarray(image_bgr)
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"uint8 HxWx3 BGR 프레임이어야 한다: {image.dtype} {image.shape}")
        if self.image_color == "rgb":
            image = image[:, :, ::-1]
        state = torch.tensor([float(v) for v in state6], dtype=torch.float32)
        if self.wrist_roll_value is not None:
            state[WRIST_ROLL_INDEX] = float(self.wrist_roll_value)
        img = torch.from_numpy(np.ascontiguousarray(image)).permute(2, 0, 1).float() / 255.0
        if (img.shape[1], img.shape[2]) != tuple(self.policy_hw):
            img = torch.nn.functional.interpolate(
                img.unsqueeze(0), size=tuple(self.policy_hw), mode="bilinear",
                align_corners=False).squeeze(0)
        return state, img

    def predict_chunk(self, image_bgr: np.ndarray, state6: Sequence[float], task: str) -> np.ndarray:
        torch = self._torch
        state, img = self._prepare(image_bgr, state6)
        if self.n_obs_steps > 1:
            states = torch.stack([state] * self.n_obs_steps).unsqueeze(0)
            images = torch.stack([img] * self.n_obs_steps).unsqueeze(0)
        else:
            states, images = state.unsqueeze(0), img.unsqueeze(0)
        batch = {"observation.state": states, self.image_key: images, "task": [task]}
        with torch.no_grad():
            self.policy.reset()
            chunk = self.policy.predict_action_chunk(self.pre(batch))
            b, t, d = chunk.shape
            out = self.post(chunk.reshape(b * t, d)).reshape(b, t, d)[0]
        result = out.float().cpu().numpy()[: self.n_action_steps]
        if result.shape[1] != NUM_JOINTS:
            raise RuntimeError(f"정책 출력 차원이 6이 아니다: {result.shape}")
        return result

    def close(self) -> None:
        if self._tmpdir is not None:
            shutil.rmtree(self._tmpdir, ignore_errors=True)
            self._tmpdir = None
