"""grasp_cam_gap — 순수 계산만 검증한다. 카메라도 ROS도 모른다."""

from __future__ import annotations

import numpy as np

from domain.task import grasp_cam_gap as gcg

FRAME_H, FRAME_W = 480, 640


def _uniform_frame(value: int) -> np.ndarray:
    return np.full((FRAME_H, FRAME_W), value, dtype=np.uint8)


def test_ROI가_전부_어두우면_미확인():
    """빈 그리퍼(손가락 끝까지 맞물려 검정) 시뮬레이션 — bright_ratio 0."""
    frame = _uniform_frame(20)  # threshold(80)보다 어두움
    verdict = gcg.score_grasp_gap(frame)
    assert verdict.bright_ratio == 0.0
    assert verdict.confirmed is False


def test_ROI가_전부_밝으면_확인():
    """물체가 틈을 가득 채운 시뮬레이션 — bright_ratio 1.0."""
    frame = _uniform_frame(200)  # threshold(80)보다 밝음
    verdict = gcg.score_grasp_gap(frame)
    assert verdict.bright_ratio == 1.0
    assert verdict.confirmed is True


def test_ROI_밖의_밝은_영역은_무시한다():
    """배경(ROI 밖)이 밝아도 신호가 안 뜬다 — ROI 크롭이 실제로
    적용되는지 확인. 프레임 전체를 밝게 하고 ROI만 어둡게 되돌린다."""
    frame = _uniform_frame(200)
    x0, y0, x1, y1 = gcg.GRASP_CAM_GAP_ROI
    frame[int(y0 * FRAME_H) : int(y1 * FRAME_H), int(x0 * FRAME_W) : int(x1 * FRAME_W)] = 20
    verdict = gcg.score_grasp_gap(frame)
    assert verdict.bright_ratio == 0.0
    assert verdict.confirmed is False


def test_ratio_threshold_경계는_초과여야_확인된다():
    """bright_ratio == ratio_threshold는 confirmed=False(>, >= 아님) —
    grasp_cam_diff.py의 threshold 부등호와 같은 규칙을 그대로 맞춘다."""
    x0, y0, x1, y1 = gcg.GRASP_CAM_GAP_ROI
    roi_h = int(y1 * FRAME_H) - int(y0 * FRAME_H)
    roi_w = int(x1 * FRAME_W) - int(x0 * FRAME_W)
    frame = _uniform_frame(20)
    ratio_threshold = 0.5
    bright_cols = int(roi_w * ratio_threshold)  # 딱 절반만 밝게
    frame[
        int(y0 * FRAME_H) : int(y1 * FRAME_H),
        int(x0 * FRAME_W) : int(x0 * FRAME_W) + bright_cols,
    ] = 200
    verdict = gcg.score_grasp_gap(frame, ratio_threshold=ratio_threshold)
    assert abs(verdict.bright_ratio - ratio_threshold) < 0.01
    assert verdict.confirmed is False


def test_밝기_컷오프를_바꾸면_판정도_바뀐다():
    """bright_threshold를 낮추면 같은 프레임도 더 쉽게 밝다고 판정된다."""
    frame = _uniform_frame(50)
    default_verdict = gcg.score_grasp_gap(frame, bright_threshold=80)
    lowered_verdict = gcg.score_grasp_gap(frame, bright_threshold=30)
    assert default_verdict.bright_ratio == 0.0
    assert lowered_verdict.bright_ratio == 1.0


def test_mean_brightness는_참고용으로_그대로_평균을_낸다():
    frame = _uniform_frame(123)
    verdict = gcg.score_grasp_gap(frame)
    assert verdict.mean_brightness == 123.0


def test_crop_roi_는_요청한_비율만큼_잘라낸다():
    frame = _uniform_frame(0)
    cropped = gcg.crop_roi(frame, roi=(0.0, 0.0, 0.5, 0.5))
    assert cropped.shape == (FRAME_H // 2, FRAME_W // 2)
