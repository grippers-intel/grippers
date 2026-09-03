"""grasp_cam_diff — 순수 계산만 검증한다. 카메라도 ROS도 모른다."""

from __future__ import annotations

import numpy as np

from domain.task import grasp_cam_diff as gcd

FRAME_H, FRAME_W = 480, 640


def _uniform_frame(value: int) -> np.ndarray:
    return np.full((FRAME_H, FRAME_W), value, dtype=np.uint8)


def test_동일한_프레임은_diff가_0이고_미확인():
    ref = _uniform_frame(50)
    cur = _uniform_frame(50)
    verdict = gcd.score_grasp_diff(ref, cur)
    assert verdict.diff_score == 0.0
    assert verdict.confirmed is False
    assert verdict.confidence == 0.0


def test_ROI_밖의_변화는_무시한다():
    """배경(ROI 밖)만 크게 바뀌어도 신호가 안 뜬다 — ROI 크롭이 실제로
    적용되는지 확인."""
    ref = _uniform_frame(50)
    cur = ref.copy()
    cur[: int(0.55 * FRAME_H), :] = 255  # ROI(y 55%~100%) 밖 상단만 변경
    verdict = gcd.score_grasp_diff(ref, cur)
    assert verdict.diff_score == 0.0
    assert verdict.confirmed is False


def test_ROI_안의_변화는_잡는다():
    ref = _uniform_frame(50)
    cur = ref.copy()
    x0, y0, x1, y1 = gcd.GRASP_CAM_ROI
    cur[int(y0 * FRAME_H) : int(y1 * FRAME_H), int(x0 * FRAME_W) : int(x1 * FRAME_W)] = 200
    verdict = gcd.score_grasp_diff(ref, cur)
    assert verdict.diff_score == 150.0
    assert verdict.confirmed is True


def test_threshold_경계는_초과여야_확인된다():
    """diff_score == threshold 는 confirmed=False (>, >= 아님) — 원안과
    같은 부등호를 그대로 재현했는지 확인."""
    ref = _uniform_frame(0)
    cur = _uniform_frame(0)
    x0, y0, x1, y1 = gcd.GRASP_CAM_ROI
    cur[int(y0 * FRAME_H) : int(y1 * FRAME_H), int(x0 * FRAME_W) : int(x1 * FRAME_W)] = (
        gcd.GRASP_CAM_DIFF_THRESHOLD_DEFAULT
    )
    verdict = gcd.score_grasp_diff(ref, cur)
    assert verdict.diff_score == gcd.GRASP_CAM_DIFF_THRESHOLD_DEFAULT
    assert verdict.confirmed is False
    assert verdict.confidence == 0.5  # threshold / (2*threshold)


def test_confidence는_1을_넘지_않는다():
    ref = _uniform_frame(0)
    cur = _uniform_frame(255)  # ROI 전체가 최대치로 바뀜 — 큰 diff
    verdict = gcd.score_grasp_diff(ref, cur, threshold=1.0)
    assert verdict.confirmed is True
    assert verdict.confidence == 1.0


def test_음수_방향_차이도_절대값으로_잡는다():
    """현재 프레임이 기준보다 어두워지는 경우(부호 반대)도 diff는 양수여야
    한다 — int16 캐스팅 없이 uint8 그대로 빼면 언더플로로 랩어라운드된다."""
    ref = _uniform_frame(200)
    cur = _uniform_frame(50)
    verdict = gcd.score_grasp_diff(ref, cur)
    assert verdict.diff_score == 150.0
    assert verdict.confirmed is True


def test_crop_roi_는_요청한_비율만큼_잘라낸다():
    frame = _uniform_frame(0)
    cropped = gcd.crop_roi(frame, roi=(0.0, 0.0, 0.5, 0.5))
    assert cropped.shape == (FRAME_H // 2, FRAME_W // 2)
