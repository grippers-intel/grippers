"""그리퍼캠 밝기 diff로 파지 여부를 가늠하는 1단계 classical CV 신호
(2026-08-21 원안, 2026-09-01 죽은 코드로 제거, 2026-09-03 참고용으로 복원).

## ⚠️ 이건 판정에 안 쓴다 — 실측으로 이미 무효였다

`domain/ports/perception.py`의 `Perception.confirm_grasp()` docstring에
실측이 남아 있다: **빈 그리퍼가 닫힌 모습(165990px²)이 룩을 문 상태
(70384px²)보다 오히려 더 컸다.** 그리퍼가 얼마나 닫혔는지가 물체를
물었는지보다 신호를 더 세게 지배한다는 뜻이다 — 아래 ROI가 손가락 실루엣
자체를 포함하므로 같은 함정을 그대로 갖고 있다. 그래서 실제 파지 판정은
`Perception.confirm_grasp()`(정면 뎁스 카메라로 "목표가 바닥에서 사라졌는가"
를 본다, load 신호와 독립된 두 번째 근거)가 맡고, 이 모듈은 **참고/로그용
신호 하나를 더 얻고 싶다는 사용자 요청**으로 원안 그대로 재구성한 것뿐이다.
GRASP 판정에 이 결과를 연결하지 말 것 — 연결하려면 먼저 위 실측을 뒤집을
새로운 ROI/지표로 재검증해야 한다.

## 원안과 무엇이 같고 다른가

숫자(ROI, 임계값, diff 계산식)는 2026-08-21 원안과 완전히 같다 — 그때도
"미실측 임시치"였고 지금도 그렇다. 다른 점은 이 파일이 카메라·ROS2를 전혀
모르는 순수 함수로만 돼 있다는 것이다(옛 버전은 `perception_node.py` 안에
캡처와 계산이 뒤섞여 있어 pytest로 못 돌렸다). 카메라를 여는 쪽은
`domain/adapters/real/gripper_cam_reader.py`에 따로 뒀다."""

from dataclasses import dataclass

import numpy as np

# (x0, y0, x1, y1) — 프레임 폭/높이에 대한 비율. 손가락은 프레임 하단
# 중앙 일부에만 작게 잡힌다(2026-08-21 실기 스냅샷 확인) — 전체 프레임으로
# diff를 내면 배경(의자·책상) 변화에 신호가 희석된다(2026-08-21 실기:
# 전체 프레임 diff는 물체 유무와 무관하게 ~1로 고정). 카메라 장착이 바뀌면
# (2026-09-03 실기: 카메라 자체가 교체됐다) 같이 바뀔 수 있으니 재장착 후
# 스냅샷으로 재확인할 것.
GRASP_CAM_ROI = (0.30, 0.55, 0.70, 1.00)

# 실측 4건(2026-08-21, n=1 각각) 기준 임시치 — 위 ⚠️ 참고, "실측 확정"이
# 아니다. 빈 그리퍼 4.65 · 축구공(위치 이탈) 1.88 · 별 7.46 · 큐브 10.97.
GRASP_CAM_DIFF_THRESHOLD_DEFAULT = 6.0


@dataclass(frozen=True)
class GraspCamDiffVerdict:
    confirmed: bool
    diff_score: float
    confidence: float  # 0.0~1.0, diff_score를 threshold의 2배로 정규화


def crop_roi(gray_frame: np.ndarray, roi=GRASP_CAM_ROI) -> np.ndarray:
    """GRASP_CAM_ROI(비율)를 실제 픽셀 슬라이스로 잘라낸다."""
    h, w = gray_frame.shape[:2]
    x0, y0, x1, y1 = roi
    return gray_frame[int(y0 * h) : int(y1 * h), int(x0 * w) : int(x1 * w)]


def score_grasp_diff(
    reference_gray: np.ndarray,
    current_gray: np.ndarray,
    roi=GRASP_CAM_ROI,
    threshold: float = GRASP_CAM_DIFF_THRESHOLD_DEFAULT,
) -> GraspCamDiffVerdict:
    """기준(빈 그리퍼) 프레임과 지금 프레임의 ROI 평균 절대 밝기 차이를 잰다.

    정교한 검출이 아니라 "뭔가 달라졌다"만 보는 1단계 임시 신호다(위 모듈
    docstring의 ⚠️ 참고 — 판정에 쓰지 말 것). 두 프레임은 이미 그레이스케일
    (H, W)이어야 한다 — 색 변환은 호출자(카메라 어댑터) 책임이다, 이 함수는
    순수 계산만 한다."""
    diff_score = float(
        np.mean(
            np.abs(
                crop_roi(current_gray, roi).astype(np.int16)
                - crop_roi(reference_gray, roi).astype(np.int16)
            )
        )
    )
    confirmed = diff_score > threshold
    confidence = max(0.0, min(1.0, diff_score / (2.0 * threshold)))
    return GraspCamDiffVerdict(confirmed=confirmed, diff_score=diff_score, confidence=confidence)
