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

2026-09-04에 대안으로 시도한 "완전히 닫은 뒤 단일 프레임 밝기"
방식(`grasp_cam_gap.py`)도 바닥 자체가 밝아서 마찬가지로 실측 무효였다 —
그 모듈 docstring 참고. 그리퍼캠 기반 판정은 이 둘 다 실패로 결론 났고,
그리퍼캠은 이제 `tools/gripper_cam_stream.py`로 실시간 모니터링(육안
확인)에만 쓴다.

## 원안과 무엇이 같고 다른가

숫자(ROI, 임계값, diff 계산식)는 2026-08-21 원안과 완전히 같다 — 그때도
"미실측 임시치"였고 지금도 그렇다. 다른 점은 이 파일이 카메라·ROS2를 전혀
모르는 순수 함수로만 돼 있다는 것이다(옛 버전은 `perception_node.py` 안에
캡처와 계산이 뒤섞여 있어 pytest로 못 돌렸다). 카메라를 여는 쪽은
`domain/adapters/real/gripper_cam_reader.py`에 따로 뒀다."""

from dataclasses import dataclass

import numpy as np

# (x0, y0, x1, y1) — 프레임 폭/높이에 대한 비율.
#
# 2026-09-03 실기 스냅샷(gripper_cam_snapshot.py)으로 재확인: 이 카메라는
# 거꾸로 붙어 있어서(또는 마운트 각도상) 그리퍼 손가락(검은 물체)이 프레임
# "하단 중앙"이 아니라 "상단"에, 두 손가락이 아래로 갈수록 좁아지며 만나는
# 모양으로 잡힌다. 처음엔 손가락 사이 틈(x 0.40~0.54)만 좁게 잡았는데,
# 사용자 판단으로 손가락 사이 틈뿐 아니라 검정 그리퍼 영역 전체(양쪽
# 손가락 + 그 사이 틈)가 다 들어가야 한다고 정정 — 스냅샷을 밝기 임계값
# (<80=검정)으로 스캔해 검정 영역 전체의 바운딩 박스(x 0.10~0.79,
# y 0.0~0.33)를 구하고 여유를 조금 더 둬서 아래 값을 잡았다. 예전 값
# (0.30, 0.55, 0.70, 1.00, 프레임 하단부)은 바닥을 보고 있어서 그리퍼와
# 무관한 배경(바구니·다른 기물)에 반응했다 — 그게 2026-09-03 오탐(빈
# 그리퍼인데 배경에 기물/바구니가 있으면 confirmed=True)의 직접 원인이었다.
# 카메라 장착이 다시 바뀌면 같이 바뀔 수 있으니 재장착 후
# tools/gripper_cam_stream.py(실시간)나 gripper_cam_snapshot.py(스냅샷)로
# 재확인할 것.
GRASP_CAM_ROI = (0.08, 0.0, 0.80, 0.35)

# 옛 ROI(하단부, 바닥 보임) 기준 실측 4건(2026-08-21, n=1 각각)의 임시치 —
# 위 ⚠️ 참고, "실측 확정"이 아니다. 빈 그리퍼 4.65 · 축구공(위치 이탈) 1.88 ·
# 별 7.46 · 큐브 10.97. 2026-09-03 ROI를 손가락 사이 틈으로 옮겼으니 이
# 수치들은 더 이상 그대로 안 맞는다 — 재측정 전까지의 임시치일 뿐이다.
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
