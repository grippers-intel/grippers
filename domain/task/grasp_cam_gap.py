"""그리퍼캠 "닫은 뒤 틈 밝기"로 파지 여부를 가늠하는 2단계 신호 (2026-09-04).

`grasp_cam_diff.py`(열림→닫힘 밝기 diff)는 그리퍼가 움직이는 것 자체가
diff를 만들어버려서 물체 유무와 구분이 안 된다는 게 실기로 확인됐다(그
모듈 docstring 참고 — 열린 기준 프레임과 닫힌 프레임을 비교하는 구조라
물체 없이 그냥 여닫기만 해도 큰 diff가 뜬다). 이 모듈은 접근 자체가
다르다: **기준 프레임과 비교하지 않는다.**

## 원리

그리퍼를 완전히 닫은 뒤 프레임 한 장만 본다. 손가락 사이 ROI 안에서:

- 그리퍼가 비어 있으면 손가락 끝까지 맞물려서 ROI가 거의 다 검정(그리퍼
  자체 색)이다.
- 물체를 물었으면 손가락이 물체에 막혀 끝까지 못 닫히고, 그 틈으로
  물체나 배경이 보여서 ROI 안에 밝은 픽셀이 남는다.

그래서 "이전 프레임과 다른가"가 아니라 "지금 이 프레임의 ROI가 밝은가"만
본다 — 그리퍼가 열림에서 닫힘으로 움직이는 과정 자체와는 무관하다(비교
대상이 없으니까).

## 전제: 기물이 밝은 색이어야 한다

사용자 확인(2026-09-04): 다루는 기물은 거의 다 흰색/나무색이고, 축구공만
반점무늬로 예외다. 그래서 "밝은 픽셀 = 물체"라는 전제가 지금은 성립한다.
검정색 물체(예: 검은 폰)를 물면 이 방식도 구분 못 한다 — 그런 기물이
생기면 이 신호는 다시 무효가 된다.

## ⚠️ 이것도 실측으로 무효였다 (2026-09-04)

실기 테스트 결과: **바닥 자체가 밝다.** 빈 그리퍼를 완전히 닫아도 ROI로
보이는 바닥이 밝게 잡혀서 bright_ratio가 높게 나온다 — "밝은 픽셀 = 물체"
전제가 배경(바닥)에도 그대로 걸려서 diff 방식과 마찬가지로 물체 유무와
구분이 안 된다는 게 확인됐다. `grasp_cam_diff.py`(열림→닫힘 비교가
무효였던 것)에 이어 **닫힌 뒤 단일 프레임 밝기 방식도 무효** — 사용자
판단으로 GRASP 판정 연결을 포기하고 이 모듈은 참고/기록용 코드로만
남긴다. 그리퍼캠은 이제 판정용이 아니라 `tools/gripper_cam_stream.py`로
실시간 모니터링(육안 확인)에만 쓴다.

아래 ROI·threshold도 "그리퍼를 연 상태" 스냅샷 기준의 미실측 임시치
그대로다 — 더 이상 재보정할 계획 없음."""

from dataclasses import dataclass

import numpy as np

# 열린 상태 검정 영역 바운딩박스를 임시로 재사용 — 위 ⚠️ 참고, 그리퍼를
# 완전히 닫은 상태로 재보정 필요.
GRASP_CAM_GAP_ROI = (0.08, 0.0, 0.80, 0.35)

# 그레이스케일 밝기 컷오프 — 이 값보다 밝으면 "그리퍼(검정)가 아니다"로
# 본다. grasp_cam_diff.py에서 검정 영역 바운딩박스를 잴 때 쓴 값과 동일.
BRIGHT_PIXEL_THRESHOLD = 80

# ROI 안에서 밝은 픽셀 비율이 이 값을 넘으면 confirmed=True. 미실측
# 임시치 — 실기(빈 그리퍼 완전히 닫은 상태의 bright_ratio)로 재보정할 것.
BRIGHT_RATIO_THRESHOLD_DEFAULT = 0.05


@dataclass(frozen=True)
class GraspGapVerdict:
    confirmed: bool
    bright_ratio: float  # ROI 안에서 밝은(비-그리퍼) 픽셀 비율, 0.0~1.0
    mean_brightness: float  # 참고용 — ROI 평균 밝기(0~255)


def crop_roi(gray_frame: np.ndarray, roi=GRASP_CAM_GAP_ROI) -> np.ndarray:
    """GRASP_CAM_GAP_ROI(비율)를 실제 픽셀 슬라이스로 잘라낸다."""
    h, w = gray_frame.shape[:2]
    x0, y0, x1, y1 = roi
    return gray_frame[int(y0 * h) : int(y1 * h), int(x0 * w) : int(x1 * w)]


def score_grasp_gap(
    gray_frame: np.ndarray,
    roi=GRASP_CAM_GAP_ROI,
    bright_threshold: int = BRIGHT_PIXEL_THRESHOLD,
    ratio_threshold: float = BRIGHT_RATIO_THRESHOLD_DEFAULT,
) -> GraspGapVerdict:
    """그리퍼를 완전히 닫은 뒤 찍은 프레임 **한 장만** 넣는다 — 기준
    프레임 비교가 아니라서 이전 프레임을 몰라도 된다. 그레이스케일
    (H, W)이어야 한다 — 색 변환은 호출자(카메라 어댑터) 책임."""
    crop = crop_roi(gray_frame, roi)
    bright_ratio = float(np.mean(crop > bright_threshold))
    mean_brightness = float(np.mean(crop))
    confirmed = bright_ratio > ratio_threshold
    return GraspGapVerdict(
        confirmed=confirmed, bright_ratio=bright_ratio, mean_brightness=mean_brightness
    )
