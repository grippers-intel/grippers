"""rerun 에 보낼 이미지를 줄여 녹화 루프가 30Hz 를 지키게 한다.

## 왜 필요한가

`--display_data=true` 를 켜면 LeRobot 이 관측 이미지를 매 프레임 rerun 으로
보낸다(`lerobot_record.py:415`). 1280x720 을 JPEG 으로 압축하는 비용이 크다.
2026-09-02 실측(뷰어 없이 인코딩+큐잉만):

    1280x720  압축 없이      3.4 ms
    1280x720  JPEG q75      19.2 ms
    320x180   JPEG q75       1.7 ms
    리사이즈만               0.5 ms

뷰어가 붙어 있으면 전송·렌더 역압까지 얹혀 **프레임당 56ms** 가 됐다. 30Hz 의
예산이 33.3ms 인데 그 하나가 예산을 넘긴다. 같은 부하를 재현하면:

    팔 + PNG           루프 중앙  1.5 ms   29.8 Hz
    팔 + PNG + rerun   루프 중앙 57.8 ms   17.2 Hz

루프가 못 따라가면 카메라 프레임이 낡아 `latest frame is too old` 로 죽는다.
카메라 자체는 멀쩡한데(대기 0.0ms) 화면이 멈추는 것처럼 보이는 이유다.

## 무엇을 하나

화면은 배치와 접근을 눈으로 확인하는 용도라 원본 해상도가 필요 없다. 긴 변을
`MAX_EDGE` 로 줄여서 보낸다. 같은 부하로 재측정한 결과:

    원본 1280x720   루프 중앙 57.8 ms   17.2 Hz   <- 예산 초과
    긴 변 480px     루프 중앙 27.0 ms   29.5 Hz
    긴 변 320px     루프 중앙 21.4 ms   29.6 Hz   <- 예산 대비 12ms 여유

실제 녹화에는 리더 읽기·send_action·add_frame 이 더 얹히므로 여유를 남긴다.

데이터셋에 저장되는 이미지는 **건드리지 않는다.** 여기서 줄이는 것은 rerun 으로
가는 사본뿐이다.
"""
from __future__ import annotations

#: rerun 으로 보낼 이미지의 긴 변(px). 배치 확인에는 이 정도면 충분하다.
MAX_EDGE = 320


def patch_rerun_downscale(max_edge: int = MAX_EDGE) -> bool:
    """`lerobot_record` 가 부르는 log_rerun_data 를 감싸 이미지를 줄인다.

    `lerobot_record.py:149` 가 이름으로 임포트하므로 그 모듈의 참조를 바꾼다 —
    원본 모듈만 고치면 이미 바인딩된 이름에는 반영되지 않는다.
    """
    import numpy as np
    from lerobot.scripts import lerobot_record

    if getattr(lerobot_record, "_rerun_downscale_patched", False):
        return False
    original = lerobot_record.log_rerun_data

    def shrink(v):
        if not isinstance(v, np.ndarray) or v.ndim != 3:
            return v
        arr = v
        chw = arr.shape[0] in (1, 3, 4) and arr.shape[-1] not in (1, 3, 4)
        if chw:
            arr = np.transpose(arr, (1, 2, 0))
        h, w = arr.shape[:2]
        if max(h, w) <= max_edge:
            return v
        import cv2

        s = max_edge / max(h, w)
        out = cv2.resize(arr, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)
        return np.transpose(out, (2, 0, 1)) if chw else out

    def wrapper(observation=None, action=None, compress_images=False):
        if observation:
            observation = {k: shrink(v) for k, v in observation.items()}
        return original(observation=observation, action=action,
                        compress_images=compress_images)

    lerobot_record.log_rerun_data = wrapper
    lerobot_record._rerun_downscale_patched = True
    return True
