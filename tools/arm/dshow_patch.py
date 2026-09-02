"""DSHOW 에서 해상도를 FOURCC 보다 **먼저** 걸도록 LeRobot 을 고쳐 끼운다.

`feetech_retry.py` 와 같은 이유로 여기 모은다 — 녹화(`record_retry.py`)와
배포(`rollout_policy.py`)가 같은 카메라 협상 문제를 겪는데, 사본이 둘이 되면
한쪽만 고쳐지고 데이터와 실기가 조용히 갈라진다.

## 무엇이 문제인가

LeRobot 은 FOURCC 를 먼저 건다(`camera_opencv.py:202`, "FOURCC first as it can
affect available FPS/resolution options"). MSMF·V4L2 에서는 맞는 순서지만
DSHOW 에서는 정확히 거꾸로다. 2026-09-02 실측(index 0, 1280x720):

    해상도 -> FOURCC                 MJPG   29.9 fps
    FOURCC -> 해상도                 YUY2   10.0 fps      <- LeRobot 의 순서
    해상도 -> FOURCC -> 해상도       YUY2   10.0 fps      <- 앞만 고치면 이렇게 된다
    FOURCC -> 해상도 -> FOURCC       MJPG   29.9 fps

**FOURCC 가 마지막이어야 한다.** 앞에 해상도를 걸어 두는 것만으로는 부족하다 —
LeRobot 은 FOURCC 를 건 뒤 해상도를 **한 번 더** 걸고(`_configure_resolution`),
그 마지막 해상도 설정이 협상을 YUY2 로 되돌린다. 2026-09-02 실측이다.

무압축 YUY2 는 1280x720x2x30 = 55 MB/s 라 USB 2.0 대역폭에 안 들어간다.
그래서 10fps 로 떨어지고, 녹화 부하가 얹히면 프레임이 500ms 보다 낡아
`OpenCVCamera(0) latest frame is too old` 로 죽는다.

## 왜 DSHOW 를 쓰나

MSMF 는 이 카메라의 index 0 을 여는 데 95~111초가 걸리고(2026-09-01),
2026-09-02 에는 index 0~7 이 전부 `isOpened()=False` 였다. 같은 카메라가
DSHOW 로는 즉시 열린다.
"""
from __future__ import annotations


def patch_dshow_property_order() -> bool:
    """DSHOW 카메라에서 FOURCC 를 **마지막에** 다시 건다.

    원래 함수를 먼저 돌리고(해상도·fps 가 다 걸린 뒤) FOURCC 를 한 번 더 쓴다.
    그래야 마지막 해상도 설정이 협상을 되돌리지 못한다.

    협상 결과를 읽어 확인까지 한다 — 조용히 YUY2 로 떨어지는 것이 이 버그의
    본질이라, 맞았는지 눈에 보여야 한다.
    """
    import cv2
    from lerobot.cameras.opencv.camera_opencv import OpenCVCamera

    if getattr(OpenCVCamera, "_dshow_order_patched", False):
        return False
    original = OpenCVCamera._configure_capture_settings

    def read_fourcc(cap) -> str:
        v = int(cap.get(cv2.CAP_PROP_FOURCC))
        return "".join(chr((v >> 8 * k) & 255) for k in range(4))

    def configure(self):
        result = original(self)
        want = getattr(self.config, "fourcc", None)
        if self.backend == cv2.CAP_DSHOW and want:
            self.videocapture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*want))
            got = read_fourcc(self.videocapture)
            if got != want:
                raise RuntimeError(
                    f"{self}: FOURCC 협상 실패 - {want} 를 요청했는데 {got} 가 왔습니다. "
                    "YUY2 는 1280x720@30 이 USB 2.0 대역폭에 안 들어가 10fps 가 되고, "
                    "녹화 중 'latest frame is too old' 로 죽습니다."
                )
        return result

    OpenCVCamera._configure_capture_settings = configure
    OpenCVCamera._dshow_order_patched = True
    return True
