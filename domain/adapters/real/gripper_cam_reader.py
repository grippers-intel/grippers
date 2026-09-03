"""GripperCamReader — /dev/gripper_cam(raw V4L2)에서 그레이스케일 프레임을
읽는 얇은 하드웨어 어댑터.

rclpy를 몰라서 ROS2 노드 밖에서도(CLI 도구, pytest 없이 실기 스모크 테스트
등) 그대로 쓸 수 있다. 실제 파지 여부 계산은 여기 없다 — 순수 함수인
`domain/task/grasp_cam_diff.py`가 한다(⚠️ 그 모듈의 docstring 참고: 이
diff 신호는 실측으로 무효였던 방식이라 참고/로그용일 뿐, GRASP 판정에
쓰면 안 된다).

옛 `perception_node.py`의 `_open_grasp_cam`/`_capture_grasp_frame`
(2026-08-21 원안, 2026-09-01 죽은 코드로 제거)과 캡처 방식은 동일하다 —
재연결·워밍업 프레임 버리기 로직까지 그대로 옮겼다."""

import cv2

from domain.task import grasp_cam_diff as gcd

DEVICE_DEFAULT = "/dev/gripper_cam"
WIDTH = 640
HEIGHT = 480
# 노출 자동조정 전 프레임은 검게 나온다(2026-08-21 실기 확인) — 앞쪽
# 몇 프레임은 버리고 읽는다.
WARMUP_FRAMES = 5


class GripperCamReader:
    def __init__(self, device: str = DEVICE_DEFAULT):
        self._device = device
        self._cap: cv2.VideoCapture | None = None

    def _open(self) -> "cv2.VideoCapture | None":
        cap = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        return cap

    def capture_gray_frame(self):
        """그레이스케일 프레임 한 장을 잡는다. 카메라가 없거나 읽기에
        실패하면 **None** — 호출자가 "확인 불가"로 접는다.

        열려 있던 캡처가 죽어 있으면(핫플러그 재연결 등) 한 번 다시 연다."""
        if self._cap is None or not self._cap.isOpened():
            self._cap = self._open()
        if self._cap is None:
            return None
        for _ in range(WARMUP_FRAMES):
            self._cap.grab()
        ok, frame = self._cap.read()
        if not ok or frame is None:
            self._cap.release()
            self._cap = None
            return None
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __enter__(self) -> "GripperCamReader":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


class GripperCamDiffConfirm:
    """옛 `_on_confirm_grasp`와 같은 흐름(기준 프레임 한 번, 이후 반복 확인)을
    묶어 둔 편의 클래스. ⚠️ 참고/로그용이다 — grasp_cam_diff 모듈 docstring
    참고, GRASP 판정에 이 결과를 쓰지 말 것."""

    def __init__(self, reader: GripperCamReader | None = None,
                 threshold: float = gcd.GRASP_CAM_DIFF_THRESHOLD_DEFAULT):
        self._reader = reader or GripperCamReader()
        self.threshold = threshold
        self._reference = None

    def capture_reference(self) -> bool:
        """지금(빈 그리퍼 상태여야 한다) 프레임을 기준으로 잡는다.
        실패하면 False — 이후 check()는 계속 confirmed=False만 낸다."""
        self._reference = self._reader.capture_gray_frame()
        return self._reference is not None

    def check(self) -> gcd.GraspCamDiffVerdict:
        current = self._reader.capture_gray_frame()
        if current is None or self._reference is None:
            return gcd.GraspCamDiffVerdict(confirmed=False, diff_score=0.0, confidence=0.0)
        return gcd.score_grasp_diff(self._reference, current, threshold=self.threshold)
