"""카메라 백엔드를 플랫폼별로 고른다 (2026-08-28, macOS 이식).

## 왜 이 파일이 생겼나

원본 Host 코드는 여덟 군데에서 `cv2.VideoCapture(i, cv2.CAP_DSHOW)` 를
직접 부른다. DirectShow 는 **Windows 전용 백엔드**다. macOS 에서 그 상수는
존재하지만(값 700) 그 백엔드가 없어서 `isOpened()` 가 조용히 False 를
돌려준다 — 예외가 안 나므로 "카메라가 안 잡힌다" 로만 보인다.

여기 한 곳에 모아 두고 나머지는 전부 이 함수를 부른다.

## 플랫폼별 백엔드

    Windows   CAP_DSHOW         원본이 쓰던 것. 그대로 둔다
    macOS     CAP_AVFOUNDATION  Apple 표준 캡처 프레임워크
    Linux     CAP_V4L2          참고용 (이 프로젝트에서 안 쓴다)

## ⚠️ macOS 에서 잃는 것 — 초점 고정

원본 `camera_devices.open_camera()` 는 오토포커스를 끄고 초점을 고정한다.
C920 은 초점이 움직이면 초점거리가 같이 변해서, 캘리브레이션해 둔 내부
파라미터가 그 순간부터 틀린 값이 되기 때문이다.

**OpenCV 의 AVFoundation 백엔드는 `CAP_PROP_AUTOFOCUS` 와 `CAP_PROP_FOCUS`
를 지원하지 않는다.** `cap.set()` 이 False 를 돌려주고 아무 일도 안 일어난다.
즉 macOS 에서는 초점이 카메라 펌웨어 마음대로 움직일 수 있고, ArUco 위치
정확도가 Windows 만큼 안 나올 수 있다.

`open_capture()` 는 이것을 조용히 넘기지 않고 **경고를 돌려준다.** 정확도가
중요한 작업(캘리브레이션, 마커 배치)에서는 이 경고를 사람이 봐야 한다.

회피 방법은 카메라 쪽에 있다 — C920 은 웹캠 설정 유틸리티나 UVC 명령으로
초점을 고정해 둘 수 있고, 한 번 고정하면 OpenCV 가 안 건드린다.

## ⚠️ 장치 인덱스 순서

Windows 에서는 DirectShow 열거 순서가 곧 `cv2.VideoCapture` 인덱스였다.
macOS 에는 그런 보장이 없다. `list_devices()` 는 `system_profiler` 로 이름을
읽지만, 그 순서가 cv2 인덱스와 같다는 보장은 없으므로 **실제로 열어 보는
방식(`probe_indices`)** 을 같이 제공한다.
"""
from __future__ import annotations

import json
import subprocess
import sys

import cv2

IS_WINDOWS = sys.platform.startswith("win")
IS_MACOS = sys.platform == "darwin"

if IS_WINDOWS:
    BACKEND = cv2.CAP_DSHOW
    BACKEND_NAME = "DirectShow"
elif IS_MACOS:
    BACKEND = cv2.CAP_AVFOUNDATION
    BACKEND_NAME = "AVFoundation"
else:
    BACKEND = getattr(cv2, "CAP_V4L2", cv2.CAP_ANY)
    BACKEND_NAME = "V4L2"

#: macOS 카메라 권한이 없을 때 쓸 안내.
#
# TCC(개인정보 보호) 가 막으면 OpenCV 는 예외를 안 던진다. stderr 에
#   "OpenCV: not authorized to capture video (status 0)"
# 를 찍고 `isOpened()` 가 False 를 돌려줄 뿐이다. 카메라가 없는 것과 구별이
# 안 돼서, 모르면 이식이 깨진 줄 안다. 실제로 이식 중에 이걸 만났다.
PERMISSION_HINT = (
    "⚠️ macOS 카메라 권한이 없을 수 있다. 시스템 설정 > 개인정보 보호 및 보안 >"
    " 카메라 에서 이 프로그램을 실행하는 앱(터미널/iTerm/VS Code)을 켤 것."
    " 목록에 없으면 카메라를 한 번 열어 본 뒤 다시 볼 것 — 시도해야 목록에 생긴다."
)

#: macOS 에서 초점 고정이 안 될 때 쓸 안내. 호출부가 사람에게 보여 준다.
FOCUS_WARNING = (
    "⚠️ 이 플랫폼에서는 오토포커스를 끌 수 없다 — 초점이 움직이면 캘리브레이션"
    " 값이 틀려진다. 카메라 유틸리티로 초점을 미리 고정해 둘 것."
)


def open_capture(index: int) -> cv2.VideoCapture:
    """플랫폼에 맞는 백엔드로 카메라를 연다. 설정은 하지 않는다.

    프로젝트 표준 설정(해상도·버퍼·초점)까지 필요하면
    `aruco.camera_devices.open_camera()` 를 쓸 것 — 그쪽이 이 함수를 부른다."""
    return cv2.VideoCapture(index, BACKEND)


def lock_focus(cap: cv2.VideoCapture, focus=None) -> str | None:
    """오토포커스를 끄고 초점을 고정한다. 실패하면 경고 문자열을 돌려준다.

    `cap.set()` 은 지원하지 않는 속성에 대해 예외가 아니라 **False** 를
    돌려준다. 그것을 확인하지 않으면 "껐다고 생각했는데 안 꺼진" 상태로
    캘리브레이션을 하게 된다."""
    ok = cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    if focus is not None:
        ok = cap.set(cv2.CAP_PROP_FOCUS, float(focus)) and ok
    return None if ok else FOCUS_WARNING


def _macos_devices() -> list[tuple[int, str, str]]:
    """system_profiler 로 카메라 이름을 읽는다.

    ⚠️ 여기서 나온 순서가 cv2 인덱스와 같다는 보장이 없다. 이름만 참고하고,
    실제 인덱스는 `probe_indices()` 로 확인할 것."""
    try:
        out = subprocess.run(
            ["system_profiler", "-json", "SPCameraDataType"],
            capture_output=True, text=True, timeout=15,
        )
    except (OSError, subprocess.SubprocessError):
        return []
    if out.returncode != 0:
        return []
    try:
        data = json.loads(out.stdout)
    except json.JSONDecodeError:
        return []
    devices = []
    for idx, cam in enumerate(data.get("SPCameraDataType", [])):
        name = cam.get("_name", "")
        uid = cam.get("spcamera_unique-id", "") or cam.get("spcamera_model-id", "")
        devices.append((idx, name, uid))
    return devices


def probe_indices(limit: int = 8) -> list[int]:
    """0..limit-1 을 실제로 열어 보고 되는 인덱스만 돌려준다.

    이름을 못 믿는 플랫폼(macOS)에서 인덱스를 확정하는 최후의 수단이다.
    카메라를 여닫으므로 느리다 — 진단용으로만 쓸 것."""
    found = []
    for i in range(limit):
        cap = open_capture(i)
        try:
            if cap.isOpened():
                found.append(i)
        finally:
            cap.release()
    return found


def diagnose() -> str:
    """카메라가 안 잡힐 때 사람이 읽을 진단문.

    "장치는 보이는데 하나도 못 연다" 는 macOS 에서 거의 항상 권한 문제다.
    그 경우를 따로 짚어 주지 않으면 카메라 고장이나 이식 실패로 오해한다."""
    lines = [platform_note()]
    named = _macos_devices() if IS_MACOS else []
    if IS_MACOS:
        lines.append("system_profiler 장치: "
                     + (", ".join(f"[{i}] {n}" for i, n, _ in named) if named else "(없음)"))
    opened = probe_indices(limit=4)
    lines.append(f"실제로 열린 인덱스: {opened if opened else '(없음)'}")
    if IS_MACOS and named and not opened:
        lines.append(PERMISSION_HINT)
    elif not opened:
        lines.append("카메라 연결과 다른 프로그램의 점유를 확인할 것.")
    return "\n".join(lines)


def platform_note() -> str:
    return f"{sys.platform} / OpenCV 백엔드 {BACKEND_NAME}({BACKEND})"
