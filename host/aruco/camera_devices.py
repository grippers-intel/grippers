"""카메라를 '번호'가 아니라 '이름'으로 찾는다.

윈도우에서 cv2.VideoCapture 의 인덱스는 고정이 아니다.
휴대폰 가상 카메라(Iriun / DroidCam / Camo ...)나 노트북 내장 캠이 끼어들면
어제 0번이던 C920 이 오늘은 2번이 된다.

여기서는 OpenCV 의 DSHOW 백엔드가 쓰는 것과 똑같은 DirectShow 장치 목록을
직접 읽어서 (인덱스, 이름) 을 얻는다. 목록의 순서가 곧 CAP_DSHOW 인덱스다.
추가 패키지는 필요 없다. ctypes 로 COM 을 직접 부른다.

단독 실행하면 이 PC 에 붙은 카메라를 전부 보여준다.
    python camera_devices.py
"""

from __future__ import annotations

import ctypes
import re
import sys

import cv2
from ctypes import POINTER, byref, c_void_p

# --- DirectShow / COM 상수 -------------------------------------------------
_CLSID_SystemDeviceEnum = "{62BE5D10-60EB-11d0-BD3B-00A0C911CE86}"
_IID_ICreateDevEnum = "{29840822-5B84-11D0-BD3B-00A0C911CE86}"
_CLSID_VideoInputDeviceCategory = "{860BB310-5D01-11d0-BD3B-00A0C911CE86}"
_IID_IPropertyBag = "{55272A00-42CB-11CE-8135-00AA004BB851}"

_CLSCTX_INPROC_SERVER = 1
_S_OK = 0


class _GUID(ctypes.Structure):
    _fields_ = [("Data1", ctypes.c_uint32),
                ("Data2", ctypes.c_uint16),
                ("Data3", ctypes.c_uint16),
                ("Data4", ctypes.c_ubyte * 8)]


def _guid(text: str) -> _GUID:
    g = _GUID()
    if ctypes.windll.ole32.CLSIDFromString(ctypes.c_wchar_p(text), byref(g)) != _S_OK:
        raise OSError(f"CLSIDFromString 실패: {text}")
    return g


def _call(ptr: c_void_p, slot: int, *args):
    """COM 인터페이스의 vtable slot 번째 메서드를 부른다."""
    vtbl = ctypes.cast(ptr, POINTER(POINTER(c_void_p)))[0]
    proto = ctypes.WINFUNCTYPE(ctypes.c_long, c_void_p, *[c_void_p] * len(args))
    return proto(vtbl[slot])(ptr, *[ctypes.cast(a, c_void_p) if not isinstance(a, int)
                                    else c_void_p(a) for a in args])


def _release(ptr: c_void_p) -> None:
    if ptr:
        _call(ptr, 2)  # IUnknown::Release


def _read_prop(bag: c_void_p, name: str) -> str:
    """IPropertyBag::Read - 문자열 속성 하나를 꺼낸다."""
    var = (ctypes.c_byte * 24)()          # VARIANT (x64 기준 24 byte)
    ctypes.windll.oleaut32.VariantInit(byref(var))
    hr = _call(bag, 3, ctypes.c_wchar_p(name), byref(var), 0)  # Read
    if hr != _S_OK:
        return ""
    bstr = ctypes.cast(byref(var, 8), POINTER(c_void_p))[0]    # VARIANT.bstrVal
    text = ctypes.wstring_at(bstr) if bstr else ""
    ctypes.windll.oleaut32.VariantClear(byref(var))
    return text


def list_video_devices() -> list[tuple[int, str, str]]:
    """(cv2 인덱스, 장치 이름, 장치 경로) 목록. 실패하면 빈 목록."""
    if not sys.platform.startswith("win"):
        return []
    ole32 = ctypes.windll.ole32
    ole32.CoInitialize(None)
    devices: list[tuple[int, str, str]] = []
    dev_enum = c_void_p()
    enum_mon = c_void_p()
    try:
        hr = ole32.CoCreateInstance(byref(_guid(_CLSID_SystemDeviceEnum)), None,
                                    _CLSCTX_INPROC_SERVER,
                                    byref(_guid(_IID_ICreateDevEnum)),
                                    byref(dev_enum))
        if hr != _S_OK or not dev_enum:
            return []
        # ICreateDevEnum::CreateClassEnumerator
        if _call(dev_enum, 3, byref(_guid(_CLSID_VideoInputDeviceCategory)),
                 byref(enum_mon), 0) != _S_OK or not enum_mon:
            return []                      # 카메라가 하나도 없는 경우
        iid_bag = _guid(_IID_IPropertyBag)
        idx = 0
        while True:
            moniker = c_void_p()
            fetched = ctypes.c_ulong(0)
            if _call(enum_mon, 3, 1, byref(moniker), byref(fetched)) != _S_OK:
                break                      # IEnumMoniker::Next
            bag = c_void_p()
            # IMoniker::BindToStorage(pbc=NULL, pmkToLeft=NULL, IID_IPropertyBag, &bag)
            if _call(moniker, 9, 0, 0, byref(iid_bag), byref(bag)) == _S_OK and bag:
                devices.append((idx,
                                _read_prop(bag, "FriendlyName"),
                                _read_prop(bag, "DevicePath")))
                _release(bag)
            else:
                devices.append((idx, "", ""))
            _release(moniker)
            idx += 1
    except OSError:
        return []
    finally:
        _release(enum_mon)
        _release(dev_enum)
    return devices


#: 이름 검색에 쓸 기본 정규식. config.py 에 CAM_NAME_PATTERN 이 있으면 그쪽이 이긴다 —
#: 이 저장소의 config.py 에는 아직 없어서(팀원 동기화 파일이라 안 건드림) 여기 둔다.
DEFAULT_NAME_PATTERN = r"C920"


def _pattern(pattern: str | None = None) -> str:
    if pattern is not None:
        return pattern
    import config as cfg
    return getattr(cfg, "CAM_NAME_PATTERN", DEFAULT_NAME_PATTERN)


def find_indices(pattern: str, want: int | None = None) -> list[int]:
    """이름이 pattern(정규식, 대소문자 무시)에 맞는 카메라의 인덱스."""
    rx = re.compile(pattern, re.I)
    hits = [i for i, name, _ in list_video_devices() if rx.search(name)]
    return hits if want is None else hits[:want]


def open_camera(index: int) -> "cv2.VideoCapture":
    """카메라 하나를 프로젝트 표준 설정으로 연다.

    해상도·버퍼뿐 아니라 **오토포커스를 끄고 초점을 고정**한다.
    C920 은 초점이 움직이면 초점거리가 같이 변해서, 캘리브레이션해 둔 값이
    그 순간부터 틀린 값이 된다. 세 실행 파일이 모두 이 함수를 쓰면
    "어떤 프로그램으로 켰느냐"에 따라 초점이 달라지는 일이 없다.
    """
    import config as cfg

    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.IMG_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.IMG_H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)          # 먼저 끄고
    focus = getattr(cfg, "CAM_FOCUS", None)     # 그다음 고정
    if focus is not None:
        cap.set(cv2.CAP_PROP_FOCUS, focus)
    return cap


class CameraNotFound(RuntimeError):
    """원하는 카메라를 못 찾았을 때. 메시지를 그대로 사용자에게 보여주면 된다."""


def resolve_indices(pattern: str | None = None,
                    want: int | None = None,
                    fallback: list[int] | None = None) -> tuple[list[int], list[str]]:
    """이름으로 카메라 인덱스를 정한다. (인덱스 목록, 이름 목록) 반환.

    pattern  : 장치 이름 정규식. 기본값은 config.CAM_NAME_PATTERN.
    want     : 필요한 대수. 기본값은 config.N_CAMS.
    fallback : 장치 목록 자체를 못 읽었을 때 쓸 예비 번호.
    """
    import config as cfg

    pattern = _pattern(pattern)
    want = getattr(cfg, "N_CAMS", len(cfg.CAM_INDICES)) if want is None else want

    devs = list_video_devices()
    if not devs:                       # 윈도우가 아니거나 COM 이 막힌 경우
        idx = list(fallback if fallback is not None else cfg.CAM_INDICES)[:want]
        return idx, ["(이름 확인 불가)"] * len(idx)

    rx = re.compile(pattern, re.I)
    hits = [(i, name) for i, name, _ in devs if rx.search(name)]
    if len(hits) < want:
        raise CameraNotFound(
            f"'{pattern}' 에 맞는 카메라가 {len(hits)}대뿐입니다 (필요: {want}대).\n"
            f"이 PC 의 카메라:\n{describe()}\n"
            "  · C920 두 대가 모두 USB 에 꽂혀 있는지 확인하세요.\n"
            "  · 다른 프로그램(줌/팀즈/카메라 앱)이 잡고 있으면 목록에서 빠질 수 있습니다.\n"
            "  · 번호를 직접 주려면: --cams 0 2")
    picked = hits[:want]
    return [i for i, _ in picked], [n for _, n in picked]


def names_of(indices: list[int]) -> list[str]:
    """인덱스에 해당하는 장치 이름 (모르면 빈 문자열)."""
    table = {i: name for i, name, _ in list_video_devices()}
    return [table.get(i, "") for i in indices]


def matches(name: str, pattern: str | None = None) -> bool:
    """장치 이름이 우리가 원하는 카메라인가."""
    return bool(re.search(_pattern(pattern), name or "", re.I))


def report(indices: list[int], names: list[str]) -> None:
    """무엇을 잡았는지 사람에게 보여준다. 엉뚱한 장치면 눈에 띄게 표시한다."""
    print("사용할 카메라")
    for i, name in zip(indices, names):
        if not name:
            print(f"  cam{i}: (이름 확인 불가)")
        elif matches(name):
            print(f"  cam{i}: {name}")
        else:
            print(f"  cam{i}: {name}   <-- 원하는 카메라가 아닙니다")


def describe() -> str:
    devs = list_video_devices()
    if not devs:
        return "  (장치 목록을 읽지 못했습니다)"
    return "\n".join(f"  {i}: {name or '(이름 없음)'}" for i, name, _ in devs)


if __name__ == "__main__":
    print("이 PC 의 카메라 (번호 = cv2.VideoCapture 인덱스)")
    print(describe())
