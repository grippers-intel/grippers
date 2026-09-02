#!/usr/bin/env python3
"""맥에 직접 물린 카메라(UVC)로 YOLO(train-9) 검출을 실시간으로 본다.

## 왜 이 도구가 있나

`tools/mac_camera_view.py --live`(grippers 저장소)는 Pi에 물린 카메라를
Pi의 ROS2 노드가 처리한 결과를 SSH로 받아오는 방식이다. 2026-09-02에 같은
뎁스캠(장치명 "ASJ ZNX_NVT", Novatek 칩셋)을 이 맥에 직접(USB 허브 경유)
물렸더니 macOS가 표준 UVC 카메라로 인식했다 — Pi/ROS2 없이도 컬러
스트림을 바로 받을 수 있다는 뜻이라 이 도구를 새로 만들었다.

⚠️ 확인된 것은 **컬러(RGB) 스트림뿐**이다. 이 카메라는 1280x720 외에도
160x768 / 320x564 / 640x642 같은 표준이 아닌(세로로 긴) 해상도를 지원
목록에 냈는데, 뎁스나 IR을 프레임 아래쪽에 덧붙이는 UVC 카메라에서 흔한
패턴이다 — 그러나 그걸 실제로 어떻게 잘라내 미터 단위로 바꾸는지는 벤더
SDK(`ascamera`, Pi 전용) 없이는 알 수 없다. 그래서 이 도구는 딱 컬러
스트림 + YOLO까지만 한다. 미터 단위 거리가 필요하면 여전히 Pi가 필요하다.

## 왜 cv2.VideoCapture가 아니라 ffmpeg 서브프로세스인가

`cv2.VideoCapture(i, cv2.CAP_AVFOUNDATION)`로 이 카메라를 열면 `isOpened()`
는 True가 나오지만 `read()`가 항상 실패한다(2026-09-02 실측) — 이 카메라의
비표준 픽셀 포맷/해상도 협상을 OpenCV의 AVFoundation 백엔드가 못 다루는
것으로 보인다. 반면 `ffmpeg -f avfoundation`은 장치가 알려주는 지원 포맷
목록을 보고 `uyvy422`로 스스로 바꿔 잡아 정상적으로 프레임을 준다. 그래서
`tools/mac_camera_view.py`와 같은 방식 — 서브프로세스를 띄우고 MJPEG로
인코딩된 프레임을 stdout 파이프에서 읽는다 — 을 그대로 쓴다.

## 실행

    python3 host/mac_local_yolo_view.py --device 1

`--device`는 `ffmpeg -f avfoundation -list_devices true -i ""`가 보여주는
번호다 — camera_backend.py의 경고대로 macOS는 카메라 열거 순서를 보장하지
않으니 새로 연결할 때마다 그 목록으로 확인할 것. 창에서 's'로 지금 화면을
캡처하고, 'q'나 Esc로 끝낸다.

⚠️ 사용자 지시(2026-09-02): 클래스별 색을 다르게, 빨간 계열은 전부 제외,
글자는 눈에 띄게 크게 — tools/mac_camera_view.py의 draw_detections와 같은
규칙을 그대로 따른다.
"""
from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys
import time

import cv2
import numpy as np

CONF_GATE = 0.70
MIN_BOTTOM_Y = 290.0

# 이 맥에서 확인된 train-9 best.pt 위치(sha256이 perception_node.py가 문서화한
# 배포본과 같음: bd13ae42...). 저장소 밖 개인 작업 폴더라 커밋에 넣을 수 없어
# 기본값으로만 두고, 다른 맥/경로에서는 --model로 지정한다.
_DEFAULT_MODEL = (pathlib.Path.home() / "Desktop" / "intel"
                  / "_작업_grippers" / "best.pt")

CLASS_COLORS = {
    "rook": (219, 152, 52),      # 하늘색
    "knight": (90, 220, 90),     # 초록
    "queen": (50, 220, 220),     # 노랑
    "soccer": (220, 220, 50),    # 시안
    "box": (220, 80, 160),       # 보라
    "star": (128, 128, 0),       # 청록
}
UNKNOWN_CLASS_COLOR = (170, 170, 170)


def draw_detections(img, model) -> str:
    """tools/mac_camera_view.py의 draw_detections와 같은 규칙 —
    클래스별 색, 통과=굵은 테두리/탈락=얇은 테두리, 큰 글자."""
    result = model(img, verbose=False, conf=0.25)[0]
    names = result.names
    passed = 0
    cv2.line(img, (0, int(MIN_BOTTOM_Y)), (img.shape[1], int(MIN_BOTTOM_Y)),
             (90, 90, 90), 1, cv2.LINE_AA)
    cv2.putText(img, "y=%d gate" % MIN_BOTTOM_Y, (6, int(MIN_BOTTOM_Y) - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (140, 140, 140), 1, cv2.LINE_AA)

    for box in result.boxes:
        x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
        conf = float(box.conf[0])
        label = names[int(box.cls[0])]
        conf_ok, y_ok = conf >= CONF_GATE, y2 >= MIN_BOTTOM_Y
        ok = conf_ok and y_ok
        passed += int(ok)
        color = CLASS_COLORS.get(label, UNKNOWN_CLASS_COLOR)
        thickness = 3 if ok else 1
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
        why = "" if ok else ("  conf<%.2f" % CONF_GATE if not conf_ok else "  too high")
        text = "%s %.2f%s" % (label, conf, why)
        ty = int(y1) - 10 if y1 > 30 else int(y2) + 26
        cv2.putText(img, text, (int(x1), ty), cv2.FONT_HERSHEY_SIMPLEX, 0.85,
                    color, 2, cv2.LINE_AA)
    return "%d/%d pass gates" % (passed, len(result.boxes))


def run_ffmpeg(device: str, width: int, height: int) -> subprocess.Popen:
    cmd = [
        "ffmpeg", "-hide_banner", "-loglevel", "error",
        "-f", "avfoundation", "-framerate", "30",
        "-video_size", f"{width}x{height}", "-i", device,
        "-f", "image2pipe", "-vcodec", "mjpeg", "-q:v", "3", "-",
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def frames_from(proc: subprocess.Popen):
    """stdout MJPEG 스트림에서 JPEG SOI/EOI 경계를 찾아 프레임 단위로 자른다."""
    buf = b""
    while True:
        chunk = proc.stdout.read(4096)
        if not chunk:
            return
        buf += chunk
        while True:
            start = buf.find(b"\xff\xd8")
            end = buf.find(b"\xff\xd9")
            if start == -1 or end == -1 or end < start:
                break
            end += 2
            yield buf[start:end]
            buf = buf[end:]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="1",
                    help="ffmpeg avfoundation 장치 번호 (기본 1)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--model", default=str(_DEFAULT_MODEL))
    ap.add_argument("--capture-dir", type=pathlib.Path,
                    default=pathlib.Path.home() / ".grippers_camview" / "captures")
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.model)
    print("MODEL", args.model)

    proc = run_ffmpeg(args.device, args.width, args.height)
    window = "grippers local camera — YOLO(train-9)"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    saved = 0
    got_any = False
    try:
        for data in frames_from(proc):
            arr = np.frombuffer(data, np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                continue
            got_any = True
            note = draw_detections(img, model)
            cv2.rectangle(img, (0, 0), (img.shape[1], 22), (0, 0, 0), -1)
            cv2.putText(img, note, (6, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(window, img)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                args.capture_dir.mkdir(parents=True, exist_ok=True)
                path = args.capture_dir / f"local_{time.strftime('%Y%m%d_%H%M%S')}.png"
                cv2.imwrite(str(path), img)
                saved += 1
                print(f"캡처 저장: {path} (총 {saved}장)")
    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate()
        cv2.destroyAllWindows()

    if not got_any:
        print("프레임을 못 받았습니다.", file=sys.stderr)
        print(proc.stderr.read().decode(errors="replace")[-600:], file=sys.stderr)
        return 1
    print(f"\n종료 — {saved}장 캡처, 저장 위치: {args.capture_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
