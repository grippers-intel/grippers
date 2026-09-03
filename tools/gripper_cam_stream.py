#!/usr/bin/env python3
"""그리퍼캠 실시간 스트리밍 — ROI 경계를 눈으로 보면서 맞추는 용도.

라즈베리파이는 헤드리스라 화면을 직접 못 띄우니, HTTP MJPEG 스트림을 띄워서
같은 네트워크의 Mac 브라우저로 실시간으로 본다. IntelPi 컨테이너는
`--network host`라 컨테이너 안에서 연 포트가 Pi 자체 IP로 바로 열린다.

쓰는 법(라즈베리파이 위, 컨테이너 안):
    python3 tools/gripper_cam_stream.py
    Mac 브라우저에서 http://<Pi IP>:8090/ 접속. Ctrl+C로 종료.

판정 로직과는 무관한 눈으로 보는 확인용 도구 — GRASP 판정에 안 쓴다."""
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2  # noqa: E402

from domain.adapters.real.gripper_cam_reader import DEVICE_DEFAULT, HEIGHT, WIDTH  # noqa: E402
from domain.task.grasp_cam_diff import GRASP_CAM_ROI  # noqa: E402

PORT = 8090
# 사용자 지정 규칙: OpenCV 오버레이에는 red 계열을 쓰지 않는다 — 초록 사용.
ROI_COLOR_BGR = (0, 200, 0)


def _open_camera() -> "cv2.VideoCapture | None":
    cap = cv2.VideoCapture(DEVICE_DEFAULT, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    return cap


def _draw_roi(frame) -> None:
    h, w = frame.shape[:2]
    x0, y0, x1, y1 = GRASP_CAM_ROI
    pt1 = (int(x0 * w), int(y0 * h))
    pt2 = (int(x1 * w), int(y1 * h))
    cv2.rectangle(frame, pt1, pt2, ROI_COLOR_BGR, 2)
    cv2.putText(
        frame, "GRASP_CAM_ROI", (pt1[0], max(pt1[1] - 8, 12)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, ROI_COLOR_BGR, 1, cv2.LINE_AA,
    )


class _StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args) -> None:  # noqa: A003
        return  # 요청 로그는 콘솔에 안 찍는다 — 매 프레임마다 시끄럽다.

    def do_GET(self) -> None:  # noqa: N802
        if self.path != "/":
            self.send_response(404)
            self.end_headers()
            return

        cap = _open_camera()
        if cap is None:
            self.send_response(500)
            self.end_headers()
            self.wfile.write(f"카메라를 열 수 없습니다: {DEVICE_DEFAULT}".encode())
            return

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        try:
            while True:
                ok, frame = cap.read()
                if not ok or frame is None:
                    time.sleep(0.1)
                    continue
                _draw_roi(frame)
                ok, jpg = cv2.imencode(".jpg", frame)
                if not ok:
                    continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                self.wfile.write(jpg.tobytes())
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            cap.release()


def main() -> int:
    server = ThreadingHTTPServer(("0.0.0.0", PORT), _StreamHandler)
    print(f"스트리밍 시작 — 브라우저에서 http://<Pi IP>:{PORT}/ 접속. Ctrl+C로 종료.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
