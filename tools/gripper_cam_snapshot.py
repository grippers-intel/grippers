#!/usr/bin/env python3
"""그리퍼캠 화면을 한 장 찍어서 GRASP_CAM_ROI 경계를 그려 저장한다.

라즈베리파이는 헤드리스라 실시간 화면을 그대로 보여줄 수 없어서, 스냅샷을
찍어 파일로 남기고 Mac으로 옮겨서 확인하는 용도다. 판정 로직과는 무관한
눈으로 보는 확인용 도구.

쓰는 법:
    python3 tools/gripper_cam_snapshot.py [출력경로]
    출력경로 생략 시 /tmp/gripper_cam_snapshot.jpg 에 저장.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2  # noqa: E402

from domain.adapters.real.gripper_cam_reader import DEVICE_DEFAULT, HEIGHT, WIDTH  # noqa: E402
from domain.task.grasp_cam_diff import GRASP_CAM_ROI  # noqa: E402

# 사용자 지정 규칙: OpenCV 오버레이에는 red 계열을 쓰지 않는다 — 초록 사용.
ROI_COLOR_BGR = (0, 200, 0)


def main() -> int:
    out_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/gripper_cam_snapshot.jpg"

    cap = cv2.VideoCapture(DEVICE_DEFAULT, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"카메라를 열 수 없습니다: {DEVICE_DEFAULT}")
        return 1
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    for _ in range(5):
        cap.grab()
    ok, frame = cap.read()
    cap.release()

    if not ok or frame is None:
        print("프레임 캡처 실패")
        return 1

    h, w = frame.shape[:2]
    x0, y0, x1, y1 = GRASP_CAM_ROI
    pt1 = (int(x0 * w), int(y0 * h))
    pt2 = (int(x1 * w), int(y1 * h))
    cv2.rectangle(frame, pt1, pt2, ROI_COLOR_BGR, 2)
    cv2.putText(
        frame, "GRASP_CAM_ROI", (pt1[0], max(pt1[1] - 8, 12)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, ROI_COLOR_BGR, 1, cv2.LINE_AA,
    )

    cv2.imwrite(out_path, frame)
    print(f"저장 완료: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
