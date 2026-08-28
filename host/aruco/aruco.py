"""인쇄할 ArUco 마커 그림 5장을 만든다.

    python aruco.py

한 번만 돌리면 되고, 이미 만들어진 PNG 가 있으면 다시 돌릴 필요 없다.
실제 인쇄 크기는 이 파일이 정하지 않는다 — 인쇄 대화상자에서 정해지므로,
뽑은 뒤 자로 재서 config.py 에 그 값을 적으면 된다. (사용법.txt 1단계)
"""

import cv2
import cv2.aruco as aruco

import config as cfg

# 인쇄 품질용 해상도. 300 dpi 로 뽑으면 약 12 cm 가 된다.
SIZE_PX = 1417

MARKER_NAMES = {
    cfg.ROBOT_MARKER_ID: "robot",   # 로봇 상판
    1: "floor_1",                   # 바닥 앞쪽 왼편
    2: "floor_2",                   # 바닥 앞쪽 오른편
    3: "floor_3",                   # 바닥 뒤쪽 왼편
    4: "floor_4",                   # 바닥 뒤쪽 오른편
}


def main() -> None:
    print(f"OpenCV {cv2.__version__}")
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, cfg.ARUCO_DICT))
    for marker_id, name in MARKER_NAMES.items():
        img = aruco.generateImageMarker(dictionary, marker_id, SIZE_PX)
        path = f"marker_{marker_id}_{name}.png"
        cv2.imwrite(path, img)
        print(f"생성됨: {path} (ID {marker_id})")


if __name__ == "__main__":
    main()
