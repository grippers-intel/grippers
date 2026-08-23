"""카메라 모드를 하나씩 열어보고 한 장씩 저장한다 — 어느 조합이 제대로 된 RGB인지 찾기."""
import os, cv2
OUT = "/grippers/recordings/_probe"
os.makedirs(OUT, exist_ok=True)
COMBOS = [
    ("/dev/video0", "MJPG", 1280, 720), ("/dev/video0", "MJPG", 640, 642),
    ("/dev/video0", "MJPG", 320, 564),  ("/dev/video0", "MJPG", 160, 768),
    ("/dev/video0", "YUYV", 1280, 1040),
    ("/dev/video1", "MJPG", 640, 480),
    ("/dev/video2", "MJPG", 1280, 720), ("/dev/video2", "MJPG", 640, 480),
]
for dev, fcc, w, h in COMBOS:
    tag = f"{os.path.basename(dev)}_{fcc}_{w}x{h}"
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"{tag:28} 열기 실패"); continue
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    ok = False
    for _ in range(12):                      # 몇 프레임 버리고 안정된 것으로
        ok, frame = cap.read()
    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.release()
    if ok and frame is not None:
        cv2.imwrite(f"{OUT}/{tag}.jpg", frame)
        print(f"{tag:28} OK  실제 {aw}x{ah}  shape={frame.shape}")
    else:
        print(f"{tag:28} 프레임 없음  (실제 {aw}x{ah})")
