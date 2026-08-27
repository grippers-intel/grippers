#!/usr/bin/env python3
"""Pi의 카메라 영상을 **맥에서** 본다 (2026-08-26).

## 왜 rqt가 아닌가

컨테이너의 DISPLAY는 `:0.0` — Pi 자신에 붙은 모니터를 가리킨다. rqt를 띄우면
Pi 화면에 뜨지 노트북에는 안 뜬다. 노트북으로 당기려면 X11 포워딩이 필요한데
맥에는 XQuartz를 깔아야 하고, 영상 스트림을 X 프로토콜로 미는 것은 느리다.

## 컨테이너에 아무것도 안 남긴다

사용자 요구가 "Pi랑 컨테이너는 최대한 건들지 말고"였다. 그래서 이 도구는
**Pi에 파일을 하나도 쓰지 않는다.**

    맥 -> ssh -> docker exec -i -> python3 -   (프로그램을 stdin으로 흘려넣음)
    Pi -> stdout으로 base64 PNG -> 맥이 디코드해서 로컬에 저장

Pi에 설치하는 것도, 남기는 것도 없다. 세션이 끝나면 흔적이 사라진다.

## cv_bridge를 안 쓰는 이유

컨테이너의 기본 python3에 cv_bridge가 없다(ROS 환경에서만 잡힌다). 그런데
sensor_msgs/Image는 그냥 바이트 버퍼라 numpy로 직접 푸는 편이 의존성도 적고
인코딩별로 무슨 일이 벌어지는지도 드러난다.

## 사용

    python3 tools/mac_camera_view.py                한 장 떠서 열기 (회전 보정 RGB)
    python3 tools/mac_camera_view.py --topic depth  깊이 영상(컬러맵)
    python3 tools/mac_camera_view.py --topic raw    회전 전 원본 RGB
    python3 tools/mac_camera_view.py --watch        브라우저에서 실시간
    python3 tools/mac_camera_view.py --watch --seconds 120

⚠️ perception_node가 죽어 있어도 이 도구는 동작한다 — 카메라 드라이버와
depth_cam_rotate_node만 있으면 된다. 반대로 `--topic rotated`가 안 나오면
depth_cam_rotate_node가 내려간 것이다.
"""

import argparse
import base64
import pathlib
import shutil
import subprocess
import sys
import time
import webbrowser

PI = "pi@10.82.133.189"
CONTAINER = "IntelPi"

TOPICS = {
    "rotated": "/depth_cam/rgb/image_rotated",
    "raw": "/ascamera/camera_publisher/rgb0/image",
    "depth": "/ascamera/camera_publisher/depth0/image_raw",
}

MARKER = "---FRAME---"
OUT_DIR = pathlib.Path.home() / ".grippers_camview"

# Pi에서 도는 프로그램. 여기 있는 것이 전부이고, 파일로 저장되지 않는다.
GRABBER = r'''
import base64, sys, time
import numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPIC, FRAMES, MARKER, YOLO = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4] == "1"

# perception_node의 게이트와 **같은 값이어야 한다.** 이 도구의 요점은 YOLO가
# 무엇을 봤는지가 아니라, 그중 무엇이 파이프라인에 실제로 들어가는지다.
MODEL_PATH = "/grippers/models/best.pt"
CONF_GATE = 0.70
MIN_BOTTOM_Y = 290.0

model = None
if YOLO:
    from ultralytics import YOLO as _Y
    model = _Y(MODEL_PATH)
    print("MODEL " + MODEL_PATH, file=sys.stderr, flush=True)


def draw_detections(img):
    """검출을 그리고 게이트 통과 여부를 색으로 구분한다.

    통과(초록)와 탈락(빨강)을 나눠 그리는 이유는, 배경 오검출이 신뢰도만으로는
    안 걸러진다는 것이 실측으로 드러났기 때문이다 - 노트북을 rook으로 0.80에
    잡은 적이 있고 그때 막은 것은 화면위치 게이트뿐이었다. 그 선을 눈으로
    보게 해 두면 왜 걸렀는지가 그림에 나온다."""
    result = model(img, verbose=False, conf=0.25)[0]
    names = result.names
    passed = 0
    # 화면위치 게이트 선. 이 아래에 bbox 아래끝이 있어야 통과한다.
    cv2.line(img, (0, int(MIN_BOTTOM_Y)), (img.shape[1], int(MIN_BOTTOM_Y)),
             (90, 90, 90), 1, cv2.LINE_AA)
    cv2.putText(img, "y=%d gate" % MIN_BOTTOM_Y, (6, int(MIN_BOTTOM_Y) - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, (140, 140, 140), 1, cv2.LINE_AA)

    for box in result.boxes:
        x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
        conf = float(box.conf[0])
        label = names[int(box.cls[0])]
        conf_ok, y_ok = conf >= CONF_GATE, y2 >= MIN_BOTTOM_Y
        ok = conf_ok and y_ok
        passed += int(ok)
        color = (90, 220, 90) if ok else (70, 70, 235)
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        why = "" if ok else ("  conf<%.2f" % CONF_GATE if not conf_ok else "  too high")
        text = "%s %.2f%s" % (label, conf, why)
        ty = int(y1) - 6 if y1 > 18 else int(y2) + 14
        cv2.putText(img, text, (int(x1), ty), cv2.FONT_HERSHEY_SIMPLEX, 0.44,
                    color, 1, cv2.LINE_AA)
    return "%d/%d pass gates" % (passed, len(result.boxes))


def to_bgr(msg):
    """sensor_msgs/Image -> 화면에 띄울 수 있는 BGR 8비트.

    step으로 잘라내는 것이 중요하다 — 행 끝에 패딩이 붙는 경우가 있어서
    width만 믿고 reshape하면 영상이 비스듬히 밀린다."""
    enc = msg.encoding
    buf = np.frombuffer(msg.data, np.uint8)
    h, w, step = msg.height, msg.width, msg.step

    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(h, step)[:, : w * 3].reshape(h, w, 3)
        return img[:, :, ::-1].copy() if enc == "rgb8" else img.copy(), ""
    if enc == "mono8":
        img = buf.reshape(h, step)[:, :w]
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), ""

    # 깊이. 미터/밀리미터 float나 16비트라 그대로 띄우면 새까맣다 —
    # 유효 화소의 분위수로 범위를 잡아 컬러맵을 씌운다.
    if enc in ("16UC1", "mono16"):
        depth = buf.view(np.uint16).reshape(h, step // 2)[:, :w].astype(np.float32) / 1000.0
    elif enc in ("32FC1",):
        depth = buf.view(np.float32).reshape(h, step // 4)[:, :w]
    else:
        raise SystemExit("지원하지 않는 인코딩: " + enc)

    valid = np.isfinite(depth) & (depth > 0.05)
    if not valid.any():
        return np.zeros((h, w, 3), np.uint8), "no valid pixels"
    lo, hi = np.percentile(depth[valid], [5, 95])
    if hi - lo < 1e-6:
        hi = lo + 1e-6
    norm = np.clip((depth - lo) / (hi - lo), 0, 1)
    color = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    color[~valid] = (40, 40, 40)
    # ⚠️ 라벨은 ASCII여야 한다. cv2.putText의 Hershey 폰트에는 한글 글리프가
    # 없어서 물음표로 찍힌다(2026-08-26에 실제로 그렇게 나왔다).
    return color, "%.2f-%.2fm  valid %d%%" % (lo, hi, 100 * valid.mean())


class Grab(Node):
    def __init__(self):
        super().__init__("mac_camera_view")
        self.sent = 0
        self.create_subscription(Image, TOPIC, self.on_frame, qos_profile_sensor_data)

    def on_frame(self, msg):
        try:
            img, note = to_bgr(msg)
        except Exception as exc:
            print("ERR " + str(exc), file=sys.stderr, flush=True)
            return
        if model is not None:
            try:
                note = (note + "  " if note else "") + draw_detections(img)
            except Exception as exc:
                note = "YOLO 실패: " + str(exc)[:60]
        label = "%s  %dx%d  %s  %s" % (
            TOPIC.rsplit("/", 1)[-1], msg.width, msg.height, msg.encoding, note)
        cv2.rectangle(img, (0, 0), (img.shape[1], 22), (0, 0, 0), -1)
        cv2.putText(img, label, (6, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (255, 255, 255), 1, cv2.LINE_AA)
        ok, png = cv2.imencode(".png", img)
        if not ok:
            return
        print(MARKER, flush=True)
        print(base64.b64encode(png.tobytes()).decode(), flush=True)
        self.sent += 1


rclpy.init()
node = Grab()
deadline = time.time() + 20 + FRAMES * 0.5
while rclpy.ok() and node.sent < FRAMES and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
if node.sent == 0:
    print("NOFRAME " + TOPIC, file=sys.stderr, flush=True)
node.destroy_node()
rclpy.shutdown()
'''

VIEWER_HTML = """<!doctype html><meta charset=utf-8><title>grippers camera</title>
<style>
 body{{margin:0;background:#111;color:#ddd;font:13px ui-monospace,monospace;
      display:flex;flex-direction:column;align-items:center;gap:10px;padding:14px}}
 img{{max-width:96vw;border-radius:6px;box-shadow:0 2px 18px #0008}}
 #s{{opacity:.65}}
</style>
<h3 style="margin:4px">{topic}</h3>
<img id=v src="frame.png">
<div id=s>연결 중…</div>
<script>
 let n = 0, last = 0;
 setInterval(() => {{
   const img = document.getElementById('v');
   const probe = new Image();
   probe.onload = () => {{ img.src = probe.src; n++;
     document.getElementById('s').textContent = n + ' 프레임 · ' + new Date().toLocaleTimeString(); }};
   probe.src = 'frame.png?t=' + Date.now();
 }}, 350);
</script>
"""


def run_grabber(topic, frames, yolo=False):
    """Pi에서 grabber를 돌리고 stdout을 그대로 넘겨준다."""
    inner = (
        "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
        "source /ros2_ws/install/setup.bash >/dev/null 2>&1; "
        "export ROS_DOMAIN_ID=21; "
        f"python3 - {topic} {frames} {MARKER} {'1' if yolo else '0'}"
    )
    return subprocess.Popen(
        ["ssh", "-o", "BatchMode=yes", PI,
         f"docker exec -i -u ubuntu {CONTAINER} bash -lc {shell_quote(inner)}"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def shell_quote(text):
    return "'" + text.replace("'", "'\\''") + "'"


def feed(process):
    """grabber 프로그램을 stdin으로 흘려 넣는다.

    `python3 -`는 stdin에서 프로그램을 읽으므로, 다 보내고 나서 반드시
    닫아야 실행이 시작된다."""
    process.stdin.write(GRABBER)
    process.stdin.close()
    return process


def frames_from(process):
    """stdout에서 base64 프레임만 골라낸다.

    ROS가 stdout에 무엇을 찍을지 알 수 없으므로 마커 다음 줄만 신뢰한다."""
    expect = False
    for line in process.stdout:
        line = line.strip()
        if line == MARKER:
            expect = True
            continue
        if expect:
            expect = False
            try:
                yield base64.b64decode(line)
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--topic", default="rotated", choices=sorted(TOPICS))
    parser.add_argument("--watch", action="store_true", help="브라우저에서 실시간")
    parser.add_argument("--yolo", action="store_true",
                        help="배포된 가중치로 검출을 그리고 게이트 통과 여부를 표시")
    parser.add_argument("--seconds", type=int, default=60, help="--watch 지속 시간")
    args = parser.parse_args()

    topic = TOPICS[args.topic]
    OUT_DIR.mkdir(exist_ok=True)

    if not args.watch:
        process = feed(run_grabber(topic, 1, args.yolo))
        for data in frames_from(process):
            path = OUT_DIR / f"{args.topic}.png"
            path.write_bytes(data)
            process.terminate()
            print(f"저장: {path}")
            subprocess.run(["open", str(path)])
            return 0
        print("프레임을 못 받았습니다.", file=sys.stderr)
        print(process.stderr.read().strip()[-600:], file=sys.stderr)
        print(f"\n  토픽이 살아 있는지 보세요: {topic}", file=sys.stderr)
        if args.topic == "rotated":
            print("  rotated가 비면 depth_cam_rotate_node가 내려간 것입니다.",
                  file=sys.stderr)
        return 1

    # --watch: 노드를 한 번만 띄우고 프레임을 계속 흘려받는다. 매번 ssh를
    # 새로 여는 것보다 훨씬 빠르다 — 노드 기동에만 1~2초가 든다.
    frame_path = OUT_DIR / "frame.png"
    (OUT_DIR / "index.html").write_text(VIEWER_HTML.format(topic=topic), encoding="utf-8")
    process = feed(run_grabber(topic, args.seconds * 10, args.yolo))

    count = 0
    started = time.time()
    try:
        for data in frames_from(process):
            tmp = frame_path.with_suffix(".part")
            tmp.write_bytes(data)
            # 브라우저가 반쯤 쓰인 파일을 읽지 않도록 원자적으로 바꿔 끼운다.
            shutil.move(tmp, frame_path)
            count += 1
            if count == 1:
                webbrowser.open((OUT_DIR / "index.html").as_uri())
                print(f"브라우저를 엽니다. Ctrl-C로 종료.  ({topic})")
            if count % 10 == 0:
                rate = count / max(time.time() - started, 1e-6)
                print(f"\r  {count} 프레임  {rate:.1f} fps", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        process.terminate()
    print()
    if count == 0:
        print("프레임을 못 받았습니다.", file=sys.stderr)
        print(process.stderr.read().strip()[-600:], file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
