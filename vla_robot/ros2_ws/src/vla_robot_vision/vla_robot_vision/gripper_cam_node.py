"""gripper_cam_node — 그리퍼 카메라의 **유일한 소유자**.

기존 코드에서는 perception_node(1280x720)와 gripper_cam_publisher_node(640x480)가 같은
/dev/gripper_cam 을 열어 "Device or resource busy" 가 났다. 여기서는 이 노드만 장치를
열고, VLA 노드는 토픽으로만 받는다.

- 캡처 스레드가 쉬지 않고 최신 프레임만 붙든다(버퍼에 낡은 프레임이 쌓이지 않게).
- 발행은 publish_hz 로, 인코딩은 bgr8, stamp 는 **캡처 시각**이다.
- 180도 회전은 여기서 한 번만 한다. 녹화(LeRobot rotation=180)와 같은 방향이어야 한다.
- cv_bridge 를 쓰지 않는다 — 기존 환경에서 numpy 2.x 와 세그폴트가 났다.
- 읽기가 reopen_after_s 동안 계속 실패하면 장치를 다시 연다(USB 끊김 복구).
"""
from __future__ import annotations

import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from vla_common.config import load_robot_config


class GripperCamNode(Node):
    def __init__(self) -> None:
        super().__init__("gripper_cam_node")
        self.declare_parameter("config_file", "")
        self.cfg = load_robot_config(str(self.get_parameter("config_file").value)).gripper_cam

        self._cap: Optional[cv2.VideoCapture] = None
        self._latest: Optional[tuple[float, np.ndarray]] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._frames = 0
        self._published_stamp = 0.0

        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(Image, self.cfg.topic, qos)
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        self.create_timer(1.0 / max(self.cfg.publish_hz, 1.0), self._publish)
        self.create_timer(10.0, self._report)

    # -- 장치 ---------------------------------------------------------------
    def _open(self) -> bool:
        cap = cv2.VideoCapture(self.cfg.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return False
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.cfg.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg.height)
        cap.set(cv2.CAP_PROP_FPS, self.cfg.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_s = "".join(chr((fourcc >> 8 * i) & 0xFF) for i in range(4))
        if (w, h) != (self.cfg.width, self.cfg.height):
            self.get_logger().warn(f"요청 {self.cfg.width}x{self.cfg.height} 인데 장치가 {w}x{h} 를 줬다")
        self.get_logger().info(f"그리퍼캠 열림 {self.cfg.device} {w}x{h} {fourcc_s}")
        self._cap = cap
        return True

    def _capture_loop(self) -> None:
        last_ok = time.monotonic()
        while not self._stop.is_set():
            if self._cap is None:
                if not self._open():
                    self.get_logger().error(f"{self.cfg.device} 를 열 수 없다 — 1초 뒤 재시도",
                                            throttle_duration_sec=5.0)
                    time.sleep(1.0)
                    continue
                last_ok = time.monotonic()
            ok, frame = self._cap.read()
            now = time.monotonic()
            if not ok or frame is None:
                if now - last_ok > self.cfg.reopen_after_s:
                    self.get_logger().warn("그리퍼캠 읽기 실패가 계속된다 — 장치를 다시 연다")
                    self._cap.release()
                    self._cap = None
                time.sleep(0.01)
                continue
            last_ok = now
            if self.cfg.rotate_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            stamp = self.get_clock().now().nanoseconds * 1e-9
            with self._lock:
                self._latest = (stamp, frame)
                self._frames += 1

    # -- 발행 ---------------------------------------------------------------
    def _publish(self) -> None:
        with self._lock:
            item = self._latest
        if item is None or item[0] == self._published_stamp:
            return
        stamp, frame = item
        self._published_stamp = stamp
        # 발행 크기로 줄인다. 정책이 쓰는 크기라 받는 쪽에서 다시 줄일 일이 없고,
        # 메시지가 16배 작아져 토픽이 실제로 제 속도를 낸다(위 설정 주석의 2026-09-22 사례).
        # bilinear 로 줄인다 — 정책 경로(torch bilinear)와 같은 보간이다.
        if self.cfg.publish_width and self.cfg.publish_height:
            if (frame.shape[1], frame.shape[0]) != (self.cfg.publish_width, self.cfg.publish_height):
                frame = cv2.resize(frame, (self.cfg.publish_width, self.cfg.publish_height),
                                   interpolation=cv2.INTER_LINEAR)
        msg = Image()
        sec = int(stamp)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = int((stamp - sec) * 1e9)
        msg.header.frame_id = "gripper_cam"
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(frame).tobytes()
        self._pub.publish(msg)

    def _report(self) -> None:
        with self._lock:
            n, self._frames = self._frames, 0
        self.get_logger().info(f"캡처 {n / 10.0:.1f} fps")

    def destroy_node(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
