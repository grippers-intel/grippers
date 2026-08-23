"""depth_cam_rotate_node — HP60C 컬러 스트림을 180도 회전해 다시 퍼블리시한다.

⚠️ 실기 확인(2026-08-21): HP60C가 차체에 정상 방향으로 장착돼 있는데도
`ascamera_node`(third_party_ros2 벤더 SDK)가 내보내는 이미지가 상하좌우로
뒤집혀 있다. 벤더 SDK에는 플립/회전 파라미터가 없어(source에 image
orientation 옵션 없음, TF 트리 회전 quaternion뿐) 여기서 픽셀 단위로
보정한다.

벤더 코드(third_party_ros2/third_party_ws)는 건드리지 않는다 — 이 노드는
독립된 구독자/퍼블리셔로 붙는다.
"""

import rclpy
from rclpy.node import Node

try:
    import cv2
    import numpy as np
    from sensor_msgs.msg import Image

    _CV_AVAILABLE = True
except ImportError:
    _CV_AVAILABLE = False

INPUT_TOPIC_DEFAULT = "/ascamera/camera_publisher/rgb0/image"
# ⚠️ 2026-08-23: 실기 확인(ros2 topic list, depth_camera.launch.py 기동 후) —
# 이전 값(/ascamera_hp60c/...)은 존재하지 않는 토픽이라 _on_image가 한 번도
# 안 불렸다. peripherals/launch/include/hp60c.launch.py가 쓰는 "ascamera_hp60c"
# 네임스페이스는 이 launch 경로에서 실제로 안 쓰인다 — depth_camera.launch.py가
# 실제로 띄우는 노드는 "ascamera" 네임스페이스로 퍼블리시한다.
OUTPUT_TOPIC_DEFAULT = "depth_cam/rgb/image_rotated"


def _bgr_from_image_msg(msg):
    """Image 메시지를 BGR cv2 배열로 바꾼다 — cv_bridge를 쓰지 않는다
    (perception_node.py의 같은 이름 함수·같은 이유 참고. tools/perception/
    floor_observer.py의 to_bgr()과 같은 방식)."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("bgr8", "rgb8"):
        img = buf.reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1] if enc == "rgb8" else img
    if enc == "mono8":
        return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    raise ValueError(f"지원하지 않는 인코딩: {msg.encoding}")


def _image_msg_from_bgr(img, header):
    """BGR cv2 배열을 Image 메시지로 바꾼다 — cv_bridge를 쓰지 않는다.

    2026-08-23 실기 확인: cv_bridge의 컴파일된 확장(cv_bridge_boost.so)이
    numpy 1.x ABI로 빌드돼 있어 이 환경의 numpy 2.x와 안 맞는다
    (`AttributeError: _ARRAY_API not found` → 세그폴트). BGR uint8 배열을
    Image 메시지로 직접 채우는 건 표준 절차라 cv_bridge 없이도 안전하다."""
    msg = Image()
    msg.header = header
    msg.height, msg.width = img.shape[0], img.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(img).tobytes()
    return msg


class DepthCamRotateNode(Node):
    def __init__(self):
        super().__init__("depth_cam_rotate_node")
        self.declare_parameter("input_topic", INPUT_TOPIC_DEFAULT)
        self.declare_parameter("output_topic", OUTPUT_TOPIC_DEFAULT)

        if not _CV_AVAILABLE:
            self.get_logger().warn("opencv/numpy 미설치 — depth_cam_rotate_node 비활성화")
            return

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._publisher = self.create_publisher(Image, output_topic, 10)
        self.create_subscription(Image, input_topic, self._on_image, 10)
        self.get_logger().info(
            f"depth_cam_rotate_node ready: {input_topic} -> {output_topic} (180°)"
        )

    def _on_image(self, msg):
        frame = _bgr_from_image_msg(msg)
        rotated = cv2.rotate(frame, cv2.ROTATE_180)
        out_msg = _image_msg_from_bgr(rotated, msg.header)
        self._publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthCamRotateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
