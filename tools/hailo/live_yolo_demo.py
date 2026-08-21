#!/usr/bin/env python3
"""Live Hailo-10H YOLO detection overlay demo (실기 검증용, 2026-08-21).

depth_cam(회전 보정된 컬러 스트림, depth_cam_rotate_node 참고)과 gripper_cam
(gripper_cam_publisher_node 참고) 양쪽을 구독해, Hailo-10H에 로드한 HEF
하나로 두 소스 모두 추론하고, 검출 박스를 그려 각자의 새 토픽으로
퍼블리시한다.

⚠️ 물리 Hailo-10H 장치는 1개뿐이다. VDevice는 프로세스당 장치를 독점하므로
(실기 확인 2026-08-21: 두 번째 프로세스가 VDevice를 열려다
HAILO_OUT_OF_PHYSICAL_DEVICES로 죽음), 카메라마다 별도 프로세스를 띄우면
안 되고 **이 프로세스 하나가 VDevice/ConfiguredInferModel을 한 번만 만들어
모든 카메라 소스가 공유**해야 한다.

⚠️ 이건 프로덕션 Perception 어댑터가 아니라 hld.md에 "별도 검증 항목으로
유지"라고 명시된 ④ HailoRT 추론 검증(hailo_status.md 참고)을 실제로 처음
해보는 탐색용 스크립트다. domain/ports/perception.py 계약에 편입하려면
별도 작업이 필요하다.

입력을 640x640으로 레터박스하기 때문에, 편의상 좌표 역변환 없이 **레터박스된
프레임 자체에 박스를 그려 퍼블리시**한다 — 원본 프레임 좌표계가 아니다.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from hailo_platform import FormatType, HailoSchedulingAlgorithm, VDevice
from rclpy.node import Node
from sensor_msgs.msg import Image

HEF_PATH_DEFAULT = "/tmp/best_640.hef"
# "input_topic=output_topic" 쌍을 콤마로 구분한다. 카메라 소스를 늘리려면
# 이 파라미터에 쌍을 더 추가하면 된다 -- 프로세스/VDevice는 그대로 하나.
CAMERA_TOPICS_DEFAULT = (
    "depth_cam/rgb/image_rotated=depth_cam/yolo/image_detections,"
    "gripper_cam/image_raw=gripper_cam/yolo/image_detections"
)
MODEL_INPUT_SIZE = 640
SCORE_THRESHOLD = 0.35
# metadata.yaml의 names — HEF 컴파일 당시 클래스 순서와 반드시 일치해야 한다.
CLASS_NAMES = ["container", "knight", "queen", "rook", "box", "soccer", "star"]
BOX_COLOR = (0, 255, 0)


def letterbox(frame, size=MODEL_INPUT_SIZE):
    h, w = frame.shape[:2]
    scale = min(size / h, size / w)
    resized = cv2.resize(frame, (round(w * scale), round(h * scale)))
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    y0 = (size - resized.shape[0]) // 2
    x0 = (size - resized.shape[1]) // 2
    canvas[y0 : y0 + resized.shape[0], x0 : x0 + resized.shape[1]] = resized
    return canvas


def draw_detections(canvas, detections_by_class):
    size = canvas.shape[0]
    for class_id, dets in enumerate(detections_by_class):
        label = CLASS_NAMES[class_id] if class_id < len(CLASS_NAMES) else str(class_id)
        for det in dets:
            y_min, x_min, y_max, x_max, score = det
            if score < SCORE_THRESHOLD:
                continue
            p1 = (int(x_min * size), int(y_min * size))
            p2 = (int(x_max * size), int(y_max * size))
            cv2.rectangle(canvas, p1, p2, BOX_COLOR, 2)
            cv2.putText(
                canvas,
                f"{label} {score:.2f}",
                (p1[0], max(0, p1[1] - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                BOX_COLOR,
                1,
                cv2.LINE_AA,
            )
    return canvas


class LiveYoloDemoNode(Node):
    def __init__(self):
        super().__init__("live_yolo_demo_node")
        self.declare_parameter("hef_path", HEF_PATH_DEFAULT)
        self.declare_parameter("camera_topics", CAMERA_TOPICS_DEFAULT)

        self._bridge = CvBridge()
        hef_path = self.get_parameter("hef_path").value
        camera_topics = self.get_parameter("camera_topics").value

        params = VDevice.create_params()
        params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        self._vdevice = VDevice(params)
        self._infer_model = self._vdevice.create_infer_model(hef_path)
        self._infer_model.input().set_format_type(FormatType.UINT8)
        self._configured_model = self._infer_model.configure()
        # NMS-by-class 출력은 자동 할당되지 않는다 -- run() 전에 raw flat
        # float32 버퍼를 직접 만들어 bindings에 물려야 한다 (실기 확인,
        # 2026-08-21: 안 하면 "not configured as view"로 run_async가 죽는다).
        # shape는 (max_bboxes_per_class*5 + 1) * num_classes = 3507 (실측).
        self._output_shape = self._infer_model.output().shape
        self.get_logger().info(
            f"Hailo-10H model loaded: {hef_path} (output shape={self._output_shape})"
        )

        for pair in camera_topics.split(","):
            input_topic, output_topic = pair.split("=")
            publisher = self.create_publisher(Image, output_topic, 10)
            # 클로저가 각자의 publisher/라벨을 잡도록 default arg로 바인딩한다.
            self.create_subscription(
                Image,
                input_topic,
                lambda msg, pub=publisher, label=input_topic: self._on_image(msg, pub, label),
                10,
            )
            self.get_logger().info(f"live_yolo_demo_node ready: {input_topic} -> {output_topic}")

    def _on_image(self, msg, publisher, source_label):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        canvas = letterbox(frame)

        bindings = self._configured_model.create_bindings()
        bindings.input().set_buffer(np.ascontiguousarray(canvas))
        bindings.output().set_buffer(np.empty(self._output_shape, dtype=np.float32))
        self._configured_model.run([bindings], timeout=1000)
        # get_buffer()가 raw flat 버퍼를 [cls0_dets, cls1_dets, ...] 형태로 바꿔 준다.
        detections_by_class = bindings.output().get_buffer()

        annotated = draw_detections(canvas, detections_by_class)
        out_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header = msg.header
        publisher.publish(out_msg)

        for class_id, dets in enumerate(detections_by_class):
            label = CLASS_NAMES[class_id] if class_id < len(CLASS_NAMES) else str(class_id)
            for det in dets:
                score = det[4]
                if score >= SCORE_THRESHOLD:
                    self.get_logger().info(f"[detect] {source_label}: {label} score={score:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = LiveYoloDemoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
