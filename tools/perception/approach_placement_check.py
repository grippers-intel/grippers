#!/usr/bin/env python3
"""APPROACH 목표 위치 캘리브레이션 콘솔 — 6개 클래스 물체를 차체 앞 목표
위치에 하나씩 놓고, 파지 기준 거리·좌우 오프셋을 실시간으로 확인한다.

perception_node.py의 observe_target 서비스(단일 프레임 bbox 관측)를 ~5Hz로
반복 호출해 갱신한다. 거리·좌우 오프셋 공식은 perception_node.py의
_approach_pose_m/_on_rgb_camera_info와 완전히 동일하게 맞춘다 — 실제
APPROACH 단계가 쓰는 것과 같은 기준값이라는 뜻이다(값을 여기 따로
하드코딩하지 않고 K_class는 perception_node에서 직접 import한다):

    z_m = K_class / sqrt(h*w)                거리(전방, m) — base_link 기준
                                              (K_class 자체가 줄자 실측으로
                                              그 기준에 맞춰져 있다)
    y_obj_m = -(x - cx) * z_m / fx            좌우 오프셋(m, +좌/-우)
    cx = camera_info.width - camera_info.k[2] (180도 회전 보정,
                                              perception_node.py와 동일)

조작 (실행 중 터미널에 입력):
  클래스 이름(rook/knight/queen/soccer/box/star) + Enter → 관측 대상 전환
  q + Enter                                              → 종료

기본 클래스: rook. box/star는 아직 K_class 미실측(perception_node.py의
CLASS_DISTANCE_CALIBRATION_SQRT_PX_M이 None)이라 거리·오프셋은 계산하지
않고 raw x/h/w만 보여준다.

사전 준비: depth_camera.launch.py + depth_cam_rotate_node + perception_node
가 떠 있어야 한다(HANDOFF.md §4-1 표준 기동 순서).
"""
import math
import sys
import threading
import time

CLASSES = ["rook", "knight", "queen", "soccer", "box", "star"]
DEFAULT_CLASS = "rook"
POLL_HZ = 5.0
CAMERA_INFO_TOPIC = "/ascamera/camera_publisher/rgb0/camera_info"
SERVICE_NAME = "perception/observe_target"
SERVICE_WAIT_SEC = 1.0


def estimate_pose(k_class, h_px, w_px, u_px, fx, cx):
    """perception_node.py와 동일한 공식. 순수 함수 — rclpy 없이 단위 테스트한다.

    반환: (z_m, y_obj_m). 계산 불가(K_class 미실측/카메라 정보 없음/면적 0)
    상황이면 (None, None)."""
    if k_class is None or fx is None or cx is None:
        return None, None
    area = h_px * w_px
    if area <= 0:
        return None, None
    z_m = k_class / math.sqrt(area)
    y_obj_m = -(u_px - cx) * z_m / fx
    return z_m, y_obj_m


def _class_k_values():
    # rclpy 의존 모듈은 여기서만 import한다 — estimate_pose()/_format_line()은
    # 이 함수 없이도 단위 테스트할 수 있게 유지한다.
    from grippers_perception.perception_node import CLASS_DISTANCE_CALIBRATION_SQRT_PX_M

    return CLASS_DISTANCE_CALIBRATION_SQRT_PX_M


def _make_node():
    # rclpy/센서 메시지/서비스 타입도 여기서만 import한다 — 같은 이유
    # (align_to_idle.py의 _connect()와 동일 규칙).
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo

    from grippers_interfaces.srv import ObserveTarget

    class PlacementCheckNode(Node):
        def __init__(self):
            super().__init__("approach_placement_check")
            self.fx = None
            self.cx = None
            self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._on_camera_info, 10)
            self.client = self.create_client(ObserveTarget, SERVICE_NAME)

        def _on_camera_info(self, msg):
            self.fx = msg.k[0]
            self.cx = msg.width - msg.k[2]  # 180도 회전 보정 — perception_node.py와 동일

        def observe(self, raw_cls, timeout_sec=SERVICE_WAIT_SEC):
            if not self.client.wait_for_service(timeout_sec=timeout_sec):
                return None
            request = ObserveTarget.Request()
            request.raw_cls = raw_cls
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            return future.result()

    return PlacementCheckNode()


def run(initial_cls=DEFAULT_CLASS):
    import rclpy

    k_values = _class_k_values()
    rclpy.init()
    node = _make_node()

    state = {"cls": initial_cls}
    stop = threading.Event()

    def input_loop():
        print(f"[placement] 클래스 입력 후 Enter로 전환 ({'/'.join(CLASSES)}), q로 종료")
        while not stop.is_set():
            try:
                text = input().strip().lower()
            except EOFError:
                stop.set()
                return
            if text in ("q", "quit", "exit"):
                stop.set()
                return
            if text in CLASSES:
                state["cls"] = text
                print(f"\n[placement] → {text}로 전환")
            elif text:
                print(f"\n[placement] 모르는 클래스: {text!r} (가능: {', '.join(CLASSES)})")

    input_thread = threading.Thread(target=input_loop, daemon=True)
    input_thread.start()

    print(f"[placement] 시작 클래스: {state['cls']}")
    period = 1.0 / POLL_HZ
    try:
        while not stop.is_set():
            t0 = time.monotonic()
            cls = state["cls"]
            result = node.observe(cls)
            line = _format_line(cls, result, k_values.get(cls), node.fx, node.cx)
            print(f"\r{line}", end="", flush=True)
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, period - elapsed))
    except KeyboardInterrupt:
        pass
    finally:
        print()
        stop.set()
        node.destroy_node()
        rclpy.shutdown()


def _format_line(cls, result, k_class, fx, cx):
    pad = " " * 10
    if result is None:
        return f"[{cls}] observe_target 서비스 응답 없음{pad}"
    if not result.found:
        return f"[{cls}] 미검출{pad}"
    z_m, y_obj_m = estimate_pose(k_class, result.h, result.w, result.x, fx, cx)
    if z_m is None:
        return (
            f"[{cls}] x={result.x:.0f} h={result.h:.0f} w={result.w:.0f} "
            f"(거리 보정값 미실측 — raw만 표시){pad}"
        )
    return (
        f"[{cls}] 거리={z_m * 100:5.1f}cm  좌우오프셋={y_obj_m * 100:+5.1f}cm  "
        f"(x={result.x:.0f} h={result.h:.0f} w={result.w:.0f}){pad}"
    )


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    initial_cls = argv[0] if argv else DEFAULT_CLASS
    if initial_cls not in CLASSES:
        print(f"알 수 없는 클래스: {initial_cls!r} (가능: {', '.join(CLASSES)})", file=sys.stderr)
        return 1
    run(initial_cls)
    return 0


if __name__ == "__main__":
    sys.exit(main())
