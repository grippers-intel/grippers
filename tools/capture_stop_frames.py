#!/usr/bin/env python3
"""주행 정지 정밀도 측정용 — 정지할 때마다 그리퍼캠 프레임과 팔 자세를 남긴다.

Pi 에서 돌린다. 차량이 목표 앞에 정지하면 **Enter 를 누르면** 그 순간의
프레임 한 장과 관절값이 저장된다. 나중에 자 눈금을 읽어 좌우 오차를 낸다.

## 왜 눈으로 안 읽고 저장하는가

화면을 눈으로 읽으면 회차마다 사람의 기준이 조금씩 달라지고, 나중에
"그때 실제로 어땠나"를 다시 볼 수가 없다. 프레임을 남기면 눈금 검출로
일관되게 재고, 실패했을 때 그 프레임을 다시 볼 수 있다.

## 카메라를 직접 열지 않는다

`/dev/gripper_cam` 은 perception_node 가 붙들고 있다. 여기서 또 열면
`Device or resource busy` 다. perception_node 의 `gripper_cam_publish_hz` 가
0 보다 커야 이 도구가 프레임을 받는다.

## ⚠️ 팔을 움직이지 말 것

정지 직후 팔이 움직이면 그리퍼캠이 같이 움직여 측정이 무의미해진다.
파지를 시작하기 **전에** 누를 것.

사용법
    python3 tools/capture_stop_frames.py --out /grippers/runs/base_precision
    python3 tools/capture_stop_frames.py --out ... --label A   # 시작점 이름
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from grippers_interfaces.srv import GetArmState
except Exception:  # 인터페이스가 아직 안 빌드된 환경에서도 프레임은 남긴다
    GetArmState = None


def bgr_from_image_msg(msg):
    if msg.encoding != "bgr8":
        return None
    return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)


class Capture(Node):
    def __init__(self, topic):
        super().__init__("capture_stop_frames")
        self.frame = None
        self.stamp = 0.0
        self.create_subscription(Image, topic, self._on_frame, 1)
        self.state_client = (self.create_client(GetArmState, "arm_driver/get_arm_state")
                             if GetArmState is not None else None)

    def _on_frame(self, msg):
        frame = bgr_from_image_msg(msg)
        if frame is not None:
            self.frame = frame
            self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def arm_state(self):
        """관절값. 못 읽으면 None — 프레임 저장을 막지는 않는다."""
        if self.state_client is None or not self.state_client.wait_for_service(timeout_sec=1.5):
            return None
        future = self.state_client.call_async(GetArmState.Request())
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return None
        r = future.result()
        if r is None:
            return None
        # ⚠️ ROS 메시지의 고정 길이 배열은 numpy 배열이고, list() 로 감싸도
        # **원소가 numpy 스칼라로 남는다.** 그대로 두면 json.dumps 가
        # `Object of type int32 is not JSON serializable` 로 죽는다.
        # (같은 함정이 pose_verify_cycle.ArmSnapshot 주석에 2026-08-25 기록으로
        #  남아 있다. 읽고도 밟았다 — 원소까지 파이썬 기본형으로 바꿀 것.)
        return {
            "ok": bool(r.ok),
            "online": [bool(v) for v in r.online],
            "position_raw": [int(v) for v in r.position_raw],
            "policy_state": ([float(v) for v in r.policy_state]
                             if getattr(r, "policy_state_valid", False) else None),
        }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/grippers/runs/base_precision")
    ap.add_argument("--topic", default="gripper_cam/image_raw")
    ap.add_argument("--label", default="", help="시작점 이름(A/B/C...). 파일명에 들어간다")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Capture(args.topic)

    print(f"저장 위치: {out}")
    print(f"토픽: {args.topic}")
    print("프레임을 기다리는 중 ...", flush=True)
    deadline = time.monotonic() + 15.0
    while node.frame is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.frame is None:
        print("프레임이 안 옵니다 — perception_node 의 gripper_cam_publish_hz 가 0 이 아닌지 보십시오")
        return 1
    print(f"프레임 수신 {node.frame.shape}. 정지할 때마다 Enter, 끝내려면 q + Enter\n")

    import cv2
    n = 0
    while rclpy.ok():
        # 최신 프레임으로 갱신해 두고 입력을 기다린다.
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        try:
            key = input(f"[{n + 1:02d}] 정지했으면 Enter (라벨 {args.label or '-'}) > ").strip()
        except EOFError:
            break
        if key.lower() == "q":
            break
        # 누른 **뒤에** 한 번 더 받아서 가장 신선한 프레임을 쓴다.
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        frame = node.frame
        if frame is None:
            print("  프레임 없음 — 건너뜁니다")
            continue
        n += 1
        tag = f"{args.label}_{n:02d}" if args.label else f"{n:02d}"
        png = out / f"stop_{tag}.png"
        cv2.imwrite(str(png), frame)
        record = {
            "n": n, "label": args.label, "file": png.name,
            "wall": time.strftime("%Y-%m-%d %H:%M:%S"),
            "frame_stamp": node.stamp,
            "arm": node.arm_state(),
        }
        # ⚠️ 로그 실패로 촬영을 끊지 않는다. 측정 중에 스크립트가 죽으면 그
        # 회차를 다시 찍어야 하고, 차량을 다시 세우는 비용이 로그 한 줄보다
        # 훨씬 크다. 프레임은 이미 저장돼 있으므로 로그가 없어도 잴 수 있다.
        try:
            with open(out / "log.jsonl", "a", encoding="utf-8") as f:
                f.write(json.dumps(record, ensure_ascii=False) + "\n")
        except Exception as e:  # noqa: BLE001 — 촬영을 계속하는 것이 우선
            print(f"  (로그 기록 실패, 프레임은 저장됨: {type(e).__name__}: {e})")
        arm = record["arm"]
        pan = (arm or {}).get("policy_state")
        print(f"  저장 {png.name}" + (f"   pan {pan[0]:+.2f}도" if pan else "   (관절값 없음)"))

    print(f"\n{n}장 저장했습니다: {out}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
