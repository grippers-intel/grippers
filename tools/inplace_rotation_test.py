#!/usr/bin/env python3
"""제자리 회전 최소 속도 테스트 콘솔.

`visual_approach_control.py`(APPROACH의 좌우 정렬)가 실제로 쓰는 회전 게인은
MIN_TURN=0.15, MAX_TURN=0.5 rad/s다. 그런데 align_to_idle 데몬 작업 중
실측한 바로는 제자리 회전이 정지마찰을 이기려면 최소 ~0.355 rad/s, 안정적
으로 돌려면 1.0~1.2 rad/s가 필요했다(HANDOFF.md §3-4). MIN_TURN=0.15는 그
최소치보다도 한참 낮다 — 즉 오차가 작을 때 APPROACH가 내는 회전 명령이
로봇을 아예 못 돌릴 가능성이 있다. 이 도구로 실제 로봇을 보면서 어느
각속도부터 눈에 보이게 도는지 직접 찾는다.

**`/odom_raw`는 이 판정에 못 쓴다.** 명령으로 받은 linear/angular 값을
그대로 반영할 뿐 실제로 바퀴가 움직였는지는 검증하지 않는다(HANDOFF.md
§3-4 (3)) — 그래서 이 도구는 오도메트리를 아예 보지 않고, **사람이 직접
보고 y/n으로 판정**하는 걸 유일한 신호로 삼는다.

base_driver_node.py/visual_approach_control.py가 실제로 쓰는 경로를 그대로
재현하려고 기본 토픽은 (odom_publisher_node.py가 ±0.5rad/s로 자르는) 평범한
`cmd_vel`이다 — 즉 이 도구가 보여주는 결과가 APPROACH 실기 동작의 실제
예상치다. 클램프 없는 `controller/cmd_vel`과 비교하고 싶으면 --topic으로
바꿀 것.

사전 준비: odom_publisher_node.py가 떠 있어야 한다(HANDOFF.md §4-1 표준
기동 순서).

조작:
  각속도(rad/s, +/-) 입력 후 Enter → burst_sec 동안 그 속도로 회전 명령을
    내고 정지
  y 또는 n + Enter → 실제로 돌았는지 판정, JSON Lines로 기록
  q + Enter → 종료, 마지막에 돈 속도/안 돈 속도 요약 출력
"""
import argparse
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

DEFAULT_TOPIC = "cmd_vel"
DEFAULT_BURST_SEC = 1.0
TICK_SEC = 0.05

# 참고용 숫자 — 판정은 사람이 한다, 이 표는 감을 잡는 용도일 뿐이다.
REFERENCE_VALUES_RAD_S = {
    "visual_approach_control.py MIN_TURN": 0.15,
    "visual_approach_control.py MAX_TURN": 0.5,
    "정지마찰 추정 최소(2026-08-24 실측)": 0.355,
    "align_to_idle 데몬 안정 회전(2026-08-24 반영)": 1.0,
}


class RotationTestNode(Node):
    def __init__(self, topic):
        super().__init__("inplace_rotation_test")
        self.pub = self.create_publisher(Twist, topic, 10)

    def spin_at(self, angular_z, burst_sec):
        twist = Twist()
        twist.angular.z = angular_z
        elapsed = 0.0
        while elapsed < burst_sec:
            self.pub.publish(twist)
            time.sleep(TICK_SEC)
            elapsed += TICK_SEC
        self.pub.publish(Twist())  # 정지


def run(topic, burst_sec, log_path):
    rclpy.init()
    node = RotationTestNode(topic)

    print(f"[rotate-test] 토픽={topic}  burst={burst_sec}s")
    print("[rotate-test] 참고값(판정은 사람이 함):")
    for label, value in REFERENCE_VALUES_RAD_S.items():
        print(f"    {label}: {value} rad/s")
    print("[rotate-test] 각속도(rad/s, +/-) 입력 후 Enter · q로 종료")

    results = []
    try:
        while True:
            try:
                text = input("rad/s> ").strip()
            except EOFError:
                break
            if text.lower() in ("q", "quit", "exit"):
                break
            try:
                angular_z = float(text)
            except ValueError:
                print(f"  숫자로 해석 안 됨: {text!r}")
                continue

            print(f"  {angular_z:+.3f} rad/s로 {burst_sec}s 회전 명령 발행...")
            node.spin_at(angular_z, burst_sec)

            try:
                verdict = input("  실제로 돌았습니까? (y/n) > ").strip().lower()
            except EOFError:
                verdict = ""
            turned = verdict.startswith("y")
            record = {
                "ts": time.time(),
                "angular_z": angular_z,
                "burst_sec": burst_sec,
                "turned": turned,
            }
            results.append(record)
            with open(log_path, "a") as f:
                f.write(json.dumps(record) + "\n")
            print(f"  기록됨: {'돌았음' if turned else '안 돌았음'}")
    except KeyboardInterrupt:
        print()
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

    if results:
        turned = sorted(r["angular_z"] for r in results if r["turned"])
        not_turned = sorted(r["angular_z"] for r in results if not r["turned"])
        print("\n[rotate-test] 요약")
        print(f"  돈 속도: {turned}")
        print(f"  안 돈 속도: {not_turned}")
        if turned:
            print(f"  → 이번 세션에서 확인된 최소 회전 속도: {min(turned):.3f} rad/s")
    print(f"[rotate-test] 로그: {log_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default=DEFAULT_TOPIC)
    ap.add_argument("--burst-sec", type=float, default=DEFAULT_BURST_SEC)
    ap.add_argument("--log", default=f"/tmp/inplace_rotation_test_{int(time.time())}.jsonl")
    args = ap.parse_args()
    run(args.topic, args.burst_sec, args.log)


if __name__ == "__main__":
    main()
