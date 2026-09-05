#!/usr/bin/env python3
"""직진의 실제 최소 속도를 라이다로 잰다.

## 왜 필요한가

`motion.AGREED_LINEAR_MPS = 0.1` 은 2026-08-24 실기에서 "0.05 는 안 돌고 0.1 은
움직인다"로 정한 값이다. 그 뒤 팔과 Hailo 가 올라가 무게가 늘었다. 정지마찰은
무게에 민감하고, **회전 쪽은 실제로 문턱이 올라가 있었다** — 합의값 0.25 가
못 도는 값이었고 실측 최소가 0.35 였다(rotation_threshold_sweep.py).

2026-09-05 실기에서 축 보정 도구가 4회 모두 "0.0~0.4cm 밖에 안 움직였다"로
끝났다. 측위 획득률은 100% 였고 명령도 매 사이클 나갔다. 즉 명령은 갔는데
바퀴가 안 돈 것이다. 그때 `linear 0.1` 이 만드는 바퀴 명령이 rps 0.49 였고,
직접 넣어 본 시험에서 rps 0.5 는 안 돌고 1.0 은 돌았다.

## 왜 라이다인가

직진은 회전과 달리 IMU 로 못 본다 — 등속 운동의 가속도는 0 이다. `/odom_raw`
는 명령을 그대로 적분할 뿐이라 처음부터 못 쓴다(rotation_threshold_sweep.py
주석 참고). 라이다는 주변까지의 거리를 실제로 재므로, 차가 움직이면 빔 값이
바뀌고 안 움직이면 안 바뀐다 — 명령과 독립인 증인이다.

판정은 **빔별 거리 변화의 중앙값**으로 한다. 평균이 아니라 중앙값을 쓰는 이유는
사람이 지나가거나 빔 하나가 튀어도 판정이 흔들리지 않게 하려는 것이다.

## 안전

차가 **앞으로 간다.** 구간당 최대 `--burst` 초이므로 0.3m/s x 1.2초 = 36cm 다.
앞을 1.5m 쯤 비우고 실행할 것. 어떤 경로로 끝나든 finally 에서 정지를 보낸다.

Host 미션을 **먼저 내릴 것.** 같은 토픽에 둘이 쓰면 명령이 섞인다.

사용법 (컨테이너 안, ROS_DOMAIN_ID=21)
    python3 tools/linear_threshold_sweep.py
    python3 tools/linear_threshold_sweep.py --speeds 0.1 0.15 0.2 0.25
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import UInt16
from sensor_msgs.msg import LaserScan

#: 빔 거리 변화의 중앙값이 이보다 크면 "움직였다"고 본다(m). 정지 상태의
#: 라이다 잡음은 보통 1cm 아래다 — 3cm 면 잡음은 확실히 넘고, 데드밴드 근처의
#: 굼뜬 움직임은 놓치지 않는 선이다.
MOVED_M = 0.03

#: 명령 직후 구간은 버린다. 정지마찰을 뜯는 동안은 아직 등속이 아니다.
RAMP_SKIP_S = 0.3

#: 명령 발행 주기. mission_orchestrator 가 워치독 정지로 /cmd_vel 에 0 을
#: ~10Hz 로 계속 쏘고, odom_publisher 가 두 토픽을 다 구독해 각각 set_motor 를
#: 낸다. 어느 토픽으로 보내든 0 이 사이사이 끼어들므로 주기를 높여 눌러야
#: 한다 — 근거는 rotation_threshold_sweep.DEFAULT_TICK_HZ 주석과 같다.
DEFAULT_TICK_HZ = 60.0
#: 이 전압 아래에서 잰 값은 못 믿는다(mV). 전압이 떨어지면 모터 토크가 같이
#: 떨어져서 데드밴드가 위로 올라간다 — 즉 배터리 상태가 측정값에 그대로
#: 섞인다.
#:
#: 2026-09-05 에 그 함정을 밟았다. 같은 날 두 번 잰 직진 문턱이 서로 모순됐고
#: (0.20 이 3.3cm 움직였다가 0.24 도 0.2cm), 그 사이에 전압이 8.2V 에서
#: 7.4V 로 떨어진 것이 원인이었다. 하마터면 그 값으로 AGREED_LINEAR_MPS 를
#: 고칠 뻔했다.
#:
#: 7600 은 저장소 실측 기준(7181 정상 주행 / 6944 베이스 정지)보다 여유를 둔
#: 값이다. 문턱을 재려면 정상 주행 하한이 아니라 넉넉한 상태여야 한다.
MIN_BATTERY_MV = 7600


class Sweep(Node):
    def __init__(self, topic: str, tick_hz: float) -> None:
        super().__init__("linear_threshold_sweep")
        self.tick_hz = tick_hz
        self.pub = self.create_publisher(Twist, topic, 10)
        self.create_subscription(LaserScan, "/scan_raw", self._on_scan, 5)
        self.create_subscription(UInt16, "/ros_robot_controller/battery",
                                 self._on_battery, 10)
        self.battery_mv = None

        self.scan = None
        self.seen = 0

    def _on_battery(self, msg) -> None:
        self.battery_mv = int(msg.data)

    def _on_scan(self, msg: LaserScan) -> None:
        self.seen += 1
        self.scan = [r for r in msg.ranges]

    def _publish(self, vx: float) -> None:
        twist = Twist()
        twist.linear.x = float(vx)
        self.pub.publish(twist)

    def snapshot(self, settle_s: float = 0.6):
        """정지 상태에서 스캔 한 장. 움직이는 중에 찍으면 비교가 무의미하다."""
        end = time.monotonic() + settle_s
        while time.monotonic() < end:
            self._publish(0.0)
            rclpy.spin_once(self, timeout_sec=1.0 / self.tick_hz)
        return list(self.scan) if self.scan else None

    def burst(self, vx: float, burst_s: float) -> None:
        end = time.monotonic() + burst_s
        while time.monotonic() < end:
            self._publish(vx)
            rclpy.spin_once(self, timeout_sec=1.0 / self.tick_hz)

    def stop(self, settle_s: float = 0.8) -> None:
        end = time.monotonic() + settle_s
        while time.monotonic() < end:
            self._publish(0.0)
            rclpy.spin_once(self, timeout_sec=1.0 / self.tick_hz)


def _median_change(a, b) -> float:
    """두 스캔의 빔별 거리 변화 중앙값(m). 둘 다 유효한 빔만 본다."""
    diffs = [abs(x - y) for x, y in zip(a, b)
             if math.isfinite(x) and math.isfinite(y) and x > 0.05 and y > 0.05]
    return statistics.median(diffs) if diffs else 0.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--speeds", type=float, nargs="+",
                    default=[0.10, 0.15, 0.20, 0.25, 0.30])
    ap.add_argument("--burst", type=float, default=1.2, help="구간당 전진 시간(초)")
    ap.add_argument("--topic", default="controller/cmd_vel")
    ap.add_argument("--rate", type=float, default=DEFAULT_TICK_HZ)
    ap.add_argument("--out", default="/grippers/runs/linear_sweep.jsonl")
    ap.add_argument("--skip-battery-check", action="store_true",
                    help="전압이 낮아도 강행한다. 값을 못 믿는다는 걸 알고 쓸 것")
    args = ap.parse_args()

    if max(args.speeds) > 0.4:
        print("0.4 m/s 를 넘는 값은 받지 않습니다 — 문턱을 재는 시험입니다")
        return 1

    rclpy.init()
    node = Sweep(args.topic, args.rate)
    # 배터리부터 본다. 낮은 전압에서 잰 문턱은 배터리 상태를 재는 것이지
    # 로봇의 성질을 재는 것이 아니다 — 근거는 MIN_BATTERY_MV 주석.
    deadline = time.monotonic() + 6.0
    while node.battery_mv is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.battery_mv is None:
        print("배터리 값을 못 읽었습니다 — ros_robot_controller 가 떠 있는지 보십시오")
    elif node.battery_mv < MIN_BATTERY_MV:
        print(f"배터리 {node.battery_mv} mV — {MIN_BATTERY_MV} mV 미만입니다.")
        print("이 전압에서 잰 문턱은 로봇이 아니라 배터리 상태를 재는 것입니다.")
        print("충전한 뒤 다시 재십시오. (--skip-battery-check 로 넘길 수 있습니다)")
        if not args.skip_battery_check:
            return 1
    else:
        print(f"배터리 {node.battery_mv} mV — 측정 가능")

    print("라이다를 기다리는 중 ...", flush=True)
    deadline = time.monotonic() + 10.0
    while node.seen == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.seen == 0:
        print("스캔이 안 옵니다 — LD19 노드가 떠 있는지 보십시오")
        return 1

    # 정지 상태에서 두 장을 찍어 잡음 바닥을 먼저 잰다. 판정 문턱이 이 바닥보다
    # 충분히 위인지 매번 확인하려는 것이다 — 주변이 바뀌면 잡음도 바뀐다.
    base_a = node.snapshot()
    base_b = node.snapshot()
    noise = _median_change(base_a, base_b) if base_a and base_b else 0.0
    print(f"\n정지 잡음(빔 변화 중앙값) {noise*100:.1f}cm   판정 문턱 {MOVED_M*100:.0f}cm"
          + ("   ⚠️ 잡음이 문턱에 가깝습니다" if noise > MOVED_M / 3 else ""))

    print(f"\n{'명령(m/s)':>10}  {'빔 변화':>10}  판정")
    print("-" * 36)
    rows = []
    try:
        for speed in args.speeds:
            before = node.snapshot()
            node.burst(speed, args.burst)
            node.stop()
            after = node.snapshot()
            if not before or not after:
                print(f"{speed:10.2f}  {'스캔 없음':>10}")
                continue
            change = _median_change(before, after)
            moved = change >= MOVED_M
            print(f"{speed:10.2f}  {change*100:9.1f}cm  "
                  + ("움직였다" if moved else "안 움직였다"))
            rows.append({"commanded": speed, "beam_change_m": change, "moved": moved})
    finally:
        # 어떤 경로로 끝나든 세운다. 예외로 전진 명령이 마지막에 남으면
        # 워치독이 잡아 줄 때까지 계속 간다.
        node.stop(settle_s=1.0)

    moved = [r["commanded"] for r in rows if r["moved"]]
    stuck = [r["commanded"] for r in rows if not r["moved"]]
    print("\n" + "=" * 36)
    if moved:
        print(f"움직인 최소 속도  {min(moved):.2f} m/s")
        print(f"안 움직인 최대    {max(stuck):.2f} m/s" if stuck else "  모두 움직였다")
        # 문턱 바로 위를 쓰면 바닥이나 적재가 조금만 달라져도 다시 멈춘다.
        # 회전 쪽과 같은 근거로 실측 최소의 1.5배를 권한다.
        print(f"\n권장 AGREED_LINEAR_MPS  {min(moved) * 1.5:.2f} m/s "
              f"(실측 최소의 1.5배)")
    else:
        print("어느 속도에서도 안 움직였습니다 — 데드밴드가 아니라 다른 문제입니다.")
        print("베이스 전원 스위치, 모터 커넥터, set_motor 가 실제로 나가는지 보십시오.")

    try:
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "a", encoding="utf-8") as f:
            f.write(json.dumps({"wall": time.strftime("%Y-%m-%d %H:%M:%S"),
                                "noise_m": noise, "rows": rows},
                               ensure_ascii=False) + "\n")
        print(f"기록: {out}")
    except Exception as e:  # noqa: BLE001 — 화면에 이미 다 나왔다
        print(f"(기록 실패: {type(e).__name__}: {e})")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
