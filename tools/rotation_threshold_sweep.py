#!/usr/bin/env python3
"""제자리 회전의 실제 최소 속도를 IMU 로 잰다.

## 왜 또 만드나 — inplace_rotation_test.py 와 무엇이 다른가

그 도구는 **사람이 보고 y/n 으로 판정**한다. 오도메트리를 못 믿기 때문인데
(`/odom_raw` 는 명령값을 그대로 적분할 뿐이다), 사람 판정은 회차마다 기준이
흔들리고 "조금 움찔했다"를 뭐라고 부를지가 사람마다 다르다.

`/ros_robot_controller/imu_raw` 는 그 문제가 없다. MCU 가 직접 재는 각속도라
명령과 독립이다. 사람 대신 이걸 증인으로 쓰면 판정이 재현 가능해지고, 덤으로
**손실률**(실제/명령)까지 나온다.

2026-09-05 에 이 신호가 실제로 한 일은 정지마찰 판정이 아니라 **전원 진단**
이었다. 명령이 set_motor 까지 멀쩡히 가는데 IMU 가 0.002 rad/s 였고, rps 1.0
을 직접 넣어도 같았다. 배터리 전압 강하가 0(정상 구동시 106mV)이어서 모터가
전류를 아예 안 먹는 것으로 좁혀졌고, 베이스 전원 스위치가 꺼져 있었다.
컨트롤러 보드는 Pi 와 USB 로 이어져 있어 **스위치가 꺼져도 로직이 살아
통신은 된다** — 텔레메트리만 보고 "보드가 살아 있으니 전원은 괜찮다"고
넘기면 몇 시간을 날린다.

## 왜 다시 재나

0.355 rad/s 는 2026-08-24 값이다. 그 뒤로 팔과 Hailo 가 올라가 무게가 늘었고
바닥도 그때와 같다는 보장이 없다. 정지마찰은 둘 다에 민감하다.

## 안전

- 로봇이 **제자리에서 돈다.** 주변을 비우고 실행할 것.
- 매 구간 뒤 정지 명령을 내고, 어떤 경로로 끝나든 finally 에서 0 을 보낸다.
- 방향을 번갈아 돌려 한쪽으로 계속 감기지 않게 한다.
- Host 미션을 **먼저 내릴 것.** 같은 토픽에 둘이 쓰면 명령이 섞인다.

사용법 (컨테이너 안, ROS_DOMAIN_ID=21)
    python3 tools/rotation_threshold_sweep.py
    python3 tools/rotation_threshold_sweep.py --speeds 0.25 0.4 0.55 0.7 0.85 1.0
"""

from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu

# 명령 구간에서 이 값을 넘는 각속도가 관측되면 "돌았다"고 본다. 정지 상태의
# IMU 잡음이 0.002 rad/s 수준이었으므로 25배 여유다 — 움찔거림을 회전으로
# 세지 않으면서, 느리게라도 도는 것은 놓치지 않는 선.
MOVED_RAD_S = 0.05

# 명령 직후 구간은 버린다. 정지마찰을 뜯는 동안은 아직 정상 회전이 아니라서
# 평균에 넣으면 손실률이 실제보다 나쁘게 나온다.
RAMP_SKIP_S = 0.35

# 명령 발행 주기. 기본을 60Hz 로 크게 잡은 이유가 있다 — mission_orchestrator
# 는 Host 미션이 내려가 있어도 워치독 정지로 /cmd_vel 에 0 을 ~10Hz 로 계속
# 쏘고, odom_publisher 가 /cmd_vel 과 /controller/cmd_vel 을 **둘 다** 구독해
# 각각 set_motor 를 낸다. 즉 어느 토픽으로 보내든 0 이 사이사이 끼어든다.
# 20Hz 로 보내면 세 번에 한 번이 0 이라 모터가 끊겨 문턱이 실제보다 높게
# 나온다(2026-09-05 첫 스윕이 그렇게 오염됐다). 60Hz 면 0 의 비율이 1/7 로
# 떨어지고 빈 구간도 17ms 이하라 정지마찰 판정을 왜곡하지 않는다.
DEFAULT_TICK_HZ = 60.0


class Sweep(Node):
    def __init__(self, topic: str, tick_hz: float = DEFAULT_TICK_HZ) -> None:
        super().__init__("rotation_threshold_sweep")
        self.tick_hz = tick_hz
        self.pub = self.create_publisher(Twist, topic, 10)
        self.create_subscription(Imu, "/ros_robot_controller/imu_raw",
                                 self._on_imu, 20)
        self._samples: list[tuple[float, float]] = []   # (수신시각, wz)
        self.imu_seen = 0

    def _on_imu(self, msg: Imu) -> None:
        self.imu_seen += 1
        self._samples.append((time.monotonic(), msg.angular_velocity.z))

    def _publish(self, wz: float) -> None:
        twist = Twist()
        twist.angular.z = float(wz)
        self.pub.publish(twist)

    def stop(self, settle_s: float = 0.8) -> None:
        end = time.monotonic() + settle_s
        while time.monotonic() < end:
            self._publish(0.0)
            rclpy.spin_once(self, timeout_sec=1.0 / self.tick_hz)

    def burst(self, wz: float, burst_s: float) -> list[float]:
        """wz 로 burst_s 동안 돌리고, 램프 구간을 뺀 IMU 각속도들을 돌려준다."""
        self._samples.clear()
        t0 = time.monotonic()
        end = t0 + burst_s
        while time.monotonic() < end:
            self._publish(wz)
            rclpy.spin_once(self, timeout_sec=1.0 / self.tick_hz)
        return [w for (t, w) in self._samples if t - t0 >= RAMP_SKIP_S]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--speeds", type=float, nargs="+",
                    default=[0.25, 0.35, 0.45, 0.55, 0.7, 0.85, 1.0])
    ap.add_argument("--burst", type=float, default=1.2, help="구간당 회전 시간(초)")
    ap.add_argument("--topic", default="cmd_vel")
    ap.add_argument("--rate", type=float, default=DEFAULT_TICK_HZ,
                    help="명령 발행 주기(Hz). 경쟁 발행자가 없으면 20 으로 낮춰도 된다")
    ap.add_argument("--out", default="/grippers/runs/rotation_sweep.jsonl")
    args = ap.parse_args()

    if max(args.speeds) > 1.5:
        print("1.5 rad/s 를 넘는 값은 받지 않습니다 — 제자리 회전 시험입니다")
        return 1

    rclpy.init()
    node = Sweep(args.topic, args.rate)

    print("IMU 를 기다리는 중 ...", flush=True)
    deadline = time.monotonic() + 10.0
    while node.imu_seen == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.imu_seen == 0:
        print("IMU 가 안 옵니다 — ros_robot_controller 가 떠 있는지 보십시오")
        return 1

    # 정지 상태의 잡음을 먼저 잰다. 판정 문턱이 이 바닥보다 충분히 위인지
    # 매번 확인하려는 것이다 — 로봇이나 바닥이 바뀌면 잡음도 바뀐다.
    node.stop(settle_s=1.0)
    idle = node.burst(0.0, 1.0)
    idle_rms = (statistics.fmean([w * w for w in idle]) ** 0.5) if idle else 0.0
    print(f"\n정지 잡음 RMS {idle_rms:.4f} rad/s   판정 문턱 {MOVED_RAD_S} rad/s"
          + ("   ⚠️ 잡음이 문턱에 가깝습니다" if idle_rms > MOVED_RAD_S / 3 else ""))

    print(f"\n{'명령':>8}  {'실측(평균)':>12}  {'최대':>8}  {'손실':>7}  판정")
    print("-" * 52)
    rows = []
    try:
        for i, speed in enumerate(args.speeds):
            # 방향을 번갈아 — 한쪽으로만 돌리면 시험 내내 계속 감긴다.
            signed = speed if i % 2 == 0 else -speed
            samples = node.burst(signed, args.burst)
            node.stop()
            if not samples:
                print(f"{speed:8.2f}  {'IMU 없음':>12}")
                continue
            mag = [abs(w) for w in samples]
            mean, peak = statistics.fmean(mag), max(mag)
            moved = mean >= MOVED_RAD_S
            loss = 1.0 - mean / speed
            print(f"{speed:8.2f}  {mean:12.3f}  {peak:8.3f}  {loss * 100:6.1f}%  "
                  + ("돌았다" if moved else "안 돌았다"))
            rows.append({"commanded": speed, "measured_mean": mean,
                         "measured_peak": peak, "loss": loss, "moved": moved,
                         "n": len(mag)})
    finally:
        # 어떤 경로로 끝나든 반드시 세운다. 예외가 나서 회전 명령이 마지막으로
        # 남으면 워치독이 잡아 줄 때까지 계속 돈다.
        node.stop(settle_s=1.0)

    moved = [r["commanded"] for r in rows if r["moved"]]
    stuck = [r["commanded"] for r in rows if not r["moved"]]
    print("\n" + "=" * 52)
    if moved:
        print(f"돈 최소 속도    {min(moved):.2f} rad/s")
        print(f"안 돈 최대 속도  {max(stuck):.2f} rad/s" if stuck else "  모두 돌았다")
        # 문턱 바로 위를 쓰면 바닥이 조금만 달라져도 다시 멈춘다. 실측한
        # 최소치의 1.5배를 권한다 — 2026-08-24 기록의 "안정 구간"도 최소치의
        # 3배 근처였다.
        print(f"\n권장 AGREED_ROTATION_RAD_S  {min(moved) * 1.5:.2f} rad/s "
              f"(실측 최소의 1.5배)")
    else:
        print("어느 속도에서도 안 돌았습니다 — 정지마찰이 아니라 다른 문제입니다")

    try:
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "a", encoding="utf-8") as f:
            f.write(json.dumps({"wall": time.strftime("%Y-%m-%d %H:%M:%S"),
                                "idle_rms": idle_rms, "rows": rows},
                               ensure_ascii=False) + "\n")
        print(f"기록: {out}")
    except Exception as e:  # noqa: BLE001 — 화면에 이미 다 나왔다
        print(f"(기록 실패: {type(e).__name__}: {e})")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
