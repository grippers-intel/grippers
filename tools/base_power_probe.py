#!/usr/bin/env python3
"""베이스 모터가 전류를 먹는지 1초마다 찍는다 — 접촉 불량 찾기용.

## 왜 필요한가

명령 경로는 멀쩡한데 바퀴가 안 도는 상태가 반복됐다(2026-09-06). 전 구간이
정상으로 보인다 — Host UDP 도달, orchestrator APPROACH 유지, cmd_vel 0.15,
set_motor -0.735, base_liveness ALIVE. 그런데 전압 강하가 0 이고 탑뷰 실측
이동도 0 이다. 한 시간 전 같은 시험에서는 강하 22~36mV 에 537mm 를 갔다.

즉 **보드 아래 어딘가가 왔다 갔다 한다.** 배터리 커넥터, 전원 스위치 접점,
모터 케이블 중 하나다.

## 왜 전압 강하로 보는가

이 차량에는 바퀴가 실제로 도는지 알 수단이 없다 — /odom_raw 는 명령을 그대로
적분하고 엔코더 피드백이 없다(domain/task/base_liveness 주석). set_motor 에
값이 실리는 것도 "명령됐다"는 뜻일 뿐 보드가 집행했다는 뜻이 아니다.

전압 강하는 다르다. 모터가 실제로 전류를 뽑으면 팩 전압이 떨어진다. 명령과
독립인 신호이고, 사람이 커넥터를 만지는 즉시 반응한다.

    정상 구동   약 100mV 강하 (2026-08 실측, 부하에 따라 20~110mV)
    전원 끊김   0~5mV — 명령은 나가는데 전류를 안 먹는다

## 쓰는 법

로봇 옆에서 **화면을 보면서** 커넥터와 스위치를 하나씩 만진다. 강하가 0 에서
수십 mV 로 바뀌는 순간 만지던 곳이 범인이다.

    배터리 커넥터 -> 전원 스위치 -> 모터 케이블 4개 순으로

## 안전

전원이 붙으면 로봇이 **제자리에서 돈다**(전진이 아니라 회전 패턴을 쓴다 —
갑자기 앞으로 튀어나가지 않게). 그래도 바퀴를 띄우거나 주변을 비울 것.
어떤 경로로 끝나든 finally 에서 0 속도를 보낸다.

    python3 tools/base_power_probe.py             # 120초
    python3 tools/base_power_probe.py --seconds 300
"""

from __future__ import annotations

import argparse
import statistics
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
from sensor_msgs.msg import Imu
from ros_robot_controller_msgs.msg import MotorsState, MotorState

#: 이만큼 넘게 떨어지면 모터가 전류를 먹고 있다고 본다(mV). 정상 구동이
#: 20~110mV 이고 잡음이 한 자릿수라, 10 이면 둘을 확실히 가른다.
DRAWING_MV = 10

#: 시험 회전 속도(rps). 정지마찰을 확실히 이기는 값이어야 "전원은 붙었는데
#: 너무 약해서 안 돈다"와 헷갈리지 않는다. 2026-09-06 실측에서 1.0 이 돌았다.
PROBE_RPS = 1.2

#: 한 회차의 정지 구간과 구동 구간 길이(초). 1초면 배터리 토픽이 몇 개는
#: 들어온다 — 이 토픽은 1Hz 근처라 더 짧게 잡으면 표본이 0 이 된다.
PHASE_S = 1.2


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("base_power_probe")
        self.pub = self.create_publisher(
            MotorsState, "/ros_robot_controller/set_motor", 10)
        self.create_subscription(
            UInt16, "/ros_robot_controller/battery", self._on_batt, 20)
        self.create_subscription(
            Imu, "/ros_robot_controller/imu_raw", self._on_imu, 30)
        self.volts: list[int] = []
        self.spins: list[float] = []

    def _on_batt(self, msg) -> None:
        self.volts.append(int(msg.data))

    def _on_imu(self, msg) -> None:
        self.spins.append(abs(msg.angular_velocity.z))

    def drive(self, rps: float, secs: float):
        """네 바퀴 같은 부호 = 제자리 회전. 앞으로 튀어나가지 않는다."""
        msg = MotorsState()
        msg.data = [MotorState(id=i, rps=float(rps)) for i in (1, 2, 3, 4)]
        self.volts.clear()
        self.spins.clear()
        end = time.monotonic() + secs
        while time.monotonic() < end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
        return list(self.volts), list(self.spins)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--rps", type=float, default=PROBE_RPS)
    args = ap.parse_args()

    rclpy.init()
    node = Probe()
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.05)

    print("커넥터와 스위치를 하나씩 만지면서 아래 숫자를 보십시오.")
    print("강하가 0 에서 수십 mV 로 바뀌는 순간 만지던 곳이 범인입니다.")
    print(f"판정 문턱 {DRAWING_MV}mV · 회전 {args.rps}rps · Ctrl+C 로 종료\n")
    print(f"{'시각':>8}  {'정지mV':>7} {'구동mV':>7} {'강하':>6}  {'IMU':>6}  판정")
    print("-" * 56)

    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            idle, _ = node.drive(0.0, PHASE_S)
            load, spin = node.drive(args.rps, PHASE_S)
            node.drive(0.0, 0.3)
            if not idle or not load:
                print(f"{time.strftime('%H:%M:%S'):>8}  배터리 표본 없음 — "
                      "ros_robot_controller 가 떠 있는지 보십시오", flush=True)
                continue
            a, b = statistics.fmean(idle), statistics.fmean(load)
            sag = a - b
            spin_mean = statistics.fmean(spin) if spin else 0.0
            verdict = "전류 먹음  ✅" if sag >= DRAWING_MV else "전원 끊김"
            if sag >= DRAWING_MV and spin_mean > 0.1:
                verdict = "돌고 있음  ✅"
            print(f"{time.strftime('%H:%M:%S'):>8}  {a:7.0f} {b:7.0f} "
                  f"{sag:6.0f}  {spin_mean:6.3f}  {verdict}", flush=True)
    except KeyboardInterrupt:
        print("\n중단")
    finally:
        # 어떤 경로로 끝나든 세운다. 예외로 회전 명령이 마지막에 남으면
        # 보드가 그 값을 무기한 집행한다(base_liveness 주석의 2026-08-28 사고).
        node.drive(0.0, 1.0)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
