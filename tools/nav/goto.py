#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""오도메트리 좌표로 주행한다 — 바구니처럼 **위치가 고정된 목적지**용.

물체 접근(`approach.py`)은 카메라로 본다. 하지만 바구니에는 마커도 없고 YOLO 도
학습돼 있지 않다. 대신 바구니는 **움직이지 않는다.** 그러니 한 번 데려다 놓고
그 자리의 오도메트리 좌표를 기억해 두면, 다음부터는 그 좌표로 돌아가면 된다.

    ./goto.py --teach basket      지금 이 자리를 'basket' 으로 기억
    ./goto.py basket              그 자리로 주행
    ./goto.py --probe             오도메트리가 횡이동·회전을 재는지 검사

**옆걸음을 쓰지 않는다.** 실측(2026-08-23) 결과 이 오도메트리는 차동구동 모델로
적분한다 — 좌우 횡이동은 Δx, Δy 모두 0 으로 **아예 안 잡힌다.** 옆으로 가면
로봇은 움직였는데 좌표는 그대로라, 제어기가 영원히 수렴하지 못한다. 그래서
**회전 → 직진 → 회전** 으로만 간다. 회전(±35.1° 대칭)과 직진(-0.1021m)은 정확했다.

**한계.** 오도메트리는 바퀴 회전만 센다. 미끄러진 몫은 **누적되며 스스로 줄지
않는다** — 시각 서보처럼 매번 다시 보는 게 아니다. 짧은 구간에 쓰고 도착 후에는
눈으로 확인하는 편이 안전하다.

**좌표계는 바닥에 못박아 쓴다.** 오도메트리는 바퀴가 돈 것만 세므로, 로봇을
손으로 들어 옮기면 그 이동은 없던 일이 된다 — 저장해 둔 좌표가 전부 어긋난다.
베이스 노드를 다시 띄워도 마찬가지다.

그래서 바닥에 **홈 표시**를 하나 붙여 두고, 로봇을 거기 놓을 때마다 `--reset` 으로
원점을 다시 찍는다. 그러면 좌표계가 실제 바닥에 고정되므로 손으로 옮겨도 되고
노드를 다시 띄워도 된다. 시연 전 준비 절차에 이 리셋을 반드시 넣을 것.

**원점은 우리가 직접 들고 있는다.** 베이스의 `/set_odom` 토픽은 쓰지 않는다 —
벤더 코드에 `self.clock()` 오타가 있어(`get_clock` 이어야 한다) 메시지를 받는
즉시 노드가 죽는다(2026-08-23 확인). 대신 목적지 파일에 원점을 함께 저장하고,
모든 좌표를 그 원점 기준으로 변환해 쓴다. 결과는 같고 노드를 건드리지 않는다.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

TARGET_FILE = "/grippers/config/nav_targets.json"


def yaw_of(q) -> float:
    """쿼터니언에서 yaw 만 뽑는다(평면 주행이라 나머지는 안 쓴다)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Navigator(Node):
    def __init__(self):
        super().__init__("nav_goto")
        self.cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pose = None                      # (x, y, yaw)
        self.create_subscription(Odometry, "odom_raw", self._on_odom, 10)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def wait_pose(self, secs=5.0):
        t0 = time.monotonic()
        while self.pose is None and time.monotonic() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose

    def stop(self):
        for _ in range(5):
            self.cmd.publish(Twist()); time.sleep(0.02)

    def drive(self, vx, vy, wz, secs):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            self.cmd.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)
        self.stop()
        t1 = time.monotonic()                 # 완전히 서고 나서 재도록
        while time.monotonic() - t1 < 0.4:
            rclpy.spin_once(self, timeout_sec=0.05)

    def warmup(self, secs=0.5):
        """첫 명령이 유실되지 않게 연결을 먼저 세운다.

        노드가 막 뜬 직후에는 발행자-구독자 결합이 아직이라 처음 몇백 ms 의
        명령이 그냥 버려진다. 실측에서 첫 전진이 10.8cm 지시에 2.7cm 만 나왔다.
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            self.cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.05)

    # ── 검사 ────────────────────────────────────────────────────────────
    def probe(self, speed=0.09, secs=1.2):
        """전진·횡이동·회전을 각각 시켜 보고 오도메트리가 뭘 재는지 본다."""
        self.warmup()
        for label, (vx, vy, wz) in [
            ("전진", (speed, 0.0, 0.0)),
            ("후진", (-speed, 0.0, 0.0)),
            ("좌횡", (0.0, speed, 0.0)),
            ("우횡", (0.0, -speed, 0.0)),
            ("좌회전", (0.0, 0.0, 0.6)),
            ("우회전", (0.0, 0.0, -0.6)),
        ]:
            a = self.pose
            self.drive(vx, vy, wz, secs)
            b = self.pose
            print(f"  {label:<5} Δx={b[0]-a[0]:+.4f}m  Δy={b[1]-a[1]:+.4f}m  "
                  f"Δyaw={math.degrees(wrap(b[2]-a[2])):+.1f}°")

    # ── 주행: 회전 → 직진 → 회전 ────────────────────────────────────────
    def turn_to(self, goal_yaw, tol_deg=3.0, gain=1.2,
                max_ang=0.7, min_ang=0.35, timeout=25.0):
        tol = math.radians(tol_deg)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            d = wrap(goal_yaw - self.pose[2])
            if abs(d) <= tol:
                self.stop(); return True
            wz = max(-max_ang, min(max_ang, gain * d))
            if abs(wz) < min_ang:            # 데드밴드
                wz = math.copysign(min_ang, wz)
            t = Twist(); t.angular.z = wz
            self.cmd.publish(t)
            time.sleep(0.05)
        self.stop(); return False

    def drive_straight_to(self, tx, ty, tol_m=0.03, gain=1.0, gain_yaw=1.0,
                          max_lin=0.12, min_lin=0.05, max_ang=0.5,
                          timeout=60.0, verbose=True):
        """목표를 향한 채 직진한다. 방위는 진행하며 계속 보정한다.

        옆으로는 절대 가지 않는다 — 오도메트리가 그걸 못 재기 때문이다.
        """
        t0 = time.monotonic()
        last = 0.0
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            x, y, yaw = self.pose
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)
            if dist <= tol_m:
                self.stop(); return True, dist
            bearing = wrap(math.atan2(dy, dx) - yaw)
            if abs(bearing) > math.radians(90):
                # 목표를 지나쳤다. 뒤로 돌지 말고 여기서 멈춘다 —
                # 다시 회전 단계로 보내는 쪽이 안전하다.
                self.stop(); return False, dist
            vx = max(0.0, min(max_lin, gain * dist * math.cos(bearing)))
            if 0.0 < vx < min_lin:           # 데드밴드
                vx = min_lin
            wz = max(-max_ang, min(max_ang, gain_yaw * bearing))
            t = Twist(); t.linear.x = vx; t.angular.z = wz
            self.cmd.publish(t)
            now = time.monotonic()
            if verbose and now - last > 1.0:
                print(f"    남은거리 {dist*100:>5.1f}cm  "
                      f"조준차 {math.degrees(bearing):>+6.1f}°", flush=True)
                last = now
            time.sleep(0.05)
        self.stop()
        x, y, _ = self.pose
        return False, math.hypot(tx - x, ty - y)

    def goto(self, tx, ty, tyaw, *, tol_m=0.03, tol_deg=4.0,
             max_lin=0.12, passes=3):
        """회전 → 직진 → (필요하면 반복) → 최종 방위 정렬."""
        self.warmup()
        for k in range(1, passes + 1):
            x, y, _ = self.pose
            dist = math.hypot(tx - x, ty - y)
            if dist <= tol_m:
                break
            print(f"  [{k}/{passes}] 조준 회전")
            self.turn_to(math.atan2(ty - y, tx - x))
            print(f"  [{k}/{passes}] 직진")
            ok, dist = self.drive_straight_to(tx, ty, tol_m=tol_m,
                                              max_lin=max_lin)
            if ok:
                break
        print("  최종 방위 정렬")
        self.turn_to(tyaw, tol_deg=tol_deg)
        x, y, yaw = self.pose
        dist = math.hypot(tx - x, ty - y)
        return dist <= tol_m, dist, math.degrees(wrap(tyaw - yaw))


def to_home(p, o):
    """오도메트리 좌표 → 홈 기준 좌표."""
    dx, dy = p[0] - o[0], p[1] - o[1]
    c, s = math.cos(-o[2]), math.sin(-o[2])
    return (c * dx - s * dy, s * dx + c * dy, wrap(p[2] - o[2]))


def to_odom(h, o):
    """홈 기준 좌표 → 오도메트리 좌표."""
    c, s = math.cos(o[2]), math.sin(o[2])
    return (o[0] + c * h[0] - s * h[1], o[1] + s * h[0] + c * h[1],
            wrap(h[2] + o[2]))


def origin_of(d):
    o = d.get("_origin")
    return None if o is None else (o["x"], o["y"], o["yaw"])


def load_all():
    if not os.path.exists(TARGET_FILE):
        return {}
    with open(TARGET_FILE, encoding="utf-8") as f:
        return json.load(f)


def save_all(d):
    os.makedirs(os.path.dirname(TARGET_FILE), exist_ok=True)
    with open(TARGET_FILE, "w", encoding="utf-8") as f:
        json.dump(d, f, ensure_ascii=False, indent=2)


def main():
    ap = argparse.ArgumentParser(description="오도메트리 좌표로 주행")
    ap.add_argument("name", nargs="?", help="갈 목적지 이름")
    ap.add_argument("--teach", metavar="NAME",
                    help="지금 서 있는 자리를 이 이름으로 저장한다")
    ap.add_argument("--list", action="store_true", help="저장된 목적지 보기")
    ap.add_argument("--reset", action="store_true",
                    help="지금 자리를 홈(좌표 원점)으로 잡는다. "
                         "바닥 홈 표시에 로봇을 놓고 부를 것 — 손으로 옮겼거나 "
                         "노드를 다시 띄운 뒤에는 필수")
    ap.add_argument("--probe", action="store_true",
                    help="오도메트리가 횡이동·회전을 재는지 검사(로봇이 조금 움직인다)")
    ap.add_argument("--tol", type=float, default=0.03, help="도착 허용 오차(m)")
    ap.add_argument("--tol-deg", type=float, default=4.0, help="방위 허용 오차(도)")
    ap.add_argument("--max-speed", type=float, default=0.12)
    args = ap.parse_args()

    if args.list:
        d = load_all()
        if not d:
            print("저장된 목적지가 없습니다.")
        for k, v in d.items():
            tag = "원점" if k == "_origin" else "홈 기준"
            print(f"  {k:<12} x={v['x']:+.3f} y={v['y']:+.3f} "
                  f"yaw={math.degrees(v['yaw']):+.1f}°   ({tag})")
        return

    rclpy.init()
    node = Navigator()
    if node.wait_pose() is None:
        print("오도메트리를 못 받았습니다 — 베이스 드라이버가 도는지 확인하세요",
              file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    try:
        if args.reset:
            x, y, yaw = node.pose
            d = load_all()
            d["_origin"] = {"x": x, "y": y, "yaw": yaw,
                            "_측정일": time.strftime("%Y-%m-%d %H:%M:%S"),
                            "_설명": "바닥 홈 표시의 오도메트리 좌표. "
                                     "저장된 목적지는 전부 이 원점 기준이다"}
            save_all(d)
            n = len([k for k in d if not k.startswith("_")])
            print(f"[원점] 이 자리를 홈으로 잡았습니다 — "
                  f"오도메트리 x={x:+.3f} y={y:+.3f} yaw={math.degrees(yaw):+.1f}°")
            print(f"       저장된 목적지 {n}개가 이 원점을 기준으로 해석됩니다.")
            print(f"       바닥 표시를 지우지 마세요.")
            return

        if args.probe:
            print("[검사] 각 방향으로 조금씩 움직입니다. 주변을 비워 주세요.\n")
            node.probe()
            print("\n  좌횡/우횡에서 Δ가 전부 0 이면 정상입니다 — 이 오도메트리는")
            print("  차동구동 모델이라 옆걸음을 못 잽니다. goto 는 옆걸음을 쓰지 않습니다.")
            return

        if args.teach:
            d = load_all()
            o = origin_of(d)
            if o is None:
                print("원점이 없습니다. 바닥 홈 표시에 로봇을 놓고 "
                      "먼저 --reset 을 실행하세요.", file=sys.stderr)
                sys.exit(5)
            hx, hy, hyaw = to_home(node.pose, o)
            d[args.teach] = {"x": hx, "y": hy, "yaw": hyaw,
                             "_측정일": time.strftime("%Y-%m-%d %H:%M:%S"),
                             "_기준": "홈 표시 기준 좌표"}
            save_all(d)
            print(f"[교시] '{args.teach}' 저장 — 홈 기준 x={hx:+.3f} y={hy:+.3f} "
                  f"yaw={math.degrees(hyaw):+.1f}°")
            print(f"       {TARGET_FILE}")
            return

        if not args.name:
            print("목적지 이름을 주거나 --teach / --list / --probe 를 쓰세요",
                  file=sys.stderr)
            sys.exit(2)

        d = load_all()
        if args.name not in d:
            print(f"'{args.name}' 이 없습니다. --list 로 확인하세요", file=sys.stderr)
            sys.exit(3)
        o = origin_of(d)
        if o is None:
            print("원점이 없습니다. 홈 표시에 놓고 --reset 을 먼저 하세요.",
                  file=sys.stderr)
            sys.exit(5)
        t = d[args.name]
        tx, ty, tyaw = to_odom((t["x"], t["y"], t["yaw"]), o)
        hx, hy, hyaw = to_home(node.pose, o)
        print(f"현재  홈 기준 x={hx:+.3f} y={hy:+.3f} "
              f"yaw={math.degrees(hyaw):+.1f}°")
        print(f"목적  홈 기준 x={t['x']:+.3f} y={t['y']:+.3f} "
              f"yaw={math.degrees(t['yaw']):+.1f}°\n")
        # 손으로 옮기면 바퀴가 안 돌아 오도메트리가 그 이동을 못 본다. 그러면
        # 로봇은 여전히 목적지에 있다고 믿고 "이미 도착" 이라 답한다 — 조용히
        # 틀리는 실패라 알아채기 어렵다. 시작부터 오차가 0 이면 짚어 준다.
        d0 = math.hypot(tx - node.pose[0], ty - node.pose[1])
        if d0 <= args.tol:
            print(f"⚠ 이미 목적지에 있다고 나옵니다(오차 {d0*100:.1f}cm).")
            print("  로봇을 손으로 옮기셨다면 오도메트리가 그 이동을 못 본 것입니다.")
            print("  홈 표시에 놓고 --reset 을 실행한 뒤 다시 시도하세요.\n")

        ok, dist, dyaw = node.goto(tx, ty, tyaw,
                                   tol_m=args.tol, tol_deg=args.tol_deg,
                                   max_lin=args.max_speed)
        if ok:
            print(f"\n도착 — 오차 {dist*100:.1f}cm, {dyaw:+.1f}°")
        else:
            print(f"\n시간 초과 — {dist*100:.1f}cm, {dyaw:+.1f}° 남음", file=sys.stderr)
            sys.exit(4)
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
