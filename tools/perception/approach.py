#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""물체 앞 파지 위치로 자동 접근한다 — 관측하고, 조금 움직이고, 다시 관측한다.

**왜 제어 루프인가.** 사람이 화면 좌표를 보며 손으로 맞추면 드리프트가 쌓여 안
맞는다(실측: x 를 목표에 맞췄는데도 팔이 물체를 지나쳐 내려갔다). 루프는 매번
다시 관측하므로 이전 오차가 누적되지 않고 스스로 수렴한다.

**두 개의 독립된 신호를 쓴다.**
  거리 ← 검출 박스의 **높이**. 가까워질수록 커지고 좌우 이동과 무관하다.
  좌우 ← 박스 아래변 중점의 **x**.

화면 y 는 쓰지 않는다. 파지 지점 부근에서 480 으로 포화돼 거리를 구분하지 못한다.

**움직인 뒤 반드시 멈추고 관측한다.** 합의 필터는 정지 상태를 전제로 한다.
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

sys.path.insert(0, "/grippers/tools/perception")
from floor_observer import FloorObserver, RELIABLE  # noqa: E402

TARGET_DIR = "/grippers/config"
LEGACY_TARGET = f"{TARGET_DIR}/approach_target.json"   # 룩 교시본. 이름 없이 저장하던 시절


def target_file(cls: str) -> str:
    """목표마다 따로 저장한다 — 바구니를 교시하다 룩 기준값을 덮어쓰면 안 된다."""
    return f"{TARGET_DIR}/approach_target_{cls}.json"
MAX_LIN = 0.2      # 컨트롤러 안전 클램프와 동일


def make_approacher(base):
    """관측기에 /cmd_vel 발행을 얹는다. 같은 노드라 카메라 구독을 공유한다.

    베이스를 인자로 받는 이유 — 목표가 물체면 YOLO(FloorObserver), 바구니면
    ArUco(ArucoObserver) 다. 두 관측기가 같은 Observation 을 내놓으므로 아래
    제어 루프는 어느 쪽이든 그대로 돈다.
    """

    class Approacher(base):
        def __init__(self, **kw):
            super().__init__(**kw)
            self.cmd = self.create_publisher(Twist, "cmd_vel", 10)
            self._odom = None
            # EKF 가 없어 /odom 은 비어 있다. 바퀴 오도메트리 원본을 쓴다.
            self.create_subscription(Odometry, "odom_raw", self._on_odom, 10)

        def _on_odom(self, msg):
            p = msg.pose.pose.position
            self._odom = (p.x, p.y)

        def nudge(self, vx: float, vy: float, secs: float, wz: float = 0.0):
            """짧게 움직이고 확실히 멈춘다. 정지 명령은 여러 번 낸다."""
            t = Twist()
            t.linear.x = float(vx); t.linear.y = float(vy)
            t.angular.z = float(wz)
            t0 = time.monotonic()
            while time.monotonic() - t0 < secs:
                self.cmd.publish(t)
                rclpy.spin_once(self, timeout_sec=0.02)
                time.sleep(0.03)
            for _ in range(5):
                self.cmd.publish(Twist()); time.sleep(0.02)

        def collect(self, n: int):
            """정지 상태에서 n 프레임을 새로 모은다."""
            self._frames.clear()
            self._t_open = time.monotonic()
            t0 = time.monotonic()
            while not self.ready and time.monotonic() - t0 < 15:
                rclpy.spin_once(self, timeout_sec=0.2)
            return self.analyse() if self.ready else []

        def push_forward(self, mm, speed=0.09, scale=1.15, timeout=25.0):
            """수렴 지점에서 파지 지점까지 **개루프로** 전진한다.

            **왜 개루프인가.** 파지 지점에서는 물체가 화면 아래로 잘려 나가
            박스 높이가 거리 신호 구실을 못 한다(실측: 교시값 +20px 부터
            검출을 잃는다). config/grasp_target.json 에 팀이 남긴 관찰과 같다.

            **짧게 밀고 멈춰서 재기를 반복한다.** 한 번에 밀면 오도메트리가
            목표에 닿는 순간 멈춤을 내도 관성으로 계속 굴러간다 — 실측에서
            25mm 지시가 42.8mm 가 됐다. 멈춘 뒤에 재면 그 몫이 다음 회차에
            자동으로 반영된다.

            **속도는 접근 루프보다 빠르게 쓴다.** min_speed(0.05)는 데드밴드
            경계라 정지 상태에서 출발하면 바퀴가 안 돈다.

            scale — 오도메트리 배율 보정. 실측: 지시 100mm 에 실제 87mm.
            """
            target = abs(mm) * scale / 1000.0
            sign = 1.0 if mm >= 0 else -1.0

            t0 = time.monotonic()
            while self._odom is None and time.monotonic() - t0 < 3.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self._odom is None:
                secs = target / speed
                print(f"  오도메트리 없음 — {secs:.2f}s 시간 기반으로 전진")
                self.nudge(sign * speed, 0.0, secs)
                return None

            x0, y0 = self._odom
            moved = 0.0
            t_start = time.monotonic()
            while moved < target and time.monotonic() - t_start < timeout:
                secs = max(0.12, min(0.30, (target - moved) / speed))
                self.nudge(sign * speed, 0.0, secs)
                t1 = time.monotonic()          # 완전히 서고 나서 잰다
                while time.monotonic() - t1 < 0.25:
                    rclpy.spin_once(self, timeout_sec=0.05)
                moved = math.hypot(self._odom[0] - x0, self._odom[1] - y0)

            got = moved * 1000.0 / scale       # 실제 이동 추정(mm)
            if got < abs(mm) * 0.6:
                print(f"  ⚠ {abs(mm):.0f}mm 를 지시했는데 {got:.1f}mm 만 갔습니다. "
                      f"--push-speed 를 올리거나 배터리를 확인하세요.", file=sys.stderr)
            return got

        def search(self, pick, n, speed=0.6, burst=0.35, tries=30):
            """제자리에서 돌며 목표를 찾는다.

            파지 직후 로봇은 물체가 있던 쪽을 보고 있다. 바구니는 대개 시야 밖이라,
            찾기 전에는 접근 자체를 시작할 수 없다.
            """
            for k in range(1, tries + 1):
                hit = pick(self.collect(n))
                if hit is not None:
                    print(f"  탐색 {k}/{tries} — 찾음")
                    return hit
                print(f"  탐색 {k}/{tries} — 안 보임, 회전")
                self.nudge(0.0, 0.0, burst, wz=speed)
            return None

    return Approacher


def apply_floor(vx, vy, min_speed, max_speed, burst, min_burst):
    """데드밴드 아래 지령은 바퀴를 돌리지 못한다(실측: 0.017 m/s 에서 정지).

    속도 벡터를 통째로 키워 가장 작은 성분이 min_speed 에 닿게 하고, 같은 배율로
    시간을 줄인다. 방향과 이동 거리는 그대로 두고 '실제로 움직이는' 속도만 만든다.
    """
    mags = [abs(v) for v in (vx, vy) if v != 0.0]
    if not mags:
        return 0.0, 0.0, burst
    k = min(min_speed / min(mags), max_speed / max(mags))
    if k > 1.0:
        vx, vy, burst = vx * k, vy * k, max(min_burst, burst / k)
    # 상한에 걸려 아직도 데드밴드 아래인 성분은 버린다 — 모터만 울릴 뿐이다
    if 0.0 < abs(vx) < min_speed:
        vx = 0.0
    if 0.0 < abs(vy) < min_speed:
        vy = 0.0
    return vx, vy, burst


def load_target(cls: str):
    path = target_file(cls)
    if not os.path.exists(path) and os.path.exists(LEGACY_TARGET):
        with open(LEGACY_TARGET, encoding="utf-8") as f:
            legacy = json.load(f)
        if legacy.get("class") == cls:       # 이름 없이 저장해 둔 예전 교시본
            return legacy
        return None
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def save_target(cls, obs, note=""):
    d = {
        "_설명": "파지 위치에서 본 물체의 화면 특징. 접근 루프가 이 값에 수렴시킨다.",
        "_측정일": time.strftime("%Y-%m-%d %H:%M:%S"),
        "_방법": "그리퍼를 파지 자세로 보내고 손가락 정중앙에 물체를 놓은 뒤, 팔을 접고 관측",
        "_메모": note,
        "class": cls,
        "x": round(obs.x, 1),
        "h": round(obs.h, 1),
        "w": round(obs.w, 1),
        "_주의": "y 는 파지 지점에서 480 으로 포화돼 거리 신호로 못 쓴다. 그래서 저장하지 않는다.",
    }
    os.makedirs(TARGET_DIR, exist_ok=True)
    with open(target_file(cls), "w", encoding="utf-8") as f:
        json.dump(d, f, ensure_ascii=False, indent=2)
    return d


def main():
    ap = argparse.ArgumentParser(description="파지 위치로 자동 접근")
    ap.add_argument("--cls", default="rook", help="접근할 클래스")
    ap.add_argument("--teach", action="store_true",
                    help="지금 보이는 물체를 **파지 위치의 기준**으로 저장하고 끝낸다")
    ap.add_argument("--note", default="", help="--teach 시 남길 메모")
    ap.add_argument("--frames", type=int, default=10)
    ap.add_argument("--tol-x", type=float, default=8.0, help="좌우 허용 오차(px)")
    ap.add_argument("--tol-h", type=float, default=6.0, help="거리 허용 오차(박스 높이 px)")
    ap.add_argument("--gain-x", type=float, default=0.0009, help="x 오차 → 횡속도")
    ap.add_argument("--gain-h", type=float, default=0.0016, help="높이 오차 → 전후속도")
    ap.add_argument("--burst", type=float, default=0.35, help="한 번에 움직이는 시간(초)")
    ap.add_argument("--max-speed", type=float, default=0.08, help="최대 속도(m/s)")
    ap.add_argument("--min-speed", type=float, default=0.05,
                    help="이 속도 아래로는 바퀴가 정지마찰을 못 이긴다. "
                         "속도를 올리고 시간을 줄여 이동량을 맞춘다")
    ap.add_argument("--min-burst", type=float, default=0.15,
                    help="한 번 움직임의 최소 시간(초)")
    ap.add_argument("--align-first", type=float, default=2.0,
                    help="x 오차가 허용치의 이 배를 넘으면 전진을 늦춘다(좌우부터 맞춘다). "
                         "0 이면 끔")
    ap.add_argument("--max-iter", type=int, default=40)
    ap.add_argument("--push-scale", type=float, default=1.15,
                    help="오도메트리 배율 보정. 실측: 지시 100mm 에 실제 87mm")
    ap.add_argument("--push-speed", type=float, default=0.09,
                    help="마무리 전진 속도(m/s). 접근용 --min-speed 보다 빨라야 한다 — "
                         "0.05 는 데드밴드 경계라 정지 상태에서 출발하면 바퀴가 안 돈다")
    ap.add_argument("--push-only", type=float, default=0.0,
                    help="접근 없이 이 거리(mm)만 전진하고 끝낸다. **교정용** — "
                         "자로 실제 이동을 재서 오도메트리가 맞는지 확인한다")
    ap.add_argument("--frames-far", type=int, default=0,
                    help="오차가 클 때 쓸 프레임 수(기본: --frames 와 동일). "
                         "멀 때는 정밀도가 필요 없어 적게 모으면 훨씬 빠르다")
    ap.add_argument("--final-push", type=float, default=0.0,
                    help="수렴한 뒤 파지 지점까지 전진할 거리(mm). 오도메트리로 잰다. "
                         "파지 지점은 카메라 시야를 벗어나 시각 서보로 갈 수 없다")
    ap.add_argument("--offset-h", type=float, default=0.0,
                    help="교시값의 박스 높이를 이만큼 키워서 목표로 삼는다. "
                         "**양수면 더 가까이 간다.** 실측 환산 1px ≈ 2mm. "
                         "교시를 다시 하지 않고 파지 지점을 앞뒤로 미세 조정할 때 쓴다")
    ap.add_argument("--offset-x", type=float, default=0.0,
                    help="교시값의 x 를 이만큼 옮겨서 목표로 삼는다(양수=오른쪽)")
    ap.add_argument("--invert-y", action="store_true",
                    help="횡방향이 반대로 가면 이걸 준다")
    ap.add_argument("--aruco", action="store_true",
                    help="YOLO 대신 ArUco 마커로 목표를 잡는다(바구니용). "
                         "--cls 는 aruco4 처럼 준다")
    ap.add_argument("--dict", default="DICT_4X4_50", help="--aruco 일 때 마커 사전")
    ap.add_argument("--search", action="store_true",
                    help="목표가 안 보이면 제자리에서 돌며 찾는다")
    ap.add_argument("--search-speed", type=float, default=0.6,
                    help="탐색 회전 속도(rad/s)")
    ap.add_argument("--search-tries", type=int, default=30,
                    help="탐색 회전 횟수. 기본값이 대략 한 바퀴")
    ap.add_argument("--dry-run", action="store_true", help="움직이지 않고 오차만 본다")
    args = ap.parse_args()

    rclpy.init()
    if args.aruco:
        from aruco_observer import ArucoObserver
        node = make_approacher(ArucoObserver)(
            n_frames=args.frames, dictionary=args.dict, warmup=0.6)
    else:
        node = make_approacher(FloorObserver)(
            n_frames=args.frames, allowed=None, min_y=0.0, warmup=0.6)

    # ── 전진 교정 모드 ──────────────────────────────────────────────────
    if args.push_only:
        print(f"[교정] {args.push_only:.0f}mm 전진 — 자로 실제 이동을 재보세요")
        got = node.push_forward(args.push_only, args.push_speed, args.push_scale)
        if got is None:
            print("[교정] 오도메트리를 못 읽어 시간 기반으로 움직였습니다")
        else:
            print(f"[교정] 오도메트리 측정 {got:.1f}mm")
            print("       자로 잰 값과 다르면 오도메트리 배율이 틀린 것입니다")
        node.destroy_node(); rclpy.shutdown(); return

    def pick(obs):
        cands = [o for o in obs if o.cls == args.cls] if args.cls else obs
        return max(cands, key=lambda o: o.h) if cands else None   # 가장 큰(가까운) 것

    # ── 교시 모드 ────────────────────────────────────────────────────────
    if args.teach:
        print(f"[교시] {args.frames}프레임 관측 중…")
        hit = pick(node.collect(args.frames))
        if hit is None and args.search:
            hit = node.search(pick, args.frames, args.search_speed,
                              args.burst, args.search_tries)
        if hit is None:
            print(f"[교시] '{args.cls}' 를 못 찾았습니다.", file=sys.stderr)
            node.destroy_node(); rclpy.shutdown(); sys.exit(1)
        d = save_target(args.cls, hit, args.note)
        print(f"[교시] 저장 — x={d['x']} 박스 {d['w']}×{d['h']}  (신뢰 {hit.conf:.2f}, 산포 {hit.spread:.1f}px)")
        print(f"        {target_file(args.cls)}")
        node.destroy_node(); rclpy.shutdown(); return

    # ── 접근 모드 ────────────────────────────────────────────────────────
    tgt = load_target(args.cls)
    if tgt is None:
        print(f"'{args.cls}' 기준값이 없습니다 ({target_file(args.cls)}). "
              f"먼저 목표 위치에서 --teach 를 실행하세요.", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)
    # 교시값을 그대로 쓰지 않고 보정을 얹는다. 교시는 사람이 물체를 손가락 사이에
    # 놓아 만드는데, 손가락은 깊이 방향으로도 폭이 있어 어디에 놓느냐로 몇 mm 씩
    # 어긋난다. 다시 교시하는 것보다 이 값을 조정하는 쪽이 빠르다.
    tgt = dict(tgt)
    tgt["x"] = tgt["x"] + args.offset_x
    tgt["h"] = tgt["h"] + args.offset_h
    print(f"목표 — x={tgt['x']:.1f} 박스높이={tgt['h']:.1f}  (클래스 {tgt['class']})")
    if args.offset_x or args.offset_h:
        print(f"       보정 적용 — x{args.offset_x:+.1f}px, 높이{args.offset_h:+.1f}px "
              f"(높이 1px ≈ 2mm, 양수면 더 가까이)")
    print(f"허용 오차 — x ±{args.tol_x}px, 높이 ±{args.tol_h}px\n")

    sign_y = -1.0 if args.invert_y else 1.0
    # 종료 코드로 결과를 알린다 — 사이클 스크립트가 이걸 보고 파지로 넘어간다.
    #   0 수렴 · 2 미수렴 · 3 물체 놓침 · 130 중단
    rc = 2
    last_err = None
    try:
        for it in range(1, args.max_iter + 1):
            # 멀 때는 적은 프레임으로 빠르게, 가까워지면 정밀하게.
            # 접근 초반의 정밀도는 어차피 다음 회차에 다시 측정된다.
            n = args.frames
            if args.frames_far and it > 1 and last_err is not None:
                if last_err[0] > 3 * args.tol_x or last_err[1] > 3 * args.tol_h:
                    n = args.frames_far
            node.p["n_frames"] = n
            hit = pick(node.collect(n))
            if hit is None and args.search:
                hit = node.search(pick, args.frames, args.search_speed,
                                  args.burst, args.search_tries)
            if hit is None:
                print(f"  {it:>2}. 목표를 못 찾음 — 중단", file=sys.stderr)
                rc = 3
                break
            ex = hit.x - tgt["x"]          # +면 물체가 목표보다 오른쪽에 보인다
            eh = tgt["h"] - hit.h          # +면 아직 멀다(박스가 작다)
            last_err = (abs(ex), abs(eh))
            done_x, done_h = abs(ex) <= args.tol_x, abs(eh) <= args.tol_h
            mark = "✓" if (done_x and done_h) else " "
            print(f"  {it:>2}.{mark} x={hit.x:>5.0f}(오차{ex:>+6.1f})  "
                  f"높이={hit.h:>5.0f}(오차{eh:>+6.1f})  신뢰 {hit.conf:.2f}")
            if done_x and done_h:
                print("\n수렴 완료")
                rc = 0
                if args.final_push and not args.dry_run:
                    print(f"파지 지점까지 {args.final_push:.0f}mm 전진")
                    got = node.push_forward(args.final_push, args.push_speed, args.push_scale)
                    if got is not None:
                        print(f"  실제 이동 {got:.1f}mm")
                print("파지 자세를 실행해도 좋습니다.")
                break
            if args.dry_run:
                continue

            vx = 0.0 if done_h else max(-args.max_speed, min(args.max_speed, eh * args.gain_h))
            vy = 0.0 if done_x else max(-args.max_speed, min(args.max_speed, -ex * args.gain_x * sign_y))

            # 좌우가 크게 어긋난 채로 전진하면 물체를 지나쳐버린다(실측 실패 사례).
            # 전진을 늦춰 횡보정이 따라잡게 한다.
            if args.align_first > 0 and abs(ex) > args.align_first * args.tol_x:
                vx *= 0.25

            vx, vy, secs = apply_floor(vx, vy, args.min_speed, args.max_speed,
                                       args.burst, args.min_burst)
            if vx == 0.0 and vy == 0.0:
                print("      (지령이 데드밴드 아래 — --min-speed 를 올리세요)",
                      file=sys.stderr)
                continue
            node.nudge(vx, vy, secs)
        else:
            print(f"\n{args.max_iter}회 안에 수렴하지 못했습니다. 게인이나 허용 오차를 조정하세요.",
                  file=sys.stderr)
    except KeyboardInterrupt:
        print("\n중단됨")
        rc = 130
    finally:
        for _ in range(5):
            node.cmd.publish(Twist()); time.sleep(0.02)
        node.destroy_node(); rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
