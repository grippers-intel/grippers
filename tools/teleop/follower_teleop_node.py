#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""파이 쪽 텔레옵 수신기 — 노트북이 보낸 패킷으로 팔과 베이스를 함께 움직인다.

  팔    : 리더 관절값 → /dev/soarm 의 SO-101 팔로워 서보
  베이스: 키보드 방향 → /cmd_vel (컨트롤러가 모터로 변환)

파이(컨테이너) 안에서 돌린다. /dev/soarm 을 직접 여는 프로그램이라
arm_driver_node 와 동시에 뜰 수 없다(하나의 시리얼 버스를 두 프로세스가
못 나눠 쓴다). bringup 은 use_fake_arm:=true 로 띄울 것.

**델타(상대) 추종**을 쓴다. 두 팔 모두 calibration.json 이 없어 절대 카운트의
물리적 의미가 서로 다르기 때문이다. 추종을 켜는 순간의 리더/팔로워 자세를
각각 기준점으로 잡고 그 뒤로는 리더의 *변화량*만 더한다. 보정 없이 동작하고,
켜는 순간 팔이 튀지 않는다는 게 더 중요한 이점이다 — 절대 추종이었다면
리더와 팔로워 자세가 다른 상태에서 켜는 즉시 팔이 최대 속도로 날아간다.
"""
from __future__ import annotations

import argparse
import socket
import sys
import time

sys.path.insert(0, "/third_party/soarm_provided_d/soarm_lab")
sys.path.insert(0, "/grippers/tools/teleop")

from driver_sdk import (  # noqa: E402
    JOINT_IDS, JOINT_LIMITS, POS_RANGE,
    STS3215Driver, unwrap_position, wrap_position,
)
from teleop_protocol import DEFAULT_PORT, decode, wrap_delta  # noqa: E402

# 기준점에서 이만큼 넘게 벗어난 목표는 통신 오류로 본다. 반 바퀴 이상은
# 사람이 리더를 들고 만들 수 있는 자세 변화가 아니다.
MAX_DELTA_COUNTS = 1400


def clamp_to_limits(sid: int, raw: int) -> int:
    """관절의 보정 창(calibration window) 안으로 목표를 가둔다.

    미보정 관절(1~5)은 창이 0..4095라 사실상 통과다 — 이때 실제 한계는
    리더 암의 기구적 스토퍼가 대신 잡아준다(같은 설계라 가동범위가 같다)."""
    lim = JOINT_LIMITS.get(sid, {})
    lo, hi = lim.get("min", 0), lim.get("max", POS_RANGE - 1)
    p = unwrap_position(int(raw) % POS_RANGE, lim)
    return wrap_position(max(lo, min(hi, p)))


class FollowerTeleop:
    def __init__(self, args):
        self.args = args
        self.drv = STS3215Driver(port=args.arm_port)
        self.epoch = None            # 현재 latch된 engage 세대
        self.leader_ref: dict = {}   # 켠 순간의 리더 카운트
        self.follower_ref: dict = {}  # 켠 순간의 팔로워 카운트
        self.last_target: dict = {}  # 슬루 제한 기준
        self.last_rx = 0.0
        self.tracking = False
        self.ros = None

    # ── 시작/종료 ────────────────────────────────────────────────────────────
    def connect(self) -> bool:
        if not self.drv.connect():
            print(f"[팔로워] {self.args.arm_port} 열기 실패", file=sys.stderr)
            return False
        alive = [sid for sid in JOINT_IDS if self.drv.ping(sid)]
        if len(alive) < len(JOINT_IDS):
            dead = [s for s in JOINT_IDS if s not in alive]
            print(f"[팔로워] 응답 없는 서보 id={dead}", file=sys.stderr)
            if not alive:
                print("[팔로워] 버스에 살아있는 서보가 하나도 없습니다.\n"
                      "         USB는 보드 로직만 먹입니다 — 암의 서보 전원\n"
                      "         라인이 연결됐는지, 스위치가 켜졌는지 확인하세요.",
                      file=sys.stderr)
            return False
        print(f"[팔로워] {self.args.arm_port} 연결, 서보 6개 정상")
        return True

    def shutdown(self):
        if self.ros:
            self.ros.stop_base()      # 무엇보다 먼저 — 로봇을 세운다
        if self.args.relax_on_exit:
            print("[팔로워] 토크 해제 — 팔이 중력으로 내려옵니다")
            self.drv.set_all_torque(False)
        else:
            print("[팔로워] 토크 유지(현재 자세 고정). 내리려면 --relax-on-exit")
        self.drv.disconnect()
        if self.ros:
            self.ros.destroy()

    # ── 기준점 latch ─────────────────────────────────────────────────────────
    def latch(self, msg) -> bool:
        """새 epoch를 만나면 리더/팔로워 양쪽의 현재 자세를 기준으로 잡는다."""
        cur = self.drv.get_all_positions()
        if any(cur.get(sid) is None for sid in JOINT_IDS):
            print("[팔로워] 기준점 읽기 실패 — 이번 engage는 무시", file=sys.stderr)
            return False
        if any(msg["pos"][i] is None for i in range(len(JOINT_IDS))):
            print("[팔로워] 리더 기준점에 결측 — 이번 engage는 무시", file=sys.stderr)
            return False

        self.follower_ref = {sid: int(cur[sid]) for sid in JOINT_IDS}
        self.leader_ref = {sid: int(msg["pos"][i]) for i, sid in enumerate(JOINT_IDS)}
        self.last_target = dict(self.follower_ref)
        self.drv.set_all_torque(True)
        self.epoch = msg["epoch"]
        self.tracking = True
        print(f"[팔로워] 팔 추종 시작 #{self.epoch} — 기준점 latch")
        return True

    # ── 한 패킷 처리 ─────────────────────────────────────────────────────────
    def on_packet(self, msg):
        self.last_rx = time.monotonic()

        # 베이스는 팔 추종 여부와 무관하게 항상 반영한다 — 둘은 독립이다.
        if self.ros:
            self.ros.publish_base(msg["base"], msg["sc"])

        if not msg["en"]:
            if self.tracking:
                print(f"[팔로워] 팔 추종 정지 #{self.epoch} — 현재 자세로 고정")
                self.tracking = False
            return
        if msg["epoch"] != self.epoch:
            self.latch(msg)
            return
        if not self.tracking:
            return

        targets = {}
        for i, sid in enumerate(JOINT_IDS):
            lp = msg["pos"][i]
            if lp is None:                       # 이 관절만 읽기 실패 → 유지
                continue
            delta = wrap_delta(int(lp), self.leader_ref[sid]) * self.args.gain
            if abs(delta) > MAX_DELTA_COUNTS:    # 통신 오류로 간주, 무시
                continue
            want = clamp_to_limits(sid, self.follower_ref[sid] + int(delta))

            # 슬루 제한 — 패킷이 몇 개 유실된 뒤 큰 점프가 들어와도
            # 한 틱에 낼 수 있는 이동량을 넘지 않게 한다.
            prev = self.last_target[sid]
            step = wrap_delta(want, prev)
            if abs(step) > self.args.slew:
                step = self.args.slew if step > 0 else -self.args.slew
            targets[sid] = wrap_position(prev + step)

        for sid, pos in targets.items():
            self.drv.set_position(sid, pos)
            self.last_target[sid] = pos

        if self.ros:
            self.ros.publish_arm(msg["pos"], self.last_target, True)

    # ── 데드맨 ───────────────────────────────────────────────────────────────
    def on_signal_lost(self):
        """리더가 조용해졌다. 베이스는 **반드시** 세우고, 팔은 추종만 멈춘다.

        팔의 토크는 켠 채로 둬서 그 자리에 서 있게 한다. 여기서 토크를 끄면
        들고 있던 물건과 함께 팔이 그대로 떨어진다. 베이스는 정반대다 —
        속도 명령을 유지하면 로봇이 계속 굴러가므로 즉시 0을 내야 한다."""
        if self.ros:
            self.ros.stop_base()
        self.tracking = False

    # ── 메인 루프 ────────────────────────────────────────────────────────────
    def run(self):
        sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
        sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)  # IPv4도 수신
        sock.bind(("::", self.args.udp_port))
        sock.settimeout(0.1)
        print(f"[팔로워] UDP {self.args.udp_port} 대기 중 (IPv4/IPv6 동시)")
        print("[팔로워] 준비 완료 — 노트북에서 조종하세요")

        warned = False
        live = False
        while True:
            try:
                raw, _ = sock.recvfrom(4096)
            except socket.timeout:
                raw = None
            except KeyboardInterrupt:
                break

            if raw is not None:
                msg = decode(raw)
                if msg:
                    self.on_packet(msg)
                    warned, live = False, True
                continue

            if live and time.monotonic() - self.last_rx > self.args.deadman:
                if not warned:
                    print(f"[팔로워] 리더 신호 끊김 {self.args.deadman}s "
                          f"— 베이스 정지, 팔 자세 유지", file=sys.stderr)
                    warned = True
                self.on_signal_lost()


def main():
    ap = argparse.ArgumentParser(description="파이 쪽 텔레옵 수신기 (팔 + 베이스)")
    ap.add_argument("--arm-port", default="/dev/soarm")
    ap.add_argument("--udp-port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--gain", type=float, default=1.0,
                    help="리더 변화량 대비 팔로워 변화량 배율")
    ap.add_argument("--slew", type=int, default=80,
                    help="한 패킷당 관절 최대 이동 카운트 (50Hz에서 80 ≈ 350°/s)")
    ap.add_argument("--deadman", type=float, default=0.4,
                    help="이 시간(초) 동안 패킷이 없으면 베이스 정지·팔 추종 해제")
    ap.add_argument("--relax-on-exit", action="store_true",
                    help="종료할 때 토크를 끈다(팔이 내려옴). 기본은 자세 유지")
    ap.add_argument("--no-ros", action="store_true",
                    help="ROS 없이 팔만 구동(벤치 테스트용). 베이스는 동작하지 않는다")
    args = ap.parse_args()

    node = FollowerTeleop(args)
    if not node.connect():
        sys.exit(1)
    if not args.no_ros:
        from teleop_ros_bridge import RosBridge
        node.ros = RosBridge()
    else:
        print("[팔로워] --no-ros: 베이스 제어 없음, 팔만 동작합니다")
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == "__main__":
    main()
