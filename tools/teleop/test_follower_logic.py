"""follower_teleop_node 의 델타/안전 로직 검증 — 실물 서보 없이 돈다.

가짜 드라이버를 끼워 넣고 set_position 호출을 기록한 뒤, 목표값이 기대와
맞는지 본다. 이 코드는 실제 서보를 움직이므로 하드웨어가 준비되기 전에
계산부터 맞춰 둔다.
"""
import sys, types, time
sys.path.insert(0, "/third_party/soarm_provided_d/soarm_lab")
sys.path.insert(0, "/grippers/tools/teleop")

import follower_teleop_node as F
from teleop_protocol import encode, decode

FOLLOWER_START = {1: 2000, 2: 2000, 3: 2000, 4: 2000, 5: 2000, 6: 1500}
LEADER_START = [1000, 1000, 1000, 1000, 1000, 1500]


class FakeDriver:
    def __init__(self, *a, **k):
        self.pos = dict(FOLLOWER_START)
        self.written = []
        self.torque = None
    def connect(self): return True
    def disconnect(self): pass
    def ping(self, sid): return True
    def get_all_positions(self): return dict(self.pos)
    def set_all_torque(self, en): self.torque = en
    def set_position(self, sid, p):
        self.pos[sid] = p
        self.written.append((sid, p))


class FakeRos:
    """베이스 발행을 기록만 하는 가짜 ROS 브리지."""
    def __init__(self): self.base = []; self.stops = 0; self.arm = 0
    def publish_base(self, vec, sc): self.base.append((tuple(vec), sc))
    def stop_base(self): self.stops += 1
    def publish_arm(self, *a): self.arm += 1
    def destroy(self): pass


class Args:
    arm_port = "/dev/null"; udp_port = 0; gain = 1.0
    slew = 80; deadman = 0.4; relax_on_exit = False; no_ros = True


def make_node():
    F.STS3215Driver = FakeDriver
    n = F.FollowerTeleop(Args())
    n.drv = FakeDriver()
    return n


def pkt(epoch, en, pos, base=(0,0,0), sc=0.0):
    return decode(encode(1, epoch, en, pos, base, sc))

fails = []
def check(name, got, want):
    ok = got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: got={got} want={want}")
    if not ok: fails.append(name)

print("1) engage 시 기준점 latch, 팔이 움직이지 않아야 한다")
n = make_node()
n.on_packet(pkt(1, True, LEADER_START))
check("torque on", n.drv.torque, True)
check("추종 시작", n.tracking, True)
check("기준점 latch 직후 이동 없음", n.drv.written, [])

print("\n2) 리더가 +50 움직이면 팔로워도 +50 (델타 추종)")
n.drv.written.clear()
moved = [1050, 1000, 1000, 1000, 1000, 1500]
n.on_packet(pkt(1, True, moved))
check("id1 목표", dict(n.drv.written).get(1), 2050)

print("\n3) 슬루 제한 — 리더가 한 번에 +500 뛰어도 80카운트만")
n.drv.written.clear()
n.on_packet(pkt(1, True, [1550, 1000, 1000, 1000, 1000, 1500]))
check("id1 목표(2050+80)", dict(n.drv.written).get(1), 2130)

print("\n4) 델타 상한 — 기준점에서 1400 초과는 통신 오류로 보고 무시")
n.drv.written.clear()
n.on_packet(pkt(1, True, [1000 + 1500, 1000, 1000, 1000, 1000, 1500]))
check("id1 명령 없음", [w for w in n.drv.written if w[0] == 1], [])

print("\n5) 카운트 랩어라운드 — 4090 → 5 는 +11 이지 +4091 이 아니다")
n2 = make_node()
n2.on_packet(pkt(1, True, [4090, 1000, 1000, 1000, 1000, 1500]))
n2.drv.written.clear()
n2.on_packet(pkt(1, True, [5, 1000, 1000, 1000, 1000, 1500]))
check("id1 목표(2000+11)", dict(n2.drv.written).get(1), 2011)

print("\n6) 그리퍼 관절 한계 클램프 (JOINT_LIMITS[6] = 984..2318)")
n3 = make_node()
n3.on_packet(pkt(1, True, LEADER_START))
n3.drv.written.clear()
for _ in range(30):   # 슬루 때문에 여러 틱에 걸쳐 밀어붙인다
    n3.on_packet(pkt(1, True, [1000, 1000, 1000, 1000, 1000, 1500 + 1300]))
check("id6 상한에서 멈춤", n3.last_target[6], 2318)

print("\n7) disengage 하면 추종만 멈추고 토크는 유지")
n3.drv.torque = None
n3.on_packet(pkt(1, False, LEADER_START))
check("추종 정지", n3.tracking, False)
check("토크 건드리지 않음", n3.drv.torque, None)

print("\n8) 재-engage 는 새 epoch 로 기준점을 다시 잡는다")
n3.drv.written.clear()
n3.on_packet(pkt(2, True, [3000, 1000, 1000, 1000, 1000, 1500]))
check("새 epoch latch", n3.epoch, 2)
check("재latch 직후 이동 없음", n3.drv.written, [])

print("\n9) 관절 하나만 읽기 실패(None)면 그 관절만 유지")
n4 = make_node()
n4.on_packet(pkt(1, True, LEADER_START))
n4.drv.written.clear()
n4.on_packet(pkt(1, True, [None, 1050, 1000, 1000, 1000, 1500]))
check("id1 명령 없음", [w for w in n4.drv.written if w[0] == 1], [])
check("id2 는 정상 반영", dict(n4.drv.written).get(2), 2050)

print("\n10) 베이스 명령은 팔 추종 여부와 무관하게 항상 반영된다")
n5 = make_node()
n5.ros = FakeRos()
n5.on_packet(pkt(0, False, LEADER_START, (1, 0, 0), 0.6))   # 팔 대기 상태
check("팔 대기여도 베이스 발행", n5.ros.base[-1], ((1.0, 0.0, 0.0), 0.6))

print("\n11) 데드맨 — 신호가 끊기면 베이스를 세우고 팔 추종만 해제한다")
n5.on_packet(pkt(1, True, LEADER_START, (1, 0, 0), 0.6))
check("추종 중", n5.tracking, True)
before = n5.ros.stops
n5.on_signal_lost()
check("베이스 정지 발행", n5.ros.stops > before, True)
check("팔 추종 해제", n5.tracking, False)
check("팔 토크는 유지", n5.drv.torque, True)

print("\n12) 종료 시 베이스 정지가 먼저 나간다")
n6 = make_node()
n6.ros = FakeRos()
n6.shutdown()
check("정지 발행됨", n6.ros.stops >= 1, True)

print("\n" + ("모든 검증 통과" if not fails else f"실패: {fails}"))
sys.exit(1 if fails else 0)
