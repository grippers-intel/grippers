"""전송로 검증용 수신기 — 서보를 건드리지 않고 패킷 통계만 낸다."""
import argparse, socket, sys, time
sys.path.insert(0, "/grippers/tools/teleop")
from teleop_protocol import DEFAULT_PORT, decode

ap = argparse.ArgumentParser()
ap.add_argument("--udp-port", type=int, default=DEFAULT_PORT)
ap.add_argument("--duration", type=float, default=20.0)
a = ap.parse_args()

s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
s.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
s.bind(("::", a.udp_port))
s.settimeout(1.0)
print(f"UDP {a.udp_port} 대기…", flush=True)

n = bad = 0
first_seq = last_seq = None
t_first = None
gaps = []
last_t = None
peer = None
while True:
    if t_first and time.monotonic() - t_first > a.duration:
        break
    try:
        raw, addr = s.recvfrom(4096)
    except socket.timeout:
        if t_first:
            break
        continue
    msg = decode(raw)
    if not msg:
        bad += 1
        continue
    now = time.monotonic()
    if t_first is None:
        t_first, first_seq, peer = now, msg["seq"], addr[0]
    else:
        gaps.append(now - last_t)
    last_t, last_seq = now, msg["seq"]
    n += 1

if not n:
    sys.exit("패킷을 하나도 못 받았습니다")
span = last_t - t_first
expected = last_seq - first_seq + 1
gaps.sort()
print(f"""
송신자      : {peer}
수신 패킷   : {n} / 기대 {expected}  (유실 {expected-n}, {100*(expected-n)/expected:.2f}%)
파손 패킷   : {bad}
실측 주파수 : {n/span:.1f} Hz  ({span:.1f}초 동안)
패킷 간격   : 중앙값 {gaps[len(gaps)//2]*1000:.1f}ms  최대 {gaps[-1]*1000:.1f}ms  (지터)""", flush=True)
