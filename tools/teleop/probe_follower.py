"""팔로워 암 통신 확인 — 읽기 전용. 서보에 아무것도 쓰지 않는다."""
import sys, time
sys.path.insert(0, "/third_party/soarm_provided_d/soarm_lab")
from driver_sdk import STS3215Driver, JOINT_IDS, JOINT_NAMES

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/soarm"
drv = STS3215Driver(port=port)
if not drv.connect():
    sys.exit("연결 실패")
print(f"연결됨: {port} @ {drv.baudrate}bps\n")

for sid, name in zip(JOINT_IDS, JOINT_NAMES):
    if not drv.ping(sid):
        print(f"  id{sid} {name:<12} 응답 없음")
        continue
    print(f"  id{sid} {name:<12} pos={drv.get_position(sid):<5} "
          f"{drv.get_voltage(sid)}V {drv.get_temperature(sid)}°C "
          f"토크={'ON' if drv.get_torque(sid) else 'off'}")

n = 30
t0 = time.perf_counter()
for _ in range(n):
    drv.get_all_positions()
dt = (time.perf_counter() - t0) / n
print(f"\n6관절 일괄 읽기: {dt*1000:.1f}ms/회 → 최대 {1/dt:.0f}Hz")
drv.disconnect()
