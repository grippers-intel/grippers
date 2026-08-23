"""서보 ID 전체 스캔 — 읽기 전용(PING만)."""
import sys
sys.path.insert(0, "/third_party/soarm_provided_d/soarm_lab")
from driver_sdk import STS3215Driver
port = sys.argv[1] if len(sys.argv) > 1 else "/dev/soarm"
drv = STS3215Driver(port=port, timeout=0.01)
if not drv.connect():
    sys.exit("연결 실패")
found = [sid for sid in range(0, 21) if drv.ping(sid)]
print(f"{port}: 응답한 ID = {found if found else '없음 (버스에 살아있는 서보 0개)'}")
drv.disconnect()
