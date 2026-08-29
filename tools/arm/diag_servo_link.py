"""서보 통신이 끊기는 원인을 구분한다 — 단선인가, 잡음인가, 전원인가.

가만히 있을 때는 1800회 읽기가 전부 성공하는데 **팔을 손으로 움직일 때만**
끊긴다. 원인이 셋 중 하나인데 대응이 전혀 다르다.

    타임아웃 (응답 없음)   -> 물리적 단선 또는 서보 리셋
    체크섬/깨짐            -> 전기적 잡음 (역기전력)
    전압 강하              -> 전원 부족

그래서 실패를 **종류별로** 세고, 전압을 같이 기록한다.

    python diag_servo_link.py COM8 30      # 30초 동안 감시
    (감시하는 동안 팔을 평소처럼 움직일 것)
"""
import sys
import time
from collections import Counter

import scservo_sdk as scs

ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_VOLTAGE = 62
IDS = [1, 2, 3, 4, 5, 6]


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0

    ph = scs.PortHandler(port)
    pk = scs.PacketHandler(0)
    if not ph.openPort():
        print(f"{port} 열기 실패")
        return 1
    ph.setBaudRate(1000000)

    print(f"{secs:.0f}초 동안 감시합니다 — 지금부터 팔을 평소처럼 움직여 주세요.\n")

    kinds = Counter()
    per_id = Counter()
    volts = []
    n = 0
    t0 = time.time()
    while time.time() - t0 < secs:
        for i in IDS:
            n += 1
            _pos, comm, err = pk.read2ByteTxRx(ph, i, ADDR_PRESENT_POSITION)
            if comm != scs.COMM_SUCCESS:
                kinds[pk.getTxRxResult(comm).strip()] += 1
                per_id[i] += 1
            elif err != 0:
                kinds[f"서보 오류비트 {err}"] += 1
                per_id[i] += 1
        v, comm, _ = pk.read1ByteTxRx(ph, 1, ADDR_PRESENT_VOLTAGE)
        if comm == scs.COMM_SUCCESS and v:
            volts.append(v / 10.0)

    ph.closePort()

    fails = sum(kinds.values())
    print(f"총 {n}회 읽기 · 실패 {fails}회 ({fails / n * 100:.2f}%)\n")

    if kinds:
        print("실패 종류:")
        for k, c in kinds.most_common():
            print(f"  {c:5d}회  {k}")
        print("\n서보별:")
        for i in IDS:
            if per_id[i]:
                print(f"  id {i}: {per_id[i]}회")
    else:
        print("실패 없음 — 움직이는 동안에도 통신이 멀쩡했습니다.")

    if volts:
        print(f"\n전압: 최소 {min(volts):.1f}V · 최대 {max(volts):.1f}V · "
              f"평균 {sum(volts) / len(volts):.1f}V  (표본 {len(volts)})")
        if min(volts) < 6.0:
            print("  ⚠️ 6V 아래로 떨어졌습니다 — 전원 부족을 의심할 것")
    return 0


if __name__ == "__main__":
    sys.exit(main())
