#!/usr/bin/env python3
"""Enter 를 칠 때마다 다음 State 로 넘어간다 (실기 단계별 시험용).

## 무엇을 하는가

미션 FSM 을 **수동 모드**로 돌린다. 조건이 충족돼도 스스로 다음 단계로 안
넘어가고, 사람이 Enter 를 칠 때까지 그 상태에 머문다.

    Enter   조건이 충족돼 있으면 다음 단계로
    b       한 단계 되돌리기
    q       정지하고 종료

조건이 아직 안 됐는데 Enter 를 치면 넘어가지 않는다 — 넘어갈 수 있는지는
매 사이클 갱신되고, 터미널의 `ready=True/False` 와 LiveMap 의 표시등에
같이 나온다. 즉 Enter 는 "지금 넘어가라"가 아니라 **"넘어가도 된다면
넘어가라"**다. 아무 때나 눌러도 안전하다.

## 왜 필요한가

2026-08-28 실기에서 실패가 어느 단계에서 나는지 가리는 데 시간을 다 썼다.
한 실행이 SEARCH → APPROACH → GRASP 를 통째로 지나가 버리니, 문제가 보이는
시점에는 이미 원인 구간을 지난 뒤였다. 단계마다 세워 두면 그 자리에서
로봇과 화면을 같이 보면서 확인할 수 있다.

되돌리기(`b`)가 있는 이유도 같다. GRASP 에서 뭔가 이상하면 APPROACH 로
되돌려 다시 세워 보는 것이, 전체를 처음부터 다시 도는 것보다 훨씬 빠르다.

## 안전에 대해

수동 모드에서 Enter 는 **정지가 아니다.** 평소 `run_mission.py` 에서 Enter 로
멈추던 것과 의미가 다르므로, 이 도구에서는 정지가 `q` 로 옮겨 갔다.

그리고 그 어느 것도 진짜 비상정지가 아니다. **차체 전원 스위치가 진짜
비상정지다** — 2026-08-28에 소프트웨어 정지를 836회 보내고도 차가 안 섰다.
구동계가 명령을 못 받는 상태가 되면 터미널에 🚨 경보가 뜬다.

## 쓰는 법

    python3 manual_step.py --cams 0 1 --show-cams --display 1 \\
      --vehicle-ip 192.168.0.7

기록은 자동으로 남는다(`host/logs/manual_*.log` 와 `.jsonl`).
`run_mission.py` 의 옵션을 전부 그대로 받는다.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mission_log         # noqa: E402
import run_mission         # noqa: E402


def main() -> int:
    argv = sys.argv[1:]

    if "--manual" not in argv:
        argv = ["--manual"] + argv
    if not any(a.startswith("--log-file") for a in argv) and "--no-log" not in argv:
        argv += ["--log-file", str(mission_log.default_log_path("manual"))]

    print(__doc__.split("## 쓰는 법")[0].rstrip())
    print("-" * 60)

    sys.argv = [sys.argv[0]] + argv
    return run_mission.main()


if __name__ == "__main__":
    raise SystemExit(main())
