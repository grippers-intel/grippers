#!/usr/bin/env python3
"""각 State 를 터미널에서 지켜보며 로그 파일을 남긴다 (실기 시험용).

## 무엇을 하는가

미션을 평소대로 돌리되, **상태 전이와 Pi 보고가 일어날 때마다** 한 줄씩
찍고 같은 내용을 파일에 남긴다.

    [   12.4s] SEARCH_TARGET → APPROACH_PIECE  목표 rook
    [   14.1s]   ↑ Pi  STATE            [APPROACH]
    [   18.7s] APPROACH_PIECE → GRASP   목표 rook
    [   20.4s]   ↑ Pi  GRASP_BLOCKED    [APPROACH] 뎁스 카메라가 ...
    [   23.4s] · GRASP          cmd=stop  pose=(1.271, 1.000,  90.0°)  13.9Hz

매 사이클을 찍지는 않는다 — 14Hz 를 그대로 찍으면 사람이 못 읽고 정작 중요한
전이가 묻힌다. 아무 사건이 없어도 3초마다 한 줄(`·`)은 나오므로, 화면이
멈춘 것과 조용한 것을 구분할 수 있다.

끝나면 상태별 체류 시간과 Pi 보고 횟수를 표로 정리해 준다.

## 남는 파일 두 벌

    host/logs/monitor_20260829_141230.log     사람이 읽는 것
    host/logs/monitor_20260829_141230.jsonl   사후 분석용 (사이클마다 한 줄)

JSONL 에는 사이클마다 자세(x, y, yaw)가 들어간다. 2026-08-28에 "바퀴가
래치된 명령을 물고 있었다"를 증명한 것이 이 궤적이었는데, 그때는 터미널
스크롤백에서 `\\r` 로 덮어쓴 바이트를 다시 갈라내야 했다.

## 쓰는 법

실기(Pi 연결):

    python3 state_monitor.py --cams 0 1 --show-cams --display 1 \\
      --vehicle-ip 192.168.0.7

하드웨어 없이 화면만 확인:

    python3 state_monitor.py --cams 0 1 --show-cams

`run_mission.py` 의 옵션을 전부 그대로 받는다 — 이 파일은 기록·모니터링
기본값만 켜 주고 나머지는 `run_mission.main()` 에 그대로 넘긴다. 실행 경로를
하나로 두는 것이 목적이다: 시험용으로 따로 복사해 두면 그 사본만 늘 낡는다.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mission_log         # noqa: E402
import run_mission         # noqa: E402


def main() -> int:
    argv = sys.argv[1:]

    # 기록 경로를 여기서 정해 이름에 용도가 드러나게 한다. 사용자가 직접
    # `--log-file` 을 줬으면 그것을 존중한다.
    if not any(a.startswith("--log-file") for a in argv) and "--no-log" not in argv:
        argv += ["--log-file", str(mission_log.default_log_path("monitor"))]

    print(__doc__.split("## 쓰는 법")[0].rstrip())
    print("-" * 60)

    sys.argv = [sys.argv[0]] + argv
    return run_mission.main()


if __name__ == "__main__":
    raise SystemExit(main())
