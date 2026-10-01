"""차체 컨트롤러 자동 복구 — ROS 없이 테스트할 수 있게 떼어 둔다.

## 무엇을 고치나

보드가 Pi 의 쓰기를 **오류 없이 무시**하는 고장이 있다(2026-09-23 · 09-30). 보드 -> Pi
텔레메트리(IMU·배터리)는 계속 들어오고 쓰기도 "성공"하므로 벤더 노드의 재연결·워치독이
반응하지 않는다. 컨트롤러 노드를 다시 띄우면(포트를 다시 열 때 DTR 이 토글돼 보드가 리셋된다)
몇 초 만에 돌아온다 — tools/ops/rrc_recover.sh --fix.

지금까지는 사람이 "바퀴가 안 돈다"를 알아채고 부저로 진단한 뒤 그 스크립트를 돌렸다.
이제 Host 가 탑뷰로 "움직이라고 했는데 안 움직인다"를 보고 HostCommand.recover_base 로
요청하면 여기서 스크립트를 돌린다. 도는 동안 pi_mission_node 는 정지만 낸다.
"""
from __future__ import annotations

import subprocess
import threading
import time
from typing import Callable, Optional

Runner = Callable[[list, float], tuple[int, str]]


def run_script(cmd: list, timeout_s: float) -> tuple[int, str]:
    """스크립트를 돌려 (종료 코드, 마지막 출력) 을 돌려준다. 시간 초과는 -1."""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
        out = (p.stdout + p.stderr).strip().splitlines()
        return p.returncode, " / ".join(out[-3:])
    except subprocess.TimeoutExpired:
        return -1, f"{timeout_s:.0f}s 안에 끝나지 않았다"
    except OSError as exc:
        return -1, f"실행 실패: {exc}"


class BaseRecovery:
    def __init__(self, script: str, timeout_s: float, cooldown_s: float,
                 runner: Runner = run_script, clock: Callable[[], float] = time.monotonic,
                 log: Callable[[str], None] = print, background: bool = True) -> None:
        self.script = script
        self.timeout_s = timeout_s
        self.cooldown_s = cooldown_s
        self._runner = runner
        self._clock = clock
        self._log = log
        self._background = background
        self._lock = threading.Lock()
        self._recovering = False
        self._count = 0
        self._last_end: Optional[float] = None
        self.last_detail = ""

    @property
    def recovering(self) -> bool:
        with self._lock:
            return self._recovering

    @property
    def count(self) -> int:
        """끝낸 복구 횟수. Host 는 이 값이 늘어난 것을 보고 끝났음을 안다."""
        with self._lock:
            return self._count

    def request(self, startup: bool = False) -> bool:
        """복구를 시작한다. 이미 도는 중이거나 방금 끝났으면(쿨다운) 무시하고 False.

        startup=True 는 스택 기동 직후의 **선제 재기동**이다. 2026-10-01: 로봇 프로그램을 새로 띄울
        때마다 첫 주행이 무응답이었고(완충·부저 정상이어도), 컨트롤러를 한 번 다시 띄우면 그 뒤로는
        정상이었다. 그래서 기동할 때 한 번 미리 한다.
        """
        now = self._clock()
        with self._lock:
            if self._recovering:
                return False
            if self._last_end is not None and now - self._last_end < self.cooldown_s:
                return False
            self._recovering = True
        if startup:
            self._log("차체 컨트롤러 선제 재기동 — 스택 기동 직후 첫 주행 무응답을 막는다")
        else:
            self._log("차체 컨트롤러 자동 복구 시작 — Host 가 '명령했는데 안 움직인다'를 봤다")
        mode = "auto-startup" if startup else "auto"
        if self._background:
            threading.Thread(target=self._run, args=(mode,), name="base-recovery", daemon=True).start()
        else:
            self._run(mode)
        return True

    def _run(self, mode: str = "auto") -> None:
        code, tail = self._runner(["bash", self.script, "--fix", mode], self.timeout_s)
        with self._lock:
            self._recovering = False
            self._count += 1
            self._last_end = self._clock()
            self.last_detail = f"복구 {self._count}회차 종료 코드 {code}: {tail}"
        self._log(self.last_detail)
