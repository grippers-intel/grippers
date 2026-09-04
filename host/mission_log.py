"""미션 상태를 터미널에 보여주고 파일로 남긴다 (2026-08-29 실기 준비).

## 왜 필요한가

2026-08-28 실기의 가장 큰 손실은 **기록이 없었다는 것**이다. 여섯 번 돌렸고
GRASP 가 한 번 성공했는데, 남은 것은 터미널 스크롤백뿐이었다. Pi 쪽
`orch.log` 는 다음 기동 때 덮어써졌고, Host 쪽은 `\r` 로 덮어쓴 출력이라
사후에 자세 궤적을 뽑으려면 원본 바이트를 다시 갈라야 했다.

그래서 여기서 만드는 기록은 두 벌이다.

  * **JSONL** — 사이클마다 한 줄. 사후 분석용이고, 사람이 읽기 좋게
    꾸미지 않는다. 2026-08-28에 자세 궤적으로 "바퀴가 래치된 명령을 물고
    있었다"를 증명했는데, 그때 필요했던 것이 정확히 이 형식이다.
  * **텍스트 로그** — 터미널에 찍히는 것과 같은 줄. 사람이 나중에 읽는다.

## 무엇을 터미널에 찍는가

매 사이클을 찍지 않는다. 14Hz 로 도는 루프를 그대로 찍으면 초당 14줄이라
사람이 못 읽고, 정작 중요한 전이가 묻힌다. **사건이 있을 때만** 찍는다.

    상태 전이 · Pi 보고가 바뀔 때 · 구동계 경보 · 주기 하트비트

하트비트를 두는 이유: 아무 사건도 없는 구간에서 화면이 완전히 멈추면
사람은 프로그램이 죽었는지 조용한 건지 구분할 수 없다.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, TextIO

# 사건이 없어도 이 간격마다 한 줄은 찍는다. 사람이 "살아 있음"을 확인하는 데
# 쓰이므로 너무 길면 안 되고, 너무 짧으면 그것만으로 화면이 찬다.
HEARTBEAT_SEC = 3.0


def default_log_path(prefix: str = "mission") -> Path:
    """`logs/mission_20260829_141230.log` 형태의 경로.

    실행할 때마다 새 파일이다 — 덮어쓰면 2026-08-28에 Pi 로그를 잃은 것과
    같은 일이 Host 에서도 난다."""
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return Path(__file__).resolve().parent / "logs" / f"{prefix}_{stamp}.log"


def append_annotation(path: Optional[Path], text: str) -> None:
    """실행이 끝난 뒤 사람이 남긴 코멘트를 로그 끝에 덧붙인다 (2026-09-04).

    `summary()`는 "GRASP_FAILED 1회 · GRASP_DONE 1회"처럼 사건의 종류와
    횟수만 남긴다 — 그게 재시도 끝에 성공한 것인지, 첫 시도가 왜 실패했는지는
    그 자리에 있던 사람만 안다. 그 맥락을 로그 파일에 같이 묻어 두면 사후
    분석 때(§ `run_mission.py` "실행 후 코멘트") 요약표만으로는 안 보이는
    사정이 드러난다.

    `MissionLogger.close()` 로 이미 닫힌 파일을 다시 여는 것을 전제로
    `"a"`(추가) 모드를 쓴다 — 로거 인스턴스를 코멘트 하나 받으려고 계속
    살려 둘 필요가 없다. `path`가 `None`(`--no-log`)이거나 빈 문자열이면
    아무것도 하지 않는다 — 남길 파일 자체가 없다."""
    if path is None or not text.strip():
        return
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    note = text.strip()
    with path.open("a", encoding="utf-8") as fh:
        fh.write(f"\n# 사용자 코멘트 ({stamp})\n# {note}\n")
    with path.with_suffix(".jsonl").open("a", encoding="utf-8") as fh:
        fh.write(json.dumps({"annotation": note, "t": stamp}, ensure_ascii=False) + "\n")


@dataclass
class _StateSpan:
    """한 상태에 머무른 구간."""

    name: str
    entered_at: float
    left_at: Optional[float] = None

    @property
    def seconds(self) -> float:
        return (self.left_at if self.left_at is not None
                else time.monotonic()) - self.entered_at


@dataclass
class MissionLogger:
    """미션 한 실행의 기록.

    `record()` 를 사이클마다 부르면 나머지는 알아서 한다 — 무엇이 사건인지
    판단하는 것이 이 클래스의 일이고, 호출부는 지금 값만 넘기면 된다."""

    path: Optional[Path] = None
    echo: bool = True                 # 터미널에도 찍을 것인가
    heartbeat_sec: float = HEARTBEAT_SEC

    _text: Optional[TextIO] = None
    _jsonl: Optional[TextIO] = None
    _t0: float = field(default_factory=time.monotonic)
    _cycles: int = 0
    _last_state: Optional[str] = None
    _last_report: Optional[tuple] = None
    _last_alarm: Optional[str] = None
    _last_beat: float = 0.0
    _spans: list = field(default_factory=list)
    _events: int = 0
    _report_counts: dict = field(default_factory=dict)
    _alarms: list = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.path is None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._text = self.path.open("w", encoding="utf-8")
        self._jsonl = self.path.with_suffix(".jsonl").open("w", encoding="utf-8")
        self._say(f"# 기록 시작 {time.strftime('%Y-%m-%d %H:%M:%S')}")
        self._say(f"# 텍스트 {self.path}")
        self._say(f"# JSONL  {self.path.with_suffix('.jsonl')}")

    # --- 기록 --------------------------------------------------------------

    def record(self, *, state: str, pose, cmd: Optional[str],
               target: Optional[str] = None, report=None,
               base_alarm: Optional[str] = None,
               ready: Optional[bool] = None,
               hz: Optional[float] = None) -> None:
        """한 사이클. 사건이 있으면 터미널에도 찍는다."""
        now = time.monotonic()
        self._cycles += 1

        self._write_jsonl(now, state, pose, cmd, target, report,
                          base_alarm, ready, hz)

        # ① 구동계 경보가 가장 급하다 — 먼저 찍는다.
        if base_alarm and base_alarm != self._last_alarm:
            self._last_alarm = base_alarm
            self._alarms.append((now - self._t0, base_alarm))
            # Pi 가 보내는 문장이 이미 "구동계 이상 (...)"으로 시작한다 —
            # 여기서 또 붙이면 "구동계 구동계"가 된다.
            self._event(now, f"🚨 {base_alarm}")

        # ② 상태 전이.
        if state != self._last_state:
            if self._spans:
                self._spans[-1].left_at = now
            self._spans.append(_StateSpan(state, now))
            arrow = ("시작" if self._last_state is None
                     else f"{self._last_state} → {state}")
            extra = f"  목표 {target}" if target else ""
            self._event(now, f"{arrow}{extra}")
            self._last_state = state

        # ③ Pi 보고가 바뀌었을 때. 같은 보고가 반복되는 것은 사건이 아니다 —
        #    워치독 거부는 초당 여러 번 나온다.
        if report is not None and report != self._last_report:
            self._last_report = report
            kind = report[0] if isinstance(report, (tuple, list)) else str(report)
            self._report_counts[kind] = self._report_counts.get(kind, 0) + 1
            self._event(now, f"  ↑ Pi  {self._format_report(report)}")

        # ④ 하트비트.
        if now - self._last_beat >= self.heartbeat_sec:
            self._last_beat = now
            self._say(f"[{now - self._t0:7.1f}s] · {state:<14} "
                      f"cmd={cmd or '-':<5} {self._format_pose(pose)}"
                      + (f"  {hz:.1f}Hz" if hz else ""))

    # --- 마무리 ------------------------------------------------------------

    def summary(self) -> str:
        """실행이 끝난 뒤 한눈에 보는 표.

        2026-08-28에 "어느 상태에서 얼마나 머물렀나"를 로그에서 손으로 세야
        했다. 그 계산은 기계가 하는 편이 맞다."""
        now = time.monotonic()
        if self._spans:
            self._spans[-1].left_at = self._spans[-1].left_at or now
        total = now - self._t0

        lines = ["", "=" * 60, f"실행 요약 — {total:.1f}초 · {self._cycles}사이클 "
                 f"· 사건 {self._events}건", "=" * 60]

        if self._spans:
            lines.append("")
            lines.append("상태별 체류")
            held: dict = {}
            visits: dict = {}
            for span in self._spans:
                held[span.name] = held.get(span.name, 0.0) + span.seconds
                visits[span.name] = visits.get(span.name, 0) + 1
            for name, seconds in sorted(held.items(), key=lambda kv: -kv[1]):
                share = seconds / total * 100 if total else 0.0
                lines.append(f"  {name:<16} {seconds:7.1f}s "
                             f"({share:4.1f}%)  {visits[name]}회 진입")

        if self._report_counts:
            lines.append("")
            lines.append("Pi 보고 (종류가 바뀐 횟수)")
            for kind, count in sorted(self._report_counts.items(),
                                      key=lambda kv: -kv[1]):
                lines.append(f"  {kind:<20} {count}회")

        lines.append("")
        if self._alarms:
            lines.append(f"🚨 구동계 경보 {len(self._alarms)}건")
            for at, detail in self._alarms:
                lines.append(f"  [{at:7.1f}s] {detail}")
            lines.append("  ▶ 이 실행에서는 소프트웨어 정지가 바퀴까지 "
                         "닿지 않았을 수 있습니다.")
        else:
            lines.append("구동계 경보 없음")

        if self.path is not None:
            lines.append("")
            lines.append(f"기록: {self.path}")
            lines.append(f"      {self.path.with_suffix('.jsonl')}")
        lines.append("=" * 60)
        return "\n".join(lines)

    def close(self) -> None:
        text = self.summary()
        if self.echo:
            print(text, flush=True)
        for handle in (self._text, self._jsonl):
            if handle is not None:
                if handle is self._text:
                    handle.write(text + "\n")
                handle.close()
        self._text = self._jsonl = None

    # --- 내부 --------------------------------------------------------------

    def _event(self, now: float, message: str) -> None:
        self._events += 1
        self._last_beat = now       # 사건을 찍었으면 하트비트를 미룬다
        self._say(f"[{now - self._t0:7.1f}s] {message}")

    def _say(self, line: str) -> None:
        if self.echo:
            # `\r\033[K` = 줄 앞으로 돌아가 끝까지 지운다. run_mission 의
            # SEARCH_TARGET 줄이 `\r` 로 제자리 덮어쓰기를 하므로, 그냥 찍으면
            # 그 잔여 문자가 뒤에 붙는다. 개행을 앞에 넣어 밀어내는 방법도
            # 있지만 그러면 출력이 통째로 두 줄 간격이 돼 읽기 나쁘다.
            print(f"\r\033[K{line}", flush=True)
        if self._text is not None:
            self._text.write(line + "\n")
            self._text.flush()      # 실기 도중 죽어도 여기까지는 남는다

    def _write_jsonl(self, now, state, pose, cmd, target, report,
                     base_alarm, ready, hz) -> None:
        if self._jsonl is None:
            return
        row = {
            "t": round(now - self._t0, 3),
            "cycle": self._cycles,
            "state": state,
            "cmd": cmd,
            "target": target,
            # bool()로 한 번 더 감싼다 — numpy 스칼라(예: pose.yaw_deg 파생 비교값)가
            # 섞여 들어오면 json.dumps가 TypeError로 미션 전체를 죽인다
            # (2026-09-01 실기 사고: GRASP_ALIGN 도중 크래시, localizer.py의
            # float() 누락이 근본 원인이었다 — 거기는 고쳤지만, 여기도 방어선을
            # 하나 더 둔다).
            "ready": None if ready is None else bool(ready),
            "pose_ok": bool(getattr(pose, "ok", False)),
        }
        if getattr(pose, "ok", False):
            row["x"] = round(pose.x, 4)
            row["y"] = round(pose.y, 4)
            row["yaw"] = round(pose.yaw_deg, 2)
        if report is not None:
            row["report"] = list(report) if isinstance(report, tuple) else report
        if base_alarm:
            row["base_alarm"] = base_alarm
        if hz:
            row["hz"] = round(hz, 2)
        self._jsonl.write(json.dumps(row, ensure_ascii=False) + "\n")
        self._jsonl.flush()

    @staticmethod
    def _format_pose(pose) -> str:
        if not getattr(pose, "ok", False):
            return "pose=없음"
        return f"pose=({pose.x:.3f}, {pose.y:.3f}, {pose.yaw_deg:6.1f}°)"

    @staticmethod
    def _format_report(report) -> str:
        if not isinstance(report, (tuple, list)):
            return str(report)
        kind, state, detail = (list(report) + ["", "", ""])[:3]
        text = f"{kind:<16} [{state}]"
        if detail:
            text += f" {detail}"
        return text
