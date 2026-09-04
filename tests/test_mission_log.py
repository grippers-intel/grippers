"""실기 기록과 단계별 진행 도구 (2026-08-29 실기 준비).

2026-08-28의 가장 큰 손실은 기록이 없었다는 것이다. 여섯 번 돌렸고 GRASP 가
한 번 성공했는데, 남은 것은 터미널 스크롤백뿐이었다. 여기서 고정하는 성질은
둘이다 — **사건은 빠짐없이 남고, 사건이 아닌 것으로 화면을 채우지 않는다.**
"""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from pathlib import Path

import pytest

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

import mission_log                                    # noqa: E402
from vehicle_link import MissionCommand               # noqa: E402


@dataclass
class FakePose:
    x: float = 1.0
    y: float = 0.5
    yaw_deg: float = 90.0
    ok: bool = True


def _logger(tmp_path, **kwargs):
    kwargs.setdefault("echo", False)
    return mission_log.MissionLogger(path=tmp_path / "run.log", **kwargs)


def _pi_lines(tmp_path) -> list:
    """Pi 보고 이벤트 줄만.

    `close()` 가 요약표를 같은 파일에 이어 쓰므로, 파일 전체에서 문자열을
    세면 요약의 집계 줄까지 같이 잡힌다."""
    text = (tmp_path / "run.log").read_text(encoding="utf-8")
    return [line for line in text.splitlines() if "↑ Pi" in line]


# ── 무엇이 사건인가 ────────────────────────────────────────────────────────


def test_상태가_바뀔_때만_전이를_남긴다(tmp_path):
    """14Hz 를 그대로 찍으면 사람이 못 읽고 정작 전이가 묻힌다."""
    log = _logger(tmp_path)
    for _ in range(20):
        log.record(state="SEARCH_TARGET", pose=FakePose(), cmd=None)
    for _ in range(20):
        log.record(state="APPROACH_PIECE", pose=FakePose(), cmd="go")
    log.close()

    text = (tmp_path / "run.log").read_text(encoding="utf-8")
    assert text.count("SEARCH_TARGET → APPROACH_PIECE") == 1


def test_같은_Pi_보고가_반복되면_한_번만_남긴다(tmp_path):
    """워치독 거부는 초당 여러 번 나온다 — 그대로 찍으면 로그가 그것뿐이다."""
    log = _logger(tmp_path)
    report = ("REJECTED", "APPROACH", "Host 명령이 3사이클 연속 없음 — 정지")
    for _ in range(50):
        log.record(state="GRASP", pose=FakePose(), cmd="stop", report=report)
    log.close()

    assert len(_pi_lines(tmp_path)) == 1


def test_보고_내용이_바뀌면_다시_남긴다(tmp_path):
    """숫자가 달라졌다는 것은 상황이 움직였다는 뜻이라 사건이다."""
    log = _logger(tmp_path)
    log.record(state="PLACE", pose=FakePose(), cmd="stop",
               report=("INSERT_BLOCKED", "APPROACH_BOX", "라이다 0.351m"))
    log.record(state="PLACE", pose=FakePose(), cmd="stop",
               report=("INSERT_BLOCKED", "APPROACH_BOX", "라이다 0.299m"))
    log.close()

    lines = _pi_lines(tmp_path)
    assert len(lines) == 2
    assert "0.351m" in lines[0] and "0.299m" in lines[1]


def test_아무_일이_없어도_하트비트가_나온다(tmp_path):
    """화면이 멈춘 것과 조용한 것을 사람이 구분할 수 있어야 한다."""
    log = _logger(tmp_path, heartbeat_sec=0.0)
    for _ in range(3):
        log.record(state="SEARCH_TARGET", pose=FakePose(), cmd=None)
    log.close()

    text = (tmp_path / "run.log").read_text(encoding="utf-8")
    assert text.count("· SEARCH_TARGET") >= 2


# ── 구동계 경보 ────────────────────────────────────────────────────────────


def test_구동계_경보는_요약에_반드시_남는다(tmp_path):
    """이 실행에서 정지가 안 닿았을 수 있다는 사실은 놓치면 안 된다."""
    log = _logger(tmp_path)
    log.record(state="GRASP", pose=FakePose(), cmd="stop",
               base_alarm="구동계 이상 (STALE_FEEDBACK) — 컨트롤러가 물렸다")
    summary = log.summary()
    log.close()

    assert "🚨" in summary
    assert "STALE_FEEDBACK" in summary


def test_경보가_없으면_없다고_적는다(tmp_path):
    """침묵은 증거가 아니다 — 봤고 없었다는 것을 적어야 한다."""
    log = _logger(tmp_path)
    log.record(state="SEARCH_TARGET", pose=FakePose(), cmd=None)
    summary = log.summary()
    log.close()

    assert "구동계 경보 없음" in summary


def test_같은_경보를_반복해도_한_건으로_센다(tmp_path):
    log = _logger(tmp_path)
    for _ in range(30):
        log.record(state="GRASP", pose=FakePose(), cmd="stop",
                   base_alarm="구동계 이상 (NO_CONSUMER) — x")
    summary = log.summary()
    log.close()

    assert "구동계 경보 1건" in summary


# ── JSONL ──────────────────────────────────────────────────────────────────


def test_자세가_사이클마다_남는다(tmp_path):
    """2026-08-28에 래치된 명령을 증명한 것이 이 궤적이다.

    그때는 `\\r` 로 덮어쓴 터미널 바이트를 다시 갈라내야 했고, 10사이클마다
    한 표본밖에 없었다."""
    log = _logger(tmp_path)
    for i in range(5):
        log.record(state="NUDGE_BOX", pose=FakePose(x=1.0 + i * 0.01),
                   cmd="go", hz=13.9)
    log.close()

    rows = [json.loads(line) for line
            in (tmp_path / "run.jsonl").read_text(encoding="utf-8").splitlines()]
    assert len(rows) == 5
    assert [r["x"] for r in rows] == [1.0, 1.01, 1.02, 1.03, 1.04]
    assert rows[0]["cmd"] == "go" and rows[0]["state"] == "NUDGE_BOX"


def test_자세를_잃은_사이클도_남는다(tmp_path):
    """Host 가 조용해지는 구간이라 오히려 더 중요하다 — pose 를 잃으면
    `fsm.step` 이 명령을 안 보내고, Pi 워치독이 그때부터 센다."""
    log = _logger(tmp_path)
    log.record(state="APPROACH_PIECE", pose=FakePose(ok=False), cmd="go")
    log.close()

    row = json.loads((tmp_path / "run.jsonl").read_text(encoding="utf-8").strip())
    assert row["pose_ok"] is False
    assert "x" not in row


def test_실행_도중_죽어도_남은_것은_읽을_수_있다(tmp_path):
    """`close()` 를 못 부르고 죽는 것이 실기의 정상 상황이다."""
    log = _logger(tmp_path)
    log.record(state="GRASP", pose=FakePose(), cmd="stop")

    rows = (tmp_path / "run.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(rows) == 1, "flush 를 안 하면 여기가 비어 있다"


# ── 요약 ──────────────────────────────────────────────────────────────────


def test_상태별_체류_시간을_센다(tmp_path):
    """2026-08-28에 이걸 로그에서 손으로 셌다."""
    log = _logger(tmp_path)
    log.record(state="SEARCH_TARGET", pose=FakePose(), cmd=None)
    log.record(state="GRASP", pose=FakePose(), cmd="stop")
    log.record(state="SEARCH_TARGET", pose=FakePose(), cmd=None)
    summary = log.summary()
    log.close()

    assert "상태별 체류" in summary
    assert "SEARCH_TARGET" in summary and "2회 진입" in summary


def test_파일_없이도_돈다(tmp_path):
    """기록을 끈 채 돌리는 것도 정상 사용이다."""
    log = mission_log.MissionLogger(path=None, echo=False)
    log.record(state="GRASP", pose=FakePose(), cmd="stop")
    log.close()          # 예외가 안 나면 통과


def test_실행마다_다른_파일에_쓴다():
    """덮어쓰면 2026-08-28에 Pi 로그를 잃은 것과 같은 일이 난다."""
    first = mission_log.default_log_path("monitor")
    assert first.name.startswith("monitor_")
    assert first.suffix == ".log"


# ── 실행 후 코멘트 ────────────────────────────────────────────────────────
#
# run_mission.py 종료 직후 사람이 남기는 한 줄 — "재시도 끝에 성공" 같은
# 요약표엔 안 남는 맥락을 로그 파일 끝에 덧붙인다.


def test_코멘트를_텍스트와_jsonl_양쪽에_남긴다(tmp_path):
    log_path = tmp_path / "run.log"
    log_path.write_text("# 기존 로그\n", encoding="utf-8")
    jsonl_path = log_path.with_suffix(".jsonl")
    jsonl_path.write_text('{"t": 0}\n', encoding="utf-8")

    mission_log.append_annotation(log_path, "knight를 떨어뜨렸지만 재시도 후 성공")

    text = log_path.read_text(encoding="utf-8")
    assert "# 기존 로그" in text, "기존 내용을 지우면 안 된다 — 추가만 한다"
    assert "knight를 떨어뜨렸지만 재시도 후 성공" in text

    rows = [json.loads(line) for line in jsonl_path.read_text(encoding="utf-8").splitlines()]
    assert rows[0] == {"t": 0}, "기존 사이클 기록은 그대로여야 한다"
    assert rows[-1]["annotation"] == "knight를 떨어뜨렸지만 재시도 후 성공"


def test_빈_코멘트는_아무것도_안_남긴다(tmp_path):
    log_path = tmp_path / "run.log"
    log_path.write_text("# 기존 로그\n", encoding="utf-8")

    mission_log.append_annotation(log_path, "   ")

    assert log_path.read_text(encoding="utf-8") == "# 기존 로그\n"
    assert not log_path.with_suffix(".jsonl").exists()


def test_기록_경로가_없으면_코멘트도_조용히_무시한다():
    mission_log.append_annotation(None, "이 실행은 --no-log 였다")  # 예외가 안 나면 통과


# ── 두 도구 ────────────────────────────────────────────────────────────────
#
# 이 셋이 확인하는 것은 **argv 배선**이다 — 도구가 기본값을 제대로 켜고
# 사용자 인자를 안 잃는가. 그런데 `run_mission` 을 import 하는 순간 geti
# SDK 까지 딸려 들어온다. 실기 맥에는 있지만 CI 나 다른 사람 기계에는 없을
# 수 있고, 그것 때문에 배선 검증을 통째로 건너뛰면 안 된다.
#
# 그래서 없을 때만 최소 스텁을 끼운다. 있는 환경에서는 진짜 모듈을 쓴다.


@pytest.fixture
def tools(monkeypatch):
    """`state_monitor` 와 `manual_step` 을 import 할 수 있게 만들어 준다.

    막는 것은 `geti_detector` 하나다 — `run_mission` 이 geti 를 만나는 유일한
    자리이고, `main()` 을 갈아 끼우므로 그 안의 어떤 함수도 안 불린다. SDK
    자체를 흉내내려 들면 그쪽 import 사슬을 따라다니게 되는데, 그건 이
    테스트가 확인하려는 것과 아무 상관이 없다."""
    import types
    try:
        import geti_detector                    # noqa: F401
    except ImportError:
        stub = types.ModuleType("geti_detector")
        stub.GetiWorker = object
        stub.load_deployment = lambda **_kw: None
        stub.draw = lambda frame, _pred: frame
        monkeypatch.setitem(sys.modules, "geti_detector", stub)
    import manual_step
    import state_monitor
    return state_monitor, manual_step


def test_모니터_도구는_기록을_켠다(monkeypatch, tools):
    state_monitor, _manual = tools
    seen = {}
    monkeypatch.setattr(state_monitor.run_mission, "main",
                        lambda: seen.update(argv=sys.argv[1:]) or 0)
    monkeypatch.setattr(sys, "argv", ["state_monitor.py", "--cams", "0"])

    state_monitor.main()

    assert "--log-file" in seen["argv"]
    assert "--cams" in seen["argv"], "원래 인자를 그대로 넘겨야 한다"


def test_단계별_도구는_수동_모드를_켠다(monkeypatch, tools):
    _monitor, manual_step = tools
    seen = {}
    monkeypatch.setattr(manual_step.run_mission, "main",
                        lambda: seen.update(argv=sys.argv[1:]) or 0)
    monkeypatch.setattr(sys, "argv", ["manual_step.py", "--vehicle-ip", "192.168.0.7"])

    manual_step.main()

    assert "--manual" in seen["argv"]
    assert "192.168.0.7" in seen["argv"]


def test_사용자가_준_기록_경로를_존중한다(monkeypatch, tmp_path, tools):
    state_monitor, _manual = tools
    seen = {}
    monkeypatch.setattr(state_monitor.run_mission, "main",
                        lambda: seen.update(argv=sys.argv[1:]) or 0)
    monkeypatch.setattr(sys, "argv",
                        ["state_monitor.py", "--log-file", str(tmp_path / "x.log")])

    state_monitor.main()

    assert seen["argv"].count("--log-file") == 1
