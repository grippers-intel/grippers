"""DriveSequencer — yaw+/yaw- 헌팅을 감지하면 잠깐 전진해서 끊는다
(2026-09-03, 사용자 지시).

## 왜 이 기능이 생겼나

RETURN_HOME 실기: 직선으로 가도 되는 구간인데도 "yaw+" -> "yaw-" -> "yaw+"...
로 방향을 계속 바꿔가며 제자리에서 한참 헌팅하다가 겨우 한 걸음 가는 게
답답하다는 지적(사용자). ROTATE가 수렴(aligned)했다가도 다음 FORWARD 판정에서
바로 반대쪽으로 안 맞다고 나오면, 그 반대 방향 ROTATE로 다시 들어가는 일이
반복될 수 있다 — 이게 몇 번 연속으로 반대 방향이면(토글), 원래 알고리즘을
믿지 않고 ROTATE_OSCILLATION_ESCAPE_CYCLES 사이클 동안 강제로 짧게
전진(ESCAPE)해서 흐름을 끊고 처음부터 다시 판단하게 한다.

시간이 아니라 **사이클 수**로 재는 이유는 mission_config.py의
ROTATE_OSCILLATION_ESCAPE_CYCLES 정의부 주석 참고 — DriveSequencer는 시계를
모르는 순수 상태기계다.

이 파일은 GridPathPlanner 전체를 불러오지 않고 `DriveSequencer.update()`를
직접 호출해, robot_xy/target_xy를 고정한 채 robot_yaw_deg만 목표각 양쪽으로
번갈아 흔들어 토글을 재현한다.

`update()`는 매 호출마다 "이번에 낼 모드"(out_mode)를 **전이 전** 값으로
돌려준다(기존 test_drive_sequencer_rotate_hysteresis.py 와 같은 관례) — 그래서
STOP -> ROTATE(또는 ESCAPE) 전이는 그다음 호출에야 반환값에 보인다. 아래
`_round_trip()`은 "정렬 -> STOP -> FORWARD -> 반대쪽으로 misalign -> STOP ->
(ROTATE 또는 ESCAPE)"까지 정확히 5번 호출해 그 결과를 돌려준다."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "host"))

import mission_config as mcfg               # noqa: E402
from navigator import DriveMode, DriveSequencer   # noqa: E402


def _round_trip(seq: DriveSequencer, robot_xy, flip_yaw: float):
    """ROTATE가 수렴한 상태에서 시작해, FORWARD -> (flip_yaw로) misalign ->
    다음 ROTATE 진입 시도까지 딱 한 바퀴 진행시키고 그 결과 DriveCommand를
    돌려준다. 호출 전 self._mode 는 STOP(next=FORWARD) 이거나 ROTATE(aligned)
    상태라고 가정한다 — 즉 반드시 이 함수를 연달아 불러야 한다."""
    cmd = None
    for i in range(5):
        yaw = 0.0 if i < 2 else flip_yaw
        cmd = seq.update(robot_xy, yaw, (1.0, 0.0), [])
    return cmd


def test_반대_방향으로_토글이_한계를_넘으면_ESCAPE로_들어간다():
    seq = DriveSequencer(yaw_tolerance_deg=5.0)
    robot_xy = (0.0, 0.0)

    cmd = seq.update(robot_xy, 170.0, (1.0, 0.0), [])
    assert cmd.mode == DriveMode.ROTATE

    flips = [170.0, -170.0] * mcfg.ROTATE_OSCILLATION_TOGGLE_LIMIT
    cmd = None
    for flip_yaw in flips:
        cmd = _round_trip(seq, robot_xy, flip_yaw)
        if cmd.mode == DriveMode.ESCAPE:
            break

    assert cmd is not None and cmd.mode == DriveMode.ESCAPE, (
        "반대 방향 토글이 한계를 넘었는데도 ESCAPE로 안 들어갔다")


def test_같은_방향으로만_이어지는_회전은_ESCAPE로_안_간다():
    """길이 굽어서 회전이 여러 번 필요해도, 매번 같은 방향이면(예: 계속
    왼쪽으로만 굽는 경로) 토글이 아니다 — ESCAPE에 걸리면 안 된다."""
    seq = DriveSequencer(yaw_tolerance_deg=5.0)
    robot_xy = (0.0, 0.0)

    cmd = seq.update(robot_xy, 170.0, (1.0, 0.0), [])
    assert cmd.mode == DriveMode.ROTATE

    for _ in range(mcfg.ROTATE_OSCILLATION_TOGGLE_LIMIT + 2):
        cmd = _round_trip(seq, robot_xy, 170.0)   # 매번 같은 쪽(170)
        assert cmd.mode == DriveMode.ROTATE, "같은 방향인데 ESCAPE로 들어갔다"


def test_ESCAPE는_설정된_사이클_동안만_지속되고_그다음_처음부터_다시_판단한다():
    seq = DriveSequencer(yaw_tolerance_deg=5.0)
    robot_xy = (0.0, 0.0)

    cmd = seq.update(robot_xy, 170.0, (1.0, 0.0), [])
    assert cmd.mode == DriveMode.ROTATE
    flips = [170.0, -170.0] * mcfg.ROTATE_OSCILLATION_TOGGLE_LIMIT
    cmd = None
    for flip_yaw in flips:
        cmd = _round_trip(seq, robot_xy, flip_yaw)
        if cmd.mode == DriveMode.ESCAPE:
            break
    assert cmd is not None and cmd.mode == DriveMode.ESCAPE

    # ESCAPE로 "관측"된 이 시점은 실제 전이 시점보다 이미 한 바퀴(5호출) 늦다
    # (out_mode가 전이 전 값을 돌려주는 관례 때문 — 위 모듈 docstring 참고).
    # 그래서 남은 사이클 수를 seq._escape_remaining 에서 직접 읽어 정확히
    # 맞춘다.
    remaining = seq._escape_remaining
    assert remaining > 0

    # 마지막 한 사이클 전까지는 계속 ESCAPE(정렬 여부와 무관).
    for _ in range(remaining - 1):
        cmd = seq.update(robot_xy, 0.0, (1.0, 0.0), [])
        assert cmd.mode == DriveMode.ESCAPE

    # 다 차면 처음(mode=None)처럼 다시 판단한다 — 여기선 이미 정렬돼
    # 있으니(robot_yaw=0) FORWARD 로 나가야 한다.
    cmd = seq.update(robot_xy, 0.0, (1.0, 0.0), [])
    assert cmd.mode == DriveMode.FORWARD
