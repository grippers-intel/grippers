"""DriveSequencer — 회전 중엔 목표각을 다시 쫓지 않는다 (2026-09-02 실기).

## 왜 이 기능이 생겼나

09-02 실기에서 GRASP_REPLAN과 RETURN_HOME 둘 다, 76도 근처에서 회전을
시작해 같은 방향("yaw+")으로 250도 이상 돌아버렸다 — 최단 방향이 훨씬 짧게
남아 있는데도 그쪽으로 안 갔다. `DriveSequencer.update()`는 회전 중에도
매 사이클 목표각을 GridPathPlanner의 부분목표에서 새로 뽑는데, 로봇이
거의 안 움직이는 회전 중에도 부분목표가(격자 재탐색의 시작 칸 근처
민감성으로) 살짝씩 흔들리면 목표각이 사이클마다 바뀌고, 그걸 계속
쫓다 보면 방향이 한쪽으로 계속 밀릴 수 있다.

이 파일은 "회전 중 목표각이 흔들려도, ROTATE 진입 시점에 한 번 정한
목표각을 정렬될 때까지 그대로 쓰는지"만 검증한다 — GridPathPlanner 전체를
불러오지 않고 `DriveSequencer.update()`를 직접 호출해, target_xy를 매
사이클 살짝 흔들어 본다."""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "host"))

from navigator import DriveMode, DriveSequencer   # noqa: E402


def test_회전_중_목표가_흔들려도_처음_정한_방향을_유지한다():
    seq = DriveSequencer(yaw_tolerance_deg=5.0)
    robot_xy = (0.0, 0.0)
    robot_yaw = 76.0   # 09-02 실기와 같은 시작각

    # 목표가 로봇 기준 대략 반대쪽(약 -170도 방향)에 있어서 회전이 필요하다.
    # 매 사이클 target_xy를 살짝 흔들어 부분목표 잔떨림을 흉내낸다.
    jitter = [0.0, 0.01, -0.01, 0.01, -0.01, 0.0]
    first_cmd = seq.update(robot_xy, robot_yaw, (-1.0, 0.02), [])
    assert first_cmd.mode == DriveMode.ROTATE
    committed_target = first_cmd.target_yaw_deg

    for j in jitter:
        cmd = seq.update(robot_xy, robot_yaw, (-1.0, 0.02 + j), [])
        if cmd.mode != DriveMode.ROTATE:
            break
        # 흔들림에도 이번 회전 구간이 쫓는 목표각은 처음 정한 값 그대로다.
        assert cmd.target_yaw_deg == committed_target


def test_정렬되면_다음_회전에서는_새_목표각을_다시_잰다():
    seq = DriveSequencer(yaw_tolerance_deg=5.0)
    robot_xy = (0.0, 0.0)

    # 첫 회전: 목표 방향 0도 쪽으로.
    cmd = seq.update(robot_xy, 90.0, (1.0, 0.0), [])
    assert cmd.mode == DriveMode.ROTATE
    first_target = cmd.target_yaw_deg
    assert math.isclose(first_target, 0.0, abs_tol=1e-6)

    # 정렬됐다고 로봇 yaw 를 맞춰 준다. out_mode는 "전이 전" 값이라 정렬된
    # 바로 이번 사이클은 아직 ROTATE로 나가고, 다음 사이클에 STOP, 그다음
    # FORWARD로 넘어간다.
    cmd = seq.update(robot_xy, 0.0, (1.0, 0.0), [])
    assert cmd.mode == DriveMode.ROTATE
    cmd = seq.update(robot_xy, 0.0, (1.0, 0.0), [])
    assert cmd.mode == DriveMode.STOP
    cmd = seq.update(robot_xy, 0.0, (1.0, 0.0), [])
    assert cmd.mode == DriveMode.FORWARD

    # 이제 목표가 완전히 다른 방향으로 바뀐다 — 새 회전은 새 목표각을
    # 잡아야 한다(묵은 값에 갇히면 안 된다). 역시 out_mode가 "전이 전"
    # 값이라 FORWARD -> STOP -> ROTATE로 두 사이클 더 걸린다.
    cmd = seq.update(robot_xy, 0.0, (0.0, 1.0), [])
    assert cmd.mode == DriveMode.FORWARD
    cmd = seq.update(robot_xy, 0.0, (0.0, 1.0), [])
    assert cmd.mode == DriveMode.STOP
    cmd = seq.update(robot_xy, 0.0, (0.0, 1.0), [])
    assert cmd.mode == DriveMode.ROTATE
    assert math.isclose(cmd.target_yaw_deg, 90.0, abs_tol=1e-6)
