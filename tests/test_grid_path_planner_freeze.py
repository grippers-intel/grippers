"""GridPathPlanner — 로봇이 실제로 안 움직였으면 경로를 다시 안 짠다
(2026-09-06, 사용자 지시로 근본 수정 — "제자리에서 이상하게 돈다").

## 왜

실기 로그(CARRY_TO_DEST): 로봇 좌표가 5cm 이내로 거의 안 움직였는데 yaw는
135도 넘게 돌았다가 다시 29도 되돌아왔다. GridPathPlanner는 매 사이클
경로를 처음부터 다시 짜고 회전량은 비용에 안 넣는다(navigator.py 모듈
docstring "왜 매 사이클 다시 짜는가" / "무엇을 최소화하는가") — 그래서
로봇이 ROTATE 중이라 실제로는 거의 안 움직였는데도, 근처 장애물을 사이에
둔 좌/우 우회 비용이 비슷하면 mm 단위 위치 흔들림만으로 sub_goal(부분목표)
이 뒤집힐 수 있다. DriveSequencer는 회전 관성(coast)으로 STOP<->ROTATE를
다시 오갈 수 있는데, 그때마다 이 흔들리는 sub_goal로 목표각을 다시 잠그면
실제 필요한 것보다 훨씬 큰 호를 그리며 뱅뱅 도는 것처럼 보였다.

tests/test_drive_sequencer_rotate_hysteresis.py가 검증하는 "ROTATE 진입
시점에 한 번 정한 목표각은 정렬될 때까지 그대로 쓴다"는 그 한 단계 위
(DriveSequencer)의 방어막이지만, STOP을 거쳐 ROTATE로 **다시** 들어갈
때마다 새로 잠그는 목표각 자체가 흔들리는 sub_goal에서 나오면 소용이
없다 — 이 파일은 그 근본 원인, GridPathPlanner가 위치 변화 없이도 매번
다시 계산하던 부분을 직접 검증한다."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "host"))

import mission_config as mcfg          # noqa: E402
from navigator import GridPathPlanner  # noqa: E402


def test_위치가_거의_안_바뀌면_target이_달라져도_직전_결과를_그대로_쓴다():
    """PATH_REPLAN_MIN_MOVE_M의 40%만 움직인 상태에서 target_xy를 완전히
    다른 방향으로 바꿔도, 진짜로 다시 계산했다면 달라졌을 sub_goal이
    그대로여야 한다 — ROTATE 중 mm 단위 흔들림에 안 낚인다는 뜻이다."""
    planner = GridPathPlanner()
    robot_xy = (0.5, 0.5)

    first = planner.update(robot_xy, 90.0, (0.5, 1.0), [])

    jittered_xy = (robot_xy[0] + mcfg.PATH_REPLAN_MIN_MOVE_M * 0.4, robot_xy[1])
    again = planner.update(jittered_xy, 90.0, (1.2, 0.1), [])

    assert again == first, "위치가 문턱 안인데도 다시 계산해 결과가 바뀌었다"


def test_문턱을_넘게_움직이면_다시_계산한다():
    """실제 전진(한 사이클 이동량이 문턱보다 뚜렷이 큼)까지 얼려 버리면
    장애물 회피·경로 진행 자체가 멈춘다 — 문턱을 넘으면 반드시 다시
    계산해야 한다."""
    planner = GridPathPlanner()
    robot_xy = (0.5, 0.5)

    first = planner.update(robot_xy, 90.0, (0.5, 1.0), [])

    moved_xy = (robot_xy[0], robot_xy[1] + mcfg.PATH_REPLAN_MIN_MOVE_M * 3)
    again = planner.update(moved_xy, 90.0, (1.2, 0.1), [])

    assert again != first, "충분히 움직였는데도 얼어붙은 결과를 그대로 냈다"


def test_reset하면_같은_자리에서도_다시_계산한다():
    """구간이 바뀌면(다른 기물/상자로) 로봇 위치가 우연히 비슷해도 새
    target을 반영해 다시 계산해야 한다 — reset()이 얼린 결과까지 지우는지
    확인한다."""
    planner = GridPathPlanner()
    robot_xy = (0.5, 0.5)

    first = planner.update(robot_xy, 90.0, (0.5, 1.0), [])
    planner.reset()
    again = planner.update(robot_xy, 90.0, (1.2, 0.1), [])

    assert again[0] != first[0], "reset 뒤에도 이전 구간의 sub_goal이 남아 있다"
