"""NUDGE_BOX — Host ArUco 기준 절대 안전 반경 (2026-09-03, 사용자 지시).

## 왜 이 기능이 생겼나

장난감(box) 바구니 INSERT 중 차체가 바구니 오른쪽으로 치우쳐 정렬한 채로
오른쪽 입구 턱을 실제로 박았다. 그 직전 로그를 보면 라이다 판독 자체가
"라이다 판독이 하한보다 가깝다 — 테두리를 넘겨보고 있을 수 있다"처럼 스스로
불안정을 알리고 있었다 — 차체가 물리적으로 가장 위험한 바로 그 순간에,
유일한 안전장치가 그 흔들리는 신호 하나뿐이었던 셈이다.

이 파일은 Pi 라이다 판독과 완전히 무관하게, Host가 직접 ArUco로 잰
"로봇-바구니 중심 거리"가 mission_config.BASKET_HARD_STOP_MARGIN_M 반경
안이면 NUDGE_BOX가 어느 축(전후/좌우/회전)이든 무조건 멈추는지 검증한다."""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "host"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "host" / "aruco"))

import mission_config as mcfg                                    # noqa: E402
import config as cfg                                              # noqa: E402
from localizer import box_pose                                    # noqa: E402
from mission import MissionFSM, State                             # noqa: E402

from conftest import PiSim                                        # noqa: E402

TOY_X, TOY_Y, _TOY_YAW = box_pose("toy")
HARD_RADIUS = cfg.BOX_L / 2.0 + mcfg.BASKET_HARD_STOP_MARGIN_M


def _fsm_near_box(axis: str, dist_from_center: float, amount_m: float = 0.05):
    """toy 바구니 중심에서 dist_from_center 만큼 떨어진 자리(정면 방향)에서
    NUDGE_BOX가 axis 계획을 실행하려는 상황을 만든다."""
    fsm = MissionFSM()
    fsm.state = State.NUDGE_BOX
    fsm.target_label = "box"   # mission_config.PIECE_DEST_BOX["box"] == "toy"
    fsm._nudge_plan = (amount_m, axis)
    # 바구니는 y+ 방향(가벽 쪽)에 있고 BOX_FACE_YAW_DEG(90도)를 정면으로
    # 본다 — 바구니 정면(y가 더 작은 쪽)에서 y+ 방향으로 접근하는 배치.
    # _nudge_from은 일부러 안 건드린다 — None으로 두면 mission.py가 첫
    # step()에서 그 시점 robot_xy로 스스로 초기화하면서 stall 타이머
    # (_nudge_stall_at)도 같이 세팅한다. 여기서 미리 채워버리면 그
    # 초기화를 건너뛰어(_nudge_stall_at이 기본값 0.0으로 남아) 첫
    # step()부터 곧바로 "6초 스톨"로 오판한다(실기 코드가 아니라 이
    # 테스트 헬퍼가 만든 문제였다).
    robot_y = TOY_Y - dist_from_center
    link = PiSim(x=TOY_X, y=robot_y, yaw_deg=mcfg.BOX_FACE_YAW_DEG)
    return fsm, link


def test_반경_밖이면_평소대로_전진한다():
    fsm, link = _fsm_near_box("forward", dist_from_center=HARD_RADIUS + 0.10)

    fsm.step(link.pose(), {}, link)

    cmds = {c for c, status in link.sent if status == "NUDGE_BOX"}
    assert cmds == {"go"}, "안전 반경 밖인데도 전진하지 않았다"


def test_반경_안이면_전진_축도_무조건_멈춘다():
    fsm, link = _fsm_near_box("forward", dist_from_center=HARD_RADIUS - 0.02)

    fsm.step(link.pose(), {}, link)

    cmds = {c for c, status in link.sent if status == "NUDGE_BOX"}
    assert cmds == {"stop"}, "절대 안전 반경 안인데도 계속 전진했다"


def test_반경_안이면_좌우_축도_무조건_멈춘다():
    fsm, link = _fsm_near_box("left", dist_from_center=HARD_RADIUS - 0.02)

    fsm.step(link.pose(), {}, link)

    cmds = {c for c, status in link.sent if status == "NUDGE_BOX"}
    assert cmds == {"stop"}, "절대 안전 반경 안인데도 좌우로 계속 밀었다"


def test_반경_안이면_회전_축도_무조건_멈춘다():
    fsm, link = _fsm_near_box("rotate_left", dist_from_center=HARD_RADIUS - 0.02)

    fsm.step(link.pose(), {}, link)

    cmds = {c for c, status in link.sent if status == "NUDGE_BOX"}
    assert cmds == {"stop"}, "절대 안전 반경 안인데도 계속 회전했다"


def test_반경_안이어도_후진은_허용한다():
    """axis="back"만은 하드스톱 예외다(2026-09-03, 사용자 지시) — 완전
    정지로 가두면 반경 안에 갇힌 채 스스로 빠져나올 방법이 없다. 후진은
    위험 반경에서 멀어지는 방향이니 그대로 허용해야 한다."""
    fsm, link = _fsm_near_box("back", dist_from_center=HARD_RADIUS - 0.02)

    fsm.step(link.pose(), {}, link)

    cmds = {c for c, status in link.sent if status == "NUDGE_BOX"}
    assert cmds == {"back"}, "절대 안전 반경 안이라고 후진마저 막아버렸다"


def test_정상_INSERT_거리는_반경에_안_걸린다():
    """BASKET_TARGET_LIDAR_M(라이다 목표 0.140m) 부근에서 성공하던 정상
    접근 거리가 이 안전 반경 때문에 막히면 안 된다 — sanity check."""
    normal_dist = (mcfg.BASKET_TARGET_LIDAR_M + cfg.BOX_L / 2.0)
    assert normal_dist > HARD_RADIUS, (
        "정상 INSERT 거리가 하드스톱 반경보다 안쪽이다 — 정상 접근까지 막는다")
