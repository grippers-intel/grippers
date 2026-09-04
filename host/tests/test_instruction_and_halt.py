"""자연어 지시 오버라이드와 비상 정지 — 통주행으로는 안 밟히는 자리들.

팀원 브랜치(2026-08-30 인수인계)의 기능을 최신 FSM 위에 얹으면서 쓴 것이다.
저쪽 브랜치에는 `skipped`(보류 기물)도 State.HALTED 도 없어서, **합쳤을 때만
생기는 상호작용**이 몇 가지 있다. 그게 이 파일의 절반이다.

pytest 로도, 단독으로도 돌아간다:

    python tests/test_instruction_and_halt.py
    pytest tests/test_instruction_and_halt.py
"""
from __future__ import annotations

import sys
from pathlib import Path

HOST = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HOST / "aruco"))
sys.path.insert(0, str(HOST))

import mission_config as mcfg
from localizer import Pose
from mission import (MissionFSM, State, _find_label, _in_workspace,
                     visible_labels)
from vehicle_link import MissionCommand, VehicleLink

POSE = Pose(x=1.0, y=0.8, yaw_deg=90.0, ok=True, n_cams=2, fresh=True)
LOST = Pose(x=0.0, y=0.0, yaw_deg=0.0, ok=False, n_cams=0, fresh=False)

PMAP = {
    "queen": [(1.2, 1.0)],
    "rook": [(0.6, 1.0), (0.7, 0.9)],
    "soccer": [(1.4, 0.5)],
    "star": [(0.3, 3.0)],        # y 가 작업 영역 밖 — 이미 상자에 넣은 것
}


class FakeLink(VehicleLink):
    def __init__(self):
        self.sent = []

    def send(self, cmd: MissionCommand) -> None:
        self.sent.append(cmd)

    def poll_status(self) -> str:
        return "IDLE"


def _fsm(state: State = State.SEARCH_TARGET) -> MissionFSM:
    fsm = MissionFSM()
    fsm.state = state
    return fsm


# --- 보이는 라벨 목록 ------------------------------------------------------

def test_visible_labels_excludes_out_of_workspace():
    """상자에 이미 넣은 기물(star)은 후보에서 빠져야 한다 — 안 그러면 Claude
    가 화면에 없는 걸 고른다."""
    assert visible_labels(PMAP) == ["queen", "rook", "soccer"]


def test_visible_labels_is_sorted_and_deduped():
    labels = visible_labels(PMAP)
    assert labels == sorted(labels)
    assert len(labels) == len(set(labels))


def test_in_workspace_matches_nearest_piece_rule():
    assert _in_workspace((1.2, 1.0))
    assert not _in_workspace((0.3, 3.0))


# --- 라벨 지정 탐색 --------------------------------------------------------

def test_find_label_picks_nearest_of_that_label():
    assert _find_label(PMAP, "rook", (0.7, 0.9)) == ("rook", (0.7, 0.9))


def test_find_label_returns_none_when_not_visible():
    assert _find_label(PMAP, "knight", (1.0, 0.8)) is None


def test_find_label_honours_skipped():
    """⚠️ 합치면서 생긴 자리. 재정렬을 다 써서 보류한 기물이 지시로 되살아나면
    같은 기물 앞에서 영원히 맴돈다. 팀원 브랜치엔 skip 개념이 없었다."""
    assert _find_label(PMAP, "queen", (1.0, 0.8), skip=[(1.2, 1.0)]) is None
    # 같은 라벨의 다른 개체는 살아 있어야 한다(라벨이 아니라 좌표로 뺀다).
    got = _find_label(PMAP, "rook", (0.6, 1.0), skip=[(0.7, 0.9)])
    assert got == ("rook", (0.6, 1.0))


# --- 지시: 즉시 적용 -------------------------------------------------------

def test_instruction_applies_now_when_hands_are_empty():
    fsm = _fsm(State.SEARCH_TARGET)
    assert fsm.set_instruction("soccer") is True
    assert fsm._instructed_label == "soccer"


def test_instruction_during_approach_restarts_search():
    fsm = _fsm(State.APPROACH_PIECE)
    fsm.target_label = "queen"
    fsm._target_xy = (1.2, 1.0)
    assert fsm.set_instruction("soccer") is True
    assert fsm.state == State.SEARCH_TARGET
    assert fsm.target_label is None


def test_search_follows_the_instructed_label_not_the_nearest():
    """queen 이 더 가깝지만 지시가 soccer 면 soccer 로 가야 한다."""
    fsm = _fsm(State.SEARCH_TARGET)
    fsm.set_instruction("soccer")
    fsm.step(POSE, PMAP, FakeLink())
    assert fsm.target_label == "soccer"
    assert fsm._instructed_label is None      # 소비됐다


# --- 지시: 큐잉 ------------------------------------------------------------

def test_instruction_is_queued_while_carrying():
    """들고 있는 걸 그냥 놓아 버리면 안 된다."""
    fsm = _fsm(State.CARRY_TO_DEST)
    assert fsm.set_instruction("soccer") is False
    assert fsm._instructed_label is None
    assert fsm._queued_instruction_label == "soccer"


def test_queued_instruction_applies_after_place():
    fsm = _fsm(State.PLACE)
    fsm.target_label = "queen"
    fsm._target_xy = (1.2, 1.0)
    fsm.dest_xy = (0.4, 1.6)
    fsm.set_instruction("soccer")
    fsm.ready_to_advance = True
    fsm._advance_requested = True
    fsm.step(POSE, PMAP, FakeLink())
    assert fsm._queued_instruction_label is None
    assert fsm._instructed_label == "soccer" or fsm.target_label == "soccer"


# --- 지시: 취소 ------------------------------------------------------------

def test_back_to_search_cancels_the_instruction():
    """뒤로가기는 사람의 개입이다 — 원래 지시를 계속 쫓으면 안 된다."""
    fsm = _fsm(State.APPROACH_PIECE)
    fsm._instructed_label = "soccer"
    fsm.request_back()
    fsm.step(POSE, PMAP, FakeLink())
    assert fsm._instructed_label is None


def test_instruction_is_dropped_when_only_skipped_pieces_remain():
    """⚠️ 합치면서 생긴 자리. 지시한 라벨이 전부 보류됐으면 놓아 주고
    최근접으로 돌아가야 한다 — 안 그러면 SEARCH_TARGET 에서 영원히 대기한다."""
    fsm = _fsm(State.SEARCH_TARGET)
    fsm.set_instruction("queen")
    fsm.skipped = [(1.2, 1.0)]          # queen 을 보류 처리
    fsm.step(POSE, PMAP, FakeLink())
    assert fsm._instructed_label is None
    assert fsm.target_label in ("rook", "soccer")


# --- 지시: 목적지(fetch/organize) -------------------------------------------
#
# "퀸 가져와"(사람 앞)와 "퀸 정리해"(상자)를 가르는 부분이다(2026-08-31 팀원
# 브랜치). 여기 테스트의 절반은 **합쳤을 때만 생기는 자리**다 — 저쪽 브랜치엔
# skipped 도 HALTED 도 NUDGE_BOX 도 없었다.

def test_default_instruction_still_goes_to_the_box():
    """목적지를 안 주면 예전 그대로여야 한다 — 애매한 지시의 기본값."""
    fsm = _fsm(State.SEARCH_TARGET)
    fsm.set_instruction("soccer")
    fsm.step(POSE, PMAP, FakeLink())
    assert fsm.target_label == "soccer"
    assert fsm.dest_xy is not None and fsm.dest_xy[1] > 1.0   # 뒤쪽 상자 앞
    assert fsm.dest_face_yaw_deg == mcfg.BOX_FACE_YAW_DEG


def test_fetch_instruction_overrides_the_box():
    fsm = _fsm(State.SEARCH_TARGET)
    fsm.set_instruction("soccer", dest_xy=mcfg.DELIVER_HERE_XY)
    fsm.step(POSE, PMAP, FakeLink())
    assert fsm.dest_xy == mcfg.DELIVER_HERE_XY
    assert fsm.dest_face_yaw_deg == mcfg.DELIVER_HERE_YAW_DEG


def test_fetch_override_is_consumed_not_sticky():
    """한 번 쓴 목적지가 남아 있으면 **다음 기물까지 사람 앞으로 온다.**"""
    fsm = _fsm(State.SEARCH_TARGET)
    fsm.set_instruction("soccer", dest_xy=mcfg.DELIVER_HERE_XY)
    fsm.step(POSE, PMAP, FakeLink())
    assert fsm._instructed_dest_xy is None
    # 다음 기물을 고르는 자리로 되돌려 놓고 다시 한 사이클.
    fsm.state = State.SEARCH_TARGET
    fsm.target_label, fsm._target_xy, fsm.dest_xy = None, None, None
    fsm.step(POSE, PMAP, FakeLink())
    assert fsm.dest_xy != mcfg.DELIVER_HERE_XY
    assert fsm.dest_face_yaw_deg == mcfg.BOX_FACE_YAW_DEG


def test_fetch_dest_is_dropped_with_the_label_when_only_skipped_remain():
    """⚠️ 합치면서 생긴 자리. 지시가 취소되면(전부 보류된 라벨) 목적지도 같이
    놓아야 한다 — 라벨만 놓으면 엉뚱한 기물이 사람 앞으로 배달된다."""
    fsm = _fsm(State.SEARCH_TARGET)
    fsm.set_instruction("queen", dest_xy=mcfg.DELIVER_HERE_XY)
    fsm.skipped = [(1.2, 1.0)]
    fsm.step(POSE, PMAP, FakeLink())
    assert fsm._instructed_dest_xy is None
    assert fsm.dest_xy != mcfg.DELIVER_HERE_XY
    assert fsm.dest_face_yaw_deg == mcfg.BOX_FACE_YAW_DEG


def test_fetch_dest_is_queued_while_carrying():
    fsm = _fsm(State.CARRY_TO_DEST)
    assert fsm.set_instruction("soccer", dest_xy=mcfg.DELIVER_HERE_XY) is False
    assert fsm._queued_instruction_dest_xy == mcfg.DELIVER_HERE_XY
    assert fsm._instructed_dest_xy is None


def test_queued_fetch_dest_survives_the_place():
    fsm = _fsm(State.PLACE)
    fsm.target_label = "queen"
    fsm._target_xy = (1.2, 1.0)
    fsm.dest_xy = (0.4, 1.6)
    fsm.set_instruction("soccer", dest_xy=mcfg.DELIVER_HERE_XY)
    fsm.ready_to_advance = True
    fsm._advance_requested = True
    fsm.step(POSE, PMAP, FakeLink())
    assert fsm._queued_instruction_dest_xy is None
    assert (fsm._instructed_dest_xy == mcfg.DELIVER_HERE_XY
            or fsm.dest_xy == mcfg.DELIVER_HERE_XY)


def test_back_cancels_the_fetch_dest_too():
    fsm = _fsm(State.APPROACH_PIECE)
    fsm._instructed_label = "soccer"
    fsm._instructed_dest_xy = mcfg.DELIVER_HERE_XY
    fsm.request_back()
    fsm.step(POSE, PMAP, FakeLink())
    assert fsm._instructed_dest_xy is None
    assert fsm.dest_face_yaw_deg == mcfg.BOX_FACE_YAW_DEG


def test_halt_queues_the_fetch_dest_as_well():
    """비상 정지 중 지시는 큐에만 쌓인다 — 목적지도 같이 쌓여야 한다."""
    fsm = _fsm(State.SEARCH_TARGET)
    fsm.request_halt()
    assert fsm.set_instruction("soccer", dest_xy=mcfg.DELIVER_HERE_XY) is False
    assert fsm._instructed_dest_xy is None
    assert fsm._queued_instruction_dest_xy == mcfg.DELIVER_HERE_XY


# --- 전달점의 방위 ---------------------------------------------------------
#
# 🔴 상자는 뒤쪽 벽에 있어 +y(90도)를 보면 되는데, 전달점은 **앞쪽**이다.
#    BOX_FACE_YAW_DEG 를 그대로 쓰면 사람에게 등을 돌리고, NUDGE_BOX 가
#    그 방향으로 5 cm 를 더 멀어진다. 팀원 브랜치엔 NUDGE_BOX 가 없어서
#    합치는 쪽에서만 보이는 자리다.

def _face_and_nudge(fsm, at_xy, yaw_now):
    """FACE_BOX 한 번, NUDGE_BOX 한 번 돌리고 (목표방위, nudge 목표점)."""
    pose = Pose(x=at_xy[0], y=at_xy[1], yaw_deg=yaw_now, ok=True,
                n_cams=2, fresh=True)
    fsm.state = State.FACE_BOX
    fsm.step(pose, PMAP, FakeLink())
    target_yaw = fsm.last_nav.target_yaw_deg
    fsm.state = State.NUDGE_BOX
    fsm._nudge_from = None
    aligned = Pose(x=at_xy[0], y=at_xy[1], yaw_deg=target_yaw, ok=True,
                   n_cams=2, fresh=True)
    fsm.step(aligned, PMAP, FakeLink())
    return target_yaw, fsm.last_nav.waypoint


def test_box_facing_and_nudge_are_unchanged():
    fsm = _fsm(State.SEARCH_TARGET)
    fsm.set_instruction("soccer")
    fsm.step(POSE, PMAP, FakeLink())
    yaw, (gx, gy) = _face_and_nudge(fsm, (1.4, 0.95), 90.0)
    assert yaw == mcfg.BOX_FACE_YAW_DEG
    assert gy > 0.95 and abs(gx - 1.4) < 1e-9      # 상자 쪽(+y)으로 붙는다


def test_deliver_facing_and_nudge_point_at_the_person():
    fsm = _fsm(State.SEARCH_TARGET)
    fsm.set_instruction("soccer", dest_xy=mcfg.DELIVER_HERE_XY)
    fsm.step(POSE, PMAP, FakeLink())
    yaw, (gx, gy) = _face_and_nudge(fsm, (0.9, 0.55), 90.0)
    assert yaw == mcfg.DELIVER_HERE_YAW_DEG
    assert gy < 0.55 and abs(gx - 0.9) < 1e-9      # 전달점 쪽(-y)으로 붙는다


def test_deliver_point_is_reachable_by_the_planner():
    """🔴 전달점은 DRIVE_AREA_Y 하한(0.30)보다 앞에 있다 — 로봇이 거기까지
    들어가는 게 아니라 PLACE_TRIGGER_DIST_M 앞에서 멈추기 때문에 성립한다.
    하한이나 전달점을 옮길 때 이 전제가 깨지면 여기서 잡힌다."""
    from navigator import GridPathPlanner
    _sub, _corner, blocked = GridPathPlanner().update(
        (0.9, 1.0), 270.0, mcfg.DELIVER_HERE_XY, [])
    assert blocked != "blocked"


# --- 비상 정지 -------------------------------------------------------------

def test_halt_sends_stop_regardless_of_state():
    for state in (State.APPROACH_PIECE, State.CARRY_TO_DEST, State.PLACE):
        fsm = _fsm(state)
        fsm.request_halt()
        link = FakeLink()
        fsm.step(POSE, PMAP, link)
        assert len(link.sent) == 1
        assert link.sent[0].cmd == "stop"
        assert link.sent[0].status == "HALTED"


def test_halt_sends_stop_even_without_pose():
    """로봇을 잃어도 정지는 나가야 한다 — 평소 경로는 pose.ok 가 아니면
    아무것도 안 보내고 빠져나간다."""
    fsm = _fsm(State.CARRY_TO_DEST)
    fsm.request_halt()
    link = FakeLink()
    fsm.step(LOST, PMAP, link)
    assert [c.cmd for c in link.sent] == ["stop"]


def test_halt_keeps_sending_every_cycle():
    fsm = _fsm(State.APPROACH_PIECE)
    fsm.request_halt()
    link = FakeLink()
    for _ in range(5):
        fsm.step(POSE, PMAP, link)
    assert [c.cmd for c in link.sent] == ["stop"] * 5


def test_only_reset_clears_halt():
    fsm = _fsm(State.APPROACH_PIECE)
    fsm.request_halt()
    fsm.request_back()
    fsm.step(POSE, PMAP, FakeLink())
    assert fsm.halted is True          # 뒤로가기로는 안 풀린다
    fsm.reset()
    assert fsm.halted is False


def test_instruction_during_halt_only_queues():
    """정지 중에 상태를 바꿔 두면 reset 직후 엉뚱한 대상으로 튄다."""
    fsm = _fsm(State.SEARCH_TARGET)
    fsm.request_halt()
    assert fsm.set_instruction("soccer") is False
    assert fsm._instructed_label is None
    assert fsm._queued_instruction_label == "soccer"


def test_two_halts_are_different_things():
    """State.HALTED(코드가 막힘)와 halted 플래그(사람이 멈춤)는 다른 것이다.
    이름이 비슷해서 헷갈리기 쉬우니 못 박아 둔다."""
    fsm = _fsm(State.SEARCH_TARGET)
    fsm._halt("투하 재시도 소진")
    assert fsm.state == State.HALTED
    assert fsm.halted is False
    fsm.request_halt()
    assert fsm.state == State.HALTED and fsm.halted is True


# --- 단독 실행 -------------------------------------------------------------

if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as exc:
            failed += 1
            print(f"  FAIL  {name}: {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - failed}/{len(tests)} 통과")
    sys.exit(1 if failed else 0)
