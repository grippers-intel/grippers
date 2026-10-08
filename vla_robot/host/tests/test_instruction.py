"""사람 지시: Claude 응답 해석 · 지시(Order)대로 고르기 · 입력창 흐름.
Claude API 는 부르지 않는다 — 가짜 해석기를 끼운다(실제 호출은 키를 넣은 뒤 tools 로 확인)."""
import json
from dataclasses import replace

from mission.commands import CommandDesk
from mission.host_fsm import HostState, MissionFSM, Order
from mission.instruction import NO_MATCH, InstructionResolver, parse_result, schema_for
from sim.sim_world import SimWorld
from tests.test_sim_end_to_end import FakeClock
from view.ui_state import UiState


class FakeResolver:
    """submit 한 문장에 정해 둔 결과를 돌려준다."""

    def __init__(self, answers):
        self.answers = answers
        self.pending = None
        self.calls = []

    @property
    def busy(self):
        return self.pending is not None

    def submit(self, text, visible):
        self.calls.append((text, list(visible)))
        self.pending = parse_result(text, self.answers[text], list(visible)) if isinstance(
            self.answers[text], dict) else self.answers[text]
        return True

    def poll(self):
        r, self.pending = self.pending, None
        return r


def test_schema_only_offers_visible_labels():
    s = schema_for(["star", "queen", "star"])
    assert s["properties"]["labels"]["items"]["enum"] == ["queen", "star", NO_MATCH]
    assert s["additionalProperties"] is False and set(s["required"]) == set(s["properties"])
    json.dumps(s)


def test_parse_result_filters_labels():
    raw = {"matched": True, "labels": ["queen", "rook", "queen", NO_MATCH], "quantity": "all",
           "intent": "fetch", "reply": "퀸", "reason": "r"}
    r = parse_result("퀸 다 가져와", raw, ["queen", "star"])
    assert r.ok and r.labels == ("queen",) and r.quantity == "all" and r.intent == "fetch"
    r = parse_result("뭐", {"matched": True, "labels": [NO_MATCH], "quantity": "one", "intent": "organize",
                           "reply": "", "reason": ""}, ["queen"])
    assert not r.ok                      # matched 여도 실제 라벨이 없으면 실패


def test_resolver_without_pieces_does_not_call_api(cfg):
    r = InstructionResolver(cfg.instruction).resolve("퀸 정리해", [])
    assert not r.ok and r.error is None and "기물이 없습니다" in r.reason


def _sim(cfg, mode="auto", seed=1):
    cfg = replace(cfg, instruction=replace(cfg.instruction, mode=mode))
    clock = FakeClock()
    world = SimWorld(cfg, clock=clock, seed=seed)
    return cfg, clock, world, MissionFSM(cfg)


def _run(cfg, clock, world, fsm, cycles, until=None):
    dt = 1.0 / cfg.mission.cycle_hz
    for _ in range(cycles):
        clock.t += dt
        world.update()
        world.link.send(fsm.step(world.pose(), world.piece_map(), world.link.latest_status(), clock.t))
        if until and until():
            return True
    return False


def test_instructed_mode_waits_then_moves_only_the_ordered_piece(cfg):
    cfg, clock, world, fsm = _sim(cfg, "instructed")
    _run(cfg, clock, world, fsm, 100)
    assert fsm.state == HostState.SEARCH_TARGET and fsm.search_reason == "waiting for command"
    assert world.pieces_in_box("basket") == []
    labels = sorted(world.piece_map())
    first = labels[-1]                               # 가까운 것과 무관하게 지시한 것을 고른다
    fsm.set_order(Order((first,), "one", "organize", f"{first} 정리해"))
    assert _run(cfg, clock, world, fsm, 6000, until=lambda: fsm.finished_order is not None)
    assert world.pieces_in_box("basket") == [first]
    assert fsm.finished_order[1] == "done" and fsm.order is None
    _run(cfg, clock, world, fsm, 100)                # 지시가 끝나면 다시 기다린다
    assert world.pieces_in_box("basket") == [first]
    assert fsm.state == HostState.SEARCH_TARGET


def test_order_for_all_and_absent(cfg):
    cfg, clock, world, fsm = _sim(cfg, "instructed")
    labels = tuple(sorted(world.piece_map()))
    fsm.set_order(Order(labels, "all", "organize", "전부 정리해"))
    assert _run(cfg, clock, world, fsm, 12000, until=lambda: fsm.finished_order is not None)
    assert sorted(world.pieces_in_box("basket")) == list(labels)
    assert fsm.finished_order[0].done == len(labels)
    fsm.set_order(Order(("rook",), "one", "organize", "룩 정리해"))     # 지도에 없는 기물
    _run(cfg, clock, world, fsm, 20)
    assert fsm.finished_order[1] == "absent"


def test_reset_cancels_order(cfg):
    _cfg, _clock, _world, fsm = _sim(cfg, "instructed")
    fsm.set_order(Order(("star",), "one", "organize", "별"))
    fsm.reset()
    assert fsm.order is None


def test_desk_flow_to_order_and_ui(cfg):
    cfg, clock, world, fsm = _sim(cfg, "instructed")
    target = sorted(world.piece_map())[0]
    fake = FakeResolver({
        "그거 정리해": {"matched": True, "labels": [target], "quantity": "one", "intent": "organize",
                     "reply": "ok", "reason": "r"},
        "모르는 말": {"matched": False, "labels": [NO_MATCH], "quantity": "one", "intent": "organize",
                  "reply": "", "reason": "기물을 특정할 수 없음"},
        "가져와": {"matched": True, "labels": [target], "quantity": "one", "intent": "fetch",
                "reply": "", "reason": ""},
    })
    desk = CommandDesk(cfg, resolver=fake)
    ui = UiState(cfg)
    pmap = world.piece_map()
    pose = world.pose()

    desk.submit("모르는 말", pmap)
    assert ui.build(pose, pmap, fsm, None, 0.0, 10.0, desk=desk)["screen"] == "command"
    desk.update(fsm)
    s = ui.build(pose, pmap, fsm, None, 0.0, 10.0, desk=desk)
    assert s["card"]["code"].startswith("E-402") and s["card"]["heard"]["text"] == "모르는 말"
    assert {r["id"] for r in s["card"]["rows"]} == set(pmap)          # 보이는 기물에서 고르기
    desk.pick_label(target, fsm)
    assert fsm.order.labels == (target,) and desk.card() is None
    fsm.cancel_order()

    desk.submit("그거 정리해", pmap)
    desk.update(fsm)
    assert fsm.order is not None and fsm.order.text == "그거 정리해"
    s = ui.build(pose, pmap, fsm, None, 0.0, 10.0, desk=desk)
    assert s["screen"] == "run" and s["run"]["quote"] == "“그거 정리해”"
    fsm.cancel_order()

    desk.submit("가져와", pmap)
    desk.update(fsm)                                   # 손이 안 보인다 -> 손 검출을 막 켰으니 잠깐 찾는다
    s = ui.build(pose, pmap, fsm, None, 0.0, 10.0, desk=desk)
    assert fsm.order is None and s["screen"] == "command" and desk.phase == "hand_wait"
    desk._hand_wait_until = 0.0                        # 그래도 없으면 접수하지 않고 카드
    desk.update(fsm)
    s = ui.build(pose, pmap, fsm, None, 0.0, 10.0, desk=desk)
    assert fsm.order is None and s["card"]["code"].startswith("E-220")


def test_desk_api_error_card(cfg):
    from mission.instruction import Instruction
    cfg2 = cfg
    fake = FakeResolver({"퀸": Instruction("퀸", False, error="Claude API 키가 없거나 틀렸습니다 (ANTHROPIC_API_KEY)")})
    desk = CommandDesk(cfg2, resolver=fake)
    fsm = MissionFSM(cfg2)
    desk.submit("퀸", {"queen": [(1.0, 0.9)]})
    desk.update(fsm)
    card = desk.card()
    assert card["tone"] == "error" and "ANTHROPIC_API_KEY" in card["detail"] and fsm.order is None


def test_missing_key_is_reported_not_raised(cfg, monkeypatch):
    """키가 하나도 없으면 SDK 가 TypeError — 화면 카드용 문구로 바꾼다(요청은 나가지 않는다)."""
    for k in ("ANTHROPIC_API_KEY", "ANTHROPIC_AUTH_TOKEN", "ANTHROPIC_PROFILE"):
        monkeypatch.delenv(k, raising=False)
    r = InstructionResolver(cfg.instruction).resolve("퀸 정리해", ["queen"])
    assert not r.ok and r.error and "ANTHROPIC_API_KEY" in r.error


# ---------------------------------------------------------------- 손에 건네기
def _run_with_hands(cfg, clock, world, fsm, cycles, until=None):
    dt = 1.0 / cfg.mission.cycle_hz
    for _ in range(cycles):
        clock.t += dt
        world.update()
        fsm.set_hands(world.hands)
        world.link.send(fsm.step(world.pose(), world.piece_map(), world.link.latest_status(), clock.t))
        if until and until():
            return True
    return False


def _hand_sim(cfg, spots, seed=1):
    cfg = replace(cfg, instruction=replace(cfg.instruction, mode="instructed"))
    clock = FakeClock()
    world = SimWorld(cfg, clock=clock, seed=seed, hands=[cfg.handover.spots[s][:2] for s in spots])
    return cfg, clock, world, MissionFSM(cfg)


def test_fetch_hands_the_piece_over(cfg):
    for spot in ("L2", "F2", "R1"):
        c, clock, world, fsm = _hand_sim(cfg, [spot])
        label = sorted(world.piece_map())[0]
        fsm.set_order(Order((label,), "one", "fetch", f"{label} 가져와"))
        sent = []
        orig = world.link.send
        world.link.send = lambda cmd: (sent.append(cmd), orig(cmd))[1]
        assert _run_with_hands(c, clock, world, fsm, 8000, until=lambda: fsm.finished_order is not None), \
            f"{spot}: state={fsm.state.name} events={list(fsm.events)}"
        assert world.pieces_in_box("hand") == [label], (spot, list(fsm.events))
        assert world.pieces_in_box("basket") == []
        assert fsm.finished_order[0].handed == 1
        assert any(cmd.state == "PLACE" and cmd.place_pose == "handover" for cmd in sent)
        assert label not in world.piece_map()            # 손에 건넨 기물은 장판에 없다


def test_fetch_without_hand_waits_then_uses_basket(cfg):
    c, clock, world, fsm = _hand_sim(cfg, ["L2"])
    c = replace(c, handover=replace(c.handover, hand_wait_s=3.0))
    fsm = MissionFSM(c)
    label = sorted(world.piece_map())[0]
    fsm.set_order(Order((label,), "one", "fetch", "가져와"))
    # 파지까지는 손이 보이다가, 운반 중에 손을 치운다
    dt = 1.0 / c.mission.cycle_hz
    for _ in range(8000):
        clock.t += dt
        world.update()
        if fsm.state == HostState.CARRY_TO_DEST and fsm.dest_kind == "hand":
            world.hands = []
        fsm.set_hands(world.hands)
        world.link.send(fsm.step(world.pose(), world.piece_map(), world.link.latest_status(), clock.t))
        if fsm.finished_order is not None:
            break
    assert world.pieces_in_box("basket") == [label], list(fsm.events)
    assert fsm.finished_order[0].handed == 0
    assert any("기다린다" in e for e in fsm.events) or any("바구니로" in e for e in fsm.events)


def test_desk_fetch_needs_a_visible_hand(cfg):
    target = "queen"
    fake = FakeResolver({"퀸 가져와": {"matched": True, "labels": [target], "quantity": "one",
                                    "intent": "fetch", "reply": "", "reason": ""}})
    desk = CommandDesk(cfg, resolver=fake)
    fsm = MissionFSM(cfg)
    pmap = {"queen": [(0.9, 0.9)]}
    desk.submit("퀸 가져와", pmap)
    assert desk.hands_wanted(fsm)                      # 해석 중 = "가져와"일 수 있다 -> 손 검출을 켠다
    desk.update(fsm, hands=[])
    assert desk.phase == "hand_wait" and fsm.order is None and desk.hands_wanted(fsm)
    desk._hand_wait_until = 0.0
    desk.update(fsm, hands=[])
    card = desk.card()
    assert card["code"].startswith("E-220") and fsm.order is None
    desk.to_basket(fsm)
    assert fsm.order.intent == "organize" and desk.card() is None
    fsm.cancel_order()
    desk.submit("퀸 가져와", pmap)
    desk.update(fsm, hands=[(0.10, 0.90)])
    assert fsm.order.intent == "fetch" and desk.hands_wanted(fsm)   # 건네는 동안 손 검출 유지
    fsm.cancel_order()
    desk.submit("퀸 가져와", pmap)                     # 해석이 끝났을 때 손이 아직 확인 전이면 기다렸다가 받는다
    desk.update(fsm, hands=[])
    assert desk.phase == "hand_wait"
    desk.update(fsm, hands=[(0.10, 0.90)])
    assert fsm.order.intent == "fetch" and desk.phase == "accepted"
    fsm.cancel_order()
    desk.dismiss()
    assert not desk.hands_wanted(fsm)                  # 할 일이 없으면 끈다


def test_ui_wording_for_handover(cfg):
    c, clock, world, fsm = _hand_sim(cfg, ["L2"])
    ui = UiState(c)
    label = sorted(world.piece_map())[0]
    fsm.set_order(Order((label,), "one", "fetch", "가져와"))
    seen = set()
    dt = 1.0 / c.mission.cycle_hz
    for _ in range(8000):
        clock.t += dt
        world.update()
        fsm.set_hands(world.hands)
        st = world.link.latest_status()
        world.link.send(fsm.step(world.pose(), world.piece_map(), st, clock.t))
        s = ui.build(world.pose(), world.piece_map(), fsm, st, 0.0, 10.0, hands=world.hands)
        seen.add(s["status"]["en"])
        if fsm.finished_order is not None:
            break
    assert {"TO_HAND", "HANDOVER"} <= seen
    assert "손에 건넸습니다" in s["done"]["title"]
