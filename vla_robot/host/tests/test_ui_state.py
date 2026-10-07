"""시연 UI state(view/ui_state.py)와 로컬 서버(view/web_view.py).
브라우저 없이 본다 — 시뮬 한 판을 돌리며 state 모양·화면 전환을, 서버는 HTTP 로 직접 두드린다."""
import json
import urllib.request

from mission.host_fsm import HostState, MissionFSM
from sim.sim_world import SimWorld
from tests.test_sim_end_to_end import FakeClock
from view.ui_state import UiState
from view.web_view import WebView

APP_KEYS = {"screen", "tone", "run", "target", "status", "done", "detail", "legend", "map", "pieces",
            "target_id", "held", "robot", "path", "estop", "notice", "card", "tray", "hands"}


def _sim_states(cfg, cycles=6000):
    clock = FakeClock()
    world = SimWorld(cfg, clock=clock, seed=1)
    fsm = MissionFSM(cfg)
    ui = UiState(cfg)
    dt = 1.0 / cfg.mission.cycle_hz
    out = []
    for _ in range(cycles):
        clock.t += dt
        world.update()
        status = world.link.latest_status()
        world.link.send(fsm.step(world.pose(), world.piece_map(), status, clock.t))
        out.append((fsm.state, ui.build(world.pose(), world.piece_map(), fsm, status, 0.0, 10.0)))
        if len(world.pieces_in_box("basket")) == 2 and fsm.state == HostState.SEARCH_TARGET:
            break
    return fsm, ui, out


def test_state_shape_and_flow(cfg):
    fsm, ui, states = _sim_states(cfg)
    for _st, s in states:
        assert APP_KEYS <= set(s)
        json.dumps(s, ensure_ascii=False)                     # 브라우저로 그대로 나간다
        assert s["screen"] in ("idle", "listen", "command", "run", "done")
    seen = {s["status"]["en"] for _st, s in states}
    assert {"APPROACH_PIECE", "GRASP", "TRANSPORT", "RELEASE"} <= seen
    # 운반 중에는 쥔 기물이 지도에 로봇 위로 따라간다
    assert any(s["held"] and st == HostState.CARRY_TO_DEST for st, s in states)
    # 접근 중 대상 링이 켜진다(대상 id 가 기물 목록에 있다)
    approach = [s for st, s in states if st == HostState.APPROACH_PIECE]
    assert any(s["target_id"] in {p["id"] for p in s["pieces"]} for s in approach)
    # 진행률은 0~1, 단계가 갈수록 커진다
    prog = [s["status"]["progress"] for _st, s in states]
    assert all(0.0 <= p <= 1.0 for p in prog)
    # 다 옮기면 완료 화면, 2개
    assert ui.done == 2 and states[-1][1]["screen"] == "done"
    assert "2개" in states[-1][1]["done"]["title"]


def test_estop_card(cfg):
    fsm, ui, states = _sim_states(cfg, cycles=50)
    fsm.request_estop()
    world_pose = type("P", (), {"ok": False, "x": 0, "y": 0, "yaw_deg": 0, "xy": (0, 0)})()
    s = ui.build(world_pose, {}, fsm, None, 9.0, 0.0)
    assert s["card"]["code"].startswith("E-000") and s["tray"]["estop_armed"]
    assert s["tray"]["led"] == "lost" and s["notice"]["code"] == "LINK"


def test_hands_and_mat_markers(cfg):
    fsm, ui, _ = _sim_states(cfg, cycles=5)
    pose = type("P", (), {"ok": True, "x": 1.0, "y": 0.5, "yaw_deg": 90.0, "xy": (1.0, 0.5), "fresh": True})()
    s = ui.build(pose, {}, fsm, None, 0.0, 10.0, hands=[(0.10, 0.90)])
    assert s["hands"] == [{"x": 0.1, "y": 0.9, "spot": "L2"}]
    assert len(s["map"]["markers"]) == 4 and s["map"]["boxes"][0]["name"] == "basket"


def _get(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return r.read().decode("utf-8")


def _post(url, obj):
    req = urllib.request.Request(url, data=json.dumps(obj).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return r.read()


def test_web_view_serves_ui_state_and_events(cfg):
    from dataclasses import replace
    from voice.voice_input import VoiceInput
    view = WebView(cfg, port=0, open_window=False,
                   voice=VoiceInput(replace(cfg.voice, enabled=False)))      # 테스트에서 모델을 읽지 않는다
    try:
        page = _get(view.url)
        assert '<script src="/host_bridge.js"></script>' in page and "window.ARENA_MAT = [1.980, 1.830]" in page
        inj = page.index('<script src="/host_bridge.js">')
        assert inj < page.index("/* ─── arena.js ─── */")                    # UI 스크립트보다 먼저
        assert page.rfind("-->", 0, inj) >= page.rfind("<!--", 0, inj)        # 주석 안이 아니다
        assert page[inj:].splitlines()[1].startswith("</head>")
        assert "__hostAttached" in _get(view.url + "host_bridge.js")
        fsm, _ui, _ = _sim_states(cfg, cycles=5)
        pose = type("P", (), {"ok": True, "x": 1.0, "y": 0.5, "yaw_deg": 90.0, "xy": (1.0, 0.5), "fresh": True})()
        assert view.update(pose, {}, fsm, None, 0.0, 10.0) is None
        assert json.loads(_get(view.url + "state"))["robot"]["ok"] is True
        _post(view.url + "event", {"action": "toggle_mode", "payload": None})
        _post(view.url + "event", {"action": "submit", "payload": "퀸 가져와"})
        assert view.update(pose, {}, fsm, None, 0.0, 10.0) == "toggle_manual"
        assert view.update(pose, {}, fsm, None, 0.0, 10.0) is None        # submit 은 해석기로
        # 지도에 기물이 없으면 API 를 부르지 않고 바로 "기물 없음" 카드
        assert json.loads(_get(view.url + "state"))["card"]["code"].startswith("E-201")
        _post(view.url + "event", {"action": "card_action", "payload": "dismiss"})
        view.update(pose, {}, fsm, None, 0.0, 10.0)
        assert json.loads(_get(view.url + "state"))["card"] is None
        _post(view.url + "event", {"action": "card_action", "payload": "reset"})
        assert view.update(pose, {}, fsm, None, 0.0, 10.0) == "reset"
        _post(view.url + "event", {"action": "quit", "payload": None})        # q 키
        assert view.update(pose, {}, fsm, None, 0.0, 10.0) == "quit"
    finally:
        view.close()


def test_battery_cells_show_volts(cfg):
    from vla_common.protocol import PiStatus
    from view.ui_state import battery
    assert battery(0.0, (6.8, 8.4)) == (None, None)
    assert battery(7.6, (6.8, 8.4)) == (50, "7.60V")
    assert battery(9.0, (6.8, 8.4))[0] == 100
    fsm, ui, _ = _sim_states(cfg, cycles=5)
    pose = type("P", (), {"ok": True, "x": 1.0, "y": 0.5, "yaw_deg": 90.0, "xy": (1.0, 0.5), "fresh": True})()
    st = PiStatus(boot_id="b", state="IDLE", busy=False, job_id=0, result=None, base_ok=True, watchdog=False,
                  battery_v=7.0, arm_v=12.0)
    s = ui.build(pose, {}, fsm, st, 0.0, 10.0)
    assert s["detail"]["veh_txt"] == "7.00V" and s["detail"]["arm_txt"] == "12.00V"
    assert s["notice"]["code"] == "BATT"                     # 7.2 V 아래 = 충전 알림


def test_map_view_while_waiting(cfg):
    from dataclasses import replace
    c = replace(cfg, instruction=replace(cfg.instruction, mode="instructed"))
    fsm = MissionFSM(c)
    ui = UiState(c)
    pose = type("P", (), {"ok": True, "x": 1.0, "y": 0.4, "yaw_deg": 90.0, "xy": (1.0, 0.4), "fresh": True})()
    pmap = {"queen": [(0.8, 1.0)], "knight": [(0.24, 0.69)], "star": [(1.5, 1.0)]}
    fsm.step(pose, pmap, None, 0.1)
    assert ui.build(pose, pmap, fsm, None, 0.0, 10.0)["screen"] == "idle"
    ui.show_map = True
    s = ui.build(pose, pmap, fsm, None, 0.0, 10.0)
    assert s["screen"] == "run" and s["run"]["mode"] == "target"
    assert s["target"]["title"] == "장판 위 기물 3개"
    assert s["target"]["reason"] == "별 1 · 퀸 1 · 나이트 1"
    assert not s["scanning"]



def test_next_explains_why_it_does_nothing(cfg):
    from dataclasses import replace
    from view.web_view import WebView
    from voice.voice_input import VoiceInput
    c = replace(cfg, instruction=replace(cfg.instruction, mode="instructed"))
    fsm = MissionFSM(c)
    pose = type("P", (), {"ok": True, "x": 1.0, "y": 0.4, "yaw_deg": 90.0, "xy": (1.0, 0.4), "fresh": True})()
    assert "MANUAL" in WebView._step_blocked(fsm, "next")
    fsm.set_manual_mode(True)
    fsm.step(pose, {"queen": [(0.8, 1.0)]}, None, 0.1)
    assert "지시 대기" in WebView._step_blocked(fsm, "next")
    from mission.host_fsm import Order
    fsm.set_order(Order(("queen",), "one", "organize", "퀸"))
    fsm.step(pose, {"queen": [(0.8, 1.0)]}, None, 0.2)
    assert fsm.ready_to_advance and WebView._step_blocked(fsm, "next") is None


def test_held_piece_disappears_when_gripper_opens(cfg):
    """PLACE 중 Pi 가 job_stage=release 를 보내면 화면에서 쥔 기물을 지운다."""
    from vla_common.protocol import PiStatus
    seen = []
    clock = FakeClock()
    world = SimWorld(cfg, clock=clock, seed=1)
    fsm = MissionFSM(cfg)
    ui = UiState(cfg)
    dt = 1.0 / cfg.mission.cycle_hz
    for _ in range(6000):
        clock.t += dt
        world.update()
        st = world.link.latest_status()
        world.link.send(fsm.step(world.pose(), world.piece_map(), st, clock.t))
        if fsm.state == HostState.PLACE:
            s = ui.build(world.pose(), world.piece_map(), fsm, st, 0.0, 10.0)
            seen.append((st.job_stage if st else "", s["held"] is not None))
        if len(world.pieces_in_box("basket")) >= 1 and seen:
            break
    assert ("move", True) in seen and ("release", False) in seen


def test_manual_carry_reaches_the_box_without_an_extra_next(cfg):
    """MANUAL: 운반 시작에 Next 한 번이면 상자 앞 맞추기까지 이어진다(운반 = 한 단계)."""
    clock = FakeClock()
    world = SimWorld(cfg, clock=clock, seed=1)
    fsm = MissionFSM(cfg, manual_mode=True)
    dt = 1.0 / cfg.mission.cycle_hz
    states, nexts = [], 0
    for _ in range(8000):
        clock.t += dt
        world.update()
        world.link.send(fsm.step(world.pose(), world.piece_map(), world.link.latest_status(), clock.t))
        if fsm.ready_to_advance and fsm.state != HostState.CARRY_TO_DEST:
            fsm.request_advance()
            nexts += 1
        states.append(fsm.state)
        if fsm.state == HostState.PLACE:
            break
    assert HostState.NUDGE_BOX in states and states[-1] == HostState.PLACE


def test_held_stays_gone_while_manual_waits_after_place(cfg):
    """MANUAL: 투입이 끝나고 Next 를 기다리는 동안(Pi job_stage 는 다시 "") 쥔 기물이 되살아나지 않는다."""
    clock = FakeClock()
    world = SimWorld(cfg, clock=clock, seed=1)
    fsm = MissionFSM(cfg, manual_mode=True)
    ui = UiState(cfg)
    dt = 1.0 / cfg.mission.cycle_hz
    waited = []
    for _ in range(9000):
        clock.t += dt
        world.update()
        st = world.link.latest_status()
        world.link.send(fsm.step(world.pose(), world.piece_map(), st, clock.t))
        if fsm.state == HostState.PLACE and fsm.place_released:
            s = ui.build(world.pose(), world.piece_map(), fsm, st, 0.0, 10.0)
            waited.append((st.job_stage, s["held"]))
            if len(waited) > 20:
                break
        elif fsm.ready_to_advance:
            fsm.request_advance()
    assert waited and all(stage == "" for stage, _ in waited[-5:])
    assert all(held is None for _, held in waited)


def test_piece_states_idle_done_outside():
    """10-07: 바구니 앞 띠의 나이트가 "done" 이라 GUI 에서 초록. 바구니 안만 done, 구역 밖은 outside(회색)."""
    from host_config import load_host_config
    from view.ui_state import UiState
    ui = UiState(load_host_config(None))
    st = {p["label"]: p["state"] for p in ui._workspace_pieces(
        {"queen": [(0.90, 0.90)], "knight": [(1.106, 1.307)], "rook": [(0.99, 1.65)]})}
    assert st == {"queen": "idle", "knight": "outside", "rook": "done"}
