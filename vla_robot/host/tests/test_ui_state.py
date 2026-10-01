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
    view = WebView(cfg, port=0, open_window=False)
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
        assert view.update(pose, {}, fsm, None, 0.0, 10.0) is None        # submit 은 알림만
        assert json.loads(_get(view.url + "state"))["notice"]["code"] == "CMD"
        _post(view.url + "event", {"action": "card_action", "payload": "reset"})
        assert view.update(pose, {}, fsm, None, 0.0, 10.0) == "reset"
    finally:
        view.close()
