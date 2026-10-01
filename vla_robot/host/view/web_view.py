"""팀원 시연 UI(ui/grippers-ui.html)를 run_host 화면으로 쓴다 — MapView 와 같은 update() 를 갖는다.

- 표준 라이브러리 HTTP 서버(스레드 하나)가 127.0.0.1 에서
  `/` (UI + host_bridge.js 주입) · `/host_bridge.js` · `/state` · `/event` 를 낸다.
- 창은 크롬/엣지의 `--app` 모드(주소창·탭 없는 창, 팀원 run-windows.bat 과 같은 방식)로 띄운다.
  둘 다 없으면 기본 브라우저로 연다. 윈도우·맥 모두 추가 설치가 없다(pywebview 불필요).
- 메인 루프와 GUI 를 한 프로세스에 섞지 않는다 — 화면은 브라우저 프로세스가 그린다
  (예전 matplotlib + cv2 GIL 크래시를 다시 만들지 않는다, map_view.py 머리말).

UI 버튼 -> MapView 키와 같은 action 문자열: estop · reset · next · prev · toggle_manual.
입력창 문장(submit)·카드 버튼(dismiss · pick_label)은 CommandDesk 가 받는다(Claude 해석 -> FSM 지시).
창에서 q(입력창 밖) = run_host 종료(창도 닫힌다). 창을 X 로 닫으면 run_host 는 돈다 — 그때는 터미널에서 Ctrl+C.
"""
from __future__ import annotations

import json
import os
import queue
import shutil
import subprocess
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

from mission.commands import CommandDesk
from voice.voice_input import VoiceInput
from view.ui_state import UiState

UI_DIR = Path(__file__).resolve().parents[1] / "ui"
UI_HTML = UI_DIR / "grippers-ui.html"
BRIDGE_JS = UI_DIR / "host_bridge.js"
# 이벤트 -> run_host action (MapView.KEYMAP 과 같은 이름)
ACTIONS = {"estop": "estop", "reset": "reset", "next": "next", "prev": "prev", "toggle_mode": "toggle_manual",
           "quit": "quit"}   # q 키(host_bridge.js) — run_host 를 끝내고 창도 닫는다


def _browser_candidates() -> list[str]:
    if sys.platform.startswith("win"):
        env = os.environ
        roots = [env.get("ProgramFiles", r"C:\Program Files"),
                 env.get("ProgramFiles(x86)", r"C:\Program Files (x86)"),
                 env.get("LOCALAPPDATA", "")]
        rel = [r"Google\Chrome\Application\chrome.exe", r"Microsoft\Edge\Application\msedge.exe"]
        return [str(Path(r) / p) for p in rel for r in roots if r]
    if sys.platform == "darwin":
        return ["/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
                "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge"]
    return [shutil.which(n) or "" for n in ("google-chrome", "chromium", "chromium-browser", "microsoft-edge")]


def _profile_dir() -> Path:
    return Path(os.environ.get("LOCALAPPDATA") or Path.home() / ".cache") / "vla_robot_ui" / "profile"


def close_app_windows() -> int:
    """전용 프로필(vla_robot_ui)로 뜬 크롬/엣지 창을 모두 닫는다. 개인 브라우저는 프로필이 달라 건드리지 않는다.
    윈도우 크롬은 처음 띄운 프로세스가 창을 다른 프로세스에 넘기고 빠지므로 그 하나만 끝내서는 안 닫힌다(2026-10-02)."""
    marker = "vla_robot_ui"
    try:
        if sys.platform.startswith("win"):
            cmd = ("$n=0; Get-CimInstance Win32_Process | Where-Object { ($_.Name -in 'chrome.exe','msedge.exe') "
                   f"-and $_.CommandLine -match '{marker}' }} | ForEach-Object {{ Stop-Process -Id $_.ProcessId "
                   "-Force -ErrorAction SilentlyContinue; $n++ }; $n")
            out = subprocess.run(["powershell", "-NoProfile", "-Command", cmd], capture_output=True,
                                 text=True, timeout=10).stdout.strip()
            return int(out or 0)
        subprocess.run(["pkill", "-f", marker], timeout=5)
    except Exception:  # noqa: BLE001 — 종료 경로, 실패해도 run_host 종료는 막지 않는다
        pass
    return 0


def open_app_window(url: str):
    """주소창 없는 세로 창. (어떤 브라우저로 열었는지, 창 프로세스 또는 None)."""
    for exe in _browser_candidates():
        if exe and Path(exe).exists():
            profile = _profile_dir()
            profile.mkdir(parents=True, exist_ok=True)
            proc = subprocess.Popen([exe, f"--app={url}", f"--user-data-dir={profile}",
                                     "--window-size=460,860", "--window-position=80,40",
                                     "--no-first-run", "--no-default-browser-check"],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return Path(exe).stem, proc
    webbrowser.open(url)
    return "default browser", None


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class WebView:
    def __init__(self, cfg, port: Optional[int] = None, open_window: bool = True, desk=None,
                 voice=None) -> None:
        self.cfg = cfg
        self.ui = UiState(cfg)
        # 입력창 -> Claude 해석 -> FSM 지시. 키가 없어도 만들어진다(요청 때 카드로 알린다).
        self.desk = desk if desk is not None else CommandDesk(cfg)
        # 마이크 버튼 = 노트북 내장 마이크. 모델은 뒤에서 미리 읽는다(~4 s).
        self.voice = voice if voice is not None else VoiceInput(cfg.voice)
        self._state_json = b"{}"
        self._lock = threading.Lock()
        self._events: "queue.Queue[tuple[str, object]]" = queue.Queue()
        page = UI_HTML.read_text(encoding="utf-8")
        a = cfg.arena
        inject = (f"<script>window.ARENA_MAT = [{a.wall_x[1] - a.wall_x[0]:.3f}, "
                  f"{a.wall_y[1] - a.wall_y[0]:.3f}];</script>\n"
                  '<script src="/host_bridge.js"></script>\n')
        # <body 바로 앞의 진짜 </head> 에 넣는다 — 주석·문자열 속 같은 글자에 끼우면 스크립트가 안 돈다.
        body_at = page.find("<body")
        head_end = page.rfind("</head>", 0, body_at if body_at >= 0 else len(page))
        if head_end < 0:
            raise RuntimeError(f"{UI_HTML}: </head> 가 없다 — UI 파일이 바뀌었나?")
        self._page = (page[:head_end] + inject + page[head_end:]).encode("utf-8")
        self._bridge = BRIDGE_JS.read_bytes()
        view = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args):      # 요청마다 콘솔에 찍지 않는다(0.1 s 폴링)
                pass

            def _send(self, body: bytes, ctype: str, code: int = 200) -> None:
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = self.path.split("?", 1)[0]
                if path in ("/", "/index.html"):
                    self._send(view._page, "text/html; charset=utf-8")
                elif path == "/host_bridge.js":
                    self._send(view._bridge, "text/javascript; charset=utf-8")
                elif path == "/state":
                    with view._lock:
                        body = view._state_json
                    self._send(body, "application/json; charset=utf-8")
                else:
                    self._send(b"not found", "text/plain", 404)

            def do_POST(self):
                if self.path != "/event":
                    self._send(b"not found", "text/plain", 404)
                    return
                try:
                    n = int(self.headers.get("Content-Length", "0"))
                    obj = json.loads(self.rfile.read(min(n, 65536)).decode("utf-8"))
                    view._events.put((str(obj.get("action", "")), obj.get("payload")))
                except (ValueError, UnicodeDecodeError):
                    pass
                self._send(b"{}", "application/json")

        port = cfg.view.web_port if port is None else port
        self._httpd = _Server(("127.0.0.1", port), Handler)
        self.url = f"http://127.0.0.1:{self._httpd.server_address[1]}/"
        threading.Thread(target=self._httpd.serve_forever, name="web-view", daemon=True).start()
        # 다시 띄울 때 창이 쌓이지 않게 — 남아 있던 시연 창을 닫고 새로 연다. close() 에서도 닫는다(2026-10-02).
        if open_window:
            close_app_windows()
        how, self._window = open_app_window(self.url) if open_window else ("not opened", None)
        print(f"[view] 시연 UI: {self.url} ({how}) — 창에서 q = 종료 (창을 X 로 닫으면 Ctrl+C 로 끝낼 것)")

    def update(self, pose, piece_map, fsm, pi_status, link_age_s: float, hz: float,
               hands=()) -> Optional[str]:
        action = self._next_action(fsm, piece_map)   # 먼저 — 버튼이 띄운 알림이 이번 화면에 바로 실린다
        self.desk.update(fsm, hands)
        state = self.ui.build(pose, piece_map, fsm, pi_status, link_age_s, hz, hands, self.desk, self.voice)
        body = json.dumps(state, ensure_ascii=False).encode("utf-8")
        with self._lock:
            self._state_json = body
        return action

    def _next_action(self, fsm, piece_map) -> Optional[str]:
        """한 사이클에 action 하나. 나머지 이벤트는 다음 사이클로."""
        while True:
            try:
                action, payload = self._events.get_nowait()
            except queue.Empty:
                return None
            if action == "card_action":
                action = str(payload)
            if action == "estop" and fsm.estop:
                # 해제는 초기화(r)로만 — 실수로 재개되지 않게(FSM reset 주석). 카드의 버튼을 쓰게 한다.
                self.ui.notify("정지 해제는 카드의 '초기화 후 재개' 로 합니다", "E-000", "error")
                continue
            if action in ACTIONS:
                if action == "reset":                   # 초기화 = 지시도 취소(FSM reset)
                    self.ui.reset()
                    self.desk.dismiss()
                return ACTIONS[action]
            if action == "submit" and isinstance(payload, str):
                self.voice.cancel()
                self.desk.submit(payload, piece_map)
            elif action == "mic":
                why = self.voice.toggle()
                if why:
                    self.ui.notify(why, "MIC", "caution")
            elif action == "run":                   # 접수 화면의 전송 — 들은 문장을 해석으로
                text = self.voice.take_final()
                if text:
                    self.desk.submit(text, piece_map)
            elif action == "basket":
                self.desk.to_basket(fsm)
            elif action in ("dismiss", "retry", "cancel"):
                self.desk.dismiss()
            elif action == "pick_label" and isinstance(payload, str):
                self.desk.pick_label(payload, fsm)
            # 지도 기물 탭(pick)·run 등은 아직 쓰지 않는다

    def close(self) -> None:
        try:
            self._httpd.shutdown()
            self._httpd.server_close()
        except Exception:  # noqa: BLE001 — 종료 경로
            pass
        if self._window is not None:
            close_app_windows()                 # 전용 프로필 창만 — 개인 크롬과 별개
