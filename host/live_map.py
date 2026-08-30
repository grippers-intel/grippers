"""로봇 pose + 기물 지도를 위에서 내려다본 단순 2D 도형으로 그린다.

카메라 원본 영상 대신 로봇(화살표) · 기물(라벨별 전용 모양) · 상자(사각형)만
그려서 한눈에 보이게 하는 게 목적이다. 카메라 원본은 필요할 때만
(run_mission.py --show-cams) 따로 켠다.

matplotlib 의 FuncAnimation/plt.show() 는 메인 스레드를 블로킹해서 카메라
캡처 루프와 같이 못 돈다. 그 대신 매 사이클 update() 를 직접 불러서
draw_idle() + pause() 로 논블로킹 갱신한다 — cv2.imshow()+waitKey(1) 와
같은 패턴이다.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Callable, Optional

import matplotlib.patches as patches
from matplotlib.colors import to_rgba
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.path import Path as MplPath
from matplotlib.transforms import Affine2D
from matplotlib.widgets import Button, TextBox

import mission_config as mcfg

# config.py/localizer.py 는 aruco/ 하위폴더에 있다(팀원이 동기화하는 파일이라
# 건드리지 않는다) — 그 폴더를 경로에 추가해서 기존처럼 bare import 로 쓴다.
sys.path.insert(0, str(Path(__file__).parent / "aruco"))

import config as cfg
from localizer import Pose
from navigator import DriveCommand

PieceMap = dict[str, list[tuple[float, float]]]
XY = tuple[float, float]

ROOM_SIZE = 1.8   # m — 가벽 안쪽 정사각형 작업 공간 (config.py 문서 참고)

# geti 가 내는 라벨(project.json 기준). 여기 순서와 무관하게 슬롯은 미리 만들어 둔다.
KNOWN_LABELS = ["star", "soccer", "box", "knight", "queen", "rook"]

# 라벨별 아이콘 — DejaVu Sans(matplotlib 기본 폰트)에 실제로 있는 유니코드
# 기호만 쓴다(dejavu_symbol_sheet.png 로 확인한 것). "box" 만 도형(흰 네모)으로
# 남겨둔다 — 딱히 대응되는 간단한 기호가 없어서.
GLYPHS = {
    "queen": "♛", "knight": "♞", "rook": "♜",
    "star": "✩",     # ✩ STRESS OUTLINED WHITE STAR
    "soccer": "❆",   # ❆ 계열 눈꽃/장식 기호 (사용자가 26BD 대신 고름)
}

# 범례를 2열 그리드로 그린다 — matplotlib legend(ncol=2) 는 column-major 로
# 채우므로(1열 위→아래, 그 다음 2열 위→아래), 화면에서 가로로 짝이 맞길
# 원하는 순서(robot|path, box|soccer, star|queen, knight|rook)대로 보이게
# 하려면 핸들을 [1열 전체][2열 전체] 순으로 나열해야 한다.
LEGEND_COL1 = ["box", "star", "knight"]
LEGEND_COL2 = ["soccer", "queen", "rook"]

# 팀원 디자인 시안을 참고해 통일한 팔레트 — 부드러운 배경 + 한글 폰트.
# 상태 패널의 행 이름. **고정이다** — 값이 없으면 "-" 로 채우고 행 자체는
# 안 없앤다. 그래야 라벨 칸을 배경에 구워 둘 수 있다(_status_labels 주석).
STATUS_ROWS = ("x", "y", "yaw", "state", "target", "cmd",
               "battery(veh)", "battery(arm)")

plt.rcParams.update({
    "font.family": ["Malgun Gothic", "DejaVu Sans", "Arial", "sans-serif"],
    "font.size": 10,
    "axes.titleweight": "bold",
    "axes.edgecolor": "#999999",
    "axes.linewidth": 0.8,
    "xtick.color": "#555555",
    "ytick.color": "#555555",
    "text.color": "#222222",
    "figure.facecolor": "#f5f6f7",
    "axes.facecolor": "#ffffff",
})

# 로봇 방향 화살표 — yaw=0(=+x 방향)일 때 뾰족한 끝이 +x 를 향하도록 정의해서,
# config.py 의 yaw 정의(+x축 기준 반시계)와 회전각을 그대로 맞춰 쓸 수 있게 한다.
_ROBOT_ARROW = MplPath(
    vertices=[(0.9, 0.0), (-0.5, 0.45), (-0.2, 0.0), (-0.5, -0.45), (0.9, 0.0)],
    codes=[MplPath.MOVETO, MplPath.LINETO, MplPath.LINETO, MplPath.LINETO, MplPath.CLOSEPOLY],
)


class _PieceSlot:
    """기물 하나(라벨+슬롯 번호)를 그리는 아티스트 묶음. 안 보이면 화면 밖에 숨긴다."""

    def __init__(self, ax: plt.Axes, label: str) -> None:
        self.label = label
        self._artists: list = []

        if label in GLYPHS:
            t = ax.text(0, 0, GLYPHS[label], fontsize=20, ha="center", va="center",
                        color="black", zorder=6, visible=False)
            self._artists = [t]
        elif label == "box":
            sc = ax.scatter([0], [0], marker="s", s=160, facecolor="white",
                            edgecolor="black", linewidth=1.2, zorder=6, visible=False)
            self._artists = [sc]
        else:
            # 모르는 라벨이 와도(모델이 바뀌는 등) 죽지 않게 기본 모양으로.
            sc = ax.scatter([0], [0], marker="o", s=120, facecolor="lightgray",
                            edgecolor="black", zorder=6, visible=False)
            self._artists = [sc]

    def set_pos(self, x: float, y: float) -> None:
        for a in self._artists:
            if hasattr(a, "set_offsets"):
                a.set_offsets([[x, y]])
            else:
                a.set_position((x, y))
            a.set_visible(True)

    def hide(self) -> None:
        for a in self._artists:
            a.set_visible(False)


class LiveMap:
    def __init__(self, on_reset: Optional[Callable[[], None]] = None,
                 on_next: Optional[Callable[[], None]] = None,
                 on_back: Optional[Callable[[], None]] = None,
                 on_toggle_mode: Optional[Callable[[], None]] = None,
                 on_instruction: Optional[Callable[[str], None]] = None,
                 on_voice: Optional[Callable[[], None]] = None,
                 on_halt: Optional[Callable[[], None]] = None,
                 blit: bool = True) -> None:
        """on_reset 은 리셋 버튼, on_next/on_back 은 Next/Prev 버튼(수동
        모드에서 다음/이전 단계로), on_toggle_mode 는 Mode 버튼(자동↔수동
        전환) 콜백이다.

        on_instruction(text) 은 "지시" 패널에서 문장을 제출했을 때(Enter
        또는 전송 버튼) 호출된다 — 자연어 문장 하나가 그대로 전달되며,
        Claude API 로 파싱해서 target_label/dest_box 를 뽑아내는 건 이
        클래스가 아니라 호출 쪽(run_mission.py) 책임이다. on_voice() 는
        음성 버튼 콜백 — 마이크 녹음을 시작하고, 다 되면 인식된 텍스트를
        set_instruction_text() 로 이 창에 채워 넣는 것까지 호출 쪽이 한다.
        아직 둘 다 연결 전이면 None 으로 둬도 되고(버튼은 눌리지만 콘솔에만
        로그가 찍힘), 나중에 API 연동할 때 이 두 콜백만 넘기면 된다.

        on_halt() 는 지도 바로 아래의 빨간 "비상 정지" 버튼 콜백이다 —
        누르면 그 즉시 무조건 정지하고, Reset(on_reset) 을 누르기 전까지는
        아무 것도 안 한다(mission.MissionFSM.request_halt() 참고). Reset과
        별개로 두는 이유: Reset 은 지금까지 계산한 목표만 잊고 다시 찾기
        시작하는 것뿐이라 실제 정지를 보장하지 않지만, 비상 정지는
        무조건·즉시 멈추는 것이 유일한 목적이라 눈에 띄게 따로 둔다.

        이 클래스는 자기가 그리는 것(기물 표시·경로선)만 지울 수 있고,
        PieceTracker/MissionFSM 같은 실제 상태는 모른다 — 그래서 그쪽까지
        건드리고 싶으면 run_mission.py 가
        tracker.reset()/fsm.reset()/fsm.request_advance()/fsm.request_back()/
        fsm.set_manual_mode() 를 부르는 콜백을 여기 넘겨준다. 지금 모드가
        뭔지도 이 클래스는 모르므로, 버튼 글자는 update() 의 manual_mode
        인자로 매 사이클 갱신한다(run_mission.py 가 fsm.manual_mode 를 넘김).
        """
        self._on_reset = on_reset
        self._on_next = on_next
        self._on_back = on_back
        self._on_toggle_mode = on_toggle_mode
        self._on_instruction = on_instruction
        self._on_voice = on_voice
        self._on_halt = on_halt

        # 세로 9.2 인치. 8.0 이면 **지시 패널이 통째로 화면 밖으로 나간다**
        # (실측 y0=-0.051). 패널 높이는 안에 들어갈 글자를 실제로 재서 정하는데,
        # 비상 정지 버튼이 추가되면서 아래로 밀린 만큼이 반영이 안 됐다.
        self.fig, self.ax = plt.subplots(figsize=(6.0, 9.2))
        try:
            self.fig.canvas.manager.set_window_title("Live Map")
        except Exception:
            pass

        self._setup_static()

        self._robot_marker = self.ax.scatter(
            [], [], s=130, facecolor="red", edgecolor="black", linewidth=0.8, zorder=8)

        # 차량 몸체 충돌반경(mission_config.ROBOT_RADIUS_M) 을 원으로 같이
        # 그린다 — 이동하면서 이 원이 다른 기물과 안 겹치는지 눈으로 바로
        # 확인하려는 용도(navigator.py 의 회피 계산이 실제로 쓰는 반경과 같음).
        # 반경이 둘인 이유는 mission_config 참고 — 하단부는 기물에, 상단 암은
        # 벽에 걸린다. navigator 가 쓰는 값과 같다. (팀원 브랜치는 이 분리
        # 이전이라 ROBOT_RADIUS_M 하나만 있었다.)
        self._robot_radius = patches.Circle(
            (0.0, 0.0), mcfg.ROBOT_RADIUS_PIECE_M, fill=False, edgecolor="red",
            linewidth=1.0, linestyle=":", alpha=0.7, zorder=7, visible=False)
        self.ax.add_patch(self._robot_radius)
        self._robot_radius_wall = patches.Circle(
            (0.0, 0.0), mcfg.ROBOT_RADIUS_WALL_M, fill=False, edgecolor="red",
            linewidth=0.8, linestyle="--", alpha=0.35, zorder=7, visible=False)
        self.ax.add_patch(self._robot_radius_wall)

        # 로봇 -> (회피 경유점) -> 목표 경로선. 매 사이클 navigator 가 새로 낸
        # 값으로 갱신한다 — 전역 경로가 아니라 "지금 이 순간의 최단 경로"다.
        self._path_line, = self.ax.plot(
            [], [], color="#2f80ed", linewidth=2.5, alpha=0.85, zorder=4, solid_capstyle="round")

        self._piece_slots: dict[str, list[_PieceSlot]] = {
            label: [_PieceSlot(self.ax, label) for _ in range(mcfg.PIECE_MAX_PER_LABEL)]
            for label in KNOWN_LABELS
        }

        self._frame = 0

        self._build_layout()

        # --- 블리팅 준비 -------------------------------------------------
        # 배경(눈금·격자·상자·범례·버튼)을 한 번만 그려 캐시하고, 매 사이클엔
        # 움직이는 것만 그 위에 얹는다. 순서가 곧 그리는 순서다 — 블리팅에서는
        # zorder 가 자동 정렬되지 않는다.
        self._dynamic = [
            self._path_line,                                    # zorder 4
            *[a for slots in self._piece_slots.values()
              for slot in slots for a in slot._artists],        # 6
            self._robot_radius, self._robot_radius_wall,        # 7
            self._robot_marker,                                 # 8
            self._status_text,
            self._total_text,
            self._ready_light,      # figure 아티스트 — 조건 표시등 색
        ]
        # 범례는 **일부러 여기 없다.** 그리는 데 96ms 가 드는데(마커에 유니코드
        # 체스 기호를 mathtext 로 넣어서 그렇다) 실제로 바뀌는 것은 "xN" 숫자뿐이고
        # 그 값은 PieceTracker 의 hold/confirm 지연 덕에 초 단위로 안정적이다.
        # 배경에 두고 **숫자가 실제로 바뀐 사이클에만** 배경을 다시 뜬다.
        self._legend_cache = None

        # 버튼도 배경에 둔다 — 평소 색으로 구워 놓고, 마우스가 올라가 색이
        # 바뀐 것만 _blit() 에서 그 위에 다시 그린다. 매 사이클 전부 그리면
        # 렌더가 눈에 띄게 늘지만(실측 25.1 -> 38.0ms), 이 방식은 안 올라가
        # 있을 때 그릴 게 0개라 비용이 없다.
        self._buttons = [b for b in (self._mode_button, self._prev_button,
                                     self._next_button, self._reset_button,
                                     self._halt_button, self._mic_button,
                                     self._send_button) if b is not None]
        self._button_idle_rgba = [to_rgba(b.color) for b in self._buttons]

        self._use_blit = blit
        self._bg = None
        # 창 크기가 바뀌면 캐시한 배경의 픽셀 크기가 안 맞는다.
        self.fig.canvas.mpl_connect("resize_event", lambda _e: self._invalidate_bg())

        plt.show(block=False)
        self.fig.canvas.draw()

    # ------------------------------------------------------------------
    # 버튼/입력 콜백
    # ------------------------------------------------------------------
    def _on_next_clicked(self, event) -> None:
        if self._on_next is not None:
            self._on_next()

    def _on_prev_clicked(self, event) -> None:
        if self._on_back is not None:
            self._on_back()

    def _on_mode_clicked(self, event) -> None:
        if self._on_toggle_mode is not None:
            self._on_toggle_mode()

    def _on_reset_clicked(self, event) -> None:
        """리셋 버튼 콜백. 화면 자체를 지우고, 있으면 상위(run_mission.py) 상태도 같이 지운다."""
        self._frame = 0
        for slots in self._piece_slots.values():
            for slot in slots:
                slot.hide()
        self._path_line.set_visible(False)
        self._status_text.set_text("")
        self._ready_light.set_facecolor("lightgray")
        self.fig.canvas.draw_idle()
        if self._on_reset is not None:
            self._on_reset()

    def _on_halt_clicked(self, event) -> None:
        """비상 정지 버튼. Reset 을 누르기 전까진 아무것도 못 되돌린다 —
        실수로 바로 재개되는 걸 막으려고 일부러 별도 "재개" 버튼을 안 뒀다."""
        if self._on_halt is not None:
            self._on_halt()
        else:
            print("[live_map] 비상 정지 눌림(아직 미연결)")

    def _on_mic_clicked(self, event) -> None:
        if self._on_voice is not None:
            self._on_voice()
        else:
            print("[live_map] 음성 버튼 눌림(아직 음성 인식 미연결)")

    def _on_send_clicked(self, event) -> None:
        self._submit_instruction(self._instr_textbox.text)

    def _on_instruction_submit(self, text: str) -> None:
        # TextBox 는 Enter 로도 on_submit(text) 을 그대로 불러준다.
        self._submit_instruction(text)

    def _submit_instruction(self, text: str) -> None:
        text = text.strip()
        if not text:
            return
        if self._on_instruction is not None:
            self._on_instruction(text)
        else:
            print(f"[live_map] 지시 입력됨(아직 처리 로직 미연결): {text}")
            self.set_instruction_feedback("(아직 Claude API 미연결 — 콘솔 로그만 확인 가능)", ok=False)
        # 빈 문자열로 되돌린다. set_val 이 submit 을 다시 부르지만 위의
        # `if not text: return` 이 받아 내므로 되돌이가 안 생긴다.
        self._instr_textbox.set_val("")
        self._invalidate_bg()

    def set_instruction_text(self, text: str) -> None:
        """음성 인식(STT) 결과 등을 입력창에 채워 넣을 때 호출 쪽에서 쓴다.
        일부러 자동 전송은 안 함 — 인식이 틀렸을 수 있으니 사용자가 보고
        고친 뒤 직접 전송 버튼을 누르게 하기 위함(voice_input.py 참고).

        ⚠️ **eventson 을 꺼야 한다.** matplotlib 의 TextBox.set_val() 은 값을
        넣으면서 submit 옵저버를 그대로 부른다(widgets.py 참고) — 그냥
        set_val() 하면 음성 인식 결과가 **자동으로 전송된다.** 그러면
        "사람이 보고 고친 뒤 보낸다"는 이 함수의 존재 이유가 사라진다
        (Whisper 가 "기물"을 "김을"로 듣는 오인식이 실측으로 확인됐다)."""
        self._instr_textbox.eventson = False
        try:
            self._instr_textbox.set_val(text)
        finally:
            self._instr_textbox.eventson = True
        self._invalidate_bg()   # 입력창 글자는 배경에 구워져 있다

    def set_mic_recording(self, recording: bool) -> None:
        """음성 버튼 모양을 "녹음 중" 상태로 바꾼다 — voice_input.VoiceRecorder.toggle()
        의 반환값을 그대로 넘기면 된다."""
        if recording:
            self._mic_button.label.set_text("● 녹음중")
            self._mic_button.color = "#ff5252"
            self._mic_button.hovercolor = "#e53935"
        else:
            self._mic_button.label.set_text("음성")
            self._mic_button.color = "#ffe0e0"
            self._mic_button.hovercolor = "#ffb3b3"
        self._mic_button.ax.set_facecolor(self._mic_button.color)
        # 평소 색이 바뀌었으니 hover 판정 기준도 같이 갱신한다 — 안 하면
        # _blit() 이 이 버튼을 "계속 hover 중"으로 보고 매 사이클 다시 그린다.
        if self._mic_button in self._buttons:
            self._button_idle_rgba[self._buttons.index(self._mic_button)] =                 to_rgba(self._mic_button.color)
        self._invalidate_bg()
        self.fig.canvas.draw_idle()

    def set_instruction_feedback(self, text: str, ok: bool = True) -> None:
        """Claude 가 지시를 어떻게 이해했는지(또는 실패했는지) 지시 패널
        맨 아래 줄에 보여준다 — 실행 전에 오인식을 바로 확인할 수 있게
        하기 위함. ok=False 면 빨간색으로 표시한다."""
        self._instr_feedback.set_text(text)
        self._instr_feedback.set_color("#1a6b1a" if ok else "#c0392b")
        self._invalidate_bg()   # 피드백 줄도 배경이다
        self.fig.canvas.draw_idle()

    # ------------------------------------------------------------------
    # 정적 지도 요소(축/워크스페이스/마커/상자)
    # ------------------------------------------------------------------
    def _setup_static(self) -> None:
        self.ax.set_xlim(0, ROOM_SIZE)
        self.ax.set_ylim(0, ROOM_SIZE)
        self.ax.set_aspect("equal")
        self.ax.set_title("Top-down Map", fontsize=15, fontweight="bold", pad=12)
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.tick_params(direction="in", labelsize=8, colors="#555555")

        # 로봇 주행 가능 범위 (점선)
        wx0, wx1 = cfg.WORKSPACE_X
        wy0, wy1 = cfg.WORKSPACE_Y
        self.ax.add_patch(patches.Rectangle(
            (wx0, wy0), wx1 - wx0, wy1 - wy0,
            fill=False, edgecolor="#8fa6c9", linestyle="--", linewidth=1))

        # 바닥 기준 ArUco 마커 4점 (참고용)
        h = cfg.FLOOR_MARKER_SIZE / 2.0
        for mid, (mx, my) in cfg.FLOOR_MARKER_WORLD.items():
            self.ax.add_patch(patches.Rectangle(
                (mx - h, my - h), cfg.FLOOR_MARKER_SIZE, cfg.FLOOR_MARKER_SIZE,
                facecolor="none", edgecolor="#2e8b57", linewidth=1))
            self.ax.text(mx, my - h - 0.03, str(mid), ha="center", va="top",
                         fontsize=7, color="#2e8b57")

        # 상자 (고정 좌표, BOXES 의 yaw 는 항상 0/180 이라 축정렬 사각형으로 충분)
        # 이름표는 상자 위가 아니라 안쪽에 — 위쪽은 방(room) 경계와 딱 붙어 있어서
        # 제목/범례와 겹치기 쉽다.
        for name, (bx, by, _byaw) in cfg.BOXES.items():
            self.ax.add_patch(patches.Rectangle(
                (bx - cfg.BOX_W / 2, by - cfg.BOX_L / 2), cfg.BOX_W, cfg.BOX_L,
                facecolor="#c9a88c", edgecolor="#8d6748", alpha=0.95))
            self.ax.text(bx, by, name, ha="center", va="center",
                         fontsize=9, color="white", weight="bold")

    # ------------------------------------------------------------------
    # 지도 아래 UI(상태정보 / 범례 / 조작 / 지시) 배치
    # ------------------------------------------------------------------
    def _build_layout(self) -> None:
        """지도 아래에 상태정보/범례/조작(버튼)/지시(자연어 명령) 4개
        패널을 배치한다.

        지도가 정사각형 비율(set_aspect("equal"))이라, 실제로 그려지는
        지도 박스는 subplots_adjust 로 예약한 사각 영역보다 좁게 중앙
        정렬된다(가로/세로 중 짧은 쪽에 맞춰짐) — 한 번 그려서 실제 좌표를
        픽셀 단위로 측정한 뒤 그 값에 맞춰 패널 폭을 잡아야 지도 폭을 안
        넘어간다. 마찬가지로 각 패널의 높이도 고정값 대신, 안에 들어갈
        텍스트/범례/버튼을 먼저 그린 뒤 실제로 어디까지 내려오는지 측정해서
        정한다(빈 여백을 최소화하기 위함).
        """
        # bottom 은 지도가 아니라 **패널들이 쓸 아래쪽 몫**이다. 0.53 이면
        # 지시 패널이 화면 밖으로 나간다(실측 피드백 줄 y=-0.065). 비상 정지
        # 버튼이 늘어난 만큼 아래가 모자란다. 0.60 + 세로 9.2 조합이
        # 지도를 가장 크게 남기면서(0.36) 패널이 다 들어가는 값이다.
        self.fig.subplots_adjust(left=0.13, right=0.87, top=0.965, bottom=0.60)
        self.fig.canvas.draw()
        renderer = self.fig.canvas.get_renderer()
        map_bbox = self.ax.get_window_extent(renderer=renderer).transformed(
            self.fig.transFigure.inverted())
        map_left, map_right, map_bottom = map_bbox.x0, map_bbox.x1, map_bbox.y0

        gap = 0.03
        col_w = (map_right - map_left - gap) / 2
        right_x0, right_x1 = map_left + col_w + gap, map_right
        # 상태정보(좌측) 칸만 우측 변을 살짝 줄여서 범례와의 간격을 0.035 로.
        left_x0, left_x1 = map_left, right_x0 - 0.035

        top_gap = 0.07   # xtick 라벨/xlabel("x (m)") 이 지도 박스 밑으로
                         # 튀어나오므로 그걸 피할 만큼의 간격
        estop_bottom = self._build_halt_button(left_x0, right_x1, map_bottom - top_gap)

        info_gap = 0.03
        panel_top = estop_bottom - info_gap

        self._build_info_and_legend(left_x0, left_x1, right_x0, right_x1, panel_top)
        btn_y0 = self._build_controls(left_x0, right_x1, self._info_y0)
        self._build_instruction_panel(left_x0, right_x1, btn_y0)

    def _build_halt_button(self, left_x0: float, right_x1: float, top: float) -> float:
        """지도 바로 아래, 폭 전체를 차지하는 빨간 "비상 정지" 바.

        다른 버튼(조작 패널 안의 Reset 등)과 섞어두면 실수로 못 찾거나
        비상 상황에 헤맬 수 있어서, 일부러 지도 바로 아래 가장 먼저 보이는
        자리에 크고 눈에 띄게 따로 둔다. 반환값은 이 버튼 아래쪽 y 좌표
        (다음 패널이 시작할 위치)다."""
        h = 0.05
        y0 = top - h
        self._halt_button_ax = self.fig.add_axes(
            [left_x0, y0, right_x1 - left_x0, h], zorder=5)
        self._halt_button = Button(self._halt_button_ax, "■ 비상 정지",
                                   color="#e53935", hovercolor="#c62828")
        self._halt_button.label.set_fontsize(11)
        self._halt_button.label.set_fontweight("bold")
        self._halt_button.label.set_color("white")
        self._halt_button.on_clicked(self._on_halt_clicked)
        return y0

    def _build_info_and_legend(self, left_x0: float, left_x1: float,
                               right_x0: float, right_x1: float,
                               panel_top: float) -> None:
        # ── 상태 정보 ──────────────────────────────────────────────
        self.fig.text(left_x0 + 0.012, panel_top - 0.014, "상태 정보", fontsize=9,
                      fontweight="bold", color="#333333", va="top", ha="left")
        # 상태 정보는 **라벨 칸과 값 칸을 나눈다.** 한 덩어리 Text 로 두면
        # 매 사이클 8줄 전체를 다시 래스터라이즈하는데, 실측으로 그것만
        # 43ms 가 들었다(블리팅 한 프레임 66ms 중 65%). 라벨은 절대 안 바뀌니
        # 배경에 굽고, 값만 동적 아티스트로 얹는다.
        _FONTS = ["Malgun Gothic", "DejaVu Sans Mono", "DejaVu Sans"]
        self._status_labels = self.fig.text(
            left_x0 + 0.012, panel_top - 0.036,
            "\n".join(STATUS_ROWS), va="top", ha="left",
            fontsize=7.3, linespacing=1.6, color="#777777", fontfamily=_FONTS)
        # 값 칸은 라벨 칸의 **실제 렌더 폭**을 재서 그 오른쪽에 둔다. 고정값
        # (0.098)으로 두면 "battery(veh)" 같은 긴 라벨과 겹친다 — 실측으로
        # 라벨이 0.314 까지 가는데 값이 0.308 에서 시작해 겹쳤다.
        self.fig.canvas.draw()
        _lb = (self._status_labels.get_window_extent(self.fig.canvas.get_renderer())
               .transformed(self.fig.transFigure.inverted()))
        self._status_text = self.fig.text(
            _lb.x1 + 0.014, panel_top - 0.036, "", va="top", ha="left",
            fontsize=7.3, linespacing=1.6, fontfamily=_FONTS)

        # ── 범례: 2열 그리드, 라벨별 개수(xN) + Total 표시 ─────────────
        self.fig.text(right_x0 + 0.012, panel_top - 0.014, "범례", fontsize=9,
                      fontweight="bold", color="#333333", va="top", ha="left")

        widest_suffix = f" x{mcfg.PIECE_MAX_PER_LABEL}"

        def _icon_handle(label: str) -> Line2D:
            wide_label = label + widest_suffix
            if label in GLYPHS:
                return Line2D([0], [0], marker=f"${GLYPHS[label]}$", color="none",
                              markerfacecolor="black", markersize=8.5, label=wide_label)
            if label == "box":
                return Line2D([0], [0], marker="s", color="none", markerfacecolor="white",
                              markeredgecolor="black", markersize=7, label=wide_label)
            return Line2D([0], [0], marker="o", color="none", markerfacecolor="lightgray",
                          markeredgecolor="black", markersize=7, label=wide_label)

        col1 = [Line2D([0], [0], marker=">", color="none", markerfacecolor="red",
                       markeredgecolor="black", markersize=7, label="robot")]
        col1 += [_icon_handle(label) for label in LEGEND_COL1]
        col2 = [Line2D([0], [0], color="#2f80ed", linewidth=2, label="path")]
        col2 += [_icon_handle(label) for label in LEGEND_COL2]

        legend = self.fig.legend(
            handles=col1 + col2, ncol=2, loc="upper left",
            bbox_to_anchor=(right_x0 + 0.002, panel_top - 0.036),
            bbox_transform=self.fig.transFigure, frameon=False, fontsize=6.8,
            labelspacing=1.9, columnspacing=1.4, handletextpad=0.45, handlelength=1.0)
        self._legend = legend
        self._legend_texts = legend.get_texts()
        self._legend_row_keys = ["robot"] + LEGEND_COL1 + ["path"] + LEGEND_COL2

        # ── 실측: 상태정보 텍스트/범례가 실제로 어디까지 내려오는지 재서
        #    패널 높이를 그 내용에 딱 맞춘다(빈 공간 최소화). Total 은
        #    범례 바로 아래 붙인다.
        self.fig.canvas.draw()
        renderer = self.fig.canvas.get_renderer()
        status_bbox = self._status_text.get_window_extent(renderer=renderer).transformed(
            self.fig.transFigure.inverted())
        legend_bbox = legend.get_window_extent(renderer=renderer).transformed(
            self.fig.transFigure.inverted())

        self._total_text = self.fig.text(
            right_x0 + 0.012, legend_bbox.y0 - 0.022, "", fontsize=7,
            color="#222222", va="top", ha="left")
        self.fig.canvas.draw()
        total_bbox = self._total_text.get_window_extent(renderer=renderer).transformed(
            self.fig.transFigure.inverted())

        self._info_y0 = min(status_bbox.y0, total_bbox.y0) - 0.018
        info_y1 = panel_top

        self.fig.add_artist(patches.FancyBboxPatch(
            (left_x0, self._info_y0), left_x1 - left_x0, info_y1 - self._info_y0,
            transform=self.fig.transFigure, boxstyle="round,pad=0.01,rounding_size=0.018",
            facecolor="#f0f0f0", edgecolor="#dddddd", linewidth=1.0, zorder=-10))
        self.fig.add_artist(patches.FancyBboxPatch(
            (right_x0, self._info_y0), right_x1 - right_x0, info_y1 - self._info_y0,
            transform=self.fig.transFigure, boxstyle="round,pad=0.01,rounding_size=0.018",
            facecolor="#f0f0f0", edgecolor="#dddddd", linewidth=1.0, zorder=-10))

    def _build_controls(self, left_x0: float, right_x1: float, info_y0: float) -> float:
        """"조작" 패널(Mode/Reset/Prev/Next + 표시등)을 만들고, 그 아래
        (다음 패널이 시작할) y 좌표를 반환한다."""
        btn_gap = 0.035
        btn_y1 = info_y0 - btn_gap
        btn_y0 = btn_y1 - 0.075

        self.fig.add_artist(patches.FancyBboxPatch(
            (left_x0, btn_y0), right_x1 - left_x0, btn_y1 - btn_y0,
            transform=self.fig.transFigure, boxstyle="round,pad=0.01,rounding_size=0.018",
            facecolor="#f0f0f0", edgecolor="#dddddd", linewidth=1.0, zorder=-10))
        self.fig.text((left_x0 + right_x1) / 2, btn_y1 - 0.014, "조작", fontsize=9,
                      fontweight="bold", color="#333333", va="top", ha="center")

        row_h = 0.026
        row_y = btn_y0 + 0.014
        gap_x = 0.014
        light_w = 0.022
        width_scale = 0.72   # 버튼 묶음이 패널 폭 전체를 안 채우고 가운데
                             # 정렬되게 — 다 채우면 옆으로 너무 길어 보임.
        total_w = (right_x1 - 0.012) - (left_x0 + 0.012)
        content_w = total_w * width_scale
        btn_w = (content_w - gap_x * 4 - light_w) / 4
        x = left_x0 + 0.012 + (total_w - content_w) / 2

        def add_btn(w: float, label: str, color: str, hover: str, cb) -> tuple:
            nonlocal x
            ax = self.fig.add_axes([x, row_y, w, row_h], zorder=5)
            btn = Button(ax, label, color=color, hovercolor=hover)
            btn.label.set_fontsize(6.3)
            btn.label.set_fontweight("bold")
            btn.on_clicked(cb)
            x += w + gap_x
            return ax, btn

        self._mode_button_ax, self._mode_button = add_btn(
            btn_w, "AUTO", "#b2ebf2", "#80deea", self._on_mode_clicked)
        self._reset_button_ax, self._reset_button = add_btn(
            btn_w, "Reset", "#ffcdd2", "#ef9a9a", self._on_reset_clicked)
        self._prev_button_ax, self._prev_button = add_btn(
            btn_w, "Prev", "#e0e0e0", "#bdbdbd", self._on_prev_clicked)
        self._next_button_ax, self._next_button = add_btn(
            btn_w, "Next", "#fff59d", "#ffe082", self._on_next_clicked)

        self._ready_light = patches.Circle(
            (x + light_w / 2 - gap_x / 2, row_y + row_h / 2), 0.009,
            transform=self.fig.transFigure, facecolor="lightgray",
            edgecolor="black", linewidth=0.5, zorder=20)
        self.fig.add_artist(self._ready_light)

        return btn_y0

    def _build_instruction_panel(self, left_x0: float, right_x1: float,
                                 btn_y0: float) -> None:
        """"지시" 패널 — 자연어 문장 입력창 + 음성 버튼 + 전송 버튼 +
        (Claude 가 뭘로 이해했는지 보여주는) 피드백 줄."""
        instr_gap = 0.035
        instr_y1 = btn_y0 - instr_gap
        self.fig.text((left_x0 + right_x1) / 2, instr_y1 - 0.014, "지시", fontsize=9,
                      fontweight="bold", color="#333333", va="top", ha="center")

        instr_row_y = instr_y1 - 0.078
        instr_row_h = 0.034
        pad_x = 0.012
        gap_x = 0.012
        total_w = (right_x1 - pad_x) - (left_x0 + pad_x)
        mic_w = 0.075
        send_w = 0.075
        text_w = total_w - mic_w - send_w - gap_x * 2

        textbox_ax = self.fig.add_axes(
            [left_x0 + pad_x, instr_row_y, text_w, instr_row_h], zorder=5)
        self._instr_textbox = TextBox(textbox_ax, "", initial="",
                                      color="#ffffff", hovercolor="#ffffff")
        self._instr_textbox.text_disp.set_fontsize(7.5)
        self._instr_textbox.text_disp.set_fontfamily(["Malgun Gothic", "DejaVu Sans"])
        self._instr_textbox.on_submit(self._on_instruction_submit)

        mic_ax = self.fig.add_axes(
            [left_x0 + pad_x + text_w + gap_x, instr_row_y, mic_w, instr_row_h], zorder=5)
        self._mic_button = Button(mic_ax, "음성", color="#ffe0e0", hovercolor="#ffb3b3")
        self._mic_button.label.set_fontsize(7)
        self._mic_button.label.set_fontweight("bold")
        self._mic_button.on_clicked(self._on_mic_clicked)

        send_ax = self.fig.add_axes(
            [left_x0 + pad_x + text_w + gap_x * 2 + mic_w, instr_row_y, send_w, instr_row_h],
            zorder=5)
        self._send_button = Button(send_ax, "전송", color="#c8e6c9", hovercolor="#a5d6a7")
        self._send_button.label.set_fontsize(7)
        self._send_button.label.set_fontweight("bold")
        self._send_button.on_clicked(self._on_send_clicked)

        # 예시 문구를 힌트로 보여주다가, 실제 지시가 들어오면
        # set_instruction_feedback() 이 같은 자리를 덮어써서 보여준다.
        self._instr_feedback = self.fig.text(
            left_x0 + pad_x, instr_row_y - 0.014,
            "예: 자유롭게 움직이는 기물 잡아줘 (입력 후 Enter 또는 전송)",
            fontsize=7, color="#888888", va="top", ha="left",
            fontfamily=["Malgun Gothic", "DejaVu Sans"])

        self.fig.canvas.draw()
        fb_bbox = self._instr_feedback.get_window_extent(
            renderer=self.fig.canvas.get_renderer()).transformed(
            self.fig.transFigure.inverted())
        instr_y0 = fb_bbox.y0 - 0.018

        self.fig.add_artist(patches.FancyBboxPatch(
            (left_x0, instr_y0), right_x1 - left_x0, instr_y1 - instr_y0,
            transform=self.fig.transFigure, boxstyle="round,pad=0.01,rounding_size=0.018",
            facecolor="#f0f0f0", edgecolor="#dddddd", linewidth=1.0, zorder=-10))

    # ------------------------------------------------------------------
    # 매 사이클 갱신
    # ------------------------------------------------------------------
    def update(self, pose: Pose, pmap: PieceMap,
               goal: Optional[XY] = None, nav: Optional[DriveCommand] = None,
               corner: Optional[XY] = None,
               state_name: Optional[str] = None,
               target_label: Optional[str] = None,
               ready: Optional[bool] = None,
               manual_mode: bool = False,
               cmd: Optional[str] = None,
               path: Optional[list] = None,
               battery_veh: Optional[float] = None,
               battery_arm: Optional[float] = None) -> None:
        """매 사이클 한 번씩 부른다. 논블로킹.

        goal/nav 는 mission.MissionFSM 이 이번 사이클에 계산한 "지금 이동
        단계의 최종 목표"와 DriveSequencer 의 이번 명령이다(mission.py 의
        fsm.nav_goal / fsm.last_nav). 이동 중이 아니면(GRASP/PLACE/대기)
        다 None 이라 경로선을 지운다.

        corner 는 축정렬("ㄱ자") 경로가 꺾이는 모서리 — 두 축 다 이미
        맞았으면(직진 한 구간만 남았으면) None 이라 로봇→목표 직선 하나만
        그린다(fsm.nav_corner).

        state_name/target_label 은 지금 미션이 어느 단계(SEARCH_TARGET 등)
        인지, 어떤 기물을 다루고 있는지 화면에 표시하기 위한 것 — 실제
        차량 없이 마커를 손으로 옮기며 시험할 때 지금 뭘 하는 중인지 눈으로
        바로 확인하려는 용도다.

        ready 는 Next 버튼 옆 표시등 색이다 — True 면 초록(다음 단계로
        넘어갈 조건 충족), False 면 빨강, None 이면 회색(로봇을 잃었거나
        판단할 게 없음. fsm.ready_to_advance).

        manual_mode 는 Mode 버튼에 지금 모드를 글자로 보여주기 위한 것
        (fsm.manual_mode) — 이 클래스는 모드를 직접 못 바꾸고 표시만 한다.

        cmd 는 이번 사이클에 실제로 차량에 보낸 신호("go"/"stop"/"yaw+"/
        "yaw-", fsm.last_cmd) — vehicle_link.MissionCommand.cmd 와 정확히
        같은 값이라, 화면에서 보는 것과 실제로 전송되는 것이 항상 일치한다.

        battery_veh/battery_arm 은 차량/로봇팔 배터리 잔량(%) — 아직 Pi
        쪽에서 Host 로 보내주는 데이터가 없어서 항상 None 이 기본값이고,
        그럴 땐 "—" 로 표시한다(연동되면 그대로 값을 넘기면 됨).
        """
        self._frame += 1

        mode_label = "MANUAL" if manual_mode else "AUTO"
        if self._mode_button.label.get_text() != mode_label:
            self._mode_button.label.set_text(mode_label)
            self._invalidate_bg()   # 버튼 글자는 배경에 구워져 있다

        if ready is None:
            self._ready_light.set_facecolor("lightgray")
        else:
            self._ready_light.set_facecolor("limegreen" if ready else "red")

        wx0, wx1 = cfg.WORKSPACE_X
        wy0, wy1 = cfg.WORKSPACE_Y
        counts: dict[str, int] = {}
        for label in KNOWN_LABELS:
            # 작업 영역 밖 관측은 표시하지 않는다 — y 밖은 상자 자리 쪽
            # 오검출, x 밖(방 폭 0~1.8m 밖)은 물리적으로 있을 수 없는
            # 자리라 오검출이 대부분이라 화면에 띄우면 헷갈린다.
            pts = [p for p in pmap.get(label, [])
                   if wx0 <= p[0] <= wx1 and wy0 <= p[1] <= wy1]
            counts[label] = len(pts)
            slots = self._piece_slots[label]
            for i, slot in enumerate(slots):
                if i < len(pts):
                    slot.set_pos(*pts[i])
                else:
                    slot.hide()

        # 범례 오른쪽에 개수 표시 — 없으면 이름만(여백), 있으면 "이름 xN".
        # Total 은 범례와 별도 Text 로, 매 사이클 합계로 갱신.
        # 범례는 배경에 구워져 있다. 숫자가 실제로 바뀐 사이클에만 다시 뜬다 —
        # 매 사이클 그리면 96ms 가 든다(마커의 유니코드 체스 기호 mathtext).
        if counts != self._legend_cache:
            for text, key in zip(self._legend_texts, self._legend_row_keys):
                cnt = counts.get(key)
                if cnt is not None:
                    text.set_text(f"{key} x{cnt}" if cnt else key)
            self._legend_cache = dict(counts)
            self._invalidate_bg()
        # Total 은 별도 Text 라 동적 아티스트로 매 사이클 얹는다(값싸다).
        self._total_text.set_text(f"Total: {sum(counts.values())}")

        if pose.ok:
            self._robot_marker.set_offsets([[pose.x, pose.y]])
            t = Affine2D().rotate_deg(pose.yaw_deg)
            self._robot_marker.set_paths([_ROBOT_ARROW.transformed(t)])
            color = "red" if pose.fresh else "orange"
            self._robot_marker.set_facecolor(color)
            for circ in (self._robot_radius, self._robot_radius_wall):
                circ.set_center((pose.x, pose.y))
                circ.set_edgecolor(color)
                circ.set_visible(True)
        else:
            self._robot_marker.set_offsets(np.empty((0, 2)))
            self._robot_radius.set_visible(False)
            self._robot_radius_wall.set_visible(False)

        if pose.ok and goal is not None and nav is not None:
            # 계획기(GridPathPlanner)가 낸 경로를 그대로 그린다. 경로의 첫 점은
            # 격자 칸 중심이라 로봇 위치와 최대 반 칸 어긋나므로 실제 pose 로 잇는다.
            if path:
                xs = [pose.x] + [q[0] for q in path[1:]]
                ys = [pose.y] + [q[1] for q in path[1:]]
            else:
                # 예비 — 계획기가 경로를 안 냈을 때(이미 도착 거리 안 등).
                xs = [pose.x, nav.waypoint[0]]
                ys = [pose.y, nav.waypoint[1]]
                if corner is not None:
                    xs.append(corner[0]); ys.append(corner[1])
                xs.append(goal[0]); ys.append(goal[1])
            self._path_line.set_data(xs, ys)
            self._path_line.set_linestyle("--" if nav.blocked_by else "-")
            self._path_line.set_visible(True)
        else:
            self._path_line.set_visible(False)

        # 행 순서는 STATUS_ROWS 와 반드시 같다 — 라벨 칸이 배경에 고정이라
        # 여기서 한 줄이라도 빼면 아래 줄들이 다른 라벨에 붙는다.
        values = [
            f"{pose.x * 1000:.1f}mm" if pose.ok else "LOST",
            f"{pose.y * 1000:.1f}mm" if pose.ok else "-",
            f"{pose.yaw_deg:.1f}°" if pose.ok else "-",
            state_name or "-",
            target_label or "-",
            cmd or "-",
            f"{battery_veh:.0f}%" if battery_veh is not None else "—",
            f"{battery_arm:.0f}%" if battery_arm is not None else "—",
        ]
        self._status_text.set_text("\n".join(values))

        if self._use_blit:
            self._blit()
        else:
            # 예비 경로. plt.pause 는 뺐다 — flush_events() 가 이미 그린 뒤라
            # 같은 그림을 한 번 더 그리는 순수 낭비였다(실측 181.8ms).
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    # -- 블리팅 -----------------------------------------------------------

    def _invalidate_bg(self) -> None:
        """다음 사이클에 배경을 다시 캐시하게 한다.

        창 크기 변경, 버튼 글자 변경, 범례 숫자 변경, 지시 패널 글자 변경처럼
        **배경에 속한 것이 바뀐** 때 부른다. 안 부르면 낡은 배경 위에 새 것을
        얹어 글자가 겹쳐 보인다."""
        self._bg = None

    def _capture_bg(self) -> None:
        """동적 아티스트를 빼고 한 번 그려서 배경을 캐시한다."""
        for a in self._dynamic:
            a.set_animated(True)
        self.fig.canvas.draw()
        self._bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)

    def _blit(self) -> None:
        # 지시 입력창에 타이핑 중이면 블리팅을 쉰다. TextBox 는 글자와 커서를
        # 자기 axes 에 직접 그리는데, 블리팅이 배경을 복원하면서 그걸 지워
        # **입력한 글자가 안 보인다.** 타이핑은 몇 초짜리라 그동안 전체
        # 다시 그리기로 돌아가는 편이 간단하고 확실하다.
        if getattr(self._instr_textbox, "capturekeystrokes", False):
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            self._invalidate_bg()   # 타이핑이 끝나면 새 글자로 다시 굽는다
            return

        if self._bg is None:
            self._capture_bg()
        self.fig.canvas.restore_region(self._bg)
        for a in self._dynamic:
            # figure 아티스트(_ready_light, _status_text)와 axes 아티스트가
            # 섞여 있어서 둘 다 받는 fig.draw_artist 로 통일한다.
            self.fig.draw_artist(a)
        # 마우스가 올라간(=hovercolor 로 바뀐) 버튼만 배경 위에 다시 얹는다.
        # 안 그리면 배경에 구워진 평소 색이 복원되면서 hovercolor 가 한 사이클
        # 만에 지워진다 — "올리면 잠깐 변했다 바로 돌아오는" 증상.
        for button, idle in zip(self._buttons, self._button_idle_rgba):
            if button.ax.get_facecolor() != idle:
                self.fig.draw_artist(button.ax)
        self.fig.canvas.blit(self.fig.bbox)
        self.fig.canvas.flush_events()

    def closed(self) -> bool:
        """사용자가 창을 닫았으면 True."""
        return not plt.fignum_exists(self.fig.number)

    def close(self) -> None:
        plt.close(self.fig)
