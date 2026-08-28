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
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.path import Path as MplPath
from matplotlib.transforms import Affine2D
from matplotlib.widgets import Button

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
LEGEND_ORDER = ["box", "soccer", "star", "queen", "knight", "rook"]

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
                 on_toggle_mode: Optional[Callable[[], None]] = None) -> None:
        """on_reset 은 리셋 버튼, on_next/on_back 은 Next/Prev 버튼(수동
        모드에서 다음/이전 단계로), on_toggle_mode 는 Mode 버튼(자동↔수동
        전환) 콜백이다.

        이 클래스는 자기가 그리는 것(기물 표시·경로선·프레임 카운터)만
        지울 수 있고, PieceTracker/MissionFSM 같은 실제 상태는 모른다 —
        그래서 그쪽까지 건드리고 싶으면 run_mission.py 가
        tracker.reset()/fsm.reset()/fsm.request_advance()/fsm.request_back()/
        fsm.set_manual_mode() 를 부르는 콜백을 여기 넘겨준다. 지금 모드가
        뭔지도 이 클래스는 모르므로, 버튼 글자는 update() 의 manual_mode
        인자로 매 사이클 갱신한다(run_mission.py 가 fsm.manual_mode 를 넘김).
        """
        self._on_reset = on_reset
        self._on_next = on_next
        self._on_back = on_back
        self._on_toggle_mode = on_toggle_mode

        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        try:
            self.fig.canvas.manager.set_window_title("Live Map")
        except Exception:
            pass

        self._setup_static()

        self._robot_marker = self.ax.scatter(
            [], [], s=130, facecolor="red", edgecolor="black", linewidth=0.8, zorder=8)

        # 차량 충돌반경을 원으로 같이 그린다 — 이동하면서 기물/벽과 안 겹치는지
        # 눈으로 바로 확인하려는 용도다. 반경이 둘인 이유는 mission_config 참고
        # (하단부는 기물, 상단 암은 벽에 걸린다). navigator 가 쓰는 값과 같다.
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
            [], [], color="dodgerblue", linewidth=2, alpha=0.8, zorder=4, solid_capstyle="round")

        self._piece_slots: dict[str, list[_PieceSlot]] = {
            label: [_PieceSlot(self.ax, label) for _ in range(mcfg.PIECE_MAX_PER_LABEL)]
            for label in KNOWN_LABELS
        }

        # 좌하단 — 기물/로봇이 있을 수 있는 영역이지만, 우상단(범례·상자)보다는
        # 덜 붐빈다. 배경을 살짝 깔아서 겹쳐도 읽히게 한다.
        self._status_text = self.ax.text(
            0.02, 0.02, "", transform=self.ax.transAxes, va="bottom", ha="left",
            fontsize=8, family="monospace",
            bbox=dict(facecolor="white", alpha=0.7, edgecolor="none", pad=2))

        self._frame = 0

        # Mode 버튼 — 자동↔수동 전환. 누르면 처음부터 다시 시작한다(모드를
        # 도중에 바꾸면 지금 상태가 애매해지므로 mission.py 가 리셋과 같이 묶음).
        self._mode_button_ax = self.fig.add_axes([0.86, 0.24, 0.12, 0.06])
        self._mode_button = Button(self._mode_button_ax, "AUTO",
                                   color="paleturquoise", hovercolor="turquoise")
        self._mode_button.label.set_fontsize(9)
        self._mode_button.on_clicked(self._on_mode_clicked)

        # Prev/Next 버튼(칸을 반씩 나눠 씀) + 조건 표시등 — 수동 모드
        # (run_mission.py --step)에서 씀. 표시등은 조건 충족(ready=True)이면
        # 초록, 아니면 빨강 — Next 쪽 조건이다(Prev 는 조건 없이 항상 동작).
        # figure 전체 기준 좌표(0~1)라 축(ax) 확대/축소와 무관하게 항상
        # 같은 자리에 있다.
        self._prev_button_ax = self.fig.add_axes([0.86, 0.15, 0.055, 0.06])
        self._prev_button = Button(self._prev_button_ax, "Prev",
                                   color="lightsteelblue", hovercolor="cornflowerblue")
        self._prev_button.label.set_fontsize(9)
        self._prev_button.on_clicked(self._on_prev_clicked)

        self._next_button_ax = self.fig.add_axes([0.925, 0.15, 0.055, 0.06])
        self._next_button = Button(self._next_button_ax, "Next",
                                   color="lightyellow", hovercolor="khaki")
        self._next_button.label.set_fontsize(9)
        self._next_button.on_clicked(self._on_next_clicked)
        self._ready_light = patches.Circle(
            (0.985, 0.18), 0.014, transform=self.fig.transFigure,
            facecolor="red", edgecolor="black", linewidth=0.6, zorder=20)
        self.fig.add_artist(self._ready_light)

        # 리셋 버튼 — 범례(우측 바깥) 아래, Next 버튼 아래 빈 공간에 둔다.
        self._reset_button_ax = self.fig.add_axes([0.86, 0.06, 0.12, 0.06])
        self._reset_button = Button(self._reset_button_ax, "Reset",
                                    color="mistyrose", hovercolor="lightcoral")
        self._reset_button.on_clicked(self._on_reset_clicked)

        plt.show(block=False)
        self.fig.canvas.draw()

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

    def _setup_static(self) -> None:
        self.ax.set_xlim(0, ROOM_SIZE)
        self.ax.set_ylim(0, ROOM_SIZE)
        self.ax.set_aspect("equal")
        self.ax.set_title("Top-down Map")
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")

        # 로봇 주행 가능 범위 (점선)
        wx0, wx1 = cfg.WORKSPACE_X
        wy0, wy1 = cfg.WORKSPACE_Y
        self.ax.add_patch(patches.Rectangle(
            (wx0, wy0), wx1 - wx0, wy1 - wy0,
            fill=False, edgecolor="gray", linestyle="--", linewidth=1))

        # 바닥 기준 ArUco 마커 4점 (참고용)
        h = cfg.FLOOR_MARKER_SIZE / 2.0
        for mid, (mx, my) in cfg.FLOOR_MARKER_WORLD.items():
            self.ax.add_patch(patches.Rectangle(
                (mx - h, my - h), cfg.FLOOR_MARKER_SIZE, cfg.FLOOR_MARKER_SIZE,
                facecolor="none", edgecolor="green", linewidth=1))
            self.ax.text(mx, my - h - 0.03, str(mid), ha="center", va="top",
                         fontsize=7, color="green")

        # 상자 (고정 좌표, BOXES 의 yaw 는 항상 0/180 이라 축정렬 사각형으로 충분)
        # 이름표는 상자 위가 아니라 안쪽에 — 위쪽은 방(room) 경계와 딱 붙어 있어서
        # 제목/범례와 겹치기 쉽다.
        for name, (bx, by, _byaw) in cfg.BOXES.items():
            self.ax.add_patch(patches.Rectangle(
                (bx - cfg.BOX_W / 2, by - cfg.BOX_L / 2), cfg.BOX_W, cfg.BOX_L,
                facecolor="saddlebrown", edgecolor="black", alpha=0.6))
            self.ax.text(bx, by, name, ha="center", va="center",
                         fontsize=9, color="white", weight="bold")

        self._build_legend()
        self.fig.tight_layout()

    def _build_legend(self) -> None:
        # row_keys 는 handles 와 같은 순서로 둔다 — update() 에서
        # legend.get_texts() 를 이 순서로 인덱싱해서 글자를 갱신한다.
        # "robot"/"path" 는 개수를 안 붙이고, 맨 끝 "__total__" 은 총합 행이다.
        handles = [
            Line2D([0], [0], marker=">", color="none", markerfacecolor="red",
                  markeredgecolor="black", markersize=10, label="robot"),
            Line2D([0], [0], color="dodgerblue", linewidth=2, label="path"),
        ]
        self._legend_row_keys = ["robot", "path"]
        # 라벨 글자는 처음부터 나올 수 있는 가장 넓은 폭("이름 xN", N=최대
        # 개수)으로 만들어 둔다 — update() 가 나중에 글자만 바꾸면 legend
        # 박스 크기는 처음 그릴 때 한 번만 정해지고 다시 안 재는데, 짧은
        # 글자로 시작해서 나중에 길어지면 박스 밖으로 넘친다.
        widest_suffix = f"  x{mcfg.PIECE_MAX_PER_LABEL}"
        for label in LEGEND_ORDER:
            wide_label = label + widest_suffix
            if label in GLYPHS:
                handles.append(Line2D(
                    [0], [0], marker=f"${GLYPHS[label]}$", color="none",
                    markerfacecolor="black", markersize=13, label=wide_label))
            elif label == "box":
                handles.append(Line2D(
                    [0], [0], marker="s", color="none", markerfacecolor="white",
                    markeredgecolor="black", markersize=10, label=wide_label))
            self._legend_row_keys.append(label)
        # 총합 행 — 마커 없이 글자만(update() 에서 "Total: N" 으로 채움).
        # 두 자리 수까지 여유를 둔다(라벨 6종 x 최대 2개 = 최대 12).
        handles.append(Line2D([0], [0], color="none", label="Total: 00"))
        self._legend_row_keys.append("__total__")

        # 지도 안쪽(상자·기물)과 안 겹치게 축 바깥 오른쪽에 둔다.
        # labelspacing 을 키워서(기본 0.5) 줄 간격을 넓힌다 — 너무 붙어 있어서
        # 잘 안 보인다는 피드백 반영.
        self._legend = self.ax.legend(
            handles=handles, loc="upper left", bbox_to_anchor=(1.02, 1.0),
            borderaxespad=0, fontsize=8, labelspacing=1.3)
        self._legend_texts = self._legend.get_texts()

    def update(self, pose: Pose, pmap: PieceMap,
               goal: Optional[XY] = None, nav: Optional[DriveCommand] = None,
               corner: Optional[XY] = None,
               path: Optional[list] = None,
               state_name: Optional[str] = None,
               target_label: Optional[str] = None,
               ready: Optional[bool] = None,
               manual_mode: bool = False,
               cmd: Optional[str] = None) -> None:
        """매 사이클 한 번씩 부른다. 논블로킹.

        goal/nav 는 mission.MissionFSM 이 이번 사이클에 계산한 "지금 이동
        단계의 최종 목표"와 DriveSequencer 의 이번 명령이다(mission.py 의
        fsm.nav_goal / fsm.last_nav). 이동 중이 아니면(GRASP/PLACE/대기)
        다 None 이라 경로선을 지운다.

        path 는 계획기가 이번 사이클에 낸 전체 경로(fsm.nav_path) — 꺾이는
        점들의 좌표다. 이게 오면 그대로 그린다. corner 는 그 두 번째 점
        하나뿐이라(fsm.nav_corner) path 가 없을 때의 예비값으로만 쓴다 —
        corner 까지만 그리면 그 뒤가 목표까지 직선으로 이어져, 실제로는
        돌아가는 경로가 화면에서는 기물을 뚫고 가는 것처럼 보인다.

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
        """
        self._frame += 1

        self._mode_button.label.set_text("MANUAL" if manual_mode else "AUTO")

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

        # 레전드 오른쪽에 개수 표시 — 없으면 이름만(여백), 있으면 "이름 xN".
        # 맨 아래 행은 전체 합계.
        for text, key in zip(self._legend_texts, self._legend_row_keys):
            if key == "__total__":
                text.set_text(f"Total: {sum(counts.values())}")
            elif key in counts:
                n = counts[key]
                text.set_text(f"{key}  x{n}" if n else key)

        if pose.ok:
            self._robot_marker.set_offsets([[pose.x, pose.y]])
            t = Affine2D().rotate_deg(pose.yaw_deg)
            self._robot_marker.set_paths([_ROBOT_ARROW.transformed(t)])
            color = "red" if pose.fresh else "orange"
            self._robot_marker.set_facecolor(color)
            self._robot_radius.set_center((pose.x, pose.y))
            self._robot_radius_wall.set_center((pose.x, pose.y))
            self._robot_radius.set_edgecolor(color)
            self._robot_radius.set_visible(True)
            self._robot_radius_wall.set_visible(True)
        else:
            self._robot_marker.set_offsets(np.empty((0, 2)))
            self._robot_radius.set_visible(False)
            self._robot_radius_wall.set_visible(False)

        if pose.ok and goal is not None and nav is not None:
            # 계획기가 낸 경로를 그대로 그린다. 경로의 첫 점은 격자 칸
            # 중심이라 로봇 위치와 최대 반 칸 어긋나므로 실제 pose 로 잇는다.
            if path:
                xs = [pose.x] + [q[0] for q in path[1:]]
                ys = [pose.y] + [q[1] for q in path[1:]]
            else:
                # 예비 — 계획기가 경로를 안 냈을 때(이미 도착 거리 안 등)
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

        lines = [f"frame: {self._frame}", str(pose) if pose.ok else "pose: LOST"]
        if state_name is not None:
            step_line = f"state: {state_name}"
            if target_label is not None:
                step_line += f"  ({target_label})"
            if cmd is not None:
                step_line += f"  cmd={cmd}"
            lines.append(step_line)
        lines.append("pieces: " + (
            ", ".join(f"{k}:{len(v)}" for k, v in pmap.items()) if pmap else "(none)"))
        self._status_text.set_text("\n".join(lines))

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def closed(self) -> bool:
        """사용자가 창을 닫았으면 True."""
        return not plt.fignum_exists(self.fig.number)

    def close(self) -> None:
        plt.close(self.fig)
