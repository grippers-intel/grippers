"""여러 창(OpenCV 카메라 창 + matplotlib Live Map)을 지정한 화면에
겹치지 않게 배치한다.

왜 별도 모듈인가: 두 창은 툴킷이 다르다. 카메라 창은 OpenCV highgui 가,
Live Map 은 matplotlib 이 띄운다. 그런데 macOS 에서는 둘 다 결국 같은
프로세스의 NSWindow 라서(실측 확인: `NSApp().windows()` 에 'cam0',
'cam1', 'Live Map' 이 모두 잡힌다) Cocoa 한 경로로 정확히 배치할 수 있다.

툴킷 API 를 쓰지 않는 이유는 둘 다 어긋나기 때문이다.

  * `cv2.moveWindow` 는 요청한 y 보다 25 px 아래에 창을 놓는다(실측).
    화면 맨 위에 붙이려 해도 그만큼 밀린다.
  * matplotlib 의 macosx 백엔드에는 창 위치 API 가 아예 없다.
    TkAgg 의 `wm_geometry` 같은 것이 없고, 이 맥의 venv 에는 tkinter 도
    Qt 도 없어서 백엔드를 바꿀 수도 없다.

좌표계가 둘이라는 점이 이 파일의 핵심이다.

  * Quartz(화면 열거·창 목록): 주 화면 **좌상단**이 원점, y 는 아래로 증가.
  * Cocoa(NSWindow): 주 화면 **좌하단**이 원점, y 는 위로 증가.

뒤집는 기준 높이는 '모든 화면을 덮는 높이'가 아니라 **주 화면의 높이**다.
실측으로 확인했다: 주 화면이 1352x878 이고 오른쪽에 세로 확장 화면
(Quartz 원점 (1352,0), 1080x1920)이 붙어 있을 때 그 화면의 Cocoa origin.y
는 -1042 = 878 - 1920 이다. 전체 데스크톱 높이(1920)로 뒤집으면 창이
1000 px 넘게 어긋난다."""

from __future__ import annotations

MARGIN = 12
# 위쪽 여백만 따로 크게 잡는다. macOS 는 창의 제목 표시줄이 메뉴 막대 띠와
# 겹치면 창을 아래로 밀어 내는데(실측: y=12 로 요청한 창이 y=25 로 내려왔다),
# 이 띠는 Cocoa 전역 y 로 정의되어서 메뉴 막대가 없는 확장 화면에도 같은
# 높이대에 적용된다. 밀려난 창은 아래 창을 침범해 배치가 깨지므로, 애초에
# 그 띠 아래에서 시작한다. NSScreen 이 알려 주는 값들(statusbar 22,
# safeArea 29, visibleFrame 기준 35)이 실측 13 px 이동과 하나도 안 맞아서
# 계산 대신 넉넉한 상수를 쓴다.
MARGIN_TOP = 40
# 창 사이 간격. 위와 같은 이유로, OS 가 창을 몇 px 밀더라도 이웃을 침범하지
# 않을 만큼은 띄워 둔다.
GAP = 16
# 창 제목 표시줄 높이(실측). Cocoa 의 frame 은 제목 표시줄을 포함하고,
# OpenCV/matplotlib 이 다루는 크기는 그 안쪽 영역이다. 빼두지 않으면
# 마지막 창이 화면 아래로 밀려 나간다.
TITLEBAR = 28
# 카메라 창이 가져갈 세로 비중. 나머지가 Live Map 몫이다.
CAM_SHARE = 0.62


def screens() -> list[tuple[int, int, int, int]]:
    """Quartz 좌표(좌상단 원점)로 (x, y, w, h) 목록. 왼쪽 화면부터.

    Quartz 가 없으면 빈 목록 — 호출자는 배치를 포기하고 OS 기본 위치에
    맡겨야 한다."""
    try:
        import Quartz
    except ImportError:
        return []
    _n, ids, _ = Quartz.CGGetActiveDisplayList(8, None, None)
    out = []
    for d in sorted(ids, key=lambda d: Quartz.CGDisplayBounds(d).origin.x):
        b = Quartz.CGDisplayBounds(d)
        out.append((int(b.origin.x), int(b.origin.y),
                    int(b.size.width), int(b.size.height)))
    return out


def plan(display: int, n_cams: int, want_map: bool = True,
         cam_aspect: float = 16.0 / 9.0,
         cam_width: int | None = None) -> dict | None:
    """화면 하나를 잘라 카메라 창과 Live Map 자리를 정한다.

    반환: {"screen": (x,y,w,h), "cams": [(x,y,w,h), ...], "map": (x,y,w,h)|None}
    좌표는 Quartz 전역 좌표이고 크기는 제목 표시줄을 **뺀** 안쪽 크기다.
    화면을 못 읽으면 None.

    `cam_width` 를 주면 카메라 창 가로를 그 값으로 고정하고 배치만
    계산한다(`--cam-width`). 화면보다 크면 화면에 맞춰 줄인다.

    세로 화면(높이>너비)이면 한 줄에 하나씩 세로로 쌓고, 가로 화면이면
    카메라를 위쪽에 나란히 놓고 Live Map 을 그 아래에 둔다. 카메라 창은
    16:9 라 가로로 길어서, 좁은 화면에 나란히 놓으면 둘 다 못 알아볼 만큼
    작아지기 때문이다."""
    scr = screens()
    if not scr:
        return None
    if display >= len(scr):
        display = 0
    sx, sy, sw, sh = scr[display]

    content_w = sw - 2 * MARGIN
    top = sy + MARGIN_TOP
    n_cams = max(n_cams, 1)
    share = CAM_SHARE if want_map else 1.0

    if sh > sw:                                   # 세로 화면 — 한 줄에 하나씩
        rows = n_cams + (1 if want_map else 0)
        content_h = sh - MARGIN_TOP - MARGIN - (rows - 1) * GAP - rows * TITLEBAR
        if cam_width:
            cam_w = cam_width
            cam_h = int(cam_w / cam_aspect)
        else:
            cam_h = int(content_h * share / n_cams)
            cam_w = int(cam_h * cam_aspect)
        if cam_w > content_w:
            cam_w = content_w
            cam_h = int(cam_w / cam_aspect)
        cams, y = [], top
        for _ in range(n_cams):
            cams.append((sx + MARGIN, y, cam_w, cam_h))
            y += cam_h + TITLEBAR + GAP
    else:                                         # 가로 화면 — 위쪽에 나란히
        rows = 2 if want_map else 1
        content_h = sh - MARGIN_TOP - MARGIN - (rows - 1) * GAP - rows * TITLEBAR
        cam_w = cam_width or int((content_w - (n_cams - 1) * GAP) / n_cams)
        if cam_w * n_cams + (n_cams - 1) * GAP > content_w:
            cam_w = int((content_w - (n_cams - 1) * GAP) / n_cams)
        cam_h = int(cam_w / cam_aspect)
        if cam_h > content_h * share:
            cam_h = int(content_h * share)
            cam_w = int(cam_h * cam_aspect)
        cams, x = [], sx + MARGIN
        for _ in range(n_cams):
            cams.append((x, top, cam_w, cam_h))
            x += cam_w + GAP
        y = top + cam_h + TITLEBAR + GAP

    map_rect = None
    if want_map:
        # Live Map 은 정사각 축이라 어느 쪽으로 늘려도 여백만 늘어난다.
        # 남은 자리에 들어가는 가장 큰 정사각형으로 잡는다.
        side = max(240, min(content_w, sy + sh - MARGIN - TITLEBAR - y))
        map_rect = (sx + MARGIN, y, side, side)

    return {"screen": (sx, sy, sw, sh), "cams": cams, "map": map_rect}


def _flip_height() -> float | None:
    """Quartz y 를 Cocoa y 로 뒤집을 기준 높이 = 주 화면 높이."""
    for x, y, _w, h in screens():
        if x == 0 and y == 0:
            return float(h)
    return None


def _move(title: str, rect: tuple[int, int, int, int]) -> bool:
    """제목이 `title` 인 이 프로세스의 Cocoa 창을 Quartz 좌표로 옮긴다.

    `rect` 의 크기는 안쪽 영역 기준이고, 창이 실제로 먹는 자리는 제목
    표시줄만큼 더 크다. 실패해도 예외를 밖으로 내지 않는다 — 창 배치는
    편의 기능이고 이것 때문에 미션이 멈추면 안 된다."""
    try:
        from AppKit import NSApp
    except ImportError:
        return False
    app = NSApp()
    flip = _flip_height()
    if app is None or flip is None:
        return False

    x, y, w, h = rect
    frame_h = h + TITLEBAR
    for win in app.windows():
        try:
            if win.title() != title:
                continue
            frame = win.frame()
            frame.origin.x = float(x)
            frame.origin.y = flip - float(y + frame_h)
            frame.size.width = float(w)
            frame.size.height = float(frame_h)
            win.setFrame_display_(frame, True)
            return True
        except Exception:
            return False
    return False


def apply(layout: dict, cam_names: list[str], fig=None) -> str:
    """계획대로 창을 옮기고, 사람이 읽을 결과 한 줄을 돌려준다.

    ⚠️ 첫 `imshow` **뒤에** 부를 것. OpenCV 는 창에 처음 그림을 넣을 때
    창을 그림 크기로 맞추므로, 그 전에 옮기면 크기가 되돌아간다."""
    moved = []
    for name, rect in zip(cam_names, layout["cams"]):
        if _move(name, rect):
            moved.append(f"{name} {rect[2]}x{rect[3]}@({rect[0]},{rect[1]})")
    if fig is not None and layout["map"] is not None:
        r = layout["map"]
        try:
            dpi = fig.get_dpi()
            fig.set_size_inches(r[2] / dpi, r[3] / dpi, forward=True)
        except Exception:
            pass
        try:
            title = fig.canvas.manager.get_window_title()
        except Exception:
            title = "Live Map"
        if _move(title, r):
            moved.append(f"map {r[2]}x{r[3]}@({r[0]},{r[1]})")
    sx, sy, sw, sh = layout["screen"]
    if not moved:
        return f"[display] 화면 배치 실패 — OS 기본 위치를 씁니다 ({sw}x{sh})"
    return f"[display] {sw}x{sh} 화면에 겹치지 않게 배치: " + ", ".join(moved)
