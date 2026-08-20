"""웹캠 배치가 물체 검출과 위치 정확도에 어떤 한계를 거는지 계산한다.

issue #149 의 D1·D3 근거 자료다. 실행하면 `docs/assets/vision/` 아래 그림 3장을
다시 만들고, 픽셀 민감도 표를 표준출력으로 낸다.

    python tools/a2/coverage_analysis.py

전제
- Logitech C270 · 720p · f = 1411 px (HFOV 48.8° 로부터 (1280/2)/tan(24.4°))
- 작업 공간 1.8 × 1.8 m
- **마주보는 두 "변 중앙" 바깥에 한 대씩** — #130 확정 (모서리 배치가 아니다)
- 높이 1,650 mm 는 삼각대 최대치라 고정값이다. 후퇴만 조절 가능하다
- 각 카메라는 자기 쪽 절반(깊이 900 mm × 폭 1800 mm)만 담당한다

`edge` 와 `corner` 를 모두 계산할 수 있게 해 둔 이유는, 왜 변 중앙이 나은지를
숫자로 남겨두기 위해서다. 결론은 fig3 에 있다.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

plt.rcParams["font.family"] = "NanumGothic"
plt.rcParams["axes.unicode_minus"] = False


def _warn_if_font_missing() -> None:
    """한글 폰트가 없으면 그림의 글자가 깨지고 PNG 바이트도 달라진다.

    수치(표준출력 표)는 폰트와 무관하게 같다 — **근거로 삼을 것은 그쪽**이고,
    PNG 는 같은 툴체인에서만 바이트가 재현된다.
    """
    from matplotlib import font_manager

    if not any(f.name == "NanumGothic" for f in font_manager.fontManager.ttflist):
        print(
            "[!] NanumGothic 이 없어 그림의 한글이 깨집니다 "
            "(sudo apt install fonts-nanum). 표준출력 수치는 영향받지 않습니다.\n",
            file=sys.stderr,
        )


FOCAL_PX = 1410.0  # 배치도 Rev.I 실측 (HFOV 48.8° 환산 1411 과 반올림 차이)
WORKSPACE_MM = 1800.0
HALF_MM = WORKSPACE_MM / 2
WALL_H_MM = 350.0  # 보수적인 쪽. 배치도 Rev.I 는 탑뷰1 쪽 0.25 m / 탑뷰2 쪽 0.35 m
U_LIMIT, V_LIMIT = 640.0, 360.0
DETECT_PX = 20.0  # YOLO 가 안정적으로 무는 최소 물체 폭
OBJ_MM = 30.0  # 폐기된 옛 규격. 왜 50 mm 로 올렸는지의 근거라 그대로 둔다
# 가벽이 만드는 사각지대는 **비용이지 제약이 아니다.**
# 예전에 204 mm 를 feasibility 필터로 썼는데, 그 값은 폐기된 모서리 배치
# (2050/1400)에서 우연히 나온 수치였을 뿐 독립적인 설계 요구가 아니었다.
# 확정 배치(1650/950)는 256 mm 라 그 필터로는 제외돼 버린다 — 같은 그림 안에서
# "탐색 결과"와 "권고안"이 다른 조건 위에 서게 된다. 그래서 필터를 걷고
# 등고선으로 같이 그린다. 유일한 하드 제약은 FOV 다.
BLIND_CONTOURS_MM = (150.0, 250.0, 350.0, 450.0)
PICK = (1650.0, 950.0)  # 배치도 Rev.I · 8/20 팀 확정 (높이, 변 중앙에서의 후퇴)
PITCH_DEG = 44.1  # 확정 하향각 — 프레임 상단을 먼 울타리에 맞춘 값
COVER_START_MM = 212.0  # 화각 전폭이 시작되는 지점. 그 앞은 마주보는 카메라가 덮는다
CORNER_OLD = (2050.0, 1400.0)  # 8/19 오전에 검토했다가 폐기한 모서리 배치
TALLER = (1900.0, 900.0)  # 더 높은 거치가 가능할 때의 대안 (#149 D3)
OUT = Path(__file__).resolve().parents[2] / "docs" / "assets" / "vision"


def _rig(h: float, s: float, pitch_deg: float, layout: str):
    """카메라 위치와 광축 기저. layout: 'edge'(변 중앙) 또는 'corner'(모서리)."""
    if layout == "edge":
        center = np.array([HALF_MM, -s, h])
        yaw = np.array([0.0, 1.0, 0.0])
    else:
        g = s / math.sqrt(2)
        center = np.array([-g, -g, h])
        yaw = np.array([1.0, 1.0, 0.0]) / math.sqrt(2)
    pr = math.radians(pitch_deg)
    fwd = np.array([yaw[0] * math.cos(pr), yaw[1] * math.cos(pr), -math.sin(pr)])
    right = np.cross(fwd, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    down /= np.linalg.norm(down)
    return center, right, down, fwd


def duty_points(layout: str) -> list[tuple[float, float]]:
    """프레임 안에 반드시 들어와야 하는 담당 구역의 귀퉁이.

    변 중앙 배치는 **가까운 벽에 딱 붙은 귀퉁이를 요구하지 않는다.** 배치도 Rev.I
    가 후퇴를 권고 1100 에서 950 mm 로 줄이면서 앞쪽 COVER_START_MM 를 포기하기로
    확정했고, 그 안쪽은 마주보는 반대편 카메라가 덮는다. 이 전제를 빼면 확정 배치가
    FOV 초과(max|u| 675/640)로 판정된다 — 예전 duty 정의가 그랬다.
    """
    if layout == "edge":
        y0 = COVER_START_MM
        return [(0.0, y0), (WORKSPACE_MM, y0), (0.0, HALF_MM), (WORKSPACE_MM, HALF_MM)]
    return [(0.0, 0.0), (WORKSPACE_MM, 0.0), (0.0, WORKSPACE_MM)]


def worst_point(layout: str) -> tuple[float, float]:
    """담당 구역에서 가장 먼 점 — 여기가 검출과 정확도 양쪽의 최악 조건이다."""
    return (0.0, HALF_MM) if layout == "edge" else (WORKSPACE_MM, 0.0)


def best_pitch(h: float, s: float, layout: str) -> tuple[float, float, float] | None:
    """담당 구역이 다 들어오는 피치 중 수직 여유가 가장 큰 것. (pitch, max|u|, max|v|)."""
    return _best_pitch_for(h, s, duty_points(layout), layout)


def _best_pitch_for(
    h: float, s: float, pts: list[tuple[float, float]], layout: str = "edge"
) -> tuple[float, float, float] | None:
    """임의의 담당 점 집합에 대한 최적 피치."""
    best = None
    for pitch in np.arange(1.0, 88.0, 0.25):
        center, right, down, fwd = _rig(h, s, float(pitch), layout)
        us, vs, ok = [], [], True
        for x, y in pts:
            d = np.array([x, y, 0.0]) - center
            z = d @ fwd
            if z <= 0:
                ok = False
                break
            us.append(abs(FOCAL_PX * (d @ right) / z))
            vs.append(abs(FOCAL_PX * (d @ down) / z))
        if ok:
            cand = (max(vs), max(us), float(pitch))
            if best is None or cand < best:
                best = cand
    if best is None:
        return None
    v, u, pitch = best
    return pitch, u, v


def fits(h: float, s: float, layout: str) -> bool:
    r = best_pitch(h, s, layout)
    return r is not None and r[1] <= U_LIMIT and r[2] <= V_LIMIT


def slant_elev(h: float, s: float, layout: str, x: float, y: float) -> tuple[float, float]:
    center, _, _, _ = _rig(h, s, 45.0, layout)  # 위치만 쓰므로 피치는 무관
    d = np.array([x, y, 0.0]) - center
    slant = float(np.linalg.norm(d))
    return slant, math.degrees(math.asin(h / slant))


def blind_band(h: float, s: float, layout: str = "edge") -> float:
    """가벽이 만드는 사각지대 폭. 벽 높이 H, 벽까지 수평거리 d → H·d/(h-H).

    변 중앙 배치는 후퇴가 곧 벽까지의 수직 거리지만,
    모서리 배치는 대각으로 물러난 것이라 각 벽까지는 s/√2 다.
    """
    d = s if layout == "edge" else s / math.sqrt(2)
    return WALL_H_MM * d / (h - WALL_H_MM)


def sigma_max(
    h: float, s: float, layout: str, x: float, y: float, pitch_deg: float | None = None
) -> float:
    """이미지→지면 사상 야코비안의 최대 특이값 [mm/px].

    클릭이 1 px 어긋났을 때 최악 방향으로 생기는 월드 오차다.
    흔히 쓰는 근사 slant/(f·sinθ) 보다 약 8% 작다.
    """
    pitch = best_pitch(h, s, layout)[0] if pitch_deg is None else pitch_deg
    center, right, down, fwd = _rig(h, s, pitch, layout)
    rot = np.vstack([right, down, fwd])

    def world(u: float, v: float) -> np.ndarray:
        dw = rot.T @ np.array([u, v, FOCAL_PX])
        return center[:2] + (-center[2] / dw[2]) * dw[:2]

    d = np.array([x, y, 0.0]) - center
    z = d @ fwd
    u, v = FOCAL_PX * (d @ right) / z, FOCAL_PX * (d @ down) / z
    e = 0.05
    jac = np.column_stack(
        [
            (world(u + e, v) - world(u - e, v)) / (2 * e),
            (world(u, v + e) - world(u, v - e)) / (2 * e),
        ]
    )
    return float(np.linalg.svd(jac, compute_uv=False)[0])


def sweep(layout: str, hs, ss, blind_cap: float | None = None):
    """(높이, 후퇴) 격자에서 최악점의 30 mm 픽셀 폭. 불가한 조합은 nan.

    하드 제약은 FOV 뿐이다. `blind_cap` 은 기본으로 걸지 않는다 — 가림은
    비용이라 등고선으로 따로 보여준다.
    """
    z = np.full((len(hs), len(ss)), np.nan)
    wx, wy = worst_point(layout)
    for i, h in enumerate(hs):
        for j, s in enumerate(ss):
            if blind_cap is not None and blind_band(float(h), float(s), layout) > blind_cap:
                continue
            if not fits(float(h), float(s), layout):
                continue
            slant, _ = slant_elev(float(h), float(s), layout, wx, wy)
            z[i, j] = OBJ_MM * FOCAL_PX / slant
    return z


def fig_placement_sweep() -> tuple[float, float, float]:
    """변 중앙 배치의 전수 탐색. 30 mm 는 어디서도 검출선을 못 넘는다.

    회색은 FOV 초과뿐이다. 가벽 가림은 필터가 아니라 **점선 등고선**으로 겹쳐
    그린다 — 낮게 달수록 픽셀은 좋아지고 가림은 나빠지는 맞바꿈이 한 장에 보인다.
    """
    hs = np.arange(900, 2601, 25)
    ss = np.arange(100, 1601, 25)
    z = sweep("edge", hs, ss)
    blind = np.array([[blind_band(float(h), float(s)) for s in ss] for h in hs])

    fig, ax = plt.subplots(figsize=(9.4, 6.2))
    im = ax.pcolormesh(ss, hs, z, cmap="viridis", shading="auto", vmin=11, vmax=DETECT_PX)
    fig.colorbar(im, ax=ax).set_label(
        f"최악점에서 {OBJ_MM:.0f} mm 물체의 픽셀 폭 [px]", fontsize=11
    )
    ax.clabel(
        ax.contour(ss, hs, z, levels=[14, 15, 16, 17], colors="white", linewidths=1.0),
        fmt="%d px",
        fontsize=9,
    )
    cb = ax.contour(
        ss,
        hs,
        np.where(np.isnan(z), np.nan, blind),
        levels=list(BLIND_CONTOURS_MM),
        colors="#ff9f0a",
        linewidths=1.3,
        linestyles=":",
    )
    ax.clabel(cb, fmt="가림 %d mm", fontsize=8.5)

    ph, ps = PICK
    cur = z[int(np.argmin(abs(hs - ph))), int(np.argmin(abs(ss - ps)))]
    top = np.unravel_index(np.nanargmax(z), z.shape)
    ax.plot(
        [ps],
        [ph],
        "*",
        ms=22,
        mec="k",
        mfc="#ff3b30",
        mew=1.4,
        label=f"확정 {ph:.0f}/{ps:.0f} — {cur:.1f} px · 가림 {blind_band(ph, ps):.0f} mm",
    )
    ax.plot(
        [ss[top[1]]],
        [hs[top[0]]],
        "P",
        ms=14,
        mec="k",
        mfc="#ffd60a",
        mew=1.2,
        label=f"픽셀 최대 {hs[top[0]]}/{ss[top[1]]} — {np.nanmax(z):.1f} px",
    )
    ax.set_xlabel("변 중앙에서의 후퇴 s [mm]", fontsize=12)
    ax.set_ylabel("카메라 높이 h [mm]", fontsize=12)
    ax.set_title(
        f"변 중앙 배치 — {OBJ_MM:.0f} mm 는 어느 배치에서도 {DETECT_PX:.0f} px 에 닿지 않는다\n"
        "회색 = FOV 초과 · 점선 = 가벽이 만드는 사각지대 "
        "(C270 720p · 1.8×1.8 m · 2대 마주보기)",
        fontsize=12.5,
        pad=12,
    )
    ax.set_facecolor("#d9d9d9")
    ax.legend(loc="lower right", fontsize=9.5, framealpha=0.95)
    ax.text(
        0.03,
        0.05,
        f"FOV 를 만족하는 {int(np.sum(~np.isnan(z)))} 조합 중 최대 {np.nanmax(z):.1f} px\n"
        f"{DETECT_PX:.0f} px 에 도달하는 조합 = 0",
        transform=ax.transAxes,
        fontsize=10.5,
        va="bottom",
        ha="left",
        bbox=dict(fc="white", ec="#c00", lw=1.4, alpha=0.95, pad=6),
    )
    fig.tight_layout()
    fig.savefig(OUT / "fig1_placement_sweep.png", dpi=150)
    plt.close(fig)
    return float(np.nanmax(z)), float(hs[top[0]]), float(ss[top[1]])


def fig_object_size() -> float:
    """물체 크기가 유일하게 듣는 지렛대임을 보인다."""
    h, s = PICK
    spots = [
        ("가까운 변 중앙 (900, 0)", slant_elev(h, s, "edge", HALF_MM, 0.0)[0], "#34c759"),
        ("담당 구역 중앙", slant_elev(h, s, "edge", HALF_MM, HALF_MM / 2)[0], "#007aff"),
        ("경계 모서리 — 최악점", slant_elev(h, s, "edge", *worst_point("edge"))[0], "#ff3b30"),
    ]
    worst = spots[-1][1]
    need = DETECT_PX * worst / FOCAL_PX
    widths = np.linspace(20, 70, 400)

    fig, ax = plt.subplots(figsize=(9, 5.6))
    for name, slant, col in spots:
        ax.plot(
            widths,
            widths * FOCAL_PX / slant,
            lw=2.4,
            color=col,
            label=f"{name}  (슬랜트 {slant / 1000:.2f} m)",
        )
    ax.axhline(DETECT_PX, color="k", ls="--", lw=1.6)
    ax.text(21, DETECT_PX + 1.0, f"YOLO 검출 하한 {DETECT_PX:.0f} px", fontsize=10.5, weight="bold")
    ax.axvline(need, color="#ff3b30", ls=":", lw=1.8)
    ax.plot([need], [DETECT_PX], "o", ms=10, mec="k", mfc="#ff3b30", zorder=5)
    ax.annotate(
        f"최악점까지 {DETECT_PX:.0f} px 를 보장하는\n최소 물체 폭 = {need:.0f} mm",
        xy=(need, DETECT_PX),
        xytext=(need + 6, 8),
        fontsize=11,
        weight="bold",
        arrowprops=dict(arrowstyle="->", lw=1.6, color="#ff3b30"),
        bbox=dict(fc="#fff3f2", ec="#ff3b30", lw=1.3, pad=5),
    )
    for x, lab, col in [(30, "현재 원기둥\n30 mm", "#8e8e93"), (50, "권고\n50 mm", "#34c759")]:
        ax.axvline(x, color=col, lw=1.2, alpha=0.8)
        ax.text(x, 1.5, lab, ha="center", va="bottom", fontsize=10, color="#3a3a3c")
        ax.plot([x], [x * FOCAL_PX / worst], "s", ms=8, mec="k", mfc="w", zorder=5)
        ax.text(
            x - 1.2,
            x * FOCAL_PX / worst + 2.2,
            f"{x * FOCAL_PX / worst:.1f} px",
            ha="right",
            fontsize=10.5,
            weight="bold",
        )
    ax.set_xlim(20, 70)
    ax.set_ylim(0, 55)
    ax.set_xlabel("물체의 실제 폭 [mm]", fontsize=12)
    ax.set_ylabel("화면에서 차지하는 픽셀 폭 [px]", fontsize=12)
    ax.set_title(
        f"권고 배치(변 중앙 · 높이 {h:.0f} / 후퇴 {s:.0f})에서 물체 크기 → 픽셀 폭\n"
        f"px = 물체폭 × f / 슬랜트,  f = {FOCAL_PX:.0f} px",
        fontsize=12.5,
        pad=12,
    )
    ax.grid(alpha=0.28)
    ax.legend(loc="upper left", fontsize=10.5)
    fig.tight_layout()
    fig.savefig(OUT / "fig2_object_size.png", dpi=150)
    plt.close(fig)
    return need


def fig_layout_compare():
    """모서리 배치와 변 중앙 배치를 같은 조건에서 비교한다."""
    rows = []
    for layout, title, col in [
        ("corner", "모서리 배치 (8/19 오전 · 폐기)", "#ff3b30"),
        ("edge", "변 중앙 배치 (#130 확정)", "#34c759"),
    ]:
        h, s = PICK if layout == "edge" else CORNER_OLD
        wx, wy = worst_point(layout)
        slant, elev = slant_elev(h, s, layout, wx, wy)
        rows.append(
            dict(
                layout=layout,
                title=title,
                col=col,
                h=h,
                s=s,
                slant=slant,
                elev=elev,
                px=OBJ_MM * FOCAL_PX / slant,
                sig=sigma_max(h, s, layout, wx, wy, PITCH_DEG if layout == "edge" else None),
                blind=blind_band(h, s, layout),
                need=DETECT_PX * slant / FOCAL_PX,
                pitch=PITCH_DEG if layout == "edge" else best_pitch(h, s, layout)[0],
            )
        )

    fig, axes = plt.subplots(1, 2, figsize=(12.6, 6.4))
    w = WORKSPACE_MM
    for ax, r in zip(axes, rows, strict=True):
        ax.add_patch(plt.Rectangle((0, 0), w, w, fc="#f2f2f7", ec="#3a3a3c", lw=2.2))
        if r["layout"] == "edge":
            cams = [(HALF_MM, -r["s"]), (HALF_MM, w + r["s"])]
            ax.plot([0, w], [HALF_MM, HALF_MM], color="#8e8e93", ls="--", lw=1.4)
            ax.add_patch(plt.Rectangle((0, 0), w, HALF_MM, fc=r["col"], alpha=0.14))
            ax.text(w / 2, HALF_MM + 70, "담당 경계", ha="center", fontsize=9.5, color="#3a3a3c")
        else:
            g = r["s"] / math.sqrt(2)
            cams = [(-g, -g), (w + g, w + g)]
            ax.plot([0, w], [w, 0], color="#8e8e93", ls="--", lw=1.4)
            ax.add_patch(plt.Polygon([(0, 0), (w, 0), (0, w)], fc=r["col"], alpha=0.14))
            ax.text(
                w / 2 + 130, w / 2 + 130, "담당 경계", ha="center", fontsize=9.5, color="#3a3a3c"
            )
        wx, wy = worst_point(r["layout"])
        for cx, cy in cams:
            ax.plot([cx], [cy], "^", ms=15, mec="k", mfc=r["col"], mew=1.2, zorder=5)
        ax.plot([cams[0][0], wx], [cams[0][1], wy], color=r["col"], lw=1.7, ls=":")
        ax.plot([wx], [wy], "o", ms=11, mec="k", mfc="#ffd60a", zorder=6)
        off = (150, -330) if r["layout"] == "edge" else (110, 110)
        ax.text(
            wx + off[0],
            wy + off[1],
            f"최악점\n{r['slant'] / 1000:.2f} m",
            fontsize=9.5,
            weight="bold",
        )
        ax.text(cams[0][0], cams[0][1] - 300, "카메라", ha="center", fontsize=9.5, color="#3a3a3c")
        ax.set_xlim(-1500, w + 1500)
        ax.set_ylim(-1700, w + 2700)
        ax.set_aspect("equal")
        ax.axis("off")
        sub = f"높이 {r['h']:.0f} / 후퇴 {r['s']:.0f} · 하향 {r['pitch']:.0f}°"
        ax.set_title(f"{r['title']}\n{sub}", fontsize=12.5, pad=6)
        face = "#fff3f2" if r["layout"] == "corner" else "#f1fbf3"
        ax.text(
            w / 2,
            w + 1500,
            f"{OBJ_MM:.0f} mm → {r['px']:.1f} px\n"
            f"20 px 최소 물체 폭 {r['need']:.0f} mm\n"
            f"σmax {r['sig']:.2f} mm/px · 고도각 {r['elev']:.1f}°\n"
            f"가림 띠 {r['blind']:.0f} mm",
            ha="center",
            fontsize=11.5,
            weight="bold",
            bbox=dict(fc=face, ec=r["col"], lw=1.6, pad=6),
        )
    fig.suptitle(
        "담당 구역의 '가장 먼 점'이 얼마나 먼가가 전부다\n"
        "변 중앙은 담당 구역이 납작한 직사각형이라 최악점이 훨씬 가깝다",
        fontsize=13,
        y=0.995,
    )
    fig.tight_layout()
    fig.savefig(OUT / "fig3_layout_compare.png", dpi=150)
    plt.close(fig)
    return rows


def four_edge_best(strip_mm: float = 450.0) -> tuple[float, float, float, float]:
    """네 변에 한 대씩 두고 각자 `strip_mm` 깊이 띠만 담당할 때의 최선.

    D6(4대 안) 철회 근거다. 대수를 늘려도 **각 카메라가 여전히 폭 1800 mm 를
    통째로 담아야** 하므로 가로 해상도가 거의 안 는다.

    반환 (30 mm 픽셀 폭, 높이, 후퇴, 고도각).
    """
    pts = [(0.0, 0.0), (WORKSPACE_MM, 0.0), (0.0, strip_mm), (WORKSPACE_MM, strip_mm)]
    best = None
    for h in np.arange(900, 2601, 25):
        for s in np.arange(100, 1601, 25):
            r = _best_pitch_for(float(h), float(s), pts)
            if r is None or r[1] > U_LIMIT or r[2] > V_LIMIT:
                continue
            center, _, _, _ = _rig(float(h), float(s), 45.0, "edge")
            slant = float(np.linalg.norm(np.array([0.0, strip_mm, 0.0]) - center))
            px = OBJ_MM * FOCAL_PX / slant
            if best is None or px > best[0]:
                best = (px, float(h), float(s), math.degrees(math.asin(h / slant)))
    return best


def sensitivity_table() -> None:
    h, s = PICK
    pitch, umax, vmax = best_pitch(h, s, "edge")
    print(
        f"# 변 중앙 배치 h={h:.0f} s={s:.0f} · 하향 {pitch:.1f}° (확정 {PITCH_DEG}°) · "
        f"max|u|={umax:.0f}/{U_LIMIT:.0f} max|v|={vmax:.0f}/{V_LIMIT:.0f} · "
        f"가림 띠 {blind_band(h, s):.0f} mm"
    )
    if not fits(h, s, "edge"):
        print(
            f"[!] 이 배치는 FOV 를 초과합니다 (max|u|={umax:.0f}/{U_LIMIT:.0f}). "
            "아래 값은 참고용입니다 — duty_points 전제와 배치도를 대조하세요."
        )
    print()
    head = (
        f"{'지점':<24}{'슬랜트':>8}{'고도각':>8}{'σmax':>9}"
        f"{'근사식':>9}{'3px':>8}{'30mm':>8}{'50mm':>8}"
    )
    print(head)
    rows = [
        ("가까운 변 중앙", HALF_MM, 0.0),
        ("가까운 변 모서리", 0.0, 0.0),
        ("담당 구역 중앙", HALF_MM, HALF_MM / 2),
        ("경계 중앙", HALF_MM, HALF_MM),
        ("경계 모서리 — 최악점", 0.0, HALF_MM),
    ]
    for name, x, y in rows:
        slant, elev = slant_elev(h, s, "edge", x, y)
        sig = sigma_max(h, s, "edge", x, y, PITCH_DEG)
        approx = slant / (FOCAL_PX * math.sin(math.radians(elev)))
        print(
            f"{name:<24}{slant / 1000:8.2f}{elev:8.1f}{sig:9.2f}{approx:9.2f}"
            f"{3 * sig:8.1f}{OBJ_MM * FOCAL_PX / slant:8.1f}{50 * FOCAL_PX / slant:8.1f}"
        )


def main() -> None:
    _warn_if_font_missing()
    OUT.mkdir(parents=True, exist_ok=True)
    sensitivity_table()
    px_max, h_max, s_max = fig_placement_sweep()
    need = fig_object_size()
    rows = fig_layout_compare()
    print(f"\n[fig1] 변 중앙 전수 탐색 최대 {px_max:.1f} px (h={h_max:.0f}, s={s_max:.0f})")
    print(f"[fig2] 권고 배치에서 20 px 최소 물체 폭 {need:.0f} mm")
    for r in rows:
        print(
            f"[fig3] {r['layout']:<7} h={r['h']:.0f} s={r['s']:.0f} → "
            f"30 mm {r['px']:.1f} px · 최소 물체 {r['need']:.0f} mm · σmax {r['sig']:.2f}"
        )
    px4, h4, s4, el4 = four_edge_best()
    print(
        f"[D6] 네 변에 4대(각자 450 mm 띠) 최선 — 30 mm {px4:.1f} px "
        f"(h={h4:.0f}, s={s4:.0f}, 고도각 {el4:.1f}°) → 여전히 {DETECT_PX:.0f} px 미달"
    )
    print(f"\n그림 3장 → {OUT}")


if __name__ == "__main__":
    main()
