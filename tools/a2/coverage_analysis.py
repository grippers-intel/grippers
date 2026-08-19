"""웹캠 배치가 물체 검출과 위치 정확도에 어떤 한계를 거는지 계산한다.

issue #149 의 D1·D3·D6 근거 자료다. 실행하면 `docs/assets/vision/` 아래
그림 3장을 다시 만들고, 픽셀 민감도 표를 표준출력으로 낸다.

    python tools/a2/coverage_analysis.py

전제
- Logitech C270 · 720p · f = 1411 px (HFOV 48.8° 로부터 (1280/2)/tan(24.4°))
- 작업 공간 1.8 × 1.8 m, 카메라는 마주 보는 두 모서리 바깥에 대각으로 후퇴
- 각 카메라는 자기 절반 + 대각 경계만 담당한다 (먼 모서리는 상대 카메라 몫)
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

plt.rcParams["font.family"] = "NanumGothic"
plt.rcParams["axes.unicode_minus"] = False

FOCAL_PX = 1411.0
WORKSPACE_MM = 1800.0
QUADRANT_MM = 900.0
WALL_H_MM = 350.0
U_LIMIT, V_LIMIT = 640.0, 360.0
DETECT_PX = 20.0          # YOLO 가 안정적으로 무는 최소 물체 폭
OBJ_MM = 30.0             # 현재 출력된 원기둥 지름
OUT = Path(__file__).resolve().parents[2] / "docs" / "assets" / "vision"

HALF = [(0.0, 0.0), (WORKSPACE_MM, 0.0), (0.0, WORKSPACE_MM)]
QUAD = [(0.0, 0.0), (QUADRANT_MM, 0.0), (0.0, QUADRANT_MM), (QUADRANT_MM, QUADRANT_MM)]


def _basis(h: float, s: float, pitch_deg: float):
    """대각 방향으로 세운 카메라의 위치와 광축 기저."""
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


def best_pitch(h: float, s: float, pts) -> tuple[float, float, float] | None:
    """pts 가 모두 프레임에 들어오는 피치 중 수직 여유가 가장 큰 것.

    반환 (pitch, max|u|, max|v|). 어떤 피치로도 못 담으면 None.
    """
    best = None
    for pitch in np.arange(1.0, 85.0, 0.25):
        center, right, down, fwd = _basis(h, s, pitch)
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


def fits(h: float, s: float, pts) -> bool:
    r = best_pitch(h, s, pts)
    return r is not None and r[1] <= U_LIMIT and r[2] <= V_LIMIT


def slant_elev(h: float, s: float, x: float, y: float) -> tuple[float, float]:
    g = s / math.sqrt(2)
    ground = math.hypot(x + g, y + g)
    return math.hypot(ground, h), math.degrees(math.atan2(h, ground))


def blind_band(h: float, s: float) -> float:
    """가벽이 만드는 사각지대 폭. 벽 높이 H, 벽까지 수평거리 d → H·d/(h-H)."""
    return WALL_H_MM * (s / math.sqrt(2)) / (h - WALL_H_MM)


def sigma_max(h: float, s: float, pitch: float, x: float, y: float) -> float:
    """이미지→지면 사상 야코비안의 최대 특이값 [mm/px].

    클릭이 1 px 어긋났을 때 최악 방향으로 생기는 월드 오차다.
    흔히 쓰는 근사 slant/(f·sinθ) 보다 약 8% 작다.
    """
    center, right, down, fwd = _basis(h, s, pitch)
    rot = np.vstack([right, down, fwd])

    def world(u: float, v: float) -> np.ndarray:
        dw = rot.T @ np.array([u, v, FOCAL_PX])
        return center[:2] + (-center[2] / dw[2]) * dw[:2]

    d = np.array([x, y, 0.0]) - center
    z = d @ fwd
    u, v = FOCAL_PX * (d @ right) / z, FOCAL_PX * (d @ down) / z
    e = 0.05
    jac = np.column_stack(
        [(world(u + e, v) - world(u - e, v)) / (2 * e),
         (world(u, v + e) - world(u, v - e)) / (2 * e)]
    )
    return float(np.linalg.svd(jac, compute_uv=False)[0])


def fig_placement_sweep(current=(2050.0, 1400.0)) -> tuple[float, float, float]:
    """배치 전수 탐색 — 어떤 (높이, 후퇴) 조합도 30 mm 를 검출선까지 못 올린다."""
    hs = np.arange(900, 2151, 25)
    ss = np.arange(500, 1851, 25)
    z = np.full((len(hs), len(ss)), np.nan)
    for i, h in enumerate(hs):
        for j, s in enumerate(ss):
            if not fits(float(h), float(s), HALF):
                continue
            slant, _ = slant_elev(float(h), float(s), WORKSPACE_MM, 0.0)
            z[i, j] = OBJ_MM * FOCAL_PX / slant

    fig, ax = plt.subplots(figsize=(9, 6.2))
    im = ax.pcolormesh(ss, hs, z, cmap="viridis", shading="auto", vmin=9, vmax=DETECT_PX)
    fig.colorbar(im, ax=ax).set_label(
        f"최악점에서 {OBJ_MM:.0f} mm 물체의 픽셀 폭 [px]", fontsize=11
    )
    ax.clabel(ax.contour(ss, hs, z, levels=[11, 12, 13], colors="white", linewidths=1.0),
              fmt="%d px", fontsize=9)
    ch, cs_ = current
    cur = z[int(np.argmin(abs(hs - ch))), int(np.argmin(abs(ss - cs_)))]
    top = np.unravel_index(np.nanargmax(z), z.shape)
    ax.plot([cs_], [ch], "*", ms=22, mec="k", mfc="#ff3b30", mew=1.4,
            label=f"현행 확정안 {ch:.0f}/{cs_:.0f} — {cur:.1f} px")
    ax.plot([ss[top[1]]], [hs[top[0]]], "P", ms=14, mec="k", mfc="#ffd60a", mew=1.2,
            label=f"픽셀 최대 배치 {hs[top[0]]}/{ss[top[1]]} — {np.nanmax(z):.1f} px")
    ax.set_xlabel("대각 후퇴 s [mm]", fontsize=12)
    ax.set_ylabel("카메라 높이 h [mm]", fontsize=12)
    ax.set_title(
        f"배치를 어떻게 바꿔도 {OBJ_MM:.0f} mm 는 {DETECT_PX:.0f} px 에 닿지 않는다\n"
        "회색 = FOV 안에 담당 구역이 안 들어오는 조합 (C270 720p · 1.8×1.8 m · 2대 마주보기)",
        fontsize=12.5, pad=12)
    ax.set_facecolor("#d9d9d9")
    ax.legend(loc="upper left", fontsize=10, framealpha=0.95)
    ax.text(0.03, 0.06,
            f"탐색 {int(np.sum(~np.isnan(z)))} 조합 중 최대 {np.nanmax(z):.1f} px\n"
            f"검출 기준 {DETECT_PX:.0f} px 에 도달하는 조합 = 0",
            transform=ax.transAxes, fontsize=10.5, va="bottom", ha="left",
            bbox=dict(fc="white", ec="#c00", lw=1.4, alpha=0.95, pad=6))
    fig.tight_layout()
    fig.savefig(OUT / "fig1_placement_sweep.png", dpi=150)
    plt.close(fig)
    return float(np.nanmax(z)), float(hs[top[0]]), float(ss[top[1]])


def fig_object_size(h: float = 2050.0, s: float = 1400.0) -> float:
    """물체 크기만이 실제로 듣는 지렛대임을 보인다."""
    spots = [
        ("가까운 모서리 (0,0)", slant_elev(h, s, 0.0, 0.0)[0], "#34c759"),
        ("작업 공간 중앙", slant_elev(h, s, 900.0, 900.0)[0], "#007aff"),
        ("옆 모서리 — 최악점", slant_elev(h, s, WORKSPACE_MM, 0.0)[0], "#ff3b30"),
    ]
    worst = spots[-1][1]
    need = DETECT_PX * worst / FOCAL_PX
    widths = np.linspace(20, 80, 400)

    fig, ax = plt.subplots(figsize=(9, 5.6))
    for name, slant, col in spots:
        ax.plot(widths, widths * FOCAL_PX / slant, lw=2.4, color=col,
                label=f"{name}  (슬랜트 {slant / 1000:.2f} m)")
    ax.axhline(DETECT_PX, color="k", ls="--", lw=1.6)
    ax.text(21, DETECT_PX + 0.9, f"YOLO 검출 하한 {DETECT_PX:.0f} px", fontsize=10.5, weight="bold")
    ax.axvline(need, color="#ff3b30", ls=":", lw=1.8)
    ax.plot([need], [DETECT_PX], "o", ms=10, mec="k", mfc="#ff3b30", zorder=5)
    ax.annotate(f"최악점까지 {DETECT_PX:.0f} px 를 보장하는\n최소 물체 폭 = {need:.0f} mm",
                xy=(need, DETECT_PX), xytext=(need + 7, 11), fontsize=11, weight="bold",
                arrowprops=dict(arrowstyle="->", lw=1.6, color="#ff3b30"),
                bbox=dict(fc="#fff3f2", ec="#ff3b30", lw=1.3, pad=5))
    for x, lab in [(30, "현재 원기둥\n30 mm"), (55, "권고\n55 mm")]:
        ax.axvline(x, color="#8e8e93", lw=1.0, alpha=0.7)
        ax.text(x, 1.5, lab, ha="center", va="bottom", fontsize=10, color="#3a3a3c")
        ax.plot([x], [x * FOCAL_PX / worst], "s", ms=8, mec="k", mfc="w", zorder=5)
        ax.text(x - 1.5, x * FOCAL_PX / worst + 2.0, f"{x * FOCAL_PX / worst:.1f} px",
                ha="right", fontsize=10.5, weight="bold")
    ax.set_xlim(20, 80)
    ax.set_ylim(0, 85)
    ax.set_xlabel("물체의 실제 폭 [mm]", fontsize=12)
    ax.set_ylabel("화면에서 차지하는 픽셀 폭 [px]", fontsize=12)
    ax.set_title(f"확정 배치(높이 {h:.0f} / 후퇴 {s:.0f})에서 물체 크기 → 픽셀 폭\n"
                 f"px = 물체폭 × f / 슬랜트,  f = {FOCAL_PX:.0f} px", fontsize=12.5, pad=12)
    ax.grid(alpha=0.28)
    ax.legend(loc="upper left", fontsize=10.5)
    fig.tight_layout()
    fig.savefig(OUT / "fig2_object_size.png", dpi=150)
    plt.close(fig)
    return need


def fig_two_vs_four(h2: float = 2050.0, s2: float = 1400.0):
    """2대와 4대를 같은 가림 띠 기준에서 비교한다."""
    cap = blind_band(h2, s2)
    best = None
    for h in np.arange(900, 2601, 25):
        for s in np.arange(250, 1601, 25):
            if blind_band(float(h), float(s)) > cap or not fits(float(h), float(s), QUAD):
                continue
            slant, elev = slant_elev(float(h), float(s), QUADRANT_MM, 0.0)
            px = OBJ_MM * FOCAL_PX / slant
            if best is None or px > best[0]:
                best = (px, float(h), float(s), elev)
    px4, h4, s4, el4 = best
    slant2, el2 = slant_elev(h2, s2, WORKSPACE_MM, 0.0)
    sig2 = sigma_max(h2, s2, best_pitch(h2, s2, HALF)[0], WORKSPACE_MM, 0.0)
    sig4 = sigma_max(h4, s4, best_pitch(h4, s4, QUAD)[0], QUADRANT_MM, 0.0)

    fig, axes = plt.subplots(1, 2, figsize=(12.6, 6.0))
    layouts = [(2, h2, s2, "#ff3b30", f"현행 · 2대   높이 {h2:.0f} / 후퇴 {s2:.0f}"),
               (4, h4, s4, "#34c759", f"대안 · 4대   높이 {h4:.0f} / 후퇴 {s4:.0f}")]
    for ax, (n, _hh, ss_, col, title) in zip(axes, layouts, strict=True):
        w = WORKSPACE_MM
        ax.add_patch(plt.Rectangle((0, 0), w, w, fc="#f2f2f7", ec="#3a3a3c", lw=2.2))
        g = ss_ / math.sqrt(2)
        if n == 2:
            ax.plot([0, w], [w, 0], color="#8e8e93", ls="--", lw=1.4)
            ax.text(w / 2 + 120, w / 2 + 120, "담당 경계", ha="center", fontsize=9.5,
                    color="#3a3a3c")
            cams = [(-g, -g), (w + g, w + g)]
            ax.add_patch(plt.Polygon([(0, 0), (w, 0), (0, w)], fc=col, alpha=0.13))
            worst = (w, 0)
        else:
            ax.plot([0, w], [QUADRANT_MM] * 2, color="#8e8e93", ls="--", lw=1.2)
            ax.plot([QUADRANT_MM] * 2, [0, w], color="#8e8e93", ls="--", lw=1.2)
            cams = [(-g, -g), (w + g, -g), (-g, w + g), (w + g, w + g)]
            ax.add_patch(plt.Rectangle((0, 0), QUADRANT_MM, QUADRANT_MM, fc=col, alpha=0.13))
            worst = (QUADRANT_MM, 0)
        for cx, cy in cams:
            ax.plot([cx], [cy], "^", ms=15, mec="k", mfc=col, mew=1.2, zorder=5)
        ax.plot([cams[0][0], worst[0]], [cams[0][1], worst[1]], color=col, lw=1.7, ls=":")
        ax.plot([worst[0]], [worst[1]], "o", ms=11, mec="k", mfc="#ffd60a", zorder=6)
        ax.text(worst[0], worst[1] - 260, "최악점", ha="center", fontsize=10, weight="bold")
        ax.text(cams[0][0], cams[0][1] - 260, "카메라", ha="center", fontsize=9.5, color="#3a3a3c")
        ax.set_xlim(-1350, w + 1350)
        ax.set_ylim(-1500, w + 2500)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_title(title, fontsize=12.5, pad=6)
    for ax, px, sig, el, hh, ss_, face, edge, tail in [
        (axes[0], OBJ_MM * FOCAL_PX / slant2, sig2, el2, h2, s2, "#fff3f2", "#ff3b30", "검출 불가"),
        (axes[1], px4, sig4, el4, h4, s4, "#f1fbf3", "#34c759", "통과"),
    ]:
        ax.text(WORKSPACE_MM / 2, WORKSPACE_MM + 1750,
                f"{OBJ_MM:.0f} mm → {px:.1f} px  —  {tail}\n"
                f"σmax {sig:.2f} mm/px · 고도각 {el:.1f}° · 가림 띠 {blind_band(hh, ss_):.0f} mm",
                ha="center", fontsize=11.5, weight="bold",
                bbox=dict(fc=face, ec=edge, lw=1.6, pad=6))
    fig.suptitle(
        "담당 면적을 1/4 로 줄이면 물체를 안 키워도 검출선을 넘는다 — 대가는 카메라 2대 추가\n"
        f"(양쪽 모두 가림 띠 {cap:.0f} mm 이하라는 같은 조건에서 비교)", fontsize=13, y=0.99)
    fig.tight_layout()
    fig.savefig(OUT / "fig3_two_vs_four.png", dpi=150)
    plt.close(fig)
    return px4, h4, s4, sig4


def sensitivity_table(h: float = 2050.0, s: float = 1400.0) -> None:
    pitch, umax, vmax = best_pitch(h, s, HALF)
    print(f"# 배치 h={h:.0f} s={s:.0f} · 피치 {pitch:.2f}° · max|u|={umax:.0f}/640 "
          f"max|v|={vmax:.0f}/360 · 가림 띠 {blind_band(h, s):.0f} mm\n")
    head = (f"{'지점':<22}{'슬랜트':>8}{'고도각':>8}{'σmax':>9}"
            f"{'근사식':>9}{'3px':>8}{'30mm':>8}{'55mm':>8}")
    print(head)
    rows = [("가까운 모서리 (0,0)", 0.0, 0.0), ("작업 공간 중앙", 900.0, 900.0),
            ("옆 모서리 — 최악점", WORKSPACE_MM, 0.0),
            ("먼 모서리 (상대 담당)", WORKSPACE_MM, WORKSPACE_MM)]
    for name, x, y in rows:
        slant, elev = slant_elev(h, s, x, y)
        sig = sigma_max(h, s, pitch, x, y)
        approx = slant / (FOCAL_PX * math.sin(math.radians(elev)))
        print(f"{name:<22}{slant / 1000:8.2f}{elev:8.1f}{sig:9.2f}{approx:9.2f}"
              f"{3 * sig:8.1f}{OBJ_MM * FOCAL_PX / slant:8.1f}{55 * FOCAL_PX / slant:8.1f}")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    sensitivity_table()
    px_max, h_max, s_max = fig_placement_sweep()
    need = fig_object_size()
    px4, h4, s4, sig4 = fig_two_vs_four()
    print(f"\n[fig1] 배치 전수 탐색 최대 {px_max:.1f} px (h={h_max:.0f}, s={s_max:.0f})")
    print(f"[fig2] 최악점 {DETECT_PX:.0f} px 를 보장하는 최소 물체 폭 {need:.0f} mm")
    print(f"[fig3] 4대 최적 {px4:.1f} px (h={h4:.0f}, s={s4:.0f}, σmax {sig4:.2f})")
    print(f"\n그림 3장 → {OUT}")


if __name__ == "__main__":
    main()
