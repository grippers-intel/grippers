"""줄자로 잰 값을 넣으면 `host.yaml` 에 붙일 `floor_markers` 좌표를 계산한다.

전부 **cm** 로 넣는다. m 로 바꾸는 것은 이 도구가 한다(단위를 두 곳에서 바꾸면 꼭 한쪽이 틀린다).

방법 1 — 쉬운 쪽. 1번 마커 위치와 가로·세로만 잰다(직사각형 전제).

    python tools/make_layout.py --x1 10 --y1 40 --width 160 --depth 100

방법 2 — 정확한 쪽. 네 장을 각각 잰다. 직사각형이 아니어도, 비뚤어도 그대로 반영된다.

    python tools/make_layout.py --m1 10 40 --m2 170 40 --m3 10 140 --m4 170 140

방법 3 — 장판 위에 놓기(배치도 REV.2, 2026-09-29~). 장판 실측 크기(cm)만 넣으면
마커·상자 좌표와 `host.yaml` 의 작업 경계·작업 구역을 한 번에 계산해 준다.

    python tools/make_layout.py --mat 198 183       # 가로(좌우, 긴 변) 세로(앞뒤)

## 무엇을 재는가

- 원점은 **장판 앞쪽 왼쪽 모서리**(0, 0). +x 오른쪽(긴 변), +y 뒤쪽(상자·CAM B 쪽).
  그래서 장판 가장자리에서 줄자로 잰 값이 곧 좌표다.
- 각 마커는 **검은 사각형의 중심**이다. 인쇄물 종이 모서리가 아니다.
- 인쇄물은 **윗변이 +y 를 향하게** 붙인다. 코너 순서가 그 전제로 짜여 있다
  (`localization/aruco_localizer.floor_object_points`).
- 번호는 1·2·3·4 순서로 **앞왼 · 앞오른 · 뒤왼 · 뒤오른** 이다.

## 왜 도구로 계산하나

2026-09-06 이전 코드에서는 이 좌표가 파이썬 상수로 흩어져 있어 한쪽만 고쳐지는 일이
잦았다. 여기서는 값을 계산해 **붙여 넣을 YAML 블록**으로 내보내고, 붙인 뒤에는
`tools/place_markers.py` 의 재투영오차로 확인한다 — 좌표가 틀리면 거기서 바로 뜬다.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

HOST_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST_ROOT))

#: 마커끼리 이보다 가까우면 배치가 잘못됐다고 본다(m). 네 점이 모이면 외부파라미터가
#: 불안정해진다 — 마커가 만드는 사각형이 클수록 자세 추정이 정확하다.
MIN_SPAN_M = 0.30


#: 마커 인쇄물(흰 여백 포함) 한 변(m). 종이 좌우 변을 장판 좌우 끝에 붙이므로
#: 마커 중심은 끝에서 이 값의 절반이다(2026-09-29 실측 140 mm).
MARKER_PAPER_M = 0.140
#: 마커 중심을 장판 앞·뒤 가장자리에서 떼는 거리(m). 앞뒤 긴 변 가운데 1.60 m 카메라가
#: 네 장을 다 보는 범위(0.35~1.50)의 가운데쯤이다 — tools/check_coverage.py 로 확인했다.
MARKER_INSET_Y_M = 0.525
#: 상자 뒷면과 장판 뒤끝 사이(m). 붙인다.
BOX_BACK_GAP_M = 0.0


def mat_layout(mat_w: float, mat_l: float, box_size,
               paper_m: float = MARKER_PAPER_M, inset_y_m: float = MARKER_INSET_Y_M,
               box_back_gap_m: float = BOX_BACK_GAP_M) -> dict:
    """장판(가로 mat_w × 세로 mat_l, m) 위의 배치도 REV.2 배치.

    원점이 장판 앞·왼 모서리라 경계는 [0, W] × [0, L] 이다. 마커는 종이 좌우 변을 장판
    끝에 붙이고, 상자 하나는 뒤쪽 긴 변 가운데에 뒷면을 붙인다. 작업 구역은 마커 네 장
    중심 안쪽이다. 반환값은 host.yaml 에 그대로 옮길 수 있는 숫자들이다.
    """
    mx = paper_m / 2.0
    markers = {1: (mx, inset_y_m), 2: (mat_w - mx, inset_y_m),
               3: (mx, mat_l - inset_y_m), 4: (mat_w - mx, mat_l - inset_y_m)}
    bl = box_size[1]
    box = (mat_w / 2.0, mat_l - box_back_gap_m - bl / 2.0, 180.0)
    return {"wall_x": (0.0, mat_w), "wall_y": (0.0, mat_l),
            "floor_markers": markers,
            "workspace_x": (mx, mat_w - mx), "workspace_y": (inset_y_m, mat_l - inset_y_m),
            "box": box,
            "box_side_gap": (mat_w - box_size[0]) / 2.0,   # 상자 좌·우 모서리 <- 장판 좌·우 끝
            "paper_edge_y": inset_y_m - paper_m / 2.0}     # 종이 앞/뒤 변 <- 장판 앞/뒤 끝


def rectangle(x1: float, y1: float, width: float, depth: float) -> dict[int, tuple[float, float]]:
    """1번(앞왼) 기준 직사각형. 단위 m."""
    return {1: (x1, y1), 2: (x1 + width, y1), 3: (x1, y1 + depth), 4: (x1 + width, y1 + depth)}


def check(points: dict[int, tuple[float, float]], wall_x, wall_y) -> list[str]:
    """배치가 성립하는지 본다. 경고 문자열 목록을 돌려준다(빈 목록이면 이상 없음)."""
    warn = []
    for mid, (x, y) in sorted(points.items()):
        if not (wall_x[0] <= x <= wall_x[1] and wall_y[0] <= y <= wall_y[1]):
            warn.append(f"마커 {mid} ({x:.3f}, {y:.3f}) 가 장판 밖이다 "
                        f"(x {wall_x[0]}~{wall_x[1]}, y {wall_y[0]}~{wall_y[1]})")
    xs = [p[0] for p in points.values()]
    ys = [p[1] for p in points.values()]
    if max(xs) - min(xs) < MIN_SPAN_M or max(ys) - min(ys) < MIN_SPAN_M:
        warn.append(f"마커들이 너무 모여 있다 (가로 {max(xs) - min(xs):.2f} m · "
                    f"세로 {max(ys) - min(ys):.2f} m, 최소 {MIN_SPAN_M} m) — 외부파라미터가 불안정해진다")
    if len(points) == 4:
        d1 = ((points[1][0] - points[4][0]) ** 2 + (points[1][1] - points[4][1]) ** 2) ** 0.5
        d2 = ((points[2][0] - points[3][0]) ** 2 + (points[2][1] - points[3][1]) ** 2) ** 0.5
        if abs(d1 - d2) > 0.02:
            warn.append(f"대각선이 {abs(d1 - d2) * 1000:.0f} mm 다르다 "
                        f"({d1:.3f} vs {d2:.3f} m) — 직사각형이 아니거나 잰 값이 틀렸다")
    return warn


def as_yaml(points: dict[int, tuple[float, float]]) -> str:
    lines = ["  floor_markers:"]
    for mid, (x, y) in sorted(points.items()):
        lines.append(f"    {mid}: [{x:.3f}, {y:.3f}]")
    return "\n".join(lines)


def main() -> int:
    import host_config
    host_config.configure_console()
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    g = ap.add_argument_group("방법 1 — 직사각형 (cm)")
    g.add_argument("--x1", type=float)
    g.add_argument("--y1", type=float)
    g.add_argument("--width", type=float, help="1번 -> 2번 가로 거리")
    g.add_argument("--depth", type=float, help="1번 -> 3번 세로 거리")
    g2 = ap.add_argument_group("방법 2 — 네 점 각각 (cm)")
    for mid in (1, 2, 3, 4):
        g2.add_argument(f"--m{mid}", type=float, nargs=2, metavar=("X", "Y"))
    g3 = ap.add_argument_group("방법 3 — 장판 위에 놓기 (cm)")
    g3.add_argument("--mat", type=float, nargs=2, metavar=("W", "L"),
                    help="장판 실측 가로(x, 좌우 긴 변) 세로(y, 앞뒤)")
    args = ap.parse_args()

    if args.mat:
        cfg = host_config.load_host_config(args.config)
        w, l = args.mat[0] / 100.0, args.mat[1] / 100.0
        if w < l:
            print("가로(좌우)가 세로보다 짧습니다 — 배치도 REV.2 는 긴 변이 좌우입니다. 순서를 확인하십시오")
            return 2
        lay = mat_layout(w, l, cfg.arena.box_size)
        warn = check(lay["floor_markers"], lay["wall_x"], lay["wall_y"])
        print(f"장판 {w:.3f} × {l:.3f} m (원점 = 앞·왼 모서리)")
        print()
        print("현장에서 (장판 가장자리 기준, cm)")
        print(f"  마커 1~4: 종이 좌우 변을 장판 좌우 끝에 붙임 · 종이 앞/뒤 변이 앞/뒤 끝에서 "
              f"{lay['paper_edge_y'] * 100:.1f}  (중심 {MARKER_INSET_Y_M * 100:.1f})")
        print(f"  상자    : 좌·우 모서리가 장판 좌·우 끝에서 {lay['box_side_gap'] * 100:.2f} · "
              f"뒷면을 장판 뒤끝에 붙임")
        print(f"  카메라  : 앞·뒤 긴 변 가운데(좌우 끝에서 {w / 2 * 100:.2f}) · 렌즈 160")
        for msg in warn:
            print(f"⚠️ {msg}")
        print()
        print("host/config/host.yaml 에 이렇게 넣으십시오")
        print()
        print(as_yaml(lay["floor_markers"]))
        print("  # arena:")
        print(f"  wall_x: [{lay['wall_x'][0]:.3f}, {lay['wall_x'][1]:.3f}]")
        print(f"  wall_y: [{lay['wall_y'][0]:.3f}, {lay['wall_y'][1]:.3f}]")
        print(f"  workspace_x: [{lay['workspace_x'][0]:.3f}, {lay['workspace_x'][1]:.3f}]")
        print(f"  workspace_y: [{lay['workspace_y'][0]:.3f}, {lay['workspace_y'][1]:.3f}]")
        bx, by, yaw = lay["box"]
        print("  boxes:")
        print(f"    basket: [{bx:.3f}, {by:.3f}, {yaw:.1f}]")
        return 0

    four = [getattr(args, f"m{mid}") for mid in (1, 2, 3, 4)]
    if all(v is not None for v in four):
        points = {mid: (v[0] / 100.0, v[1] / 100.0) for mid, v in zip((1, 2, 3, 4), four)}
        how = "네 점 실측"
    elif None not in (args.x1, args.y1, args.width, args.depth):
        points = rectangle(args.x1 / 100.0, args.y1 / 100.0,
                           args.width / 100.0, args.depth / 100.0)
        how = "직사각형 (1번 + 가로·세로)"
    else:
        ap.print_help()
        print("\n네 점(--m1..--m4) 전부 또는 --x1 --y1 --width --depth 를 주십시오.")
        return 2

    cfg = host_config.load_host_config(args.config)
    print(f"입력: {how}\n")
    print(f"{'마커':<6}{'x (m)':>10}{'y (m)':>10}")
    for mid, (x, y) in sorted(points.items()):
        print(f"{mid:<6}{x:>10.3f}{y:>10.3f}")

    warn = check(points, cfg.arena.wall_x, cfg.arena.wall_y)
    print()
    for w in warn:
        print(f"⚠️ {w}")
    if not warn:
        print("배치 확인 이상 없음")

    print("\nhost/config/host.yaml 의 aruco: 블록에서 floor_markers 를 이렇게 바꾸십시오\n")
    print(as_yaml(points))
    print("\n바꾼 뒤 tools/place_markers.py 로 재투영오차를 확인하십시오 (1 px 미만이면 GOOD).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
