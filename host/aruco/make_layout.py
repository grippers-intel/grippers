"""잰 숫자를 넣으면 config.py 에 넣을 마커 좌표를 계산해 준다.

전부 cm 단위로 넣으면 된다. 미터로 바꾸는 건 이 프로그램이 한다.

방법 1) 쉬운 방법 — 마커끼리의 거리만 재기 (추천)
    1번 마커 위치 + 가로폭 + 세로만 재면 나머지 3장은 계산으로 나온다.

    python make_layout.py --x1 40.3 --y1 45.1 --width 100.4 --depth 90.2

방법 2) 정확한 방법 — 4장을 각각 재기
    직사각형이 아니어도 되고, 비뚤어져 있어도 그대로 반영된다.

    python make_layout.py --m1 40.3 45.1 --m2 140.5 45.0 \
                          --m3 40.1 135.4 --m4 140.6 135.2

검산용 대각선을 같이 넣으면 잘 붙였는지 확인해 준다.
    ... --diag14 134.6 --diag23 134.2

계산 결과를 config.py 에 바로 써 넣으려면 맨 뒤에 --write 를 붙인다.
(원본은 config_backup.py 로 자동 저장된다)
"""

import argparse
import math
import re
import shutil
import sys
from pathlib import Path

import config as cfg

CONFIG = Path(__file__).with_name("config.py")

# 바닥 마커는 이 세로 범위 안에 있어야 두 카메라가 4장 모두 본다.
# (현재 카메라 배치 기준. 밖에 두면 한 대가 2장만 보게 된다)
SAFE_Y_CM = (30.0, 150.0)


def build(args) -> dict[int, tuple[float, float]]:
    if args.m1:
        got = {1: args.m1, 2: args.m2, 3: args.m3, 4: args.m4}
        missing = [k for k, v in got.items() if v is None]
        if missing:
            sys.exit(f"오류: --m{missing[0]} 처럼 4장을 모두 넣어야 합니다.")
        return {k: (v[0], v[1]) for k, v in got.items()}

    for name, val in (("--x1", args.x1), ("--y1", args.y1),
                      ("--width", args.width), ("--depth", args.depth)):
        if val is None:
            sys.exit(f"오류: {name} 값이 없습니다. 사용법은 맨 위 설명을 보세요.")
    x1, y1, w, d = args.x1, args.y1, args.width, args.depth
    return {1: (x1, y1), 2: (x1 + w, y1), 3: (x1, y1 + d), 4: (x1 + w, y1 + d)}


def report(pts: dict[int, tuple[float, float]], args) -> bool:
    """검산. 문제가 있으면 False."""
    ok = True
    print("\n" + "=" * 62)
    print("잰 값 확인")
    print("=" * 62)

    def dist(a, b):
        return math.hypot(pts[a][0] - pts[b][0], pts[a][1] - pts[b][1])

    print(f"  가로  1-2 : {dist(1,2):7.1f} cm      3-4 : {dist(3,4):7.1f} cm")
    print(f"  세로  1-3 : {dist(1,3):7.1f} cm      2-4 : {dist(2,4):7.1f} cm")
    d14, d23 = dist(1, 4), dist(2, 3)
    print(f"  대각선 1-4 : {d14:7.1f} cm     2-3 : {d23:7.1f} cm  "
          f"(차이 {abs(d14-d23)*10:.0f} mm)")

    if abs(d14 - d23) > 0.5:
        print(f"  ⚠ 두 대각선이 {abs(d14-d23)*10:.0f} mm 차이납니다 "
              f"— 직사각형이 아닙니다.")
        if args.m1:
            print("    4장을 각각 잰 값이므로 비뚤어진 그대로 반영됩니다.")
            print("    잰 값이 맞다면 이대로 써도 됩니다.")
        else:
            print("    폭/세로만으로는 직사각형이라고 가정합니다.")
            print("    --m1 --m2 --m3 --m4 로 4장을 각각 재서 넣으세요.")

    # 줄자로 실제 잰 대각선과 비교
    for label, given, calc in (("1-4", args.diag14, d14), ("2-3", args.diag23, d23)):
        if given is None:
            continue
        gap = abs(given - calc) * 10
        mark = "OK" if gap <= 5 else "⚠ 다시 재세요"
        print(f"  줄자로 잰 {label} 대각선 {given:.1f} cm vs 계산값 "
              f"{calc:.1f} cm  -> 차이 {gap:.0f} mm  {mark}")
        if gap > 5:
            ok = False

    print()
    span_x = max(p[0] for p in pts.values()) - min(p[0] for p in pts.values())
    span_y = max(p[1] for p in pts.values()) - min(p[1] for p in pts.values())
    print(f"  마커가 차지하는 범위 : 가로 {span_x:.1f} cm x 세로 {span_y:.1f} cm")
    if span_x < 60 or span_y < 55:
        print("  ⚠ 너무 좁습니다. 넓을수록 정확해집니다 (가로 80cm 이상 권장)")

    lo, hi = SAFE_Y_CM
    bad = [k for k, (x, y) in pts.items() if not (lo <= y <= hi)]
    if bad:
        print(f"  ⚠ 마커 {bad} 의 세로 위치가 {lo:.0f}~{hi:.0f} cm 를 벗어납니다.")
        print("    이 범위 밖이면 한쪽 카메라가 그 마커를 못 봅니다.")
        ok = False
    else:
        print(f"  세로 위치 {lo:.0f}~{hi:.0f} cm 범위 안 : OK")

    # 상자 그림자 검사.
    # 상자(높이 22 cm)는 지면에 그림자를 드리운다. 마커가 상자와 좌우 위치가
    # 겹치면서 앞뒤로도 가까우면, 그쪽 카메라가 마커를 통째로 못 본다.
    # 시뮬레이션으로 잰 결과 상자에서 21 cm 까지가 그림자였다 → 25 cm 로 잡는다.
    # (좌우가 겹쳐도 25 cm 이상 떨어져 있으면 괜찮다)
    SHADOW_CM = 25.0
    ws_mid = (cfg.WORKSPACE_Y[0] + cfg.WORKSPACE_Y[1]) * 50   # 작업 영역 중앙 (cm)
    shaded = []
    for k, (mx, my) in pts.items():
        m0, m1 = mx - cfg.FLOOR_MARKER_SIZE * 50, mx + cfg.FLOOR_MARKER_SIZE * 50
        for name, (bx, by, _) in cfg.BOXES.items():
            b0, b1 = bx * 100 - cfg.BOX_W * 50, bx * 100 + cfg.BOX_W * 50
            if not (m1 > b0 and m0 < b1):
                continue                      # 좌우가 안 겹치면 그림자와 무관
            # 상자에서 작업 영역 쪽을 향한 면
            byc = by * 100
            edge = byc - cfg.BOX_L * 50 if byc > ws_mid else byc + cfg.BOX_L * 50
            gap = abs(my - edge)
            inside = (my < edge) if byc > ws_mid else (my > edge)
            if inside and gap < SHADOW_CM:
                shaded.append(f"{k}번↔{name}({gap:.0f}cm)")
    if shaded:
        print(f"  ⚠ 상자 그림자에 들어간 마커: {', '.join(shaded)}")
        print(f"    좌우가 겹치는 상자에서 {SHADOW_CM:.0f} cm 이상 떨어져야 합니다.")
        print("    마커를 앞으로 당기거나, 좌우로 옮겨 상자를 피하세요.")
        ok = False
    else:
        print("  상자 그림자 피함 : OK")
    return ok


def block(pts: dict[int, tuple[float, float]]) -> str:
    names = {1: "앞쪽 왼편", 2: "앞쪽 오른편",
             3: "1번의 반대편 대칭점", 4: "2번의 반대편 대칭점"}
    lines = ["FLOOR_MARKER_WORLD = {"]
    for k in (1, 2, 3, 4):
        x, y = pts[k]
        lines.append(f"    {k}: ({x/100:.3f}, {y/100:.3f}),   # {names[k]}")
    lines.append("}")
    return "\n".join(lines)


def write_config(new_block: str) -> None:
    if not CONFIG.exists():
        sys.exit(f"오류: {CONFIG} 를 찾을 수 없습니다.")
    text = CONFIG.read_text(encoding="utf-8")
    pat = re.compile(r"^FLOOR_MARKER_WORLD = \{.*?^\}", re.S | re.M)
    if not pat.search(text):
        sys.exit("오류: config.py 안에서 FLOOR_MARKER_WORLD 를 찾지 못했습니다.")
    backup = CONFIG.with_name("config_backup.py")
    shutil.copy2(CONFIG, backup)
    CONFIG.write_text(pat.sub(new_block.replace("\\", "\\\\"), text), encoding="utf-8")
    print(f"\nconfig.py 에 반영했습니다. (원본은 {backup.name} 에 저장)")
    print("이제 'python selftest.py' 를 한 번 돌려 확인하세요.")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="마커 좌표 계산기 (모든 값은 cm)",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--x1", type=float, help="1번 마커 중심의 오른쪽 거리(cm)")
    ap.add_argument("--y1", type=float, help="1번 마커 중심의 뒤쪽 거리(cm)")
    ap.add_argument("--width", type=float, help="1번-2번 중심 거리(cm)")
    ap.add_argument("--depth", type=float, help="1번-3번 중심 거리(cm)")
    for k in (1, 2, 3, 4):
        ap.add_argument(f"--m{k}", type=float, nargs=2, metavar=("X", "Y"),
                        help=f"{k}번 마커 중심 좌표(cm)")
    ap.add_argument("--diag14", type=float, help="줄자로 잰 1-4 대각선(cm)")
    ap.add_argument("--diag23", type=float, help="줄자로 잰 2-3 대각선(cm)")
    ap.add_argument("--write", action="store_true", help="config.py 에 바로 반영")
    args = ap.parse_args()

    if not any([args.m1, args.x1 is not None]):
        ap.print_help()
        return 1

    pts = build(args)
    ok = report(pts, args)

    print("\n" + "=" * 62)
    print("config.py 에 넣을 내용")
    print("=" * 62)
    print(block(pts))
    print("=" * 62)

    if args.write:
        if not ok:
            print("\n⚠ 위에 경고가 있어 반영하지 않았습니다. 다시 재고 확인하세요.")
            print("  그래도 넣으려면 경고를 해결한 뒤 --write 를 다시 쓰세요.")
            return 1
        write_config(block(pts))
    else:
        print("\n위 내용을 config.py 의 같은 부분과 바꿔 넣으세요.")
        print("자동으로 넣으려면 명령 끝에 --write 를 붙이세요.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
