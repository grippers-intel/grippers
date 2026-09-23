"""카메라를 그 높이·거리·각도로 세우면 작업장이 덮이는지 **하드웨어 없이** 확인한다.

    python tools/check_coverage.py
    python tools/check_coverage.py --height 1.30 --setback 0.0 --tilt 42.8
    python tools/check_coverage.py --wall-height 0.25 --grid 40

가상 카메라 두 대를 앞뒤 가벽 쪽에 세우고, **가벽과 상자의 가림까지 반영해서**
바닥 마커와 작업 구역을 투영한다. 카메라를 실제로 옮기기 전에 배치를 먼저 판단하는 도구다.

## 왜 가림을 같이 보나

화각 안에 들어오는 것만으로는 부족하다. 카메라가 가벽 너머를 내려다보므로 **가벽 자체가
가까운 바닥을 가리고**, 상자는 그 뒤를 가린다. 이걸 빼면 "다 보인다"는 답이 나오지만
실제로는 근거리 띠가 통째로 죽는다.

## 나오는 값

- **마커 가시성** — 바닥 마커 4장을 각 카메라가 보는가. 카메라마다
  `aruco.min_floor_markers` 이상이어야 외부파라미터가 풀린다.
- **실효 커버 시작** — 카메라 쪽 가벽에서 몇 m 부터 실제로 보이기 시작하는가.
  작업 구역(`arena.workspace_y`)이 그 안에 들어와야 한다.
- **두 대 이중 관측 %** — 로봇 마커 높이 기준. 높을수록 pose 가 안정된다.
- **최악 mm/px** — 가장 먼 지점에서 1 px 이 몇 mm 인가. 40 mm 물체가 몇 px 로 잡히는지도 같이 낸다.

⚠️ 이 도구는 **기하만** 본다. 초점·노출·모션블러는 실물에서 `tools/place_markers.py` 로 본다.
⚠️ 카메라 위치는 `host.yaml` 에 없다(외부파라미터를 바닥 마커로 매번 푼다). 그래서 여기서는
   인자로 받는다 — 실제로 세운 값을 넣어야 답이 맞는다.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HOST_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST_ROOT))

#: 검출이 성립하는 최소 마커 변 길이(px). 이보다 작으면 ArUco 가 코너를 못 잡는다.
MIN_MARKER_PX = 12.0


def camera_pose(side: str, cam_x: float, setback: float, height: float, tilt_deg: float,
                wall_y) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """가상 카메라의 중심 C 와 map -> cam 회전 R, 이동 t.

    side 'A' 는 y<0 에서 +y 를 보고, 'B' 는 y>뒤벽 에서 -y 를 본다.
    """
    t_rad = math.radians(tilt_deg)
    if side == "A":
        C = np.array([cam_x, wall_y[0] - setback, height])
        f = np.array([0.0, math.cos(t_rad), -math.sin(t_rad)])   # 광축
        r = np.array([1.0, 0.0, 0.0])                            # 화면 오른쪽
    else:
        C = np.array([cam_x, wall_y[1] + setback, height])
        f = np.array([0.0, -math.cos(t_rad), -math.sin(t_rad)])
        r = np.array([-1.0, 0.0, 0.0])
    d = np.cross(f, r)                                           # 화면 아래쪽
    R = np.vstack([r, d, f])
    t = (-R @ C).reshape(3, 1)
    return C, R, t


def wall_blocks(C: np.ndarray, target: np.ndarray, wall_y, wall_height: float) -> bool:
    """카메라 앞 가벽이 시선을 막는가.

    카메라에서 목표점까지 직선이 가벽 평면을 지날 때의 높이가 가벽보다 낮으면 가려진다.
    """
    plane = wall_y[0] if C[1] < wall_y[0] else wall_y[1]
    dy = target[1] - C[1]
    if abs(dy) < 1e-9:
        return False
    s = (plane - C[1]) / dy
    if not (0.0 < s < 1.0):           # 가벽이 카메라와 목표 사이에 없다
        return False
    return (C[2] + (target[2] - C[2]) * s) < wall_height


def box_blocks(C: np.ndarray, target: np.ndarray, boxes, box_size) -> bool:
    """상자(직육면체)가 시선을 막는가 — 선분-AABB 교차."""
    d = target - C
    bw, bl, bh = box_size
    for bx, by, _yaw in boxes.values():
        lo = np.array([bx - bw / 2.0, by - bl / 2.0, 0.0])
        hi = np.array([bx + bw / 2.0, by + bl / 2.0, bh])
        t0, t1, hit = 0.0, 1.0, True
        for k in range(3):
            if abs(d[k]) < 1e-12:
                if C[k] < lo[k] or C[k] > hi[k]:
                    hit = False
                    break
                continue
            ta, tb = (lo[k] - C[k]) / d[k], (hi[k] - C[k]) / d[k]
            if ta > tb:
                ta, tb = tb, ta
            t0, t1 = max(t0, ta), min(t1, tb)
            if t0 > t1:
                hit = False
                break
        if hit and t0 < 0.999:        # 목표점 바로 앞까지만 막는 것으로 본다
            return True
    return False


def project(R, t, K, pts_3d) -> np.ndarray:
    """왜곡 없는 핀홀 투영(기하 판단에는 충분하다)."""
    X = (R @ np.asarray(pts_3d, dtype=np.float64).T + t).T
    z = X[:, 2:3]
    if np.any(z <= 1e-6):
        return np.full((len(X), 2), np.nan)
    return (K @ (X / z).T).T[:, :2]


def in_frame(px, w: int, h: int) -> bool:
    return bool(np.all(np.isfinite(px)) and np.all((px[:, 0] >= 0) & (px[:, 0] < w)
                                                   & (px[:, 1] >= 0) & (px[:, 1] < h)))


def square_corners(x: float, y: float, z: float, size: float) -> np.ndarray:
    h = size / 2.0
    return np.array([[x - h, y + h, z], [x + h, y + h, z],
                     [x + h, y - h, z], [x - h, y - h, z]], dtype=np.float64)


def sees(cam, target_xyz: np.ndarray, size: float, cfg, wall_height: float) -> tuple[bool, float]:
    """이 카메라가 그 점의 마커를 보는가. (보임, 마커 변 px) 를 돌려준다."""
    C, R, t = cam
    corners = square_corners(target_xyz[0], target_xyz[1], target_xyz[2], size)
    px = project(R, t, cfg["K"], corners)
    if not in_frame(px, cfg["w"], cfg["h"]):
        return False, 0.0
    if wall_blocks(C, target_xyz, cfg["wall_y"], wall_height):
        return False, 0.0
    if box_blocks(C, target_xyz, cfg["boxes"], cfg["box_size"]):
        return False, 0.0
    side_px = float(np.mean([np.linalg.norm(px[(k + 1) % 4] - px[k]) for k in range(4)]))
    return side_px >= MIN_MARKER_PX, side_px


def coverage(cams, cfg, z: float, size: float, wall_height: float, n: int) -> dict:
    """작업 구역 격자에서 몇 %가 보이는지. 반환: any/both %, 실효 커버 시작 y, 최악 mm/px."""
    xs = np.linspace(cfg["workspace_x"][0] + 0.05, cfg["workspace_x"][1] - 0.05, n)
    ys = np.linspace(cfg["workspace_y"][0], cfg["workspace_y"][1], n)
    grid = np.zeros((n, n), dtype=int)       # 본 카메라 수
    worst_mm_px, worst_at = 0.0, None
    for j, gy in enumerate(ys):
        for i, gx in enumerate(xs):
            target = np.array([gx, gy, z])
            for cam in cams:
                ok, side_px = sees(cam, target, size, cfg, wall_height)
                if ok:
                    grid[j, i] += 1
                    mm_px = size * 1000.0 / side_px
                    if mm_px > worst_mm_px:
                        worst_mm_px, worst_at = mm_px, (gx, gy)
    any_pct = 100.0 * float(np.mean(grid >= 1))
    both_pct = 100.0 * float(np.mean(grid >= 2))
    start_y = None
    for j, gy in enumerate(ys):
        if np.all(grid[j] >= 1):
            start_y = float(gy)
            break
    return {"grid": grid, "xs": xs, "ys": ys, "any": any_pct, "both": both_pct,
            "start_y": start_y, "worst_mm_px": worst_mm_px, "worst_at": worst_at}


def print_map(rep: dict) -> None:
    """위가 +y(상자 쪽). 2 = 두 대, 1 = 한 대, . = 사각지대."""
    print("      " + "".join("+" if i % 10 == 0 else " " for i in range(len(rep["xs"]))))
    for j in range(len(rep["ys"]) - 1, -1, -1):
        row = "".join("2" if v >= 2 else ("1" if v == 1 else ".") for v in rep["grid"][j])
        print(f"y={rep['ys'][j]:4.2f} {row}")


def main() -> int:
    import host_config
    host_config.configure_console()
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--cam-x", type=float, default=0.9, help="두 카메라의 좌우 위치(m)")
    ap.add_argument("--setback", type=float, default=0.0, help="가벽에서 뒤로 물러난 거리(m)")
    ap.add_argument("--height", type=float, default=1.30, help="렌즈 높이(m)")
    ap.add_argument("--tilt", type=float, default=42.8, help="하향 각도(도)")
    ap.add_argument("--wall-height", type=float, default=0.25, help="가벽 높이(m)")
    ap.add_argument("--grid", type=int, default=36)
    args = ap.parse_args()

    hc = host_config.load_host_config(args.config)
    from localization.aruco_localizer import approx_camera_matrix
    K = approx_camera_matrix(hc.cameras.width, hc.cameras.height, hc.cameras.hfov_deg)
    cfg = {"K": K, "w": hc.cameras.width, "h": hc.cameras.height,
           "wall_y": hc.arena.wall_y, "boxes": hc.arena.boxes, "box_size": hc.arena.box_size,
           "workspace_x": hc.arena.workspace_x, "workspace_y": hc.arena.workspace_y}
    cams = [camera_pose(s, args.cam_x, args.setback, args.height, args.tilt, hc.arena.wall_y)
            for s in ("A", "B")]

    print(f"카메라 2대: x={args.cam_x} m · 후퇴 {args.setback} m · 높이 {args.height} m · "
          f"하향 {args.tilt}° · HFOV {hc.cameras.hfov_deg}° · {hc.cameras.width}x{hc.cameras.height}")
    print(f"가벽 {hc.arena.wall_x} x {hc.arena.wall_y} m · 가벽 높이 {args.wall_height} m\n")

    print("바닥 마커 가시성")
    per_cam_seen = [0, 0]
    for mid, (mx, my) in sorted(hc.aruco.floor_markers.items()):
        cells = []
        for k, cam in enumerate(cams):
            ok, side_px = sees(cam, np.array([mx, my, 0.0]), hc.aruco.floor_marker_size_m,
                               cfg, args.wall_height)
            per_cam_seen[k] += int(ok)
            cells.append(f"{'보임' if ok else '가림'} {side_px:5.0f}px")
        print(f"  ID {mid} ({mx:.3f}, {my:.3f})   camA {cells[0]}   camB {cells[1]}")
    need = hc.aruco.min_floor_markers
    for k, n in enumerate(per_cam_seen):
        mark = "OK" if n >= need else "부족"
        print(f"  cam{'AB'[k]} 가 보는 바닥 마커 {n}/{len(hc.aruco.floor_markers)} — {mark} "
              f"(최소 {need})")

    rep = coverage(cams, cfg, hc.aruco.robot_marker_height_m, hc.aruco.robot_marker_size_m,
                   args.wall_height, args.grid)
    print(f"\n로봇 마커 높이 {hc.aruco.robot_marker_height_m} m 커버리지")
    print(f"  한 대 이상 {rep['any']:.0f} %   두 대 {rep['both']:.0f} %")
    if rep["start_y"] is None:
        print("  ⚠️ 작업 구역 전체가 덮이는 y 띠가 없습니다")
    else:
        print(f"  전 폭이 덮이기 시작하는 y = {rep['start_y']:.2f} m "
              f"(작업 구역 {hc.arena.workspace_y[0]}~{hc.arena.workspace_y[1]})")
    print(f"  최악 해상도 {rep['worst_mm_px']:.2f} mm/px"
          + (f" @ ({rep['worst_at'][0]:.2f}, {rep['worst_at'][1]:.2f})" if rep["worst_at"] else "")
          + f"  → 40 mm 물체 {40.0 / max(rep['worst_mm_px'], 1e-9):.1f} px")
    print()
    print_map(rep)

    ok = (rep["any"] >= 99.0 and all(n >= need for n in per_cam_seen))
    print()
    print("배치 판정: " + ("이상 없음" if ok else "⚠️ 위 경고를 보고 카메라를 다시 세우십시오"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
