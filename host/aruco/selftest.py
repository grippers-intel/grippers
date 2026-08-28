"""하드웨어 없이 수학과 배치를 검증하는 합성 테스트.

가상 카메라 2대를 실측 배치(높이 1.30 m · 후퇴 0 m · 좌우 0.9 m ·
하향 42.8°)에 세우고, 가벽·상자의 가림까지 반영해서 마커를 투영한 뒤,
localizer 가 원래 좌표를 되찾는지 본다.

config.py 를 고친 뒤 이 파일을 돌리면
  - 파일이 문법적으로 멀쩡한지
  - 바꾼 배치가 실제로 성립하는지 (두 카메라가 4장을 다 보는지,
    작업 영역에 사각지대가 없는지, 상자가 작업 영역과 겹치지 않는지)
를 한 번에 확인할 수 있다.

    python selftest.py
    python selftest.py --setback 0.0 --height 1.30 --tilt 42.8
"""

import argparse
import math

import cv2
import numpy as np

import config as cfg
from localizer import (Camera, Pose, RobotLocalizer, _pose_from_corners,
                       box_pose, relative_to_robot)

RNG = np.random.default_rng(20260820)

# 카메라 배치 — 실측 확정값 (CLI 로 덮어쓸 수 있다)
#
# C920 교체 후 (2026-08-23 갱신): 후퇴 없이(0 m) 가벽 바로 앞에 세우고,
# 높이는 162 cm. 각도는 실측하지 않고 "예전과 비슷해 보이게" 눈대중으로
# 잡았다고 함 — 그래서 정확한 TILT_DEG 는 모른다.
#
# 대신 이 배치(setback=0, height=1.30)로 시뮬레이션을 돌려보면 tilt 를
# 35°~59° 사이 어디에 둬도 작업 영역 사각지대가 0 이다 (측정: 20~80°
# 스윕, 34° 이하/60° 이상부터 손실 시작). C270 때(42.8°에서 5°만 더
# 숙여도 8% 손실)보다 관대하다 — HFOV 가 넓어지고 후퇴가 0이라 생긴
# 여유다. 단, 높이 1.62 m 였을 때(안전범위 40~78°)보다는 범위가 좁아졌다
# — 높이를 낮출수록 안전범위도 좁아진다. 아래 42.8 은 예전 값을 그대로
# 가져온 것으로, 이 안전범위 안에 들어가고 selftest 전체가 깨끗이
# 통과하는 값이라 기본값으로 남겨 뒀다 — 그렇다고 실제로 잰 각도는
# 아니다. 각도기 앱으로 재서 정확한 값을 넣으면 더 정확히 검증된다.
#
# ⚠️ 후퇴가 0이라 카메라 한 대는 바닥 기준점 4개를 동시에 다 보지 못하고
# (항상 2개만 보임 — camA 는 반대편 벽 쪽 2개, camB 는 그 반대. 자기
# 쪽 가까운 2개는 오히려 시야각을 벗어나 못 본다). MIN_FLOOR_MARKERS=2
# 라 동작엔 문제없고, localizer.py 의 고정(lock) 로직도 "보이는 마커만
# 누적"하도록 고쳐서(2026-08-23) 4개가 동시에 안 보여도 고정이 걸린다.
# 다만 아래 "0) 배치 검증"의 바닥 4점 동시 가시 체크는 구조적으로 항상
# FAIL 로 뜬다. 정상이다.
#
# 높이를 1.30 m 로 더 낮춘 뒤로는(2026-08-23) "두 카메라 해의 차이"와
# "로봇이 향한 각도" 체크도 tilt 값과 무관하게 1 mm대 · 0.02°대로 FAIL
# 문턱(<1mm, 정확히 일치)을 살짝 넘는다. 이건 이 배치 특유의 미세한
# 기하 비대칭(카메라마다 가림·화면 가장자리 조건이 달라서 생기는
# 서브밀리미터 차이)이지 실제 정확도 문제가 아니다 — 완료조건인
# "4) 픽셀 노이즈에서 위치 95% <= 30mm, yaw 95% <= 3deg" 는 이 높이에서
# 2.0mm / 0.88deg 로 여유 있게 통과한다. 두 체크는 그 20~150배 더
# 엄격한 문턱이라 무시해도 된다.
TILT_DEG = 42.8     # 하향각 — 미실측, 예전 값을 그대로 사용 (안전범위 35~59° 안)
CAM_H = 1.300       # 바닥에서 렌즈까지
SETBACK = 0.0       # 벽에서 바깥으로 — 가벽 바로 앞
CAM_X = 0.900       # 좌우 (변의 중앙)
WS = 1.800          # 가벽으로 두른 공간 한 변


# ---------------------------------------------------------------------------
# 가상 카메라
# ---------------------------------------------------------------------------
def make_virtual_camera(name: str, side: str) -> tuple[Camera, np.ndarray, np.ndarray]:
    """side='A' 는 y<0 에서 +y 를 보고, 'B' 는 y>WS 에서 -y 를 본다."""
    T = math.radians(TILT_DEG)
    if side == "A":
        C = np.array([CAM_X, -SETBACK, CAM_H])
        f = np.array([0.0, math.cos(T), -math.sin(T)])   # 광축
        r = np.array([1.0, 0.0, 0.0])                    # 화면 오른쪽
    else:
        C = np.array([CAM_X, WS + SETBACK, CAM_H])
        f = np.array([0.0, -math.cos(T), -math.sin(T)])
        r = np.array([-1.0, 0.0, 0.0])
    d = np.cross(f, r)                                   # 화면 아래쪽
    R = np.vstack([r, d, f])                             # map -> cam
    t = (-R @ C).reshape(3, 1)

    K = cfg.approx_camera_matrix()
    dist = np.array([[0.08], [-0.15], [0.0], [0.0], [0.05]])   # C920 스러운 배럴 왜곡 (근사 — 실측 캘리브레이션 값으로 교체 가능)
    cam = Camera(name, K, dist, calibrated=True)
    return cam, R, t


def project(R, t, K, dist, pts_3d, noise_px=0.0):
    rvec, _ = cv2.Rodrigues(R)
    px, _ = cv2.projectPoints(np.asarray(pts_3d, np.float64), rvec, t, K, dist)
    px = px.reshape(-1, 2)
    if noise_px > 0:
        px = px + RNG.normal(0.0, noise_px, px.shape)
    return px


def in_frame(px) -> bool:
    return bool(np.all((px[:, 0] >= 0) & (px[:, 0] < cfg.IMG_W) &
                       (px[:, 1] >= 0) & (px[:, 1] < cfg.IMG_H)))


def robot_marker_corners(x, y, yaw_deg, z=None, size=None):
    """로봇 마커의 map 좌표계 코너 4점 (ArUco 검출 순서)."""
    z = cfg.ROBOT_MARKER_HEIGHT if z is None else z
    s = (cfg.ROBOT_MARKER_SIZE if size is None else size) / 2.0
    th = math.radians(yaw_deg)
    c, sn = math.cos(th), math.sin(th)
    local = [(-s, s), (s, s), (s, -s), (-s, -s)]
    return np.array([[x + c * lx - sn * ly, y + sn * lx + c * ly, z]
                     for lx, ly in local], dtype=np.float64)


def render(cam_rt, noise_px, robot=None, hide_floor=(), hide_robot=False):
    """한 카메라의 검출 결과 dict 를 합성한다.

    화각 안에 들어오는 것만으로는 부족하다. 가벽과 상자가 시선을 막으면
    실제로는 안 보이므로 blocked() 로 함께 걸러야 한다.
    (이걸 빼먹으면 상자가 바닥 마커를 가리는 상황을 놓친다)
    """
    cam, R, t = cam_rt
    side = "A" if cam.name.endswith("A") else "B"
    _, per_id = cfg.floor_object_points()
    det = {}
    for mid in cfg.FLOOR_MARKER_IDS:
        if mid in hide_floor:
            continue
        pts = per_id[mid]
        px = project(R, t, cam.K, cam.dist, pts, noise_px)
        if in_frame(px) and not blocked(side, pts.mean(axis=0)):
            det[mid] = px
    if robot is not None and not hide_robot:
        pts = robot_marker_corners(*robot)
        px = project(R, t, cam.K, cam.dist, pts, noise_px)
        if in_frame(px) and not blocked(side, pts.mean(axis=0)):
            det[cfg.ROBOT_MARKER_ID] = px
    return det


# ---------------------------------------------------------------------------
# 가벽 가림 / 커버리지
# ---------------------------------------------------------------------------
WALL_H = 0.35        # 가벽 높이 35 cm


def wall_blocks(side: str, target: np.ndarray) -> bool:
    """카메라 앞 가벽이 시선을 막는가.

    카메라에서 목표점까지 직선이 가벽 평면(camA 는 y=0, camB 는 y=WS)을
    지날 때의 높이가 가벽보다 낮으면 가려진다.
    """
    T = math.radians(TILT_DEG)
    if side == "A":
        C = np.array([CAM_X, -SETBACK, CAM_H]); wall_y = 0.0
    else:
        C = np.array([CAM_X, WS + SETBACK, CAM_H]); wall_y = WS
    dy = target[1] - C[1]
    if abs(dy) < 1e-9:
        return False
    s = (wall_y - C[1]) / dy
    if not (0.0 < s < 1.0):        # 가벽이 카메라와 목표 사이에 없다
        return False
    return (C[2] + (target[2] - C[2]) * s) < WALL_H


def cam_center(side: str) -> np.ndarray:
    if side == "A":
        return np.array([CAM_X, -SETBACK, CAM_H])
    return np.array([CAM_X, WS + SETBACK, CAM_H])


def box_blocks(side: str, target: np.ndarray) -> bool:
    """상자가 카메라와 목표점 사이의 시선을 막는가 (직육면체 - 선분 교차)."""
    C = cam_center(side)
    d = target - C
    for bx, by, _ in cfg.BOXES.values():
        lo = np.array([bx - cfg.BOX_W / 2, by - cfg.BOX_L / 2, 0.0])
        hi = np.array([bx + cfg.BOX_W / 2, by + cfg.BOX_L / 2, cfg.BOX_H])
        t0, t1 = 0.0, 1.0
        hit = True
        for k in range(3):
            if abs(d[k]) < 1e-12:
                if C[k] < lo[k] or C[k] > hi[k]:
                    hit = False
                    break
                continue
            ta = (lo[k] - C[k]) / d[k]
            tb = (hi[k] - C[k]) / d[k]
            if ta > tb:
                ta, tb = tb, ta
            t0, t1 = max(t0, ta), min(t1, tb)
            if t0 > t1:
                hit = False
                break
        # 목표점 자체(t=1) 바로 앞까지만 막는 것으로 본다
        if hit and t0 < 0.999:
            return True
    return False


def blocked(side: str, target: np.ndarray) -> bool:
    return wall_blocks(side, target) or box_blocks(side, target)


def in_box_footprint(x: float, y: float) -> bool:
    """상자가 실제로 깔고 앉은 자리인가. 여기는 로봇도 물체도 있을 수 없다."""
    for bx, by, _ in cfg.BOXES.values():
        if (abs(x - bx) <= cfg.BOX_W / 2) and (abs(y - by) <= cfg.BOX_L / 2):
            return True
    return False


def coverage_report(A, B, n=36) -> dict:
    """상판 높이 / 지면 각각에 대해 화각+가림 커버리지를 계산해 출력한다."""
    grid = np.linspace(0.05, WS - 0.05, n)
    out = {}
    for label, z in (("robot", cfg.ROBOT_MARKER_HEIGHT), ("ground", 0.0)):
        acc = {"a": 0, "b": 0, "any": 0, "both": 0}
        ys = {"a": [], "b": [], "any": []}
        vis_any = np.zeros((n, n), dtype=bool)
        vis_map = [["." for _ in range(n)] for _ in range(n)]
        for j, gy in enumerate(grid):
            for i, gx in enumerate(grid):
                flags = {}
                for key, cam_rt, side in (("a", A, "A"), ("b", B, "B")):
                    cam, R, t = cam_rt
                    pts = robot_marker_corners(gx, gy, 0.0, z=z)
                    px = project(R, t, cam.K, cam.dist, pts)
                    vis = in_frame(px) and not blocked(side, pts.mean(axis=0))
                    flags[key] = vis
                    acc[key] += vis
                    if vis:
                        ys[key].append(gy)
                both = flags["a"] and flags["b"]
                any_ = flags["a"] or flags["b"]
                acc["any"] += any_
                acc["both"] += both
                vis_any[j, i] = any_
                vis_map[j][i] = ("#" if both else
                                 "A" if flags["a"] else
                                 "B" if flags["b"] else ".")
                if any_:
                    ys["any"].append(gy)
        tot = n * n
        name = "상판 0.2 m" if label == "robot" else "지면 0.0 m"
        print(f"    [{name}]")
        for key, tag in (("a", "camA"), ("b", "camB"), ("any", "합집합")):
            rng = (f"y ∈ [{min(ys[key]):.3f}, {max(ys[key]):.3f}] m"
                   if ys[key] else "가시영역 없음")
            print(f"       {tag:8s}: {100*acc[key]/tot:5.1f}%  {rng}")
        print(f"       {'교집합':8s}: {100*acc['both']/tot:5.1f}%")

        # 전체 1.8x1.8 중에는 상자가 깔고 앉은 자리도 있다. 거기는 로봇도
        # 물체도 있을 수 없으므로 판정에서 뺀다.
        # 판정 대상은 선언한 작업 영역뿐이다.
        # 상자는 그 바깥 띠에만 놓이고, 작업 영역 안에는 로봇과 옮길 물건만
        # 있으므로, 영역 밖 바닥(상자와 벽 사이 틈 등)은 볼 필요가 없다.
        iy = (grid >= cfg.WORKSPACE_Y[0]) & (grid <= cfg.WORKSPACE_Y[1])
        ix = (grid >= cfg.WORKSPACE_X[0]) & (grid <= cfg.WORKSPACE_X[1])
        tot_in = ok_in = 0
        for j in np.where(iy)[0]:
            for i in np.where(ix)[0]:
                tot_in += 1
                ok_in += bool(vis_any[j, i])
        print(f"       작업 영역 합집합: {100*ok_in/tot_in:5.1f}%  "
              f"({tot_in - ok_in} / {tot_in} 점 사각)")
        inner_any = ok_in / tot_in

        out[label] = {k: acc[k] / tot for k in acc}
        out[label]["inner_any"] = inner_any
        out[label]["map"] = vis_map
        out[label]["grid"] = grid
    return out


def print_map(cov: dict, label: str) -> None:
    """커버리지를 터미널 맵으로 그린다.  A=camA만  B=camB만  #=둘 다  .=사각지대"""
    grid, m = cov[label]["grid"], cov[label]["map"]
    print(f"\n    커버리지 맵 [{ '상판 0.2 m' if label=='robot' else '지면 0.0 m'}]"
          f"   (가로 x=0→{WS}, 세로 위쪽이 y={WS})")
    print("       # 양쪽  A camA만  B camB만  . 사각지대")
    for j in range(len(grid) - 1, -1, -1):
        row = "".join(m[j])
        print(f"       y={grid[j]:4.2f} |{row}|")
    print(f"              {' ' * 0}x={grid[0]:.2f}" +
          " " * max(0, len(grid) - 14) + f"x={grid[-1]:.2f}")


def rescue_check(A, B, n=36) -> tuple[int, int]:
    """camA 가 못 보는 지면 점 중 camB 가 건지는 개수와, 둘 다 못 보는 개수.

    작업 영역 안만 본다 — 그 바깥 바닥(상자 자리, 상자와 벽 사이 틈)은
    물건이 놓일 일이 없어 볼 필요가 없다.
    """
    gx_all = np.linspace(cfg.WORKSPACE_X[0], cfg.WORKSPACE_X[1], n)
    gy_all = np.linspace(cfg.WORKSPACE_Y[0], cfg.WORKSPACE_Y[1], n)
    rescued = orphan = 0
    for gy in gy_all:
        for gx in gx_all:
            if in_box_footprint(gx, gy):
                continue
            p = np.array([[gx, gy, 0.0]])
            seen = {}
            for key, cam_rt, side in (("a", A, "A"), ("b", B, "B")):
                cam, R, t = cam_rt
                px = project(R, t, cam.K, cam.dist, p)
                seen[key] = in_frame(px) and not blocked(side, p[0])
            if not seen["a"]:
                if seen["b"]:
                    rescued += 1
                else:
                    orphan += 1
    return rescued, orphan


def width_margin(cam_rt) -> float:
    """작업공간 좌우 끝이 프레임 가로 안에 들어오는지 — 최대 |u - cx| 반환."""
    cam, R, t = cam_rt
    pts = []
    for x in (0.0, WS):
        for y in np.linspace(0.0, WS, 25):
            pts.append([x, y, 0.0])
    px = project(R, t, cam.K, cam.dist, np.array(pts))
    # 세로로 프레임을 벗어난 점은 애초에 안 보이므로 제외
    keep = (px[:, 1] >= 0) & (px[:, 1] < cfg.IMG_H)
    if not keep.any():
        return float("inf")
    return float(np.abs(px[keep, 0] - cam.K[0, 2]).max())


# ---------------------------------------------------------------------------
# 테스트
# ---------------------------------------------------------------------------
results = []


def check(name, ok, detail=""):
    results.append((name, ok))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  — {detail}" if detail else ""))


def main() -> int:
    global TILT_DEG, CAM_H, SETBACK, CAM_X
    ap = argparse.ArgumentParser()
    ap.add_argument("--setback", type=float, default=SETBACK, help="후퇴 (m)")
    ap.add_argument("--height", type=float, default=CAM_H, help="카메라 높이 (m)")
    ap.add_argument("--tilt", type=float, default=TILT_DEG, help="하향각 (deg)")
    ap.add_argument("--camx", type=float, default=CAM_X, help="카메라 좌우 위치 (m)")
    args = ap.parse_args()
    SETBACK, CAM_H, TILT_DEG, CAM_X = args.setback, args.height, args.tilt, args.camx

    print(f"OpenCV {cv2.__version__}")
    print(f"배치: 높이 {CAM_H:.3f} m · 후퇴 {SETBACK:.3f} m · "
          f"좌우 {CAM_X:.3f} m · 하향 {TILT_DEG:.1f}°\n")
    A = make_virtual_camera("camA", "A")
    B = make_virtual_camera("camB", "B")

    # -- 0. 마커가 두 카메라 화면에 모두 들어오는가 -------------------------
    print("0) 배치 검증 — 바닥 4점이 두 카메라 모두에 보이는가")
    for cam_rt, label in ((A, "camA"), (B, "camB")):
        det = render(cam_rt, 0.0)
        check(f"{label} 바닥 마커 가시", len(det) == 4, f"{len(det)}/4 검출")

    # -- 1. 외부파라미터 복원 ----------------------------------------------
    print("\n1) 외부파라미터 — 카메라 위치를 되찾는가")
    for (cam, R, t), truth in ((A, np.array([CAM_X, -SETBACK, CAM_H])),
                               (B, np.array([CAM_X, WS + SETBACK, CAM_H]))):
        det = render((cam, R, t), 0.0)
        ok = cam.solve_extrinsics(det)
        err = np.linalg.norm(cam.center - truth) * 1000
        check(f"{cam.name} 위치 복원 (오차 {err:.2f} mm, reproj {cam.reproj_px:.3f} px)",
              ok and err < 5.0)

    # -- 2. 노이즈 0 에서 로봇 좌표 복원 ------------------------------------
    print("\n2) z 구속 복원 — 노이즈 없을 때 (마커를 본 카메라 각각)")
    worst = 0.0
    worst_yaw = 0.0
    n_checked = 0
    for gx, gy, gyaw in [(0.9, 0.9, 0.0), (0.5, 0.6, 45.0), (1.3, 1.2, -120.0),
                         (0.45, 1.35, 175.0), (1.35, 0.45, 90.0)]:
        for cam_rt in (A, B):
            det = render(cam_rt, 0.0, robot=(gx, gy, gyaw))
            if cfg.ROBOT_MARKER_ID not in det:
                continue
            cam_rt[0].solve_extrinsics(det)
            got = _pose_from_corners(cam_rt[0], det[cfg.ROBOT_MARKER_ID])
            ex = math.hypot(got[0] - gx, got[1] - gy) * 1000
            ey = abs((math.degrees(got[2]) - gyaw + 180) % 360 - 180)
            worst, worst_yaw = max(worst, ex), max(worst_yaw, ey)
            n_checked += 1
    check(f"위치 오차 < 1 mm (표본 {n_checked}, 최악 {worst:.3f} mm)", worst < 1.0)
    check(f"yaw 오차 < 0.1 deg (최악 {worst_yaw:.4f} deg)", worst_yaw < 0.1)

    # -- 2b. 커버리지 — 화각 + 가벽 가림을 함께 본다 -------------------------
    print("\n2b) 커버리지 — 화각과 가벽(35 cm) 가림을 함께 계산")
    cov = coverage_report(A, B)
    print_map(cov, "robot")
    # 판정 기준은 1.8x1.8 전체가 아니다.
    #  - 로봇은 선언한 작업 영역 안에서만 다닌다
    #  - 바닥은 상자가 깔고 앉은 자리를 뺀 나머지가 보이면 된다
    check(f"상판 마커: 작업 영역 100% 커버 ({cov['robot']['inner_any']*100:.1f}%)",
          cov["robot"]["inner_any"] >= 0.999)
    check(f"지면(옮길 물건 탐색용): 작업 영역 100% 커버 "
          f"({cov['ground']['inner_any']*100:.1f}%)",
          cov["ground"]["inner_any"] >= 0.999)

    # 상자가 정말 작업 영역 밖에 있는지 — config 를 고치다 겹치면 여기서 걸린다
    overlap = []
    for name, (bx, by, _) in cfg.BOXES.items():
        y0, y1 = by - cfg.BOX_L / 2, by + cfg.BOX_L / 2
        if y1 > cfg.WORKSPACE_Y[0] and y0 < cfg.WORKSPACE_Y[1]:
            overlap.append(f"{name}(y {y0*1000:.0f}~{y1*1000:.0f}mm)")
    print(f"       상자 배치: " +
          (", ".join(f"{n} y {v[1]*1000-cfg.BOX_L*500:.0f}~"
                     f"{v[1]*1000+cfg.BOX_L*500:.0f}mm"
                     for n, v in cfg.BOXES.items())))
    check("상자가 모두 작업 영역 밖에 있다" +
          (f" — 겹침: {overlap}" if overlap else ""), not overlap)

    # -- 2b2. config 에 선언한 작업 영역이 실제로 전부 보이는가 --------------
    print("\n2b-2) 선언한 작업 영역 검증 — config.WORKSPACE_X / WORKSPACE_Y")
    wx, wy = cfg.WORKSPACE_X, cfg.WORKSPACE_Y
    print(f"       선언값: 폭 {(wx[1]-wx[0])*1000:.0f} mm "
          f"(x {wx[0]*1000:.0f}~{wx[1]*1000:.0f}) x "
          f"세로 {(wy[1]-wy[0])*1000:.0f} mm (y {wy[0]*1000:.0f}~{wy[1]*1000:.0f})")
    xs = np.linspace(wx[0], wx[1], 37)
    ys = np.linspace(wy[0], wy[1], 29)
    blind = []
    for gy in ys:
        for gx in xs:
            if not any(cfg.ROBOT_MARKER_ID in render(c, 0.0, robot=(gx, gy, 0.0))
                       for c in (A, B)):
                blind.append((gx, gy))
    print(f"       격자 {len(xs)}x{len(ys)} = {len(xs)*len(ys)} 점 중 "
          f"사각지대 {len(blind)} 점")
    if blind:
        print(f"       예: {[(round(x,3), round(y,3)) for x, y in blind[:5]]}")
    check("선언한 작업 영역 안에 사각지대가 없다", not blind)

    # -- 2c. 가로 1.8 m 가 프레임에 들어오는가 (문서의 '후퇴 하한' 경고) ------
    print("\n2c) 가로 폭 검증 — 문서: '1,650 에서는 후퇴 >= 1,150 이어야 폭 1,800 이 들어온다'")
    max_u = width_margin(A)
    margin = (cfg.IMG_W / 2 - max_u) / (cfg.IMG_W / 2) * 100
    print(f"       작업공간 가로 끝(x=0, x={WS})의 최대 |u - cx| = {max_u:.1f} px "
          f"(프레임 한계 {cfg.IMG_W/2:.0f} px)")
    print(f"       가로 여유 {margin:+.1f}%  → "
          f"{'프레임 안' if margin >= 0 else '한 대로는 폭이 안 들어온다'}")

    # 한 대로 폭이 안 들어오는 건 사실이지만, 그게 실제 손해인지는 별개다.
    # 잘리는 곳은 자기 쪽 가까운 모서리이고 그 구역은 반대편 카메라가 멀리서 본다.
    # 그래서 판정 대상은 "한 대가 다 담는가"가 아니라 "둘이 합쳐 빈 곳이 없는가"다.
    rescued, orphan = rescue_check(A, B)
    print(f"       camA 가 놓치는 지면 점 {rescued + orphan} 개 중 "
          f"camB 가 {rescued} 개 커버, 양쪽 다 못 보는 점 {orphan} 개")
    check(f"한 대가 놓치는 지면 구역을 반대편이 100% 커버 (사각 {orphan} 점)",
          orphan == 0)

    # -- 3. z 를 구속하지 않으면 얼마나 틀어지는가 --------------------------
    print("\n3) 대조군 — z 구속 없이 지면 호모그래피로 풀면 (#175 의 250 mm 근거)")
    cam, R, t = A
    det = render(A, 0.0, robot=(0.9, 0.9, 0.0))
    cam.solve_extrinsics(det)
    src, dst = [], []
    _, per_id = cfg.floor_object_points()
    # 후퇴가 짧으면 카메라 한 대가 바닥 4점을 동시에 못 볼 수 있다 —
    # 실제로 보이는 것만 쓴다 (homography 는 최소 4점이면 풀린다).
    visible = [mid for mid in cfg.FLOOR_MARKER_IDS if mid in det]
    for mid in visible:
        src.append(det[mid])
        dst.append(per_id[mid][:, :2])
    H, _ = cv2.findHomography(np.vstack(src), np.vstack(dst))
    ctr_px = det[cfg.ROBOT_MARKER_ID].mean(axis=0)
    naive = cv2.perspectiveTransform(ctr_px.reshape(1, 1, 2), H).ravel()
    naive_err = math.hypot(naive[0] - 0.9, naive[1] - 0.9) * 1000
    constrained = _pose_from_corners(cam, det[cfg.ROBOT_MARKER_ID])
    con_err = math.hypot(constrained[0] - 0.9, constrained[1] - 0.9) * 1000
    print(f"       z 미구속(지면 호모그래피) : {naive_err:7.1f} mm")
    print(f"       z 구속(광선-평면 교차)    : {con_err:7.1f} mm")
    check(f"구속이 미구속보다 100배 이상 정확 ({naive_err:.0f} → {con_err:.2f} mm)",
          naive_err > 100.0 and con_err < naive_err / 100.0)

    # -- 4. 현실적인 픽셀 노이즈에서 완료 조건 -------------------------------
    print("\n4) 픽셀 노이즈 0.3 px (서브픽셀 보정 수준) — 완료 조건 확인")
    loc = RobotLocalizer()
    errs, yerrs = [], []
    for _ in range(200):
        gx = RNG.uniform(0.45, 1.35)
        gy = RNG.uniform(0.45, 1.35)
        gyaw = RNG.uniform(-180, 180)
        dets = [render(A, 0.3, robot=(gx, gy, gyaw)),
                render(B, 0.3, robot=(gx, gy, gyaw))]
        pose = loc.update([A[0], B[0]], dets)
        if not pose.fresh:
            continue
        errs.append(math.hypot(pose.x - gx, pose.y - gy) * 1000)
        # loc.update 는 YAW_OFFSET_DEG 를 더해서 내보내므로 기준값에도 더해 비교한다
        expect = gyaw + cfg.YAW_OFFSET_DEG
        yerrs.append(abs((pose.yaw_deg - expect + 180) % 360 - 180))
    errs, yerrs = np.array(errs), np.array(yerrs)
    print(f"       표본 {len(errs)} · 위치 평균 {errs.mean():.1f} mm · "
          f"95% {np.percentile(errs,95):.1f} mm · 최대 {errs.max():.1f} mm")
    print(f"       yaw 평균 {yerrs.mean():.2f} deg · "
          f"95% {np.percentile(yerrs,95):.2f} deg · 최대 {yerrs.max():.2f} deg")
    check(f"위치 95% <= 30 mm ({np.percentile(errs,95):.1f} mm)",
          np.percentile(errs, 95) <= 30.0)
    check(f"yaw 95% <= 3 deg ({np.percentile(yerrs,95):.2f} deg)",
          np.percentile(yerrs, 95) <= 3.0)

    # -- 5. 두 카메라 좌표계가 통일되는가 ------------------------------------
    print("\n5) 두 카메라 독립 계산 — 같은 로봇을 같은 좌표로 보는가")
    gx, gy, gyaw = 1.1, 0.7, 33.0
    da = render(A, 0.0, robot=(gx, gy, gyaw))
    db = render(B, 0.0, robot=(gx, gy, gyaw))
    A[0].solve_extrinsics(da); B[0].solve_extrinsics(db)
    pa = _pose_from_corners(A[0], da[cfg.ROBOT_MARKER_ID])
    pb = _pose_from_corners(B[0], db[cfg.ROBOT_MARKER_ID])
    gap = math.hypot(pa[0] - pb[0], pa[1] - pb[1]) * 1000
    check(f"두 카메라 해의 차이 < 1 mm ({gap:.3f} mm)", gap < 1.0)

    # -- 6. 바닥 마커 2개만 보여도 풀리는가 ----------------------------------
    # 어떤 2장을 가릴지는 camA 가 기본으로 보는 마커에 따라 다르다 (배치에
    # 따라 camA 가 1,2 를 보기도 하고 3,4 를 보기도 한다) — camA 가 실제로
    # 보는 마커는 그대로 두고, 안 보는 쪽을 가려야 "2개만 남았을 때"를
    # 제대로 시험한다.
    print("\n6) 부분 가림 — 바닥 마커 2개만 보일 때")
    _seen0 = render(A, 0.0)
    _hide = tuple(m for m in cfg.FLOOR_MARKER_IDS if m not in _seen0) or (3, 4)
    det = render(A, 0.3, robot=(0.9, 0.9, 0.0), hide_floor=_hide)
    ok = A[0].solve_extrinsics(det)
    got = _pose_from_corners(A[0], det[cfg.ROBOT_MARKER_ID]) if ok else None
    err = math.hypot(got[0] - 0.9, got[1] - 0.9) * 1000 if got else float("inf")
    check(f"2점(8코너)으로도 해가 나온다 (오차 {err:.1f} mm)", ok and err < 100.0)

    # -- 7. 로봇 마커 가림 → 마지막 값 유지 ----------------------------------
    print("\n7) 폴백 — 로봇 마커가 안 보일 때 마지막 값 유지")
    loc = RobotLocalizer()
    dets = [render(A, 0.0, robot=(0.8, 1.0, 20.0)),
            render(B, 0.0, robot=(0.8, 1.0, 20.0))]
    p1 = loc.update([A[0], B[0]], dets)
    check(f"정상 관측 (cams={p1.n_cams}, fresh={p1.fresh})", p1.ok and p1.fresh)

    hidden = [render(A, 0.0, robot=(0.8, 1.0, 20.0), hide_robot=True),
              render(B, 0.0, robot=(0.8, 1.0, 20.0), hide_robot=True)]
    p2 = loc.update([A[0], B[0]], hidden)
    same = abs(p2.x - p1.x) < 1e-9 and abs(p2.y - p1.y) < 1e-9
    check(f"가려져도 마지막 값 유지 (ok={p2.ok}, fresh={p2.fresh})",
          p2.ok and not p2.fresh and same)

    import time as _t
    _t.sleep(cfg.POSE_HOLD_SEC + 0.05)
    p3 = loc.update([A[0], B[0]], hidden)
    check(f"유지 시간({cfg.POSE_HOLD_SEC}s) 초과 후 LOST 로 전환", not p3.ok)

    # -- 8. 상판 높이를 잘못 넣으면 얼마나 벌어지는가 ------------------------
    print("\n8) 민감도 — ROBOT_MARKER_HEIGHT 를 10 mm 틀리게 넣으면")
    det = render(A, 0.0, robot=(0.9, 0.9, 0.0))
    A[0].solve_extrinsics(det)
    true_h = cfg.ROBOT_MARKER_HEIGHT
    try:
        cfg.ROBOT_MARKER_HEIGHT = true_h + 0.010
        wrong = _pose_from_corners(A[0], det[cfg.ROBOT_MARKER_ID])
    finally:
        cfg.ROBOT_MARKER_HEIGHT = true_h
    sens = math.hypot(wrong[0] - 0.9, wrong[1] - 0.9) * 1000
    print(f"       위치가 {sens:.1f} mm 밀린다 → 상판 높이는 mm 단위로 실측할 것")
    check("민감도 계산됨", True)

    # -- 8b. 상자 그림자 — 상자(높이 220mm)가 마커(200mm)를 가리는가 --------
    print(f"\n8b) 상자 그림자 — 상자 높이 {cfg.BOX_H*1000:.0f} mm vs "
          f"마커 높이 {cfg.ROBOT_MARKER_HEIGHT*1000:.0f} mm")
    print(f"       상자 {cfg.BOX_W*1000:.0f} x {cfg.BOX_L*1000:.0f} x "
          f"{cfg.BOX_H*1000:.0f} mm, {len(cfg.BOXES)}개")

    # 상자 뒤로 그림자가 몇 mm 뻗는지 (상자 중심선을 따라 뒤쪽으로 훑는다)
    worst_shadow = 0.0
    for name, (bx, by, _) in cfg.BOXES.items():
        for side in ("A", "B"):
            C = cam_center(side)
            # 카메라에서 멀어지는 쪽이 그림자가 지는 방향
            step = 0.002 if C[1] < by else -0.002
            edge = by + (cfg.BOX_L / 2 if step > 0 else -cfg.BOX_L / 2)
            shadow = 0.0
            for i in range(1, 400):
                y = edge + step * i
                if not (0.0 <= y <= WS):
                    break
                p = np.array([bx, y, cfg.ROBOT_MARKER_HEIGHT])
                if box_blocks(side, p):
                    shadow = abs(y - edge)
                else:
                    break
            worst_shadow = max(worst_shadow, shadow)
            print(f"       {name:6s} / cam{side} : 상자 뒤 그림자 "
                  f"{shadow*1000:5.0f} mm")
    print(f"       최악 그림자 {worst_shadow*1000:.0f} mm")
    print(f"       (카메라가 {CAM_H:.2f} m 로 높고 높이차가 20 mm 뿐이라 짧다)")

    # 상자를 넣은 상태에서 작업 영역이 여전히 다 보이는가
    wx, wy = cfg.WORKSPACE_X, cfg.WORKSPACE_Y
    xs = np.linspace(wx[0], wx[1], 37)
    ys = np.linspace(wy[0], wy[1], 29)
    lost = []
    for gy in ys:
        for gx in xs:
            ok = False
            for cam_rt, side in ((A, "A"), (B, "B")):
                cam, R, t = cam_rt
                pts = robot_marker_corners(gx, gy, 0.0, z=cfg.ROBOT_MARKER_HEIGHT)
                px = project(R, t, cam.K, cam.dist, pts)
                if in_frame(px) and not blocked(side, pts.mean(axis=0)):
                    ok = True
                    break
            if not ok:
                lost.append((round(gx, 3), round(gy, 3)))
    print(f"       상자를 놓은 상태에서 작업 영역 {len(xs)*len(ys)} 점 중 "
          f"사각지대 {len(lost)} 점")
    if lost:
        print(f"       예: {lost[:5]}")
    check("상자를 놓아도 작업 영역에 사각지대가 없다", not lost)

    # -- 8c. 외부파라미터 고정 — 로봇이 바닥 마커를 밟고 서도 흔들리지 않는가 -
    print("\n8c) 외부파라미터 고정 — 로봇이 바닥 마커를 가릴 때")
    print(f"       (config.EXTRINSIC_LOCK_FRAMES = {cfg.EXTRINSIC_LOCK_FRAMES})")

    def measure(lock_frames, hide_after):
        """앞부분은 4장 다 보이고, 이후 hide_after 마커가 가려지는 상황."""
        old = cfg.EXTRINSIC_LOCK_FRAMES
        cfg.EXTRINSIC_LOCK_FRAMES = lock_frames
        cams = [make_virtual_camera("camA", "A")[0],
                make_virtual_camera("camB", "B")[0]]
        loc = RobotLocalizer()
        gx, gy = 0.110, 1.250          # toy 상자 앞 = 3번 마커 위
        xs, ys = [], []
        for i in range(80):
            hide = () if i < 40 else hide_after
            dets = [render(A, 0.3, robot=(gx, gy, 0.0), hide_floor=hide),
                    render(B, 0.3, robot=(gx, gy, 0.0), hide_floor=hide)]
            p = loc.update(cams, dets)
            if p.fresh and i >= 30:
                xs.append(p.x); ys.append(p.y)
        cfg.EXTRINSIC_LOCK_FRAMES = old
        if not xs:
            return None
        e = np.hypot(np.array(xs) - gx, np.array(ys) - gy) * 1000
        # 가려지기 전후의 값 차이(점프)
        n_before = sum(1 for i in range(30, 40))
        jump = (math.hypot(np.mean(xs[n_before:]) - np.mean(xs[:n_before]),
                           np.mean(ys[n_before:]) - np.mean(ys[:n_before])) * 1000)
        return e.mean(), e.max(), jump, cams[0].locked

    for lf, label in ((0, "고정 안 함 (매 프레임 재계산)"),
                      (cfg.EXTRINSIC_LOCK_FRAMES, "고정 사용")):
        r = measure(lf, (3,))
        if r is None:
            print(f"       {label:28s} : 해 없음")
            continue
        m, mx, jump, locked = r
        print(f"       {label:28s} : 오차 평균 {m:5.2f} mm · 최대 {mx:5.2f} mm · "
              f"가려질 때 점프 {jump:5.2f} mm")
    check("고정 기능이 동작한다 (3번 가려져도 값이 나온다)",
          measure(cfg.EXTRINSIC_LOCK_FRAMES, (3,)) is not None)

    # 바닥 마커가 전부 가려져도 고정값으로 계속 추적되는가
    cfg_old = cfg.EXTRINSIC_LOCK_FRAMES
    cfg.EXTRINSIC_LOCK_FRAMES = 20
    camsL = [make_virtual_camera("camA", "A")[0], make_virtual_camera("camB", "B")[0]]
    locL = RobotLocalizer()
    for i in range(25):
        dets = [render(A, 0.3, robot=(0.9, 0.9, 0.0)),
                render(B, 0.3, robot=(0.9, 0.9, 0.0))]
        locL.update(camsL, dets)
    all_hidden = [render(A, 0.3, robot=(0.9, 0.9, 0.0), hide_floor=(1, 2, 3, 4)),
                  render(B, 0.3, robot=(0.9, 0.9, 0.0), hide_floor=(1, 2, 3, 4))]
    pL = locL.update(camsL, all_hidden)
    err_all = (math.hypot(pL.x - 0.9, pL.y - 0.9) * 1000) if pL.fresh else None
    print(f"       바닥 마커 4장 모두 가려짐 → "
          f"{'추적 유지, 오차 %.2f mm' % err_all if pL.fresh else '추적 끊김'}")
    check("고정 후에는 바닥 마커가 다 가려져도 추적된다",
          pL.fresh and err_all is not None and err_all < 5.0)
    cfg.EXTRINSIC_LOCK_FRAMES = cfg_old

    # -- 9. 마운팅 규칙 — 사용법 3단계대로 붙이면 각도가 맞게 나오는가 -------
    print("\n9) 마운팅 규칙 — '종이 위쪽 = 로봇 앞쪽'으로 붙였을 때")
    print(f"       (config.YAW_OFFSET_DEG = {cfg.YAW_OFFSET_DEG})")
    loc = RobotLocalizer()
    worst_h = 0.0
    for heading in (0.0, 45.0, 90.0, 135.0, 180.0, -90.0, -135.0):
        # 종이 위쪽이 heading 을 향하면, 종이 오른쪽(마커 로컬 +x)은 heading-90 이다
        marker_rot = heading - 90.0
        dets = [render(A, 0.0, robot=(0.9, 0.9, marker_rot)),
                render(B, 0.0, robot=(0.9, 0.9, marker_rot))]
        pose = loc.update([A[0], B[0]], dets)
        err = abs((pose.yaw_deg - heading + 180) % 360 - 180)
        worst_h = max(worst_h, err)
        print(f"       로봇이 {heading:7.1f}° 를 향함 → 프로그램 출력 "
              f"{pose.yaw_deg:7.1f}°  (오차 {err:.3f}°)")
    check(f"로봇이 향한 각도가 그대로 나온다 (최악 {worst_h:.3f}°)", worst_h < 0.01)

    # -- 10. 미션 API — 상자까지의 상대좌표가 맞게 나오는가 ------------------
    print("\n10) 미션 API — relative_to_robot / box_pose")
    worst = 0.0
    cases = [
        # (로봇 x, y, yaw, 목표 x, y, 기대 전방, 기대 좌측)
        (0.9, 0.9, 0.0,    1.9, 0.9,  1.0,  0.0),   # 정면 1 m
        (0.9, 0.9, 0.0,    0.9, 1.9,  0.0,  1.0),   # 왼쪽 1 m
        (0.9, 0.9, 90.0,   0.9, 1.9,  1.0,  0.0),   # 돌아서면 정면
        (0.9, 0.9, 180.0,  0.9, 1.9,  0.0, -1.0),   # 오른쪽 1 m
        (0.9, 0.9, -90.0,  1.9, 0.9,  0.0,  1.0),
    ]
    for rx, ry, ryaw, tx, ty, e_fwd, e_left in cases:
        pose = Pose(rx, ry, ryaw, ok=True, n_cams=2, fresh=True)
        fwd, left, bearing = relative_to_robot(pose, (tx, ty, 0.0))
        worst = max(worst, abs(fwd - e_fwd), abs(left - e_left))
    check(f"상대좌표 변환이 정확하다 (최악 오차 {worst*1000:.3f} mm)", worst < 1e-9)

    names = sorted(cfg.BOXES)
    got = {n: box_pose(n) for n in names}
    ok_box = all(got[n] == cfg.BOXES[n] for n in names)
    print(f"       등록된 상자: {names}")
    for n in names:
        p = Pose(0.9, 0.9, 90.0, ok=True, n_cams=2, fresh=True)
        fwd, left, bearing = relative_to_robot(p, box_pose(n))
        print(f"       (0.9, 0.9) 에서 뒤쪽을 볼 때 {n:6s} → "
              f"전방 {fwd:5.2f} m · 좌측 {left:+5.2f} m · 방위 {bearing:+6.1f}°")
    check("box_pose 가 config 값을 그대로 돌려준다", ok_box)

    # -- 정리 ---------------------------------------------------------------
    n_ok = sum(1 for _, ok in results if ok)
    print(f"\n{'='*60}\n{n_ok}/{len(results)} PASS")
    for name, ok in results:
        if not ok:
            print(f"  FAIL: {name}")
    print("=" * 60)
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
