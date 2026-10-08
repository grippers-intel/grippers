"""실기 배치로 6기물 판을 시뮬레이션에서 돌려 회귀를 찾는다(카메라·로봇 없음, 2026-10-08).

    python tools/sim_regress.py              # 10-08 교육장 배치 7개 × 시드 2
    python tools/sim_regress.py rand 7 20    # 무작위 배치 20개(시드 7)

각 판: 끝까지 넣었는지 · HALTED · 보류 · 기물과 차체 최소 간격(1.5 cm 아래만, 0 아래 = 닿음) ·
운반(CARRY+NUDGE)마다 움직임 모양(R = 제자리 회전, F = 직진)과 돈 각도. 바구니 접근 "한 모양"은 RFR · RF · R 이다.
시뮬은 회전 뒤 직진 시작 때 7.5° 더 도는 실차 특성(base_trace)을 흉내 낸다 — 기물을 밀지는 않는다(간격만 잰다).
"""
import math, random, sys
from collections import defaultdict
sys.path[:0] = [str(__import__("pathlib").Path(__file__).resolve().parents[1])]
from host_config import load_host_config
from sim.sim_world import SimWorld
from tests.test_sim_end_to_end import FakeClock
from mission.host_fsm import MissionFSM, Order, HostState
from mission.trajectory_log import body_gap

cfg = load_host_config(None)
C = cfg.planner
LAYOUTS = {
    "3차(퀸 밖·상자 왼쪽)": [("box", .573, 1.202), ("knight", .144, .675), ("queen", 1.118, 1.334), ("rook", .774, .925), ("soccer", 1.142, .786), ("star", 1.516, 1.189)],
    "4차(별 왼쪽·상자 오른쪽 끝)": [("box", 1.652, 1.277), ("knight", .144, .674), ("queen", 1.118, 1.334), ("rook", .774, .926), ("soccer", 1.105, .758), ("star", .585, 1.262)],
    "5차(상자 퀸 사이)": [("box", .776, 1.106), ("knight", .145, .675), ("queen", 1.166, 1.355), ("soccer", 1.463, .797), ("star", .713, .807)],
    "6차": [("rook", .78, .86), ("queen", 1.18, .98), ("star", 1.45, 1.14), ("box", .53, 1.20), ("soccer", 1.46, .80), ("knight", .14, .67)],
    "8차": [("soccer", 1.00, .60), ("queen", 1.24, 1.14), ("rook", .60, 1.16), ("knight", .44, .83), ("box", 1.64, 1.23), ("star", 1.49, .88)],
    "9차": [("rook", 1.00, .77), ("soccer", .77, 1.16), ("box", .51, 1.10), ("queen", 1.59, 1.18), ("star", 1.63, .78), ("knight", .38, .67)],
    "11차": [("knight", .30, .90), ("rook", 1.03, 1.04), ("queen", .53, 1.16), ("star", .59, .73), ("box", 1.63, 1.13), ("soccer", 1.71, .80)],
}


def random_layout(rng):
    a = cfg.arena
    labels = ["queen", "knight", "rook", "soccer", "star", "box"]
    pts = []
    while len(pts) < 6:
        x = rng.uniform(a.workspace_x[0] + 0.05, a.workspace_x[1] - 0.05)
        y = rng.uniform(a.workspace_y[0] + 0.05, a.workspace_y[1] - 0.02)
        if math.hypot(x - 0.93, y - 0.33) < 0.35:
            continue
        if all(math.hypot(x - px, y - py) >= 0.30 for _, px, py in pts):
            pts.append((labels[len(pts)], x, y))
    return pts


def run(pieces, seed, max_s=600.0):
    world = SimWorld(cfg, pieces=pieces, start=(0.93, 0.33, 90.0), clock=FakeClock(), seed=seed,
                     pos_noise_m=0.004, yaw_noise_deg=0.6)
    fsm = MissionFSM(cfg)
    a = cfg.arena
    labels = tuple(sorted({l for l, x, y in pieces
                           if a.workspace_x[0] <= x <= a.workspace_x[1] and a.workspace_y[0] <= y <= a.workspace_y[1]}))
    fsm.set_order(Order(labels, "all", "organize", "sim"))
    clock = world.clock
    dt = 1.0 / cfg.mission.cycle_hz
    min_gap = defaultdict(lambda: 9.0)
    seen, halts, carries = set(), [], []
    cur = None
    while clock.t < max_s:
        clock.t += dt
        world.update()
        cmd = fsm.step(world.pose(), world.piece_map(), world.link.latest_status(), clock.t)
        world.link.send(cmd)
        for e in list(fsm.events)[-3:]:
            seen.add(e)
        st = fsm.state
        if st in (HostState.APPROACH_PIECE, HostState.CARRY_TO_DEST, HostState.NUDGE_BOX, HostState.SEARCH_TARGET):
            for p in world.pieces:
                if p.held or p.in_box or p.label == fsm.target_label and st == HostState.APPROACH_PIECE:
                    continue
                g = body_gap(world.x, world.y, world.yaw_deg, (p.x, p.y), C.robot_length_m, C.robot_width_m,
                             C.piece_obstacle_radius_m)
                min_gap[p.label] = min(min_gap[p.label], g)
        carrying = st in (HostState.CARRY_TO_DEST, HostState.NUDGE_BOX)
        if carrying and cur is None:
            cur = {"label": fsm.target_label, "rot": 0.0, "segs": [], "t0": clock.t}
        if cur is not None:
            k = "R" if cmd.angular_z else "F" if (cmd.linear_x or cmd.linear_y) else None
            if k and (not cur["segs"] or cur["segs"][-1] != k):
                cur["segs"].append(k)
            cur["rot"] += abs(math.degrees(cmd.angular_z)) * dt
            if not carrying:
                cur["t"] = clock.t - cur["t0"]
                carries.append(cur)
                cur = None
        if st == HostState.HALTED:
            halts.append(fsm.halt_reason)
            break
        if fsm.order is None:
            break
    done = len([p for p in world.pieces if p.in_box])
    return {"done": done, "n": len(labels), "t": clock.t, "halts": halts,
            "min_gap": dict(min_gap), "carries": carries,
            "notes": sorted(e for e in seen if any(k in e for k in ("skip", "HALT", "missed", "approach again", "FAILED", "폭주", "turn other way", "빠듯")))}


def show(name, r):
    contact = {k: round(v * 100, 1) for k, v in r["min_gap"].items() if v < 0.015}
    shapes = [c["label"] + ":" + "".join(c["segs"]) for c in r["carries"]]
    rot = [round(c["rot"]) for c in r["carries"]]
    print(f"{name}: {r['done']}/{r['n']} t={r['t']:.0f}s halts={r['halts']} close(cm)={contact}")
    print(f"    carries: {shapes}  rot: {rot}")
    for e in r["notes"]:
        print("    ", e[:150])


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "fixed"
    if which == "fixed":
        for name, lay in LAYOUTS.items():
            for seed in (1, 2):
                show(f"{name} s{seed}", run(lay, seed))
    else:
        rng = random.Random(int(sys.argv[2]) if len(sys.argv) > 2 else 7)
        n = int(sys.argv[3]) if len(sys.argv) > 3 else 20
        for i in range(n):
            lay = random_layout(rng)
            show(f"rand{i} {[(l, round(x, 2), round(y, 2)) for l, x, y in lay]}", run(lay, i))
