"""주행 궤적 CSV — 차체 간격 계산과 한 줄 기록."""
import csv

from localization.pose import Pose
from mission.host_fsm import MissionFSM, Order
from mission.trajectory_log import TrajectoryLog, body_gap
from vla_common.protocol import HostCommand, State


def test_body_gap_beside_and_ahead():
    # 차체 20 x 25 cm, 마커 중심. 오른쪽 옆 14 cm 기물(반경 2 cm) -> 옆면에서 2 cm
    assert abs(body_gap(0, 0, 90.0, (0.14, 0.0), 0.25, 0.20, 0.02) - 0.02) < 1e-9
    assert abs(body_gap(0, 0, 90.0, (0.0, 0.20), 0.25, 0.20, 0.02) - 0.055) < 1e-9   # 앞
    assert body_gap(0, 0, 90.0, (0.11, 0.0), 0.25, 0.20, 0.02) < 0                   # 겹침


def test_writes_one_row_per_cycle(tmp_path):
    from host_config import load_host_config
    cfg = load_host_config(None)
    fsm = MissionFSM(cfg)
    fsm.set_order(Order(labels=("queen",)))                                        # 목표는 퀸 — 상자는 옆 기물
    log = TrajectoryLog(tmp_path / "t.csv", cfg.planner)
    pose = Pose(1.25, 0.90, 90.0, ok=True, n_cams=2, fresh=True)
    pmap = {"box": [(1.143, 0.895)], "queen": [(0.40, 1.20)]}
    fsm.step(pose, pmap, None, 0.0)
    log.row(0.0, fsm, pose, HostCommand(State.IDLE, stop=True), pmap)
    log.close()
    rows = list(csv.DictReader(open(tmp_path / "t.csv", encoding="utf-8")))
    assert len(rows) == 1 and rows[0]["state"]
    assert rows[0]["nearest"] == "box" and abs(float(rows[0]["nearest_center_m"]) - 0.107) < 0.001
    assert float(rows[0]["nearest_gap_m"]) < 0.0                                  # 10.7 cm 면 닿는다
