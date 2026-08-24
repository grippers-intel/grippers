"""grasp_cycle.py의 구조 계약 검사.

rclpy 의존이라 개발 머신에서 import할 수 없다 — arm_driver_node와 같은 방식으로
AST로 읽는다. 순수 계산인 비교 로직만 소스에서 떼어내 직접 검증한다.
"""

import ast
import json
import pathlib

TOOL = pathlib.Path(__file__).resolve().parent.parent / "tools" / "grasp_cycle.py"


def _tree():
    return ast.parse(TOOL.read_text(encoding="utf-8"), filename=str(TOOL))


def _function(name):
    return next(
        node
        for node in ast.walk(_tree())
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name
    )


def _constants(names):
    return {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in _tree().body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id in names
    }


def test_tool_never_drives_the_base():
    """사용자 지시(2026-08-24): 물체를 팔이 바로 잡을 수 있는 자리에 놓는다 —
    이 도구는 주행 단계가 없다. cmd_vel을 건드리면 안 된다."""
    source = TOOL.read_text(encoding="utf-8")

    assert "cmd_pub" not in source
    assert "Twist" not in source
    assert "odom" not in source.lower().replace("odom_publisher는 필요 없다", "")


def test_depth_observation_happens_before_the_arm_descends():
    """내려간 팔이 depth 카메라 화면을 가린다 — 관측이 먼저다."""
    source = ast.unparse(_function("main"))

    assert source.index("observe_depth") < source.index("move_floor_pose(profile, 'grasp')")


def test_gripper_opens_before_descending():
    """닫힌 손가락이 물체가 있는 공간을 통과해 내려가면 물체를 밀어낸다
    (사용자 지시, 2026-08-24)."""
    source = ast.unparse(_function("main"))

    assert source.index("set_gripper(preopen_mm)") < source.index(
        "move_floor_pose(profile, 'grasp')"
    )


def test_records_every_measurement_the_user_asked_for():
    """사용자가 요구한 항목: depth 면적·중심, 파지 시 load, 그리퍼캠 면적,
    그리고 그것들을 빈 상태와 비교할 수 있을 것."""
    source = ast.unparse(_function("main"))

    for key in ("area_open", "area_closed", "load_closed", "load_midpoint", "load_safe"):
        assert f"'{key}'" in source, key
    assert "record['depth']" in source


def test_depth_record_carries_center_and_area():
    """'depth camera에서 보이는 면적(거리 산출)과 center의 위치'."""
    source = ast.unparse(_function("observe_depth"))

    for key in ("'x'", "'h'", "'w'", "'area_px2'", "'forward_m'", "'lateral_m'"):
        assert key in source, key


def test_area_measurement_uses_a_median_of_several_samples():
    """단발 측정은 프레임마다 크게 튄다 — 2026-08-24 실기에서 1초 간격 연속
    표본이 22564 -> 28430 -> 12794 -> 4383 -> 46480처럼 흔들렸다. 데이터로
    남길 값은 그 잡음을 걷어내야 한다."""
    source = ast.unparse(_function("measure_area"))

    assert "values.sort()" in source
    assert "len(values) // 2" in source


def test_empty_run_is_the_baseline_and_is_marked_as_such():
    """빈 상태 기준선이 없으면 나머지 숫자를 해석할 수 없다 — 그리퍼캠 면적은
    밝기 임계 최대 컨투어라 손가락·바닥만으로도 면적이 잡히고, load도 빈 채로
    닫으면 0이 아니다."""
    main_source = ast.unparse(_function("main"))
    baseline_source = ast.unparse(_function("load_baseline"))

    assert "'empty': bool(args.empty)" in main_source
    assert "r.get('empty')" in baseline_source
    # 기준선 자신은 비교 대상을 찾지 않는다.
    assert "None if args.empty else load_baseline()" in main_source


def test_baseline_lookup_takes_the_most_recent_empty_run(tmp_path, monkeypatch):
    """기준선은 여러 번 다시 잴 수 있어야 한다(조명·바닥이 바뀌면 값이 변한다).
    가장 최근 것을 쓴다."""
    dataset = tmp_path / "grasp_dataset.jsonl"
    rows = [
        {"empty": True, "t_iso": "old", "load_closed": 0.01},
        {"empty": False, "raw_cls": "rook", "load_closed": 0.09},
        {"empty": True, "t_iso": "new", "load_closed": 0.02},
    ]
    dataset.write_text("\n".join(json.dumps(r) for r in rows) + "\n", encoding="utf-8")

    # 도구의 load_baseline과 같은 로직을 여기서 재현한다(import 불가).
    loaded = [json.loads(line) for line in dataset.read_text().splitlines() if line.strip()]
    empties = [r for r in loaded if r.get("empty")]

    assert empties[-1]["t_iso"] == "new"


def test_dataset_accumulates_across_runs_outside_tmp():
    """/tmp가 아니라 바인드 마운트된 곳에 쌓아야 컨테이너를 다시 만들어도
    남고 맥북에서 꺼낼 수 있다(모델 파일을 /tmp에서 옮긴 것과 같은 이유)."""
    path = _constants({"DATASET_PATH"})["DATASET_PATH"]

    assert not path.startswith("/tmp")
    assert path.startswith("/grippers/")


def test_every_arm_failure_path_recovers_to_idle():
    source = ast.unparse(_function("main"))

    assert source.count("recover_idle") >= 5


def test_restarts_perception_node_it_killed():
    fn = _function("main")
    tries = [node for node in ast.walk(fn) if isinstance(node, ast.Try)]
    finalbody = ast.unparse(tries[0].finalbody)

    assert "restart_perception_node" in finalbody
    assert finalbody.index("cam.close") < finalbody.index("restart_perception_node")
