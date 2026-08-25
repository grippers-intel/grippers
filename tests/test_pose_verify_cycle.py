"""pose_verify_cycle.py의 계약 검사.

두 겹으로 나눠 본다:

  - tools/pose_verify_expectations.py는 ROS 의존이 없어 **그대로 import해**
    실제로 계산을 돌려 본다. 기대 자세·잔차·판정이 이 도구의 본체다.
  - tools/pose_verify_cycle.py는 rclpy를 import해 개발 머신에서 실행할 수
    없으므로, 다른 실기 도구 테스트와 같은 방식으로 AST를 읽어 순서와 안전
    규칙만 검사한다.
"""

import ast
import importlib.util
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
GRIPPERS_ARM_SRC = ROOT / "ros2_ws" / "src" / "grippers_arm"
TOOL = ROOT / "tools" / "pose_verify_cycle.py"
EXPECTATIONS = ROOT / "tools" / "pose_verify_expectations.py"

if str(GRIPPERS_ARM_SRC) not in sys.path:
    sys.path.insert(0, str(GRIPPERS_ARM_SRC))


def _load_expectations():
    spec = importlib.util.spec_from_file_location("pose_verify_expectations", EXPECTATIONS)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


pv = _load_expectations()


def _tree(path=TOOL):
    return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def _function(name, path=TOOL):
    return next(
        node
        for node in ast.walk(_tree(path))
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name
    )


# --- 기대 자세 계산 -------------------------------------------------------


def test_deg_to_raw_matches_the_driver_formula_including_truncation():
    """driver_sdk.degrees_to_position은 round가 아니라 int(버림)다.

    round로 바꾸면 대부분 같지만 일부 관절이 1 raw 어긋난다 — 판정에는
    영향이 없어도, 잔차 표를 읽는 사람이 그 1을 계속 의심하게 된다."""
    from grippers_arm.floor_grasp_profiles import HORIZONTAL_CHESS_ROOK_45_DEG

    assert pv.deg_to_raw(93.87) == int(2048 + (93.87 / 360.0) * 4095) == 3115
    assert pv.deg_to_raw(-1.67) == 2029
    # 실측 자세 전체가 driver 공식과 정확히 일치한다.
    assert [pv.deg_to_raw(d) for d in HORIZONTAL_CHESS_ROOK_45_DEG] == [
        int(2048 + (d / 360.0) * 4095) for d in HORIZONTAL_CHESS_ROOK_45_DEG
    ]


def test_safe_grasp_and_midpoint_use_the_frozen_servo1_but_idle_and_drop_do_not():
    """arm_driver_node._move_floor_stage와 같은 계약 — servo1은 safe/grasp/
    midpoint 동안 얼려 두고, idle/drop만 등록 절대값을 쓴다. 도구가 이걸
    틀리면 좌우 정렬이 어긋난 회차에서 멀쩡한 자세를 실패로 보고한다."""
    from grippers_arm.floor_grasp_profiles import BASKET_DROP_195_RAW, IDLE_CRADLE_RAW

    poses = pv.expected_poses("chess_rook", frozen_servo1=1900)

    assert poses["safe"][0] == 1900
    assert poses["grasp"][0] == 1900
    assert poses["midpoint"][0] == 1900
    assert poses["idle"] == tuple(IDLE_CRADLE_RAW)
    assert poses["drop"] == tuple(BASKET_DROP_195_RAW)


def test_midpoint_is_the_per_joint_average_of_grasp_and_safe():
    poses = pv.expected_poses("chess_knight", frozen_servo1=2029)
    for i in range(5):
        assert poses["midpoint"][i] == round((poses["grasp"][i] + poses["safe"][i]) / 2.0)


def test_expected_safe_pose_matches_the_registered_measurement():
    from grippers_arm.floor_grasp_profiles import HORIZONTAL_SAFE_145_RAW

    poses = pv.expected_poses("cube", frozen_servo1=HORIZONTAL_SAFE_145_RAW[0])
    assert poses["safe"] == tuple(HORIZONTAL_SAFE_145_RAW)


def test_every_profile_has_an_expected_pose():
    from grippers_arm.floor_grasp_profiles import FLOOR_GRASP_PROFILES

    for profile in FLOOR_GRASP_PROFILES:
        poses = pv.expected_poses(profile, frozen_servo1=2048)
        assert set(poses) == {"idle", "safe", "grasp", "midpoint", "drop"}


# --- 잔차와 허용치 --------------------------------------------------------


def test_pose_tolerance_equals_the_drivers_start_gate():
    """이 도구가 통과시킨 자세는 정의상 다음 단계가 받아들이는 자세여야 한다."""
    source = (
        ROOT / "ros2_ws" / "src" / "grippers_arm" / "grippers_arm" / "arm_driver_node.py"
    ).read_text(encoding="utf-8")
    driver_gate = next(
        ast.literal_eval(node.value)
        for node in ast.parse(source).body
        if isinstance(node, ast.Assign)
        and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id == "FLOOR_POSE_START_TOLERANCE_RAW"
    )
    assert pv.POSE_TOLERANCE_RAW == driver_gate


def test_residuals_are_actual_minus_expected_and_pass_within_tolerance():
    expected = (2029, 2492, 2513, 1133, 3007)
    actual = [2029, 2492 + 119, 2513 - 119, 1133, 3007]
    residuals = pv.pose_residuals(expected, actual)
    assert residuals == [0, 119, -119, 0, 0]
    assert pv.pose_ok(residuals)
    assert not pv.pose_ok(pv.pose_residuals(expected, [2029, 2492 + 121, 2513, 1133, 3007]))


# --- 판정 ----------------------------------------------------------------


def test_load_verdict_compares_against_this_sessions_empty_baseline():
    """하드코딩된 상수가 아니라 같은 세션의 빈 회차 값과 비교한다 — 빈
    그리퍼의 부하는 배터리 전압과 서보 온도에 따라 움직인다."""
    assert pv.load_verdict(0.0821, 0.0352, 0.0078) is True
    assert pv.load_verdict(0.0391, 0.0352, 0.0078) is False
    # 기준선이 다르면 같은 값이 다르게 판정된다 — 그게 요점이다.
    assert pv.load_verdict(0.0391, 0.0250, 0.0078) is True


def test_load_verdict_is_undecided_when_either_reading_is_missing():
    assert pv.load_verdict(None, 0.0352, 0.0078) is None
    assert pv.load_verdict(0.0821, None, 0.0078) is None


# --- 2026-08-25 knight 회차 회귀 --------------------------------------------
#
# knight를 CARRY_IDLE로 접다가 놓쳤는데 load와 시각 두 신호가 **모두 성공**을
# 냈다. 이 도구의 존재 이유를 정면으로 깨는 오판이라, 그날의 실측값을 그대로
# 넣어 다시는 통과하지 못하게 못박는다.

# (회차, 파지 직후 폭, CARRY 폭, CARRY load) — 2026-08-25 실측
MEASURED_2026_08_25 = {
    "empty": (10.2, 10.0, 0.0235),
    "rook": (13.1, 12.9, 0.0782),
    "knight": (13.1, 10.0, 0.0313),   # ← 운반 중 놓침
    "queen": (11.6, 11.0, 0.0508),
    "soccer": (35.6, 35.6, 0.1017),
    "box": (32.0, 31.6, 0.1369),
    "star": (34.8, 34.4, 0.0978),
}
EMPTY_CARRY_LOAD_2026_08_25 = 0.0235


def _cycle_constant(name):
    tree = _tree()
    return next(
        ast.literal_eval(node.value)
        for node in tree.body
        if isinstance(node, ast.Assign)
        and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id == name
    )


def test_the_dropped_knight_is_caught_by_the_slip_signal():
    """물체가 빠지면 그것을 막던 것이 없어지므로 턱이 그만큼 더 닫힌다.
    knight는 13.1 -> 10.0mm(빈 그리퍼 폭)로 3.1mm 닫혔다."""
    closed, carry, _ = MEASURED_2026_08_25["knight"]

    assert pv.slip_verdict(closed, carry) is True


def test_the_slip_signal_separates_the_dropped_knight_from_every_held_object():
    """같은 회차의 나머지 실측이 전부 반대편에 있어야 임계가 의미를 갖는다."""
    slipped = {
        name: pv.slip_verdict(closed, carry)
        for name, (closed, carry, _) in MEASURED_2026_08_25.items()
    }

    assert slipped["knight"] is True
    assert [name for name, value in slipped.items() if value] == ["knight"]


def test_the_load_margin_no_longer_passes_the_dropped_knight():
    """⚠️ 회귀 — 놓친 knight의 load 여유가 정확히 2양자였고, 예전 임계가
    2양자여서 성공으로 통과했다. 임계는 그 위에 있어야 한다."""
    margin = _cycle_constant("LOAD_MARGIN")
    quantum = 4 / 1023

    assert margin > 2 * quantum

    verdicts = {
        name: pv.load_verdict(load, EMPTY_CARRY_LOAD_2026_08_25, margin)
        for name, (_, _, load) in MEASURED_2026_08_25.items()
        if name != "empty"
    }
    assert verdicts["knight"] is False
    assert all(value for name, value in verdicts.items() if name != "knight")


def test_the_load_margin_keeps_the_thinnest_real_grasp():
    """queen이 실측 중 가장 얇았다(7양자). 임계를 올리다 이걸 잘라내면
    안 된다 — 그러면 진짜 파지를 실패로 보고하게 된다."""
    margin = _cycle_constant("LOAD_MARGIN")
    quantum = 4 / 1023
    _, _, queen_load = MEASURED_2026_08_25["queen"]

    assert pv.load_verdict(queen_load, EMPTY_CARRY_LOAD_2026_08_25, margin) is True
    assert margin < queen_load - EMPTY_CARRY_LOAD_2026_08_25 - quantum


def test_the_no_drive_vision_rule_does_not_excuse_a_shrunken_bbox():
    """⚠️ 회귀 — 떨어진 knight가 46.4cm로 굴러가 h가 263.5 -> 61.0px로 줄었다.
    비율 규칙은 그걸 "멀리 있는 다른 개체"로 보고 성공이라고 했다. 주행이
    없는 회차에서는 보이면 그냥 바닥에 있는 것이다."""
    assert pv.vision_verdict(263.5, True, 61.0, 0.8) is True  # 예전 규칙의 오판
    assert pv.vision_verdict_no_drive(263.5, found=True) is False


def test_the_no_drive_vision_rule_still_reports_a_real_disappearance():
    assert pv.vision_verdict_no_drive(268.4, found=False) is True
    assert pv.vision_verdict_no_drive(None, found=False) is None
    assert pv.vision_verdict_no_drive(268.4, found=None) is None


def test_the_cycle_uses_the_no_drive_vision_rule_not_the_ratio_one():
    source = ast.unparse(_function("run_cycle"))

    assert "vision_verdict_no_drive" in source
    assert "STILL_THERE_H_RATIO" not in source


def test_a_slip_makes_the_combined_verdict_a_failure():
    """slip은 극성이 반대다(True=놓침). 그 극성을 놓치면 놓친 회차가
    "세 신호 모두 성공"으로 찍힌다 — 바로 그 사고가 났었다."""
    source = ast.unparse(_function("print_verdicts"))

    assert "if verdicts['slip'] is True:" in source
    assert "if verdicts['load'] is False:" in source
    assert "if verdicts['vision'] is False:" in source
    assert source.index("failures") < source.index("파지 실패")


def test_vision_verdict_says_gone_only_when_the_object_is_absent_or_much_smaller():
    assert pv.vision_verdict(120.0, found=False, h_after=None, ratio=0.8) is True
    # 문턱은 h_before * ratio = 96.0px다.
    assert pv.vision_verdict(120.0, found=True, h_after=110.0, ratio=0.8) is False
    assert pv.vision_verdict(120.0, found=True, h_after=96.0, ratio=0.8) is False
    assert pv.vision_verdict(120.0, found=True, h_after=95.9, ratio=0.8) is True


def test_vision_verdict_is_undecided_without_a_baseline_observation():
    assert pv.vision_verdict(None, found=True, h_after=100.0, ratio=0.8) is None
    assert pv.vision_verdict(120.0, found=None, h_after=None, ratio=0.8) is None


# --- 회차 순서와 안전 규칙 -------------------------------------------------


def test_the_checkpoints_follow_the_mission_order():
    names = [name for name, _, _ in pv.CYCLE_CHECKPOINTS]
    assert names == [
        "idle_start", "safe_down", "preopen", "grasp", "closed",
        "midpoint_up", "safe_up", "carry_idle", "drop", "released",
        "closed_to_fold", "idle_end",
    ]


def test_the_gripper_opens_before_the_arm_descends():
    """확립된 안전 규칙 — 닫힌 손가락이 물체 자리를 통과해 내려가면 안 된다."""
    source = ast.unparse(_function("run_cycle"))
    assert source.index("set_gripper(spec.preopen_width_mm)") < source.index(
        "move_floor_pose(profile, 'grasp')"
    )


def test_the_arm_lifts_through_the_verified_chain_and_never_straight_to_idle():
    """바닥에서 IDLE로 곧장 가면 그리퍼가 바닥을 쓸어간다."""
    source = ast.unparse(_function("run_cycle"))
    chain = source.index(
        "(('midpoint', 'midpoint_up'), ('safe', 'safe_up'), ('idle', 'carry_idle'))"
    )
    assert source.index("set_gripper(spec.close_width_mm)") < chain


def test_the_gripper_closes_before_folding_back_to_idle():
    """사용자 지시(2026-08-25) — 투하 후 닫고 나서 IDLE로 접는다."""
    source = ast.unparse(_function("run_cycle"))
    release = source.index("set_gripper(spec.release_width_mm)")
    close = source.index("set_gripper(GRIPPER_CLOSED_MM)")
    fold = source.index("move_floor_pose(profile, 'idle')")
    assert release < close < fold


def test_the_release_width_is_not_the_full_opening():
    """활짝 여는 대신 물체 폭 + 여유만 연다 — 손가락 판이 바구니 위로
    쓸리지 않게(사용자 지시, 2026-08-25)."""
    source = ast.unparse(_function("run_cycle"))
    assert "set_gripper(spec.preopen_width_mm)" in source
    # 투하 단계에서는 preopen이 아니라 release를 쓴다.
    release_call = source.index("set_gripper(spec.release_width_mm)")
    assert source.index("'drop'") < release_call


def test_the_tool_never_drives():
    """사용자 지시(2026-08-25): "이동은 없음". cmd_vel을 건드리는 흔적이
    하나도 없어야 한다 — drive_phase를 import만 해 둬도 다음 사람이 쓴다."""
    # docstring은 "주행하지 않는다"고 **설명**하므로 원문 검색은 자기 자신에
    # 걸린다. 코드만 본다 — ast.unparse는 docstring도 문자열 리터럴로 되살리므로
    # 모듈 docstring을 먼저 떼어 낸다.
    tree = _tree()
    if (
        tree.body
        and isinstance(tree.body[0], ast.Expr)
        and isinstance(tree.body[0].value, ast.Constant)
    ):
        tree.body = tree.body[1:]
    source = ast.unparse(tree)
    for forbidden in ("cmd_vel", "drive_phase", "Twist", "APPROACH_SPEED", "TURN_IN_PLACE"):
        assert forbidden not in source, f"주행 흔적: {forbidden}"


def test_the_empty_baseline_runs_before_the_object_cycles():
    """빈 회차가 그 세션의 기준선이므로 반드시 먼저 돌아야 한다."""
    source = ast.unparse(_function("main"))
    assert source.index("empty=True") < source.index("empty=False")


def test_object_cycles_receive_the_baseline_they_are_compared_against():
    source = ast.unparse(_function("main"))
    assert "baseline=baseline" in source


def test_the_gripper_width_error_is_reported_not_failed():
    """servo 6은 토크 제한 레지스터가 없어 위치 오차가 곧 파지력이다 —
    물체를 문 상태에서 명령 폭에 도달하지 못하는 것이 정상이다."""
    source = ast.unparse(_function("report_checkpoint"))
    assert "파지력 대리값" in source
    # 폭 오차로 pose_ok를 뒤집지 않는다.
    assert "width_error" in source
    assert source.index("ok = pose_ok(residuals)") < source.index("width_error")


# --- numpy 직렬화 회귀 (2026-08-25 첫 실행이 여기서 끊겼다) ----------------


def _exec_isolated(path, name):
    """rclpy를 import하는 파일에서 함수 하나만 떼어 실행한다.

    모듈 전체는 개발 머신에서 import할 수 없지만, 순수 함수는 소스만 있으면
    그대로 돌려 볼 수 있다 — AST 검사보다 실제 동작을 본다."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    fn = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )
    namespace = {}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), "exec"), namespace)
    return namespace[name]


def test_run_log_can_serialise_the_numpy_types_ros_messages_carry():
    """⚠️ 회귀 — GetArmState의 고정 길이 배열은 numpy dtype이라 list()로
    감싸도 원소가 numpy 스칼라로 남고, json.dumps가 "Object of type int32 is
    not JSON serializable"로 죽는다. 2026-08-25 pose_verify_cycle 첫 실행이
    정확히 첫 체크포인트에서 이렇게 끊겼다."""
    import json

    import numpy as np

    default = _exec_isolated(ROOT / "tools" / "grasp_test_console.py", "_json_default")

    payload = {
        "position_raw": [np.int32(2064), np.int32(834)],
        "load_ratio": [np.float32(0.0195), np.float32(0.0)],
        "array": np.array([1, 2, 3], dtype=np.int32),
        "scalar_array": np.array([7], dtype=np.int32),
        "plain": [1, 2.5, True, None, "ok"],
    }
    decoded = json.loads(json.dumps(payload, ensure_ascii=False, default=default))

    assert decoded["position_raw"] == [2064, 834]
    assert decoded["load_ratio"][1] == 0.0
    assert decoded["array"] == [1, 2, 3]
    assert decoded["scalar_array"] == 7
    assert decoded["plain"] == [1, 2.5, True, None, "ok"]


def test_run_log_default_still_refuses_genuinely_unserialisable_values():
    """모르는 타입을 조용히 삼키면 로그에 쓰레기가 남는다."""
    import pytest

    default = _exec_isolated(ROOT / "tools" / "grasp_test_console.py", "_json_default")

    with pytest.raises(TypeError):
        default(object())


def test_arm_snapshot_converts_every_field_to_plain_python_types():
    """읽자마자 한 번 변환해 두면 아래로 흐르는 코드가 numpy를 만나지 않는다 —
    json뿐 아니라 잔차 산술 결과까지 numpy로 전파되는 것을 막는다."""
    import numpy as np

    tree = _tree()
    cls = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "ArmSnapshot"
    )
    namespace = {}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(TOOL), "exec"), namespace)

    class FakeResponse:
        position_raw = np.array([2064, 834, 3095, 2751, 3070, 1155], dtype=np.int32)
        load_ratio = np.array([0.0195, 0.0313, 0.0, 0.0, 0.0, 0.0235], dtype=np.float32)
        temperature_c = np.array([34, 37, 34, 34, 34, 38], dtype=np.int32)
        torque_on = [np.True_] * 6

    snapshot = namespace["ArmSnapshot"](FakeResponse())

    assert all(type(v) is int for v in snapshot.position_raw)
    assert all(type(v) is float for v in snapshot.load_ratio)
    assert all(type(v) is int for v in snapshot.temperature_c)
    assert all(type(v) is bool for v in snapshot.torque_on)
    assert snapshot.position_raw[0] == 2064
    # 잔차 산술도 기본형으로 남는다.
    assert type(pv.pose_residuals((2029,) * 5, snapshot.position_raw[:5])[0]) is int


def test_read_state_returns_a_snapshot_not_the_raw_response():
    source = ast.unparse(_function("read_state"))
    assert "ArmSnapshot(response)" in source


# --- 클래스 순서 (2026-08-25) ----------------------------------------------


def test_the_default_sweep_does_not_start_with_the_unreliable_box_class():
    """box(큐브)는 검출이 가장 불안정한 클래스다(사용자 확인, 2026-08-25).
    알파벳 순으로 두면 맨 앞에 와서, 첫 회차 실패가 도구 고장처럼 보인다."""
    order = _exec_isolated_names(TOOL, ["DEFAULT_CLASS_ORDER"])["DEFAULT_CLASS_ORDER"]

    assert order[0] == "rook"
    assert order.index("box") > order.index("rook")
    assert order.index("star") > order.index("rook")


def test_the_default_order_never_silently_drops_a_class():
    """순서는 손으로 관리하지만 누락은 막는다 — 새 클래스가 생겨도 뒤에 붙는다."""
    namespace = _exec_isolated_names(
        TOOL, ["DEFAULT_CLASS_ORDER", "default_class_order"]
    )
    namespace["CLASS_TO_PROFILE"] = {
        "rook": "chess_rook", "box": "cube", "newthing": "whatever",
    }
    ordered = namespace["default_class_order"]()

    assert set(ordered) == {"rook", "box", "newthing"}
    assert ordered[0] == "rook"
    assert ordered.index("newthing") > ordered.index("box")


def _exec_isolated_names(path, names):
    """모듈에서 이름 몇 개만 떼어 한 네임스페이스에 실행한다.

    rclpy를 import하는 파일이라 모듈 전체는 개발 머신에서 못 올린다.
    돌려주는 네임스페이스는 그대로 쓰기 가능해서, 테스트가 의존 이름을
    (예: CLASS_TO_PROFILE) 대신 꽂아 넣을 수 있다."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    wanted = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in names:
            wanted.append(node)
        elif (
            isinstance(node, ast.Assign)
            and isinstance(node.targets[0], ast.Name)
            and node.targets[0].id in names
        ):
            wanted.append(node)
    found = {
        node.name if isinstance(node, ast.FunctionDef) else node.targets[0].id
        for node in wanted
    }
    assert found == set(names), f"찾지 못한 이름: {set(names) - found}"
    namespace = {}
    exec(compile(ast.Module(body=wanted, type_ignores=[]), str(path), "exec"), namespace)
    return namespace


# --- 오염된 빈 기준선 (2026-08-25) ------------------------------------------


def test_a_contaminated_empty_cycle_is_detected_by_gripper_width():
    """⚠️ 회귀 — 물체 여섯 개를 팔 앞에 늘어놓은 채 빈 회차를 돌렸더니 그중
    하나를 집어 올렸고, 그 오염된 기준선(CARRY load 0.0821)으로 이후 회차를
    전부 비교했다. 진짜 파지를 실패로 보고하게 만드는, 조용히 틀리는 사고다."""
    assert pv.empty_cycle_is_contaminated(4.4) is True    # 그날 물었던 회차
    assert pv.empty_cycle_is_contaminated(1.2) is False   # 진짜 빈 회차 실측 최대
    assert pv.empty_cycle_is_contaminated(0.7) is False
    assert pv.empty_cycle_is_contaminated(None) is None


def test_the_contamination_threshold_sits_above_every_measured_empty_close():
    """빈 회차 실측 폭 오차는 +0.5~+1.2mm였다. 임계는 그 위, 오염(+4.4) 아래."""
    assert 1.2 < pv.EMPTY_CLOSE_WIDTH_ERROR_MM < 4.4


def test_the_run_refuses_to_use_a_contaminated_baseline():
    source = ast.unparse(_function("main"))

    assert "_contaminated" in source
    assert source.index("_contaminated") < source.index("empty=False")


def test_the_contaminated_cycle_still_finishes_instead_of_stranding_the_arm():
    """물고 있는 채로 중단하면 팔이 바닥 높이에 물체를 든 채 남는다 —
    회차는 끝까지 돌려 바구니에 놓고 IDLE로 복귀해야 한다."""
    source = ast.unparse(_function("run_cycle"))

    detect = source.index("empty_cycle_is_contaminated")
    drop = source.index("move_floor_pose(profile, 'drop')")
    fold = source.index("move_floor_pose(profile, 'idle')")
    assert detect < drop < fold
    # 감지 지점에서 회차를 끊지 않는다.
    tail = source[detect:drop]
    assert "return results" not in tail


def test_the_operator_is_told_the_placement_distance_not_just_the_place():
    """사용자 지시(2026-08-25): GRASP는 물체 중심이 차체 전면 19cm 앞에
    있다고 전제한다. 프롬프트가 그 숫자를 말해야 전제가 실제로 지켜진다."""
    source = ast.unparse(_function("run_cycle"))

    assert "GRASP_OBJECT_CENTER_FORWARD_MM" in source
    assert "빈 회차입니다" in source  # 빈 회차는 반대로 '비워 두라'고 말한다


def test_the_placement_premise_comes_from_the_profiles_not_a_local_copy():
    """수치를 도구에 베껴 적으면 프로파일이 바뀔 때 조용히 어긋난다."""
    source = TOOL.read_text(encoding="utf-8")

    assert "GRASP_OBJECT_CENTER_FORWARD_MM," in source
    assert "190" not in ast.unparse(_function("run_cycle"))
