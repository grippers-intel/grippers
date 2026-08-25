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
