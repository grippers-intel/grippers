"""arm_driver_node의 실물 하드웨어 실패 계약 정적 검사 (#156).

arm_driver_node는 rclpy·grippers_interfaces와 실제 SO-ARM101 의존성이 있어
일반 개발 머신에서 직접 import하기 어렵다. 따라서 소스를 AST로 읽어 기동 및
모션 경계의 핵심 안전 계약이 사라지지 않는지 검사한다.

실제 USB 단선·torque OFF 동작은 Pi + SO-ARM101 실기 검증 대상이다.
"""

import ast
import importlib.util
import pathlib

ARM_NODE = (
    pathlib.Path(__file__).resolve().parent.parent
    / "ros2_ws"
    / "src"
    / "grippers_arm"
    / "grippers_arm"
    / "arm_driver_node.py"
)
DOMAIN_STATES = pathlib.Path(__file__).resolve().parent.parent / "domain" / "task" / "states.py"
GRIPPER_CALIBRATION = ARM_NODE.with_name("gripper_calibration.py")


def _parse():
    return ast.parse(
        ARM_NODE.read_text(encoding="utf-8"),
        filename=str(ARM_NODE),
    )


def _function(name):
    return next(
        node
        for node in ast.walk(_parse())
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name
    )


def _module_constants(path, names):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    return {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in tree.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id in names
    }


def _load_gripper_calibration():
    spec = importlib.util.spec_from_file_location("gripper_calibration", GRIPPER_CALIBRATION)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _calls(node):
    return [n for n in ast.walk(node) if isinstance(n, ast.Call)]


def _called_name(call):
    func = call.func
    if isinstance(func, ast.Attribute):
        return func.attr
    if isinstance(func, ast.Name):
        return func.id
    return None


def test_torque_auto_enable_is_opt_in():
    init = _function("__init__")

    declarations = [
        call
        for call in _calls(init)
        if _called_name(call) == "declare_parameter"
        and call.args
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == "enable_torque_on_start"
    ]

    assert len(declarations) == 1
    call = declarations[0]
    assert len(call.args) >= 2
    assert isinstance(call.args[1], ast.Constant)
    assert call.args[1].value is False


def test_startup_checks_serial_connection_and_torque():
    init = _function("__init__")
    names = [_called_name(call) for call in _calls(init)]

    assert "is_connected" in names
    assert "_check_startup_torque" in names


def test_startup_torque_checks_each_servo():
    fn = _function("_check_startup_torque")
    names = [_called_name(call) for call in _calls(fn)]

    assert "get_torque" in names
    assert "set_torque" in names
    assert "set_all_torque" not in names


def test_move_checks_servos_before_and_after_motion():
    fn = _function("_execute_move")
    names = [_called_name(call) for call in _calls(fn)]

    assert names.count("_require_operational_servos") >= 2
    assert "go" in names


def test_horizontal_floor_pose_uses_checked_interpolated_joint_writes():
    execute = _function("_execute_floor_pose")
    move_stage = _function("_move_floor_stage")
    glide = _function("_glide_to_raw_positions")
    execute_names = [_called_name(call) for call in _calls(execute)]
    move_stage_names = [_called_name(call) for call in _calls(move_stage)]
    glide_names = [_called_name(call) for call in _calls(glide)]

    assert execute_names.count("_require_operational_servos") >= 2
    assert "_move_floor_stage" in execute_names
    assert "get_temperature" in execute_names
    assert "_near_pose" in move_stage_names
    assert "_glide_to_raw_positions" in move_stage_names
    assert "get_position" in glide_names
    assert "set_position" in glide_names


def test_horizontal_idle_safe_transition_does_not_use_vertical_waypoints():
    source = ast.unparse(_function("_move_floor_stage"))

    assert "VERTICAL_SAFE_OVERHEAD" not in source
    assert "HORIZONTAL_OVERHEAD" not in source
    assert "self._glide_to_raw_positions(backend, idle)" in source
    assert "self._glide_to_raw_positions(backend, safe)" in source


def test_floor_stage_freezes_servo1_for_safe_and_grasp_but_not_idle_or_drop():
    """2026-08-24 사용자 지시: APPROACH가 이미 물체 정면으로 맞춘 servo1을
    safe/grasp/midpoint 전환 중엔 절대 건드리지 않는다. idle(=CARRY_IDLE로
    복귀)과 drop은 등록된 절대 servo1 값을 그대로 써야 하므로 freeze하지
    않는다."""
    source = ast.unparse(_function("_move_floor_stage"))

    assert "frozen_servo1" in source
    assert "_freeze_servo1(self._tuple_goals(HORIZONTAL_SAFE_145_RAW))" in source
    assert "_freeze_servo1(self._raw_goals(backend, HORIZONTAL_GRASP_POSES_DEG[profile]))" in source
    assert "idle = self._tuple_goals(IDLE_CRADLE_RAW)" in source
    assert "drop = self._tuple_goals(BASKET_DROP_195_RAW)" in source


def test_fold_to_cradle_checks_servos_before_and_after_motion():
    fn = _function("_on_fold_to_cradle")
    names = [_called_name(call) for call in _calls(fn)]

    assert names.count("_require_operational_servos") >= 2
    assert "go" in names


def test_gripper_checks_servo_and_position_write_result():
    fn = _function("_on_set_gripper")
    names = [_called_name(call) for call in _calls(fn)]

    assert "_require_operational_servos" in names
    assert "set_position" in names


def test_gripper_calibration_matches_measured_safe_contract():
    calibration = _load_gripper_calibration()
    domain = _module_constants(DOMAIN_STATES, {"CLOSED_MM", "OPEN_MM"})

    assert calibration.GRIPPER_CALIBRATION_POINTS == (
        (9.0, 1150),
        (96.0, 1578),
        (168.0, 2000),
    )
    assert domain == {"CLOSED_MM": 9.0, "OPEN_MM": 168.0}


def test_gripper_calibration_interpolates_and_clamps():
    calibration = _load_gripper_calibration()

    assert calibration.position_from_width(9.0) == 1150
    assert calibration.position_from_width(90.0) == 1548
    assert calibration.position_from_width(96.0) == 1578
    assert calibration.position_from_width(168.0) == 2000
    assert calibration.position_from_width(-1.0) == 1150
    assert calibration.position_from_width(999.0) == 2000


def test_gripper_uses_piecewise_calibration_not_third_party_defaults():
    fn = _function("_on_set_gripper")
    position_call = next(call for call in _calls(fn) if _called_name(call) == "position_from_width")

    assert len(position_call.args) == 1


def test_hold_position_does_not_use_lossy_bulk_torque_helper():
    fn = _function("_on_hold_position")
    names = [_called_name(call) for call in _calls(fn)]

    assert "set_torque" in names
    assert "set_all_torque" not in names


def test_load_read_failure_is_logged():
    fn = _function("_read_load")
    names = [_called_name(call) for call in _calls(fn)]

    assert "get_load" in names
    assert "warn" in names


def test_startup_logs_idle_offset_but_never_moves_a_servo():
    init = _function("__init__")
    names = [_called_name(call) for call in _calls(init)]

    assert "_log_idle_offset" in names
    assert names.index("_check_startup_torque") < names.index("_log_idle_offset")


def test_idle_offset_logging_reads_position_and_never_writes_it():
    fn = _function("_log_idle_offset")
    names = [_called_name(call) for call in _calls(fn)]

    assert "get_position" in names
    assert "set_position" not in names
    assert "set_torque" not in names
    assert {"info", "warn", "error"} & set(names)


def test_idle_offset_thresholds_match_documented_warn_and_error_levels():
    constants = _module_constants(ARM_NODE, {"IDLE_OFFSET_WARN_RAW", "IDLE_OFFSET_ERROR_RAW"})

    assert constants == {"IDLE_OFFSET_WARN_RAW": 120, "IDLE_OFFSET_ERROR_RAW": 800}


def test_startup_hardware_failure_is_caught_by_main():
    main = _function("main")

    caught_names = set()
    for handler in [node for node in ast.walk(main) if isinstance(node, ast.ExceptHandler)]:
        typ = handler.type
        if isinstance(typ, ast.Name):
            caught_names.add(typ.id)
        elif isinstance(typ, ast.Tuple):
            caught_names.update(elt.id for elt in typ.elts if isinstance(elt, ast.Name))

    assert "ArmHardwareUnavailableError" in caught_names


def test_glide_sets_servo_speed_instead_of_inheriting_it():
    """2026-08-24 실기 회귀 — 서보 속도를 상속하면 안 된다.

    STS3215의 goal_speed는 레지스터에 남는 상태값이라, 이 노드가 안 쓰면
    마지막으로 쓴 쪽의 값이 그대로 적용된다. 실제로 tools/align_to_idle.py의
    느린 SPEED_RAW=150이 남아 IDLE->safe 이동(servo 2가 1663 raw)이 글라이드
    시간 안에 끝나지 못했고(실측 153 raw/s), safe 단계가 통째로 실패했다."""
    glide_names = [_called_name(call) for call in _calls(_function("_glide_to_raw_positions"))]

    assert "set_speed" in glide_names
    assert "set_acceleration" in glide_names


def test_glide_speed_can_finish_the_longest_registered_move_in_time():
    """속도 상한이 보간이 요구하는 속도를 막지 않아야 한다.

    상한이 병목이 되면 waypoint를 다 써 넣어도 팔이 못 따라와 다음 단계의
    시작 자세 게이트에서 떨어진다 — 위 회귀의 실패 방식 그 자체다. 실측
    단위는 대략 raw/s다(레지스터 150에서 153 raw/s)."""
    timing = _module_constants(
        ARM_NODE, {"FLOOR_POSE_STEPS", "FLOOR_POSE_STEP_SEC", "FLOOR_POSE_SPEED_RAW"}
    )
    poses = _module_constants(
        ARM_NODE.with_name("floor_grasp_profiles.py"),
        {"IDLE_CRADLE_RAW", "HORIZONTAL_SAFE_145_RAW"},
    )
    longest_raw = max(
        abs(a - b) for a, b in zip(poses["IDLE_CRADLE_RAW"], poses["HORIZONTAL_SAFE_145_RAW"])
    )
    glide_sec = timing["FLOOR_POSE_STEPS"] * timing["FLOOR_POSE_STEP_SEC"]

    assert timing["FLOOR_POSE_SPEED_RAW"] >= longest_raw / glide_sec
