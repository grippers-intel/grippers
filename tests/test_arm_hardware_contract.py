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
    # 위치 읽기는 재시도 헬퍼를 거친다 — 시리얼 패킷 유실 한 번으로 이동이
    # 한복판에서 끊기지 않게 하기 위해서다(2026-08-24 실기,
    # _read_joint_positions 주석 참고).
    assert "_read_joint_positions" in glide_names
    assert "get_position" in [
        _called_name(call) for call in _calls(_function("_read_joint_positions"))
    ]
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


def test_arrival_wait_extends_while_the_arm_is_still_making_progress():
    """2026-08-24 실기 회귀 — 느린 것과 걸린 것을 구분해야 한다.

    고정 4.0s 상한으로는 servo 2(어깨)가 매번 592 raw를 남기고 실패했는데,
    타임아웃 뒤에 보니 목표 +5 raw에 도착해 있었다 — 멈춘 게 아니라 느렸을
    뿐이었다. 어깨는 팔 전체를 중력에 맞서 들어올려 goal_speed를 올려도
    실측 153 raw/s가 한계였다(같은 거리의 servo 4는 230 raw/s). 잔차가
    줄고 있는 동안에는 계속 기다려야 한다."""
    source = ast.unparse(_function("_wait_floor_pose_arrived"))

    assert "FLOOR_POSE_STALL_SEC" in source
    assert "FLOOR_POSE_PROGRESS_RAW" in source
    assert "FLOOR_POSE_ARRIVE_MAX_SEC" in source


def test_arrival_wait_still_gives_up_on_a_genuinely_stuck_joint():
    """진전 기준으로 바꿨다고 무한정 매달리면 안 된다 — 최후의 한계선이
    실제로 정지마찰 대기 시간보다 길되 유한해야 한다."""
    limits = _module_constants(
        ARM_NODE, {"FLOOR_POSE_STALL_SEC", "FLOOR_POSE_ARRIVE_MAX_SEC", "FLOOR_POSE_PROGRESS_RAW"}
    )

    assert 0 < limits["FLOOR_POSE_STALL_SEC"] < limits["FLOOR_POSE_ARRIVE_MAX_SEC"]
    assert limits["FLOOR_POSE_ARRIVE_MAX_SEC"] < 60
    assert limits["FLOOR_POSE_PROGRESS_RAW"] > 0


def test_recover_idle_skips_the_start_pose_gate_that_normal_idle_enforces():
    """실패 복구 경로는 등록된 시작 자세 게이트를 건너뛴다(사용자 요청,
    2026-08-24). 이동이 실패하면 팔은 정의상 등록된 자세들 사이에 멈춰
    서는데, 그 상태가 "idle"의 게이트에 걸려 거부되기 때문이다 — 정작
    복구가 필요한 순간에만 복구가 막히는 모순이 생긴다."""
    source = ast.unparse(_function("_move_floor_stage"))

    assert "recover_idle" in source
    # 일반 idle 경로의 게이트는 그대로 살아 있어야 한다 — recover_idle은
    # 기본값이 아니라 예외다.
    assert "idle 복귀는 safe/drop 자세에서만 시작할 수 있습니다" in source


def test_recover_idle_is_an_accepted_stage():
    source = ast.unparse(_function("_execute_floor_pose"))

    assert "recover_idle" in source


def test_recover_idle_lifts_through_registered_waypoints_instead_of_sweeping():
    """⚠️ 2026-08-24 실기 사고 회귀 — 복구가 바닥을 긁으면 안 된다.

    첫 구현은 recover_idle에서 곧장 idle로 보간했다. 실제 실패는 팔이 바닥에
    내려간 grasp 자세에서 났고, 거기서 idle로 직선 보간하자 그리퍼가 바닥을
    긁으며 쓸려 갔다(사용자: "이렇게 움직이는건 절대로 안돼").

    팔이 어느 자세에 **도착하는가**만으로는 부족하고 **가는 경로 자체가**
    안전 요구사항이다 — 이 로봇의 작업 공간이 곧 바닥이기 때문이다."""
    source = ast.unparse(_function("_move_floor_stage"))
    recover = source[source.index("recover_idle"):]

    # grasp에서 시작하면 반드시 midpoint를 거쳐 올라가야 한다.
    assert "'grasp': (midpoint, safe, idle)" in recover
    assert "'midpoint': (safe, idle)" in recover
    # 등록된 자세 어디에도 안 붙으면 추측해서 움직이지 않는다.
    assert "RECOVER_MATCH_TOLERANCE_RAW" in recover


def test_recover_idle_refuses_rather_than_guessing_a_path():
    source = ast.unparse(_function("_move_floor_stage"))

    assert "안전한 복구 경로를 정할 수 없습니다" in source
    limits = _module_constants(
        ARM_NODE, {"RECOVER_MATCH_TOLERANCE_RAW", "FLOOR_POSE_START_TOLERANCE_RAW"}
    )
    # 복구 판정은 정상 게이트보다 넉넉해야 한다 — 복구가 필요한 상황은
    # 정의상 팔이 목표에 못 미친 상황이라 120으로는 아무 자세에도 안 붙는다.
    assert limits["RECOVER_MATCH_TOLERANCE_RAW"] > limits["FLOOR_POSE_START_TOLERANCE_RAW"]


def test_gripper_sets_its_own_speed_instead_of_inheriting_it():
    """2026-08-24 실기 회귀 — servo 6도 속도를 상속하면 안 된다.

    align_to_idle의 SPEED_RAW=150이 servo 6에 남아, 완전 개방(168mm)에서
    파지(15mm)까지의 약 820 raw 행정이 5.5s가 걸렸다 —
    GRIPPER_MOTION_TIMEOUT_SEC(4.0s)을 넘겨 "그리퍼 닫기 실패"로 끝났다."""
    names = [_called_name(call) for call in _calls(_function("_on_set_gripper"))]

    assert "set_speed" in names
    assert "set_acceleration" in names


def test_gripper_speed_finishes_full_travel_well_inside_the_motion_timeout():
    limits = _module_constants(
        ARM_NODE, {"GRIPPER_SPEED_RAW", "GRIPPER_MOTION_TIMEOUT_SEC"}
    )
    full_travel_raw = 850  # 168mm <-> 9mm, GRIPPER_MOTION_TIMEOUT_SEC 주석의 실측값

    travel_sec = full_travel_raw / limits["GRIPPER_SPEED_RAW"]
    assert travel_sec < limits["GRIPPER_MOTION_TIMEOUT_SEC"] / 2


def _lagged_ratio(ratio, lag):
    """arm_driver_node._lagged_ratio의 동작을 테스트에서 재현한다 — 이 모듈은
    rclpy 의존 때문에 개발 머신에서 import할 수 없어 AST로만 검사한다."""
    if lag <= 0.0:
        return ratio
    if ratio <= lag:
        return 0.0
    return (ratio - lag) / (1.0 - lag)


def test_joint_lag_never_changes_the_final_pose():
    """지연은 **경로만** 바꾸는 장치다 — 어떤 lag 값이든 보간이 끝나는
    순간에는 정확히 목표에 닿아야 한다. 이게 깨지면 자세 자체가 조용히
    틀어져 다음 단계의 시작 자세 게이트에서 떨어진다."""
    for lag in (0.0, 0.25, 0.45, 0.9):
        assert _lagged_ratio(1.0, lag) == 1.0


def test_joint_lag_holds_the_joint_still_for_the_first_part_of_the_move():
    lag = 0.45

    assert _lagged_ratio(0.1, lag) == 0.0
    assert _lagged_ratio(0.45, lag) == 0.0
    assert _lagged_ratio(0.45001, lag) > 0.0


def test_wrist_pitch_is_the_lagged_joint_not_wrist_roll():
    """2026-08-24 실기 — 룩을 문 채 복귀할 때 차체 전면을 긁었다.

    safe -> idle 구간에서 손목 피치(servo 4)는 1618 raw(142도)를 접지만
    손목 롤(servo 5)은 64 raw밖에 안 움직인다. 즉 늦춰야 하는 건 4다."""
    lag = _module_constants(ARM_NODE, {"FLOOR_POSE_JOINT_LAG"})["FLOOR_POSE_JOINT_LAG"]
    poses = _module_constants(
        ARM_NODE.with_name("floor_grasp_profiles.py"),
        {"IDLE_CRADLE_RAW", "HORIZONTAL_SAFE_145_RAW"},
    )
    travel = {
        servo_id: abs(poses["IDLE_CRADLE_RAW"][servo_id - 1] - poses["HORIZONTAL_SAFE_145_RAW"][servo_id - 1])
        for servo_id in range(1, 6)
    }

    assert 4 in lag and 0.0 < lag[4] < 1.0
    # 늦춘 관절은 실제로 크게 움직이는 관절이어야 한다 — 안 움직이는 관절을
    # 늦추는 건 아무것도 안 하는 것과 같다.
    assert travel[4] > 1000
    assert travel[5] < 100


def test_glide_applies_the_joint_lag():
    source = ast.unparse(_function("_glide_to_raw_positions"))

    assert "_lagged_ratio" in source
    assert "FLOOR_POSE_JOINT_LAG" in source
