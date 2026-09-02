"""arm_driver_node의 실물 하드웨어 실패 계약 정적 검사 (#156).

arm_driver_node는 rclpy·grippers_interfaces와 실제 SO-ARM101 의존성이 있어
일반 개발 머신에서 직접 import하기 어렵다. 따라서 소스를 AST로 읽어 기동 및
모션 경계의 핵심 안전 계약이 사라지지 않는지 검사한다.

실제 USB 단선·torque OFF 동작은 Pi + SO-ARM101 실기 검증 대상이다.
"""

import ast
import importlib.util
import math
import pathlib

ARM_NODE = (
    pathlib.Path(__file__).resolve().parent.parent
    / "ros2_ws"
    / "src"
    / "grippers_arm"
    / "grippers_arm"
    / "arm_driver_node.py"
)
DOMAIN_MISSION = (pathlib.Path(__file__).resolve().parent.parent
                  / "domain" / "task" / "baseline_mission.py")
DOMAIN_POLICY = DOMAIN_MISSION.with_name("floor_grasp_policy.py")
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
    glide = _function("_glide_phase")
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
    # idle로 가는 이동에는 손목 지연이 붙는다(RETURN_TO_IDLE_DEFERRED_JOINTS
    # 주석 참고) — 목표 자세 자체는 그대로 idle이다.
    assert "self._glide_to_raw_positions(backend, idle, defer_joints=" in source
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
    """접기는 서보 상태를 앞뒤로 확인하고 **검증된 IDLE 경로**로 가야 한다.

    ⚠️ 이 테스트는 예전에 `soarm.go` 호출을 요구했다 — 즉 결함을 고정하고
    있었다. 그 구현은 CRADLE_XYZ_M(자리표시자 좌표, "실측 필요" TODO 가
    달린)으로 역기구학 이동을 한 뒤 무조건 성공을 반환했고, 2026-08-28
    실기에서 성공을 반환한 직후 팔이 IDLE 에서 s3=-856 s5=-935 raw 떨어진
    자세에 서 있었다. 지금은 `_auto_align_to_idle` 에 위임한다 — 안전한
    경로를 고르고 도달까지 기다렸다가 답하는 유일한 경로다."""
    fn = _function("_on_fold_to_cradle")
    names = [_called_name(call) for call in _calls(fn)]

    assert names.count("_require_operational_servos") >= 2
    assert "_auto_align_to_idle" in names
    assert "go" not in names


def test_gripper_checks_servo_and_position_write_result():
    fn = _function("_on_set_gripper")
    names = [_called_name(call) for call in _calls(fn)]

    assert "_require_operational_servos" in names
    assert "set_position" in names


def test_gripper_calibration_matches_measured_safe_contract():
    calibration = _load_gripper_calibration()
    domain = _module_constants(DOMAIN_MISSION, {"CLOSED_MM"})
    domain.update(_module_constants(DOMAIN_POLICY, {"GRIPPER_MAX_SAFE_OPEN_MM"}))

    assert calibration.GRIPPER_CALIBRATION_POINTS == (
        (9.0, 1150),
        (96.0, 1578),
        (168.0, 2000),
    )
    assert domain == {"CLOSED_MM": 9.0, "GRIPPER_MAX_SAFE_OPEN_MM": 168.0}


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
    # 보간 본체는 _glide_phase에 있다 — 지연 관절 분리 이후
    # _glide_to_raw_positions는 구간을 나누는 역할만 한다.
    glide_names = [_called_name(call) for call in _calls(_function("_glide_phase"))]

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
    assert "idle 복귀는 safe/drop/carry 자세에서만 시작할 수 있습니다" in source


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


def test_wrist_pitch_moves_only_after_every_other_joint_has_stopped():
    """⚠️ 2026-08-24 실기 — 룩을 문 채 복귀할 때 차체 전면을 심하게 긁었다.

    처음에는 손목에 부분 지연(진행률 45%까지 정지)만 줬는데 그래도 긁었다 —
    겹치는 구간이 조금이라도 남으면 소용이 없다는 뜻이다. 사용자 지시대로
    아예 분리한다: 나머지가 목표에 도달해 정지한 뒤에야 손목이 움직인다."""
    deferred = _module_constants(ARM_NODE, {"RETURN_TO_IDLE_DEFERRED_JOINTS"})[
        "RETURN_TO_IDLE_DEFERRED_JOINTS"
    ]

    assert 4 in deferred
    # 안 움직이는 관절을 지연시키는 건 아무것도 안 하는 것과 같다 — 지연
    # 대상은 실제로 크게 움직이는 관절이어야 한다.
    poses = _module_constants(
        ARM_NODE.with_name("floor_grasp_profiles.py"),
        {"IDLE_CRADLE_RAW", "HORIZONTAL_SAFE_145_RAW"},
    )
    travel = {
        servo_id: abs(
            poses["IDLE_CRADLE_RAW"][servo_id - 1]
            - poses["HORIZONTAL_SAFE_145_RAW"][servo_id - 1]
        )
        for servo_id in range(1, 6)
    }
    assert travel[4] > 1000
    assert travel[5] < 100  # 사용자가 처음 지목한 관절 — 여기선 사실상 안 움직인다


def test_deferred_glide_runs_two_separate_phases():
    """1구간은 지연 관절을 출발 위치에 고정하고, 2구간에서만 움직인다.
    두 구간 각각이 도달을 확인하므로 겹칠 수 없다."""
    source = ast.unparse(_function("_glide_to_raw_positions"))
    names = [_called_name(call) for call in _calls(_function("_glide_to_raw_positions"))]

    assert names.count("_glide_phase") == 3  # 지연 없음 1회 + 지연 있음 2회
    assert "hold_deferred" in source
    # 도달 확인은 구간마다 일어나야 한다 — 그래야 "멈춘 뒤"가 보장된다.
    assert "_wait_floor_pose_arrived" in [
        _called_name(call) for call in _calls(_function("_glide_phase"))
    ]


def test_deferred_glide_never_changes_the_final_pose():
    """경로만 쪼갠다 — 두 구간의 최종 목표는 원래 goal 그대로여야 한다."""
    source = ast.unparse(_function("_glide_to_raw_positions"))

    # 1구간은 goal을 펼친 뒤 지연 관절만 start로 덮어쓴다.
    assert "hold_deferred = {**goal," in source
    # 2구간은 goal 자체를 그대로 쓴다.
    assert "self._glide_phase(backend, goal, label=" in source


def test_glide_phase_skips_a_leg_with_nothing_to_move():
    """지연 관절이 이미 목표에 있으면 빈 구간에 시간을 쓰지 않는다."""
    source = ast.unparse(_function("_glide_phase"))

    assert "if not moving:" in source
    assert "FLOOR_POSE_START_TOLERANCE_RAW" in source


def test_wrist_deferral_applies_only_when_returning_to_idle():
    """⚠️ 2026-08-24 실기 — 지연을 전역으로 걸었더니 방향 하나를 새로 깨뜨렸다.

    safe -> idle(차체로 복귀)은 지연으로 고쳐졌지만, 같은 지연이 걸린
    idle -> safe(차체에서 나감)가 새로 긁기 시작했다. 방향이 반대면 안전한
    관절 순서도 반대다 — 돌아올 때는 어깨가 먼저 물러난 뒤 손목이 접혀야
    하고, 나갈 때는 그 반대다. 그래서 지연은 이동마다 호출부가 정한다."""
    source = ast.unparse(_function("_move_floor_stage"))

    # idle을 목표로 하는 이동에만 붙는다.
    assert "backend, idle, defer_joints=RETURN_TO_IDLE_DEFERRED_JOINTS" in source
    # safe/grasp/midpoint/drop으로 가는 이동은 지연 없이 예전 그대로다.
    for target in ("safe", "drop"):
        assert f"self._glide_to_raw_positions(backend, {target})" in source
    assert "defer_joints" not in source.split("if stage == 'safe'")[1].split("if stage == 'drop'")[0]


def test_glide_defaults_to_no_deferral():
    """기본값이 '지연 없음'이어야 한다 — 새 호출부가 실수로 전역 지연을
    물려받는 일이 없도록."""
    glide = _function("_glide_to_raw_positions")
    # 상수 이름을 기본값으로 쓰는 인자(speed_raw=FLOOR_POSE_SPEED_RAW)도 있어
    # literal_eval이 통하지 않는다 — 리터럴인 것만 골라 본다.
    defaults = {}
    for arg, default in zip(glide.args.args[-len(glide.args.defaults):], glide.args.defaults):
        try:
            defaults[arg.arg] = ast.literal_eval(default)
        except ValueError:
            defaults[arg.arg] = ast.unparse(default)

    assert defaults["defer_joints"] == ()
    # 속도도 기본은 정상 이동 속도다 — 정렬 경로만 명시적으로 낮춰 부른다.
    assert defaults["speed_raw"] == "FLOOR_POSE_SPEED_RAW"


def test_domain_grasp_state_also_opens_before_descending():
    """FSM도 도구와 같은 순서여야 한다 — safe로 올라간 뒤 그리퍼를 열고,
    그다음에 grasp로 내려간다(사용자 지시, 2026-08-24). 도구만 고치고 FSM이
    반대로 남아 있으면 자동 시연에서 같은 사고가 난다."""
    source = DOMAIN_MISSION.read_text(encoding="utf-8")
    grasp = source[source.index("class BaselineGraspState"):
                   source.index("class BaselineCarryState")]

    open_at = grasp.index("set_gripper(gp.preopen_width_mm)")
    descend_at = grasp.index('move_to_floor_pose(gp.profile, "grasp")')
    assert open_at < descend_at


# --- 첫 이동 자동 IDLE 정렬 (2026-08-25 사용자 지시) -----------------------


def test_auto_align_on_first_move_is_on_by_default():
    """사용자 지시: "맨처음 이동 게이트에서 최초 로봇암의 자세를 파악하고
    무조건 자동으로 align_idle을 할 수 있게". 기본값이 True여야 그 지시가
    실제로 지켜진다 — 파라미터는 끄기 위한 탈출구일 뿐이다."""
    init = _function("__init__")

    declarations = [
        call
        for call in _calls(init)
        if _called_name(call) == "declare_parameter"
        and call.args
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == "auto_align_on_first_move"
    ]

    assert len(declarations) == 1
    assert declarations[0].args[1].value is True


def test_auto_align_runs_before_the_torque_gate_that_would_block_it():
    """정렬이 필요한 전형적 상황이 전원 투입 직후 torque OFF이고, 그건 정확히
    _require_operational_servos가 막는 상태다. 순서를 뒤집으면 자동 정렬이
    영영 불려 오지 못한다."""
    source = ast.unparse(_function("_execute_floor_pose"))

    assert source.index("_auto_align_to_idle()") < source.index(
        "_require_operational_servos"
    )


def test_auto_align_happens_once_per_session():
    """매 이동마다 정렬하면 팔이 단계마다 IDLE로 되돌아간다 — 첫 이동
    한 번만이어야 한다."""
    execute = ast.unparse(_function("_execute_floor_pose"))

    assert "self._auto_align_pending and req.stage != 'recover_idle'" in execute
    assert "self._auto_align_pending = False" in execute


def test_auto_align_skips_recover_idle_which_already_does_the_same_work():
    execute = ast.unparse(_function("_execute_floor_pose"))

    assert "req.stage != 'recover_idle'" in execute


def test_auto_align_latches_torque_before_writing_any_target():
    """⚠️ STS3215는 goal_position write에 torque를 자동으로 켠다. 늘어져 있는
    관절에 목표를 곧장 쓰면 그 순간 급하게 움직인다 — goal<-present를 먼저
    써서 torque만 켜고 이동량을 0으로 만든 뒤에야 목표를 쓴다."""
    align = ast.unparse(_function("_auto_align_to_idle"))

    assert align.index("_latch_torque_at_present") < align.index("_glide_to_raw_positions")

    latch = _function("_latch_torque_at_present")
    names = [_called_name(call) for call in _calls(latch)]
    # present를 읽고 그대로 되쓴다 — 이 순서가 계약의 전부다.
    assert names.index("get_position") < names.index("set_position")


def test_auto_align_never_interpolates_straight_to_idle_from_a_low_pose():
    """확립된 안전 규칙 — 바닥 높이에서 IDLE로 직선 보간하면 그리퍼가 바닥을
    쓸고 간다(2026-08-24 실기, 사용자: "이렇게 움직이는건 절대로 안돼").
    정렬도 recover_idle과 똑같이 등록 waypoint를 밟아 올라가야 한다."""
    align = ast.unparse(_function("_auto_align_to_idle"))

    # 등록 자세에 붙으면 검증된 상승 체인.
    assert "'grasp': (named[nearest.replace('grasp:', 'midpoint:')], safe, idle)" in align
    assert "'midpoint': (safe, idle)" in align
    # 어디에도 안 붙는데 팔이 앞·아래로 뻗어 있으면 safe로 먼저 들어올린다.
    assert "AUTO_ALIGN_LIFT_VIA_SAFE_SERVO2_RAW" in align
    assert "chain = (safe, idle)" in align


def test_auto_align_lift_threshold_sits_between_idle_and_the_grasp_poses():
    """문턱값이 '접혀 있다'와 '앞으로 뻗어 있다'를 실제로 가르는지 본다.
    IDLE servo2는 이 아래, 모든 grasp 자세의 servo2는 이 위여야 한다."""
    threshold = _module_constants(ARM_NODE, {"AUTO_ALIGN_LIFT_VIA_SAFE_SERVO2_RAW"})[
        "AUTO_ALIGN_LIFT_VIA_SAFE_SERVO2_RAW"
    ]
    profiles = ARM_NODE.with_name("floor_grasp_profiles.py")
    poses = _module_constants(profiles, {"IDLE_CRADLE_RAW", "HORIZONTAL_SAFE_145_RAW"})
    grasp_degs = _module_constants(
        profiles,
        {
            "HORIZONTAL_GABE_LOW_26_DEG",
            "HORIZONTAL_CHESS_ROOK_45_DEG",
            "HORIZONTAL_CHESS_QUEEN_50_DEG",
            "HORIZONTAL_CHESS_KNIGHT_60_DEG",
        },
    )

    assert poses["IDLE_CRADLE_RAW"][1] < threshold
    assert threshold < poses["HORIZONTAL_SAFE_145_RAW"][1]
    for name, angles in grasp_degs.items():
        servo2_raw = int(2048 + (angles[1] / 360.0) * 4095)
        assert servo2_raw > threshold, name


def test_auto_align_considers_every_profiles_grasp_pose():
    """정렬은 어느 profile로 파지하다 멈췄는지 모르는 채 불려 온다 — 정상
    경로와 달리 요청에 profile이 없으므로 여섯 자세를 전부 대봐야 한다."""
    named = ast.unparse(_function("_align_named_poses"))

    assert "HORIZONTAL_GRASP_POSES_DEG.items()" in named
    assert "midpoint:" in named


def test_auto_align_verifies_it_actually_reached_idle():
    """도달을 확인하지 않으면 '정렬했다'고 보고해 놓고 다음 단계가 시작
    자세 게이트에서 거부되는, 원인과 증상이 어긋난 실패가 난다."""
    align = ast.unparse(_function("_auto_align_to_idle"))
    tail = align[align.index("for waypoint in chain"):]

    assert "_read_joint_positions" in tail
    assert "_near_pose" in tail
    assert "ArmHardwareUnavailableError" in tail


def test_auto_align_only_closes_a_gripper_that_is_clearly_empty():
    """활짝 열린 손가락 판을 단 채 IDLE로 접으면 차체에 닿는다. 그렇다고
    무조건 닫으면, 물체를 문 채 정렬이 불려 왔을 때 그 물체를 으깬다 —
    servo 6에는 토크 제한 레지스터가 없어 위치 오차가 곧 힘이다."""
    threshold = _module_constants(ARM_NODE, {"AUTO_ALIGN_GRIPPER_CLOSE_ABOVE_MM"})[
        "AUTO_ALIGN_GRIPPER_CLOSE_ABOVE_MM"
    ]
    profiles_src = ARM_NODE.with_name("floor_grasp_profiles.py").read_text(encoding="utf-8")
    squeeze = next(
        ast.literal_eval(node.value)
        for node in ast.parse(profiles_src).body
        if isinstance(node, ast.Assign)
        and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id == "GRIPPER_SQUEEZE_MM"
    )
    calibration = _load_gripper_calibration()
    widest_object_mm = 46.0  # soccer_polyhedron — FLOOR_GRASP_PROFILES 최대 폭
    widest_close_mm = max(calibration.GRIPPER_CLOSED_MM, widest_object_mm - squeeze)

    assert threshold > widest_close_mm

    fn = ast.unparse(_function("_close_gripper_before_folding"))
    assert "width_from_position" in fn
    assert "AUTO_ALIGN_GRIPPER_CLOSE_ABOVE_MM" in fn


def test_auto_align_moves_more_slowly_than_a_verified_transition():
    """정렬은 출발 자세가 검증되지 않은 유일한 이동이다."""
    speeds = _module_constants(ARM_NODE, {"AUTO_ALIGN_SPEED_RAW", "FLOOR_POSE_SPEED_RAW"})

    assert speeds["AUTO_ALIGN_SPEED_RAW"] < speeds["FLOOR_POSE_SPEED_RAW"]
    align = ast.unparse(_function("_auto_align_to_idle"))
    assert "speed_raw=AUTO_ALIGN_SPEED_RAW" in align


def test_arm_state_service_reports_unreadable_servos_instead_of_zeroing_them():
    """못 읽은 것을 0으로 보고하면 '부하 없음'과 구분되지 않는다 — 검증
    도구가 통신 실패를 정상 측정값으로 착각한다."""
    fn = _function("_on_get_arm_state")
    source = ast.unparse(fn)

    assert "online.append(False)" in source
    assert "response.ok = not offline" in source
    source_names = ast.unparse(fn)
    for reader in ("get_position", "get_load", "get_temperature", "get_torque"):
        assert reader in source_names


def test_arm_state_retries_each_register_read():
    """⚠️ 회귀 — get_arm_state는 서보 6개 × 레지스터 4개 = 24회 연속 읽기다.
    이 버스는 패킷을 이따금 흘리므로, 재시도가 없으면 정지 상태에서도 묶음이
    10번에 1번꼴로 깨진다(2026-08-25 실측). 파지력 측정 도구가 첫 표본에서
    그대로 멈췄다."""
    source = ast.unparse(_function("_on_get_arm_state"))

    # 드라이버를 직접 부르지 않고 반드시 재시도 헬퍼를 거친다.
    assert "_read_with_retry" in source
    assert "backend.drv.get_position(servo_id)" not in source

    retry = _function("_read_with_retry")
    retry_source = ast.unparse(retry)
    assert "JOINT_READ_ATTEMPTS" in ast.unparse(retry.args)
    assert "JOINT_READ_RETRY_SEC" in retry_source
    # 이동 중 폴링과 같은 계약 — 다 실패해야 None이다.
    assert retry_source.rstrip().endswith("return None")


# --- 파지 전용 그리퍼 하한 (2026-08-25) --------------------------------------


def test_the_grasp_floor_is_pushed_to_the_servo_limit():
    """2026-08-25 gripper_force_probe 실측(knight을 문 채 부하 9.0mm 0.0235 /
    8.0mm 0.0430 / 7.0mm 0.0626, 그 아래는 전부 0.0626)은 **모터 축 부하
    기준** 포화점이었다. 2026-09-02 사용자 지시: 기어 사이 이격(백래시)이
    있어 그 판독만으로는 핑거 끝의 실제 조임을 다 못 본다 — 서보가 받는
    한계(0.0)까지 더 내린다."""
    calibration = _load_gripper_calibration()

    assert calibration.GRIPPER_GRASP_MIN_MM == 0.0
    # 빈 닫힘 하한은 건드리지 않는다 — 얻을 것이 없다(빈 턱은 raw 1144에서
    # 멈추고 그 아래로 명령해도 부하가 안 는다).
    assert calibration.GRIPPER_CLOSED_MM == 9.0
    assert calibration.GRIPPER_GRASP_MIN_MM < calibration.GRIPPER_CLOSED_MM


def test_position_from_width_is_unchanged_when_no_floor_is_given():
    """기본 인자로 부르는 기존 호출부의 동작이 한 raw도 달라지면 안 된다."""
    calibration = _load_gripper_calibration()

    assert calibration.position_from_width(9.0) == 1150
    assert calibration.position_from_width(96.0) == 1578
    assert calibration.position_from_width(168.0) == 2000
    # 하한 아래 요청은 여전히 하한으로 clamp된다.
    assert calibration.position_from_width(4.0) == calibration.position_from_width(9.0)


def test_a_lowered_floor_extrapolates_the_first_calibration_segment():
    """보정표 아래는 실측점이 없다 — 첫 구간 기울기를 외삽하고, 그 사실을
    docstring이 분명히 말한다(돌아오는 raw는 실제 개구 폭의 예측이 아니다)."""
    calibration = _load_gripper_calibration()
    slope = (1578 - 1150) / (96.0 - 9.0)

    assert calibration.position_from_width(4.0, min_width_mm=2.0) == round(
        1150 + (4.0 - 9.0) * slope
    )
    assert calibration.position_from_width(4.0, min_width_mm=2.0) < 1150
    assert "외삽" in calibration.position_from_width.__doc__


def test_the_gripper_floor_is_a_runtime_parameter_defaulting_to_the_grasp_floor():
    """평소 경로에서 실수로 낮은 값이 쓰이지 않도록 기본값은 파지 하한이고,
    낮추려면 런타임에 명시적으로 바꿔야 한다."""
    init = _function("__init__")
    declarations = [
        call
        for call in _calls(init)
        if _called_name(call) == "declare_parameter"
        and call.args
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == "min_gripper_width_mm"
    ]

    assert len(declarations) == 1
    assert declarations[0].args[1].id == "GRIPPER_GRASP_MIN_MM"

    source = ast.unparse(_function("_on_set_gripper"))
    assert "get_parameter('min_gripper_width_mm')" in source
    assert "min_width_mm=min_width_mm" in source
    # 빈 닫힘 하한보다 좁게 명령하면 반드시 경고를 남긴다.
    assert "GRIPPER_CLOSED_MM" in source


# --- 시리얼 포트 배타 잠금 (2026-08-25) --------------------------------------


def test_the_serial_port_is_claimed_exclusively_at_startup():
    """⚠️ 회귀 — 재기동 스크립트의 pkill 패턴이 설치된 노드 실행 파일을 놓쳐
    arm_driver 세 개가 같은 포트를 동시에 쓰고 있었다. 증상이 하드웨어 고장과
    구분되지 않는다: 실패율이 호출마다 달라지고, 실패 서보 목록이 바뀌고,
    깨진 값(servo 3 = 55841)이 정상인 척 통과한다."""
    init = ast.unparse(_function("__init__"))
    assert "_claim_serial_port" in init
    # 포트를 연 직후여야 한다 — 그 전에는 잠글 핸들이 없다.
    assert init.index("RealBackend(port=arm_port)") < init.index("_claim_serial_port")

    claim = ast.unparse(_function("_claim_serial_port"))
    assert "LOCK_EX" in claim and "LOCK_NB" in claim
    assert "ArmPortConflictError" in claim
    # 핸들을 인스턴스에 붙들어야 GC가 닫아 잠금이 풀리지 않는다.
    assert "self._port_lock_file" in claim


def test_a_port_conflict_stops_the_node_instead_of_running_degraded():
    """두 번째 인스턴스가 조용히 함께 도는 것이 최악이다 — 그 상태에서
    나오는 값은 틀린 줄도 모르고 쓰인다. main이 이미 이 예외를 잡아 노드를
    띄우지 않고 종료한다."""
    main_source = ast.unparse(_function("main"))

    assert "ArmPortConflictError" in main_source
    assert "fatal" in main_source


def test_out_of_range_positions_are_discarded_not_reported_as_valid():
    """STS3215 위치는 정의상 0..4095다. 그 밖의 값은 응답 바이트가 섞인
    것이므로 online=True로 보고하면 안 된다."""
    constants = _module_constants(ARM_NODE, {"POSITION_RAW_MAX"})
    assert constants["POSITION_RAW_MAX"] == 4095

    source = ast.unparse(_function("_on_get_arm_state"))
    assert "POSITION_RAW_MAX" in source
    # 범위 검사가 online 판정보다 먼저 와야 버려진다.
    assert source.index("POSITION_RAW_MAX") < source.index("online.append(True)")


def test_carry_stage_defers_the_wrist_exactly_like_idle_does():
    """CARRY 이동에도 servo 4 지연이 걸려야 한다 (2026-08-26).

    safe에서 carry로 갈 때 servo 4 이동량은 +1381 raw(121도)로, idle로 갈
    때(+1618)와 같은 성격의 큰 이동이다. 지연 없이 보내면 어깨가 내려가는
    동안 손목이 같은 비율로 접혀 그리퍼가 차체 전면을 긁는다 —
    RETURN_TO_IDLE_DEFERRED_JOINTS 주석이 기록한 2026-08-24 사고가 그것이고,
    거기서 룩을 놓쳤다.

    지연 조건이 `waypoint is idle` 동일성 검사라, carry를 별도 자세로 만들면
    조건에서 빠지기 쉽다. 그 회귀를 여기서 막는다."""
    source = ast.unparse(_function("_move_floor_stage"))

    carry_block = source.split("if stage == 'carry':", 1)
    assert len(carry_block) == 2, "carry 분기가 없다"
    body = carry_block[1].split("if stage ==", 1)[0]
    assert "RETURN_TO_IDLE_DEFERRED_JOINTS" in body, (
        "carry 이동이 손목 지연 없이 보간한다 — 차체 전면을 긁는다"
    )


def test_carry_is_an_accepted_stage_and_reachable_from_safe():
    assert "'carry'" in ast.unparse(_function("_execute_floor_pose"))
    source = ast.unparse(_function("_move_floor_stage"))
    # INSERT는 carry에서 곧장 drop으로 간다.
    assert "drop 이동은 idle/safe/carry 자세에서만 시작할 수 있습니다" in source

def test_좌우_보정_허용오차가_정렬_허용치를_분간할_수_있다():
    """2026-08-28 실기 회귀 — 보정이 서보 격자에 걸려 통째로 버려졌다.

    `_glide_phase` 는 목표까지의 차이가 허용오차보다 작으면 그 관절을
    "이미 도착"으로 보고 이동 대상에서 뺀다. 교시 자세용 기본값
    FLOOR_POSE_START_TOLERANCE_RAW(120 raw ≈ 10.5도)를 좌우 보정에도 쓰면,
    분간해야 할 단위보다 격자가 커서 **어떤 보정도 실행되지 않는다.**
    그때 로그는 이렇게 남았고 서비스는 성공을 보고했다.

        offset_base_yaw: servo 1 2067 -> 2067 (+5.2도)

    분간해야 하는 단위는 좌우 허용치 GRASP_CENTERING_TOLERANCE_M(10mm)를
    servo 1 회전각으로 옮긴 값이다. 그보다 격자가 크면 원리적으로 못 맞춘다.
    """
    arm = _module_constants(ARM_NODE, {"BASE_YAW_TOLERANCE_RAW",
                                       "FLOOR_POSE_START_TOLERANCE_RAW"})
    domain = _module_constants(
        DOMAIN_MISSION.with_name("baseline_constants.py"),
        {"GRASP_CENTERING_TOLERANCE_M", "SERVO1_AXIS_TO_JAW_MM"})

    # STS3215 는 한 바퀴가 4096 카운트다(arm_driver_node.RAW_PER_RADIAN).
    raw_per_rad = 4096.0 / (2.0 * math.pi)
    tolerance_rad = math.atan2(domain["GRASP_CENTERING_TOLERANCE_M"],
                               domain["SERVO1_AXIS_TO_JAW_MM"] / 1000.0)
    tolerance_raw = tolerance_rad * raw_per_rad

    assert arm["BASE_YAW_TOLERANCE_RAW"] < tolerance_raw, (
        f"보정 허용오차 {arm['BASE_YAW_TOLERANCE_RAW']} raw 가 "
        f"정렬 허용치 {tolerance_raw:.0f} raw 보다 크다 — 보정이 버려진다")
    # 그리고 교시 자세용 격자를 그대로 쓰면 안 된다는 것도 같이 못 박는다.
    assert arm["FLOOR_POSE_START_TOLERANCE_RAW"] > tolerance_raw


def test_좌우_보정은_전용_허용오차로_움직인다():
    """`_on_offset_base_yaw` 가 기본 격자를 그대로 쓰면 위 계산이 무의미하다."""
    fn = _function("_on_offset_base_yaw")
    source = ast.unparse(fn)

    assert "tolerance_raw=" in source, "허용오차를 안 넘긴다 — 기본 격자로 간다"
    assert "BASE_YAW_TOLERANCE_RAW" in source


def test_좌우_보정은_도달을_확인하고_답한다():
    """예전엔 무조건 ok=True 였다. 안 움직였는데 성공이 나가면 Pi 는 보정이
    먹은 줄 알고 같은 관측·같은 보정을 무한히 반복한다."""
    fn = _function("_on_offset_base_yaw")
    source = ast.unparse(fn)

    assert "response.ok = abs(" in source or "abs(error) <= BASE_YAW_TOLERANCE_RAW" in source, (
        "도달 확인 없이 ok 를 정하고 있다")
    assert "response.ok = True" not in source, "무조건 성공을 반환하면 안 된다"


def test_한계각은_교시_정면_기준_절대각이다():
    """이 서비스는 현재 위치 기준 **상대** 회전이다. 한 번의 요청 크기만
    보면 같은 요청이 반복될 때 servo 1 이 얼마든지 멀리 걸어간다."""
    fn = _function("_on_offset_base_yaw")
    source = ast.unparse(fn)

    assert "IDLE_CRADLE_RAW" in source, "교시 정면을 기준으로 안 잡는다"
    assert "MAX_BASE_YAW_OFFSET_RAD" in source


# ── servo 1이 틀어진 채로 GRASP를 도는가 (2026-08-29) ──────────────────────


def _class_constant(name):
    """`MissionArmDriverNode` 클래스 몸통의 상수. `math.radians(...)`도 푼다.

    `_module_constants`는 모듈 최상위의 리터럴만 읽는다 — 한계각은 클래스
    속성이고 값도 호출식이라 그쪽으로는 안 잡힌다."""
    tree = ast.parse(ARM_NODE.read_text(encoding="utf-8"), filename=str(ARM_NODE))
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)
                and node.targets[0].id == name):
            return eval(ast.unparse(node.value), {"math": math})   # noqa: S307
    raise AssertionError(f"{name} 을 찾지 못했다")


def test_한계각까지_틀어진_servo1은_교시_자세_게이트를_통과할_수_없다():
    """**freeze 가 왜 필수인지**를 못 박는다.

    `_move_floor_stage`는 단계마다 `_near_pose`(±FLOOR_POSE_START_TOLERANCE_RAW)
    로 "지금 등록된 이전 자세에 있는가"를 검사한다. servo 1을 좌우 보정으로
    돌려 놓으면 그 값이 교시 절대값에서 그만큼 벌어지므로, freeze 없이 교시
    자세를 그대로 비교하면 **보정을 건 회차는 GRASP 전 단계가 통째로 거부된다**
    ("safe 이동 시작 자세가 등록된 ... 이 아닙니다").

    한계각 15도 = 171 raw 는 게이트 120 raw 보다 크다. 즉 이 충돌은 이론이
    아니라 한계각 안의 정상 보정에서 실제로 일어난다."""
    limit_rad = _class_constant("MAX_BASE_YAW_OFFSET_RAD")
    gate = _module_constants(ARM_NODE, {"FLOOR_POSE_START_TOLERANCE_RAW"})
    limit_raw = limit_rad * 4096.0 / (2.0 * math.pi)

    assert limit_raw > gate["FLOOR_POSE_START_TOLERANCE_RAW"], (
        f"한계각 {math.degrees(limit_rad):.0f}도 = {limit_raw:.0f} raw 가 게이트 "
        f"{gate['FLOOR_POSE_START_TOLERANCE_RAW']} raw 보다 작아졌다 — 그렇다면 "
        "freeze 없이도 통과하므로 이 테스트의 전제를 다시 확인할 것")


def test_GRASP_이_밟는_세_자세가_전부_현재_servo1을_물려받는다():
    """safe -> grasp -> midpoint -> safe 가 GRASP 하강 경로다.

    셋 중 하나라도 교시 절대값을 쓰면 그 단계에서 팔이 정면으로 홱 돌아가고,
    이미 겨눠 둔 좌우 정렬이 그 순간 사라진다. midpoint 는 grasp 와 safe 의
    관절별 평균이라 둘이 얼어 있으면 자동으로 따라온다."""
    source = ast.unparse(_function("_move_floor_stage"))

    assert "safe = _freeze_servo1(" in source
    assert "grasp = _freeze_servo1(" in source
    assert "midpoint = {" in source and "(grasp[servo_id] + safe[servo_id])" in source


def test_carry_는_교시_절대값이라_servo1이_정면으로_돌아온다():
    """GRASP 의 마지막 단계는 carry 이고, 여기서 보정이 풀리는 것이 **맞다.**

    carry 는 물체를 들고 주행하는 전달 자세다. 좌우로 돌아간 채 접으면
    그리퍼가 라이다 시야로 들어와 바구니를 못 본다(CARRY_RAW 주석). 파지가
    끝난 뒤에는 물체가 이미 턱 안에 있으므로 정면으로 되돌려도 잃을 것이 없다.

    즉 보정의 수명은 `offset_base_yaw` 부터 `carry` 까지다."""
    source = ast.unparse(_function("_move_floor_stage"))

    assert "carry = self._tuple_goals(CARRY_RAW)" in source
    assert "carry = _freeze_servo1(" not in source
    # idle·drop 도 같은 이유로 절대값이다.
    assert "idle = self._tuple_goals(IDLE_CRADLE_RAW)" in source
    assert "drop = self._tuple_goals(BASKET_DROP_195_RAW)" in source


def test_팔_길이_주석이_실측값과_어긋나지_않는다():
    """2026-08-29 정정 — 주석 세 곳이 실측 전 어림값(214mm·240mm)을 들고 있었다.

    코드는 안 틀렸지만 그 숫자를 근거로 한계각이나 허용오차를 조정하면
    틀린 값이 퍼진다. 팔 길이의 단일 출처는 baseline_constants 다."""
    domain = _module_constants(
        DOMAIN_MISSION.with_name("baseline_constants.py"), {"SERVO1_AXIS_TO_JAW_MM"})
    assert domain["SERVO1_AXIS_TO_JAW_MM"] == 294.0

    # 옛 값이 **이력으로** 남는 것은 좋다 — 왜 바뀌었는지가 다음 사람에게
    # 필요하다. 막으려는 것은 그것이 다시 **현재 근거**로 쓰이는 것이다.
    # 그래서 줄 단위로 보고, 이력 표식이 있는 줄만 봐준다.
    stale_claims = [
        line.strip()
        for line in ARM_NODE.read_text(encoding="utf-8").splitlines()
        if ("214mm" in line or "240mm" in line) and "적혀 있었다" not in line
    ]
    assert not stale_claims, (
        f"팔 길이 실측은 {domain['SERVO1_AXIS_TO_JAW_MM']:.0f}mm 인데 실측 전 값을 "
        f"현재 근거로 쓰는 줄이 있다:\n  " + "\n  ".join(stale_claims))
