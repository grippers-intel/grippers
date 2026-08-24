"""auto_grasp_sequence.py의 상수·구조 계약 검사.

이 도구는 rclpy와 grasp_test_console(역시 rclpy 의존)을 import하므로 일반
개발 머신에서 직접 실행할 수 없다. arm_driver_node와 같은 방식으로 소스를
AST로 읽어, 실기에서 확정한 값과 안전 구조가 조용히 사라지지 않는지 본다.
"""

import ast
import pathlib

TOOL = pathlib.Path(__file__).resolve().parent.parent / "tools" / "auto_grasp_sequence.py"


def _tree():
    return ast.parse(TOOL.read_text(encoding="utf-8"), filename=str(TOOL))


def _constants(names):
    return {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in _tree().body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id in names
    }


def _function(name):
    return next(
        node
        for node in ast.walk(_tree())
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name
    )


def test_alignment_targets_match_the_agreed_values():
    """사용자 지정(2026-08-24): 카메라 기준 25cm 정면, 회전 속도 0.25 rad/s."""
    c = _constants({"TARGET_FORWARD_M", "ALIGN_TURN_RAD_S"})

    assert c["TARGET_FORWARD_M"] == 0.25
    assert c["ALIGN_TURN_RAD_S"] == 0.25


def test_grip_area_target_matches_the_agreed_value():
    """사용자 지정: 그리퍼캠 면적 12만 이상이면 정지하고 파지."""
    assert _constants({"GRIPPER_AREA_TARGET_PX2"})["GRIPPER_AREA_TARGET_PX2"] == 120000.0


def test_area_target_sits_above_every_measured_successful_grasp_margin():
    """실측 파지 성공 사례(2026-08-24): 면적 44365에서 load 0.0665로 가장
    아슬아슬했고, 141293에서 0.0899로 가장 확실했다. 목표 면적은 그 아슬아슬한
    쪽보다 충분히 높아야 의미가 있다."""
    target = _constants({"GRIPPER_AREA_TARGET_PX2"})["GRIPPER_AREA_TARGET_PX2"]

    assert target > 44365 * 2


def test_alignment_tolerances_are_looser_than_the_distance_model_error():
    """거리 모델의 실측 오차는 40/70/104cm에서 ±0.4cm였다. 허용 오차를 그보다
    타이트하게 잡으면 정렬이 영원히 수렴하지 못한다."""
    c = _constants({"FORWARD_TOL_M", "LATERAL_TOL_M"})

    assert c["FORWARD_TOL_M"] > 0.004
    assert c["LATERAL_TOL_M"] > 0.004


def test_lateral_tolerance_is_well_inside_the_gripper_opening():
    """물체가 손가락 사이로 들어와야 한다 — 개구 168mm의 절반보다 훨씬 작아야
    한다."""
    assert _constants({"LATERAL_TOL_M"})["LATERAL_TOL_M"] < 0.168 / 4


def test_approach_stops_on_area_not_on_odometry():
    """⚠️ /odom_raw는 명령을 그대로 적분한다 — 바퀴가 멈춰 있어도 이동했다고
    보고한다. 2026-08-24 실기 로그 두 건에서 '잔여거리 x 면적'이 물리적으로
    일정해야 하는데 1830 대 996으로 1.8배 어긋났다. 그래서 정지 판정은 반드시
    면적으로만 하고, 오도메트리는 폭주 방지 상한으로만 쓴다."""
    source = ast.unparse(_function("approach_until_area"))

    # 도달 판정 = 면적 비교
    assert "area >= GRIPPER_AREA_TARGET_PX2" in source
    # 오도메트리는 상한 검사에만 등장한다
    assert "travelled > APPROACH_MAX_TRAVEL_M" in source
    assert "travelled >= " not in source
    assert "travelled ==" not in source


def test_approach_has_travel_time_and_blind_guards():
    """면적이 영원히 안 잡히거나 물체를 지나쳐도 멈춰야 한다."""
    source = ast.unparse(_function("approach_until_area"))

    assert "APPROACH_MAX_TRAVEL_M" in source
    assert "APPROACH_MAX_SEC" in source
    assert "APPROACH_MAX_BLIND_SEC" in source


def test_approach_always_stops_the_base_even_on_failure():
    """어떤 경로로 빠져나가든 정지 명령이 나가야 한다 — finally에 둔다."""
    fn = _function("approach_until_area")
    tries = [node for node in ast.walk(fn) if isinstance(node, ast.Try)]

    assert tries and tries[0].finalbody
    assert "Twist()" in ast.unparse(tries[0].finalbody)


def test_align_issues_turn_and_drive_separately():
    """2026-08-23 실기에서 회전과 전진을 섞어 냈다가 로봇이 좌측으로 90도 돌아
    목표를 이탈했다. 한 번에 하나씩만 낸다."""
    source = ast.unparse(_function("align"))

    # 회전을 낸 뒤에는 바로 다음 관측으로 넘어간다(continue) — 같은 반복에서
    # 전진을 함께 내지 않는다.
    turn_index = source.index("turn_burst")
    drive_index = source.index("drive_burst")
    assert "continue" in source[turn_index:drive_index]


def test_align_detects_a_turn_that_never_actually_happens():
    """회전 속도 0.25는 실측으로 도는 것이 확인된 최저값(0.3)보다 낮다.
    안 돌면 명령만 나가는데 오도메트리는 돌았다고 보고하므로, 실제로 도는지는
    **관측 x가 움직였는지**로만 알 수 있다."""
    source = ast.unparse(_function("align"))

    assert "TURN_PROGRESS_PX" in source
    assert "TURN_STALL_LIMIT" in source


def test_every_failure_path_recovers_the_arm():
    """팔을 중간 자세에 세워 둔 채 끝내지 않는다 — 팔을 움직인 뒤의 실패
    경로는 전부 recover_idle을 부른다(2026-08-24 사용자 요청)."""
    source = ast.unparse(_function("main"))

    # 팔이 움직인 뒤 생길 수 있는 실패 반환값 3(진입) · 4(접근) · 5(닫기) ·
    # 6(복귀) 각각에 복구가 붙어 있어야 한다.
    assert source.count("recover_idle") >= 4
    # Ctrl+C는 예외다 — 사람이 일부러 끊은 것이라 팔을 자동으로 움직이지
    # 않고 주행만 멈춘다(grasp_test_console.py와 같은 규약).
    handlers = [n for n in ast.walk(_function("main")) if isinstance(n, ast.ExceptHandler)]
    interrupt = next(h for h in handlers if ast.unparse(h.type) == "KeyboardInterrupt")
    assert "recover_idle" not in ast.unparse(interrupt)


def test_main_restarts_perception_node_it_killed():
    """그리퍼캠을 넘겨받으려고 죽인 perception_node는 반드시 되살린다 —
    안 그러면 다음 실행의 정렬이 조용히 무력화된다."""
    fn = _function("main")
    tries = [node for node in ast.walk(fn) if isinstance(node, ast.Try)]
    finalbody = ast.unparse(tries[0].finalbody)

    assert "restart_perception_node" in finalbody
    # 카메라를 놓아준 뒤에 되살려야 장치 경합이 안 난다.
    assert finalbody.index("cam.close") < finalbody.index("restart_perception_node")
