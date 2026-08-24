"""demo_rook_run.py의 구조 계약 검사.

rclpy 의존이라 개발 머신에서 import할 수 없다 — test_grasp_cycle.py와 같은
방식으로 AST로 읽는다. 순수 계산인 정지 밴드 로직만 따로 검증한다.
"""

import ast
import pathlib

TOOL = pathlib.Path(__file__).resolve().parent.parent / "tools" / "demo_rook_run.py"
CONSOLE = pathlib.Path(__file__).resolve().parent.parent / "tools" / "grasp_test_console.py"


def _tree(path=TOOL):
    return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def _function(name, path=TOOL):
    return next(
        node
        for node in ast.walk(_tree(path))
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


def test_the_four_phases_run_in_the_order_the_user_specified():
    """사용자 지시(2026-08-24): APPROACH(space/a/d, c) -> g로 GRASP 및
    CARRY_IDLE -> wasd 운반, c -> g로 바구니 투하 및 IDLE 복귀."""
    source = ast.unparse(_function("main"))

    approach = source.index("SPACE_KEYMAP")
    grasp = source.index("move_floor_pose(profile, 'grasp')")
    carry = source.index("WASD_KEYMAP")
    drop = source.index("move_floor_pose(profile, 'drop')")
    assert approach < grasp < carry < drop


def test_both_stage_transitions_wait_for_the_g_key():
    """c로 멈춘 자리에서 사람이 g를 누를 때까지 팔은 움직이지 않는다."""
    source = ast.unparse(_function("main"))

    assert source.count("wait_for_key(kr, 'g'") == 2
    # 첫 g는 GRASP 앞, 두 번째 g는 투하 앞이다.
    first = source.index("wait_for_key(kr, 'g'")
    second = source.index("wait_for_key(kr, 'g'", first + 1)
    assert first < source.index("move_floor_pose(profile, 'safe')") < second
    assert second < source.index("move_floor_pose(profile, 'drop')")


def test_gripper_opens_before_the_arm_descends():
    """닫힌 손가락이 물체가 있는 공간을 통과해 내려가면 물체를 밀어낸다
    (사용자 지시, 2026-08-24)."""
    source = ast.unparse(_function("main"))

    assert source.index("set_gripper(preopen_mm)") < source.index(
        "move_floor_pose(profile, 'grasp')"
    )


def test_lift_chain_goes_through_midpoint_and_safe():
    """바닥에서 IDLE로 곧장 가면 그리퍼가 바닥을 쓸어간다 — 검증된 상승
    체인을 그대로 밟는다."""
    source = ast.unparse(_function("main"))

    assert "('midpoint', 'safe', 'idle')" in source


def test_every_arm_failure_path_recovers_to_idle():
    source = ast.unparse(_function("main"))

    # safe / grasp / 닫기 / 상승 / drop / 투하 후 idle — 여섯 지점.
    assert source.count("recover_to_idle") >= 6


def test_the_wheels_are_stopped_on_every_exit_path():
    """팔은 자세 게이트가 지켜 주지만 cmd_vel은 마지막 값이 그대로 유지된다 —
    q로 끊든 실패로 빠지든 바퀴부터 세워야 한다."""
    fn = _function("main")
    tries = [node for node in ast.walk(fn) if isinstance(node, ast.Try)]
    finalbody = ast.unparse(tries[0].finalbody)

    assert "node.stop()" in finalbody


def test_the_tool_does_not_touch_the_gripper_camera():
    """그리퍼캠을 열려면 perception_node를 죽여야 하는데, 그러면 APPROACH의
    거리 표시가 함께 죽는다. 게다가 2026-08-24 6종 수집에서 그리퍼캠 면적은
    파지 판정에 쓸 수 없다는 것이 확인됐다(빈 그리퍼가 룩을 문 상태보다 큼)."""
    # 주석에서는 "왜 안 쓰는지"를 설명하므로 이름이 등장한다 — 실제로 불러
    # 쓰는지를 본다.
    names = {node.id for node in ast.walk(_tree()) if isinstance(node, ast.Name)}
    imported = {
        alias.name
        for node in ast.walk(_tree())
        if isinstance(node, ast.ImportFrom)
        for alias in node.names
    }

    assert "GripperCam" not in names | imported
    assert "measure_area" not in names
    assert "restart_perception_node" not in names | imported


def test_grasp_verdict_uses_load_and_respects_quantisation():
    """load는 4/1023 = 0.00391 단위로 양자화돼 있다 — 한 단위 차이는 잡음과
    구분이 안 되므로 두 단위 이상을 요구한다."""
    constants = _constants({"EMPTY_CARRY_LOAD", "LOAD_MARGIN"})

    quantum = 4 / 1023
    assert constants["EMPTY_CARRY_LOAD"] == 0.0352  # 2026-08-24 --empty 실측
    assert 1.9 <= constants["LOAD_MARGIN"] / quantum <= 2.5
    # 판정은 CARRY_IDLE의 load로 한다 — 닫은 직후 값은 아직 손가락이 물체를
    # 밀고 있는 과도값일 수 있다.
    source = ast.unparse(_function("main"))
    assert "carry_load - EMPTY_CARRY_LOAD > LOAD_MARGIN" in source


def test_stop_band_for_rook_brackets_both_measured_successful_placements():
    """정지 거리는 계산이 아니라 실측이다 — 2026-08-24 grasp_cycle에서 팔이
    그 자리에서 바로 잡는 데 성공한 rook 배치는 16.0cm와 18.6cm였다."""
    constants = _constants({"STOP_BAND_M", "DEFAULT_STOP_BAND_M"})
    low, high = constants["STOP_BAND_M"]["rook"]

    assert low <= 0.160
    assert high >= 0.186
    # 다른 클래스에는 이 실측이 없다 — 기본값이 rook 값을 흉내내면 안 된다.
    assert constants["DEFAULT_STOP_BAND_M"] != constants["STOP_BAND_M"]["rook"]


def test_approach_report_only_advises_and_never_brakes():
    """사용자의 보정 방식 선호: 사람이 몰고, 멈출 조건만 실시간으로 알려준다.
    시연 중 자동 개입은 영상에서 무슨 일이 일어났는지 알아보기 어렵게 만든다."""
    source = ast.unparse(_function("approach_report"))

    assert "cmd_pub" not in source
    assert "Twist" not in source
    assert "stop()" not in source


def test_drive_loop_republishes_after_a_slow_report():
    """report()가 observe 서비스로 수백 ms를 쓰는 동안 cmd_vel이 끊기면 base
    드라이버 워치독이 차를 세워 주행이 툭툭 끊긴다 — 시연 영상에서는 그게
    바로 보인다."""
    source = ast.unparse(_function("drive_phase", CONSOLE))

    report_at = source.index("report()")
    assert "node.cmd_pub.publish(t)" in source[report_at:]
