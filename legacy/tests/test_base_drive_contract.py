"""base_driver drive_to의 #148 2단계 주행 계약을 정적으로 검사한다.

로컬에는 ROS2/rclpy가 없어 노드를 import해 실행할 수 없다. 대신 AST로
소스를 검사해 회전+직진 동시 제어 회귀와 서버 timeout 제거를 막는다.
"""

import ast
import math
import pathlib

BASE_NODE = (
    pathlib.Path(__file__).resolve().parent.parent
    / "ros2_ws"
    / "src"
    / "grippers_base"
    / "grippers_base"
    / "base_driver_node.py"
)


def _parse():
    return ast.parse(BASE_NODE.read_text(encoding="utf-8"), filename=str(BASE_NODE))


def _drive_to():
    return next(
        node
        for node in ast.walk(_parse())
        if isinstance(node, ast.FunctionDef) and node.name == "_execute_drive_to"
    )


def _phase_branch(fn):
    for node in ast.walk(fn):
        if not isinstance(node, ast.If):
            continue
        test = node.test
        if not isinstance(test, ast.Compare):
            continue
        if not isinstance(test.left, ast.Name) or test.left.id != "phase":
            continue
        if len(test.comparators) != 1:
            continue
        comp = test.comparators[0]
        if isinstance(comp, ast.Constant) and comp.value == "ALIGN":
            return node.body, node.orelse
    raise AssertionError("phase == ALIGN 분기를 찾지 못했다")


def _module_constants():
    out = {}
    for node in _parse().body:
        if not isinstance(node, ast.Assign):
            continue
        if len(node.targets) != 1 or not isinstance(node.targets[0], ast.Name):
            continue
        if isinstance(node.value, ast.Constant):
            out[node.targets[0].id] = node.value.value
    return out


def _source(node):
    text = BASE_NODE.read_text(encoding="utf-8")
    return ast.get_source_segment(text, node)


def test_two_phase_thresholds_have_hysteresis():
    constants = _module_constants()

    assert constants["YAW_ALIGN_TOL_RAD"] < constants["YAW_REALIGN_TRIG_RAD"]
    assert constants["REALIGN_MIN_DIST_M"] > constants["ARRIVE_XY_TOL"]
    assert (
        constants["REALIGN_MIN_DIST_M"] * math.sin(constants["YAW_REALIGN_TRIG_RAD"])
        < constants["ARRIVE_XY_TOL"]
    )


def test_server_timeout_precedes_client_timeout():
    constants = _module_constants()

    assert 0 < constants["DRIVE_TO_TIMEOUT_SEC"] < 60.0


def test_drive_timeout_uses_monotonic_clock():
    source = _source(_drive_to())

    assert "time.monotonic()" in source
    assert "self.get_clock().now()" not in source


def test_align_phase_commands_rotation_without_translation():
    align_body, _ = _phase_branch(_drive_to())
    source = "\n".join(_source(node) or "" for node in align_body)

    assert "twist.linear.x = 0.0" in source
    assert "KP_ANGULAR * yaw_err" in source
    assert "forward_speed(" not in source


def test_drive_phase_commands_translation_without_rotation():
    _, drive_body = _phase_branch(_drive_to())
    source = "\n".join(_source(node) or "" for node in drive_body)

    assert "forward_speed(dist, yaw_err)" in source
    assert "twist.angular.z = 0.0" in source
    assert "KP_ANGULAR * yaw_err" not in source


def test_drive_phase_speed_is_signed_by_heading():
    """#148 잔여 회귀 방지 — 부호 없는 `dist` 를 그대로 속도로 쓰면 안 된다.

    근접 구간에서는 재정렬을 하지 않으므로, 목표가 등 뒤일 때 전진하면
    거리가 늘어난다. 전진축 투영이 그 부호를 만든다.
    """
    _, drive_body = _phase_branch(_drive_to())
    source = "\n".join(_source(node) or "" for node in drive_body)

    assert "KP_LINEAR * dist" not in source


def test_realign_requires_distance_and_larger_yaw_error():
    source = _source(_drive_to())

    assert "dist > REALIGN_MIN_DIST_M" in source
    assert "abs(yaw_err) >= YAW_REALIGN_TRIG_RAD" in source


def test_cancel_success_and_timeout_publish_stop():
    fn = _drive_to()

    terminal_calls = {"canceled", "succeed", "abort"}
    checked = []

    for node in ast.walk(fn):
        if not isinstance(node, ast.Expr) or not isinstance(node.value, ast.Call):
            continue

        call = node.value
        if not isinstance(call.func, ast.Attribute):
            continue
        if call.func.attr not in terminal_calls:
            continue

        checked.append(call.func.attr)

    assert set(checked) == terminal_calls

    source = _source(fn)
    assert source.count("self._cmd_vel_pub.publish(Twist())") >= 4


def test_phase_transitions_are_logged():
    source = _source(_drive_to())

    assert "drive_to phase ALIGN -> DRIVE" in source
    assert "drive_to phase DRIVE -> ALIGN" in source
