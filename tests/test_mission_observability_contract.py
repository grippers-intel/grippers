"""Issue #46: executable observability and fake/real wiring contracts."""

import ast
import pathlib

import pytest

from domain.adapters.logged_port import LoggedPort

ROOT = pathlib.Path(__file__).resolve().parent.parent
ORCHESTRATOR = (
    ROOT
    / "ros2_ws"
    / "src"
    / "grippers_mission"
    / "grippers_mission"
    / "mission_orchestrator_node.py"
)
BRINGUP = ROOT / "ros2_ws" / "src" / "grippers_bringup" / "launch" / "bringup.launch.py"


class RecordingLogger:
    def __init__(self):
        self.info_messages = []
        self.error_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def error(self, message):
        self.error_messages.append(message)


class ExamplePort:
    def succeed(self, value, *, scale=1):
        return value * scale

    def fail(self):
        raise RuntimeError("injected failure")


def _parse(path):
    return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def _function(path, name):
    return next(
        node
        for node in ast.walk(_parse(path))
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def _called_names(function):
    return [
        call.func.id if isinstance(call.func, ast.Name) else call.func.attr
        for call in ast.walk(function)
        if isinstance(call, ast.Call)
    ]


def test_logged_port_records_call_and_return_without_changing_result():
    logger = RecordingLogger()
    port = LoggedPort("ExamplePort", ExamplePort(), logger)

    assert port.succeed(3, scale=2) == 6
    assert "[PORT] CALL ExamplePort.succeed" in logger.info_messages[0]
    assert "[PORT] RETURN ExamplePort.succeed result=6" == logger.info_messages[1]


def test_logged_port_records_exception_and_reraises_it():
    logger = RecordingLogger()
    port = LoggedPort("ExamplePort", ExamplePort(), logger)

    with pytest.raises(RuntimeError, match="injected failure"):
        port.fail()

    assert "[PORT] ERROR ExamplePort.fail" in logger.error_messages[0]


def test_hardware_ports_are_wrapped_for_boundary_logging():
    """경계 로깅이 빠지면 실기에서 어느 포트가 무엇을 돌려줬는지 못 본다.

    Host 링크와 라이다는 감싸지 않는다 — 전자는 사이클마다 도는 순수
    입출력이라 로그가 폭주하고, 후자는 판정 결과를 그대로 Host에 보고하므로
    이미 기록이 남는다."""
    source = ORCHESTRATOR.read_text(encoding="utf-8")

    assert source.count("LoggedPort(") == 3
    for port in ("Ros2MecanumBase", "Ros2ArmDriver", "Ros2Perception"):
        assert port in source


def test_logged_port_calls_pass_the_name_first():
    """2026-08-27 실기 크래시의 재발 방지.

    LoggedPort.__init__(self, name, delegate, logger)인데, 이 파일의 세
    호출이 (delegate, "이름", logger) 순서로 뒤바뀌어 있었다 — 그러면
    `self._delegate`에 문자열이 들어가 `ports.base.stop()` 같은 모든 실호출이
    `'str' object has no attribute 'stop'`으로 죽는다. 도메인 pytest 스위트는
    ROS 전용인 이 파일을 안 건드리므로 실기에서만 터졌었다. AST로 첫 위치
    인자가 항상 문자열 리터럴인지 확인해 순서가 다시 안 바뀌게 막는다."""
    tree = _parse(ORCHESTRATOR)
    calls = [
        call for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name) and call.func.id == "LoggedPort"
    ]
    assert len(calls) == 3
    for call in calls:
        first_arg = call.args[0]
        assert isinstance(first_arg, ast.Constant) and isinstance(first_arg.value, str), (
            "LoggedPort의 첫 인자는 이름 문자열이어야 한다 — "
            f"실제로는 {ast.dump(first_arg)}")


def test_state_is_published_every_cycle():
    """`/mission/state`는 아레나 오버레이와 디버깅이 보는 유일한 창이다."""
    run_forever = _function(ORCHESTRATOR, "_run_forever")

    assert "_publish_state" in _called_names(run_forever)
    assert '"/mission/state"' in ORCHESTRATOR.read_text(encoding="utf-8")


def test_orchestrator_holds_the_arm_when_the_fsm_throws():
    """예외로 FSM이 죽을 때 팔을 놓으면 파지물이 떨어진다."""
    run_forever = _function(ORCHESTRATOR, "_run_forever")
    names = _called_names(run_forever)

    assert "stop" in names
    assert "hold_position" in names


def test_launch_exposes_the_fake_switches_and_optional_rosbag():
    source = BRINGUP.read_text(encoding="utf-8")

    for name in (
        "use_fake_base",
        "use_fake_arm",
        "use_fake_perception",
        "use_fake_interpreter",
    ):
        assert f'LaunchConfiguration("{name}")' in source
        assert f'"{name}",' in source

    assert 'LaunchConfiguration("record_bag")' in source
    assert '["ros2", "bag", "record", "-a", "-o", bag_output]' in source
    # depth_camera_launch, lidar_launch, perception_node, depth_cam_rotate_node
    # (2026-08-27: 이 launch에서 빠져 있어 매번 손으로 따로 띄워야 했다 —
    # perception_node는 회전 보정된 스트림만 구독하므로 없으면 뒤집힌
    # 프레임에서 YOLO가 매 프레임 오검출을 낸다).
    assert "depth_cam_rotate_node" in source
    assert source.count("UnlessCondition(use_fake_perception)") == 4
