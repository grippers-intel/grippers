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


def _assigned_attributes(function):
    return {
        target.attr
        for node in ast.walk(function)
        if isinstance(node, (ast.Assign, ast.AnnAssign))
        for target in (node.targets if isinstance(node, ast.Assign) else [node.target])
        if isinstance(target, ast.Attribute)
    }


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


def test_all_four_mission_ports_are_wrapped_for_boundary_logging():
    run_fsm = _function(ORCHESTRATOR, "_run_fsm")
    names = _called_names(run_fsm)

    assert names.count("_logged") == 4


def test_state_is_published_in_normal_and_abort_paths():
    run_fsm = _function(ORCHESTRATOR, "_run_fsm")
    abort = _function(ORCHESTRATOR, "_abort_mission")

    assert "publish" in _called_names(run_fsm)
    assert "publish" in _called_names(abort)
    assert '"/mission/state"' in ORCHESTRATOR.read_text(encoding="utf-8")


def test_elapsed_time_is_set_once_for_normal_and_abort_state_messages():
    """elapsed_s는 공통 메시지 생성기에서 monotonic 경과 시간으로 채운다."""
    message_builder = _function(ORCHESTRATOR, "_mission_state_message")
    run_fsm = _function(ORCHESTRATOR, "_run_fsm")
    abort = _function(ORCHESTRATOR, "_abort_mission")

    assert "elapsed_s" in _assigned_attributes(message_builder)
    assert "monotonic" in _called_names(message_builder)
    assert "monotonic" in _called_names(run_fsm)
    assert "_mission_state_message" in _called_names(run_fsm)
    assert "_mission_state_message" in _called_names(abort)


def test_launch_exposes_four_fake_switches_and_optional_rosbag():
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
    assert source.count("UnlessCondition(use_fake_base)") == 2
    assert source.count("UnlessCondition(use_fake_perception)") == 3
