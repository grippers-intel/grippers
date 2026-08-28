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


def test_terminal_state_reason_is_logged_once(monkeypatch):
    """`MissionState.msg` 는 상태 이름만 싣는다 — 원인은 로그에만 남는다.

    `PerceptionFailedState.reason` 이 객체에만 있고 아무 데도 안 찍히면 실기에서
    "PERCEPTION_FAILED 인데 왜?" 를 알 수 없다 (이슈 #194). 오케스트레이터는
    rclpy 없이 import 할 수 없으므로 AST 로 그 한 줄이 있는지 본다."""
    source = ORCHESTRATOR.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(ORCHESTRATOR))

    reason_reads = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "getattr"
        and len(node.args) >= 2
        and isinstance(node.args[1], ast.Constant)
        and node.args[1].value == "reason"
    ]
    assert reason_reads, "상태의 reason 을 읽는 곳이 없다"
    assert 'f"[MISSION] {state.name} 사유: {reason}"' in source
