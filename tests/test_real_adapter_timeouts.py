"""real 어댑터의 대기 상한 정적 검사 (이슈 #123).

domain/adapters/real/* 는 rclpy·grippers_interfaces 가 있어야 import 되므로 로컬
CI에서는 **실행**할 수 없다. 대신 소스를 AST로 읽어 "인자 없는 대기가 하나도
남지 않았는지"를 검사한다 — 이 파일이 막는 건 서비스·액션 서버가 안 떠 있을 때
FSM 스레드가 영원히 블록되는 회귀다.

정적 검사라 계약의 '값'까지는 못 본다. 실기 검증은 별도다."""

import ast
import pathlib

REAL_DIR = pathlib.Path(__file__).resolve().parent.parent / "domain" / "adapters" / "real"
ADAPTER_FILES = sorted(p for p in REAL_DIR.glob("ros2_*.py"))
CALL_HELPER = REAL_DIR / "_ros_call.py"

# 무한 대기를 만들 수 있는 호출 — 전부 timeout_sec 를 받아야 한다.
BLOCKING_CALLS = {"wait_for_service", "wait_for_server", "spin_until_future_complete"}

# 응답을 기다리지 않는 것이 계약인 E-STOP 경로. 여기만 헬퍼를 거치지 않고
# call_async 를 직접 부른다 (baseline_mission.BaselineEstopState).
ESTOP_METHODS = {"stop", "hold_position"}


def _parse(path):
    return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def _called_name(node):
    """Call 노드에서 호출되는 이름을 뽑는다 — a.b.c() 면 'c', f() 면 'f'."""
    func = node.func
    if isinstance(func, ast.Attribute):
        return func.attr
    if isinstance(func, ast.Name):
        return func.id
    return None


def _all_calls(tree):
    return [n for n in ast.walk(tree) if isinstance(n, ast.Call)]


def _keywords(node):
    return {kw.arg for kw in node.keywords}


def test_adapter_files_are_all_covered():
    """검사 대상이 조용히 비어 버리면 이 파일 전체가 무의미해진다."""
    names = {p.name for p in ADAPTER_FILES}
    assert names == {
        "ros2_arm_driver.py",
        "ros2_command_interpreter.py",
        "ros2_lidar.py",
        "ros2_mecanum_base.py",
        "ros2_perception.py",
    }
    assert CALL_HELPER.exists()


def test_no_blocking_call_without_timeout():
    """인자 없는 wait_for_service() / wait_for_server() /
    spin_until_future_complete(node, future) 가 하나도 남아 있지 않다."""
    offenders = []
    for path in [*ADAPTER_FILES, CALL_HELPER]:
        for call in _all_calls(_parse(path)):
            if _called_name(call) in BLOCKING_CALLS and "timeout_sec" not in _keywords(call):
                offenders.append(f"{path.name}:{call.lineno} {_called_name(call)}()")
    assert (
        not offenders
    ), "상한 없는 대기가 남아 있다 — 서버가 없으면 FSM이 영원히 멈춘다:\n" + "\n".join(offenders)


def test_timeout_constants_are_module_level():
    """상한 값은 하드코딩이 아니라 모듈 상수다 — 값을 바꿀 때 한 곳만 고치면 된다."""
    expected = {
        "ESTOP_TIMEOUT_SEC": 0.5,
        "SAFETY_TIMEOUT_SEC": 0.5,
        "ACTION_TIMEOUT_SEC": 5.0,
        "ACTION_RESULT_TIMEOUT_SEC": 60.0,
        "SERVICE_TIMEOUT_SEC": 3.0,
    }
    found = {
        target.id: node.value.value
        for node in _parse(CALL_HELPER).body
        if isinstance(node, ast.Assign)
        for target in node.targets
        if isinstance(target, ast.Name) and isinstance(node.value, ast.Constant)
    }
    assert {k: found.get(k) for k in expected} == expected


def test_monitor_clearance_uses_the_short_safety_timeout():
    """안전 판정만 상한이 짧다 — INSERT 중 반복 호출되므로 일반 서비스와 같은
    3초를 기다리면 베이스가 움직이는 도중 3초간 판단이 멈춘다."""
    tree = _parse(REAL_DIR / "ros2_perception.py")
    fn = next(
        n
        for n in ast.walk(tree)
        if isinstance(n, ast.FunctionDef) and n.name == "monitor_clearance"
    )
    timeouts = [
        kw.value.id
        for call in _all_calls(fn)
        for kw in call.keywords
        if kw.arg == "timeout_sec" and isinstance(kw.value, ast.Name)
    ]
    assert timeouts == ["SAFETY_TIMEOUT_SEC"]


def test_only_estop_paths_call_the_client_directly():
    """일반 경로는 전부 _ros_call 헬퍼를 거친다 — 어댑터에서 직접 call_async 를
    부르면 그 자리만 상한·경고 로그 없이 남는다."""
    offenders = []
    for path in ADAPTER_FILES:
        for fn in ast.walk(_parse(path)):
            if not isinstance(fn, ast.FunctionDef) or fn.name in ESTOP_METHODS:
                continue
            if any(_called_name(c) == "call_async" for c in _all_calls(fn)):
                offenders.append(f"{path.name}:{fn.name}()")
    assert not offenders, "헬퍼를 거치지 않은 직접 호출:\n" + "\n".join(offenders)


def test_every_failure_return_is_logged():
    """모든 타임아웃 지점이 경고를 남긴다 — 어느 서비스가 응답하지 않았는지
    알 수 없으면 실기 디버깅이 불가능하다."""
    unlogged = []
    for fn in ast.walk(_parse(CALL_HELPER)):
        if not isinstance(fn, ast.FunctionDef):
            continue
        for block in [n for n in ast.walk(fn) if isinstance(getattr(n, "body", None), list)]:
            for index, stmt in enumerate(block.body):
                is_bare_return = (
                    isinstance(stmt, ast.Return)
                    and isinstance(stmt.value, ast.Constant)
                    and stmt.value.value is None
                )
                if not is_bare_return:
                    continue
                logged = any(
                    _called_name(call) in {"warn", "error"}
                    for earlier in block.body[:index]
                    for call in _all_calls(earlier)
                )
                if not logged:
                    unlogged.append(f"{CALL_HELPER.name}:{stmt.lineno}")
    assert not unlogged, "실패 반환 앞에 경고 로그가 없다:\n" + "\n".join(unlogged)
