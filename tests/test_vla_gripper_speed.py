"""VLA 재생 중 그리퍼 닫는 속도를 촬영 조건으로 묶는다 (2026-09-06).

## 왜 필요한가

사용자 관찰: VLA 파지에서 턱이 물체를 치고 앞으로 밀어낸다. 원인은 재생
경로에 그리퍼 속도 상한이 **아예 없었다**는 것이다 — `_execute_joint_chunk`
가 servo 1..6 전부에 `VLA_SPEED_RAW = 0`(무제한)을 걸었다.

관절 1..5 에는 그게 맞다(VLA_SPEED_RAW 주석의 2026-09-02 회귀). 팔은 빈
공간을 지나가므로 속도가 곧 충격이 되지 않는다. 그리퍼는 반대다 — **물체와
부딪히는 것이 일**이라 무제한 속도가 그대로 충격이 된다.

## 값의 출처

리눅스 세션이 학습 데이터(v5_all 118회차)에서 잰 실측이다. 정책 단위(0~100)
를 이 노드의 raw 로 옮기는 환산은 캘리브레이션 파일이 정하므로, 여기서도
그 파일을 읽어 계산한다 — 캘리브레이션이 바뀌면 이 테스트가 같이 따라간다.

    촬영 실제 평균  100.5 unit/s     <- 턱이 실제로 낸 속도
    촬영 실제 최대  141.3 unit/s
"""

import ast
import json
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent
ARM_NODE = ROOT / "ros2_ws" / "src" / "grippers_arm" / "grippers_arm" / "arm_driver_node.py"
CALIBRATION = ROOT / "host" / "vla" / "calibration" / "grippers_arm.json"

#: 촬영 때 팔로워가 **실제로** 낸 닫힘 속도(정책 단위/초). 지령이 아니라
#: state 다 — 우리가 재현해야 하는 것은 지령이 아니라 물리적 결과다.
DEMO_ACTUAL_MEAN_UNITS_S = 100.5
DEMO_ACTUAL_PEAK_UNITS_S = 141.3

#: 닫힘 주 구간(지령 60 -> 20). 정책 단위.
DEMO_CLOSING_TRAVEL_UNITS = 40.0

#: 청크 하나의 길이(초). 100스텝 / 30fps.
CHUNK_SEC = 100.0 / 30.0


def _tree():
    return ast.parse(ARM_NODE.read_text(encoding="utf-8"), filename=str(ARM_NODE))


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


@pytest.fixture(scope="module")
def raw_per_unit():
    """정책 1 unit 이 몇 raw 인가 — RANGE_0_100 이 캘리브레이션 범위에 걸린다."""
    grip = json.loads(CALIBRATION.read_text(encoding="utf-8"))["gripper"]
    span = grip["range_max"] - grip["range_min"]
    assert span > 0, "gripper range 가 비었습니다"
    return span / 100.0


def test_그리퍼는_무제한이_아니다():
    """이 테스트 하나가 2026-09-06 고장 전체다 — 0 이면 상한이 없다."""
    assert _constants({"VLA_GRIPPER_SPEED_RAW"})["VLA_GRIPPER_SPEED_RAW"] > 0


def test_관절_1에서_5는_여전히_무제한이다():
    """그리퍼를 묶는 김에 팔까지 묶으면 2026-09-02 회귀가 돌아온다.

    그때 align_to_idle 이 남긴 150 raw/s 때문에 정책이 shoulder_lift 를
    50도로 부르는데 팔은 12.1도/s 로만 갔다."""
    assert _constants({"VLA_SPEED_RAW"})["VLA_SPEED_RAW"] == 0


def test_촬영_때_턱이_낸_속도_범위_안에_있다(raw_per_unit):
    """평균보다는 빠르고 최대보다는 느려야 촬영 조건의 재현이다.

    평균 아래로 내리면 정책이 의도한 것보다 느려져 팔이 먼저 들리고,
    최대 위로 올리면 애초에 고치려던 충격이 남는다."""
    speed = _constants({"VLA_GRIPPER_SPEED_RAW"})["VLA_GRIPPER_SPEED_RAW"]
    floor = DEMO_ACTUAL_MEAN_UNITS_S * raw_per_unit
    ceiling = DEMO_ACTUAL_PEAK_UNITS_S * raw_per_unit
    assert floor <= speed <= ceiling, (
        f"{speed} raw/s 는 촬영 실측 {floor:.0f}~{ceiling:.0f} raw/s 밖입니다"
    )


def test_닫힘_행정이_청크_안에서_끝난다(raw_per_unit):
    """속도를 묶었으니 늦어진다 — 청크를 넘기면 팔이 먼저 움직인다."""
    speed = _constants({"VLA_GRIPPER_SPEED_RAW"})["VLA_GRIPPER_SPEED_RAW"]
    travel_raw = DEMO_CLOSING_TRAVEL_UNITS * raw_per_unit
    assert travel_raw / speed < CHUNK_SEC / 4.0


def test_그리퍼_속도는_전체_루프_뒤에_쓴다():
    """⚠️ 순서가 전부다.

    `_execute_joint_chunk` 는 ALL_SERVO_IDS(1..6) 를 돌며 VLA_SPEED_RAW 를
    건다. servo 6 지정을 그 **앞**에 두면 루프가 0 으로 덮어써서, 코드는
    멀쩡해 보이는데 상한이 사라진다 — 로그에도 안 남는 종류의 고장이다."""
    fn = _function("_execute_joint_chunk")

    loop_lines = [
        node.lineno
        for node in ast.walk(fn)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "set_speed"
        and any(isinstance(a, ast.Name) and a.id == "VLA_SPEED_RAW" for a in node.args)
    ]
    grip_lines = [
        node.lineno
        for node in ast.walk(fn)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "set_speed"
        and any(isinstance(a, ast.Name) and a.id == "GRIPPER_SERVO_ID" for a in node.args)
    ]

    assert loop_lines, "전체 서보 속도 설정이 사라졌습니다"
    assert grip_lines, "servo 6 속도 지정이 사라졌습니다"
    assert min(grip_lines) > max(loop_lines), (
        "servo 6 속도를 전체 루프보다 먼저 쓰면 루프가 무제한으로 덮어씁니다"
    )


def test_0을_주면_예전_동작으로_돌아갈_수_있다():
    """실기에서 원인을 가르려면 되돌릴 수 있어야 한다 — A/B 없이는 못 고친다."""
    fn = _function("_execute_joint_chunk")
    guards = [
        node
        for node in ast.walk(fn)
        if isinstance(node, ast.If)
        and isinstance(node.test, ast.Compare)
        and isinstance(node.test.left, ast.Name)
        and node.test.left.id == "grip_speed"
    ]
    assert guards, "grip_speed > 0 가드가 없으면 0 이 그대로 서보에 써집니다"


def test_런타임에_바꿀_수_있다():
    """값이 실기에서 정해질 성질이라 재빌드 없이 흔들어 봐야 한다."""
    init = _function("__init__")
    declared = {
        node.args[0].value
        for node in ast.walk(init)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "declare_parameter"
        and node.args
        and isinstance(node.args[0], ast.Constant)
    }
    assert "vla_gripper_speed_raw" in declared
