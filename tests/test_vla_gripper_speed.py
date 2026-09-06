"""VLA 재생 중 그리퍼 닫는 속도를 촬영 조건으로 묶는다 (2026-09-06).

## 왜 필요한가

사용자 관찰: VLA 파지에서 턱이 물체를 치고 앞으로 밀어낸다. 원인은 재생
경로에 그리퍼 속도 상한이 **아예 없었다**는 것이다 — `_execute_joint_chunk`
가 servo 1..6 전부에 `VLA_SPEED_RAW = 0`(무제한)을 걸었다.

관절 1..5 에는 그게 맞다(VLA_SPEED_RAW 주석의 2026-09-02 회귀). 팔은 빈
공간을 지나가므로 속도가 곧 충격이 되지 않는다. 그리퍼는 반대다 — **물체와
부딪히는 것이 일**이라 무제한 속도가 그대로 충격이 된다.

## 그런데 닫을 때만이다

열기까지 묶으면 반대편에서 같은 사고를 만든다. VLA 시작에서 턱이 다 벌어지기
전에 팔이 내려가면 그 자체로 물체를 건드린다. 촬영 실측에서 열기가 닫기보다
빨랐다는 것이 이 판단의 근거다.

## 값의 출처

리눅스 세션이 학습 데이터(v5_all)에서 잰 실측이다. 정책 단위(0~100)를 이
노드의 raw 로 옮기는 환산은 캘리브레이션 파일이 정하므로 여기서도 그 파일을
읽어 계산한다 — 캘리브레이션이 바뀌면 이 테스트가 같이 따라간다.
"""

import ast
import importlib.util
import json
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent
ARM_PKG = ROOT / "ros2_ws" / "src" / "grippers_arm" / "grippers_arm"
ARM_NODE = ARM_PKG / "arm_driver_node.py"
CALIBRATION = ROOT / "host" / "vla" / "calibration" / "grippers_arm.json"

#: 촬영 때 팔로워가 **실제로** 낸 닫힘 속도(정책 단위/초). 지령이 아니라
#: state 다 — 재현해야 하는 것은 지령이 아니라 물리적 결과다.
DEMO_ACTUAL_MEAN_UNITS_S = 100.5
DEMO_ACTUAL_PEAK_UNITS_S = 141.3

#: 닫힘 주 구간(지령 60 -> 20). 정책 단위.
DEMO_CLOSING_TRAVEL_UNITS = 40.0

#: 청크 하나의 길이(초). 100스텝 / 30fps.
CHUNK_SEC = 100.0 / 30.0


def _load(name):
    """rclpy 없이 import 되는 패키지 내부 모듈을 그대로 불러온다."""
    spec = importlib.util.spec_from_file_location(name, ARM_PKG / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gm = _load("gripper_motion")


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


# ── 규칙 자체를 실행해서 본다 ──────────────────────────────────────────────
#
# ⚠️ 여기부터는 AST 가 아니라 **진짜 호출**이다. 이 저장소는 도메인 테스트
# 579개가 전부 통과하는 채로 병합 누락 6건을 놓친 적이 있다 — 코드의 존재를
# 보는 검사와 결과를 보는 검사는 다른 물건이다.

CLOSE, OPEN, LIMIT = -50, +50, 600
UNL = 0          # 무제한 = 지금 서보에 걸린 값이 0 이라는 뜻


def test_닫기_시작에_상한을_건다():
    assert gm.gripper_speed_change(CLOSE, LIMIT, current_speed=UNL) == LIMIT


def test_열기_시작에_무제한으로_되돌린다():
    """기본은 여는 쪽 무제한 — 턱이 다 벌어지기 전에 팔이 내려가면 안 된다."""
    assert gm.gripper_speed_change(OPEN, LIMIT, current_speed=LIMIT) == gm.UNLIMITED


def test_여는_상한을_주면_그_값을_건다():
    """2026-09-06 2차 — 여는 쪽도 손잡이가 필요해졌다."""
    assert gm.gripper_speed_change(OPEN, LIMIT, current_speed=LIMIT, open_speed=400) == 400


def test_여는_상한이_이미_걸려_있으면_안_쓴다():
    assert gm.gripper_speed_change(OPEN, LIMIT, current_speed=400, open_speed=400) is None


def test_같은_방향이_이어지면_아무것도_안_쓴다():
    """방향이 바뀔 때만 써야 한다 — 매 스텝 쓰면 30Hz 재생에서 시리얼이 붐빈다."""
    assert gm.gripper_speed_change(CLOSE, LIMIT, current_speed=LIMIT) is None
    assert gm.gripper_speed_change(OPEN, LIMIT, current_speed=UNL) is None


@pytest.mark.parametrize("move", [0, 1, -1, 3, -3])
def test_데드밴드_안에서는_방향을_안_바꾼다(move):
    """지령이 멈춘 구간에서 부호가 잡음으로 흔들려도 넘어가면 안 된다."""
    assert gm.gripper_speed_change(move, LIMIT, current_speed=UNL) is None
    assert gm.gripper_speed_change(move, LIMIT, current_speed=LIMIT) is None


def test_데드밴드는_정상_이동보다_한참_작다():
    """3 raw 가 정상 프레임 이동(15~25 raw)을 삼키면 방향 전환을 놓친다."""
    assert 0 < gm.DIRECTION_DEADBAND_RAW < 15


def test_0을_주면_아무것도_안_한다():
    """되돌릴 수 있어야 A/B 로 원인을 가린다."""
    assert gm.gripper_speed_change(CLOSE, 0, current_speed=UNL) is None
    assert gm.gripper_speed_change(OPEN, 0, current_speed=UNL) is None


def test_한_번_닫고_열고_닫는_동안_두_번만_바뀐다():
    """실제 파지 한 번의 순서를 그대로 흘려 본다.

    시작은 무제한(재생 시작 루프가 servo 6 에도 0 을 걸어 둔다)이고,
    열림 -> 닫힘 -> 열림(놓기) 순으로 간다."""
    moves = [+40] * 5 + [0, 1, -2] + [-40] * 5 + [0] * 3 + [+40] * 5
    now, writes = UNL, []
    for move in moves:
        new = gm.gripper_speed_change(move, LIMIT, now)
        if new is not None:
            writes.append(new)
            now = new

    assert writes == [LIMIT, gm.UNLIMITED], f"쓰기가 {writes} 입니다"


# ── 값과 배선 ──────────────────────────────────────────────────────────────


def test_그리퍼는_무제한이_아니다():
    """이 테스트 하나가 2026-09-06 고장 전체다 — 0 이면 상한이 없다."""
    assert _constants({"VLA_GRIPPER_SPEED_RAW"})["VLA_GRIPPER_SPEED_RAW"] > 0


def test_관절_1에서_5는_여전히_무제한이다():
    """그리퍼를 묶는 김에 팔까지 묶으면 2026-09-02 회귀가 돌아온다.

    그때 align_to_idle 이 남긴 150 raw/s 때문에 정책이 shoulder_lift 를
    50도로 부르는데 팔은 12.1도/s 로만 갔다."""
    assert _constants({"VLA_SPEED_RAW"})["VLA_SPEED_RAW"] == 0
    assert gm.UNLIMITED == 0


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


def test_재생_루프가_그_규칙을_실제로_부른다():
    """모듈만 맞고 노드가 안 부르면 아무 일도 안 일어난다."""
    fn = _function("_execute_joint_chunk")
    called = {
        node.func.id
        for node in ast.walk(fn)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    assert "gripper_speed_change" in called


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


# ── 플래그와 레지스터가 어긋나지 않는다 ────────────────────────────────────
#
# `Goal_Velocity` 는 RAM 레지스터라 서보 전원이 살아 있으면 노드를 재시작해도
# 값이 남는다. 그래서 "지금 상한이 걸려 있는가"를 노드가 플래그로만 들고 있으면
# 재시작 뒤 플래그(False)와 레지스터(600)가 어긋날 수 있다 — 그러면 열기가
# 계속 묶인 채로 돌아, 고치려던 증상이 반대편에서 되살아난다.
#
# 지금 코드는 그 창이 없다. 재생 시작 루프가 **청크마다** ALL_SERVO_IDS(1..6)
# 전부에 무제한을 다시 쓰고, `grip_limited = False` 는 그 **뒤에** 온다. 즉
# 모든 청크가 알려진 상태에서 출발한다.
#
# 아래 두 시험이 그 가정을 못 박는다. 없으면 "그리퍼는 어차피 아래에서 관리하니
# 이 루프에서 빼자"는 최적화 한 번으로 조용히 깨진다.


def test_재생_시작마다_그리퍼도_무제한으로_되돌린다():
    """servo 6 이 리셋 루프에서 빠지면 재시작 뒤 열기가 묶인 채로 돈다."""
    fn = _function("_execute_joint_chunk")

    resets = [
        node
        for node in ast.walk(fn)
        if isinstance(node, ast.For)
        and ast.unparse(node.iter) == "ALL_SERVO_IDS"
        and any(
            isinstance(c, ast.Call)
            and isinstance(c.func, ast.Attribute)
            and c.func.attr == "set_speed"
            and any(isinstance(a, ast.Name) and a.id == "VLA_SPEED_RAW" for a in c.args)
            for c in ast.walk(node)
        )
    ]
    assert resets, (
        "청크 시작에서 ALL_SERVO_IDS 전부를 VLA_SPEED_RAW 로 되돌리는 루프가 "
        "없습니다 — servo 6 의 레지스터 잔류가 다음 재생으로 새어 나갑니다"
    )

    # ALL_SERVO_IDS 가 그리퍼를 포함해야 위 루프가 의미를 갖는다.
    # `range(1, 7)` 은 literal_eval 로 못 읽으니 소스를 평가해서 본다.
    src = next(
        ast.unparse(n.value) for n in _tree().body
        if isinstance(n, ast.Assign)
        and len(n.targets) == 1
        and ast.unparse(n.targets[0]) == "ALL_SERVO_IDS"
    )
    assert src.startswith("range("), f"예상 못 한 형태입니다: {src}"
    grip = _constants({"GRIPPER_SERVO_ID"})["GRIPPER_SERVO_ID"]
    assert grip in eval(src, {"__builtins__": {"range": range}})


def test_추적값_초기화는_리셋_뒤에_온다():
    """순서가 뒤집히면 추적값이 레지스터보다 앞서 거짓말을 한다."""
    fn = _function("_execute_joint_chunk")

    reset_line = max(
        node.lineno
        for node in ast.walk(fn)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "set_speed"
        and any(isinstance(a, ast.Name) and a.id == "VLA_SPEED_RAW" for a in node.args)
    )
    # 추적 변수는 "지금 서보에 걸린 값" 이고, 리셋 루프가 쓴 값과 같아야 한다.
    init_line = min(
        node.lineno
        for node in ast.walk(fn)
        if isinstance(node, ast.Assign)
        and ast.unparse(node.targets[0]) == "grip_speed_now"
        and ast.unparse(node.value) == "VLA_SPEED_RAW"
    )
    assert init_line > reset_line


def test_어긋나면_어떻게_되는지를_규칙_자체로_보인다():
    """왜 위 두 시험이 필요한가 — 어긋난 상태를 실제로 흘려 본다.

    레지스터에는 상한이 남아 있는데 플래그만 False 인 상태를 흉내 내면,
    열기 지령이 와도 규칙은 **아무것도 안 쓴다.** 규칙의 결함이 아니라
    입력이 거짓이라 그런 것이고, 그래서 시작 리셋이 규칙의 전제다."""
    assert gm.gripper_speed_change(OPEN, LIMIT, current_speed=UNL) is None
