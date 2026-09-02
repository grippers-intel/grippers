"""팔에 실린 캘리브레이션이 교시 자세와 맞는지 (2026-08-30).

## 왜 이 검사가 필요한가

교시 자세는 RAW 서보값이고, RAW 값이 가리키는 물리 자세는 서보 EEPROM 의
`Homing_Offset` 에 달려 있다. **오프셋은 서보 안에 있지 git 에 있지 않다** —
브랜치를 바꿔도 팔은 안 바뀐다.

2026-08-29 에 VLA 시연 수집을 준비하며 LeRobot 캘리브레이션이 그 오프셋을
덮어썼다. 그 뒤로 "코드는 베이스라인인데 팔은 VLA 캘리브레이션"인 조합이
아무 경고 없이 만들어질 수 있다. shoulder_pan 가동폭이 2493 -> 2087 로
줄어 있어(차체·라이다에 막힘) 어긋난 채 움직이면 부딪힌다.

여기서 고정하는 것은 **모르는 것을 같다고 하지 않는다**와 **다르면
기동하지 않는다** 둘이다.
"""

import ast
import importlib.util
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
GRIPPERS_ARM_SRC = ROOT / "ros2_ws" / "src" / "grippers_arm"
NODE = GRIPPERS_ARM_SRC / "grippers_arm" / "arm_driver_node.py"
BACKUP = ROOT / "tools" / "arm" / "servo_backup" / "servo_COM8_20260829_181124.json"

if str(GRIPPERS_ARM_SRC) not in sys.path:
    sys.path.insert(0, str(GRIPPERS_ARM_SRC))


def _load(name):
    path = GRIPPERS_ARM_SRC / "grippers_arm" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ci = _load("calib_identity")
TAUGHT = {1: -1945, 2: -1762, 3: 1307, 4: 1760, 5: -1848, 6: 1343}


# ── 판정 ───────────────────────────────────────────────────────────────────


def test_같으면_통과한다():
    assert ci.verdict(dict(TAUGHT), TAUGHT).ok


def test_한_관절만_달라도_거부한다():
    """오프셋 하나가 바뀌면 그 관절의 모든 교시 자세가 어긋난다."""
    current = dict(TAUGHT)
    current[1] += 1

    result = ci.verdict(current, TAUGHT)

    assert not result.ok
    assert result.state == ci.MISMATCH
    assert set(result.differences) == {1}


def test_못_읽은_것을_같다고_하지_않는다():
    """읽기 실패는 '문제 없음'이 아니다. 여기서 통과시키면 통신이 불안한
    날에 검사가 통째로 무력해진다."""
    current = dict(TAUGHT)
    current[3] = None

    result = ci.verdict(current, TAUGHT)

    assert not result.ok
    assert result.state == ci.UNREADABLE
    assert result.unreadable == [3]


def test_결측이_불일치보다_먼저다():
    """둘 다 있으면 '못 읽었다'를 말해야 한다 — 못 읽은 관절이 실제로
    어떤지 모르는 채로 차이 목록을 내밀면 사람이 그 목록만 고친다."""
    current = dict(TAUGHT)
    current[1] += 500
    current[3] = None

    assert ci.verdict(current, TAUGHT).state == ci.UNREADABLE


def test_교시에_없는_관절은_보지_않는다():
    """교시 자세가 안 걸린 관절까지 기동을 막을 이유가 없다."""
    current = dict(TAUGHT)
    current[7] = 999

    assert ci.verdict(current, TAUGHT).ok


def test_기본_허용치는_0이다():
    """오프셋은 EEPROM 정수라 저절로 흔들리지 않는다. 1카운트라도 다르면
    누군가 캘리브레이션을 다시 돌린 것이다."""
    assert ci.TOLERANCE_DEFAULT == 0


def test_read_offsets_도_재시도한다():
    """노드 밖 도구(restore_taught_offsets.py)도 같은 버스를 쓴다."""

    class Flaky:
        def __init__(self):
            self.calls = 0

        def get_homing_offset(self, sid):
            self.calls += 1
            return None if self.calls == 1 else TAUGHT[sid]

    drv = Flaky()
    out = ci.read_offsets(drv, [1, 2], attempts=3)

    assert out == {1: TAUGHT[1], 2: TAUGHT[2]}, "첫 유실을 재시도로 넘겨야 한다"


def test_끝까지_못_읽으면_None_이다():
    """재시도해도 안 되면 그건 진짜 모르는 것이다 — 지어내지 않는다."""

    class Dead:
        def get_homing_offset(self, sid):
            return None

    assert ci.read_offsets(Dead(), [1], attempts=2) == {1: None}


def test_재시도_횟수가_노드와_같다():
    node = NODE.read_text(encoding="utf-8")

    assert f"JOINT_READ_ATTEMPTS = {ci.READ_ATTEMPTS_DEFAULT}" in node


# ── 사람이 읽는 메시지 ─────────────────────────────────────────────────────


def test_얼마나_어긋났는지_각도로_말한다():
    """카운트만 보여주면 이것이 심각한지 사람이 판단할 수 없다."""
    current = dict(TAUGHT)
    current[1] += 100        # 100카운트 = 8.8도

    text = ci.verdict(current, TAUGHT).message()

    assert "+100" in text
    assert "8.8도" in text


def test_되돌리는_명령을_같이_알려준다():
    """이 메시지를 보는 사람은 지금 실기 앞에 서 있다."""
    current = dict(TAUGHT)
    current[2] += 10

    text = ci.verdict(current, TAUGHT).message()

    assert "restore_taught_offsets.py" in text, "파이에서 쓸 수 있는 길이 먼저다"
    assert "backup_servo_offsets.py" in text
    assert "servo_COM8_20260829_181124.json" in text


def test_어느_브랜치로_가야_하는지_알려준다():
    """베이스라인과 VLA 수집이 브랜치로 갈려 있다 — 팔이 VLA
    캘리브레이션이면 베이스라인이 아니라 그쪽으로 가야 한다."""
    current = dict(TAUGHT)
    current[2] += 10

    assert "smolVLA-version" in ci.verdict(current, TAUGHT).message()


# ── 복구 도구 ──────────────────────────────────────────────────────────────


def test_복구_도구가_기준값을_다시_적지_않는다():
    """사본이 늘면 갈라진다. 교시 자세와 오프셋은 한 파일에 있어야 한다."""
    tool = (ROOT / "tools" / "arm" / "restore_taught_offsets.py") \
        .read_text(encoding="utf-8")

    assert "TAUGHT_HOMING_OFFSETS" in tool
    assert "TAUGHT_POSITION_LIMITS" in tool
    assert "-1945" not in tool, "숫자를 여기에 다시 적으면 안 된다"
    assert "1140" not in tool, "각도제한 숫자도 여기에 다시 적으면 안 된다"


def test_복구_도구가_각도제한도_같이_다룬다():
    """2026-09-01: Homing_Offset만 복구하고 각도제한을 안 보던 시절
    그리퍼가 먹통이 됐다 — 이제 이 도구가 둘 다 확인·복구해야 한다."""
    tool = (ROOT / "tools" / "arm" / "restore_taught_offsets.py") \
        .read_text(encoding="utf-8")

    assert "position_limit_registers" in tool
    assert "set_position_limits" in tool


def test_복구_도구가_토크_해제를_승인받는다():
    """EEPROM 쓰기는 토크가 꺼져야 하고, 그러면 팔이 내려온다. 물건을 들고
    있는 채로 실행하면 떨어뜨린다."""
    tool = (ROOT / "tools" / "arm" / "restore_taught_offsets.py") \
        .read_text(encoding="utf-8")

    assert "--yes" in tool
    assert "if not args.yes:" in tool


def test_복구_도구가_쓴_뒤에_다시_읽는다():
    """EEPROM 쓰기는 조용히 실패할 수 있다. 확인 없이 성공을 알리면
    이 검사가 막으려던 바로 그 상태로 미션을 돌리게 된다."""
    tool = (ROOT / "tools" / "arm" / "restore_taught_offsets.py") \
        .read_text(encoding="utf-8")
    body = tool[tool.index("drv.set_all_torque(False)"):]

    assert "read_offsets" in body


# ── 사본이 원본과 갈라지지 않는지 ──────────────────────────────────────────


def test_교시_오프셋이_백업_파일과_같다():
    """TAUGHT_HOMING_OFFSETS 는 백업 JSON 의 사본이다. 갈라지면 검사가
    엉뚱한 기준으로 기동을 막거나 통과시킨다."""
    profiles = _load("floor_grasp_profiles")
    data = json.loads(BACKUP.read_text(encoding="utf-8"))

    ids = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
           "wrist_flex": 4, "wrist_roll": 5, "gripper": 6}
    expected = {ids[n]: r["Homing_Offset"] for n, r in data["motors"].items()}

    assert profiles.TAUGHT_HOMING_OFFSETS == expected


def test_한_바퀴_카운트가_서보_규격과_같다():
    assert ci.COUNTS_PER_REV == 4096


# ── 노드가 실제로 이것을 쓰는가 ────────────────────────────────────────────


def _node_tree():
    return ast.parse(NODE.read_text(encoding="utf-8"), filename=str(NODE))


def test_기동할_때_검사한다():
    """검사 함수가 있어도 안 부르면 아무 일도 안 일어난다."""
    tree = _node_tree()
    init = next(n for n in ast.walk(tree)
                if isinstance(n, ast.FunctionDef) and n.name == "__init__")

    assert "_check_taught_calibration" in ast.dump(init)


def test_불일치면_기동을_거부한다():
    """경고만 하면 사람이 그 줄을 지나쳐 시연을 시작한다."""
    tree = _node_tree()
    check = next(n for n in ast.walk(tree)
                 if isinstance(n, ast.FunctionDef)
                 and n.name == "_check_taught_calibration")
    raises = [n for n in ast.walk(check) if isinstance(n, ast.Raise)]

    assert raises, "예외를 던져야 한다"
    assert "ArmCalibrationMismatchError" in ast.dump(check)


def test_하드웨어_고장과_다른_예외를_쓴다():
    """팔은 멀쩡하다 — 사람이 할 일이 '고치기'가 아니라 '오프셋 되돌리기'다.
    같은 예외를 쓰면 기존 복구 절차가 엉뚱하게 걸린다."""
    tree = _node_tree()
    names = {n.name for n in ast.walk(tree) if isinstance(n, ast.ClassDef)}

    assert "ArmCalibrationMismatchError" in names
    assert "ArmHardwareUnavailableError" in names


def test_오프셋_읽기에_재시도를_건다():
    """이 버스는 패킷을 이따금 흘린다 — 서보 6개 연속 읽기라 묶음이 깨질
    확률이 쌓인다(arm_driver_node._read_with_retry 주석, 2026-08-25).

    재시도가 없으면 패킷 하나 유실이 그대로 기동 거부가 된다. 그건 이
    검사가 막으려는 위험(어긋난 캘리브레이션)과 아무 상관이 없는 실패이고,
    실기 당일에 원인을 찾느라 시간을 태우게 된다."""
    tree = _node_tree()
    check = next(n for n in ast.walk(tree)
                 if isinstance(n, ast.FunctionDef)
                 and n.name == "_check_taught_calibration")
    body = ast.dump(check)

    assert "_read_with_retry" in body
    assert "get_homing_offset" in body


def test_각도제한도_같이_검사한다():
    """2026-09-01: Homing_Offset만 봐서는 그리퍼 먹통 사고를 못 잡는다 —
    Min/Max_Angle_Limit도 같이 봐야 한다(position_limit_registers.py)."""
    tree = _node_tree()
    check = next(n for n in ast.walk(tree)
                 if isinstance(n, ast.FunctionDef)
                 and n.name == "_check_taught_calibration")
    body = ast.dump(check)

    assert "get_position_limits" in body
    assert "TAUGHT_POSITION_LIMITS" in body


def test_각도제한_불일치도_기동을_거부한다():
    tree = _node_tree()
    check = next(n for n in ast.walk(tree)
                 if isinstance(n, ast.FunctionDef)
                 and n.name == "_check_taught_calibration")
    raises = [n for n in ast.walk(check) if isinstance(n, ast.Raise)]

    # Homing_Offset 불일치용 하나 + min/max 불일치용 둘, 최소 3개는 있어야
    # 각 경로가 다 거부로 이어진다.
    assert len(raises) >= 3


def test_검사를_끌_수_있다():
    """팔을 다시 교시하는 중처럼 자세가 무효인 줄 알고 있을 때가 있다."""
    tree = _node_tree()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and getattr(node.func, "attr", "") == "declare_parameter"
                and node.args
                and getattr(node.args[0], "value", "") == "verify_calibration"):
            assert node.args[1].value is True, "기본값은 켜짐이어야 한다"
            return
    raise AssertionError("verify_calibration 파라미터가 없다")
