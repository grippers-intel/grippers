"""같은 물리량의 사본들이 갈라지지 않게 막는다.

## 왜 사본이 있는가

없앨 수 없는 것들이다. 도메인 계층은 ROS 패키지를 import하지 않고
(`floor_grasp_policy.py`의 계층 분리 주석), `tools/`의 실측 도구들은
rclpy를 쓰므로 도메인 테스트가 그것들을 import할 수 없다. 그래서 같은
숫자를 여러 파일에 손으로 적어 둔다.

## 왜 테스트로 막아야 하는가

**이 저장소가 이미 여러 번 당했다.**

- queen의 K가 `perception_node.py`에서 35.1155로 고쳐졌는데
  `grasp_geometry_calibrate.CURRENT_K` 사본이 안 따라가서, 그 도구가 낸
  "보정값"이 스테일 28.3382 기준으로 계산됐다(30.94 — 실제로 써야 할
  38.3357이 아니었다).
- 턱 선이 `baseline_constants` / `grasp_geometry_calibrate` /
  `test_proximity_gate` 세 곳에서 각각 따로 낡아 있었다.
- 2026-08-28에 또 발견: `JAW_LINE_FOR_HINT`에 `star`가 통째로 빠져 있었다.
  star의 턱 선은 2026-08-27에 실측했는데 이 사본에만 안 들어갔다.

주석으로 "두 파일을 항상 같이 고칠 것"이라고 적어 두는 것만으로는 세 번
연속 못 막았다. 그래서 실행되는 검사로 바꾼다.

## 왜 AST로 읽는가

`perception_node.py`는 rclpy를, `tools/*.py`는 rclpy와 하드웨어 드라이버를
import하므로 이 스위트에서 **import할 수 없다.** 소스를 파싱해 모듈 레벨
리터럴 대입만 꺼내 쓴다 — `test_mission_observability_contract.py`가
`LoggedPort` 인자 순서를 검사하는 것과 같은 방식이다.
"""

import ast
import pathlib

import pytest

from domain.task import baseline_constants as bc
from domain.task import floor_grasp_policy as policy
from domain.task import motion

ROOT = pathlib.Path(__file__).resolve().parent.parent

PERCEPTION_NODE = (ROOT / "ros2_ws" / "src" / "grippers_perception"
                   / "grippers_perception" / "perception_node.py")
PROXIMITY_GATE = (ROOT / "ros2_ws" / "src" / "grippers_perception"
                  / "grippers_perception" / "proximity_gate.py")
FLOOR_PROFILES = (ROOT / "ros2_ws" / "src" / "grippers_arm"
                  / "grippers_arm" / "floor_grasp_profiles.py")
GRIPPER_CALIB = (ROOT / "ros2_ws" / "src" / "grippers_arm"
                 / "grippers_arm" / "gripper_calibration.py")
BASKET_ALIGN = (ROOT / "ros2_ws" / "src" / "grippers_base"
                / "grippers_base" / "basket_lidar_align.py")
MECANUM_BASE = ROOT / "domain" / "adapters" / "real" / "ros2_mecanum_base.py"
CALIBRATE_TOOL = ROOT / "tools" / "grasp_geometry_calibrate.py"
TEST_CONSOLE = ROOT / "tools" / "grasp_test_console.py"


def _literals(path):
    """모듈 레벨 리터럴 대입만 {이름: 값}으로 꺼낸다.

    계산식(`12.0 / 256.0`)이나 다른 이름을 참조하는 대입은 건너뛴다 —
    여기서 대조하는 값은 전부 리터럴이고, 계산식까지 흉내내기 시작하면
    이 헬퍼가 두 번째 인터프리터가 된다.
    """
    tree = ast.parse(path.read_text(encoding="utf-8"))
    out = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if not isinstance(target, ast.Name):
            continue
        try:
            out[target.id] = ast.literal_eval(node.value)
        except (ValueError, SyntaxError):
            continue
    return out


@pytest.fixture(scope="module")
def sources():
    return {
        "perception_node": _literals(PERCEPTION_NODE),
        "proximity_gate": _literals(PROXIMITY_GATE),
        "floor_profiles": _literals(FLOOR_PROFILES),
        "gripper_calib": _literals(GRIPPER_CALIB),
        "basket_align": _literals(BASKET_ALIGN),
        "mecanum_base": _literals(MECANUM_BASE),
        "calibrate_tool": _literals(CALIBRATE_TOOL),
        "test_console": _literals(TEST_CONSOLE),
    }


# --------------------------------------------------------------------------
# 클래스별 실측 표
# --------------------------------------------------------------------------

def test_K표_사본_셋이_모두_같다(sources):
    """거리 보정 K의 권위는 perception_node다 — 실제로 배포되어 도는 값이다.

    나머지 둘은 실측 도구의 참고용 사본인데, 어긋나면 그 도구가 **틀린
    보정값을 계산해 낸다.** 2026-08-26~27에 queen에서 실제로 그랬다.
    """
    authoritative = sources["perception_node"]["CLASS_DISTANCE_CALIBRATION_SQRT_PX_M"]

    assert sources["calibrate_tool"]["CURRENT_K"] == authoritative, (
        "tools/grasp_geometry_calibrate.py의 CURRENT_K가 perception_node와 다르다")
    assert sources["test_console"]["K_CLASS"] == authoritative, (
        "tools/grasp_test_console.py의 K_CLASS가 perception_node와 다르다")


def test_턱선_사본이_baseline과_같다(sources):
    """턱 선의 권위는 baseline_constants다 — GRASP 전진량이 여기서 나온다.

    2026-08-28: 이 검사가 없어서 `JAW_LINE_FOR_HINT`에 star가 빠진 채로
    하루가 지났다.
    """
    assert sources["calibrate_tool"]["JAW_LINE_FOR_HINT"] == bc.JAW_LINE_DEPTH_FORWARD_M, (
        "tools/grasp_geometry_calibrate.py의 JAW_LINE_FOR_HINT가 "
        "baseline_constants.JAW_LINE_DEPTH_FORWARD_M와 다르다")


def test_여섯_클래스가_모든_표에_다_있다(sources):
    """한 클래스가 어느 한 표에서만 빠지는 것이 이 저장소의 실제 실패 양상이다.

    값이 틀린 것보다 **없는 것**이 더 조용하다 — `.get()`이 None을 돌려주고
    그 자리에서 기능이 꺼질 뿐 아무도 안 죽는다.
    """
    expected = set(bc.JAW_LINE_DEPTH_FORWARD_M)
    assert len(expected) == 6, f"여섯 클래스가 기준이다: {sorted(expected)}"

    tables = {
        "baseline.DEPTH_LATERAL_TO_JAW_CENTER_M": set(bc.DEPTH_LATERAL_TO_JAW_CENTER_M),
        "perception_node.K": set(
            sources["perception_node"]["CLASS_DISTANCE_CALIBRATION_SQRT_PX_M"]),
        "calibrate_tool.CURRENT_K": set(sources["calibrate_tool"]["CURRENT_K"]),
        "calibrate_tool.JAW_LINE_FOR_HINT": set(
            sources["calibrate_tool"]["JAW_LINE_FOR_HINT"]),
        "test_console.K_CLASS": set(sources["test_console"]["K_CLASS"]),
    }
    for name, labels in tables.items():
        assert labels == expected, f"{name}에 {sorted(expected - labels)}가 없다"


def test_MIN_MEASURED_K가_실제_최솟값이다(sources):
    """`proximity_gate.MIN_MEASURED_K_SQRT_PX_M`은 K 표에서 **파생된** 값이다.

    이보다 작은 K를 가진 클래스가 생기면 그 클래스에서 근접 게이트가 늦게
    걸린다. 주석에 "반드시 낮춰야 한다"고 적혀 있지만 코드로는 강제되지
    않았다 — 여기서 강제한다.
    """
    k_table = sources["perception_node"]["CLASS_DISTANCE_CALIBRATION_SQRT_PX_M"]
    assert sources["proximity_gate"]["MIN_MEASURED_K_SQRT_PX_M"] == min(k_table.values()), (
        "proximity_gate.MIN_MEASURED_K_SQRT_PX_M이 K 표의 최솟값이 아니다")


# --------------------------------------------------------------------------
# 스칼라 사본
# --------------------------------------------------------------------------

def test_그리퍼_폭_사본이_같다(sources):
    """열린 그리퍼 최대 폭 168mm가 세 곳에 있다."""
    assert policy.GRIPPER_MAX_SAFE_OPEN_MM == bc.GRIPPER_OPEN_MM
    assert sources["gripper_calib"]["GRIPPER_OPEN_MM"] == bc.GRIPPER_OPEN_MM


def test_파지_전제_배치가_같다(sources):
    """교시 자세가 전제하는 물체 중심 위치.

    ⚠️ Host가 200mm 조준을 구현하면 **두 곳을 같이** 190 -> 200으로 바꿔야
    한다. 한쪽만 바꾸면 팔은 190을 전제하고 계산은 200을 쓴다.
    """
    assert (sources["floor_profiles"]["GRASP_OBJECT_CENTER_FORWARD_MM"]
            == bc.GRASP_OBJECT_CENTER_FORWARD_MM)


def test_그리퍼_여닫이_폭_사본이_같다(sources):
    assert sources["floor_profiles"]["GRIPPER_SQUEEZE_MM"] == policy.GRIPPER_SQUEEZE_MM
    assert sources["floor_profiles"]["GRIPPER_RELEASE_MM"] == policy.GRIPPER_RELEASE_MM


def test_라이다_기하_사본이_같다(sources):
    """도메인이 ROS 패키지를 import하지 않아 생긴 사본 세 쌍."""
    align = sources["basket_align"]
    assert align["LIDAR_HEIGHT_M"] == bc.LIDAR_HEIGHT_M
    assert align["LIDAR_TILT_DEG"] == bc.LIDAR_TILT_DEG
    assert align["BASKET_RIM_HEIGHT_M"] == bc.BASKET_RIM_HEIGHT_M


def test_저속_구간_속도가_같다(sources):
    """바구니 접근 0.06 m/s.

    `motion.BASKET_APPROACH_MPS`는 Host가 보낸 속도를 자르는 상한이고,
    `ros2_mecanum_base.CREEP_SPEED_MPS`는 미세 전진 버스트가 실제로 내는
    속도다. 둘은 **같은 물리적 사실**(데드밴드 0.05 위에서 실제로 도는 최저
    속도)에서 나왔으므로 갈라지면 안 된다.
    """
    assert sources["mecanum_base"]["CREEP_SPEED_MPS"] == motion.BASKET_APPROACH_MPS


def test_bbox_보정값_사본이_같다(sources):
    """검출기 성질이라 클래스와 무관한 두 값."""
    node, gate = sources["perception_node"], sources["proximity_gate"]
    assert gate["BBOX_PADDING_PX"] == node["BBOX_PADDING_PX"]
    assert gate["MIN_BBOX_AREA_PX"] == node["MIN_BBOX_AREA_PX"]
