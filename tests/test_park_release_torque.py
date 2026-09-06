"""인수인계 전 토크 해제의 안전 관문 (2026-09-07).

토크를 푸는 것 자체는 한 줄이지만, **어디서 푸는지**가 안전을 가른다.
servo 2-5 는 중력 부하가 있는 관절이라 IDLE 크래들이 아닌 자세에서 풀면
팔이 자체 무게로 쓰러진다(reteach_idle_pose.py 상단 경고와 같은 이유).

그래서 도구가 먼저 IDLE 과의 차이를 재고 넘으면 아무것도 안 한다. 이
파일은 그 판정만 시험한다 — 실제 시리얼 통신은 여기 없다.
"""

import importlib.util
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
_spec = importlib.util.spec_from_file_location(
    "park_release_torque", ROOT / "tools" / "arm" / "park_release_torque.py")
park = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(park)

IDLE = park.IDLE_CRADLE_RAW


def _at_idle(**delta):
    """IDLE 자세에서 몇 관절만 어긋난 위치 표."""
    pos = {sid: IDLE[sid - 1] for sid in range(1, 6)}
    pos[6] = 1112                       # 그리퍼는 IDLE 판정에 안 들어간다
    for name, off in delta.items():
        sid = int(name[1:])
        pos[sid] = None if off is None else IDLE[sid - 1] + off
    return pos


# ── 차이 계산 ──────────────────────────────────────────────────────────────


def test_IDLE_에_있으면_차이가_0이다():
    assert park.idle_offsets(_at_idle()) == {1: 0, 2: 0, 3: 0, 4: 0, 5: 0}


def test_그리퍼는_IDLE_판정에_안_들어간다():
    """IDLE_CRADLE_RAW 는 servo 1-5 뿐이다 — 그리퍼 목표는 mm 보정표에서
    따로 나온다(floor_grasp_profiles 주석)."""
    assert 6 not in park.idle_offsets(_at_idle())


def test_못_읽은_관절은_None으로_남는다():
    assert park.idle_offsets(_at_idle(s3=None))[3] is None


# ── 안전 관문 ──────────────────────────────────────────────────────────────


def test_실측_인수인계_자세는_통과한다():
    """2026-09-07 실측: fold_to_cradle 직후 [2060, 843, 3097, 2750, 3069].
    이 자세에서 풀었고 문제가 없었다 — 통과하지 못하면 도구가 무용지물이다."""
    measured = {1: 2060, 2: 843, 3: 3097, 4: 2750, 5: 3069, 6: 1112}
    assert not park.too_far_from_idle(park.idle_offsets(measured))


def test_크게_어긋난_관절이_있으면_막는다():
    """⚠️ 이 관문이 없으면 팔이 쓰러진다."""
    bad = park.too_far_from_idle(park.idle_offsets(_at_idle(s2=400)))
    assert set(bad) == {2}


def test_못_읽은_관절도_막는다():
    """모르는 것을 '괜찮다'로 치면 안 된다 — 그게 가장 위험한 오판이다."""
    bad = park.too_far_from_idle(park.idle_offsets(_at_idle(s4=None)))
    assert set(bad) == {4}


def test_경계는_통과한다():
    """> 이지 >= 가 아니다 — 경계에서 조용히 막으면 원인을 못 찾는다."""
    assert not park.too_far_from_idle(
        park.idle_offsets(_at_idle(s5=park.IDLE_TOLERANCE_RAW)))
    assert park.too_far_from_idle(
        park.idle_offsets(_at_idle(s5=park.IDLE_TOLERANCE_RAW + 1)))


def test_허용치가_자동정렬_통과폭보다_좁다():
    """arm_driver 의 자동 정렬은 ±120 까지 '이미 IDLE' 로 통과시킨다. 여기서
    틀리면 팔이 쓰러지므로 통과 조건이 그보다 엄해야 한다."""
    assert park.IDLE_TOLERANCE_RAW < 120


# ── 사본이 원본과 갈라지지 않는지 ──────────────────────────────────────────


def test_IDLE_교시값이_원본과_같다():
    """계층이 달라 복제해 둔 값이다 — 재교시하면 같이 움직여야 한다."""
    src = (ROOT / "ros2_ws" / "src" / "grippers_arm" / "grippers_arm"
           / "floor_grasp_profiles.py").read_text(encoding="utf-8")
    line = next(l for l in src.splitlines() if l.startswith("IDLE_CRADLE_RAW"))
    assert eval(line.split("=", 1)[1].strip()) == park.IDLE_CRADLE_RAW
