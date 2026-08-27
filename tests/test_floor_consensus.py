"""floor_consensus.py 순수 값 테스트 — rclpy/카메라 없이 돈다.

RELIABLE_CLASSES 회귀 방지용. 2026-08-23 train-8 시절엔 box/star가 실제로
불안정(box 0/60, star conf 0.31)해서 허용목록에서 뺐었는데, 2026-08-27
train-9로 재검증하니 나머지 넷보다도 깨끗해(60/60, 순도 1.00) 다시 넣었다 —
이 파일이 다시 좁아지면 그 재검증 결과를 잊었다는 뜻이다."""

import importlib.util
import pathlib

MODULE = (
    pathlib.Path(__file__).resolve().parent.parent
    / "ros2_ws" / "src" / "grippers_perception" / "grippers_perception" / "floor_consensus.py"
)


def _load():
    spec = importlib.util.spec_from_file_location("floor_consensus", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


fc = _load()


def test_여섯_클래스_전부_허용목록에_있다():
    """2026-08-27 train-9 재검증(box 60/60, star 60/60) 이후의 상태다."""
    assert set(fc.RELIABLE_CLASSES) == {
        "knight", "queen", "rook", "soccer", "box", "star",
    }
