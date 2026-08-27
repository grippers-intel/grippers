"""proximity_gate.py 순수 계산 테스트 — rclpy/카메라 없이 돈다.

이 게이트는 Host 경로의 보완재이지 대체재가 아니다(사용자 지시 2026-08-25).
여기서는 수학과 안전 기본값만 검증한다 — 실제 정지 거리는 CPU YOLO 추론
지연을 실기로 재서 조정해야 한다."""

import importlib.util
import math
import pathlib

MODULE = (
    pathlib.Path(__file__).resolve().parent.parent
    / "ros2_ws" / "src" / "grippers_perception" / "grippers_perception" / "proximity_gate.py"
)


def _load():
    spec = importlib.util.spec_from_file_location("proximity_gate", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


pg = _load()
FRAME_W = 640.0


def _det(area, cx=FRAME_W / 2.0):
    return pg.Detection(area_px=area, center_x_px=cx)


def test_면적임계와_거리하한이_서로_역함수다():
    for stop_m in (0.20, 0.25, 0.40):
        area = pg.area_threshold_px(stop_distance_m=stop_m)
        assert math.isclose(pg.lower_bound_distance_m(area), stop_m, rel_tol=1e-9)


def test_면적이_클수록_거리하한이_짧다():
    assert pg.lower_bound_distance_m(20000.0) < pg.lower_bound_distance_m(5000.0)


def test_관측이_없으면_위험이다():
    """'모르면 멈춘다' — 빈 리스트(봤는데 없음)와 None(못 봄)은 다르다."""
    assert pg.evaluate(None, FRAME_W).contact_risk is True
    assert pg.evaluate([], FRAME_W).contact_risk is False


def test_먼_물체는_통과시킨다():
    area = pg.area_threshold_px(stop_distance_m=0.25) / 4.0
    assert pg.evaluate([_det(area)], FRAME_W).contact_risk is False


def test_가까운_물체는_멈춘다():
    area = pg.area_threshold_px(stop_distance_m=0.25) * 1.2
    verdict = pg.evaluate([_det(area)], FRAME_W, stop_distance_m=0.25)
    assert verdict.contact_risk is True
    assert "정지 거리" in verdict.reason


def test_임계_바로_위아래를_가른다():
    threshold = pg.area_threshold_px(stop_distance_m=0.25)
    assert pg.evaluate([_det(threshold * 1.01)], FRAME_W, 0.25).contact_risk is True
    assert pg.evaluate([_det(threshold * 0.99)], FRAME_W, 0.25).contact_risk is False


def test_아주_작은_bbox는_무시한다():
    """오검출이거나 너무 멀다 — 거리 추정을 시도하지 않는다."""
    assert pg.lower_bound_distance_m(pg.MIN_BBOX_AREA_PX - 1.0) is None
    assert pg.evaluate([_det(5.0)], FRAME_W).contact_risk is False


def test_화면_가장자리도_위험으로_본다():
    """프레임이 차체보다 좁아서 좌우 끝도 진로 안이다 — 모듈 docstring 참고."""
    area = pg.area_threshold_px(stop_distance_m=0.25) * 1.2
    for cx in (10.0, FRAME_W / 2.0, FRAME_W - 10.0):
        assert pg.evaluate([_det(area, cx)], FRAME_W, 0.25).contact_risk is True


def test_구간별_최근접거리를_따로_낸다():
    near = pg.area_threshold_px(stop_distance_m=0.25) * 1.2
    far = pg.area_threshold_px(stop_distance_m=1.0)
    verdict = pg.evaluate([_det(near, 10.0), _det(far, FRAME_W - 10.0)], FRAME_W, 0.25)
    assert verdict.left_m < verdict.right_m
    assert math.isinf(verdict.front_m)


def test_직전에_가까웠다가_사라지면_위험을_유지한다():
    """근거리 사각지대(12.8cm) — 안 보인다가 없다를 뜻하지 않는다."""
    verdict = pg.evaluate([], FRAME_W, previously_close=True)
    assert verdict.contact_risk is True
    assert "사각지대" in verdict.reason


def test_정지거리가_사각지대보다_넉넉히_앞이다():
    """사각지대에 닿고 나서 멈추면 이미 늦다."""
    assert pg.DEFAULT_STOP_DISTANCE_M > pg.NEAR_FIELD_BLIND_M * 1.5


def test_최소K를_쓰므로_거리를_과소평가한다():
    """실측 K 중 어느 것을 써도 하한보다 멀게 나와야 한다 — 늦게 멈추지 않는다."""
    area = 5000.0
    bound = pg.lower_bound_distance_m(area)
    for k in (39.5578, 38.3357, 37.7658, 25.8794, 23.2733, 24.1690):  # 2026-08-27 box·star 추가
        actual = k / (math.sqrt(area) - pg.BBOX_PADDING_PX)
        assert actual >= bound - 1e-9


def test_정지거리가_0이하면_거부한다():
    for bad in (0.0, -0.1):
        try:
            pg.area_threshold_px(stop_distance_m=bad)
        except ValueError:
            continue
        raise AssertionError("양수가 아닌 정지 거리를 받아들였다")
