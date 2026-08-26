"""basket_lidar_align.py 순수 계산 테스트 — rclpy/라이다 없이 돈다.

⚠️ 실기 미검증이다. 여기서 검증하는 것은 수학과 "모르면 실패" 기본값뿐이고,
이 모듈이 실제로 성립하려면 라이다 평면(바닥 위 91mm)이 바구니 테두리보다
낮아야 한다 — 모듈 docstring의 높이 전제 참고."""

import importlib.util
import math
import pathlib

import pytest

MODULE = (
    pathlib.Path(__file__).resolve().parent.parent
    / "ros2_ws" / "src" / "grippers_base" / "grippers_base" / "basket_lidar_align.py"
)


def _load():
    spec = importlib.util.spec_from_file_location("basket_lidar_align", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


bl = _load()


def _face_points(distance, yaw=0.0, width=bl.BASKET_FACE_WIDTH_M, n=15):
    """법선이 yaw 방향이고 원점에서 수직거리 distance인 평면 조각."""
    cx, cy = distance * math.cos(yaw), distance * math.sin(yaw)
    dx, dy = -math.sin(yaw), math.cos(yaw)
    return [
        (cx + (-width / 2.0 + width * i / (n - 1)) * dx,
         cy + (-width / 2.0 + width * i / (n - 1)) * dy)
        for i in range(n)
    ]


def test_정면_바구니의_거리와_정렬을_복원한다():
    fit = bl.fit_basket_face(_face_points(0.30), expected_bearing_rad=0.0)
    assert fit.ok is True
    assert math.isclose(fit.distance_m, 0.30, abs_tol=1e-6)
    assert math.isclose(fit.yaw_error_rad, 0.0, abs_tol=1e-6)
    assert math.isclose(fit.face_width_m, bl.BASKET_FACE_WIDTH_M, abs_tol=1e-6)


def test_비스듬히_선_경우_yaw오차를_낸다():
    for yaw_deg in (-20.0, -8.0, 8.0, 20.0):
        yaw = math.radians(yaw_deg)
        fit = bl.fit_basket_face(_face_points(0.30, yaw), expected_bearing_rad=yaw)
        assert fit.ok is True, f"{yaw_deg}도에서 실패: {fit.reason}"
        assert math.isclose(fit.yaw_error_rad, yaw, abs_tol=1e-6)
        assert math.isclose(fit.distance_m, 0.30, abs_tol=1e-6)


def test_뒤쪽_벽이_섞이지_않는다():
    """바구니가 벽에서 175mm 나와 있어 최근접 덩어리만 취하면 벽이 빠진다."""
    points = _face_points(0.30) + _face_points(0.30 + 0.175, width=1.2, n=40)
    fit = bl.fit_basket_face(points, expected_bearing_rad=0.0)
    assert fit.ok is True
    assert math.isclose(fit.distance_m, 0.30, abs_tol=1e-6)
    assert fit.point_count == 15


def test_옆_바구니는_방위각_창_밖이다():
    """두 바구니가 90cm 떨어져 있다 — Host가 알려준 방향만 본다."""
    here = _face_points(0.30, 0.0)
    there = [(x, y + 0.90) for x, y in _face_points(0.30, 0.0)]
    fit = bl.fit_basket_face(here + there, expected_bearing_rad=0.0)
    assert fit.ok is True
    assert fit.point_count == 15


def test_점이_모자라면_실패한다():
    fit = bl.fit_basket_face(_face_points(0.30, n=3), expected_bearing_rad=0.0)
    assert fit.ok is False
    assert "점 부족" in fit.reason


def test_모서리가_섞이면_실패한다():
    """두 면이 직각으로 만나는 덩어리 — 단일 평면이 아니다.

    이것이 잔차 검사가 실제로 잡아야 할 실패 모드다(바구니 모서리를 비스듬히
    보거나 벽과 바구니가 한 덩어리로 묶인 경우)."""
    corner = (
        [(0.30, t / 100.0) for t in range(0, 11)]
        + [(0.30 + t / 100.0, 0.10) for t in range(1, 11)]
    )
    fit = bl.fit_basket_face(corner, expected_bearing_rad=math.radians(10.0))
    assert fit.ok is False
    assert "잔차" in fit.reason


def test_완만한_곡면은_구분하지_못한다():
    """알려진 한계 — 이 동작을 못 박아 둔다.

    반지름 0.30m 호를 200mm 구간에서 보면 잔차가 5.9mm다. 이보다 조이면
    라이다 거리 잡음(0.3m에서 약 3mm)과 구분이 안 되므로 일부러 통과시킨다.
    이 아레나에 완만한 곡면 장애물이 없다는 전제에 기대는 부분이다."""
    arc = [
        (0.30 * math.cos(a), 0.30 * math.sin(a))
        for a in [math.radians(d) for d in range(-20, 21, 3)]
    ]
    fit = bl.fit_basket_face(arc, expected_bearing_rad=0.0)
    assert fit.ok is True
    assert fit.residual_m < bl.DEFAULT_MAX_RESIDUAL_M


def test_겉보기_폭이_이상하면_실패한다():
    narrow = bl.fit_basket_face(_face_points(0.30, width=0.04), expected_bearing_rad=0.0)
    assert narrow.ok is False and "폭" in narrow.reason
    wide = bl.fit_basket_face(_face_points(0.30, width=0.60, n=40), expected_bearing_rad=0.0)
    assert wide.ok is False and "폭" in wide.reason


def test_관측이_없으면_실패한다():
    fit = bl.fit_basket_face([], expected_bearing_rad=0.0)
    assert fit.ok is False
    assert math.isinf(fit.distance_m)


def test_scan_to_points가_무효값을_버린다():
    ranges = [0.30, float("inf"), float("nan"), 0.01, 5.0, 0.31]
    pts = bl.scan_to_points(ranges, angle_min=0.0, angle_increment=math.radians(1.0),
                            range_min=0.05, range_max=3.0)
    assert len(pts) == 2


def test_scan_to_points의_각도_배치가_맞다():
    pts = bl.scan_to_points([1.0], angle_min=math.radians(90.0), angle_increment=0.0)
    assert math.isclose(pts[0][0], 0.0, abs_tol=1e-9)
    assert math.isclose(pts[0][1], 1.0, abs_tol=1e-9)


def test_라이다_높이가_실측값이다():
    """URDF(0.1625)도 2026-08-25에 잘못 적은 0.091도 아닌, 재실측한 0.140이다."""
    assert math.isclose(bl.LIDAR_HEIGHT_M, 0.140, abs_tol=1e-9)
    assert math.isclose(bl.LIDAR_TILT_DEG, 11.3, abs_tol=1e-9)


def test_빔이_테두리를_넘으려면_최소한_이만큼_떨어져야_한다():
    """라이다가 테두리보다 **높이** 있으므로, 이제 "충분히 멀리"가 조건이다.

    기울기 덕분에 앞으로 갈수록 빔이 낮아진다. 테두리(115mm) 아래로
    내려오는 지점이 약 125mm이고, 그보다 가까이 붙으면 빔이 테두리 위를
    스쳐 지나가 바구니를 통째로 놓친다 — 예전 전제("평면이 테두리보다
    낮다")와 부호가 반대다."""
    assert bl.LIDAR_HEIGHT_M > bl.BASKET_RIM_HEIGHT_M

    너무_가까움 = bl.beam_height_m(0.10)
    assert 너무_가까움 > bl.BASKET_RIM_HEIGHT_M

    경계 = bl.LIDAR_HEIGHT_M - bl.BASKET_RIM_HEIGHT_M
    경계 /= math.tan(math.radians(bl.LIDAR_TILT_DEG))
    assert math.isclose(경계, 0.125, abs_tol=0.002)


def test_실기_검증된_정지_거리에서_빔이_테두리_아래에_있다():
    """2026-08-26에 투하가 성공한 라이다 판독 0.1386m 지점."""
    여유 = bl.BASKET_RIM_HEIGHT_M - bl.beam_height_m(0.1386)
    assert 0.0 < 여유 < 0.005          # 성립하지만 3mm 안팎으로 아슬아슬하다


def test_빔은_70cm에서_바닥에_닿는다():
    """그 너머의 정면 반사는 바구니가 아니라 바닥이다."""
    바닥_도달 = bl.LIDAR_HEIGHT_M / math.tan(math.radians(bl.LIDAR_TILT_DEG))
    assert math.isclose(바닥_도달, 0.70, abs_tol=0.01)


def test_scan_to_front_points가_정면을_x축에_놓는다():
    """정면 = 라이다 +90도(2026-08-26 판 실험). 그 각도의 반사가 +x로 와야 한다."""
    pts = bl.scan_to_front_points([0.30], angle_min=math.radians(90.0), angle_increment=0.0)
    x, y = pts[0]
    assert math.isclose(y, 0.0, abs_tol=1e-9)
    # 기울기 보정으로 판독값보다 짧아진 수평거리가 나온다.
    assert math.isclose(x, 0.30 * math.cos(math.radians(bl.LIDAR_TILT_DEG)), abs_tol=1e-9)
    assert x < 0.30


def test_법선이_로봇에서_바구니를_향한다():
    fit = bl.fit_line(_face_points(0.30, math.radians(15.0)))
    nx, ny, cx, cy, _residual, _width = fit
    assert nx * cx + ny * cy > 0.0


# ── 좌우 오프셋 검출 (2026-08-26 추가) ────────────────────────────────────


def _basket_points(distance_m, offset_m, window_rad=math.radians(35.0), n=400):
    """`offset_m`만큼 옆으로 밀린 바구니 정면을, 방위각 창으로 잘라서 낸다."""
    points = []
    for i in range(-n // 2, n // 2 + 1):
        y = offset_m + i * bl.BASKET_FACE_WIDTH_M / n
        if abs(math.atan2(y, distance_m)) <= window_rad:
            points.append((distance_m, y))
    return points


def test_창을_양쪽_다_채우면_오프셋을_모른다():
    """0은 '가운데'가 아니라 '모른다'다 — 판정하는 쪽이 그렇게 읽어야 한다."""
    offset, known = bl.face_lateral_offset_m(_basket_points(0.140, 0.0), 0.140)

    assert known is False
    assert offset == 0.0


def test_한쪽_가장자리가_보이면_오프셋을_역산한다():
    for actual in (0.045, 0.080, -0.080):
        offset, known = bl.face_lateral_offset_m(
            _basket_points(0.140, actual), 0.140)
        assert known is True
        assert offset == pytest.approx(actual, abs=0.006)


def test_투하가_실패하는_오프셋은_반드시_검출된다():
    """검출 시작(약 40mm)이 위험 시작(약 78mm)보다 먼저여야 구멍이 없다."""
    _offset, known = bl.face_lateral_offset_m(_basket_points(0.140, 0.078), 0.140)

    assert known is True


def test_점이_없으면_모른다():
    assert bl.face_lateral_offset_m([], 0.140) == (0.0, False)


def test_정상_피팅이_오프셋을_함께_낸다():
    fit = bl.fit_basket_face(_basket_points(0.140, 0.080), expected_bearing_rad=0.0)

    assert fit.ok
    assert fit.lateral_known is True
    assert fit.lateral_offset_m == pytest.approx(0.080, abs=0.006)
