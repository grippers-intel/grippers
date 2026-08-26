"""Ros2Lidar — Lidar 포트의 실기 구현. `/scan_raw`를 직접 구독한다.

서비스 왕복을 두지 않은 이유: 라이다 판정은 INSERT 전환 직전에 한 번
필요할 뿐이고, 스캔은 이미 토픽으로 흐르고 있다. 사이에 노드를 하나 더
두면 인터페이스와 실패 모드만 늘어난다.

선분 피팅 수학은 여기 없다 — `grippers_base.basket_lidar_align`에 있고
이 어댑터는 그것을 불러 결과만 포트 자료형으로 옮긴다(포트 docstring의
계층 분리 규약)."""

import math

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from grippers_base import basket_lidar_align as align

from domain.ports.baseline_ports import BasketFace, Lidar

SCAN_TOPIC = "/scan_raw"

# 이 시간 안에 새 스캔이 안 오면 관측 실패로 본다. 라이다는 10Hz라
# 정상이면 한참 전에 도착한다.
SCAN_STALE_SEC = 1.0

# LD19의 최소 측정 거리(실측 확인). 스캔 메시지의 range_min과 큰 쪽을 쓴다.
LD19_MIN_RANGE_M = 0.020


class Ros2Lidar(Lidar):
    def __init__(self, node, clock=None):
        self._node = node
        self._clock = clock or (lambda: node.get_clock().now().nanoseconds / 1e9)
        self._scan = None
        self._stamp = None
        node.create_subscription(
            LaserScan, SCAN_TOPIC, self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg):
        self._scan = msg
        self._stamp = self._clock()

    def basket_face(self, bearing_rad: float = 0.0) -> BasketFace:
        """정면 바구니까지의 거리와 정렬 오차. **모르면 실패**."""
        scan = self._scan
        if scan is None:
            return BasketFace(False, math.inf, math.inf, f"{SCAN_TOPIC} 스캔 없음")
        age = self._clock() - (self._stamp or 0.0)
        if age > SCAN_STALE_SEC:
            return BasketFace(False, math.inf, math.inf, f"스캔이 오래됐다 ({age:.1f}s)")

        points = align.scan_to_front_points(
            scan.ranges, scan.angle_min, scan.angle_increment,
            range_min=max(scan.range_min, LD19_MIN_RANGE_M),
            range_max=min(scan.range_max, 3.0))
        fit = align.fit_basket_face(points, expected_bearing_rad=bearing_rad)
        return BasketFace(fit.ok, fit.distance_m, fit.yaw_error_rad, fit.reason,
                          fit.point_count, fit.lateral_offset_m, fit.lateral_known)
