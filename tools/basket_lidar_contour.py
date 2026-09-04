"""라이다 원시 스캔으로 바구니 주변 윤곽을 찍어 본다 (2026-09-04).

`basket_approach_insert_test.py`의 `lidar_face()`는 기대 방위각 창(±35도)
안에서 **가장 가까운 덩어리만** 골라 정면 한 면만 직선으로 피팅한다 —
INSERT 판정에는 그거면 충분하지만, "바구니 영역이 실제로 어디까지인가"를
눈으로 보려면 그 창 밖 점과 옆벽까지 다 필요하다.

이 도구는 판정을 전혀 하지 않는다. 그냥 전방 부채꼴 전체의 점을 차량
정면 기준 (x, y)로 바꿔서 표로 찍고, 위에서 내려다본 ASCII 그림으로도
그려 준다. 대화형 입력이 없어 비-TTY(SSH 논인터랙티브)로도 그대로 돈다.

사용:
    export ROS_DOMAIN_ID=21
    python3 tools/basket_lidar_contour.py [--range 1.0] [--width 0.5]
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy                                                    # noqa: E402
from rclpy.node import Node                                     # noqa: E402
from rclpy.qos import qos_profile_sensor_data                   # noqa: E402
from sensor_msgs.msg import LaserScan                           # noqa: E402

# basket_approach_insert_test.py와 같은 이유·같은 방식으로 원본을 경로에
# 직접 집어넣는다(basket_lidar_align은 아직 Pi의 ros2_ws에 빌드돼 있지 않다).
_ALIGN_DIR = Path(__file__).resolve().parent.parent / "ros2_ws/src/grippers_base/grippers_base"
sys.path.insert(0, str(_ALIGN_DIR))
import basket_lidar_align as align  # noqa: E402


class ScanNode(Node):
    def __init__(self):
        super().__init__("basket_lidar_contour")
        self.create_subscription(LaserScan, "/scan_raw", self._on_scan,
                                  qos_profile_sensor_data)
        self._scan = None

    def _on_scan(self, msg):
        self._scan = msg

    def wait_scan(self, timeout_s=3.0):
        self._scan = None
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._scan is not None:
                return self._scan
        return None


def render_ascii(points, forward_max_m, half_width_m, cell_m=0.02):
    """위에서 내려다본 그림. 위쪽이 라이다(원점), 아래로 갈수록 전방.

    가로 한 칸 = 세로 한 칸 = cell_m. 칸 안에 점이 있으면 '#'."""
    cols = int(2 * half_width_m / cell_m) + 1
    rows = int(forward_max_m / cell_m) + 1
    grid = [[" "] * cols for _ in range(rows)]
    origin_col = cols // 2
    for x, y in points:
        if not (0.0 <= x <= forward_max_m and abs(y) <= half_width_m):
            continue
        row = int(x / cell_m)
        col = origin_col - int(y / cell_m)   # y는 왼쪽이 +이므로 그림에서는 왼쪽=작은 col
        if 0 <= row < rows and 0 <= col < cols:
            grid[row][col] = "#"
    lines = []
    header = " " * 6 + "".join(
        "|" if c == origin_col else ("." if c % 5 == 0 else " ")
        for c in range(cols))
    lines.append(header)
    for row in range(rows):
        forward = row * cell_m
        marker = " " if row % 5 else f"{forward:4.2f}m"
        lines.append(f"{marker:>6}" + "".join(grid[row]))
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--range", type=float, default=1.0,
                        help="전방으로 볼 최대 거리 m (기본 1.0)")
    parser.add_argument("--width", type=float, default=0.5,
                        help="좌우 절반 폭 m (기본 0.5 -> 전체 1.0m)")
    parser.add_argument("--samples", type=int, default=5,
                        help="이어 찍을 스캔 장수 — 여러 장 겹쳐 찍어 잡음을 눈으로 확인")
    args = parser.parse_args()

    rclpy.init()
    node = ScanNode()
    try:
        all_points = []
        for i in range(args.samples):
            msg = node.wait_scan()
            if msg is None:
                print(f"  [{i + 1}/{args.samples}] 스캔 없음 — /scan_raw 확인", file=sys.stderr)
                continue
            points = align.scan_to_front_points(
                msg.ranges, msg.angle_min, msg.angle_increment,
                range_min=max(msg.range_min, 0.02),
                range_max=min(msg.range_max, args.range + 0.2))
            kept = [(x, y) for x, y in points
                    if 0.0 <= x <= args.range and abs(y) <= args.width]
            all_points.extend(kept)
            print(f"  [{i + 1}/{args.samples}] 점 {len(kept)}개 (전체 {len(points)}개 중)")

        if not all_points:
            print("점을 하나도 못 얻었습니다 — /scan_raw가 살아 있는지, "
                  "바구니가 전방에 있는지 확인하세요.")
            return 1

        print()
        print("=" * 70)
        print("전방 부채꼴 윤곽 (라이다 원점, 정면=+x, 왼쪽=+y)")
        print("=" * 70)
        print(render_ascii(all_points, args.range, args.width))
        print()
        print(f"총 {len(all_points)}점 (스캔 {args.samples}장 누적)")
        print()

        # 참고용 — 각도순 정렬 표(가까운 순 30개만, 너무 길면 못 읽으니).
        by_range = sorted(all_points, key=lambda p: math.hypot(*p))
        print("가까운 순 상위 30점 (mm 단위):")
        print("   거리   각도(정면=0,왼쪽+)    x(전방)    y(좌우)")
        for x, y in by_range[:30]:
            r = math.hypot(x, y) * 1000.0
            bearing = math.degrees(math.atan2(y, x))
            print(f"  {r:6.1f}mm   {bearing:+6.1f}deg      {x * 1000:+6.1f}   {y * 1000:+6.1f}")

        # fit_basket_face와 같은 방식(기대 방위각 창 안 최근접 덩어리)으로
        # 골라 그 덩어리의 실제 좌우 폭·전후 범위를 mm로 낸다 — ASCII 그림을
        # 눈대중으로 읽는 대신 정확한 숫자로 "바구니 영역"의 경계를 준다.
        face = align.select_face_points(all_points, expected_bearing_rad=0.0)
        if face:
            xs = [x for x, _y in face]
            ys = [y for _x, y in face]
            print()
            print(f"정면 덩어리(기대 방위각 0도 창 안 최근접 덩어리) {len(face)}점:")
            print(f"  전후 범위 x = {min(xs) * 1000:.1f} ~ {max(xs) * 1000:.1f} mm")
            print(f"  좌우 범위 y = {min(ys) * 1000:.1f} ~ {max(ys) * 1000:.1f} mm "
                  f"(폭 {(max(ys) - min(ys)) * 1000:.1f} mm)")
            fit = align.fit_basket_face(all_points, expected_bearing_rad=0.0)
            print(f"  fit_basket_face 결과: ok={fit.ok}  거리={fit.distance_m:.4f}m  "
                  f"yaw={math.degrees(fit.yaw_error_rad):+.2f}deg  {fit.reason}")

            # 잔차가 커서 단일 평면으로 실패했다면(=모서리를 하나의 덩어리로
            # 봤다는 신호), 최근접점(모서리 꼭짓점으로 가정)을 기준으로
            # 방위각 좌/우로 갈라 각각 따로 직선을 맞춰 본다 — 사선 접근
            # (2026-09-04 사용자 지시)에서 두 벽면이 동시에 보이는 경우를
            # 확인하려는 용도.
            if not fit.ok and "평면이 아니다" in fit.reason:
                vertex = min(face, key=lambda p: math.hypot(*p))
                vertex_bearing = math.atan2(vertex[1], vertex[0])
                left = [p for p in face if math.atan2(p[1], p[0]) > vertex_bearing]
                right = [p for p in face if math.atan2(p[1], p[0]) <= vertex_bearing]
                print()
                print(f"  단일 평면 실패 — 모서리로 보고 꼭짓점 기준 좌/우로 갈라 "
                      f"따로 맞춰 본다 (꼭짓점 ≈ x={vertex[0] * 1000:.1f}mm, "
                      f"y={vertex[1] * 1000:.1f}mm, 거리 "
                      f"{math.hypot(*vertex) * 1000:.1f}mm)")
                for name, group in (("왼쪽 벽", left), ("오른쪽 벽", right)):
                    if len(group) < 2:
                        print(f"    {name}: 점 부족({len(group)}개) — 못 맞춤")
                        continue
                    line = align.fit_line(group)
                    if line is None:
                        print(f"    {name}: 피팅 실패")
                        continue
                    nx, ny, cx, cy, residual, width = line
                    distance = nx * cx + ny * cy
                    face_yaw = math.atan2(ny, nx)
                    print(f"    {name} ({len(group)}점): 거리 {distance * 1000:.1f}mm  "
                          f"이 벽의 법선 방위각 {math.degrees(face_yaw):+.1f}deg  "
                          f"잔차 {residual * 1000:.1f}mm  폭 {width * 1000:.1f}mm")
                if len(left) >= 2 and len(right) >= 2:
                    ll, rl = align.fit_line(left), align.fit_line(right)
                    if ll and rl:
                        yaw_l, yaw_r = math.atan2(ll[1], ll[0]), math.atan2(rl[1], rl[0])
                        included = abs(math.degrees(yaw_l) - math.degrees(yaw_r))
                        print(f"    두 벽 사이 끼인각 ≈ {included:.1f}deg "
                              "(정육면체 바구니라면 90도에 가까워야 정상)")
        else:
            print()
            print("정면 덩어리를 못 골랐습니다 — 창(±35도) 안에 점이 없습니다.")

        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
