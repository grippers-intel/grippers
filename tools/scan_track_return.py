#!/usr/bin/env python3
"""SCAN → 제자리회전+직진 추적 → (선택) 원위치 복귀 — 실기 테스트 콘솔.

2026-08-24 요청 사항 3개를 하나로 묶었다:
  1) 입력받은 물체(6종 중 하나)를 찾아 그 앞 지정 거리(기본 35cm)까지
     제자리회전+직진으로 추적한다.
  2) 추적 중 base가 실제로 이동한 (x,y,yaw)를 기억해뒀다가, 시작점까지
     "제자리 회전 1회 + 직진 1회"로 되돌아간다(경로를 그대로 재생하지
     않는다 — scan_track_control.compute_return_vector 참고, 왜 이
     방식을 골랐는지는 그 함수 docstring에 적어뒀다).
  3) 추적·복귀 중 목표가 아닌 다른 물체가 경로 위에 있으면 linear.y로
     한쪽으로만 피한다(LiDAR가 아니라 YOLO로 본다 — 2026-08-23 실기
     확인: LD19는 이 높이의 체스말을 아예 못 본다).

⚠️ 제자리 회전 실패 원인 조사(2026-08-24, 코드 조사) 결과 이 스크립트는
`controller/cmd_vel`(클램프 없음)에 발행한다 — `cmd_vel`은
`odom_publisher_node.py`가 angular.z를 ±0.5 rad/s로 자른다. 자세한
근거는 `ros2_ws/src/grippers_base/grippers_base/scan_track_control.py`
모듈 docstring 참고.

⚠️ `/odom_raw`는 엔코더가 아니라 명령을 적분한 값이다 — 회전이 실제로
안 먹히면 원위치 복귀 벡터도 같이 틀어진다. 처음 실행할 때는 반드시
사람이 옆에서 지켜보며 실제로 도는지 눈으로 확인할 것.

실행 (grasp_test_console.py와 같은 환경 — `perception_node`,
`odom_publisher.launch.py`+`depth_camera.launch.py`(+third_party_ws
소싱), `depth_cam_rotate_node`가 떠 있어야 한다. 이 스크립트는 팔은 전혀
안 건드리므로 arm_driver는 필요 없다):

    python3 /grippers/tools/scan_track_return.py --raw-cls rook

    (거리·복귀 생략 옵션)
    python3 /grippers/tools/scan_track_return.py --raw-cls queen --target-distance-m 0.4
    python3 /grippers/tools/scan_track_return.py --raw-cls soccer --no-return

언제든 q+Enter 또는 Ctrl+C로 즉시 정지.
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from grippers_interfaces.srv import ObserveTarget

sys.path.insert(0, "/grippers/tools")
from grasp_test_console import KeyReader, RunLog  # noqa: E402

from grippers_base.scan_track_control import (  # noqa: E402
    K_CLASS,
    ObstacleObservation,
    bbox_area_distance_m,
    choose_dodge_side,
    DEFAULT_ALIGN_TURN_RAD_S,
    compute_align_command,
    compute_dodge_command,
    compute_drive_command,
    compute_return_vector,
    establish_target_h,
    find_path_obstacle,
    h_signal_reliable,
    lateral_offset_m,
    z_from_established_h,
)

FRAME_CENTER_X_PX = 320.0  # 카메라 프레임 640폭 실측(camera_info) 기준 정가운데
TOL_X_PX = 15.0
TOL_DIST_M = 0.03
OBSTACLE_PATH_HALF_WIDTH_M = 0.15  # 경로 좌우 폭(편측) — 실기 미검증 자리표시자
ASPECT_RATIO_MAX_DEVIATION = 0.4

BURST_S = 0.3
SETTLE_S = 0.2
DODGE_BURST_S = 0.4

MAX_TRACK_ITERS = 80
MAX_RETURN_DRIVE_ITERS = 40
MAX_CONSECUTIVE_MISSES = 10
OBSTACLE_CHECK_TIMEOUT_S = 0.5  # 관측 대상이 아닌 5개 클래스 확인용 — 짧게 끊는다


def _all_other_classes(target_cls: str) -> list[str]:
    return [c for c in K_CLASS if c != target_cls]


class ScanTrackNode(Node):
    def __init__(self):
        super().__init__("scan_track_return")
        # ⚠️ controller/cmd_vel — cmd_vel이 아니다. 모듈 docstring 참고.
        self.cmd_pub = self.create_publisher(Twist, "controller/cmd_vel", 10)
        self.create_subscription(Odometry, "odom_raw", self._on_odom, 10)
        self._pose = None  # (x, y, yaw_rad)
        self._observe_client = self.create_client(ObserveTarget, "perception/observe_target")

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        # yaw_from_quaternion을 여기서 다시 안 쓰고 직접 계산 — scan_track_control은
        # rclpy 의존을 안 가지므로, 쿼터니언 필드 자체는 여기서 풀어 넘긴다.
        import math

        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self._pose = (p.x, p.y, yaw)

    def pump(self):
        rclpy.spin_once(self, timeout_sec=0.0)

    def observe(self, raw_cls: str, timeout_sec: float = 3.0):
        if not self._observe_client.wait_for_service(timeout_sec=timeout_sec):
            return None
        future = self._observe_client.call_async(ObserveTarget.Request(raw_cls=raw_cls))
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result() if future.done() else None

    def stop(self):
        for _ in range(6):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)

    def publish_and_settle(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, burst_s=BURST_S, settle_s=SETTLE_S):
        t = Twist()
        t.linear.x = linear_x
        t.linear.y = linear_y
        t.angular.z = angular_z
        self.cmd_pub.publish(t)
        time.sleep(burst_s)
        self.stop()
        time.sleep(settle_s)


def check_path_obstacle(node: ScanTrackNode, target_cls: str, max_range_m: float | None) -> ObstacleObservation | None:
    """목표가 아닌 5개 클래스를 각각 한 번씩 관측해 경로 위 장애물을 찾는다.
    ⚠️ 반복당 최대 5회 추가 관측 — 느릴 수 있다(실기에서 체감상 너무 느리면
    OBSTACLE_CHECK_TIMEOUT_S를 줄이거나 매 반복이 아니라 N반복마다 한 번만
    체크하도록 바꿀 것)."""
    observations = []
    for cls in _all_other_classes(target_cls):
        node.pump()
        r = node.observe(cls, timeout_sec=OBSTACLE_CHECK_TIMEOUT_S)
        if r is None or not r.found:
            continue
        k = K_CLASS.get(cls)
        if k is None:
            continue
        z_m = bbox_area_distance_m(r.h, r.w, k)
        if z_m is None:
            continue
        lat_m = lateral_offset_m(r.x, z_m)
        observations.append(ObstacleObservation(cls=cls, forward_m=z_m, lateral_m=lat_m))
    return find_path_obstacle(observations, path_half_width_m=OBSTACLE_PATH_HALF_WIDTH_M, max_range_m=max_range_m)


def track_to_target(node: ScanTrackNode, kr: KeyReader, log: RunLog, target_cls: str, target_distance_m: float) -> bool:
    k_class = K_CLASS.get(target_cls)
    if k_class is None:
        print(f"  [실패] '{target_cls}'는 거리 보정값(K_CLASS) 미실측 — 이 스크립트로는 추적 불가")
        return False

    target_h = None
    ref_aspect = None
    misses = 0

    for i in range(MAX_TRACK_ITERS):
        if kr.getch_nonblocking() == "q":
            print("  [중단] 사용자 요청")
            node.stop()
            return False
        node.pump()
        obs = node.observe(target_cls)
        if obs is None or not obs.found:
            misses += 1
            print(f"  [{i}] 물체 못 찾음 ({misses}/{MAX_CONSECUTIVE_MISSES})")
            if misses >= MAX_CONSECUTIVE_MISSES:
                print("  [실패] 연속 미검출 상한 도달")
                return False
            node.stop()
            time.sleep(SETTLE_S)
            continue
        misses = 0

        if target_h is None:
            target_h = establish_target_h(obs.h, obs.w, k_class, target_distance_m)
            ref_aspect = obs.w / obs.h if obs.h > 0 else None
            print(f"  목표 확립: target_h={target_h:.1f}px (첫 관측 h={obs.h:.1f} w={obs.w:.1f})")
            log.log("target_established", target_h=target_h, ref_aspect=ref_aspect)

        # 현재 거리(장애물 판단용 max_range) — 신뢰도와 무관하게 면적 공식으로
        current_forward_m = bbox_area_distance_m(obs.h, obs.w, k_class)

        obstacle = check_path_obstacle(node, target_cls, max_range_m=current_forward_m)
        if obstacle is not None:
            side = choose_dodge_side(obstacle.lateral_m)
            print(f"  [{i}] 장애물 감지: {obstacle.cls} 전방 {obstacle.forward_m*100:.0f}cm "
                  f"좌우 {obstacle.lateral_m*100:+.0f}cm -> {'좌' if side>0 else '우'}측 회피")
            log.log("obstacle_dodge", cls=obstacle.cls, forward_m=obstacle.forward_m,
                     lateral_m=obstacle.lateral_m, side=side)
            cmd = compute_dodge_command(side)
            node.publish_and_settle(linear_y=cmd.linear_y, burst_s=DODGE_BURST_S)
            continue

        err_x = obs.x - FRAME_CENTER_X_PX

        reliable = h_signal_reliable(obs.h, obs.w, ref_aspect) if ref_aspect else False
        if reliable:
            z_now = z_from_established_h(obs.h, target_h, target_distance_m)
            signal = "h"
        else:
            z_now = bbox_area_distance_m(obs.h, obs.w, k_class)
            signal = "area(fallback)"
        if z_now is None:
            print(f"  [{i}] 거리 계산 실패 — 이번 프레임 건너뜀")
            node.stop()
            time.sleep(SETTLE_S)
            continue
        err_dist_m = z_now - target_distance_m

        print(f"  [{i}] x={obs.x:.1f} err_x={err_x:+.1f} z={z_now*100:.1f}cm({signal}) "
              f"err_dist={err_dist_m*100:+.1f}cm")
        log.log("track_step", i=i, x=obs.x, err_x=err_x, z_m=z_now, err_dist_m=err_dist_m, signal=signal)

        aligned = abs(err_x) <= TOL_X_PX
        if not aligned:
            cmd = compute_align_command(err_x, tol_x=TOL_X_PX)
            node.publish_and_settle(angular_z=cmd.angular_z)
            continue

        if abs(err_dist_m) <= TOL_DIST_M:
            node.stop()
            print(f"  [{i}] 도착 — 정렬+목표거리 도달")
            log.log("track_arrived", iters=i)
            return True

        cmd = compute_drive_command(err_dist_m, tol_dist_m=TOL_DIST_M)
        node.publish_and_settle(linear_x=cmd.linear_x)

    print("  [실패] 반복 상한 도달 — 수렴 못 함")
    return False


def return_to_start(node: ScanTrackNode, kr: KeyReader, log: RunLog, target_cls: str, start_pose) -> bool:
    node.pump()
    if node._pose is None:
        print("  [실패] 현재 위치(odom) 없음")
        return False
    heading_error, distance_m = compute_return_vector(start_pose[0], start_pose[1], node._pose[0], node._pose[1], node._pose[2])
    print(f"  복귀 벡터: 회전 {heading_error:+.2f}rad({heading_error*57.3:+.0f}deg), 거리 {distance_m*100:.1f}cm")
    log.log("return_vector", heading_error_rad=heading_error, distance_m=distance_m)

    # 1) 제자리 회전 — 목표 heading에 도달할 때까지
    for i in range(MAX_TRACK_ITERS):
        if kr.getch_nonblocking() == "q":
            node.stop()
            return False
        node.pump()
        if node._pose is None:
            break
        remaining = compute_return_vector(start_pose[0], start_pose[1], node._pose[0], node._pose[1], node._pose[2])[0]
        if abs(remaining) <= 0.05:  # ~2.9deg
            break
        # compute_align_command는 픽셀 오차용이라 라디안 오차에 그대로 못
        # 쓴다 — 여기서는 부호(어느 쪽으로 돌지)만 필요하므로 직접 낸다.
        wz = DEFAULT_ALIGN_TURN_RAD_S if remaining > 0 else -DEFAULT_ALIGN_TURN_RAD_S
        node.publish_and_settle(angular_z=wz)
    node.stop()

    # 2) 장애물 확인하며 직진 — odom 거리 기준
    node.pump()
    drive_start = node._pose
    for i in range(MAX_RETURN_DRIVE_ITERS):
        if kr.getch_nonblocking() == "q":
            node.stop()
            return False
        node.pump()
        if node._pose is None or drive_start is None:
            break
        moved = ((node._pose[0] - drive_start[0]) ** 2 + (node._pose[1] - drive_start[1]) ** 2) ** 0.5
        remaining_m = distance_m - moved

        obstacle = check_path_obstacle(node, target_cls, max_range_m=remaining_m)
        if obstacle is not None:
            side = choose_dodge_side(obstacle.lateral_m)
            print(f"  [복귀 {i}] 장애물 회피: {obstacle.cls} -> {'좌' if side>0 else '우'}측")
            cmd = compute_dodge_command(side)
            node.publish_and_settle(linear_y=cmd.linear_y, burst_s=DODGE_BURST_S)
            continue

        if remaining_m <= TOL_DIST_M:
            node.stop()
            print(f"  복귀 완료 — {moved*100:.1f}cm 이동")
            log.log("return_done", moved_m=moved)
            return True
        cmd = compute_drive_command(remaining_m, tol_dist_m=TOL_DIST_M)
        node.publish_and_settle(linear_x=cmd.linear_x)

    print("  [실패] 복귀 직진 반복 상한 도달")
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raw-cls", required=True, choices=sorted(K_CLASS))
    ap.add_argument("--target-distance-m", type=float, default=0.35)
    ap.add_argument("--no-return", action="store_true", help="추적만 하고 복귀는 생략")
    args = ap.parse_args()

    log = RunLog(args.raw_cls, "scan_track_return")
    print(f"대상: {args.raw_cls}  목표거리: {args.target_distance_m*100:.0f}cm  분석용 로그: {log.path}")

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ScanTrackNode()
    try:
        with KeyReader() as kr:
            kr.wait_enter("\nEnter로 SCAN+추적 시작 (q+Enter로 취소): ")
            node.pump()
            start_pose = node._pose
            if start_pose is None:
                print("  [경고] 시작 시점 odom 없음 — 복귀 단계는 못 함")

            print("\n[SCAN+TRACK]")
            arrived = track_to_target(node, kr, log, args.raw_cls, args.target_distance_m)
            if not arrived:
                print("추적 실패 — 종료합니다.")
                return

            if args.no_return:
                print("\n완료(복귀 생략).")
                return
            if start_pose is None:
                print("\n시작 위치를 몰라 복귀를 못 합니다.")
                return

            kr.wait_enter("\nEnter로 원위치 복귀 시작 (q+Enter로 취소): ")
            print("\n[RETURN]")
            ok = return_to_start(node, kr, log, args.raw_cls, start_pose)
            print("\n완료 — 원위치 복귀." if ok else "\n복귀 실패 — 수동으로 위치 확인할 것.")

    except KeyboardInterrupt:
        print("\n[중단] 정지 명령 발행 중...")
        node.stop()
        log.log("aborted")
    finally:
        log.log("run_end")
        log.close()
        print(f"\n분석용 로그: {log.path}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
