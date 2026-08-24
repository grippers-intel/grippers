#!/usr/bin/env python3
"""직진 접근 캘리브레이션 콘솔 — Enter로 전진 시작, Enter로 정지.

GRASP 마지막 미세 접근 단계("그리퍼 사이로 물체가 들어올 때까지 서서히
똑바로 직진하다가, 그리퍼캠으로 좌우 안 치우치고 면적이 기준을 넘으면
정지") 의 정지 조건(이동 거리·면적 임계값)을 실기로 찾기 위한 도구다.
grasp_test_console.py의 이미 검증된 부품을 그대로 재사용한다 — 같은
cmd_vel/odom_raw/그리퍼캠 채널을 여기서 따로 다시 만들지 않는다:

    GraspTestNode  — cmd_vel 발행 + odom_raw 구독
    GripperCam     — 그리퍼캠 컨투어 최대 면적 측정(measure_area_px2)
    KeyReader      — Enter/q 논블로킹 키 입력
    odom_distance_m — 두 (x,y) 사이 유클리드 거리

절차:
  Enter (첫 번째) → 저속 순수 직진 시작(회전 없음, linear.x=FINE_SPEED_MPS)
  Enter (두 번째) → 정지. 이동 거리와 정지 시점 그리퍼캠 면적을 출력한다.
  q                → 종료(전진 중이 아닐 때만 — "시작하려면 Enter" 프롬프트에서)

같은 물체를 여러 번 반복 실행하며 "이만큼 이동 + 이 정도 면적이면 멈춰도
된다"는 조합을 찾는 용도다.

⚠️ perception_node가 /dev/gripper_cam을 __init__ 시점에 무조건 잡고
있다(grasp_test_console.py의 GripperCam 클래스 docstring 참고) — 시작
전에 반드시 꺼야 GripperCam이 카메라를 열 수 있다:
    pkill -f grippers_perception/perception_node
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

from grasp_test_console import (
    FINE_SPEED_MPS,
    TICK_S,
    GraspTestNode,
    GripperCam,
    KeyReader,
    odom_distance_m,
)


def run():
    rclpy.init()
    node = GraspTestNode()
    cam = None
    try:
        cam = GripperCam()
    except Exception as exc:  # noqa: BLE001 -- 카메라 없어도 거리 측정만으로도 쓸모 있다
        print(f"[approach] 그리퍼캠 열기 실패({exc}) — 거리만 표시합니다", file=sys.stderr)

    print("[approach] Enter로 직진 시작 · 그다음 Enter로 정지 · (정지 상태에서) q로 종료")
    try:
        with KeyReader() as kr:
            while True:
                kr.wait_enter("시작하려면 Enter (q로 종료) > ")

                for _ in range(5):  # odom_raw 최신값 확보
                    node.pump()
                    time.sleep(0.02)
                start_pose = node._pose

                print("  전진 중... 정지하려면 Enter")
                twist = Twist()
                twist.linear.x = FINE_SPEED_MPS
                while True:
                    node.cmd_pub.publish(twist)
                    node.pump()
                    key = kr.getch_nonblocking()
                    if key in ("\n", "\r"):
                        break
                    time.sleep(TICK_S)
                node.stop()

                end_pose = node._pose
                distance_m = odom_distance_m(start_pose, end_pose)
                area_px2 = cam.measure_area_px2() if cam is not None else None

                distance_str = (
                    f"{distance_m * 100:.1f}cm" if distance_m is not None else "측정불가(odom_raw 미수신)"
                )
                area_str = f"{area_px2:.0f}px²" if area_px2 is not None else "검출 안 됨"
                print(f"  결과 — 이동거리={distance_str}  그리퍼캠 면적={area_str}")
    except KeyboardInterrupt:
        print("\n[approach] 종료")
    finally:
        node.stop()
        if cam is not None:
            cam.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    run()
