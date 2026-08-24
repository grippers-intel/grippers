#!/usr/bin/env python3
"""로봇 전원 상태 확인 — 팔(SO-ARM101) 배터리 전압 + 베이스/파이 배터리 전압.

읽기 전용, 서보에 아무것도 쓰지 않는다. 각 서브시스템의 전용 하드웨어 채널로
직접 읽으므로 ROS 노드가 하나도 안 떠 있어도 동작한다(둘 다 시리얼을 직접 연다).
단, arm_driver/ros_robot_controller가 이미 해당 포트를 점유 중이면 열기 실패나
불안정한 응답이 나올 수 있으니, 가능하면 노드 기동 전에 먼저 돌려서 확인할 것.

  팔 전원   : STS3215 서보 6개 각각의 present-voltage 레지스터 (/dev/soarm)
  베이스/파이 전원 : MentorPi 베이스 보드 배터리 텔레메트리 (/dev/rrc)
"""
import sys
import time

sys.path.insert(0, "/third_party/soarm_provided_d/soarm_lab")
sys.path.insert(0, "/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller")


def check_arm(port="/dev/soarm"):
    from driver_sdk import STS3215Driver, JOINT_IDS, JOINT_NAMES

    print(f"[팔] {port}")
    drv = STS3215Driver(port=port)
    if not drv.connect():
        print("  연결 실패 (다른 프로세스가 포트를 점유 중일 수 있음)")
        return
    volts = []
    for sid, name in zip(JOINT_IDS, JOINT_NAMES):
        if not drv.ping(sid):
            print(f"  id{sid} {name:<12} 응답 없음")
            continue
        v = drv.get_voltage(sid)
        if v is not None:
            volts.append(v)
        print(f"  id{sid} {name:<12} {v}V")
    drv.disconnect()
    if volts:
        print(f"  → 평균 {sum(volts)/len(volts):.1f}V (최소 {min(volts):.1f}V)")


def check_base(device="/dev/rrc"):
    from ros_robot_controller_sdk import Board

    print(f"[베이스/파이] {device}")
    try:
        board = Board(device=device)
    except Exception as e:
        print(f"  열기 실패: {e}")
        return
    board.enable_reception(True)
    mv = None
    for _ in range(50):
        v = board.get_battery()
        if v:
            mv = v
            break
        time.sleep(0.1)
    if mv is None:
        print("  5초 내 배터리 패킷 응답 없음")
    else:
        print(f"  {mv} mV ({mv/1000:.2f}V)")
        print("  참고(2026-08-23 실측): 6944mV에서 베이스 정지, 7181mV에서 정상 주행")


if __name__ == "__main__":
    arm_port = sys.argv[1] if len(sys.argv) > 1 else "/dev/soarm"
    base_device = sys.argv[2] if len(sys.argv) > 2 else "/dev/rrc"
    check_arm(arm_port)
    print()
    check_base(base_device)
