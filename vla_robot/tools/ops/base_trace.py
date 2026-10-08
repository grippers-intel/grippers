#!/usr/bin/env python3
"""차체 명령 경로 기록 — "명령했는데 안 움직인다"(첫 출발 무응답)가 어디서 끊기는지 보려고(2026-10-08).

경로:  pi_mission_node --/controller/cmd_vel--> odom_publisher --/ros_robot_controller/set_motor-->
       ros_robot_controller --시리얼--> 보드 --> 바퀴.   보드 쪽 읽기는 imu_raw(49 Hz)로 살아 있음을 본다.

이 노드는 **듣기만 한다**(아무것도 내지 않는다). 스택 기동(robot.launch.py, use_base_trace:=true 기본)이 같이 띄운다.
따로 띄우려면(use_base_trace:=false 로 기동했을 때) 컨테이너 안에서:

    docker exec -d IntelPi bash -lc "export ROS_DOMAIN_ID=21 && source /opt/ros/humble/setup.bash && \\
        source /ros2_ws/install/setup.bash && source /grippers/vla_deploy/vla_robot/ros2_ws/install/setup.bash && \\
        exec python3 /grippers/vla_deploy/vla_robot/tools/ops/base_trace.py"

/tmp/base_trace.csv 에 0.1 s 마다 한 줄(명령이 0 이 아니거나 멈춘 뒤 3 s 안) · 그 밖에는 1 s 마다 한 줄.
열: 벽시계 · 명령(선속·각속) · 마지막 명령 뒤 경과 · 마지막 set_motor 뒤 경과와 바퀴 rps · 자이로 z · 가속도 크기 ·
imu 마지막 수신 뒤 경과 · Pi 상태. 무응답이면 Host 로그의 시각(벽시계)으로 찾아 위아래를 본다:
  - set_motor 가 안 나왔다            → odom_publisher(명령 변환) 쪽
  - set_motor 는 나왔는데 자이로·가속도가 그대로 → 보드가 쓰기를 안 받는다(09-23 · 09-30 과 같은 고장)
  - imu 도 끊겼다                      → 시리얼/컨트롤러 노드 자체
"""
from __future__ import annotations

import math
import os
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros_robot_controller_msgs.msg import MotorsState
from sensor_msgs.msg import Imu
from vla_robot_interfaces.msg import RobotStatus

PATH = "/tmp/base_trace.csv"
MAX_BYTES = 20 * 1024 * 1024
ACTIVE_HOLD_S = 3.0
COLUMNS = "wall,cmd_lin,cmd_ang,cmd_age,motor_age,rps1,rps2,rps3,rps4,gyro_z,acc,imu_age,state,base_ok\n"


def _age(now: float, t: float | None) -> str:
    return f"{now - t:.2f}" if t is not None else ""


class BaseTrace(Node):
    def __init__(self) -> None:
        super().__init__("base_trace")
        self.cmd = (0.0, 0.0)
        self.cmd_at: float | None = None
        self.active_until = 0.0
        self.rps = ("", "", "", "")
        self.motor_at: float | None = None
        self.gyro_z, self.acc = 0.0, 0.0
        self.imu_at: float | None = None
        self.state, self.base_ok = "", ""
        self.last_idle_row = 0.0
        self._open()
        best = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Twist, "/controller/cmd_vel", self._on_cmd, 10)
        self.create_subscription(MotorsState, "/ros_robot_controller/set_motor", self._on_motor, 10)
        self.create_subscription(Imu, "/ros_robot_controller/imu_raw", self._on_imu, best)
        self.create_subscription(RobotStatus, "/vla_robot/status", self._on_status, 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"기록: {PATH}")

    def _open(self) -> None:
        new = not os.path.exists(PATH)
        self.f = open(PATH, "a", buffering=1)
        if new:
            self.f.write(COLUMNS)

    def _on_cmd(self, m: Twist) -> None:
        now = time.time()
        self.cmd, self.cmd_at = (m.linear.x, m.angular.z), now
        if abs(m.linear.x) > 1e-6 or abs(m.linear.y) > 1e-6 or abs(m.angular.z) > 1e-6:
            self.active_until = now + ACTIVE_HOLD_S

    def _on_motor(self, m: MotorsState) -> None:
        vals = {s.id: s.rps for s in m.data}
        self.rps = tuple(f"{vals[i]:.2f}" if i in vals else "" for i in (1, 2, 3, 4))
        self.motor_at = time.time()

    def _on_imu(self, m: Imu) -> None:
        a = m.linear_acceleration
        self.gyro_z = m.angular_velocity.z
        self.acc = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        self.imu_at = time.time()

    def _on_status(self, m: RobotStatus) -> None:
        self.state, self.base_ok = m.state, int(m.base_ok)

    def _tick(self) -> None:
        now = time.time()
        if now > self.active_until and now - self.last_idle_row < 1.0:
            return
        self.last_idle_row = now
        self.f.write(f"{now:.2f},{self.cmd[0]:.3f},{self.cmd[1]:.3f},{_age(now, self.cmd_at)},"
                     f"{_age(now, self.motor_at)},{','.join(self.rps)},{self.gyro_z:.3f},{self.acc:.2f},"
                     f"{_age(now, self.imu_at)},{self.state},{self.base_ok}\n")
        if self.f.tell() > MAX_BYTES:
            self.f.close()
            os.replace(PATH, PATH + ".1")
            self._open()


def main() -> None:
    rclpy.init()
    node = BaseTrace()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.f.close()


if __name__ == "__main__":
    main()
