# -*- coding: utf-8 -*-
"""mission_orchestrator_node — domain/task의 FSM을 ROS2로 감싼다.
FSM 자체는 별도 스레드에서 순차 실행, rclpy는 MultiThreadedExecutor로
스핀해서 E-STOP이 FSM 블로킹 도중에도 즉시 들어올 수 있게 한다."""
import sys
import threading

sys.path.insert(0, '/grippers')  # PYTHONPATH 미설정 환경 대비 안전장치

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Empty
from grippers_interfaces.msg import MissionState

from domain.task.mission_task import MissionTask, Ports
from domain.adapters.real.ros2_mecanum_base import Ros2MecanumBase
from domain.adapters.real.ros2_arm_driver import Ros2ArmDriver
# TODO: Ros2Perception 어댑터 (perception 노드 만든 뒤 추가)
from domain.adapters.fake.fake_perception import FakePerception
from domain.adapters.real.ros2_perception import Ros2Perception


class MissionOrchestratorNode(Node):
    def __init__(self):
        super().__init__('mission_orchestrator')
        cb_group = ReentrantCallbackGroup()

        self._state_pub = self.create_publisher(
            MissionState, '/mission/state',
            qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
            ),
        )
        self.create_subscription(
            Empty, '/mission/emergency_stop', self._on_estop, 10,
            callback_group=cb_group,
        )
        self._estop_flag = threading.Event()
        self.declare_parameter('use_fake_perception', True)

        self._fsm_thread = threading.Thread(target=self._run_fsm, daemon=True)
        self._fsm_thread.start()
        self.get_logger().info('mission_orchestrator ready')

    def _on_estop(self, msg):
        self.get_logger().warn('EMERGENCY STOP received')
        self._estop_flag.set()

    def _run_fsm(self):
        ports = Ports(
            base=Ros2MecanumBase(self),
            arm=Ros2ArmDriver(self),
            perception=self._make_perception(),
            estop=self._estop_flag,
        )
        task = MissionTask(ports)
        for state in task.run():
            self.get_logger().info(f'[MISSION] -> {state.name}')
            msg = MissionState()
            msg.state = state.name
            self._state_pub.publish(msg)

    def _make_perception(self):
        use_fake = self.get_parameter('use_fake_perception').value
        if use_fake:
            self.get_logger().warn('use_fake_perception=True — FakePerception 사용 중')
            return FakePerception()
        return Ros2Perception(self)



def main(args=None):
    rclpy.init(args=args)
    node = MissionOrchestratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
