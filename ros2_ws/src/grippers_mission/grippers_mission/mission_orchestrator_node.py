"""
mission_orchestrator_node — domain/task의 FSM을 ROS2로 감싼다.
FSM 자체는 별도 스레드에서 순차 실행, rclpy는 MultiThreadedExecutor로
스핀해서 E-STOP이 FSM 블로킹 도중에도 즉시 들어올 수 있게 한다.

명령 입력: /command(std_msgs/String)를 구독해 큐에 쌓는다. FSM 스레드는
하나만 떠서 큐에서 블로킹으로 다음 명령을 기다렸다가 MissionTask.run(
raw_text)을 끝까지 돌리고, 끝나면 다시 큐를 기다린다 — 재실행(미션 완료
후 새 명령)이 이 루프 구조로 자연스럽게 된다. Ports/어댑터는 한 번만
만들어서 계속 재사용한다. voice_io 노드는 STT 결과를 그대로 이 토픽에
발행할 뿐이고, 복창(confirm_phrase)은 voice_io가 처리한다 — 도메인
FSM은 parse()만 안다(state_machine.md §3 IDLE 계약).
"""

import queue
import sys
import threading

sys.path.insert(0, "/grippers")  # PYTHONPATH 미설정 환경 대비 안전장치

import rclpy
from grippers_interfaces.msg import MissionState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String

# TODO: Ros2Perception 어댑터 (perception 노드 만든 뒤 추가)
from domain.adapters.fake.scripted_interpreter import ScriptedInterpreter
from domain.adapters.fake.scripted_perception import ScriptedPerception
from domain.adapters.real.ros2_arm_driver import Ros2ArmDriver
from domain.adapters.real.ros2_command_interpreter import Ros2CommandInterpreter
from domain.adapters.real.ros2_mecanum_base import Ros2MecanumBase
from domain.adapters.real.ros2_perception import Ros2Perception
from domain.task.mission_task import MissionTask, Ports


class MissionOrchestratorNode(Node):
    def __init__(self):
        super().__init__("mission_orchestrator")
        cb_group = ReentrantCallbackGroup()

        self._state_pub = self.create_publisher(
            MissionState,
            "/mission/state",
            qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
            ),
        )
        self.create_subscription(
            Empty,
            "/mission/emergency_stop",
            self._on_estop,
            10,
            callback_group=cb_group,
        )
        self._command_queue = queue.Queue()
        self.create_subscription(
            String,
            "/command",
            self._on_command,
            10,
            callback_group=cb_group,
        )
        self._estop_flag = threading.Event()
        self.declare_parameter("use_fake_perception", True)
        self.declare_parameter("use_fake_interpreter", True)

        self._fsm_thread = threading.Thread(target=self._run_fsm, daemon=True)
        self._fsm_thread.start()
        self.get_logger().info("mission_orchestrator ready")

    def _on_estop(self, msg):
        self.get_logger().warn("EMERGENCY STOP received")
        self._estop_flag.set()

    def _on_command(self, msg):
        self.get_logger().info(f"[COMMAND] 큐에 추가: {msg.data!r}")
        self._command_queue.put(msg.data)

    def _run_fsm(self):
        ports = Ports(
            base=Ros2MecanumBase(self),
            arm=Ros2ArmDriver(self),
            perception=self._make_perception(),
            interpreter=self._make_interpreter(),
            estop=self._estop_flag,
        )
        task = MissionTask(ports)
        while rclpy.ok():
            raw_text = self._command_queue.get()  # 다음 명령이 올 때까지 블로킹 대기
            self.get_logger().info(f"[MISSION] 시작: {raw_text!r}")
            for state in task.run(raw_text):
                self.get_logger().info(f"[MISSION] -> {state.name}")
                msg = MissionState()
                msg.state = state.name
                self._state_pub.publish(msg)

    def _make_perception(self):
        use_fake = self.get_parameter("use_fake_perception").value
        if use_fake:
            self.get_logger().warn("use_fake_perception=True — ScriptedPerception 사용 중")
            return ScriptedPerception()
        return Ros2Perception(self)

    def _make_interpreter(self):
        use_fake = self.get_parameter("use_fake_interpreter").value
        if use_fake:
            self.get_logger().warn("use_fake_interpreter=True — ScriptedInterpreter 사용 중")
            return ScriptedInterpreter()
        return Ros2CommandInterpreter(self)


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


if __name__ == "__main__":
    main()
