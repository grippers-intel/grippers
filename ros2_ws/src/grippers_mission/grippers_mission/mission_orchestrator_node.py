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
import traceback

sys.path.insert(0, "/grippers")  # PYTHONPATH 미설정 환경 대비 안전장치

import rclpy
from grippers_interfaces.msg import MissionState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.scripted_interpreter import ScriptedInterpreter
from domain.adapters.fake.scripted_perception import ScriptedPerception
from domain.adapters.logged_port import LoggedPort
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
        self.declare_parameter("use_fake_base", True)
        self.declare_parameter("use_fake_arm", True)
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
            base=self._logged("BaseDriver", self._make_base()),
            arm=self._logged("ArmDriver", self._make_arm()),
            perception=self._logged("Perception", self._make_perception()),
            interpreter=self._logged("CommandInterpreter", self._make_interpreter()),
            estop=self._estop_flag,
        )
        task = MissionTask(ports)
        while rclpy.ok():
            raw_text = self._command_queue.get()  # 다음 명령이 올 때까지 블로킹 대기
            self.get_logger().info(f"[MISSION] 시작: {raw_text!r}")
            try:
                for state in task.run(raw_text):
                    self.get_logger().info(f"[MISSION] -> {state.name}")
                    # MissionState.msg 는 상태 **이름**만 싣는다. 관측 실패처럼
                    # 원인이 상태에 실려 오는 경우, 여기서 한 번 찍지 않으면
                    # 무엇이 끊겼는지가 어디에도 남지 않는다 (이슈 #194).
                    reason = getattr(state, "reason", None)
                    if reason:
                        self.get_logger().error(f"[MISSION] {state.name} 사유: {reason}")
                    msg = MissionState()
                    msg.state = state.name
                    self._state_pub.publish(msg)
            except Exception:
                # 이 except가 없으면 FSM 스레드만 조용히 죽고 노드는 계속 스핀한다 —
                # /command는 큐에 쌓이기만 하고 /mission/state는 끊겨서, 밖에서는
                # 멈춘 이유를 알 수 없다. 미션 하나만 버리고 루프는 살려 둔다.
                self._abort_mission(ports)

    def _abort_mission(self, ports):
        """미션 실행 중 예외를 수습하고 IDLE로 돌아간다.

        순서가 중요하다 — **로봇을 먼저 세운다**. 예외는 FSM이 어디까지 진행한
        뒤에 날지 알 수 없고(SCAN 이후라면 베이스가 이미 움직이는 중이다),
        오케스트레이터가 지시를 멈춰도 base_driver/arm_driver는 별도 프로세스라
        진행 중인 액션을 알아서 취소하지 않는다.

        정지 호출 자체도 실패할 수 있으므로 각각 따로 감싼다 — base.stop()이
        실패해도 arm.hold_position()은 시도해야 한다. 두 포트 모두 E-STOP
        경로라 서비스가 없어도 0.5초 안에 돌아온다.

        노드는 내리지 않는다. bringup.launch.py에 respawn 설정이 없어 한 번
        내리면 복구되지 않고, 명령 하나가 잘못됐다고 로봇 전체를 못 쓰게 되는
        것은 과하다."""
        # rclpy 로거는 logging.Logger가 아니라 exc_info= 를 받지 않는다.
        self.get_logger().fatal(
            f"[MISSION] 예외로 중단 — 정지 후 IDLE 복귀\n{traceback.format_exc()}"
        )

        try:
            ports.base.stop()
        except Exception:
            self.get_logger().error(
                f"[MISSION] 중단 처리 중 base.stop() 실패\n{traceback.format_exc()}"
            )
        try:
            ports.arm.hold_position()
        except Exception:
            self.get_logger().error(
                f"[MISSION] 중단 처리 중 arm.hold_position() 실패\n{traceback.format_exc()}"
            )

        msg = MissionState()
        msg.state = "IDLE"
        self._state_pub.publish(msg)

    def _logged(self, name, adapter):
        return LoggedPort(name, adapter, self.get_logger())

    def _make_base(self):
        use_fake = self.get_parameter("use_fake_base").value
        if use_fake:
            self.get_logger().warn("use_fake_base=True — FakeBase 사용 중")
            return FakeBase()
        return Ros2MecanumBase(self)

    def _make_arm(self):
        use_fake = self.get_parameter("use_fake_arm").value
        if use_fake:
            self.get_logger().warn("use_fake_arm=True — FakeArm 사용 중")
            return FakeArm()
        return Ros2ArmDriver(self)

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
