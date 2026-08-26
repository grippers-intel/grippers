"""Ros2MecanumBase — mission_orchestrator가 쓰는 BaseDriver 포트 구현.

⚠️ 2026-08-26 팀 확정으로 이 어댑터가 크게 줄었다. 예전에는 `drive_to`(액션),
`approach_object`(액션), `align_to_box`(서비스)로 base_driver_node에 "어디로
갈지"를 넘겼는데, 그 판단이 전부 Host로 갔다. 남은 것은 속도를 그대로 내는
것과 멈추는 것, 그리고 GRASP 전용 미세 전진뿐이다.

속도는 액션이 아니라 **토픽**으로 낸다. Host가 사이클마다 새 속도를 보내므로
목표-결과 왕복이 필요 없고, 오히려 왕복 지연이 제어 주기를 늘린다."""

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from domain.adapters.real._ros_call import ESTOP_TIMEOUT_SEC, call_service
from domain.ports.base_driver import BaseDriver

# 미세 전진을 나누는 버스트 길이(초)와 속도(m/s).
#
# 데드밴드 때문에 속도를 낮춰서 짧게 갈 수 없다 — 0.05 m/s 아래로는 바퀴가
# 아예 안 도는데 /odom_raw는 움직였다고 보고한다(2026-08-24 실기). 실제로
# 도는 최저 속도로 **짧게 여러 번** 나눠 낸다. 2026-08-26 실기에서 이 방식의
# 이동량 예측이 실측과 0.5% 이내로 맞았다.
CREEP_SPEED_MPS = 0.06
CREEP_BURST_S = 0.35


class Ros2MecanumBase(BaseDriver):
    def __init__(self, node, clock_sleep=None):
        self._node = node
        self._cmd_pub = node.create_publisher(Twist, "cmd_vel", 10)
        self._stop_client = node.create_client(Trigger, "base_driver/stop")
        # 테스트에서 실제로 잠들지 않게 주입할 수 있도록 열어 둔다.
        self._sleep = clock_sleep

    def apply_velocity(self, linear_x: float, linear_y: float,
                       angular_z: float) -> None:
        """받은 속도를 cmd_vel로 낸다. 다시 자르지 않는다 — 한계 집행은
        `domain/task/motion.py` 한 곳에만 있어야 한다."""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.angular.z = float(angular_z)
        self._cmd_pub.publish(twist)

    def creep_forward(self, distance_m: float) -> bool:
        """정지 상태에서 이만큼 앞으로 밀고 멈춘다.

        버스트를 반복해 목표 거리를 채운다. 마지막 조각이 한 버스트보다
        짧아도 **버스트 하나를 다 낸다** — 데드밴드 아래로 잘게 쪼개면 그
        조각은 아예 움직이지 않기 때문이다. 그래서 실제 이동량은 목표보다
        한 버스트 안쪽에서 길어질 수 있고, 그 오차가 이 방식의 분해능이다."""
        if distance_m <= 0.0:
            return False
        if self._sleep is None:
            import time
            self._sleep = time.sleep

        burst_travel = CREEP_SPEED_MPS * CREEP_BURST_S
        bursts = max(1, int(round(distance_m / burst_travel)))
        try:
            for _ in range(bursts):
                self.apply_velocity(CREEP_SPEED_MPS, 0.0, 0.0)
                self._sleep(CREEP_BURST_S)
            self.stop()
        except Exception:                       # noqa: BLE001 -- 실기 경로
            self.stop()
            self._node.get_logger().error("creep_forward: 실패 — 정지")
            return False
        return True

    def stop(self) -> None:
        """즉시 정지. cmd_vel 0을 직접 내고, 노드 쪽 정지 서비스도 부른다.

        둘 다 하는 이유: cmd_vel 0은 이 프로세스에서 바로 나가 가장 빠르고,
        서비스는 base_driver_node가 자체 루프를 돌고 있을 때 그것까지 멈춘다.
        E-STOP 경로라 **응답을 기다리지 않는다.**"""
        self.apply_velocity(0.0, 0.0, 0.0)
        if not self._stop_client.wait_for_service(timeout_sec=ESTOP_TIMEOUT_SEC):
            # 서비스가 없어도 위의 cmd_vel 0은 이미 나갔다 — 치명적이지 않다.
            self._node.get_logger().warn("stop: base_driver/stop 서비스 없음 — cmd_vel 0만 냄")
            return
        self._stop_client.call_async(Trigger.Request())
