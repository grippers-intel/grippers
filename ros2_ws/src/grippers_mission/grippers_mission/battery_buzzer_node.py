"""battery_buzzer_node — 배터리 전압이 낮으면 STM32 부저로 짧게 경고한다.

판정 로직은 domain/task/battery_alert.py(순수 함수)에 있다 — 여기는
`/ros_robot_controller/battery`를 구독해서 그 함수에 넘기고, 결과가 있으면
`/ros_robot_controller/set_buzzer`로 내보내는 얇은 어댑터일 뿐이다."""

import sys
import time

sys.path.insert(0, "/grippers")  # PYTHONPATH 미설정 환경 대비 안전장치

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from ros_robot_controller_msgs.msg import BuzzerState  # noqa: E402
from std_msgs.msg import UInt16  # noqa: E402

from domain.task.battery_alert import BatteryAlertState, check_battery  # noqa: E402


class BatteryBuzzerNode(Node):
    def __init__(self):
        super().__init__("battery_buzzer_node")
        self._state = BatteryAlertState()
        self._buzzer_pub = self.create_publisher(
            BuzzerState, "/ros_robot_controller/set_buzzer", 5
        )
        self.create_subscription(
            UInt16, "/ros_robot_controller/battery", self._on_battery, 5
        )
        self.get_logger().info(
            "battery_buzzer_node 시작 — 경고 문턱 domain/task/battery_alert.py 참고"
        )

    def _on_battery(self, msg: UInt16) -> None:
        now = time.monotonic()
        self._state, cmd = check_battery(float(msg.data), now, self._state)
        if cmd is None:
            return
        out = BuzzerState()
        out.freq = cmd.freq
        out.on_time = cmd.on_time
        out.off_time = cmd.off_time
        out.repeat = cmd.repeat
        self._buzzer_pub.publish(out)
        self.get_logger().warn(f"배터리 낮음 경고 — {msg.data}mV")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BatteryBuzzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
