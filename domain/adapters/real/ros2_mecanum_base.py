"""Ros2MecanumBase — mission_orchestrator가 쓰는 BaseDriver 포트 구현.

⚠️ 2026-08-26 팀 확정으로 이 어댑터가 크게 줄었다. 예전에는 `drive_to`(액션),
`approach_object`(액션), `align_to_box`(서비스)로 base_driver_node에 "어디로
갈지"를 넘겼는데, 그 판단이 전부 Host로 갔다. 남은 것은 속도를 그대로 내는
것과 멈추는 것, 그리고 GRASP 전용 미세 전진뿐이다.

속도는 액션이 아니라 **토픽**으로 낸다. Host가 사이클마다 새 속도를 보내므로
목표-결과 왕복이 필요 없고, 오히려 왕복 지연이 제어 주기를 늘린다."""

import time

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from domain.ports.base_driver import BaseDriver
from domain.task import base_liveness

# 구동계가 살아 있음을 증명하는 토픽. 값은 쓰지 않고 **도착 시각만** 본다 —
# `/odom_raw`는 명령을 적분할 뿐이라 값 자체로는 바퀴가 도는지 알 수 없지만,
# 발행이 끊긴다는 것은 그 노드의 루프가 멈췄다는 뜻이라 물림을 잡아낸다
# (2026-08-28 정지 실패 사고, base_liveness 모듈 docstring 참고).
FEEDBACK_TOPIC = "odom_raw"

# 미세 전진을 나누는 버스트 길이(초)와 속도(m/s).
#
# 데드밴드 때문에 속도를 낮춰서 짧게 갈 수 없다 — 0.05 m/s 아래로는 바퀴가
# 아예 안 도는데 /odom_raw는 움직였다고 보고한다(2026-08-24 실기). 실제로
# 도는 최저 속도로 **짧게 여러 번** 나눠 낸다. 2026-08-26 실기에서 이 방식의
# 이동량 예측이 실측과 0.5% 이내로 맞았다.
CREEP_SPEED_MPS = 0.06
CREEP_BURST_S = 0.35


class Ros2MecanumBase(BaseDriver):
    def __init__(self, node, clock_sleep=None, clock=time.monotonic):
        self._node = node
        self._cmd_pub = node.create_publisher(Twist, "cmd_vel", 10)
        self._stop_client = node.create_client(Trigger, "base_driver/stop")
        # 정지 서비스가 없다는 경고를 사이클마다 찍지 않기 위한 래치.
        self._stop_service_missing = False
        # 테스트에서 실제로 잠들지 않게 주입할 수 있도록 열어 둔다.
        self._sleep = clock_sleep

        # --- 구동계 생존 감시 (2026-08-28) ---
        self._clock = clock
        self._born = clock()
        self._feedback_at = None
        self._feedback_sub = None
        try:
            from nav_msgs.msg import Odometry
        except ImportError:
            # 피드백 토픽을 못 구독해도 구독자 수 신호는 그대로 살아 있다.
            # 감시를 통째로 포기하는 것보다 한 눈으로 보는 편이 낫다.
            node.get_logger().warn(
                "nav_msgs 없음 — 구동계 피드백 감시 비활성화 "
                "(cmd_vel 구독자 수만으로 판정한다)")
        else:
            self._feedback_sub = node.create_subscription(
                Odometry, FEEDBACK_TOPIC, self._on_feedback, 10)

    def _on_feedback(self, _msg):
        """내용은 안 본다. **도착했다는 사실만** 기록한다."""
        self._feedback_at = self._clock()

    def liveness(self):
        """`cmd_vel` 아래가 명령을 받아 갈 상태인가.

        ⚠️ 논블로킹이어야 한다. 이 함수는 미션 루프가 매 사이클 부른다 —
        여기서 기다리면 `stop()`의 `wait_for_service`가 루프를 10Hz에서
        1.6Hz로 떨어뜨렸던 사고를 그대로 재현한다(아래 stop() 주석 참고).
        `get_subscription_count()`도 콜백 카운터 조회도 둘 다 즉시 돌아온다."""
        now = self._clock()
        try:
            subscribers = self._cmd_pub.get_subscription_count()
        except Exception:                       # noqa: BLE001 -- 실기 경로
            subscribers = None
        age = None if self._feedback_at is None else now - self._feedback_at
        if self._feedback_sub is None:
            # 피드백을 못 보는 구성 — 나이를 "없음"이 아니라 "안 봄"으로
            # 다뤄야 한다. None을 그대로 넘기면 판정이 STALE로 굳는다.
            age = 0.0
        return base_liveness.judge(subscribers, age, now - self._born)

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

        버스트를 반복해 목표 거리를 채운다. 한 버스트가 약 21mm이고 그보다
        잘게 못 쪼갠다 — 데드밴드 아래 속도는 아무리 오래 줘도 안 움직인다.

        ⚠️ 목표가 반 버스트보다 짧으면 **아무것도 하지 않고 성공을 돌려준다.**
        예전에는 최소 한 버스트를 강제했는데, 2026-08-26 실측으로 그게
        위험하다는 것이 드러났다. 턱 목의 깊이가 23mm라, 5mm만 가면 되는
        상황에서 21mm를 밀면 물체를 턱 안쪽 끝까지 처박고 계속 민다.
        반올림해서 0이 나오면 남은 거리가 최대 10mm인데, 그건 턱 목 안이라
        그냥 두는 편이 낫다."""
        if distance_m <= 0.0:
            return False
        if self._sleep is None:
            import time
            self._sleep = time.sleep

        burst_travel = CREEP_SPEED_MPS * CREEP_BURST_S
        bursts = int(round(distance_m / burst_travel))
        if bursts == 0:
            self._node.get_logger().info(
                f"creep_forward: 목표 {distance_m * 1000:.0f}mm가 반 버스트보다 "
                f"짧다 — 움직이지 않는다(턱 목 깊이 23mm 안)")
            return True
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
        # ⚠️ `wait_for_service`가 아니라 `service_is_ready`다. 둘의 차이가
        # 이 노드의 제어 주기를 통째로 바꾼다.
        #
        # 2026-08-28 실기: base_driver_node를 안 띄운 구성에서 이 경로가
        # 사이클마다 0.5초(_ros_call.ESTOP_TIMEOUT_SEC)를 꼬박 기다렸다. 오케스트레이터
        # 루프는 설계상 10Hz인데 실측 1.6Hz로 떨어졌고, Host가 한두 사이클만
        # 머무는 상태(APPROACH_PIECE)를 Pi가 통째로 놓쳤다 — 명령이 안 온 게
        # 아니라 읽을 때가 되기 전에 다음 명령으로 덮인 것이다. 워치독
        # 3사이클도 0.3초가 아니라 1.8초가 됐다.
        #
        # 이 함수의 계약은 원래 "응답을 기다리지 않는다"였다(아래 docstring).
        # 기다림을 없애는 것이 그 계약을 지키는 쪽이다. 서비스가 있으면
        # `service_is_ready()`가 True를 주므로 정상 구성의 동작은 그대로다.
        if not self._stop_client.service_is_ready():
            # 서비스가 없어도 위의 cmd_vel 0은 이미 나갔다 — 치명적이지 않다.
            # 경고는 상태가 바뀔 때만 남긴다. 사이클마다 찍으면 로그가 이걸로
            # 가득 차서 정작 봐야 할 줄이 묻힌다.
            if not self._stop_service_missing:
                self._node.get_logger().warn(
                    "stop: base_driver/stop 서비스 없음 — cmd_vel 0만 냄 "
                    "(서비스가 생기면 다시 알린다)")
                self._stop_service_missing = True
            return
        if self._stop_service_missing:
            self._node.get_logger().info("stop: base_driver/stop 서비스 복구됨")
            self._stop_service_missing = False
        self._stop_client.call_async(Trigger.Request())
