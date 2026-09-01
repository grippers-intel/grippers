"""Ros2MecanumBase — mission_orchestrator가 쓰는 BaseDriver 포트 구현.

⚠️ 2026-08-26 팀 확정으로 이 어댑터가 크게 줄었다. 예전에는 `drive_to`(액션),
`approach_object`(액션), `align_to_box`(서비스)로 base_driver_node에 "어디로
갈지"를 넘겼는데, 그 판단이 전부 Host로 갔다. 남은 것은 속도를 그대로 내는
것과 멈추는 것, 그리고 GRASP 전용 미세 전진뿐이다.

속도는 액션이 아니라 **토픽**으로 낸다. Host가 사이클마다 새 속도를 보내므로
목표-결과 왕복이 필요 없고, 오히려 왕복 지연이 제어 주기를 늘린다."""

import math
import os
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

# 제자리회전도 같은 데드밴드를 갖고 있었다(2026-09-01 발견) — 다만 creep_forward와
# 달리 회전은 **Host가 매 사이클 계속 명령하는 닫힌 루프**라 한 번 정해진 총
# 거리만큼 블로킹으로 미는 방식을 못 쓴다. 그래서 여기서는 `apply_velocity()`
# 호출마다(블로킹 없이) on/off를 토글하는 펄스(PWM과 같은 발상)로 낸다 —
# 회전 명령이 들어오는 한 계속 펄싱되고, Host가 다음 사이클에 방향을 바꾸거나
# 멈추면 그 즉시 반영된다.
#
# ⚠️ 발견 경위: 메카넘 역기구학(controller/mecanum.py)에 합의 회전속도
# AGREED_ROTATION_RAD_S(0.25 rad/s, domain/task/motion.py)를 넣으면 필요한
# 바퀴 선속도가 0.25 x (wheelbase+track_width)/2 = 0.0347 m/s — CREEP_SPEED_MPS
# 위 데드밴드(0.05m/s) **아래**다. "yaw- 명령이 계속 나가는데 한참 있다가
# 겨우 도는" 증상이 실기로 확인됐다(2026-09-01).
#
# ROTATE_BURST_SPEED_RAD_S는 그 역기구학을 거꾸로 풀어 데드밴드를 확실히
# 넘기는 값을 잡았다: 0.05 / ((wheelbase+track_width)/2) ≈ 0.36 rad/s가
# 최소선이고, 여유를 더해 0.4로 뒀다(바퀴 선속도 0.0556m/s, 데드밴드
# 대비 CREEP_SPEED_MPS와 비슷한 수준의 여유).
#
# on/off 비율은 **시간이 아니라 호출 횟수 기반 분수 누적기**(Bresenham 직선
# 알고리즘과 같은 방식)로 정한다 — 처음엔 벽시계 시간으로 주기를 나누려
# 했으나, `apply_velocity()`가 실제로 불리는 간격 자체가 이미 Pi FSM
# 사이클(CYCLE_PERIOD_S=0.1초)에 묶여 있어서 그보다 촘촘한 시간 주기를
# 잡으면 샘플링 앨리어싱으로 평균이 어긋난다(예: 0.2초 주기로 시도했더니
# 평균이 -0.35가 나와야 할 -0.25 대신 나옴 — 시뮬레이션으로 확인, 이번
# 구현에서 폐기). 호출마다 duty를 누적하다 1.0을 넘으면 그 호출만 켠다 —
# 실제로 불리는 간격이 얼마든 장기 평균이 정확히 요청값에 수렴한다.
#
# ⚠️ **아직 실기 미검증**이다. 이 설계가 전제하는 것은 "데드밴드가 순간
# 토크/속도 문턱이지, 지속시간 문턱이 아니다"인데(정지마찰을 넘는 순간
# 크기가 필요조건이라는 뜻) 이건 creep_forward의 기존 문서에서 유추한
# 것이지 회전축으로 직접 실측한 적은 없다. 내일 펄싱 중에도 여전히 안
# 돌면 이 전제부터 의심할 것 — 그땐 켜짐 구간을 몇 사이클 연속으로
# 묶는 식으로 바꿔야 할 수 있다.
ROTATE_BURST_SPEED_RAD_S = 0.4
_ROTATE_EPSILON = 1e-6

# 실기 미검증 기능이라 켜는 즉시 되돌릴 수단이 필요하다 — colcon build나
# git revert 없이, **재기동 시 환경변수 하나로** 펄싱 이전 동작(요청 각속도를
# 그대로 cmd_vel에 낸다, 데드밴드 아래면 이전처럼 안 돎)으로 되돌린다.
# 로봇 앞에서 문제가 보이면:
#     export GRIPPERS_DISABLE_ROTATE_BURST=1
# 을 export하고 mission_orchestrator만 재기동하면 된다(다른 노드는 안 건드림).
ROTATE_BURST_DISABLE_ENV = "GRIPPERS_DISABLE_ROTATE_BURST"


class Ros2MecanumBase(BaseDriver):
    def __init__(self, node, clock_sleep=None, clock=time.monotonic):
        self._node = node
        self._cmd_pub = node.create_publisher(Twist, "cmd_vel", 10)
        self._stop_client = node.create_client(Trigger, "base_driver/stop")
        # 정지 서비스가 없다는 경고를 사이클마다 찍지 않기 위한 래치.
        self._stop_service_missing = False
        # 테스트에서 실제로 잠들지 않게 주입할 수 있도록 열어 둔다.
        self._sleep = clock_sleep

        # --- 제자리회전 데드밴드 펄싱 (2026-09-01) ---
        self._rotate_accumulator = 0.0
        self._rotate_sign = 0.0
        self._rotate_burst_enabled = not os.environ.get(ROTATE_BURST_DISABLE_ENV)
        if not self._rotate_burst_enabled:
            node.get_logger().warn(
                f"{ROTATE_BURST_DISABLE_ENV} 설정됨 — 회전 펄싱 비활성화, "
                "요청 각속도를 그대로 낸다(데드밴드 아래면 2026-09-01 이전처럼 안 돎)")

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
        """받은 속도를 cmd_vel로 낸다. 크기는 다시 자르지 않는다 — 한계 집행은
        `domain/task/motion.py` 한 곳에만 있어야 한다.

        제자리회전(병진 없이 angular_z만 있는 경우)만 예외다. 요청한 각속도가
        ROTATE_BURST_SPEED_RAD_S보다 작으면 그 크기 그대로 내보내지 않고,
        더 빠른 속도로 껐다 켰다 하는 펄스로 바꾼다 — 그대로 내보내면 바퀴
        데드밴드 아래라 아무리 오래 줘도 안 돈다(모듈 상단 주석 참고). 몇 번째
        호출에서 켤지는 분수 누적기(`_rotate_accumulator`)로 정하므로, 호출
        간격이 정확히 일정하지 않아도 장기 평균 각속도는 요청값에 수렴한다."""
        is_pure_rotation = (abs(angular_z) >= _ROTATE_EPSILON
                            and abs(linear_x) < _ROTATE_EPSILON
                            and abs(linear_y) < _ROTATE_EPSILON)
        if (not self._rotate_burst_enabled or not is_pure_rotation
                or abs(angular_z) >= ROTATE_BURST_SPEED_RAD_S):
            self._rotate_accumulator = 0.0
            self._rotate_sign = 0.0
            self._publish(linear_x, linear_y, angular_z)
            return

        sign = math.copysign(1.0, angular_z)
        if sign != self._rotate_sign:
            # 새로 회전을 시작했거나 방향이 바뀌었다 — 누적기를 리셋해서
            # 이전 방향의 잔여 누적치가 새 방향으로 새지 않게 한다.
            self._rotate_accumulator = 0.0
            self._rotate_sign = sign
        duty = min(1.0, abs(angular_z) / ROTATE_BURST_SPEED_RAD_S)
        self._rotate_accumulator += duty
        if self._rotate_accumulator >= 1.0:
            self._rotate_accumulator -= 1.0
            self._publish(0.0, 0.0, sign * ROTATE_BURST_SPEED_RAD_S)
        else:
            self._publish(0.0, 0.0, 0.0)

    def _publish(self, linear_x: float, linear_y: float, angular_z: float) -> None:
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
