"""arm_driver_node — SO-ARM101 실제 하드웨어를 쥔 노드.
soarm_lab.arm을 그대로 감싼다. 새 IK/서보 로직은 없음.

⚠️ 두 가지를 반드시 지킬 것 (디버깅으로 찾아낸 함정):

1. `soarm.grip()` 은 `real` 인자를 받지 않고 내부에서 항상
   `self._backend(False)` (SimBackend)를 쓴다 — 실물 명령으로 절대
   못 쓴다. 실물 그리퍼는 `soarm._backend(real=True).drv.set_position(6, ...)`
   로 서보에 직접 명령해야 한다 (soarm_lab/arm.py의 Arm.grip 정의 참고).

2. 실물 포트는 `arm_port` 노드 파라미터로 받는다 — 심볼릭 링크로 포트를
   고정하던 방식은 폐기했다. `Arm._backend(real=True)` 는 `self._real` 이
   비어 있을 때만 기본 포트(`/dev/ttyACM0`)로 `RealBackend()` 를 새로
   만들므로, 커스텀 포트를 쓰려면 첫 호출 전에 `soarm._real` 을 미리
   채워 둬야 한다 — __init__ 에서 그렇게 한다.
"""

import sys
import time

sys.path.insert(0, "/third_party/soarm_provided_d")  # PYTHONPATH 미설정 환경 대비 안전장치

import rclpy  # noqa: I001
from grippers_interfaces.action import MoveToCartesian, ReorientArm
from grippers_interfaces.srv import GetLoad, SetGripper
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger

# 아래 3개는 순서가 고정이다(위 import rclpy의 noqa: I001이 이 블록 전체의
# 자동 정렬을 막아 준다) — 알파벳 순으로 바꾸면 깨진다.
# soarm 자체가 Arm() 싱글턴이다 — 이 뒤로는 soarm.go()이지 soarm.arm.go()가 아니다.
# soarm_lab을 import하면 soarm_lab/__init__.py가 자기 디렉터리를 sys.path에 얹어
# 둬서(내부 모듈끼리 flat import 되게) 그 다음부터 real/driver_sdk를 이렇게 바로
# 가져올 수 있다 — 반드시 soarm_lab import 다음에 와야 한다.
from soarm_lab import arm as soarm
from driver_sdk import JOINT_LIMITS, position_from_fraction
from real import RealBackend

WRIST_SERVO_ID = 4
GRIPPER_SERVO_ID = 6
ALL_SERVO_IDS = range(1, 7)

# TODO: 미결 #4 (엔드이펙터 개구 폭 실측) — 아래 두 상수는 자리 표시자다.
# domain/task/states.py의 OPEN_MM/CLOSED_MM과 반드시 같은 값을 유지할 것 —
# 도메인 계층이 이 범위로 폭(mm)을 계산해 보내므로, 여기서 다른 범위로
# 해석하면 "닫으라고 보낸 명령이 살짝 벌어진 채로 멈추는" 식의 조용한
# 단위 불일치가 생긴다.
GRIPPER_CLOSED_MM = 0.0
GRIPPER_OPEN_MM = 90.0

CRADLE_XYZ_M = [0.15, 0.0, 0.20]  # TODO: 실측 — 이동용 거치 자세 손끝 좌표


class ArmDriverNode(Node):
    def __init__(self):
        super().__init__("arm_driver_node")
        cb_group = ReentrantCallbackGroup()

        self.declare_parameter("arm_port", "/dev/ttyACM0")
        arm_port = self.get_parameter("arm_port").value
        soarm._real = RealBackend(port=arm_port)
        self.get_logger().info(f"arm_port={arm_port}")

        self._move_action_server = ActionServer(
            self,
            MoveToCartesian,
            "arm_driver/move_to_cartesian",
            execute_callback=self._execute_move,
            callback_group=cb_group,
        )
        self._reorient_action_server = ActionServer(
            self,
            ReorientArm,
            "arm_driver/reorient",
            execute_callback=self._execute_reorient,
            callback_group=cb_group,
        )
        self.create_service(
            SetGripper,
            "arm_driver/set_gripper",
            self._on_set_gripper,
            callback_group=cb_group,
        )
        self.create_service(
            GetLoad,
            "arm_driver/get_load",
            self._on_get_load,
            callback_group=cb_group,
        )
        self.create_service(
            Trigger,
            "arm_driver/fold_to_cradle",
            self._on_fold_to_cradle,
            callback_group=cb_group,
        )
        self.create_service(
            Trigger,
            "arm_driver/hold_position",
            self._on_hold_position,
            callback_group=cb_group,
        )
        self.get_logger().info("arm_driver_node ready")

    def _execute_move(self, goal_handle):
        req = goal_handle.request
        xyz = [req.target.x, req.target.y, req.target.z]
        result = MoveToCartesian.Result()
        try:
            # grip=None — 그리퍼는 이 액션이 건드리지 않는다. GRASP는
            # move_to_cartesian()과 set_gripper()를 분리 호출한다
            # (state_machine.md §3 GRASP 계약).
            angles_deg, err = soarm.go(
                xyz,
                grip=None,
                real=True,
                down=req.down,
                secs=1.2,
            )
            time.sleep(1.2)  # RealBackend.move는 즉시 반환하므로 정착 시간만큼 대기
            # ⚠️ MoveToCartesian.action의 Result에는 distance_remaining이 없다
            # (Feedback에만 있음) — IK 잔차는 로그로만 남긴다.
            self.get_logger().info(f"move_to_cartesian 완료 — IK 잔차 {err * 1000:.1f}mm")
            result.reached = True
            goal_handle.succeed()
        except ValueError as e:
            # Arm.go()가 IK 잔차 초과 시 던지는 예외 — "팔 범위 밖" 같은 정상적인
            # 실패 경로다.
            self.get_logger().warn(f"도달 불가: {e}")
            result.reached = False
            goal_handle.abort()
        except Exception as e:
            # 시리얼 연결 끊김 등 하드웨어 예외 — 노드가 죽으면 안 되므로 여기서
            # 잡아 실패 응답으로 돌린다.
            self.get_logger().error(f"move_to_cartesian 하드웨어 오류: {e}")
            result.reached = False
            goal_handle.abort()
        return result

    def _execute_reorient(self, goal_handle):
        # TODO: 손목 φ 재조정 IK/모션 로직. POSE_PLAN이 아직 ⏸ 보류라 phi는
        # 지금 항상 0.0으로 들어온다(domain/task/states.py PosePlanState.
        # _solve_phi). soarm_lab에는 손목 단독 회전 프리미티브가 없어 φ≠0을
        # 실제로 지원하려면 이 노드에서 직접 φ 제약을 포함한 IK를 풀어야
        # 한다 — 재도입 시 구현. 지금은 정직하게 스텁으로 항상 성공 응답.
        phi = goal_handle.request.phi
        self.get_logger().warn(
            f"reorient(phi={phi:.3f}rad): 손목 재조정 미구현 — settled=True로 스텁 응답"
        )
        result = ReorientArm.Result()
        result.settled = True
        result.current_phi = phi
        result.wrist_load = self._read_load(WRIST_SERVO_ID)
        goal_handle.succeed()
        return result

    def _on_set_gripper(self, request, response):
        width_mm = max(GRIPPER_CLOSED_MM, min(GRIPPER_OPEN_MM, request.width_mm))
        fraction = (width_mm - GRIPPER_CLOSED_MM) / (GRIPPER_OPEN_MM - GRIPPER_CLOSED_MM)
        raw_position = position_from_fraction(fraction, JOINT_LIMITS[GRIPPER_SERVO_ID])
        try:
            # soarm.grip()을 쓰지 않는다 — 항상 SimBackend를 움직이는 함정이 있다
            # (모듈 docstring 참고). 실물 백엔드의 드라이버에 직접 명령한다.
            backend = soarm._backend(real=True)
            backend.drv.set_position(GRIPPER_SERVO_ID, raw_position)
            response.ok = True
            response.load_ratio = self._read_load()
        except Exception as e:
            self.get_logger().error(f"set_gripper 실패: {e}")
            response.ok = False
            response.load_ratio = 0.0
        return response

    def _on_get_load(self, request, response):
        response.load_ratio = self._read_load()
        return response

    def _on_fold_to_cradle(self, request, response):
        try:
            soarm.go(CRADLE_XYZ_M, grip=None, real=True, down=False, secs=1.2)
            time.sleep(1.2)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"fold_to_cradle 실패: {e}")
            response.success = False
            response.message = str(e)
        return response

    def _on_hold_position(self, request, response):
        # 현재 관절 자세를 래치한다 — E-STOP 시 파지물 낙하 방지용
        # (states.py EstopState가 호출). 새 목표를 보내지 않고 전 관절
        # 토크를 켜 두는 것만으로 STS3215가 지금 위치를 유지한다.
        try:
            backend = soarm._backend(real=True)
            for servo_id in ALL_SERVO_IDS:
                backend.drv.set_torque(servo_id, True)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"hold_position 실패: {e}")
            response.success = False
            response.message = str(e)
        return response

    def _read_load(self, servo_id: int = GRIPPER_SERVO_ID) -> float:
        backend = soarm._backend(real=True)
        load = backend.drv.get_load(servo_id)
        return float(load) if load is not None else 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ArmDriverNode()
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
