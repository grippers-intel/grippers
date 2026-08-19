"""arm_driver_node — SO-ARM101 실제 하드웨어를 쥔 노드.
soarm_lab.arm을 그대로 감싼다. 새 IK/서보 로직은 없음.

⚠️ 두 가지를 반드시 지킬 것 (디버깅으로 찾아낸 함정):

1. `soarm.grip()` 은 `real` 인자를 받지 않고 내부에서 항상
   `self._backend(False)` (SimBackend)를 쓴다 — 실물 명령으로 절대
   못 쓴다. 실물 그리퍼는 `soarm._backend(real=True).drv.set_position(6, ...)`
   로 서보에 직접 명령해야 한다 (soarm_lab/arm.py의 Arm.grip 정의 참고).

2. 실물 포트는 `arm_port` 노드 파라미터로 받는다 — 심볼릭 링크로 포트를
   고정하던 방식은 폐기했다. `Arm._backend(real=True)` 는 `self._real` 이
   비어 있을 때만 기본 포트(`/dev/soarm`)로 `RealBackend()` 를 새로
   만들므로, 커스텀 포트를 쓰려면 첫 호출 전에 `soarm._real` 을 미리
   채워 둬야 한다 — __init__ 에서 그렇게 한다.
"""

import os
import sys
import time

sys.path.insert(0, "/third_party/soarm_provided_d")  # PYTHONPATH 미설정 환경 대비 안전장치

import rclpy  # noqa: I001
import rclpy.logging  # main()이 노드 생성 실패 시 노드 없이 로그를 남긴다
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

# STS3215 PRESENT_LOAD 는 하위 10비트가 크기(0~1023), 0x400 비트가 방향이다
# (third_party/soarm_provided_d/soarm_lab/driver_sdk.py get_load).
# 도메인 계층은 0~1 비율만 안다 — 서보 각도 변환과 같은 이유로 원시값 →
# 비율 변환은 이 노드가 담당한다 (class_diagram.md §2).
GRIPPER_LOAD_MAX_RAW = 1023.0

# 그리퍼 닫힘 명령 후 부하가 정착할 때까지의 대기 시간.
# 실측(2026-08-18): 0.26~0.51s 는 이동 중 포화(±500)라 빈 채와 물체가 구분되지
# 않고, 0.77s 에 거의 안정, 1.03s 이후로는 10초까지 한 틱도 변하지 않았다.
# 1.0s 안정 + 여유로 1.5s. 정착 타이밍은 서보 물리 지식이므로 도메인이 아니라
# 여기 둔다 — GraspState 에 sleep 을 넣지 않는다.
GRASP_SETTLE_SEC = 1.5

CRADLE_XYZ_M = [0.15, 0.0, 0.20]  # TODO: 실측 — 이동용 거치 자세 손끝 좌표

# MentorPi 베이스 보드가 잡는 장치. arm_port가 이걸 가리키면 팔 드라이버가
# 베이스 보드의 시리얼을 열어 버려 주행 명령이 통째로 깨진다.
BASE_BOARD_DEVICE = "/dev/rrc"


class ArmPortConflictError(RuntimeError):
    """arm_port가 베이스 보드 장치를 가리킬 때 __init__에서 올린다.
    main()이 잡아서 노드를 띄우지 않고 종료한다."""


class ArmHardwareUnavailableError(RuntimeError):
    """SO-ARM101 또는 서보 버스가 응답하지 않거나 동작 불가 상태일 때 발생한다."""


class ArmDriverNode(Node):
    def __init__(self):
        super().__init__("arm_driver_node")
        cb_group = ReentrantCallbackGroup()

        self.declare_parameter("arm_port", "/dev/soarm")
        self.declare_parameter("enable_torque_on_start", False)

        arm_port = self.get_parameter("arm_port").value
        enable_torque_on_start = bool(self.get_parameter("enable_torque_on_start").value)

        # RealBackend(port=...) 는 생성 즉시 시리얼 포트를 연다 — 검사는 반드시
        # 그 앞에 와야 한다. 뒤에 두면 이미 베이스 보드를 열어 버린 뒤가 된다.
        self._reject_base_board_port(arm_port)
        soarm._real = RealBackend(port=arm_port)
        if not soarm._real.drv.is_connected():
            raise ArmHardwareUnavailableError(f"SO-ARM101 serial connection failed: {arm_port}")

        self.get_logger().info(f"arm_port={arm_port}")

        self._check_startup_torque(
            soarm._real,
            enable_torque_on_start=enable_torque_on_start,
        )

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

    def _check_startup_torque(
        self,
        backend,
        *,
        enable_torque_on_start: bool,
    ) -> None:
        """기동 시 6개 서보의 통신·torque 상태를 확인한다."""
        states = {servo_id: backend.drv.get_torque(servo_id) for servo_id in ALL_SERVO_IDS}

        unreadable = [servo_id for servo_id, enabled in states.items() if enabled is None]
        if unreadable:
            raise ArmHardwareUnavailableError(
                "SO-ARM101 torque 상태를 읽지 못했습니다 — " f"servo IDs: {unreadable}"
            )

        disabled = [servo_id for servo_id, enabled in states.items() if not enabled]
        if not disabled:
            self.get_logger().info("SO-ARM101 torque 상태 정상 — all enabled")
            return

        self.get_logger().warn(f"torque OFF servo IDs: {disabled}")

        if not enable_torque_on_start:
            self.get_logger().warn(
                "enable_torque_on_start=False — 자동 torque enable을 수행하지 않습니다"
            )
            return

        self.get_logger().warn(
            "enable_torque_on_start=True — 1초 후 전체 servo torque를 활성화합니다"
        )
        time.sleep(1.0)

        enable_failed = [
            servo_id for servo_id in ALL_SERVO_IDS if not backend.drv.set_torque(servo_id, True)
        ]
        if enable_failed:
            raise ArmHardwareUnavailableError(
                "SO-ARM101 torque enable 명령 실패 — " f"servo IDs: {enable_failed}"
            )

        still_disabled = [
            servo_id for servo_id in ALL_SERVO_IDS if backend.drv.get_torque(servo_id) is not True
        ]
        if still_disabled:
            raise ArmHardwareUnavailableError(
                "torque enable 후 상태 확인 실패 — " f"servo IDs: {still_disabled}"
            )

        self.get_logger().info("SO-ARM101 torque enable 완료")

    def _require_operational_servos(self, servo_ids=ALL_SERVO_IDS) -> None:
        """모션 전후 대상 서보가 통신 가능하고 torque ON인지 확인한다."""
        backend = soarm._backend(real=True)

        unavailable = []
        torque_off = []

        for servo_id in servo_ids:
            enabled = backend.drv.get_torque(servo_id)
            if enabled is None:
                unavailable.append(servo_id)
            elif not enabled:
                torque_off.append(servo_id)

        if unavailable:
            raise ArmHardwareUnavailableError(
                "SO-ARM101 servo 통신 실패 — " f"servo IDs: {unavailable}"
            )

        if torque_off:
            raise ArmHardwareUnavailableError("SO-ARM101 torque OFF — " f"servo IDs: {torque_off}")

    def _reject_base_board_port(self, arm_port: str) -> None:
        """arm_port가 MentorPi 베이스 보드와 같은 장치를 가리키면 노드를 띄우지 않는다.

        udev 규칙에 따라 /dev/rrc 는 /dev/ttyACM0 같은 실제 장치를 가리키는 심볼릭
        링크라, 경로 문자열만 비교하면 충돌을 놓친다 — realpath로 양쪽을 풀어서
        같은 장치인지 본다.

        /dev/rrc 가 없는 환경(개발 머신, CI, 시뮬레이션)에서는 비교할 대상이 없으니
        검사를 건너뛴다 — 베이스 보드가 없다면 충돌할 것도 없다."""
        if not os.path.exists(BASE_BOARD_DEVICE):
            return
        if os.path.realpath(arm_port) != os.path.realpath(BASE_BOARD_DEVICE):
            return
        raise ArmPortConflictError(
            f"arm_port({arm_port})가 {BASE_BOARD_DEVICE}(MentorPi 베이스 보드)와 같은 "
            f"장치입니다. SO-ARM101 포트를 확인하세요 — 이 상태로 진행하면 베이스 보드 "
            f"시리얼 통신이 깨집니다."
        )

    def _execute_move(self, goal_handle):
        req = goal_handle.request
        xyz = [req.target.x, req.target.y, req.target.z]
        result = MoveToCartesian.Result()
        try:
            self._require_operational_servos(range(1, 6))

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

            # 명령 직후 USB가 빠지거나 torque가 꺼진 경우도 성공으로 보고하지 않는다.
            # vendored RealBackend.move()는 servo write 결과를 반환하지 않으므로
            # ROS2 경계에서 통신 상태를 다시 확인한다.
            self._require_operational_servos(range(1, 6))

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
            self._require_operational_servos((GRIPPER_SERVO_ID,))

            # soarm.grip()을 쓰지 않는다 — 항상 SimBackend를 움직이는 함정이 있다
            # (모듈 docstring 참고). 실물 백엔드의 드라이버에 직접 명령한다.
            backend = soarm._backend(real=True)
            if not backend.drv.set_position(GRIPPER_SERVO_ID, raw_position):
                self.get_logger().error("set_gripper 실패: servo 6 position write 실패")
                response.ok = False
                response.load_ratio = 0.0
                return response

            # 정착 전에 읽으면 빈 채와 물체가 구분되지 않는다 — 이동 중에는
            # 양쪽 다 포화값(±500)이 나온다. set_position()은 즉시 반환하므로
            # 여기서 기다린다.
            #
            # 이 대기는 응답의 load_ratio 뿐 아니라 **그 다음 get_load() 를 위한
            # 것이기도 하다.** GraspState 는 set_gripper() 응답을 쓰지 않고
            # get_load() 를 따로 부르는데(ArmDriver.set_gripper 는 None 반환),
            # 그 호출이 정착 후에 도착하려면 set_gripper 가 정착까지 붙들고
            # 있어야 한다.
            #
            # 개방(OPEN_MM) 명령에도 똑같이 걸린다 — 개폐 판정이 필요 없는
            # 경로에서는 순수 지연이지만, 분기해서 "어떤 호출은 안 기다린다"를
            # 만드는 것보다 한 가지 계약(반환 시점 = 정착 완료)이 낫다.
            time.sleep(GRASP_SETTLE_SEC)
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
            self._require_operational_servos(range(1, 6))

            soarm.go(CRADLE_XYZ_M, grip=None, real=True, down=False, secs=1.2)
            time.sleep(1.2)

            self._require_operational_servos(range(1, 6))
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
            failed_ids = [
                servo_id for servo_id in ALL_SERVO_IDS if not backend.drv.set_torque(servo_id, True)
            ]
            if failed_ids:
                response.success = False
                response.message = "torque enable 실패 — " f"servo IDs: {failed_ids}"
                self.get_logger().error(response.message)
                return response

            response.success = True
        except Exception as e:
            self.get_logger().error(f"hold_position 실패: {e}")
            response.success = False
            response.message = str(e)
        return response

    def _read_load(self, servo_id: int = GRIPPER_SERVO_ID) -> float:
        """서보 원시 부하값을 0~1 비율로 정규화해 돌려준다.

        ⚠️ 부호를 버리고 **절대값**을 쓴다. PRESENT_LOAD 의 0x400 비트는 부하의
        방향인데, 실측에서 같은 '빈 채로 닫음' 조건이 음수(-88)로도 양수로도
        나와 방향이 일관되지 않았다. 파지 판정에 필요한 건 '얼마나 버티고
        있나'(크기)이지 어느 쪽으로 밀리나(방향)가 아니므로 크기만 본다.

        읽기에 실패하면(None) 0.0 — 즉 '빈 채'로 본다. 부하를 못 읽는 상태에서
        파지 성공으로 판정해 물체를 든 줄 알고 이송하는 것보다, 실패로 보고
        재시도하는 쪽이 안전하다."""
        backend = soarm._backend(real=True)
        raw = backend.drv.get_load(servo_id)
        if raw is None:
            self.get_logger().warn(f"servo {servo_id} load read 실패 — 안전값 0.0으로 처리")
            return 0.0
        return abs(raw) / GRIPPER_LOAD_MAX_RAW


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArmDriverNode()
    except (ArmPortConflictError, ArmHardwareUnavailableError) as e:
        # 노드를 띄우면 안 되는 설정/하드웨어 오류다. 장치가 없거나 서보 버스가
        # 응답하지 않는데 계속 실행하면 이후 명령이 성공처럼 보일 수 있다.
        rclpy.logging.get_logger("arm_driver_node").fatal(str(e))
        rclpy.shutdown()
        return 1
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
