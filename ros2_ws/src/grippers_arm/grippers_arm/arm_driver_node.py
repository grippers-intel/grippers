"""arm_driver_node — SO-ARM101 실제 하드웨어를 쥔 노드.
soarm_lab.arm을 그대로 감싼다. 새 IK/서보 로직은 없음."""

import sys
import time

sys.path.insert(0, "/third_party/soarm_provided_d")  # PYTHONPATH 미설정 환경 대비 안전장치

import rclpy
from grippers_interfaces.action import MoveToCartesian
from grippers_interfaces.srv import GetLoad, SetGripper
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from soarm_lab import arm as soarm


class ArmDriverNode(Node):
    def __init__(self):
        super().__init__("arm_driver_node")
        cb_group = ReentrantCallbackGroup()

        self._move_action_server = ActionServer(
            self,
            MoveToCartesian,
            "arm_driver/move_to_cartesian",
            execute_callback=self._execute_move,
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
        self.get_logger().info("arm_driver_node ready")

    def _execute_move(self, goal_handle):
        req = goal_handle.request
        xyz = [req.target.x, req.target.y, req.target.z]
        grip = req.grip if req.grip else None
        result = MoveToCartesian.Result()
        try:
            angles_deg, err = soarm.arm.go(
                xyz,
                grip=grip,
                real=True,
                down=req.down,
                secs=1.2,
            )
            time.sleep(1.2)  # RealBackend.move는 즉시 반환하므로 정착 시간만큼 대기
            result.reached = True
            result.distance_remaining = float(err)
            goal_handle.succeed()
        except ValueError as e:
            self.get_logger().warn(f"도달 불가: {e}")
            result.reached = False
            goal_handle.abort()
        return result

    def _on_set_gripper(self, request, response):
        try:
            soarm.arm.grip(request.closed and 0.0 or 100.0)  # TODO: 실제 열림/닫힘 각도로 교체
            response.ok = True
            response.load_ratio = self._read_load()
        except Exception as e:
            self.get_logger().error(f"set_gripper 실패: {e}")
            response.ok = False
        return response

    def _on_get_load(self, request, response):
        response.load_ratio = self._read_load()
        return response

    def _read_load(self) -> float:
        backend = soarm.arm._backend(real=True)
        load = backend.drv.get_load(6)
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
