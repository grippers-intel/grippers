"""Ros2Perception — mission_orchestrator가 쓰는 Perception 포트 구현.
perception_node에 서비스로 말을 건다. geometry_msgs <-> domain.values 변환은
여기서만 한다 (domain은 ROS2 타입을 모름)."""

from types import SimpleNamespace

import rclpy
from grippers_interfaces.srv import (
    DetectTarget,
    MeasureGap,
    MonitorClearance,
    SetLightProfile,
)

from domain.ports.perception import Perception
from domain.values import Point3, Pose2D


class Ros2Perception(Perception):
    def __init__(self, node):
        self._node = node
        self._detect_client = node.create_client(DetectTarget, "perception/detect_target")
        self._gap_client = node.create_client(MeasureGap, "perception/measure_gap")
        self._light_client = node.create_client(SetLightProfile, "perception/set_light_profile")
        self._clearance_client = node.create_client(
            MonitorClearance, "perception/monitor_clearance"
        )

    def detect_target(self):
        self._detect_client.wait_for_service()
        future = self._detect_client.call_async(DetectTarget.Request())
        rclpy.spin_until_future_complete(self._node, future)
        res = future.result()
        pose = Point3(x=res.pose.position.x, y=res.pose.position.y, z=res.pose.position.z)
        dims = Point3(x=res.dims.x, y=res.dims.y, z=res.dims.z)
        return res.found, pose, dims

    def measure_gap(self):
        self._gap_client.wait_for_service()
        future = self._gap_client.call_async(MeasureGap.Request())
        rclpy.spin_until_future_complete(self._node, future)
        res = future.result()
        centerline = Pose2D(x=res.centerline.x, y=res.centerline.y, theta=res.centerline.theta)
        return SimpleNamespace(h_gap=res.h_gap, centerline=centerline)

    def set_light_profile(self, profile: str) -> bool:
        self._light_client.wait_for_service()
        req = SetLightProfile.Request(profile=profile)
        future = self._light_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().ready

    def monitor_clearance(self):
        self._clearance_client.wait_for_service()
        future = self._clearance_client.call_async(MonitorClearance.Request())
        rclpy.spin_until_future_complete(self._node, future)
        res = future.result()
        return SimpleNamespace(
            front=res.front,
            left=res.left,
            right=res.right,
            top=res.top,
            contact_risk=res.contact_risk,
        )
