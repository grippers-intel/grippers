# -*- coding: utf-8 -*-
"""README 미션 흐름 그대로: IDLE -> TRANSIT_OUT -> LIGHT_ADAPT -> DOCKING
-> IDENTIFY -> GRASP -> POSE_PLAN -> NARROW_EXIT -> RETURN -> RELEASE.
각 State.execute(ports)가 다음 State를 반환하는 제너레이터 체인."""
from domain.task.state import State
from domain.values import Pose2D


class IdleState(State):
    name = "IDLE"

    def execute(self, ports):
        return TransitOutState()


class TransitOutState(State):
    name = "TRANSIT_OUT"

    def __init__(self, target=None):
        # TODO: 실제 파지 지점 근처 목표 좌표로 교체
        self.target = target or Pose2D(x=1.0, y=0.0, theta=0.0)

    def execute(self, ports):
        arrived = ports.base.drive_to(self.target)
        if not arrived:
            return TransitOutState(self.target)  # 재시도
        return LightAdaptState()


class LightAdaptState(State):
    name = "LIGHT_ADAPT"

    def execute(self, ports):
        ports.perception.set_light_profile("default")
        return DockingState()


class DockingState(State):
    name = "DOCKING"

    def execute(self, ports):
        ports.base.align_to_centerline()
        return IdentifyState()


class IdentifyState(State):
    name = "IDENTIFY"

    def __init__(self, retries=0):
        self.retries = retries

    def execute(self, ports):
        found, pose, dims = ports.perception.detect_target()
        if not found:
            if self.retries >= 3:
                return IdentifyFailedState()
            return IdentifyState(self.retries + 1)
        return GraspState(target_pose=pose)


class GraspState(State):
    name = "GRASP"

    def __init__(self, target_pose):
        self.target_pose = target_pose

    def execute(self, ports):
        xyz = [self.target_pose.x, self.target_pose.y, self.target_pose.z]
        ok = ports.arm.move_to_cartesian(xyz, down=True)
        if not ok:
            return GraspFailedState()
        ports.arm.set_gripper(0.0)  # TODO: 실제 캘리브레이션 기준 닫힘 각도
        return PosePlanState()


class PosePlanState(State):
    name = "POSE_PLAN"

    def execute(self, ports):
        gap = ports.perception.measure_gap()
        phi = self._solve_phi(gap.h_gap)  # TODO: 실제 회피각 계산
        return NarrowExitState(phi=phi)

    def _solve_phi(self, h_gap):
        return 0.0


class NarrowExitState(State):
    name = "NARROW_EXIT"

    def __init__(self, phi):
        self.phi = phi

    def execute(self, ports):
        ports.arm.move_to_cartesian(self._phi_to_xyz(self.phi), down=False)
        home = Pose2D(x=0.0, y=0.0, theta=0.0)  # TODO: 실제 복귀 목표
        while True:
            clearance = ports.perception.monitor_clearance()
            if clearance.contact_risk:
                ports.base.stop()
                return NarrowExitFailedState()
            if ports.base.drive_to(home):
                break
        return ReturnState()

    def _phi_to_xyz(self, phi):
        return [0.2, 0.0, 0.15]  # TODO: φ -> 손끝 좌표 변환


class ReturnState(State):
    name = "RETURN"

    def execute(self, ports):
        return ReleaseState()


class ReleaseState(State):
    name = "RELEASE"

    def execute(self, ports):
        ports.arm.set_gripper(100.0)
        return None  # 정상 종료


class EstopState(State):
    name = "ESTOP"

    def execute(self, ports):
        ports.base.stop()
        return None


class NarrowExitFailedState(State):
    name = "NARROW_EXIT_FAILED"

    def execute(self, ports):
        ports.base.stop()
        return None


class GraspFailedState(State):
    name = "GRASP_FAILED"

    def execute(self, ports):
        return None


class IdentifyFailedState(State):
    name = "IDENTIFY_FAILED"

    def execute(self, ports):
        return None
