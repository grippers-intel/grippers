"""arm_driver_node — SO-ARM101 시리얼 버스의 **유일한 소유자**.

다른 노드는 절대 시리얼을 직접 열지 않는다. 필요한 것은 전부 아래 인터페이스로 받는다.

    srv    arm/get_state        (GetArmState)        관절 raw + 정책 단위 + 부하/온도/전압
    srv    arm/set_gripper      (SetGripper)         그리퍼 0..100
    srv    arm/set_torque       (std_srvs/SetBool)
    srv    arm/hold             (std_srvs/Trigger)   지금 자세에서 멈춘다
    action arm/execute_chunk    (ExecuteJointChunk)  정책 청크 재생
    action arm/move_to_pose     (MoveToPose)         이름 포즈/목표로 보간 이동

## 기동 시 거부하는 경우 (추측해서 움직이지 않는다)

1. 포트가 차체 보드(/dev/rrc)를 가리킨다 — 팔 드라이버가 주행 보드 시리얼을 열어 버리던 사고.
2. 서보 6개 중 하나라도 응답이 없다.
3. 서보 EEPROM 의 Homing_Offset 이 arm_calibration.json 과 다르다.
   → 정책 좌표계가 어긋난다(기존 코드에서 wrist_roll 85.8도). tools/write_calibration.py 로 맞춘다.

## 동시에 하나의 동작만

청크 재생·포즈 이동·그리퍼 이동은 `_motion_lock` 으로 하나만 돈다. 이미 도는 중이면
새 요청은 즉시 거부된다(기다리게 하면 E-STOP 뒤에 옛 명령이 실행된다).
상태 읽기(get_state)는 동작 중에도 된다 — 버스 락이 패킷 단위로 직렬화한다.
"""
from __future__ import annotations

import math
import os
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from vla_common.arm_units import (GRIPPER_INDEX, NUM_JOINTS, SERVO_IDS, ArmCalibration,
                                  degrees_to_raw_delta)
from vla_common.config import load_poses, load_robot_config
from vla_robot_arm.feetech_bus import FeetechBus, describe_error
from vla_robot_interfaces.action import ExecuteJointChunk, MoveToPose
from vla_robot_interfaces.srv import GetArmState, SetGripper

BASE_BOARD_DEVICE = "/dev/rrc"
#: 포즈 이동 후 "도착"으로 보는 오차(도).
POSE_ARRIVE_TOL = 3.0
#: 그리퍼(%)는 따로 둔다. TPU 패드가 닫힘 끝단에서 천천히 눌려 관절보다 늦게 정착한다
#: (2026-09-22 실측: idle 로 이동 후 3초 안에 12.2% 까지만 들어갔다).
#: 여기서 엄격하게 잡으면 파지 전 start_pose 이동이 매번 실패로 판정된다 —
#: 물체를 쥐었는지는 grasp_check 가 따로 보므로 이 값이 느슨해도 잃는 것이 없다.
POSE_ARRIVE_TOL_GRIPPER = 8.0
POSE_SETTLE_MAX_S = 2.0
GRIPPER_SETTLE_MAX_S = 2.0


class StartupError(RuntimeError):
    pass


class ArmDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_driver_node")
        self.declare_parameter("config_file", "")
        path = str(self.get_parameter("config_file").value or "")
        if not path:
            raise StartupError("config_file 파라미터가 필요하다 (robot.yaml 경로)")
        self.cfg = load_robot_config(path)
        acfg = self.cfg.arm
        self.calib = ArmCalibration.load(acfg.calibration_file)
        self.poses = load_poses(acfg.poses_file)
        self.get_logger().info(
            f"설정 {path} / 캘리브레이션 {acfg.calibration_file} / 포즈 {sorted(self.poses)}")

        self._reject_base_board_port(acfg.port)
        self.bus = FeetechBus(acfg.port, acfg.baudrate, retries=acfg.read_retries).open()
        try:
            self._check_servos_online()
            if acfg.verify_homing_offsets:
                self._verify_homing_offsets()
            else:
                self.get_logger().warn("verify_homing_offsets=false — 정책 좌표계를 검증하지 않는다")
            # 토크를 켜기 전에 목표를 현재 위치로 맞춘다. 안 그러면 RAM 에 남은 옛 목표로 튄다.
            self._hold_here()
            self.bus.set_goal_velocity(SERVO_IDS, 0)
            if acfg.torque_on_start:
                self.bus.set_torque(SERVO_IDS, True)
        except Exception:
            self.bus.close()
            raise

        self._motion_lock = threading.Lock()
        cb = ReentrantCallbackGroup()
        self.create_service(GetArmState, "arm/get_state", self._on_get_state, callback_group=cb)
        self.create_service(SetGripper, "arm/set_gripper", self._on_set_gripper, callback_group=cb)
        self.create_service(SetBool, "arm/set_torque", self._on_set_torque, callback_group=cb)
        self.create_service(Trigger, "arm/hold", self._on_hold, callback_group=cb)
        self._chunk_server = ActionServer(
            self, ExecuteJointChunk, "arm/execute_chunk", execute_callback=self._execute_chunk,
            goal_callback=self._accept_if_idle, cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb)
        self._pose_server = ActionServer(
            self, MoveToPose, "arm/move_to_pose", execute_callback=self._execute_pose,
            goal_callback=self._accept_if_idle, cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb)
        self.get_logger().info("arm_driver_node 준비 완료")

    # ------------------------------------------------------------------ 기동 검사
    def _reject_base_board_port(self, port: str) -> None:
        if os.path.exists(BASE_BOARD_DEVICE) and os.path.exists(port):
            if os.path.realpath(port) == os.path.realpath(BASE_BOARD_DEVICE):
                raise StartupError(
                    f"arm.port={port} 가 차체 보드 {BASE_BOARD_DEVICE} 와 같은 장치다. "
                    "udev 규칙을 확인할 것 (udev/99-vla-robot.rules)")

    def _check_servos_online(self) -> None:
        missing = [sid for sid in SERVO_IDS if not self.bus.ping(sid)]
        if missing:
            raise StartupError(f"응답 없는 서보: {missing} — 전원(3S LiPo)과 버스 배선을 확인할 것")

    def _verify_homing_offsets(self) -> None:
        expected = self.calib.homing_offsets
        wrong = []
        for sid in SERVO_IDS:
            live = self.bus.read_homing_offset(sid)
            if live != expected[sid]:
                wrong.append(f"servo {sid}: EEPROM {live} != 캘리브레이션 {expected[sid]}")
        if wrong:
            raise StartupError(
                "서보 Homing_Offset 이 캘리브레이션과 다르다 — 정책 좌표계가 어긋나므로 움직이지 않는다.\n  "
                + "\n  ".join(wrong)
                + "\n  맞추려면: python3 tools/write_calibration.py --config <robot.yaml> --apply")
        self.get_logger().info("Homing_Offset 6개가 캘리브레이션과 일치")

    # ------------------------------------------------------------------ 도우미
    def _read_raw(self) -> list[int]:
        raw = self.bus.read_positions(SERVO_IDS)
        if raw is None:
            raise RuntimeError("관절 위치 읽기 실패 (버스 패킷 유실 반복)")
        return raw

    def _read_policy(self) -> list[float]:
        return self.calib.raw_to_policy(self._read_raw())

    def _hold_here(self) -> None:
        raw = self._read_raw()
        self.bus.write_goal_positions(dict(zip(SERVO_IDS, raw)))

    def _check_voltage(self) -> None:
        v = self.bus.read_voltage(SERVO_IDS[0])
        if v is not None and v < self.cfg.arm.min_voltage_v:
            raise RuntimeError(f"서보 전압 {v:.1f}V < {self.cfg.arm.min_voltage_v:.1f}V — 배터리 충전 필요")

    def _check_temperature(self) -> None:
        """이동 전 서보 온도. 읽히는 것만 본다 — 못 읽은 서보 때문에 이동을 막지는 않는다.

        그리퍼는 닫힘 끝단을 토크로 누르고 있으면 계속 달궈진다. 서보 자체 보호(70°C 근처)가
        걸리면 토크가 끊겨 팔이 떨어지므로, 그 앞에서 우리가 먼저 멈춘다."""
        limit = self.cfg.arm.max_servo_temp_c
        hot = []
        for servo_id in SERVO_IDS:
            temp = self.bus.read_temperature(servo_id)
            if temp is not None and temp >= limit:
                hot.append(f"servo {servo_id} {temp}°C")
        if hot:
            raise RuntimeError(f"서보 온도 상한({limit:.0f}°C) 초과: {', '.join(hot)} — 식을 때까지 대기")

    def _servo_error_note(self) -> str:
        errs = {sid: describe_error(e) for sid, e in self.bus.stats.last_error.items() if e}
        return f" 서보 오류비트 {errs}" if errs else ""

    def _accept_if_idle(self, _goal) -> GoalResponse:
        if self._motion_lock.locked():
            self.get_logger().warn("다른 팔 동작이 진행 중이라 새 goal 을 거부한다")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # ------------------------------------------------------------------ 서비스
    def _on_get_state(self, _req, res):
        res.ok = False
        try:
            raw, online = [0] * NUM_JOINTS, [False] * NUM_JOINTS
            for i, sid in enumerate(SERVO_IDS):
                pos = self.bus.read_position(sid)
                if pos is not None:
                    raw[i], online[i] = pos, True
                load = self.bus.read_load_ratio(sid)
                res.load_ratio[i] = float(load) if load is not None else 0.0
                temp = self.bus.read_temperature(sid)
                res.temperature_c[i] = int(temp) if temp is not None else 0
            res.online = online
            res.position_raw = raw
            volt = self.bus.read_voltage(SERVO_IDS[0])
            res.voltage_v = float(volt) if volt is not None else 0.0
            torque = self.bus.read_torque(SERVO_IDS[GRIPPER_INDEX])
            res.torque_on = bool(torque)
            if all(online):
                res.policy_state = [float(v) for v in self.calib.raw_to_policy(raw)]
                res.ok = True
                res.message = "ok" + self._servo_error_note()
            else:
                res.message = f"읽기 실패 servo {[s for s, o in zip(SERVO_IDS, online) if not o]}"
        except Exception as exc:  # noqa: BLE001 — 서비스는 죽지 않는다
            res.message = f"예외: {exc}"
        return res

    def _on_set_gripper(self, req, res):
        res.ok = False
        if not self._motion_lock.acquire(blocking=False):
            res.message = "다른 팔 동작이 진행 중"
            return res
        try:
            self._check_voltage()
            self._check_temperature()
            target = min(100.0, max(0.0, float(req.percent)))
            start = self._read_policy()
            goal = list(start)
            goal[GRIPPER_INDEX] = target
            span = abs(target - start[GRIPPER_INDEX])
            duration = max(0.2, span / max(self.cfg.arm.gripper_speed_pct_s, 1.0))
            self._interpolate(start, goal, duration, cancel=None, feedback=None)
            final, load = self._settle_gripper(target)
            res.final_percent = float(final)
            res.load_ratio = float(load)
            res.ok = True
            res.message = f"그리퍼 {start[GRIPPER_INDEX]:.1f} -> {final:.1f}% 부하 {load:.2f}"
        except Exception as exc:  # noqa: BLE001
            res.message = f"그리퍼 실패: {exc}"
            self.get_logger().error(res.message)
        finally:
            self._motion_lock.release()
        return res

    def _settle_gripper(self, target: float) -> tuple[float, float]:
        """물체를 쥐면 목표에 못 간다 — 위치 변화가 멈추면 도착으로 본다."""
        deadline = time.monotonic() + GRIPPER_SETTLE_MAX_S
        last = None
        while True:
            time.sleep(0.1)
            pct = self._read_policy()[GRIPPER_INDEX]
            if abs(pct - target) < 1.0 or (last is not None and abs(pct - last) < 0.2):
                break
            if time.monotonic() > deadline:
                break
            last = pct
        load = self.bus.read_load_ratio(SERVO_IDS[GRIPPER_INDEX]) or 0.0
        return pct, load

    def _on_set_torque(self, req, res):
        try:
            if req.data:
                self._hold_here()
            ok = self.bus.set_torque(SERVO_IDS, bool(req.data))
            res.success, res.message = ok, ("토크 켬" if req.data else "토크 끔 — 팔이 처질 수 있다")
        except Exception as exc:  # noqa: BLE001
            res.success, res.message = False, str(exc)
        return res

    def _on_hold(self, _req, res):
        try:
            self._hold_here()
            res.success, res.message = True, "현재 자세 유지"
        except Exception as exc:  # noqa: BLE001
            res.success, res.message = False, str(exc)
        return res

    # ------------------------------------------------------------------ 청크 재생
    def _execute_chunk(self, goal_handle):
        req = goal_handle.request
        result = ExecuteJointChunk.Result()
        if not self._motion_lock.acquire(blocking=False):
            result.ok, result.message = False, "다른 팔 동작이 진행 중"
            goal_handle.abort()
            return result
        sent = clamped_steps = 0
        try:
            count = len(req.positions)
            if req.fps <= 0.0:
                raise ValueError(f"fps 는 양수여야 한다: {req.fps}")
            if count == 0 or count % NUM_JOINTS:
                raise ValueError(f"positions 길이는 6의 배수여야 한다: {count}")
            steps = count // NUM_JOINTS
            if steps > self.cfg.arm.max_chunk_steps:
                raise ValueError(f"청크가 너무 길다: {steps} > {self.cfg.arm.max_chunk_steps}")
            values = [float(v) for v in req.positions]
            if not all(math.isfinite(v) for v in values):
                raise ValueError("positions 에 NaN/Inf 가 있다")
            max_step_deg = req.max_step_deg if req.max_step_deg > 0 else self.cfg.arm.max_step_deg
            max_step_raw = max(1, round(degrees_to_raw_delta(max_step_deg)))

            self._check_voltage()
            self._check_temperature()
            # ⚠️ 재생마다 명시적으로 다시 쓴다. 다른 동작이 남긴 속도 제한이 RAM 에 남아
            # 팔이 정책을 못 따라가던 사고가 있었다(2026-09-02).
            self.bus.set_goal_velocity(SERVO_IDS, 0)
            self.bus.set_acceleration(SERVO_IDS, self.cfg.arm.chunk_acceleration)

            # 스텝 제한 기준은 처음 한 번만 실측하고 이후엔 보낸 목표를 잇는다.
            # 30Hz 로 매번 6개를 되읽으면 버스가 못 버틴다.
            last = self._read_raw()
            period = 1.0 / float(req.fps)
            feedback = ExecuteJointChunk.Feedback()
            started = time.monotonic()
            for step in range(steps):
                if goal_handle.is_cancel_requested:
                    self._hold_here()
                    goal_handle.canceled()
                    result.ok, result.message = False, f"취소됨 {sent}/{steps}"
                    result.steps_executed, result.clamped_steps = sent, clamped_steps
                    return result
                goal = self.calib.policy_to_raw(values[step * NUM_JOINTS:(step + 1) * NUM_JOINTS])
                was_clamped = False
                for i in range(NUM_JOINTS):
                    delta = goal[i] - last[i]
                    if abs(delta) > max_step_raw:
                        goal[i] = last[i] + (max_step_raw if delta > 0 else -max_step_raw)
                        was_clamped = True
                self.bus.write_goal_positions(dict(zip(SERVO_IDS, goal)))
                last = goal
                sent = step + 1
                clamped_steps += int(was_clamped)
                feedback.step = step
                goal_handle.publish_feedback(feedback)
                # 절대 시각 기준으로 잔다 — 쓰기 시간이 누적돼 청크가 늘어나지 않게.
                remaining = started + (step + 1) * period - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
            elapsed = time.monotonic() - started
            result.ok = True
            result.message = (f"{sent} 스텝 {elapsed:.2f}s (요청 {steps / req.fps:.2f}s), "
                              f"스텝 제한 {clamped_steps}회{self._servo_error_note()}")
            if clamped_steps > steps // 2:
                self.get_logger().warn(
                    f"청크 {clamped_steps}/{steps} 스텝이 max_step_deg={max_step_deg:.1f} 에 걸렸다 — "
                    "궤적이 눌리고 있다")
            goal_handle.succeed()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"청크 재생 실패: {exc}")
            try:
                self._hold_here()
            except Exception:  # noqa: BLE001
                pass
            result.ok, result.message = False, str(exc)
            goal_handle.abort()
        finally:
            result.steps_executed, result.clamped_steps = sent, clamped_steps
            self._motion_lock.release()
        return result

    # ------------------------------------------------------------------ 포즈 이동
    def _interpolate(self, start, goal, duration: float, cancel, feedback) -> bool:
        """정책 단위 선형 보간(smoothstep). 취소되면 False."""
        rate = max(self.cfg.arm.pose_rate_hz, 5.0)
        n = max(1, int(math.ceil(duration * rate)))
        started = time.monotonic()
        for k in range(1, n + 1):
            if cancel is not None and cancel():
                self._hold_here()
                return False
            s = k / n
            s = s * s * (3.0 - 2.0 * s)
            point = [a + (b - a) * s for a, b in zip(start, goal)]
            self.bus.write_goal_positions(dict(zip(SERVO_IDS, self.calib.policy_to_raw(point))))
            if feedback is not None:
                feedback(k / n)
            remaining = started + k / rate - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
        return True

    def _execute_pose(self, goal_handle):
        req = goal_handle.request
        result = MoveToPose.Result()
        if not self._motion_lock.acquire(blocking=False):
            result.ok, result.message = False, "다른 팔 동작이 진행 중"
            goal_handle.abort()
            return result
        try:
            if req.pose_name:
                pose = self.poses.get(req.pose_name)
                if pose is None:
                    raise ValueError(f"모르는 포즈: {req.pose_name} (있는 것: {sorted(self.poses)})")
                if not pose.measured:
                    raise ValueError(f"포즈 '{req.pose_name}' 는 아직 실측되지 않았다 — tools/teach_pose.py")
                target = list(pose.values)
            else:
                target = [float(v) for v in req.target]
            if not all(math.isfinite(v) for v in target):
                raise ValueError("목표에 NaN/Inf 가 있다")
            self._check_voltage()
            self._check_temperature()
            self.bus.set_goal_velocity(SERVO_IDS, 0)
            start = self._read_policy()
            if req.gripper_mode == 1:
                target[GRIPPER_INDEX] = start[GRIPPER_INDEX]
            span = max(abs(b - a) for a, b in zip(start[:GRIPPER_INDEX], target[:GRIPPER_INDEX]))
            duration = req.duration_s if req.duration_s > 0 else max(
                0.5, span / max(self.cfg.arm.pose_speed_deg_s, 1.0))
            fb = MoveToPose.Feedback()

            def publish(progress: float) -> None:
                fb.progress = float(progress)
                goal_handle.publish_feedback(fb)

            if not self._interpolate(start, target, duration,
                                     cancel=lambda: goal_handle.is_cancel_requested, feedback=publish):
                goal_handle.canceled()
                result.ok, result.message = False, "취소됨"
                return result
            final = self._wait_arrival(target, req.gripper_mode == 1)
            result.final_policy_state = [float(v) for v in final]
            errors = [abs(a - b) for a, b in zip(final, target)]
            if req.gripper_mode == 1:
                errors[GRIPPER_INDEX] = 0.0
            worst = max(errors)
            name = req.pose_name or "target"
            if not self._arrived(errors, slack=2.0):
                result.ok = False
                result.message = f"{name} 도착 실패: 최대 오차 {worst:.1f} (관절 {errors.index(worst) + 1})"
                goal_handle.abort()
            else:
                result.ok = True
                result.message = f"{name} 도착 {duration:.1f}s, 최대 오차 {worst:.1f}"
                goal_handle.succeed()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"포즈 이동 실패: {exc}")
            try:
                self._hold_here()
            except Exception:  # noqa: BLE001
                pass
            result.ok, result.message = False, str(exc)
            goal_handle.abort()
        finally:
            self._motion_lock.release()
        return result

    @staticmethod
    def _arrived(errs, slack: float = 1.0) -> bool:
        """관절과 그리퍼를 다른 기준으로 본다(POSE_ARRIVE_TOL_GRIPPER 주석 참고)."""
        return (max(errs[:GRIPPER_INDEX]) <= POSE_ARRIVE_TOL * slack
                and errs[GRIPPER_INDEX] <= POSE_ARRIVE_TOL_GRIPPER * slack)

    def _wait_arrival(self, target, ignore_gripper: bool) -> list[float]:
        deadline = time.monotonic() + POSE_SETTLE_MAX_S
        while True:
            now = self._read_policy()
            errs = [abs(a - b) for a, b in zip(now, target)]
            if ignore_gripper:
                errs[GRIPPER_INDEX] = 0.0
            if self._arrived(errs) or time.monotonic() > deadline:
                return now
            time.sleep(0.1)

    def shutdown(self) -> None:
        """전원을 끄기 전까지 팔이 떨어지지 않게 토크는 유지하고 현재 자세를 잡는다."""
        try:
            self._hold_here()
        except Exception:  # noqa: BLE001
            pass
        self.bus.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[ArmDriverNode] = None
    try:
        node = ArmDriverNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        print(f"[arm_driver_node] 기동 실패: {exc}")
        raise
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
