"""Host 가 보낸 속도를 Pi 가 실제로 낼 속도로 바꾼다. 순수 계산이다.

## 왜 Pi 가 자르는가

바퀴를 돌리는 쪽이 한계를 집행해야 한다. Host 의 버그나 패킷 손상으로 1.0 m/s 가
오더라도 여기서 잘린다. 방향은 그대로 두고 크기만 자른다.

## 기존 실측에서 가져온 값의 근거 (hardware/grippers/domain/task/motion.py)

- 직진 데드밴드가 배터리 전압을 따라 오른다: 8.4V 에서 0.10 m/s 가 겨우 움직이고
  8.0V 에서는 안 움직였다. 그래서 기본 0.15 m/s.
- 회전 데드밴드: 0.25 rad/s 는 안 돌고 0.35 부터 돈다.
- ⚠️ 벤더 `odom_publisher` 는 `cmd_vel` 토픽을 **±0.5 rad/s 로 자른다.** 기존 팀이
  "0.6 으로 약 20°/s" 라고 잰 값은 실제로 0.5 가 적용된 결과다. 이 프로젝트는
  자르지 않는 `controller/cmd_vel` 로 내므로, 같은 물리 동작을 위해 기본값을 0.5 로
  둔다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

EPSILON = 1e-6


@dataclass(frozen=True)
class MotionLimits:
    max_linear_mps: float = 0.15
    max_angular_rad_s: float = 0.5
    # 제자리회전에 병진이 섞인 명령을 거부할지. Host 경로계획은 둘을 섞지 않는다.
    reject_mixed_rotation: bool = True


@dataclass(frozen=True)
class Motion:
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0

    @property
    def is_stop(self) -> bool:
        return (abs(self.linear_x) < EPSILON and abs(self.linear_y) < EPSILON
                and abs(self.angular_z) < EPSILON)


STOP = Motion()


@dataclass(frozen=True)
class MotionDecision:
    ok: bool            # False 면 명령을 거부했다(motion 은 정지)
    motion: Motion
    clamped: bool = False
    reason: str = ""


def _clamp(value: float, limit: float) -> tuple[float, bool]:
    if abs(value) < EPSILON:
        return 0.0, False
    clipped = math.copysign(min(abs(value), limit), value)
    return clipped, abs(clipped) + EPSILON < abs(value)


def resolve_motion(linear_x: float, linear_y: float, angular_z: float, stop: bool,
                   limits: MotionLimits) -> MotionDecision:
    """우선순위: stop > 섞인 명령 거부 > 크기 제한."""
    if stop:
        return MotionDecision(True, STOP, reason="stop")
    values = (linear_x, linear_y, angular_z)
    if not all(math.isfinite(v) for v in values):
        return MotionDecision(False, STOP, reason=f"유한하지 않은 속도: {values}")

    rotating = abs(angular_z) >= EPSILON
    translating = abs(linear_x) >= EPSILON or abs(linear_y) >= EPSILON
    if limits.reject_mixed_rotation and rotating and translating:
        return MotionDecision(
            False, STOP,
            reason=f"제자리회전에 병진이 섞였다 (vx={linear_x:.3f}, vy={linear_y:.3f}, "
                   f"wz={angular_z:.3f})")

    vx, cx = _clamp(linear_x, limits.max_linear_mps)
    vy, cy = _clamp(linear_y, limits.max_linear_mps)
    wz, cz = _clamp(angular_z, limits.max_angular_rad_s)
    clamped = cx or cy or cz
    reason = "속도 상한으로 잘렸다" if clamped else ""
    return MotionDecision(True, Motion(vx, vy, wz), clamped=clamped, reason=reason)
