"""Host가 보낸 속도 명령을 Pi가 실제로 낼 속도로 바꾼다 (팀 확정, 2026-08-26).

순수 계산이다 — 포트도 ROS도 모른다. 주행 안전의 마지막 한 겹이라 단위
테스트로 고정해 둘 수 있어야 한다.

## 왜 Pi가 자르는가

Host가 좌표와 경로를 소유하지만, **바퀴를 실제로 돌리는 것은 Pi다.** 속도
한계는 그 한계를 어길 수 있는 쪽이 아니라 물리적으로 집행할 수 있는 쪽에
있어야 한다. Host의 버그나 패킷 손상으로 0.1이 1.0으로 오더라도 Pi가 잘라
낸다. Pi가 명령을 **고르지는** 않는다 — 방향은 Host 것 그대로 두고 크기만
합의된 값으로 제한한다.

## 제자리회전은 정말 제자리여야 한다

팀이 합의한 명령 어휘는 직진·수평이동·제자리회전·제자리정지 네 가지다.
"제자리"회전에 병진이 섞여 들어오면 그것은 합의된 네 가지 중 무엇도 아니다.
추측해서 둘 중 하나를 골라 실행하는 대신 **거부하고 Host에 되돌려준다** —
이 저장소의 "모르면 실패" 관례 그대로다.

직진과 수평이동이 함께 오는 것은 막지 않는다. 메카넘 베이스에서 그 둘은
자연스러운 한 동작(대각선 이동)이고, "제자리"라는 단서가 붙은 쪽은 회전뿐이다.
"""

import math
from dataclasses import dataclass

# 팀 합의 속도 (2026-08-26). 직진과 수평이동이 같은 값이다.
#
# ⚠️ 이 베이스에는 데드밴드가 있다 — 0.05 m/s 명령에는 바퀴가 아예 안 돈다
# (2026-08-24 실기, tools/grasp_test_console.py의 APPROACH_SPEED_MPS 주석).
# 합의된 0.1은 그 위라 실제로 움직인다. 더 느리게 가야 한다면 속도를 낮추지
# 말고 짧은 버스트와 정지를 반복할 것 — 데드밴드 아래 속도는 아무리 오래
# 줘도 안 움직이는데 /odom_raw는 움직였다고 보고한다.
AGREED_LINEAR_MPS = 0.1
AGREED_ROTATION_RAD_S = 0.25

# 부동소수 잡음을 0으로 본다. UDP+JSON을 거치며 0.0이 1e-17로 오는 경우가
# 있는데, 그걸 "회전 명령"으로 읽으면 병진과 섞였다고 오판해 거부한다.
EPSILON = 1e-6


@dataclass(frozen=True)
class Motion:
    """실제로 베이스에 낼 속도."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0

    @property
    def is_stop(self) -> bool:
        return (abs(self.linear_x) < EPSILON
                and abs(self.linear_y) < EPSILON
                and abs(self.angular_z) < EPSILON)


STOP = Motion()


@dataclass(frozen=True)
class MotionDecision:
    """`resolve_motion`의 결과. 거부됐으면 `ok=False`이고 `motion`은 정지다."""

    ok: bool
    motion: Motion
    reason: str = ""


def _clamp(value: float, limit: float) -> float:
    """부호는 두고 크기만 limit로 자른다."""
    if abs(value) < EPSILON:
        return 0.0
    return math.copysign(min(abs(value), limit), value)


def resolve_motion(command) -> MotionDecision:
    """`HostCommand`를 실제 속도로 바꾼다.

    우선순위:
      1. `stop=True`면 나머지를 보지 않고 정지한다. 제자리정지가 가장 센
         명령이어야 한다 — 정지 의도가 다른 필드의 잔여값에 지면 안 된다.
      2. 제자리회전에 병진이 섞였으면 거부한다(정지 + 사유).
      3. 나머지는 합의된 크기로 자른다.
    """
    if command is None:
        return MotionDecision(False, STOP, "명령 없음")

    if command.stop:
        return MotionDecision(True, STOP, "제자리정지")

    rotating = abs(command.angular_z) >= EPSILON
    translating = (abs(command.linear_x) >= EPSILON
                   or abs(command.linear_y) >= EPSILON)
    if rotating and translating:
        return MotionDecision(
            False, STOP,
            "제자리회전에 병진이 섞였다 — "
            f"linear=({command.linear_x:.3f}, {command.linear_y:.3f}), "
            f"angular={command.angular_z:.3f}")

    return MotionDecision(True, Motion(
        linear_x=_clamp(command.linear_x, AGREED_LINEAR_MPS),
        linear_y=_clamp(command.linear_y, AGREED_LINEAR_MPS),
        angular_z=_clamp(command.angular_z, AGREED_ROTATION_RAD_S),
    ))
