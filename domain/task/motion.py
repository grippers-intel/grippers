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
# ⚠️ 0.1 은 만충에서만 여유가 있다. 2026-09-06 실측(탑뷰 ArUco, mm 정확도):
#
#   배터리 8.4V   명령 0.05 -> 0.015 m/s (손실 70%)   0.10 -> 0.088 (12%)
#                 0.15 -> 0.129 (14%)   0.20 -> 0.186 (7%)   0.25 -> 0.223 (11%)
#   배터리 8.0V   명령 0.10 -> 안 움직임
#   배터리 7.4V   명령 0.25 까지 올려도 안 움직임
#
# 데드밴드가 전압을 따라 올라간다. 만충 기준으로 겨우 넘는 값을 쓰면 시연
# 도중에 멈춘다 — 실제로 8.4V 에서 되던 0.1 이 8.0V 에서 안 됐다.
# 0.15 는 만충 실측 최소(0.10)의 1.5배다.
#
# ⚠️ 라이다 빔 변화로는 이 값을 못 잰다. 평행이동에 둔감해서 실제로 70mm 를
# 가도 "안 움직였다"로 찍힌다. 탑뷰 ArUco 로 재야 한다.
AGREED_LINEAR_MPS = 0.15
# ⚠️ 회전에도 데드밴드가 있고, 합의값 0.25 는 **그 아래였다.** 2026-09-05
# 실측(IMU, tools/rotation_threshold_sweep.py):
#
#     명령 0.15  실측 0.021  안 돎
#     명령 0.25  실측 0.035  안 돎      <- 예전 합의값
#     명령 0.35  실측 0.120  돎
#     명령 0.45  실측 0.223  돎
#     명령 0.70  실측 0.467  돎
#
# 0.25 를 쓰던 동안 차는 ALIGN_YAW 에서 회전 명령을 계속 받으면서 제자리에
# 버텼다. 명령은 set_motor 까지 정상으로 내려가므로 로그만 보면 멀쩡해 보이고,
# /odom_raw 는 명령을 그대로 적분해 "돌았다"고 보고한다 — 실제로 돌았는지는
# IMU 로만 알 수 있다(2026-08-24 기록의 0.355 와 거의 같은 값이 재측정됐다).
#
# 0.6 은 실측 최소(0.35)의 1.7배다. 바닥이나 적재 무게가 조금 달라져도 문턱을
# 넘도록 여유를 뒀다. 오버슛은 문제가 안 된다 — 0.6 의 실제 회전은 약 20°/s
# 이고 Host 루프가 10Hz 이므로 사이클당 2° 인데, DRIVE_YAW_TOLERANCE_DEG 는
# 5° 다. 더 올리면 그 여유가 줄어든다.
AGREED_ROTATION_RAD_S = 0.6

# 바구니 최종 접근에서만 쓰는 더 낮은 상한 (2026-08-26 사용자 승인).
#
# ## 왜 구간을 나누는가 — 지연이 허용폭보다 크다
#
# Host가 "지금 멈춰"라고 판단한 순간부터 바퀴가 실제로 서기까지 지연이 쌓인다.
#
#     Host 루프 한 바퀴(8Hz)    125 ms
#     UDP + Pi 수신              10 ms
#     Pi 사이클(10Hz)           100 ms
#     ------------------------------
#     합계                      235 ms
#
# 0.1 m/s면 그동안 **23.5 mm**를 더 간다. 그런데 INSERT 허용폭은 ±15 mm
# (BASKET_STOP_TOLERANCE_M)다 — **오버슈트가 창보다 크다.** 창 안에 우연히
# 들어갈 수는 있어도 제어되는 것이 아니다.
#
# 0.06으로 낮추면 같은 지연이 14 mm가 되어 창 안에 들어온다.
#
# ## 왜 더 낮추지 못하는가
#
# 데드밴드가 0.05다. 그 아래는 아무리 오래 줘도 안 움직인다. 0.06이 실제로
# 도는 최저 속도이므로 여기가 바닥이다. 더 잘게 가야 하면 속도가 아니라
# **끊어 가기**로 해야 한다(ros2_mecanum_base.creep_forward).
#
# ## 왜 Pi가 자르는가 — 이건 경로가 아니라 센서 제약이다
#
# 차량 제어는 Host 소유다. 그런데 이 상한은 "어디로 갈지"가 아니라 "이보다
# 빠르면 내 센서로 판정 자체가 불가능하다"는 Pi 쪽 사실이다. 데드밴드나
# 바구니 절벽과 같은 성격이라 Pi가 지킨다 — Host가 0.1을 보내도 이 구간에서는
# 0.06으로 실행되고, 그 사실을 보고에 적는다.
BASKET_APPROACH_MPS = 0.06

# 부동소수 잡음을 0으로 본다. UDP+JSON을 거치며 0.0이 1e-17로 오는 경우가
# 있는데, 그걸 "회전 명령"으로 읽으면 병진과 섞였다고 오판해 거부한다.
EPSILON = 1e-6

# MissionState.APPROACH_BOX와 같은 문자열이어야 한다. baseline_ports를
# import하면 순환이 되므로 값을 직접 적고, 테스트가 두 값을 대조한다.
_APPROACH_BOX = "APPROACH_BOX"


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

    # 바구니로 붙는 구간만 더 낮은 상한을 쓴다. 회전은 안 낮춘다 — 회전은
    # 한 사이클에 1.8도라 이미 허용치(5도)의 3분의 1이다.
    linear_cap = (BASKET_APPROACH_MPS if command.state == _APPROACH_BOX
                  else AGREED_LINEAR_MPS)
    motion = Motion(
        linear_x=_clamp(command.linear_x, linear_cap),
        linear_y=_clamp(command.linear_y, linear_cap),
        angular_z=_clamp(command.angular_z, AGREED_ROTATION_RAD_S),
    )
    slowed = (linear_cap < AGREED_LINEAR_MPS
              and (abs(command.linear_x) > linear_cap + EPSILON
                   or abs(command.linear_y) > linear_cap + EPSILON))
    if slowed:
        return MotionDecision(
            True, motion,
            f"바구니 접근 구간이라 {linear_cap:.2f} m/s로 낮췄다 "
            f"(명령 {max(abs(command.linear_x), abs(command.linear_y)):.2f})")
    return MotionDecision(True, motion)
