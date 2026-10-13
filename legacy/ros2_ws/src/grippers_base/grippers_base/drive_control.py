"""`drive_to` 의 ROS 비의존 주행 계산.

노드는 rclpy 를 import 하므로 로컬 테스트에서 불러올 수 없다. 수치가 걸린
부분만 여기로 빼서 순수 함수로 검증한다 — 나머지 2단계 제어 구조(#148 · #166)는
`base_driver_node._execute_drive_to` 에 그대로 두고 AST 계약으로 지킨다.
"""

import math

KP_LINEAR = 0.6
MAX_LINEAR = 0.2  # app_cmd_vel_callback 클램프와 동일


def forward_speed(dist: float, yaw_err: float) -> float:
    """DRIVE 단계의 전진 속도 [m/s].

    잔여 거리를 **로봇 전진축에 투영**한다. `dist` 는 부호가 없어서 목표가 등
    뒤에 있어도 양수가 되는데, 도착 근처(`REALIGN_MIN_DIST_M` 안)에서는 재정렬을
    하지 않으므로 그대로 쓰면 방위를 못 고친 채 목표에서 멀어진다.

    투영값은 목표가 뒤에 있으면 음수가 되어 **후진으로 거리를 줄인다.** 정렬된
    상태에서는 `cos(yaw_err) ≈ 1` 이라 기존 동작과 같다.
    """
    return max(-MAX_LINEAR, min(MAX_LINEAR, KP_LINEAR * dist * math.cos(yaw_err)))
