"""VLA 청크 재생 중 그리퍼 속도를 방향에 따라 정하는 규칙.

`arm_driver_node` 에서 떼어 낸 것은 **시험할 수 있게** 하기 위해서다. 그
모듈은 rclpy·grippers_interfaces·실물 SO-ARM101 을 끌어와 개발 머신에서
import 가 안 되고, 그래서 지금껏 AST 로 "그런 코드가 있는지"만 봐 왔다.
그 방식은 코드의 존재는 보지만 **결과가 맞는지는 못 본다.**

여기 있는 것은 순수 함수라 그냥 부르면 된다.

## 왜 방향에 따라 다른가

턱이 물체를 치고 밀어내는 것을 막으려면 **닫는** 속도를 묶어야 한다. 그런데
**여는** 것은 빈 공간에서 일어나므로 묶을 이유가 없고, 묶으면 오히려 해롭다 —
VLA 시작에서 턱이 다 벌어지기 전에 팔이 내려가면 고치려던 것과 같은 사고를
반대편에서 만든다.

촬영 데이터의 그리퍼 순간 속도(2026-09-06, 리눅스 세션이 v5_all 에서 실측,
raw/s 환산):

           중앙   p90   p95    p99    최대
    닫기    270   603   670    844  1,110
    열기    450   670   780  1,110  1,530

600 raw/s 상한을 양방향에 걸면 **열기 프레임의 18.22%** 가 잘린다. 닫기는
중앙값이 270 이라 대부분 상한에 안 닿고 **상위 10.81%** 만 잘리는데, 그것이
정확히 물체를 치는 구간이라 이 상한의 목적 그대로다.
"""

from __future__ import annotations

#: Goal_Velocity 0 = 무제한. 관절 1..5 가 재생 내내 쓰는 값이고, 그리퍼도
#: 여는 동안에는 여기로 돌아온다.
UNLIMITED = 0

#: 방향 판정 데드밴드(raw). 이보다 작은 변화는 방향 전환으로 안 본다.
#:
#: 지령이 거의 멈춘 구간에서는 부호가 잡음으로 흔들린다. 흔들릴 때마다
#: 레지스터를 쓰면 30Hz 재생 중에 시리얼이 그만큼 더 붐빈다. 정상 이동은
#: 프레임당 15~25 raw 라(촬영 실측 3.3~5.4 unit x 4.67), 3 raw 는 잡음만
#: 걸러내고 진짜 방향 전환은 놓치지 않는 수준이다.
DIRECTION_DEADBAND_RAW = 3


def gripper_speed_change(move_raw: int, close_speed: int, current_speed: int,
                         open_speed: int = UNLIMITED):
    """이번 스텝에서 servo 6 의 Goal_Velocity 를 바꿔야 하면 그 값을 돌려준다.

    바꿀 필요가 없으면 ``None`` — 그래야 호출부가 **정말 바뀔 때만** 레지스터를
    쓴다. 매 스텝 쓰면 30Hz 재생에서 시리얼이 그만큼 붐빈다.

    :param move_raw: 이번 스텝의 그리퍼 이동량(raw). **음수가 닫힘**이다 —
        gripper_calibration 의 표가 9.0mm -> 1150, 168.0mm -> 2000 이라
        raw 가 작을수록 닫힌 것이다.
    :param close_speed: 닫을 때 걸 상한(raw/s). 0 이하면 무제한.
    :param current_speed: 지금 서보에 걸려 있는 값. 돌려준 값을 호출부가
        여기에 넣어 이어 간다.

        ⚠️ 예전에는 `limited: bool` 이었다. 방향마다 값이 달라지면서 불리언으로는
        "지금 얼마가 걸려 있는가"를 표현할 수 없게 됐다 — 그대로 두면 매 스텝
        레지스터를 쓰게 되고, 그건 이 함수가 처음부터 피하려던 것이다.
    :param open_speed: 열 때 걸 상한(raw/s). 0 이하면 무제한.

    ⚠️ 2026-09-06 2차: 여는 쪽에도 손잡이가 필요해졌다. 실기에서 정책이 한
    청크(1.07초)에 9.0mm -> 64.8mm 를 여는데(약 270 raw/s), 촬영 실측 열기
    중앙값 450 raw/s 보다 오히려 느린데도 사용자에게는 "동작 속도에 비해 너무
    빠르다"로 보였다. 원인은 그리퍼가 아니라 **주변이 느린 것**이다 — 추론
    대기로 팔이 사이클의 27~45% 를 서 있어서 그리퍼만 홱 움직이는 것이
    도드라진다. 기본은 무제한이라 안 주면 예전 동작 그대로다.
    """
    if move_raw < -DIRECTION_DEADBAND_RAW:
        want = close_speed if close_speed > 0 else UNLIMITED
    elif move_raw > DIRECTION_DEADBAND_RAW:
        want = open_speed if open_speed > 0 else UNLIMITED
    else:
        return None                      # 데드밴드 안 — 방향을 안 바꾼다
    return None if want == current_speed else want
