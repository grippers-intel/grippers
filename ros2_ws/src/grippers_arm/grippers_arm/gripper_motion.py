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


def gripper_speed_change(move_raw: int, close_speed: int, limited: bool):
    """이번 스텝에서 servo 6 의 Goal_Velocity 를 바꿔야 하면 그 값을 돌려준다.

    바꿀 필요가 없으면 ``None`` — 그래야 호출부가 **방향이 바뀔 때만**
    레지스터를 쓴다. 매 스텝 쓰면 30Hz 재생에서 시리얼이 그만큼 붐빈다.

    :param move_raw: 이번 스텝의 그리퍼 이동량(raw). **음수가 닫힘**이다 —
        gripper_calibration 의 표가 9.0mm -> 1150, 168.0mm -> 2000 이라
        raw 가 작을수록 닫힌 것이다.
    :param close_speed: 닫을 때 걸 상한(raw/s). 0 이하면 아무것도 안 한다 —
        예전(무제한) 동작으로 되돌리는 경로이고, A/B 로 원인을 가리려면
        이 길이 있어야 한다.
    :param limited: 지금 서보에 상한이 걸려 있는가.
    """
    if close_speed <= 0:
        return None
    if move_raw < -DIRECTION_DEADBAND_RAW and not limited:
        return close_speed
    if move_raw > DIRECTION_DEADBAND_RAW and limited:
        return UNLIMITED
    return None
