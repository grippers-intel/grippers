"""파지 한 번에 몇 청크를 줄 것인가 — 개수가 아니라 시간으로 정한다.

`vla_inference_node` 에서 떼어 낸 것은 **시험할 수 있게** 하기 위해서다. 그
모듈은 rclpy·grippers_interfaces 를 끌어와 개발 머신에서 import 가 안 된다.

## 왜 개수가 아니라 시간인가

원래는 `MAX_CHUNKS = 10` 이라는 개수 상한이었다. 그 10 은 "ACT 실측 최대가
7.1청크"에서 나온 값인데, **ACT 의 한 청크는 100스텝 / 30fps = 3.33초**다.
즉 그 상한이 실제로 뜻한 것은 **33.3초의 동작**이지 "10번"이 아니었다.

DP 로 갈아 끼우자 그게 그대로 고장이 됐다. DP 는 `n_action_steps` 가 32라
한 청크가 1.07초다 — 같은 10개면 10.7초, 원래 예산의 3분의 1이다.
2026-09-06 실기 기록(runs/vla/20260906_131521_queen)이 정확히 그 모양이었다:

    청크 1~2   가만히 (학습된 시작 동작)
    청크 3     그리퍼 활짝 열기 (7.7 -> 94.1)
    청크 4~6   손목 내리기
    청크 7~10  팔을 뻗는 중 (lift -102 -> -0.7)
    여기서 소진 — 물체에 닿지도, 닫지도 못했다

그 뒤 `fold_to_cradle` 이 실패하고 공유 경로의 carry 자세로 넘어가는데,
밖에서 보면 **"정책이 아니라 하드코딩 자세가 파지한다"**로 보인다. 실제로
사용자가 그렇게 보고했다 — "동료의 하드코딩 방식과 너무 똑같은데".

## 시계와 개수는 둘 다 있어야 한다

개수 상한만 있으면 각 청크가 예상보다 오래 걸릴 때(추론이 느려지거나 재생이
밀릴 때) 무한정 늘어진다. 시계 상한만 있으면 청크 길이가 바뀔 때마다 사람이
다시 잡아야 한다. 그래서 **개수는 청크 길이에서 계산하고, 시계는 그 개수를
다 쓸 수 있는지 검사하는 데** 쓴다.
"""

from __future__ import annotations

import math

#: 파지 한 번에 허용하는 동작 시간(초).
#: 33.3 = 기존 ACT 동작을 그대로 재현하는 값이다(10청크 x 3.33초).
MAX_GRASP_MOTION_SEC = 33.3

#: 청크 하나당 재생 밖에서 드는 시간(초) — 관측 읽기, 추론, 액션 왕복.
#: 실측: DP 원격 0.42초, ACT 원격 0.12초, ACT Pi 로컬 0.47초. 0.5 면 셋 다 덮는다.
CHUNK_OVERHEAD_SEC = 0.5

#: 완료 판정 전에 최소 이만큼은 돈다. 시작 자세가 이미 "접힘"이라 첫 청크에서
#: 곧바로 끝난 것으로 읽히는 것을 막는다.
MIN_CHUNKS = 2


def chunk_budget(n_action_steps: int, fps: float, timeout_s: float):
    """(청크 길이, 최대 청크 수, 써야 할 시계 상한) 을 돌려준다.

    :param n_action_steps: 정책이 한 청크로 내는 스텝 수 (ACT 100, DP 32).
    :param fps: 재생 주파수. 학습과 같아야 한다(30).
    :param timeout_s: 호출자가 정한 시계 상한. 청크를 다 쓰기에 모자라면
        **늘려서** 돌려준다 — 조용히 끊기면 "정책이 중간에 멈췄다"로 보인다.

    돌려주는 시계 상한이 입력보다 크면 호출자가 그 사실을 로그로 남겨야 한다.
    """
    fps = max(float(fps), 1.0)
    chunk_sec = max(int(n_action_steps) / fps, 1e-6)
    # 최소 개수를 보장한다 — 청크가 아주 길면 ceil 이 1 을 줄 수도 있는데,
    # 그러면 완료 판정(MIN_CHUNKS)이 성립할 기회조차 없다.
    max_chunks = max(MIN_CHUNKS + 1, math.ceil(MAX_GRASP_MOTION_SEC / chunk_sec))
    needed_s = max_chunks * (chunk_sec + CHUNK_OVERHEAD_SEC)
    return chunk_sec, max_chunks, max(float(timeout_s), needed_s)
