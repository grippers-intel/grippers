"""VLA 파지 한 사이클이 어디서 끝나는지 — 순수 계산(ROS·torch 없음).

정책이 낸 청크의 **명령 궤적**에서 찾는다. 청크 경계(3.33초)에서 실측값만 보면 그 사이에
일어난 일을 통째로 놓친다 — 2026-09-22 실기에서 실제로는 여러 번 시도했는데 로그에는 한
번으로 보였다. 청크 안에는 30Hz 100스텝이 들어 있고, 그것이 정책의 의도다.

## 재시도는 **상대값**으로 본다 (2026-09-30)

예전에는 "절대값 retry_dip_deg(-60) 아래까지 내려왔다가 다시 뻗으면"이었다. 그런데 시도 사이
저점이 매번 다르다 — 09-22 에 -66, 09-30 에 **-58**. -80 으로 두었을 때도, -60 으로 두었을
때도 한 번씩 빗나갔고, 09-30 에는 정책이 같은 자리에서 세 번째 시도까지 이어갔다.
그래서 **최고점에서 drop_deg 이상 내려온 뒤, 골짜기에서 rise_deg 이상 다시 오르면** 재시도다.

    과거 실기 파지 13회(성공·실패)의 청크 명령 lift 로 정한 값:
      성공한 사이클   최고점 63~79 -> 곧장 -88~-104 복귀. 내려온 뒤 다시 오른 적 없음
      재시도          골짜기 -58 / -66 에서 +35 까지 **+93~+101** 상승
      청크 경계 튐    다음 청크 첫 값이 이전 끝보다 최대 **+14**(3 -> 14, 40 -> 54)
    -> drop 60 · rise 30 (경계 튐의 2배 · 재시도의 1/3)
"""
from __future__ import annotations

#: scan_cycle 에 처음 넘기는 상태: (above, ever, peak, dip_min, dip_idx)
#: dip_idx 가 -1 이면 골짜기가 **이전 청크**에 있었다는 뜻이다.
CYCLE_START = (False, False, None, None, 0)

#: 복귀 문턱을 넘은 뒤 **바닥까지** 재생한다(2026-10-01). 문턱을 처음 넘는 순간 끊으면 정책이
#: 아직 접는 중이라, 이어지는 idle 이동이 남은 몫을 한꺼번에 메우며 그리퍼를 "살짝 드는" 것처럼
#: 보였다(사용자 관찰). 바닥 = 더 내려가지 않는 지점:
#:   - 바닥에서 SETTLE_RISE_DEG 넘게 다시 오르면(다음 시도의 시작) 바닥 스텝까지
#:   - SETTLE_FLAT_STEPS 동안 새 바닥이 안 나오면(0.5° 넘게) 거기까지
#:   - 그 전에 청크가 끝나면 청크 끝까지(다음 청크를 추론하러 가지 않는다)
SETTLE_RISE_DEG = 3.0
SETTLE_FLAT_STEPS = 10


def scan_cycle(lift_cmd, state, extended_deg: float, drop_deg: float, rise_deg: float,
               returned_deg: float):
    """명령 궤적에서 **한 사이클이 끝나는 지점**을 찾는다.

    한 사이클 = 팔이 뻗었다가(> extended_deg) 돌아오는 것. 끝나는 모양이 둘이다.

        복귀    returned_deg 아래로 내려온다 (학습된 정상 종료, 실측 -103)
        재상승  최고점에서 drop_deg 이상 내려왔다가 골짜기에서 rise_deg 이상 다시 오른다
                (정책이 같은 자리에서 다음 시도를 시작한다)

    둘 중 **먼저 오는 지점에서 멈춘다.** 재상승은 골짜기 바닥에서 끊으므로 팔이 다시 뻗지
    않는다. 골짜기가 이전 청크였으면 stop_at 0 — 이번 청크는 한 스텝도 재생하지 않는다.
    복귀는 문턱을 넘은 뒤 **바닥까지** 재생한다(SETTLE_* 참고).

    ⚠️ 성공·실패를 여기서 가르지 않는다. 판정은 그리퍼캠 근접 변화가 한다.

    반환: (state, stop_at, reason). stop_at 이 None 이면 아직 사이클 중이다.
    stop_at 은 재생할 스텝 수다(청크[:stop_at]).
    """
    above, ever, peak, dip_min, dip_idx = state
    for i, raw in enumerate(lift_cmd):
        value = float(raw)
        if not ever:
            if value > extended_deg:
                ever, above, peak = True, True, value
            continue
        if value < returned_deg:
            return CYCLE_START, _settle_end(lift_cmd, i), "복귀"
        if above:
            peak = value if peak is None else max(peak, value)
            if value < peak - drop_deg:
                above, dip_min, dip_idx = False, value, i
            continue
        if value < dip_min:
            dip_min, dip_idx = value, i
        if value > dip_min + rise_deg:
            return CYCLE_START, max(dip_idx, 0), "재상승"
    # 청크가 끝났다. 골짜기 위치는 이제 "이전 청크"다.
    return (above, ever, peak, dip_min, -1 if dip_min is not None else 0), None, None


def _settle_end(lift_cmd, start: int) -> int:
    """복귀 문턱을 넘은 start 부터 바닥을 찾아, 재생할 스텝 수(바닥 스텝 포함)를 돌려준다."""
    bottom, bottom_idx = float(lift_cmd[start]), start
    for i in range(start + 1, len(lift_cmd)):
        value = float(lift_cmd[i])
        if value < bottom - 0.5:
            bottom, bottom_idx = value, i
            continue
        if value < bottom:
            bottom = value                      # 0.5° 안쪽의 미세한 하강은 바닥을 갱신만 한다
        if value > bottom + SETTLE_RISE_DEG or i - bottom_idx >= SETTLE_FLAT_STEPS:
            return bottom_idx + 1
    return len(lift_cmd)
