"""VLA 파지 한 사이클이 어디서 끝나는지 — 순수 계산(ROS·torch 없음).

정책이 낸 청크의 **명령 궤적**에서 찾는다. 청크 경계(3.33초)에서 실측값만 보면 그 사이에
일어난 일을 통째로 놓친다 — 2026-09-22 실기에서 실제로는 여러 번 시도했는데 로그에는 한
번으로 보였다. 청크 안에는 30Hz 100스텝이 들어 있고, 그것이 정책의 의도다.
"""
from __future__ import annotations

#: 골짜기 바닥에서 이만큼(도) 다시 오르면 "다음 시도를 시작한다"고 본다.
RETRY_RISE_DEG = 3.0


def scan_cycle(lift_cmd, state, extended_deg: float, dip_deg: float, returned_deg: float):
    """명령 궤적에서 **한 사이클이 끝나는 지점**을 찾는다.

    한 사이클 = 팔이 뻗었다가(> extended_deg) 돌아오는 것. 끝나는 모양이 둘이다.

        복귀    returned_deg 아래로 내려온다 (학습된 정상 종료, 실측 -103)
        재상승  골짜기 바닥에서 다시 오른다 (정책이 다음 시도를 시작한다, 실측 저점 -66~-74)

    둘 중 **먼저 오는 지점에서 멈춘다.** 재상승은 바닥에서 끊으므로 팔이 다시 뻗지 않는다.

    ⚠️ 성공·실패를 여기서 가르지 않는다. 물체가 없어도 정책은 올라갔다 내려오기를 반복해서
    궤적 모양이 비슷하다(2026-09-22 실기: 한 청크 안에서 -104 -> 상승 -> -104 -> 29).
    판정은 그리퍼캠 근접 변화가 한다 — 빈손 0% 대 파지 27~42% 로 갈린다.

    ⚠️ 청크 경계에서만 실측값을 보면 이 지점들을 통째로 놓친다. 청크 안에는 30Hz 100스텝이
    들어 있고 그것이 정책의 의도다.

    state = (above, ever, dip_min, dip_idx) — 청크 사이에 이어서 넘긴다.
    반환: (state, stop_at, reason). stop_at 이 None 이면 아직 사이클 중이다.
    """
    above, ever, dip_min, dip_idx = state
    for i, raw in enumerate(lift_cmd):
        value = float(raw)
        if not ever:
            if value > extended_deg:
                ever, above = True, True
            continue
        if above:
            if value < returned_deg:
                return (above, ever, None, 0), i, "복귀"
            if value < dip_deg:
                above, dip_min, dip_idx = False, value, i
            continue
        if dip_min is None or value < dip_min:
            dip_min, dip_idx = value, i
        if value < returned_deg:
            return (above, ever, None, 0), i, "복귀"
        if value > dip_min + RETRY_RISE_DEG:
            return (above, ever, None, 0), dip_idx, "재상승"
    return (above, ever, dip_min, dip_idx), None, None


