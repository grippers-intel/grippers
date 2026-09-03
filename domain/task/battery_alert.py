"""저전압 부저 경고 판정 — 순수 함수, ROS2도 하드웨어도 모른다 (2026-09-04).

## 문턱을 왜 이 값으로 잡았나

2026-09-03 실기: "차가 안 움직인다" 증상이 나온 구간에서 배터리를 여러 번
읽었더니 8405 → 8035 → 7741 → **7737mV**로 내려가는 추세였다(무부하). 그런데
이 4개 값 전부 기존에 문서화된 무부하 경고선(`pi_redeploy_checklist.md`의
7150mV)보다 높다 — 즉 그 문턱은 이 증상을 못 잡는다. 그래서 이 부저 경고는
그 문턱을 그대로 쓰지 않고, 실측된 문제 구간(7737mV)보다 여유 있게 위에서
미리 울리도록 `WARN_MV`를 따로 잡았다.

⚠️ 이 증상이 정말 전압 때문이었는지는 그 세션에서 끝내 확정하지 못했다(재기동
후에도 재발한 적이 있어 순수 소프트웨어 문제일 가능성도 남아 있다) — 이
경고는 "그때와 비슷한 전압 구간에 다시 들어왔다"를 알려줄 뿐, 원인을
전압이라고 단정하지 않는다.

## 부저 소리 자체

`BUZZER_FREQ_HZ`/`BUZZER_ON_S`는 사용자 지시(2026-08-28)를 따른다 — 압전
소자의 공진대(2~4kHz)를 피하고 아주 짧게 문다. 400Hz는 같은 소자에서
2000Hz보다 훨씬 조용하다. 경고가 계속 유효한 동안은 `repeat`를 줄이는 대신
`MIN_REPEAT_INTERVAL_S` 간격으로 짧게 반복해서, 전체 경고 시간은 유지하되
소음은 줄인다(같은 지시의 "repeat를 줄이지 말고 간격을 늘려라" 원칙)."""

from dataclasses import dataclass
from typing import Optional

# 경고를 켜는 문턱 — 위 ⚠️ 참고, 2026-09-03 실측 문제 구간(7737mV)보다
# 여유를 두고 위에 잡은 값이라 아직 "실측 확정"은 아니다.
WARN_MV = 7800

# 이 값보다 올라가야 "회복"으로 보고 경고를 끈다. WARN_MV보다 높게 잡아서
# 문턱 바로 위/아래를 오갈 때 매번 켜졌다 꺼졌다 하는 걸 막는다(히스테리시스).
RECOVER_MV = 7900

# 경고가 유효한 동안 이보다 자주 다시 울리지 않는다.
MIN_REPEAT_INTERVAL_S = 15.0

# 부저 파라미터 — 위 모듈 docstring 참고.
BUZZER_FREQ_HZ = 400
BUZZER_ON_S = 0.05
BUZZER_OFF_S = 0.15
BUZZER_REPEAT = 2


@dataclass(frozen=True)
class BatteryAlertState:
    """호출자가 다음 호출에 그대로 넘겨주는 상태 — 이 모듈은 내부 상태를
    갖지 않는다."""

    warning_active: bool = False
    last_beep_at: Optional[float] = None


@dataclass(frozen=True)
class BuzzerCommand:
    freq: int
    on_time: float
    off_time: float
    repeat: int


def check_battery(
    voltage_mv: float, now: float, state: BatteryAlertState
) -> tuple[BatteryAlertState, Optional[BuzzerCommand]]:
    """전압 한 번 읽은 값과 지금 시각(단조 증가 초, 임의 기준점 가능)을
    넣는다. (다음에 넘겨줄 상태, 지금 당장 울릴 부저 명령 또는 None)을
    낸다.

    히스테리시스: `WARN_MV` 아래로 내려가야 경고를 켜고, `RECOVER_MV`
    위로 올라가야 끈다 — 그 사이 값은 이전 상태를 그대로 유지한다."""
    warning_active = state.warning_active
    if voltage_mv <= WARN_MV:
        warning_active = True
    elif voltage_mv >= RECOVER_MV:
        warning_active = False

    if not warning_active:
        return BatteryAlertState(warning_active=False, last_beep_at=None), None

    if state.last_beep_at is not None and now - state.last_beep_at < MIN_REPEAT_INTERVAL_S:
        return BatteryAlertState(warning_active=True, last_beep_at=state.last_beep_at), None

    cmd = BuzzerCommand(
        freq=BUZZER_FREQ_HZ, on_time=BUZZER_ON_S, off_time=BUZZER_OFF_S, repeat=BUZZER_REPEAT
    )
    return BatteryAlertState(warning_active=True, last_beep_at=now), cmd
