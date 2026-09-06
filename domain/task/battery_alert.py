"""저전압 부저 경고 판정 — 순수 함수, ROS2도 하드웨어도 모른다 (2026-09-04).

## 문턱을 왜 이 값으로 잡았나

2026-09-03 실기: "차가 안 움직인다" 증상이 나온 구간에서 배터리를 여러 번
읽었더니 8405 → 8035 → 7741 → **7737mV**로 내려가는 추세였다(무부하). 그런데
이 4개 값 전부 기존에 문서화된 무부하 경고선(`pi_redeploy_checklist.md`의
7150mV)보다 높다 — 즉 그 문턱은 이 증상을 못 잡는다. 그래서 이 부저 경고는
그 문턱을 그대로 쓰지 않고, 실측된 문제 구간(7737mV)보다 여유 있게 위에서
미리 울리도록 `WARN_MV`를 따로 잡았다.

## ✅ 2026-09-06 밤 — 전압이 원인인 경우가 확정됐다

위 문단은 원래 "정말 전압 때문이었는지 끝내 확정하지 못했다"였다. 그 미확정
때문에 2026-09-04 에 이 노드를 런치에서 뺐고, 오늘 그 대가를 치렀다.

같은 증상이 다시 났고 이번엔 깨끗하게 갈렸다:

    8030 mV   base_power_probe 에서 IMU 1.61 rad/s — 정상 회전
    7900 mV   정상 주행
    6290 mV   apply_velocity(0.15, 0, 0) 이 보드까지 도달하는데 바퀴가 안 돈다

6290mV 시점에 소프트웨어는 **전부 멀쩡했다** — 노드 5개 생존, 시리얼
양방향 정상(배터리 토픽이 계속 나옴), motor_watchdog 0회, apply_velocity
호출이 로그에 그대로 찍힘. 그런데 탑뷰 포즈가 6초 동안 1mm 도 안 변했다.

그 사이에 "재부팅 + 배터리 재연결로 고쳤다"고 두 번 오진했다 — 고친 게
아니라 그때는 아직 전압이 남아 있었던 것이다.

⚠️ 그렇다고 **모든** "안 움직인다"가 전압인 것은 아니다. 2026-09-03 의
회전 데드밴드 문제는 실재했다. 이 경고는 여전히 "그 전압 구간에 들어왔다"를
알릴 뿐이고, 원인 판정은 IMU(회전)와 탑뷰(병진)로 따로 해야 한다.

## 부저 소리 자체

`BUZZER_FREQ_HZ`/`BUZZER_ON_S`는 사용자 지시(2026-08-28)를 따른다 — 압전
소자의 공진대(2~4kHz)를 피하고 아주 짧게 문다. 400Hz는 같은 소자에서
2000Hz보다 훨씬 조용하다. 경고가 계속 유효한 동안은 `repeat`를 줄이는 대신
`MIN_REPEAT_INTERVAL_S` 간격으로 짧게 반복해서, 전체 경고 시간은 유지하되
소음은 줄인다(같은 지시의 "repeat를 줄이지 말고 간격을 늘려라" 원칙)."""

from dataclasses import dataclass
from typing import Optional

# 경고를 켜는 문턱.
#
# ⚠️ 2026-09-06 밤에 7800 -> 7200 으로 내렸다. 7800 은 "정상 동작 중에도
# 계속 울린다"는 것이 실기에서 확인됐다 — 사용자 보고 "부저가 계속 울리는데".
#
# 7800 은 2026-09-03 에 "문제 구간이 7737mV 였으니 그보다 위에서 미리
# 울리자"로 잡은 값이었다. 그런데 그 7737 이 정말 고장 지점인지는 그때
# 확정하지 못했고(모듈 docstring 참고), 오늘 실측으로 **아니었다**는 것이
# 드러났다:
#
#     7660~7750 mV   회전 부하에서 IMU 1.62 rad/s — 완전히 정상
#     6290 mV        바퀴가 아예 안 돎
#
# 즉 7737 근처는 멀쩡히 도는 구간이다. 거기서 울리면 경보가 소음이 되고,
# 소음이 되면 진짜 경고를 무시하게 된다.
#
# 7200 은 실측 고장점(6290)보다 910mV 위다. 오늘 소모 속도가 몇 시간에
# 1.7V 였으니 7200 에서 알리면 여유가 충분하다.
WARN_MV = 7200

# 이 값보다 올라가야 "회복"으로 보고 경고를 끈다. WARN_MV보다 높게 잡아서
# 문턱 바로 위/아래를 오갈 때 매번 켜졌다 꺼졌다 하는 걸 막는다(히스테리시스).
RECOVER_MV = 7400

# 경고가 유효한 동안 이보다 자주 다시 울리지 않는다.
#
# ⚠️ 2026-09-06 밤에 15 -> 60 으로 늘렸다. 시연 중에 15초마다 울리면
# 사람이 견디지 못하고 그냥 꺼 버린다 — 그러면 경보가 없는 것만 못하다
# (2026-09-04 에 이 노드를 통째로 뺐던 것이 정확히 그 경로였다).
# 문턱을 낮춰(7200) 정말 급할 때만 울리게 했으니 간격도 넉넉히 둔다.
MIN_REPEAT_INTERVAL_S = 60.0

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
