"""서보 EEPROM 의 Min/Max_Angle_Limit(주소 9/11) 읽기·쓰기.

## 왜 별도 파일인가

`calib_identity.py` 는 Homing_Offset 하나만 본다 — 그리고 그게 2026-09-01까지
알려진 문제의 전부인 줄 알았다. 그런데 그날 그리퍼(servo 6)가 어떤 폭을
명령해도 전혀 안 닫히는 사고가 났다. Homing_Offset 은 이미 `restore_taught_
offsets.py` 로 복구된 상태였는데도 그랬다 — 원인은 **다른 EEPROM 레지스터**,
서보가 실제로 움직일 수 있는 물리적 허용 범위(Min/Max_Angle_Limit)였다.
LeRobot/VLA 캘리브레이션은 이 레지스터를 Homing_Offset 과 **같이** 덮어쓰는데
(그날 실측: 1140~2090 -> 1960~2378), 복구 도구는 이 레지스터를 아예 보지도
쓰지도 않았다. "닫힘"에 필요한 목표(raw ~1150)가 서보 펌웨어의 허용 범위
밖이라 조용히 무시됐다 — ROS 서비스는 `ok=True` 를 그대로 반환했다
(`set_position()` 이 ACK 는 받지만 목표 자체가 서보 안에서 버려진다).

그날은 스크래치패드 즉석 스크립트로 레지스터를 직접 복구했고, 그 수정은
저장소에 반영되지 않았다(`grippers_handover_20260901.md` §2-4). 이 파일이
그 즉석 스크립트를 저장소 코드로 옮긴 것이다 — 다음에 같은 사고가 나면
다시 스크래치패드 스크립트를 짜는 대신 `restore_taught_offsets.py` 하나로
끝나야 한다.

## 왜 driver_sdk 의 공개 API 가 아니라 내부 메서드를 쓰는가

`driver_sdk.STS3215Driver` 는 `get_homing_offset`/`set_homing_offset` 는
공개 메서드로 제공하지만, Min/Max_Angle_Limit 을 위한 대응 메서드가 **없다**
(third_party/soarm_provided_d, 이 저장소가 추적하지 않는 서드파티 SDK).
그래서 그 SDK가 내부적으로 쓰는 것과 같은 저수준 프리미티브
(`_read_u16`/`_write_u16`/`set_eeprom_lock`, 뒤 둘은 `set_homing_offset` 의
구현과 같은 패턴)를 직접 쓴다. SDK 가 나중에 이 레지스터의 공개 API를
추가하면 그쪽으로 옮기는 것이 맞다 — 지금은 그런 API가 없다.

## 순수 함수가 아니다

`calib_identity.py` 와 달리 이 파일은 드라이버에 직접 읽고 쓴다. 판정
자체는 여전히 `calib_identity.verdict()` 를 그대로 재사용한다 — min과
max를 각각 `{servo_id: int}` 로 펼치면 그 함수가 이미 하는 일(정수 딕셔너리
비교)과 같아지기 때문에, 새 판정 로직을 만들지 않는다.
"""

from __future__ import annotations

import time

# STS3215 레지스터 주소. driver_sdk.py 의 ADDR_HOMING_OFFSET(31)·ADDR_LOCK(55)
# 과 같은 표에 있는 값이다(2026-09-01 스크래치 스크립트가 확인).
ADDR_MIN_ANGLE_LIMIT = 9
ADDR_MAX_ANGLE_LIMIT = 11

# calib_identity.READ_ATTEMPTS_DEFAULT / READ_RETRY_SEC 와 같은 값·같은
# 이유다 — 이 버스는 패킷을 이따금 흘린다.
READ_ATTEMPTS_DEFAULT = 3
READ_RETRY_SEC = 0.05


def get_position_limits(driver, servo_id: int) -> tuple[int, int] | None:
    """(Min_Angle_Limit, Max_Angle_Limit) 한 벌을 읽는다. 못 읽으면 None."""
    lo = driver._read_u16(servo_id, ADDR_MIN_ANGLE_LIMIT)
    hi = driver._read_u16(servo_id, ADDR_MAX_ANGLE_LIMIT)
    if lo is None or hi is None:
        return None
    return (lo, hi)


def set_position_limits(driver, servo_id: int, lo: int, hi: int) -> bool:
    """(Min_Angle_Limit, Max_Angle_Limit) 을 쓴다.

    `driver_sdk.set_homing_offset` 과 같은 패턴 — EEPROM 은 잠금을 풀어야
    쓸 수 있고, 쓴 뒤 다시 잠근다."""
    driver.set_eeprom_lock(servo_id, False)
    ok_lo = driver._write_u16(servo_id, ADDR_MIN_ANGLE_LIMIT, int(lo))
    ok_hi = driver._write_u16(servo_id, ADDR_MAX_ANGLE_LIMIT, int(hi))
    driver.set_eeprom_lock(servo_id, True)
    return ok_lo and ok_hi


def read_all(driver, servo_ids, attempts: int = READ_ATTEMPTS_DEFAULT):
    """여러 서보의 (min, max) 를 한 벌씩 읽는다 — 실패하면 재시도한다.

    `calib_identity.read_offsets` 와 같은 재시도 계약이다."""
    out = {}
    for sid in servo_ids:
        value = None
        for attempt in range(max(1, attempts)):
            value = get_position_limits(driver, sid)
            if value is not None:
                break
            if attempt + 1 < attempts:
                time.sleep(READ_RETRY_SEC)
        out[sid] = value
    return out


def split_limits(limits: dict[int, tuple[int, int] | None]):
    """{servo_id: (min, max) 또는 None} -> (mins, maxes) 두 개의
    {servo_id: int 또는 None} 딕셔너리.

    이렇게 펼치면 `calib_identity.verdict()` 를 그대로 두 번(min용, max용)
    호출할 수 있다 — 그 함수는 애초에 정수 딕셔너리 비교만 하지, "Homing_
    Offset" 이라는 특정 레지스터에 묶여 있지 않다."""
    mins = {sid: (v[0] if v is not None else None) for sid, v in limits.items()}
    maxes = {sid: (v[1] if v is not None else None) for sid, v in limits.items()}
    return mins, maxes


def split_taught(taught: dict[int, tuple[int, int]]):
    """`TAUGHT_POSITION_LIMITS` 형태를 (mins, maxes) 로 펼친다 — 위와 짝."""
    mins = {sid: lo for sid, (lo, _hi) in taught.items()}
    maxes = {sid: hi for sid, (_lo, hi) in taught.items()}
    return mins, maxes
