# SO-ARM101 그리퍼 캘리브레이션

2026-08-20에 `/dev/soarm`의 servo 6을 실측한 결과다. 개구 폭은 물체가 들어가는
두 핑거의 마주 보는 안쪽 면 사이 거리로 측정했다.

| 상태 | goal | present | 개구 폭 | load |
|---|---:|---:|---:|---:|
| CLOSED 안전점 | 1150 | 1153 | 9 mm | +24 |
| 중간 실측점 | 1578 | 1572 | 96 mm | -36 |
| OPEN 안전점 | 2000 | 1994 | 약 168 mm | -36 |

- `homing_offset = 1343`
- count 증가 = OPEN, count 감소 = CLOSE
- 안전 명령 범위 = `1150..2000`
- 폭 계약 = `9..168 mm`

endpoint 두 점을 단일 선형 보간했을 때 `90 mm → goal 1578` 명령의 실제 개구 폭은
약 96 mm였다. 따라서 서비스는 요청 폭을 `9..168 mm`로 clamp한 뒤
`(9, 1150)`, `(96, 1578)`, `(168, 2000)` 세 실측점 사이를 구간별 선형 보간한다.
보정 후 `90 mm`의 goal은 `1548`이다. endpoint에는 부하와 위치 오차가 포함된 present
값이 아니라, 실제로 명령하고 안전성을 확인한 goal 값을 사용한다.

`homing_offset`은 측정 당시 좌표계를 식별하기 위한 기록이며 EEPROM 설정을 코드에서
변경하지 않는다. 과거의 `984`, `2318`, 다른 `homing_offset`에서 기록된
`follower.json`의 `2046..3093`은 현재 조립체의 안전 목표로 사용하지 않는다.

third-party `driver_sdk.py`의 `JOINT_LIMITS[6]` 기본값은 다른 조립체용이고,
`data/calibration.json`에 의해 import 시 override될 수도 있다. 이 저장소는 서브모듈을
수정하지 않고 application-owned `gripper_calibration.py`에서 servo 6의 폭 변환을
관리한다. 따라서 Pi의 외부 calibration 파일과 무관하게 이 서비스의 안전 범위가
유지된다. URDF의 `rad_min/rad_max`는 렌더링/관절 모델 범위이므로 이번 실측에서
변경하지 않았다.
