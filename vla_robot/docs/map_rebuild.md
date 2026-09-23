# 맵 재구축 절차

작업장을 다시 세운 뒤 `host/config/host.yaml` 을 확정하는 순서입니다. 도구는 전부
`host/` 에서 실행합니다(`cd vla_robot/host`).

각 단계는 **재고 → 확인하고 → 적는다** 순입니다. 눈대중으로 적으면 그 오차가 주행
전체에 퍼지고, 어디서 틀렸는지 나중에 찾기 어렵습니다.

## 0. 카메라 내부파라미터 (한 번만)

```bash
python tools/calibrate_camera.py --cam 0 --cols 9 --rows 6 --square 0.025
python tools/calibrate_camera.py --cam 1 --cols 9 --rows 6 --square 0.025
```

`host/calib/cam0.npz` · `cam1.npz` 가 생깁니다. **없으면 근사 내부파라미터로 돌아 위치
오차가 수 cm 까지 커집니다.** 초점이 바뀌면(오토포커스가 켜졌다면) 다시 잽니다.

## 1. 카메라 배치를 먼저 판단한다 — 하드웨어 없이

```bash
python tools/check_coverage.py --height 1.30 --setback 0.0 --tilt 42.8 --wall-height 0.25
```

세울 높이·후퇴·하향 각도를 넣으면 가벽과 상자의 **가림까지 반영해서** 판정합니다.

- 바닥 마커를 카메라마다 `aruco.min_floor_markers`(기본 2) 이상 보는가
- 작업 구역이 전부 덮이는가(한 대 이상 100 %), 두 대가 겹쳐 보는 띠는 얼마나 되는가
- 최악 지점의 mm/px — 40 mm 물체가 몇 px 로 잡히는지

현재 배치(1.30 m · 후퇴 0 · 42.8°) 기준값: 한 대 이상 **100 %** · 두 대 **66 %** ·
최악 **2.38 mm/px**. 이보다 나빠지면 높이나 각도를 바꿔 다시 돌려 봅니다.

> ⚠️ 카메라 위치는 `host.yaml` 에 없습니다 — 외부파라미터를 바닥 마커로 매번 풀기
> 때문입니다. 그래서 이 도구에는 **실제로 세운 값**을 인자로 넣어야 답이 맞습니다.

## 2. 바닥 마커 좌표를 계산한다

줄자로 재서(cm) 넣으면 붙여 넣을 YAML 블록이 나옵니다.

```bash
python tools/make_layout.py --x1 10 --y1 40 --width 160 --depth 100        # 직사각형
python tools/make_layout.py --m1 10 40 --m2 170 40 --m3 10 140 --m4 170 140  # 네 점 실측
```

- 원점은 **가벽 앞쪽 왼쪽 바닥 모서리**, +x 오른쪽, +y 뒤쪽(상자 쪽)
- 재는 지점은 **검은 사각형의 중심**
- 인쇄물은 **윗변이 +y 를 향하게** 붙입니다(코너 순서 전제)
- 번호 1·2·3·4 = 앞왼 · 앞오른 · 뒤왼 · 뒤오른

가벽 밖, 너무 모임, 대각선 불일치는 이 도구가 경고합니다.

## 3. 붙이면서 실시간으로 확인한다

```bash
python tools/place_markers.py
```

STATUS 창만 보면 됩니다. **재투영오차 1 px 미만이면 GOOD**, 2 px 이상이면 좌표나 붙인
위치가 틀린 것이니 2번으로 돌아갑니다. 대향 배치에서 카메라마다 가까운 2장만 보이는
것은 정상입니다.

## 4. 측위 품질을 잰다

로봇을 작업 구역 가운데에 세워 두고:

```bash
python tools/check_localization.py --seconds 20
```

| 값 | 기준 |
|---|---|
| 재투영오차 | < 1 px |
| 위치 흔들림 | < 5 mm |
| yaw 흔들림 | < 1.0° |
| 놓침 비율 | 0 % |

놓치면 주행 중 Host 가 그때마다 정지합니다(`pose_hold_s` 만큼만 버팀).

## 5. 로봇 마커 축 보정

차가 움직일 수 있으면 주행 측정이 정확합니다(사람 판단이 안 들어갑니다).

```bash
python tools/calib_yaw.py --drive --pi 192.168.0.7 --runs 4 --distance 0.25
```

차가 못 움직이면 정지 측정으로, **두 축 모두** 잽니다.

```bash
python tools/calib_yaw.py --facing x
python tools/calib_yaw.py --facing y
```

한 축만 재면 놓기 오차와 실제 축 어긋남을 구분할 수 없습니다. 두 축이 같은 방향·같은
크기로 치우칠 때만 `aruco.yaw_offset_deg` 를 고칩니다.

> 이 값이 틀리면 로봇이 "정면을 봤다"고 믿고 비스듬히 갑니다 — 좌우로 흔들리며 전진하는
> 증상(2026-09-05 실기)이 그것입니다.

## 6. 상자와 투입 목표

상자는 뒷 가벽에 **밀착**, 긴 변(0.35 m)이 **y 축과 나란하게** 놓습니다. 그러면
중심 y = 1.8 − 0.175 = **1.625** 입니다. 좌우 위치는 측벽에서 상자 변까지 재서
`중심 x = 측벽거리 + 폭/2` 로 적습니다(2026-09-22 실측: 좌측벽 0.35 m · 폭 0.21 →
toy 중심 0.455, 현재 값 0.450 과 5 mm 차이라 그대로 둠).

투입 목표(`insert_*`)와 정차점은 이 값에서 자동으로 나옵니다 —
`host/mission/basket_target.py`, 도면은 `docs/layout/workspace_layout.md`.

## 7. 확정 후

```bash
cd .. && python -m pytest host/tests -q
```

`host.yaml` 이 테스트를 통과하는지 봅니다(좌표·부채꼴·정차 판정이 함께 검사됩니다).
그다음 차체 기동 시험(`tools/udp_teleop.py`) → `place.base_yaw_deg` 재측정 순서입니다.
