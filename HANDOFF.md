# Grippers — 작업 인수인계 (2026-08-23 06:00)

실기체 자율 파지 파이프라인 구축 현황. 검증된 수치와 다음 할 일.

---

## 지금 바로 할 일 (진행 중이던 순서)

### ① 팔 복귀 — 아직 안 했으면 최우선

팔이 바닥을 밀며 차체를 들어올린 상태로 방치하면 어깨 서보가 상한다.

```bash
ssh -t pi 'docker exec -it IntelPi bash -lc "cd /grippers && PYTHONPATH=/ros2_ws/src/grippers_arm:/third_party/soarm_provided_d python3 tools/return_home.py --accel 40"'
```

**어댑터 연결 필수.** 메인 배터리가 6.9V까지 떨어져 베이스가 안 움직인다(2셀 팩 기준 거의 방전).

### ② 기준값 교시 — 파지 위치를 다시 정확히 잡는다

a. 파지 자세로 그리퍼를 보낸다 (Enter 3번, "파지 중심 45mm로 이동"까지):

```bash
ssh -t pi 'docker exec -it IntelPi bash -lc "cd /grippers && PYTHONPATH=/ros2_ws/src/grippers_arm:/third_party/soarm_provided_d python3 tools/horizontal_grasp_hardware_test.py chess_rook --accel 40"'
```

b. 룩을 두 손가락 **정중앙**에 놓고 `q` 로 중단
c. 팔 접기 — `return_home.py --accel 40`
d. 그 위치를 기준으로 저장:

```bash
ssh pi 'docker exec IntelPi bash -lc "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=21 && python3 -u /grippers/tools/perception/approach.py --teach --note \"chess_rook 파지위치\""'
```

### ③ 접근 루프 — 먼저 움직이지 않고 방향만 확인

로봇을 뒤로 20~30cm 물린 뒤:

```bash
ssh pi 'docker exec IntelPi bash -lc "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=21 && python3 -u /grippers/tools/perception/approach.py --dry-run"'
```

`높이=` 오차가 **+(양수)면 아직 멀다**는 뜻. 정상이다.

### ④ 실제 접근

```bash
ssh pi 'docker exec IntelPi bash -lc "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=21 && python3 -u /grippers/tools/perception/approach.py"'
```

- 좌우가 반대로 가면 → `--invert-y`
- 너무 느리면 → `--gain-h 0.003 --gain-x 0.0018`
- 출렁이면 → 게인을 절반으로

**수렴하면 자율 파지의 마지막 조각이 맞춰진다.** 이후 파지→운반→투하는 전부 검증된 것이라 이어 붙이기만 하면 된다.

---

## 검증 완료된 것

### 인식

| 항목 | 결과 |
|---|---|
| 추론 | **NCNN CPU 14.3 FPS** (640px, 학습 해상도 그대로) |
| 합의 필터 | 프레임당 4~12개 검출 → 물체 단위로 수렴, **산포 0.2~1.1px** |
| 6개 클래스 | `b_hard` 촬영본에서 **전부 순도 1.00** (queen .95 / knight .93 / box .90 / star .86 / rook .82 / soccer .61) |
| 거리 게이트 | **y ≥ 290** — 빈 바닥 대조군에서 오탐 최대 y=277, 진짜 물체 최소 y=293 |

동작점: `conf 0.45 · k-of-n 0.6 · 순도 ≥0.80 · y ≥290 · 산포 ≤40px`

**한계** — 빈 아레나에서 오탐 4개가 60프레임 내내 일관되게 나온다(벽 밑동·바구니 주변).
배경이 정지해 있어 합의 필터로는 원리적으로 못 거른다. 거리 게이트로 막는다.

### 파지 (하드웨어 실측 검증 완료)

`horizontal_grasp_hardware_test.py chess_rook --drop-to-basket` 전 구간 성공.

- 파지 부하 `0.0665` → 운반 중 `0.0587` 유지 (최소 기준 0.04)
- 온도 20→25°C, 수렴 오차 servo2 13카운트
- 바구니 투하 정상 (테두리 위 80mm에서 놓음, 튐 허용 범위)

**수평 파지가 맞다.** 체스 기물은 몸통(45~60mm)을 옆에서 감싸 쥔다. 수직 하강은 이 하드웨어에 부적합.

### 개선한 것

- **움직임 진동** — `glide` 보간을 30스텝×0.1s → **90스텝×0.034s**. 총 시간 동일, 스텝당 이동 1/3.
  수렴 오차가 servo2 기준 36→13카운트로 개선. `--accel 40` 도 쓸 만하다.
- `return_home.py` 신규 — `align_to_idle` 은 IDLE 근처(편차 800)만 처리한다.
  파지 자세(편차 2296)에서는 거부하므로, 검증된 경로(현재→SAFE_145→IDLE)로 복귀시키는 도구를 만들었다.

---

## 실패한 접근과 이유 (반복하지 말 것)

**Hailo-10H — 사용 불가로 판정.** PCIe 열거·드라이버 적재·펌웨어 파일 모두 정상인데
칩이 u-boot 단계에서 100% 재현적으로 멈춘다(`Timeout waiting for firmware file` →
`Failed writing SOC firmware on stage 2`). Gen3→Gen2, 완전 전원 차단, 모듈 재적재,
`force_hailo10h_legacy_mode`(Hailo-15용이라 무관), 패키지 업데이트(5.1.1이 최신) 전부 실패.
파이 전원은 정상(5A 인식, `throttled=0x0`). **하드웨어 또는 물리적 접촉 문제로 본다.**
판정용 후속 시험 — 여분의 파이 5에 모듈을 옮겨 꽂아보면 모듈 불량인지 로봇 쪽 문제인지 갈린다.

**수동으로 파지 위치 맞추기 — 실패.** 화면 x 를 목표(168)에 맞췄는데도 팔이 물체를
지나쳐 내려가 차체가 들렸다. 원인: **x 는 거리와 좌우 위치를 함께 담는다.**
물체가 중심선에서 벗어나 있어 가까워지면 x 가 줄지만, 로봇이 옆으로 밀리거나
살짝 회전해도 똑같이 변한다. 앞뒤로 여러 번 오가며 드리프트가 쌓였다.
→ **해결: 박스 높이(거리) + x(좌우) 두 독립 신호로 제어 루프를 돌린다.** 그게 `approach.py`.

---

## 실측 수치

| 값 | 결과 |
|---|---|
| 파지 지점 | 화면 (168, 480) — **y=480 은 포화값이라 거리로 못 쓴다** |
| 커밋 라인 | 화면 (221, 423), 전진 0.15m (±0.02) |
| 파지 프로파일 | `chess_rook` 폭 24.5mm, 파지높이 45mm, 예열림 80mm, 닫기 15mm |
| 파지 기하 | 차체 전면 기준 전방 185mm, 중심선 좌측 20mm |
| 텔레옵 전송 | 50Hz, 패킷 유실 0%, 지터 중앙값 19.9ms |
| 리더/팔로워 서보 | 6관절 읽기 2.0ms / 1.7ms |

---

## 환경 함정 (다시 안 밟도록)

- **뎁스카메라는 OpenCV 로 직접 열면 안 된다.** YUYV 1280x1040 은 RGB+뎁스 결합
  원시 스트림이라 초록/보라 띠만 나온다. `ascamera` ROS 드라이버 경유가 유일한 길.
  그 패키지는 `/ros2_ws` 가 아니라 **`third_party_ros2/third_party_ws`** 에 있다.
  **카메라가 거꾸로 달려 있어 180도 회전 필수.**
- **환경변수 누락이 잦다** — `need_compile`, `DEPTH_CAMERA_TYPE=ascamera` 가 없으면 런치가 KeyError 로 죽는다.
- **`imu_calib` 패키지가 없다.** `controller.launch.py` 는 그것 때문에 런치 전체가 SIGINT 로 죽는다.
  대신 `odom_publisher.launch.py` 를 띄운다. 그래서 **EKF 가 없고 `/odom` 도 없다 — `/odom_raw` 를 쓸 것.**
- **경로 두 가지** — 호스트 `/home/pi/docker/shared/grippers/…` = 컨테이너 `/grippers/…`.
  래퍼(`~/capture`, `~/teach`)는 호스트에서 실행한다.
- **SSH 별칭** `pi` = 192.168.2.3 (유선), `pihotspot` = 아이폰 핫스팟 링크로컬.

---

## 만든 도구

| 파일 | 역할 |
|---|---|
| `tools/perception/consensus.py` | 다중 프레임 합의 — 클래스별 클러스터링, k-of-n, 중앙값, 순도 |
| `tools/perception/floor_observer.py` | 카메라 → 확정된 물체 목록 (게이트 포함) |
| `tools/cycle.sh` | **자율 사이클: 접근→파지→운반.** 호스트에서 `ssh -t pi '~/cycle'` |
| `tools/perception/approach.sh` | 접근 루프만 실행 (스택 자동 기동) — `~/approach` |
| `tools/perception/approach.py` | 자동 접근 루프 — 부분 검증(수렴 직전까지) |
| `tools/perception/aruco_observer.py` | ArUco 관측기 — 바구니용. YOLO 와 같은 `Observation` 을 낸다 |
| `tools/perception/make_markers.py` | 인쇄용 마커 생성 (`/grippers/markers/`) |
| `tools/perception/overlay.py` | 합의 전/후 비교 그림 생성 — 발표 자료용 |
| `tools/perception/measure_commit.py` | 커밋 거리 측정 (거리가 부호 없음 — 개선 필요) |
| `tools/perception/eval_consensus.py` | 촬영본으로 임계값 튜닝 |
| `tools/capture/capture.sh` | 프레임 촬영 (드라이버 자동 기동) — 호스트에서 `~/capture <라벨> [초]` |
| `tools/return_home.py` | 팔을 검증된 경로로 IDLE 복귀 |
| `tools/teach/teach.sh` | 자세 교시 (텔레옵 필요) — 아직 미사용 |
| `tools/teleop/` | 리드암+키보드 텔레옵. `./teleop.sh --base-only` 로 베이스만도 가능 |
| `config/grasp_target.json` | 파지 지점·커밋 라인 실측값 |
| `config/approach_target.json` | 접근 루프 기준값 — **②에서 생성됨** |

---

## 접근 루프에서 배운 것 (2026-08-23)

- **모터 데드밴드가 실재한다.** 오차가 줄면 P 제어 지령이 0.02 m/s 아래로 내려가는데
  그 속도로는 바퀴가 정지마찰을 못 이긴다. 명령은 정상인데 로봇만 안 움직이는,
  가장 헷갈리는 증상이 나온다. `apply_floor()` 가 속도를 올리고 시간을 줄여
  **이동 거리는 유지한 채** 데드밴드를 넘긴다.
- **전압이 낮으면 데드밴드가 올라간다.** 7.2V 에서 같은 증상이 재현됐다.
  그래서 `cycle.sh` 는 9.5V 미만이면 아예 시작하지 않는다.
- **전진과 좌우 보정은 결합돼 있다.** 전진하면 물체의 좌우 편차가 원근으로 확대되므로,
  횡보정이 못 따라가면 x 가 목표를 지나쳐 계속 밀려난다(실측: 224→134, 목표 170).
  `--align-first` 가 좌우가 크게 어긋난 동안 전진을 1/4 로 줄여 끊는다.
- **부호는 맞다.** `--invert-y` 는 필요 없었다.

## 바구니 접근 — 인식기만 갈아끼운다 (2026-08-23)

접근 루프는 목표가 무엇인지 모른다. `Observation(x, h, …)` 하나만 받는다. 그래서
바구니는 **새 내비게이션이 아니라 새 인식기** 문제다.

    YOLO   → Observation ┐
                         ├→ approach.py → 목표 앞 정렬
    ArUco  → Observation ┘

- 바구니를 YOLO 로 잡으려면 렌더·라벨링·재학습이 필요하다. ArUco 는 학습이 없다.
- ID 로 바구니를 구분한다 (빨강=4, 파랑=5 식).
- **마커가 보여야만 한다.** 파지 직후 로봇은 물체 쪽을 보고 있으므로
  `--search` 가 제자리에서 돌며 찾는다.
- 교시값은 목표별로 나뉜다 (`approach_target_<cls>.json`). 예전 룩 교시본
  (`approach_target.json`) 은 그대로 읽힌다.
- **아직 안 푼 것: 요(yaw).** x·박스높이는 2 자유도만 묶는다. 로봇이 비스듬히
  접근하면 그리퍼가 바구니 중심에서 벗어난다. 바구니 입구가 넓어 넘어갈 수도
  있고, 안 되면 마커 좌우 변의 높이 차(원근 왜곡)를 세 번째 신호로 쓰면 된다.

검증 상태 — 카메라→검출→합의 파이프라인 동작 확인. 마커 6종 생성·왕복 검출 6/6.
**실제 바구니 접근은 미검증**(마커 미부착).

## 사람을 기물로 확정한다 (2026-08-23 실측)

카메라가 사무실 쪽을 향한 상태로 `overlay.py` 를 돌렸더니 **사람을 `knight` 0.90 으로,
10 프레임 전부에서** 검출했다. 산포 1.5px. 합의 필터는 이걸 못 막는다 — 여러
프레임에서 *일관되게 틀린* 검출은 필터가 정답과 구분할 수 없다.

학습 데이터가 아레나 렌더뿐이라 도메인 밖 물체는 무엇이든 6 클래스 중 하나로
끌려간다. 거리 게이트(y≥290)가 먼 곳을 잘라주지만, 가까이 선 사람은 통과한다.

**How to apply:** 촬영·시연 중에는 로봇 카메라 시야에 사람이 들어가지 않게 한다.

## 파지 지점은 카메라가 못 본다 (2026-08-23 실측)

**시각 서보만으로는 파지 지점에 갈 수 없다.** 가까워질수록 물체가 화면 아래로
잘려 나가 박스 높이가 거리 신호 구실을 못 하고, 결국 검출 자체를 잃는다.

    교시값 +15px  →  안정적으로 수렴. 손가락 끝까지 약 10mm 남음
    교시값 +20px  →  검출을 잃는다
    교시값 +30px  →  검출을 잃는다

이건 `config/grasp_target.json` 에 팀이 이미 적어둔 관찰과 같다 — y 가 480 으로
포화되고 그보다 가까운 거리는 전부 같은 값으로 읽힌다. 그 문서의 처방(커밋 라인까지
서보 → 개루프 전진)이 맞았다.

**다만 개루프 구간을 훨씬 짧게 만들 수 있다.** 문서의 커밋 라인은 0.15m 를 개루프로
가야 해서 오차가 ±0.02m 였다. 접근 루프로 10mm 앞까지 붙인 뒤 남은 구간만 가면
그 오차가 거의 사라진다. `approach.py --final-push <mm>` 이 `/odom_raw` 로 재면서
전진한다.

**튜닝 단위가 픽셀에서 밀리미터로 바뀐 게 실질적인 이득이다.** 눈으로 본 간격을
그대로 넣으면 된다.

## 마무리 전진 — 실측으로 얻은 것들 (2026-08-23)

**1. 전진 속도는 접근 속도와 달라야 한다.** `--min-speed` 0.05 는 데드밴드
경계라 정지 상태에서 출발하면 바퀴가 안 돈다. 전진 15mm 와 25mm 가 **같은
결과**를 낸 게 그 증거였다 — 둘 다 안 움직인 것이다. `--push-speed` 0.09 를 쓴다.

**2. 오도메트리는 약 13% 과대 보고한다.** 지시 100mm 에 실제 87mm.
`--push-scale` 1.15 로 보정한다.

**3. 한 번에 밀면 관성으로 넘어간다.** 25mm 지시가 42.8mm 가 됐다. 그래서
짧게 밀고 **완전히 멈춘 뒤 재기**를 반복한다. 관성 몫이 다음 회차 측정에
자동으로 반영된다.

**4. 파지 부하로는 "얼마나 빗나갔는지" 를 알 수 없다.** 그리퍼는 손가락 안쪽
밑동에서 닫히므로, 물체가 손끝 바깥에 있으면 10mm 바깥이든 60mm 바깥이든
부하가 똑같이 0.0313 이다. **부하가 안 변한다고 거리 문제가 아니라고 판단하면
안 된다** — 실제로 그렇게 오판해서 좌우 오차를 의심했으나 사진으로 확인하니
순수한 앞뒤 문제였다.

**진단은 사진이 가장 빠르다.** 접근만 실행 → 파지 자세까지 전개(Enter 3번) →
위에서 찍어 손가락과 물체의 관계를 본다. 숫자로 추측하는 것보다 훨씬 빠르다.

## ✔ 전체 사이클 성공 (2026-08-23)

    ssh -t pi '~/cycle --push 57 -- --frames-far 4'

접근(20회 수렴) → 전진 57.9mm → 파지 → 상승 → 운반 → 복귀까지 자동으로 완주했다.

    파지 부하   0.0665  (임계 0.04)
    mid-lift    68
    safe-145    68      세 단계 모두 동일 = 미끄러짐 없음
    carry-idle  68
    서보 온도   최고 30°C

**확정된 값** — `--offset-h 15`, `--push 50`. 둘 다 cycle.sh 기본값이라
`ssh -t pi '~/cycle'` 만으로 돈다. 50 에서 물체를 밀지 않고 깨끗하게 잡는다.
두 번 연속 재현했다(57 → 살짝 밀림, 50 → 깨끗).

## 알려진 미해결
- `measure_commit.py` 의 거리가 원점으로부터의 **직선거리**라 앞뒤 구분이 안 된다
- 빈 배경 오탐 4개 (거리 게이트로 회피 중)
- 루트 파티션 79% (13GB 여유)
