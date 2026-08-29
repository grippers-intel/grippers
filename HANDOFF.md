# Grippers Pi — 인수인계 (2026-08-30 갱신)

> ## ⚠️ 먼저 — 이 팔이 지금 어느 캘리브레이션인지 확인하세요
>
> 2026-08-29 에 VLA 시연 수집을 준비하며 **LeRobot 캘리브레이션이 서보의
> `Homing_Offset` 을 덮어썼습니다.**
>
> ```
> Present_Position = Actual_Position - Homing_Offset
> ```
>
> `floor_grasp_profiles.py` 의 교시 자세는 RAW 서보값이라, 오프셋이 바뀌면
> **같은 숫자가 다른 물리 자세**가 됩니다.
>
> **오프셋은 서보 EEPROM 에 있지 git 에 있지 않습니다.** `git checkout` 으로
> 바뀌지 않습니다. 그래서 두 갈래를 이렇게 나눠 씁니다.
>
> | 하는 일 | 브랜치 | 팔의 캘리브레이션 |
> |---|---|---|
> | 베이스라인 미션 | `kica927/baseline_mission` | **교시 당시** (되돌린 상태) |
> | VLA 시연 수집·추론 | `kica927/smolVLA-version` | **LeRobot 새 캘리브레이션** |
>
> 확인:
> ```
> python3 tools/arm/restore_taught_offsets.py
> ```
>
> 베이스라인으로 되돌리기 (**팔이 중력으로 내려옵니다 — 아래를 비우고**):
> ```
> python3 tools/arm/restore_taught_offsets.py --apply --yes
> ```
>
> `arm_driver_node` 가 기동할 때 이것을 대조하고, 다르면 **기동을
> 거부합니다**(`ArmCalibrationMismatchError`). 경고가 아니라 거부인 이유는
> shoulder_pan 가동폭이 2493 → 2087 로 줄어 있어(차체·라이다에 막힘)
> 어긋난 채 움직이면 부딪히기 때문입니다.
>
> 팔을 다시 교시했다면 `floor_grasp_profiles.TAUGHT_HOMING_OFFSETS` 도 같이
> 갱신하세요. 자세와 오프셋은 한 쌍입니다.

이 파일은 **짧은 진입점**이다. 상세 이력·실측·근거는 `grippers_docs/`
(맥 `~/Desktop/intel/grippers_docs`)의 다음 두 문서가 현행이다.

- `grippers_작업정리_20260828.md` — 문서·Pi 저장소·Host 저장소 종합 (먼저 읽을 것)
- `grippers_handover_20260827.md` — 08-27~28 Pi 작업 상세, 실측표, Pi 실행 상태
- `grippers_host_requests_20260827.md` — Host 팀이 고쳐야 할 것 + 번역 코드 초안

이전 버전(2026-08-24)에 있던 `scan_track_return.py`·`auto_approach_grasp_rook.py`·
그리퍼캠 절차는 **2026-08-26 역할 분담 확정으로 전부 삭제됐다.** 그 문서를
근거로 작업하지 말 것.

---

## 0. 작업 규칙 (먼저 읽을 것)

- **존댓말**: 한국어 응답은 항상 존댓말.
- **Pi 접속**: `ssh pi@raspberrypi.local` (mDNS가 안 잡히면 IP 직접, 과거 `10.82.133.189`, DHCP).
- **컨테이너 진입 (사람, 대화형)** — 반드시 진짜 TTY에서:
  ```
  cd ~/docker && ./exec_shell.sh
  ```
- **컨테이너 진입 (자동화, 비대화형)**:
  ```
  docker exec IntelPi bash -lc '명령'
  ```
- **셸 방언**: `exec_shell.sh` 세션은 zsh → `setup.zsh`. `bash -lc` 경로는 bash → `setup.bash`.
- **`ROS_DOMAIN_ID=21`** — 컨테이너 안 모든 셸에서 예외 없이 가장 먼저 export.
- **ROS 환경 (bash 경로)**:
  ```
  export ROS_DOMAIN_ID=21
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash
  ```
  `peripherals/depth_camera.launch.py`·`controller/odom_publisher.launch.py`는
  `need_compile`, `DEPTH_CAMERA_TYPE=ascamera`, `MACHINE_TYPE=MentorPi_Mecanum`을 export로 넘길 것.
- **경로**: Pi 호스트 `~/docker/shared/grippers` = 컨테이너 `/grippers`. 맥 클론과 별개 클론.
- **배포**: `domain/`·`tools/`는 `git pull`만. `ros2_ws/src/**`는 `colcon build --packages-select <패키지>`
  후 해당 노드를 PID로 골라 재기동 (`pkill -f "ros2 run grippers"`처럼 뭉뚱그리지 말 것).
- **배포·재시작 뒤 `perception_node` 반드시 재기동** (`depth_cam_rotate_node`도 같이).
- **원격**: `origin`(조직) + `personal-mirror`(개인) 둘 다 push. 브랜치·PR은 `kica927/` 접두어.
  PR은 올리되 사용자 확인 전 병합 금지.
- 사용자에게 주는 셸 블록에 `#` 주석 금지. 저장소 `docs/`는 권위 자료로 취급하지 않음.

---

## 1. 지금 상태 (2026-08-28)

| 항목 | 상태 |
|---|---|
| 브랜치 | `kica927/baseline_mission` — `origin`·`personal-mirror` 동기화, 개인 미러 main은 PR #42까지 머지 |
| 테스트 | `tests/` 422개 통과 (`PYTHONPATH=. python -m pytest tests`) |
| Pi 단독 기능 | 파지 → CARRY → 저속 접근 → 자동정지 → INSERT, **여섯 클래스 전부 실기 검증** (08-27) |
| Pi 실기 배포 | Pi는 `c3a2bb1`에서 멈춤 — `28d4626`→`aca9d75` 네 커밋 미배포(LAN 끊김). `git pull`만 하면 됨 |
| Host ↔ Pi 연동 | 🔴 한 번도 붙여 본 적 없음. Host 저장소가 확정 이전 규격이라 그대로 붙이면 차량이 안 움직임 |

## 2. 구조 한 줄씩

- `domain/task/baseline_mission.py` — 명령 구동형 FSM `IDLE→APPROACH→GRASP→CARRY→APPROACH_BOX→INSERT→DONE`.
- `domain/task/baseline_constants.py` — 실측/지시 상수. `unresolved()`는 비어 있다.
- `domain/task/motion.py` / `preconditions.py` / `corrections.py` — 속도 클램프, GRASP/INSERT 조건, Host용 `fix`.
- `domain/adapters/real/udp_host_link.py` — Host↔Pi UDP(5005 명령 / 5006 보고).
- `ros2_ws/src/grippers_mission` — `mission_orchestrator_node` (10 Hz 루프).
- `tools/basket_approach_insert_test.py --profile <클래스>` — INSERT 통합 harness.
- `tools/grasp_geometry_calibrate.py --mode k|jaw|load|scale|confirm` — 파지 기하 실측 도구.
- `tools/host_link_conformance.py --as-is|--translated` — Host 실제 코드와 로컬 적합성 시험(하드웨어 불필요).

## 3. 다음 접속 시 순서

1. LAN 복구 → 컨테이너 `/grippers`에서 `git pull` → `aca9d75` 확인.
2. `28d4626`(INSERT 좌우 오프셋 게이팅) 실기 확인 — 일부러 오프셋을 만들어 ⛔ 분기가 도는지.
3. `mission_orchestrator`를 `use_fake_base:=false`로 재기동 (지금은 `true`로 떠 있음).
4. `perception_node`·`depth_cam_rotate_node` 확인.
5. Host 팀이 `grippers_host_requests_20260827.md` 1~2번을 반영하면 루프백 → 실기 통합.
