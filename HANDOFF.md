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

## 1. 지금 상태 (2026-09-05 갱신 — git log·코드로 재확인, 아래 §1a 참고)

| 항목 | 상태 |
|---|---|
| 브랜치 | `kica927/baseline_mission` @ `b855096` — `origin` 대비 28커밋, `personal-mirror` 대비 2커밋 앞섬(둘 다 미push) |
| 테스트 | 08-27 기준 422개 통과 확인 이후 `test_stm32_motor_watchdog.py` 등 다수 추가됨 — 정확한 현재 개수는 `PYTHONPATH=. python -m pytest tests` 재실행 확인 필요 |
| Pi 단독 기능 | 파지 → CARRY → 저속 접근 → 자동정지 → INSERT, **여섯 클래스 전부 실기 검증** (08-27) |
| Host ↔ Pi 연동 | 09-02부터 실기로 연동된 것으로 보임(아래 §1a) — 이전 버전의 "🔴 한 번도 안 붙어봄"은 08-28 스냅샷이 갱신 안 된 채 남아 있던 것 |
| `use_fake_base` 기본값 | 코드(`mission_orchestrator_node.py:68`)상 `False`(진짜 바퀴) — 이전 버전의 "지금은 true로 떠 있음"도 같은 이유로 낡은 값이었음 |
| 모터 워치독 | `d289195`(09-05)로 STM32 write_timeout(0.2s)·모터 워치독(0.5s) 추가. **두 값 다 추측값** — 실기 정상 왕복 지연을 재고 조정할 것(§3-1 계속 미해결) |

### 1a. Host↔Pi 연동 판단 근거와 남은 확인

- `domain/ports/baseline_ports.py`의 `HostCommand`가 이미 확정 5필드(`state`/`linear_x`/`linear_y`/`angular_z`/`stop`) 규격으로 구현돼 있다 — 8/27 요청 문서가 지적한 "확정 이전 규격" 문제가 아니다.
- git log에 09-02 실기, 09-04 밤(toy 입구 밖 투하 사고) 등 **Host 명령으로 로봇이 실제로 움직인 사건**이 날짜별로 기록돼 있다. "투하 사고"는 옮기다 실패한 사건이지 연결이 안 됐다는 뜻이 아니다.
- 다만 이 갱신은 로컬 git log·코드 대조로 재구성한 것이고, **지금 이 순간 Pi가 그 상태로 떠 있는지는 이 세션에서 SSH로 재확인하지 못했다.** 다음 접속 시 컨트롤러→orchestrator 순서로 띄운 뒤 `ros2 topic info /cmd_vel`의 구독자 수로 확인할 것(`RUNBOOK_2026-09-08.md` §3.5 참고).

## 2. 구조 한 줄씩

- `domain/task/baseline_mission.py` — 명령 구동형 FSM `IDLE→APPROACH→GRASP→CARRY→APPROACH_BOX→INSERT→DONE`.
- `domain/task/baseline_constants.py` — 실측/지시 상수. `unresolved()`는 비어 있다.
- `domain/task/motion.py` / `preconditions.py` / `corrections.py` — 속도 클램프, GRASP/INSERT 조건, Host용 `fix`.
- `domain/adapters/real/udp_host_link.py` — Host↔Pi UDP(5005 명령 / 5006 보고).
- `ros2_ws/src/grippers_mission` — `mission_orchestrator_node` (10 Hz 루프).
- `tools/basket_approach_insert_test.py --profile <클래스>` — INSERT 통합 harness.
- `tools/grasp_geometry_calibrate.py --mode k|jaw|load|scale|confirm` — 파지 기하 실측 도구.
- `tools/host_link_conformance.py --as-is|--translated` — Host 실제 코드와 로컬 적합성 시험(하드웨어 불필요).

## 3. 다음 접속 시 순서 (2026-09-05 갱신 — 아래는 09-05 세션 기준 재작성, 실기 재확인 전제)

1. 컨테이너 `/grippers`에서 `git pull` → `b855096` 확인(§1 참고, 위 항목들은 로컬 git log 기준이라 Pi 쪽이 뒤처져 있을 수 있음).
2. §1a대로 `ros2 topic info /cmd_vel` 구독자 수로 Host↔Pi 연동이 실제로 살아 있는지 확인.
3. `perception_node`·`depth_cam_rotate_node` 확인.
4. 물리 상수 실측 3종 진행 — `T_stop`(정지 지연·오버슈트), 모터 워치독 발동 시간(위 §1의 write_timeout/watchdog 추측값 검증 겸), `identify_target` 6클래스 왕복 지연. `pi_capture/mac/analyze_stop.py`·`analyze_watchdog.py`로 분석.
5. 사선 진입 INSERT 15°/30° 실측 — `grippers-host-mac/host/manual_insert_probe.py` (WASD 수동 접근, 화면에 `pose.yaw_deg`·`gate.facing_error_deg` 실시간 표시됨 — 추가 개발 불필요, 바로 사용 가능).
