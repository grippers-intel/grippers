# 8/23 Hailo-free scripted demo runbook

이 런북은 Hailo 장애가 해소되지 않은 상태에서 2026-08-23 제출 영상을 만들기 위한
비상 경로다. Hailo를 영구 폐기하는 결정이 아니며, 제출 후 진단·교차시험·RMA는
GitHub #189에서 계속한다.

## 시연의 정확한 범위

이 경로는 **perception-bypassed, operator-gated scripted pick-and-place**다.

- 물체 위치는 카메라가 찾지 않는다. 물체를 바닥 표시점에 사람이 사전 배치한다.
- 팔은 전역 지도 좌표 `(x, y, z)`가 아니라 저장소에 등록된 STS3215 서보 waypoint를
  순서대로 실행한다.
- 베이스는 정지시킨다. 이동이 꼭 필요하면 사람이 정지점까지 수동 조작한 뒤 팔 동작과
  분리한다.
- 모든 물리 전환 전에 운영자가 Enter로 승인하며 `q`는 다음 전환 전에 중단한다.
- 저장소에는 리드암 자세 녹화·재생 구현이 없다. 별도의 follower/teach 프로그램이
  **해당 장비에서 이미 확인된 경우에만** 표시점·자세 교시에 보조적으로 쓴다.

다음 문구를 영상과 PPT에 그대로 표시한다.

> 긴급 보조 시연: Hailo 인식을 우회하고, 사전 배치한 물체를 고정 서보 자세로
> 운영자 확인 아래 집어 투입했습니다. 인식 화면과 pick-and-place는 분리 시연이며
> 통합 자율 E2E 성공으로 계수하지 않습니다.

## 중단 조건

- 베이스가 움직일 수 있는 상태이거나 작업 반경 안에 사람이 있음
- 시작 자세가 등록된 idle/safe/grasp 허용 범위 밖임
- servo 2 시작 온도가 40 °C를 초과함
- 손가락 또는 물체가 하강 중 밀리거나 부딪힘
- 파지 부하가 스크립트 임계값을 통과하지 못함
- jam, 케이블 장력, 비정상 소리, 과열 또는 예상 밖 이동이 한 번이라도 발생함

하나라도 해당하면 촬영을 중단한다. 영상 완성을 이유로 안전 검사를 우회하지 않는다.
프로필과 부하 임계값은 저장소에 기록된 값일 뿐이며, 원시 실측 근거가 확보되지 않은
수치는 성능 사실로 발표하지 않는다.

## D-1 실행 순서

1. 바퀴를 고정하고 비상정지 담당자를 둔다.
2. 물체 중심과 바구니 중심을 테이프로 표시한다. 한 프로필만 선택한다.
3. 팔 상태를 **읽기 전용**으로 검사한다.

   ```bash
   cd ~/docker/shared/grippers
   PYTHONPATH=third_party/soarm_provided_d/soarm_lab:ros2_ws/src/grippers_arm \
     python3 tools/align_to_idle.py --dry-run
   ```

4. 출력된 현재 자세·온도·통신 상태를 사람이 확인한다. 정렬이 필요하면 작업공간을 비운
   뒤에만 `--dry-run`을 제거해 실행한다.
5. 바구니 경로는 빈손 전용 도구로 먼저 확인한다.

   ```bash
   PYTHONPATH=third_party/soarm_provided_d/soarm_lab:ros2_ws/src/grippers_arm \
     python3 tools/basket_drop_pose_hardware_test.py
   ```

6. 기존 기본 모드는 물체가 없는 상태로 파지 높이까지 하강한 뒤, 운영자가 손가락
   사이에 물체를 놓고 상승·투하 경로를 확인한다. 빈 그리퍼는 부하 검사에서 중단되므로
   전체 경로의 무부하 시험으로 오해하지 않는다.

   ```bash
   PYTHONPATH=third_party/soarm_provided_d/soarm_lab:ros2_ws/src/grippers_arm \
     python3 tools/horizontal_grasp_hardware_test.py cube --drop-to-basket
   ```

7. 하강 경로와 상승·투하 경로를 각각 확인한 뒤에만 사전 배치 물체 모드를 실행한다.

   ```bash
   PYTHONPATH=third_party/soarm_provided_d/soarm_lab:ros2_ws/src/grippers_arm \
     python3 tools/horizontal_grasp_hardware_test.py cube \
       --prepositioned-object --drop-to-basket
   ```

8. 같은 물체·표시점·바구니 배치로 3회 리허설한다. 실패를 숨기지 말고 원본 영상과
   콘솔 로그를 모두 보존한다.

지원 프로필은 `cube`, `star_column`, `soccer_polyhedron`, `chess_rook`,
`chess_queen`, `chess_knight`다. 제출 전에는 가장 단순한 물체 하나만 고정하고,
리허설 없이 다른 프로필로 바꾸지 않는다.

## 촬영 구성

1. 한 화면에 로봇, 물체 표시점, 바구니, 운영자 승인 동작이 보이게 한다.
2. 첫 3초에 위의 “긴급 보조 시연” 문구를 표시한다.
3. 별도 인식 영상이 있다면 **과거/별도 인식 검증 화면**이라고 캡션을 붙인다.
4. scripted pick-and-place 구간과 인식 화면을 연속 자율 동작처럼 편집하지 않는다.
5. 마지막 슬라이드에 Hailo 복구 후 실제 인식→접근→파지→투입 통합 검증을 후속
   마일스톤으로 적는다.

## Hailo 제출 전 백업

HEF, ONNX, labels, DFC 설정, HailoRT·드라이버·커널 버전, `dmesg` 전체 로그,
`hailortcli` 출력, 이전 정상 동작 원본 영상을 한 묶음으로 보존한다. 제출 전에는
펌웨어·드라이버 조합을 무리하게 바꾸지 않는다. 하드웨어 손상 여부는 다른 보드/Pi/전원
교차시험 또는 구매처·Hailo 판정 전까지 확정하지 않는다.

관련 이슈: #146, #186, #189, #191, #195, #197, #198
