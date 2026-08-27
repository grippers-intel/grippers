"""ROS2 서비스·액션 호출의 대기 상한과 실패 처리. Ros2* 어댑터 4종이 전부 같은
패턴(서버 대기 → 호출 → 응답 대기)을 반복하므로, `_ros_convert.py` 와 같은 이유로
여기 한 곳에만 둔다 — 파일마다 흩어 두면 타임아웃 값과 실패 처리가 갈라지고,
값을 바꿀 때 한 곳을 놓치기 쉽다.

**두 단계 모두에 상한이 필요하다.** `wait_for_service()` 만 막으면 "서비스는 떠
있는데 응답을 주지 않는" 경우를 놓친다 — 2026-08-18 실기에서 ros_robot_controller
수신 스레드가 죽어 노드는 살아 있는데 응답이 없던 상황이 실제로 있었다. 그래서
future 대기에도 상한을 두고 미완료 future를 처리한다.

**타임아웃은 예외를 던지지 않는다.** 실패는 `None` 으로 돌아오고, 각 어댑터가 자기
포트의 실패 계약(`False` · `None` · 빈 목록 · `contact_risk=True`)으로 번역한다.
그래야 서비스 부재가 미션 종료가 아니라 각 상태(`baseline_mission.py`)가 이미
알고 있는 정상적인 실패 경로로 흡수된다 — 예: `BaselineGraspState._failed()`가
GRASP 실패를 Host에 보고하고 APPROACH로 되돌아간다. (⚠️ 2026-08-28: 예전엔
여기서 `docs/design/state_machine.md §3`의 "보류 등록 후 SCAN 복귀"를
인용했는데, `SCAN`은 지금 `MissionState`에 없는 상태고 그 문서 자체가
Host-Pi 역할 분담이 확정되기 전의 것이라 실제 동작과 다르다 — repo docs는
확인 없이 인용하지 말 것.)

⚠️ 2026-08-23 실기 확인(첫 전체 FSM 실기 테스트): `rclpy.spin_until_future_complete
(node, ...)` 를 쓰지 않는다 — 예전에는 여기서 그걸 썼다. `node`는
mission_orchestrator 자신이고, 그 노드는 이미 `executor.spin()`(메인 스레드,
MultiThreadedExecutor)이 계속 돌리고 있다. 이 함수들은 FSM 전용 스레드
(mission_orchestrator_node.py의 `_run_fsm`, ROS 콜백이 아님)에서 불리므로 그
executor 워커 스레드 자체를 점유하지는 않지만, spin_until_future_complete()는
**같은 노드를 스핀하는 두 번째(임시) executor** 를 만든다 — 같은 노드를 두
executor가 동시에 스핀하는 건 rclpy에서 지원되지 않는 조합이다. 실기에서
미션 하나(SCAN/APPROACH 여러 번 반복 호출)를 끝까지 돌리고 나면
mission_orchestrator의 `/command` 구독 콜백 자체가 응답하지 않는 상태가 됐다
— 새 미션 명령을 보내도 큐에 들어가지조차 않았다(재시작 전까지 복구 안 됨).
base_driver_node.py의 같은 날짜 경고(`_observe_target_once`)가 콜백 안에서
중첩 스핀하는 쪽의 증상이라면, 이건 FSM 스레드에서 불러도 여전히 위험하다는
쪽의 증거다. 대신 future 완료를 `threading.Event`로 기다린다 — 완료 자체는
이미 돌고 있는 (유일한) executor가 처리하므로 추가 executor가 필요 없다."""

import threading


def _wait(future, timeout_sec):
    """future가 끝날 때까지 최대 timeout_sec 기다린다. 이미 돌고 있는
    executor(이 프로세스의 유일한 executor.spin())가 future를 완료시키는
    콜백을 처리하므로, 여기서는 그 완료를 그냥 기다리기만 한다 — 추가
    executor를 만들지 않는다."""
    done_event = threading.Event()
    future.add_done_callback(lambda _f: done_event.set())
    return done_event.wait(timeout=timeout_sec)

# E-STOP 경로 전용 상한. 이 경로는 "응답을 기다리지 않는다"가 계약이라 대기 자체가
# 위험하므로, 서비스가 없으면 즉시 포기하고 로그만 남긴다.
ESTOP_TIMEOUT_SEC = 0.5

# monitor_clearance 전용 상한. INSERT 중 반복 호출되는 안전 판정이라 여기서 일반
# 서비스와 같은 3초를 기다리면 **베이스가 움직이는 도중 3초간 판단이 멈춘다** —
# 안전 장치가 오히려 위험 요인이 된다. 늦은 정답보다 빠른 '모르겠다'(=정지)가 낫다.
SAFETY_TIMEOUT_SEC = 0.5

# 액션 서버 discovery + goal 수락 상한. 서비스보다 넉넉한 이유는 액션 서버의
# discovery가 더 느리고, 이 지연은 노드 기동 직후 한 번만 겪기 때문이다.
ACTION_TIMEOUT_SEC = 5.0

# 액션 **결과** 상한. 서버 수락과 같은 값을 쓰면 안 된다 — 결과는 실제 동작
# 시간만큼 걸린다. base_driver_node의 drive_to는 도착까지 상한이 없고(P 제어
# 루프가 도착 판정까지 돈다), MAX_LINEAR=0.2m/s 기준 방을 가로지르면 30초에
# 가깝다. 정상 주행을 실패로 끊지 않으면서 무한 대기는 막는 값으로 60초를 둔다.
ACTION_RESULT_TIMEOUT_SEC = 60.0

# 나머지 서비스 상한.
SERVICE_TIMEOUT_SEC = 3.0


def call_service(node, client, request, *, label, timeout_sec=SERVICE_TIMEOUT_SEC):
    """서비스를 호출해 응답을 반환한다. 서버가 없거나 응답이 없으면 `None`.

    `label` 은 경고 로그에 그대로 실린다 — 어느 서비스가 응답하지 않았는지 알 수
    없으면 실기 디버깅이 불가능하다."""
    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.get_logger().warn(f"{label}: 서비스 없음 ({timeout_sec}s 대기) — 실패로 처리")
        return None

    future = client.call_async(request)
    _wait(future, timeout_sec)
    if not future.done():
        # 늦게 도착한 응답에 매달리지 않도록 정리한다.
        future.cancel()
        node.get_logger().warn(f"{label}: 응답 없음 ({timeout_sec}s 대기) — 실패로 처리")
        return None
    return future.result()


def call_action(
    node,
    client,
    goal,
    *,
    label,
    server_timeout_sec=ACTION_TIMEOUT_SEC,
    result_timeout_sec=ACTION_RESULT_TIMEOUT_SEC,
):
    """액션 goal을 보내고 결과 메시지를 반환한다. 실패하면 `None`.

    상한이 두 종류인 이유는 모듈 상단 `ACTION_RESULT_TIMEOUT_SEC` 주석 참고 —
    수락은 즉시 끝나야 하지만 결과는 동작 시간만큼 걸린다.

    goal 거부(`accepted=False`)도 `None` 으로 돌린다. 거부된 goal에
    `get_result_async()` 를 부르면 예외가 나는데, 서버가 "지금은 못 한다"고
    답한 것은 어댑터가 예외로 번역할 일이 아니라 포트의 정상적인 실패다."""
    if not client.wait_for_server(timeout_sec=server_timeout_sec):
        node.get_logger().warn(
            f"{label}: 액션 서버 없음 ({server_timeout_sec}s 대기) — 실패로 처리"
        )
        return None

    goal_future = client.send_goal_async(goal)
    _wait(goal_future, server_timeout_sec)
    if not goal_future.done():
        goal_future.cancel()
        node.get_logger().warn(
            f"{label}: goal 수락 응답 없음 ({server_timeout_sec}s 대기) — 실패로 처리"
        )
        return None

    goal_handle = goal_future.result()
    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().warn(f"{label}: goal 거부됨 — 실패로 처리")
        return None

    result_future = goal_handle.get_result_async()
    _wait(result_future, result_timeout_sec)
    if not result_future.done():
        # ⚠️ 그냥 빠져나가면 로봇은 계속 움직인다 — 취소를 요청하고 응답은
        # 기다리지 않는다. 여기서 또 블록되면 상한을 둔 의미가 없다.
        goal_handle.cancel_goal_async()
        result_future.cancel()
        node.get_logger().warn(
            f"{label}: 결과 없음 ({result_timeout_sec}s 대기) — 취소 요청 후 실패로 처리"
        )
        return None

    wrapped = result_future.result()
    if wrapped is None:
        node.get_logger().warn(f"{label}: 결과 메시지 없음 — 실패로 처리")
        return None
    return wrapped.result
