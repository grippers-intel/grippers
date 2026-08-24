#!/usr/bin/env python3
"""전원 재투입 자동 IDLE 정렬 데몬 — 무인 백그라운드 버전.

align_to_idle.py 모듈 docstring은 "arm_driver_node는 IDLE 편차를 로그로만
남기고 절대 자동으로 움직이지 않는다 — 사람이 지켜보며 align_to_idle.py를
실행하는 것이 유일한 정렬 경로"라는 안전 계약을 명시하고 있다. 이 데몬은
그 계약을 사용자 요청으로 의도적으로 깬다: 사람 개입 없이 백그라운드에서
포트를 감시하다가 팔이 재연결되면 스스로 torque를 켜고 IDLE로 이동시킨다.
즉 주변에 사람이 없거나 팔이 예상 못한 자세로 놓여 있어도 무조건 움직인다
(align_to_idle.py의 편차 상한/servo2 과열 검사는 그대로 통과해야만 움직이지만,
그 검사를 통과하는 한 사람 확인 없이 실행된다는 뜻). 무인 운용을 원치 않으면
align_to_idle.py를 사람이 직접 실행할 것.

동작 순서:
  1) 다른 프로세스가 이미 포트를 쓰고 있는지 매 poll마다 pgrep으로 먼저
     확인한다(패턴: "grippers_arm/arm_driver" — HANDOFF.md에 기록된, 실제
     실기 테스트 중 arm_driver_node가 /dev/soarm을 쥐고 있을 때의 프로세스
     이름 규칙). 걸리면 이번 poll은 건너뛴다 — 실제 테스트가 돌고 있는
     동안 이 데몬이 같은 시리얼 포트를 열어 끼어들면 팔 torque가 꺼지는
     사고(§6 HANDOFF.md 기록)로 이어지기 때문에, 이 검사가 이 파일에서
     가장 중요한 안전장치다.
  2) 아무도 안 쓰고 있으면 포트를 열어 전 서보(1-6) ping을 시도한다.
  3) '이전 poll엔 응답 없었는데 이번엔 전부 응답' 전이가 감지되면 —
     즉 방금 전원이 들어온 것으로 보이면 — settle 시간만큼 기다렸다가
     align_to_idle의 절차(안전검사 → torque latch → IDLE 보간 이동)를
     그대로 재사용해 실행한다.
  4) 정렬 성공/실패와 무관하게, 연결이 한 번 끊겼다가 다시 붙기 전까지는
     재실행하지 않는다 — 연결이 유지되는 동안 매 poll마다 반복 이동하는
     것을 막기 위함이다.
  5) 모든 이벤트를 JSON Lines로 로그 파일에 남긴다(사람이 안 지켜보는
     동안 무슨 일이 있었는지 나중에 확인하기 위함 — 이 데몬을 쓰는
     이유 자체가 사람이 안 지켜본다는 것이므로 로그가 유일한 기록이다).

시작:  nohup python3 idle_align_daemon.py > /tmp/idle_align_daemon.out 2>&1 &
중단:  pkill -f idle_align_daemon.py
확인:  tail -f /tmp/idle_align_daemon.jsonl
"""

import argparse
import json
import subprocess
import sys
import time

import align_to_idle as ai

DEFAULT_PORT = ai.DEFAULT_PORT
DEFAULT_LOG = "/tmp/idle_align_daemon.jsonl"
POLL_INTERVAL_SEC = 2.0
SETTLE_AFTER_CONNECT_SEC = 3.0
OWNER_PGREP_PATTERN = "grippers_arm/arm_driver"

ALL_SERVO_IDS = list(ai.SERVO_IDS) + [ai.GRIPPER_SERVO_ID]


def _log(log_path, event, **fields):
    record = {"ts": time.time(), "event": event, **fields}
    with open(log_path, "a") as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")
    print(f"[daemon] {event} {fields}", flush=True)


def _port_owned_elsewhere(pattern=OWNER_PGREP_PATTERN):
    """arm_driver_node 등 다른 프로세스가 이미 포트를 쥐고 있으면 True.

    포트를 열어보지 않고 pgrep만으로 판단한다 — 열어서 확인하는 순간 이미
    간섭이 시작되기 때문에, 간섭 여부는 반드시 열기 전에 알아야 한다."""
    result = subprocess.run(["pgrep", "-f", pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return result.returncode == 0


def _probe_online(port):
    """포트를 짧게 열어 전 서보 ping 후 즉시 닫는다. 전부 응답하면 True."""
    driver = ai._connect(port)
    if driver is None:
        return False
    try:
        return all(driver.ping(sid) for sid in ALL_SERVO_IDS)
    finally:
        driver.disconnect()


def _align_once(port, log_path):
    targets = ai.idle_targets()
    driver = ai._connect(port)
    if driver is None:
        _log(log_path, "align_skip_reconnect_failed")
        return
    try:
        status = driver.get_all_status()
        problems = ai.check_safe_to_align(status, targets)
        if problems:
            _log(log_path, "align_rejected", problems=problems, offsets=ai.report_offsets(status, targets))
            return

        _log(log_path, "align_start", offsets=ai.report_offsets(status, targets))
        start = ai.latch_torque_at_present(driver, targets)
        for servo_id in targets:
            driver.set_speed(servo_id, ai.SPEED_RAW)
            driver.set_acceleration(servo_id, ai.ACCELERATION_RAW)

        try:
            final = ai.glide_to_targets(driver, start, targets)
        except ai.JamDetected as e:
            _log(log_path, "align_jam", error=str(e))
            return

        offsets = {sid: final[sid] - targets[sid] for sid in targets if final.get(sid) is not None}
        worst = max(offsets.values(), key=abs) if offsets else None
        if worst is None or abs(worst) > ai.DEFAULT_TOLERANCE_RAW:
            _log(log_path, "align_incomplete", offsets=offsets)
        else:
            _log(log_path, "align_done", offsets=offsets)
    except Exception as e:
        _log(log_path, "align_error", error=repr(e))
    finally:
        driver.disconnect()


def _tick(port, log_path, was_online, settle):
    """poll 한 번 분량의 판단만 순수하게 분리해 둔다 — run()의 while True 밖에서
    단위 테스트하기 위함(하드웨어/pgrep을 모두 monkeypatch로 대체)."""
    if _port_owned_elsewhere():
        return False  # 다른 프로세스가 쥔 동안의 상태는 신뢰하지 않는다

    online = _probe_online(port)
    if online and not was_online:
        _log(log_path, "arm_connected", note=f"{settle}s 안정화 대기 후 정렬 시작")
        time.sleep(settle)
        if not _port_owned_elsewhere():
            _align_once(port, log_path)
        else:
            _log(log_path, "align_skip_owner_appeared")
    return online


def run(port, log_path, poll, settle):
    _log(log_path, "daemon_start", port=port, poll=poll, settle=settle)
    was_online = False
    while True:
        was_online = _tick(port, log_path, was_online, settle)
        time.sleep(poll)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--log", default=DEFAULT_LOG)
    parser.add_argument("--poll", type=float, default=POLL_INTERVAL_SEC)
    parser.add_argument("--settle", type=float, default=SETTLE_AFTER_CONNECT_SEC)
    args = parser.parse_args(argv)
    try:
        run(args.port, args.log, args.poll, args.settle)
    except KeyboardInterrupt:
        print("\n[daemon] 중단됨", file=sys.stderr)


if __name__ == "__main__":
    main()
