"""run_mission.py 의 자동 bringup/teardown (2026-09-04, 사용자 지시).

ssh 왕복을 실제로 하지 않는다 — `pi_lifecycle._ssh` 하나만 가짜로 바꿔서
bringup_now.sh/stop_bringup.sh 가 낼 수 있는 실제 출력 문구에 대해
`bringup()`/`teardown()` 이 옳게 반응하는지만 본다."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))

import pi_lifecycle  # noqa: E402


def _cp(stdout: str = "", stderr: str = "", returncode: int = 0):
    return subprocess.CompletedProcess(args=[], returncode=returncode,
                                       stdout=stdout, stderr=stderr)


def test_처음부터_깨끗하면_한_번에_뜬다(monkeypatch):
    calls = []

    def fake_ssh(pi_host, pi_user, remote_cmd, timeout):
        calls.append(remote_cmd)
        if "bringup_now.sh" in remote_cmd:
            return _cp(stdout="기동 중 — 로그: /tmp/bringup.log\nlaunch PID/PGID = 123\n")
        if "ros2 node list" in remote_cmd:
            return _cp(stdout="/arm_driver_node\n/perception_node\n/odom_publisher\n")
        raise AssertionError(f"예상 밖의 명령: {remote_cmd}")

    monkeypatch.setattr(pi_lifecycle, "_ssh", fake_ssh)
    result = pi_lifecycle.bringup("192.168.0.7", host_ip="192.168.0.9", ready_timeout=5)
    assert result.ok
    assert not any("stop_bringup.sh" in c for c in calls), "깨끗한 상태에서 teardown 을 부르면 안 된다"


def test_남은_노드가_있으면_정리하고_다시_띄운다(monkeypatch):
    calls = []
    launch_n = 0

    def fake_ssh(pi_host, pi_user, remote_cmd, timeout):
        nonlocal launch_n
        calls.append(remote_cmd)
        if "bringup_now.sh" in remote_cmd:
            launch_n += 1
            if launch_n == 1:
                return _cp(stdout="이미 떠 있는 노드가 있습니다 — 먼저 stop_bringup.sh 를 돌리세요:\n...")
            return _cp(stdout="기동 중\nlaunch PID/PGID = 456\n")
        if "stop_bringup.sh" in remote_cmd:
            return _cp(stdout="프로세스 그룹 123 에 SIGINT 전송...\n정상 종료됨\n")
        if "ros2 node list" in remote_cmd:
            return _cp(stdout="/arm_driver_node\n/perception_node\n/odom_publisher\n")
        raise AssertionError(f"예상 밖의 명령: {remote_cmd}")

    monkeypatch.setattr(pi_lifecycle, "_ssh", fake_ssh)
    result = pi_lifecycle.bringup("192.168.0.7", host_ip="192.168.0.9", ready_timeout=5)
    assert result.ok
    assert launch_n == 2, "첫 시도가 막히면 정리 후 한 번 더 시도해야 한다"
    assert any("stop_bringup.sh" in c for c in calls)


def test_핵심_노드가_끝내_안_뜨면_실패로_보고한다(monkeypatch):
    def fake_ssh(pi_host, pi_user, remote_cmd, timeout):
        if "bringup_now.sh" in remote_cmd:
            return _cp(stdout="기동 중\nlaunch PID/PGID = 789\n")
        if "ros2 node list" in remote_cmd:
            return _cp(stdout="/arm_driver_node\n")   # perception_node/odom_publisher 없음
        raise AssertionError(f"예상 밖의 명령: {remote_cmd}")

    monkeypatch.setattr(pi_lifecycle, "_ssh", fake_ssh)
    result = pi_lifecycle.bringup("192.168.0.7", host_ip="192.168.0.9", ready_timeout=2)
    assert not result.ok
    assert "perception_node" in result.detail
    assert "odom_publisher" in result.detail


def test_정상_종료를_성공으로_본다(monkeypatch):
    def fake_ssh(pi_host, pi_user, remote_cmd, timeout):
        if "topic pub" in remote_cmd:
            return _cp()
        assert "stop_bringup.sh" in remote_cmd
        return _cp(stdout="프로세스 그룹 123 에 SIGINT 전송...\n정상 종료됨\n")

    monkeypatch.setattr(pi_lifecycle, "_ssh", fake_ssh)
    result = pi_lifecycle.teardown("192.168.0.7")
    assert result.ok


def test_kill_전에_0속도를_먼저_박는다(monkeypatch):
    """2026-09-04 실기 — 모터 노드를 죽인다고 로봇이 멈추는 게 아니다
    (마지막 명령이 래치된다). kill 하기 전에 0 속도를 반드시 먼저
    보내야 한다."""
    calls = []

    def fake_ssh(pi_host, pi_user, remote_cmd, timeout):
        calls.append(remote_cmd)
        if "topic pub" in remote_cmd:
            return _cp()
        return _cp(stdout="정상 종료됨\n")

    monkeypatch.setattr(pi_lifecycle, "_ssh", fake_ssh)
    pi_lifecycle.teardown("192.168.0.7")

    assert calls, "ssh 를 한 번도 안 불렀다"
    assert "topic pub" in calls[0] and "cmd_vel" in calls[0], (
        "0 속도 명령이 stop_bringup.sh 보다 먼저 나가야 한다")
    assert any("stop_bringup.sh" in c for c in calls[1:])


def test_PGID_파일이_없어도_실제로_노드가_살아있으면_실패다(monkeypatch):
    """stop_bringup.sh 로 못 잡는 방식(손으로 띄움 등)으로 떠 있는 경우 —
    파일이 없다고 무조건 성공으로 믿으면 안 된다."""
    def fake_ssh(pi_host, pi_user, remote_cmd, timeout):
        if "topic pub" in remote_cmd:
            return _cp()
        if "stop_bringup.sh" in remote_cmd:
            return _cp(stdout="/tmp/bringup.pgid 가 없습니다 — bringup_now.sh 로 띄운 게 아니면...")
        if "ros2 node list" in remote_cmd:
            return _cp(stdout="/arm_driver_node\n/perception_node\n/odom_publisher\n")
        raise AssertionError(f"예상 밖의 명령: {remote_cmd}")

    monkeypatch.setattr(pi_lifecycle, "_ssh", fake_ssh)
    result = pi_lifecycle.teardown("192.168.0.7")
    assert not result.ok
    assert "arm_driver_node" in result.detail


def test_벤더_기본값_단독기동도_PGID_없이_실제로_살아있으면_실패다(monkeypatch):
    """Pi를 막 전원 켰을 때는 grippers_bringup과 무관한 벤더 기본값
    ros_robot_controller 혼자만 자동 기동돼 있을 수 있다 — 핵심 노드
    3개(arm_driver_node 등)엔 안 들지만, 그래도 "떠 있는 것"이다."""
    def fake_ssh(pi_host, pi_user, remote_cmd, timeout):
        if "topic pub" in remote_cmd:
            return _cp()
        if "stop_bringup.sh" in remote_cmd:
            return _cp(stdout="/tmp/bringup.pgid 가 없습니다 — bringup_now.sh 로 띄운 게 아니면...")
        if "ros2 node list" in remote_cmd:
            return _cp(stdout="/ros_robot_controller\n")
        raise AssertionError(f"예상 밖의 명령: {remote_cmd}")

    monkeypatch.setattr(pi_lifecycle, "_ssh", fake_ssh)
    result = pi_lifecycle.teardown("192.168.0.7")
    assert not result.ok
    assert "ros_robot_controller" in result.detail


def test_PGID_파일이_없고_노드도_없으면_정말_정리할_게_없는_것이다(monkeypatch):
    def fake_ssh(pi_host, pi_user, remote_cmd, timeout):
        if "topic pub" in remote_cmd:
            return _cp()
        if "stop_bringup.sh" in remote_cmd:
            return _cp(stdout="/tmp/bringup.pgid 가 없습니다 — bringup_now.sh 로 띄운 게 아니면...")
        if "ros2 node list" in remote_cmd:
            return _cp(stdout="")
        raise AssertionError(f"예상 밖의 명령: {remote_cmd}")

    monkeypatch.setattr(pi_lifecycle, "_ssh", fake_ssh)
    result = pi_lifecycle.teardown("192.168.0.7")
    assert result.ok


def test_로컬_IP를_실제로_구할_수_있다():
    ip = pi_lifecycle.get_local_ip_for("192.168.0.7")
    assert ip.count(".") == 3
    assert ip != "0.0.0.0"
