"""run_mission.py 시작/종료에 Pi 쪽 bringup/teardown 을 자동으로 묶는다.

## 왜 (2026-09-04, 사용자 지시)

지금까지는 "테스트 준비"(test_ready.sh) 로 사람이 먼저 Pi 를 기동해 두고,
그 다음에 사람이 run_mission.py 를 실행하는 두 단계였다. 그런데 이 세션에
차가 "한 번은 잘 움직였다가 재실행하면 갑자기 아예 안 움직이는" 증상이
반복됐다 — 이전 run_mission.py 세션이 남긴 상태(DDS 세션·구독자 캐시 등)가
다음 실행에 영향을 준다는 의심이 있다. 그래서 run_mission.py 실행 자체에
"시작할 때 깨끗하게 새로 띄우고, 끝날 때 깨끗하게 정리한다"를 박아 넣는다
— 매번 완전히 새 프로세스로 시작하면 그 의심되는 원인 하나는 확실히
제거된다.

⚠️ `stop_bringup.sh` 자신의 주석은 "kill 을 실제로 하므로 사용자가 직접
실행해야 한다"고 적어 뒀다(모터 제어에 영향 줄 수 있는 명령은 Claude Code
가 대필하지 않는다는 이 프로젝트의 표준 원칙, CHANGES_2026-09-02.md).
이 파일은 그 원칙을 뒤집는 것이 아니라, **사용자가 이번에 명시적으로
run_mission.py 자체에 그 실행을 넣어 달라고 지시**한 것을 반영한 것이다
— 그 지시가 없었다면 이 자동화는 만들지 않았을 것이다.

## 무엇을 자동화하고 무엇을 안 하는가

여기서 대신하는 것은 test_ready.sh 의 3단계 중 **bringup 단 하나뿐**이다.
코드 동기화(git fetch/merge)와 EEPROM 캘리브레이션 비교(읽기 전용)는
여전히 별개다 — 그건 "지금 실행할 코드가 맞는가"를 보는 것이라 매
run_mission.py 실행마다 자동으로 할 일이 아니고, 필요할 때 test_ready.sh
로 따로 확인한다.
"""

from __future__ import annotations

import shlex
import socket
import subprocess
import time
from dataclasses import dataclass

PI_USER_DEFAULT = "pi"
CONTAINER = "IntelPi"
BRINGUP_CMD = "/grippers/tools/ops/bringup_now.sh"
STOP_CMD = "/grippers/tools/ops/stop_bringup.sh"
# bringup_now.sh 가 "이미 떠 있다"고 보는 노드 전부(그 스크립트의 STALE
# 정규식 그대로) — teardown()이 "정리할 게 없다"고 잘못 믿지 않으려면 이
# 목록으로 봐야 한다. 이 중 실제로 그리퍼스 미션에 필요한 핵심만 추린
# 부분집합(_READY_NODE_MARKERS)은 준비 완료 판정에 쓴다 — 전부를 기다리면
# 카메라 드라이버 초기화가 늦어질 때 불필요하게 오래 걸린다.
#
# ⚠️ 2026-09-04: Pi를 막 전원 켰을 때는 벤더 기본값 ros_robot_controller가
# grippers_bringup과 무관하게 혼자 자동 기동돼 있을 수 있다(PGID 파일
# 없음 — bringup_now.sh로 띄운 게 아니라서). 그런데 이건
# _READY_NODE_MARKERS(핵심 3개)엔 없어서, 그 좁은 목록만 보면 "이미 떠
# 있는 것 없음"으로 잘못 판단해 stop_bringup.sh 없이 그냥 성공으로
# 넘어갈 뻔했다 — 넓은 이 목록으로 확인해야 그 경우도 잡는다.
_STALE_NODE_MARKERS = ("ros_robot_controller", "odom_publisher", "ekf_node",
                       "joint_state_publisher", "ascamera_node",
                       "arm_driver_node", "perception_node",
                       "robot_state_publisher")
_READY_NODE_MARKERS = ("arm_driver_node", "perception_node", "odom_publisher")


def get_local_ip_for(remote_ip: str) -> str:
    """`remote_ip` 로 나가는 데 쓰일 이 기기의 로컬 IP를 알아낸다.

    UDP 소켓의 connect() 는 실제로 패킷을 보내지 않는다 — 라우팅 테이블만
    참조해서 로컬 쪽 주소를 정한다. bringup_now.sh 의 host_ip 인자(Pi 쪽
    ROS 노드가 Host 로 보고를 쏠 주소)를 사람이 매번 IP를 확인해 넘길
    필요 없이 여기서 자동으로 구한다."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((remote_ip, 1))
        return s.getsockname()[0]
    finally:
        s.close()


@dataclass
class LifecycleResult:
    ok: bool
    detail: str


def _ssh(pi_host: str, pi_user: str, remote_cmd: str, timeout: float) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["ssh", "-o", "ConnectTimeout=10", "-o", "BatchMode=yes",
         f"{pi_user}@{pi_host}", remote_cmd],
        capture_output=True, text=True, timeout=timeout,
        # ⚠️ text=True 만 주면 Windows 는 로캘 기본(cp949)으로 디코드한다.
        # Pi 는 한글 UTF-8 을 내므로 첫 한글 바이트에서 UnicodeDecodeError 가
        # 나고, 그게 subprocess 의 리더 스레드 안에서 터져 bringup 확인이
        # 통째로 실패한다(2026-09-06, Windows Host). errors 까지 주는 이유는
        # 원격 로그에 깨진 바이트가 섞여도 확인 자체는 계속돼야 하기 때문이다.
        encoding="utf-8", errors="replace")


def _docker_exec(pi_host: str, pi_user: str, inner_cmd: str, timeout: float,
                  login_shell: bool = False) -> subprocess.CompletedProcess:
    shell = "bash -lc" if login_shell else "bash -c"
    remote = f"docker exec -u ubuntu {CONTAINER} {shell} {shlex.quote(inner_cmd)}"
    return _ssh(pi_host, pi_user, remote, timeout=timeout)


def _node_list(pi_host: str, pi_user: str, timeout: float = 15.0) -> str:
    res = _docker_exec(
        pi_host, pi_user,
        "source /opt/ros/humble/setup.bash && ros2 node list",
        timeout=timeout)
    return res.stdout


def _send_zero_cmd_vel(pi_host: str, pi_user: str, attempts: int = 5,
                        timeout: float = 8.0) -> None:
    """모터 제어 노드를 죽이기 **직전에** 0 속도를 한 번 더 못박아 둔다.

    ⚠️ 2026-09-04 실기(교훈, grippers-stopping-the-vehicle 메모리): 마지막
    명령은 그대로 래치된다 — 모터에 쓰던 노드를 죽인다고 로봇이 멈추지
    않는다. run_mission.py 자신도 종료할 때 stop 을 8번 보내지만, 그
    직후 이 teardown 이 곧바로 노드를 죽이러 들어오는 순서라 정말로
    받아들여졌는지 다시 확인할 방법이 없다 — 실제로 한 번, run_mission
    종료 뒤에도 회전이 안 멎어 사람이 직접 확인해야 했다. 그래서 여기서
    독립적으로 한 번 더, 여러 번 0 속도를 박아 둔 다음에만 kill 로
    넘어간다. 노드가 이미 죽어 있으면(실패해도) 어차피 래치할 것도 없으니
    조용히 넘어간다."""
    cmd = ("export ROS_DOMAIN_ID=21; source /opt/ros/humble/setup.bash; "
           "source /ros2_ws/install/setup.bash; "
           "for i in $(seq 1 %d); do ros2 topic pub --once /cmd_vel "
           "geometry_msgs/msg/Twist "
           "\"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" "
           "2>/dev/null; sleep 0.1; done" % attempts)
    try:
        _docker_exec(pi_host, pi_user, cmd, timeout=timeout, login_shell=True)
    except Exception:  # noqa: BLE001 -- 최선 노력이다, 이게 실패해도 kill 은 계속 진행한다
        pass


def teardown(pi_host: str, pi_user: str = PI_USER_DEFAULT,
             timeout: float = 25.0) -> LifecycleResult:
    """Pi 쪽 grippers bringup 을 잔재 없이 정지한다 (stop_bringup.sh 그대로).

    PGID 파일이 없으면(예: 이번 run_mission.py 가 애초에 아무것도 못
    띄웠거나, Pi 가 이미 꺼진 상태) stop_bringup.sh 자체가 exit 1 을
    내는데, 그건 실패가 아니라 "정리할 게 없다"는 뜻이라 성공으로
    본다."""
    _send_zero_cmd_vel(pi_host, pi_user)
    try:
        res = _ssh(pi_host, pi_user, f"docker exec -u ubuntu {CONTAINER} {STOP_CMD}",
                   timeout=timeout)
    except subprocess.TimeoutExpired:
        return LifecycleResult(False, f"{timeout:.0f}초 안에 응답이 없었습니다")
    except Exception as exc:  # noqa: BLE001 -- ssh 자체가 안 될 수도 있다
        return LifecycleResult(False, f"ssh 실패: {exc}")

    out = (res.stdout or "") + (res.stderr or "")
    if "가 없습니다" in out:
        # PGID 파일이 없다 — bringup_now.sh 로 띄운 게 아니라 다른 방식으로
        # (손으로, 또는 예전 방식으로) 떠 있을 수도 있다. 파일이 없다고
        # 무조건 "정리할 게 없다"고 믿으면 그 경우를 놓치고 성공으로
        # 착각한다 — 실제 노드 목록으로 다시 확인한다.
        try:
            listing = _node_list(pi_host, pi_user)
        except Exception:  # noqa: BLE001
            listing = ""
        still_alive = [m for m in _STALE_NODE_MARKERS if m in listing]
        if not still_alive:
            return LifecycleResult(True, "정리할 것이 없었습니다(이미 내려가 있음)")
        return LifecycleResult(
            False,
            f"stop_bringup.sh 가 못 잡는 방식으로 이미 떠 있습니다"
            f"({', '.join(still_alive)}) — 사람이 직접 정리해야 합니다")
    if res.returncode == 0 and ("정상 종료됨" in out or "SIGKILL" in out):
        return LifecycleResult(True, out.strip().splitlines()[-1] if out.strip() else "종료됨")
    return LifecycleResult(False, out.strip() or f"exit={res.returncode}")


def bringup(pi_host: str, pi_user: str = PI_USER_DEFAULT,
            host_ip: str | None = None, ready_timeout: float = 45.0
            ) -> LifecycleResult:
    """Pi 쪽 grippers bringup 을 새로 띄우고, 핵심 노드가 뜰 때까지 기다린다.

    이미 떠 있는 노드가 있으면(이전 run_mission.py 가 비정상 종료해
    teardown 을 못 거쳤거나, 다른 곳에서 손으로 띄워 둔 경우) 한 번
    stop_bringup.sh 로 정리하고 다시 시도한다 — "다시 실행할 때마다
    깨끗하게 새로 띄운다"는 목적 자체가 이 자가치유를 요구한다."""
    if host_ip is None:
        host_ip = get_local_ip_for(pi_host)

    def _launch() -> subprocess.CompletedProcess:
        # 2026-09-04 실기: Pi를 막 부팅한 직후엔 docker/ROS 워크스페이스
        # 네 개를 순서대로 sourcing하는 것만도 20초를 넘길 수 있었다
        # (launch 자체는 setsid ... & 로 즉시 백그라운드라 빠르지만, 그 앞의
        # sourcing이 느렸다) — 그 순간 이 타임아웃이 ssh 연결을 끊어버려서
        # bringup_now.sh가 PGID 파일을 쓰는 줄까지 못 갔다(그런데도 이미
        # 백그라운드로 넘어간 launch 자체는 살아남아 결국 정상 기동됨 —
        # 그래서 다음 준비 확인 폴링에서는 성공으로 보였다). 콜드 부팅
        # 여유를 넉넉히 준다.
        return _docker_exec(pi_host, pi_user, f"{BRINGUP_CMD} {host_ip}", timeout=45.0)

    try:
        res = _launch()
    except subprocess.TimeoutExpired:
        return LifecycleResult(False, "bringup_now.sh 20초 안에 응답이 없었습니다")
    except Exception as exc:  # noqa: BLE001
        return LifecycleResult(False, f"ssh 실패: {exc}")

    if "이미 떠 있는 노드가 있습니다" in (res.stdout or ""):
        print("[pi] 이전에 남은 노드를 발견 — 먼저 정리하고 다시 띄웁니다", flush=True)
        cleanup = teardown(pi_host, pi_user)
        if not cleanup.ok:
            return LifecycleResult(
                False, f"남은 노드 정리 실패: {cleanup.detail}")
        time.sleep(1.0)
        try:
            res = _launch()
        except Exception as exc:  # noqa: BLE001
            return LifecycleResult(False, f"재시도 ssh 실패: {exc}")

    if res.returncode != 0:
        return LifecycleResult(
            False, (res.stdout or res.stderr or f"exit={res.returncode}").strip())

    deadline = time.monotonic() + ready_timeout
    seen: set[str] = set()
    while time.monotonic() < deadline:
        try:
            listing = _node_list(pi_host, pi_user)
        except Exception:  # noqa: BLE001 -- 아직 ROS 데몬이 안 떠서 실패할 수 있다
            listing = ""
        seen = {m for m in _READY_NODE_MARKERS if m in listing}
        if len(seen) == len(_READY_NODE_MARKERS):
            return LifecycleResult(True, f"{host_ip} 기준 {len(seen)}개 핵심 노드 확인")
        time.sleep(1.5)

    missing = set(_READY_NODE_MARKERS) - seen
    return LifecycleResult(
        False, f"{ready_timeout:.0f}초 안에 준비 안 됨 — 못 본 노드: {', '.join(sorted(missing))}")
