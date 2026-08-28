"""구동계가 살아 있는지 판정한다 (2026-08-28 실기 사고 대응).

## 왜 필요한가

2026-08-28 마지막 시험에서 **차량이 멈추지 않았다.** 정지 명령은 전부
정상적으로 나갔다 — Pi 워치독이 828사이클(82.8초) 연속으로 `base.stop()`을
불렀고, Host도 종료 시 정지를 8회 보냈다. 그런데 바퀴는 0.092 m/s로 등속
직진을 계속했다(표본 10개, 표준편차 1.6mm, yaw 변화 0.9도).

`cmd_vel`을 모터 보드 쓰기로 옮기는 노드가 물려 있었고, STM32는 **마지막으로
받은 속도를 무기한 계속 집행한다.** 그래서 `cmd_vel` 위쪽의 모든 정지 시도가
아무 데도 닿지 않았다.

같은 버그가 그날 오후에는 반대 얼굴로 나왔다 — `NUDGE_BOX`에서 `go`가 155회
나가는 동안 로봇이 1mm도 안 움직였다. 물린 순간 보드에 걸려 있던 값이 0이면
"안 움직인다", 0이 아니면 "안 멈춘다"로 보일 뿐 원인은 하나다.

## 이 모듈이 하는 일과 못 하는 일

**하는 일**: `cmd_vel` 아래가 명령을 받아 갈 상태인지 본다. 아니면 그 사실을
Host에 보고해서 **사람을 부른다.**

**못 하는 일**: 바퀴가 실제로 도는지는 모른다. 그것을 아는 수단이 이 차량에
없다 — `/odom_raw`는 명령을 적분할 뿐이라 같은 것을 되돌려주고, 바퀴에
엔코더 피드백이 없다. 진짜 확인은 오버헤드 ArUco(Host 소유)나 사람 눈이다.

그래서 이것은 **자동 복구 장치가 아니라 경보기**다. 근본 대책은 보드 쪽
명령 타임아웃인데 그건 벤더 펌웨어라 손댈 수 없다.

## 두 가지 신호

`NO_CONSUMER` — `cmd_vel`을 구독하는 노드가 하나도 없다. 노드가 죽었거나
    누가 껐다는 뜻이고, 즉시 확정할 수 있는 가장 강한 신호다. 2026-08-28에
    내가 `ros_robot_controller`를 죽인 순간이 정확히 이 상태였다.

`STALE_FEEDBACK` — 구독자는 있는데 구동계가 자기 주기 발행을 멈췄다. 노드는
    떠 있는데 루프가 물린 경우이고, 그날 실제로 일어난 쪽이다. 프로세스
    목록으로는 멀쩡해 보이기 때문에 이 신호가 없으면 사람이 못 알아챈다.

둘 다 **명령 경로와 독립적인 관측**이라는 점이 중요하다. `cmd_vel`이 0인
것과 `set_motor`에 트래픽이 없는 것은 무엇이 **명령됐는지**를 말할 뿐
바퀴가 무엇을 **하고 있는지**를 말하지 않는다 — 2026-08-28에 내가 정확히
그것을 거꾸로 읽고 "정지했다"고 잘못 보고했다.
"""

from dataclasses import dataclass
from typing import Optional

# 판정 결과.
ALIVE = "ALIVE"
NO_CONSUMER = "NO_CONSUMER"
STALE_FEEDBACK = "STALE_FEEDBACK"
UNKNOWN = "UNKNOWN"

# 기동 유예. 노드가 뜨고 서로를 발견하기까지 ROS2 디스커버리에 시간이 걸린다.
# 이 시간 안에는 판정하지 않는다 — 안 그러면 매 기동마다 거짓 경보가 뜨고,
# 거짓 경보가 일상이 되면 진짜 경보를 아무도 안 본다.
STARTUP_GRACE_SEC = 3.0

# 구동계 발행이 이만큼 끊기면 물린 것으로 본다. `/odom_raw`는 정상일 때
# 수십 Hz로 나오므로 1초는 매우 넉넉하다 — 일시적인 지연을 물림으로
# 오해하지 않으려고 크게 잡았다.
FEEDBACK_STALE_SEC = 1.0


@dataclass(frozen=True)
class LivenessVerdict:
    """구동계 판정 결과."""

    state: str
    detail: str = ""

    @property
    def alive(self) -> bool:
        """명령이 닿을 수 있는 상태인가.

        `UNKNOWN`을 살아 있는 쪽에 넣는 이유: 근거가 없을 때 위험을 선언하면
        기동할 때마다 경보가 뜬다. 모르는 것과 고장난 것은 다르다."""
        return self.state in (ALIVE, UNKNOWN)


def judge(subscriber_count: Optional[int],
          feedback_age_s: Optional[float],
          commanding_for_s: Optional[float],
          *,
          grace_s: float = STARTUP_GRACE_SEC,
          stale_s: float = FEEDBACK_STALE_SEC) -> LivenessVerdict:
    """구동계 상태를 판정한다.

    `subscriber_count`   `cmd_vel` 구독자 수. 셀 수 없으면 None.
    `feedback_age_s`     구동계가 마지막으로 발행한 뒤 지난 시간. 한 번도
                         못 받았으면 None.
    `commanding_for_s`   이 어댑터가 만들어진 뒤 지난 시간. 유예 판단용이고,
                         모르면 None(=유예 없음).
    """
    if commanding_for_s is not None and commanding_for_s < grace_s:
        return LivenessVerdict(UNKNOWN, "기동 유예 중")

    # 구독자 0은 확정이다. 피드백 나이보다 먼저 본다 — 아무도 안 듣고 있으면
    # 피드백이 낡은 것은 결과이지 원인이 아니고, 사람에게는 원인을 말해야 한다.
    if subscriber_count == 0:
        return LivenessVerdict(
            NO_CONSUMER,
            "cmd_vel 을 구독하는 노드가 없다 — 명령이 모터 보드까지 가지 않는다")

    if feedback_age_s is None:
        return LivenessVerdict(
            STALE_FEEDBACK,
            "구동계 피드백을 한 번도 받지 못했다 — 컨트롤러가 떠 있는지 확인")

    if feedback_age_s > stale_s:
        return LivenessVerdict(
            STALE_FEEDBACK,
            f"구동계 피드백이 {feedback_age_s:.1f}초 끊겼다 — 컨트롤러가 물렸다")

    if subscriber_count is None:
        # 구독자를 못 세는 구현이지만 피드백은 신선하다. 신선한 피드백이
        # 더 강한 증거이므로 살아 있다고 본다.
        return LivenessVerdict(ALIVE)

    return LivenessVerdict(ALIVE)


class LivenessLatch:
    """판정이 **바뀔 때만** 보고할 문장을 낸다.

    래치를 두는 이유는 워치독 경고와 같다 — 10Hz로 도는 루프에서 매 사이클
    같은 경고를 내면 로그가 그것으로 가득 차고, 정작 봐야 할 줄이 묻힌다.
    다만 눌러 버리지는 않는다: 상태가 바뀌는 순간(고장 발생·복구)은 둘 다
    사람이 꼭 알아야 하는 사건이라 반드시 한 번 내보낸다."""

    def __init__(self) -> None:
        self.state: Optional[str] = None

    def observe(self, verdict: Optional[LivenessVerdict]) -> Optional[str]:
        """바뀌었으면 보고할 문장, 아니면 None.

        `verdict`가 None이면(=liveness를 모르는 어댑터) 아무 일도 하지 않는다."""
        if verdict is None:
            return None
        previous, self.state = self.state, verdict.state
        if previous == verdict.state:
            return None
        if not verdict.alive:
            return f"구동계 이상 ({verdict.state}) — {verdict.detail}"
        if previous is not None and previous not in (ALIVE, UNKNOWN):
            return "구동계 응답 복구됨"
        return None
