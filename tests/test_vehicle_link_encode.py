"""encode() 의 상태 매핑 표(_STATE_TO_PI) 누락을 잡는다 (2026-09-02 실기 사고).

## 왜 이 파일이 생겼나

07:12 rook 시험에서 `GRASP_FORCE` 가 `_STATE_TO_PI` 에서 빠진 채 몇 주를
지났다. `encode()`의 "모르는 상태는 IDLE+정지" 안전장치가 매번 조용히 걸려서,
강제 파지를 보낼 때마다 전선에는 IDLE 이 나갔다 — Pi 가 진짜로 IDLE 에 들어가
Host 는 `State.GRASP`/`_forcing_grasp=True` 에 갇힌 채로 33초간 락업됐다.
같은 감사에서 `RETURN_HOME`(`_skip_target` 이 기물을 포기할 때 쓰는 경로)도
빠져 있는 걸 하나 더 찾았다 — 아직 실기에서 걸리지 않았을 뿐 같은 사고다.

`test_grasp_force.py` 등 기존 미션 FSM 테스트는 `PiSim` 더미로 `link.sent`만
보고 `encode()`(전선 인코딩)를 아예 거치지 않는다 — 그래서 이 표가 빠진 걸
아무 테스트도 못 잡았다. 이 파일은 그 틈을 메운다: `mission.py`가 실제로
낼 수 있는 status 문자열이 전부 `_STATE_TO_PI`에 있는지, `State` 이름이 하나
늘어도 자동으로 걸리게 exhaustive 하게 확인한다."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_HOST = Path(__file__).resolve().parent.parent / "host"
sys.path.insert(0, str(_HOST))
sys.path.insert(0, str(_HOST / "aruco"))

from mission import State                                    # noqa: E402
import vehicle_link                                            # noqa: E402

# mission.py 가 `self.state.name` 을 그대로 status 로 보내는 자리
# (_approach -> _send_drive) 는 State 멤버 전부가 대상이다. 그 밖에
# "GRASP_FORCE" 처럼 State 멤버가 아닌 합성 status 문자열도 따로 낸다
# (mission.py 의 `status = "GRASP_FORCE" if self._forcing_grasp else "GRASP"`).
_SYNTHETIC_STATUSES = ("GRASP_FORCE",)


@pytest.mark.parametrize("status", [s.name for s in State] + list(_SYNTHETIC_STATUSES))
def test_every_status_is_in_state_to_pi_table(status):
    """mission.py 가 낼 수 있는 status 는 전부 _STATE_TO_PI 에 있어야 한다.

    없으면 encode() 의 "모르는 상태는 IDLE+정지" 안전장치가 조용히 걸린다 —
    그건 곧 Host 의 의도(예: 강제 파지, 기본 위치 복귀)가 무시되고 Pi 가
    실제로 IDLE 에 들어간다는 뜻이다. wire 출력 모양(IDLE+stop)으로는 못
    가른다 — SEARCH_TARGET 은 의도적으로 IDLE 에 매핑되기 때문에, 표에
    있는지를 직접 확인한다."""
    assert status in vehicle_link._STATE_TO_PI, (
        f"status={status!r} 가 _STATE_TO_PI 에 없다 — 이 상태를 보내면 "
        "encode() 가 조용히 IDLE+정지로 대체해 Pi 가 진짜 IDLE 에 들어간다 "
        "(2026-09-02 GRASP_FORCE 락업과 같은 사고)"
    )
