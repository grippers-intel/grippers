"""차체 자동 복구 — 스크립트 대신 가짜 실행기로 순서와 쿨다운만 본다."""
from vla_robot_mission.base_recovery import BaseRecovery


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def make(clock, calls):
    def runner(cmd, timeout_s):
        calls.append((cmd, timeout_s))
        return 0, "후: 1 개"
    return BaseRecovery("/x/rrc_recover.sh", 40.0, 5.0, runner=runner, clock=clock,
                        log=lambda m: None, background=False)


def test_request_runs_the_fix_script_and_counts():
    clock, calls = Clock(), []
    r = make(clock, calls)
    assert r.count == 0 and not r.recovering
    assert r.request()
    assert calls == [(["bash", "/x/rrc_recover.sh", "--fix", "auto"], 40.0)]
    assert r.count == 1 and not r.recovering
    assert "종료 코드 0" in r.last_detail


def test_cooldown_ignores_repeated_requests():
    """Host 는 복구가 끝날 때까지 매 패킷 요청을 싣는다 — 한 번만 돌아야 한다."""
    clock, calls = Clock(), []
    r = make(clock, calls)
    r.request()
    clock.t += 1.0
    assert not r.request()
    clock.t += 5.0
    assert r.request()
    assert len(calls) == 2 and r.count == 2


def test_startup_reset_runs_the_script_in_its_own_quiet_mode():
    """10-01: 스택 기동 직후 선제 재기동. 스크립트는 auto* 에서 부저를 울리지 않는다."""
    clock, calls = Clock(), []
    r = make(clock, calls)
    assert r.request(startup=True)
    assert calls == [(["bash", "/x/rrc_recover.sh", "--fix", "auto-startup"], 40.0)]
    assert r.count == 1
    # Host 가 곧바로 복구를 요청해도 쿨다운으로 한 번만
    clock.t += 1.0
    assert not r.request()


def test_timeout_still_ends_the_recovery():
    clock = Clock()
    r = BaseRecovery("/x", 40.0, 5.0, runner=lambda c, t: (-1, "40s 안에 끝나지 않았다"),
                     clock=clock, log=lambda m: None, background=False)
    r.request()
    assert r.count == 1 and not r.recovering and "-1" in r.last_detail
